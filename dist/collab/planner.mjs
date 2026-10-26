import {ROBOT_CONFIGS} from '../multi/robot-runtime.mjs';
import {createPlanningWorkerRobot} from '../multi/planning-worker-core.mjs';
import {planningControllerClass} from '../multi/planning.mjs';
import {capturePlanningResult,encodePlanningValue,decodePlanningValue} from '../multi/planning-serialization.mjs';
import {createRobotAgent} from './robot-adapter.mjs';
import {IDS,robotFrame,restorePrivateSnapshot} from './state.mjs';

/** The planner owns a private COPY of the complete shared model. Results only
 * contain validated command trajectories; no physical arrays are returned. */
export function createSharedPlanningService({api,xml,metadata,urdfs}){
  const vfs=new api.MjVFS();let model,data;const agents=new Map();
  try{
    model=api.MjModel.from_xml_string(xml,vfs);data=new api.MjData(model);
    for(const id of IDS){
      const robot=createPlanningWorkerRobot(urdfs[id],{rootHeight:ROBOT_CONFIGS[id].rootHeight});
      const Controller=planningControllerClass(ROBOT_CONFIGS[id].Controller,{robotId:id});
      const agent=createRobotAgent({api,model,data,robot,id,metadata:robotFrame(metadata.robots[id],metadata.workbench),Controller,targets:[]});
      // Keep bounded diagnostic evidence when the search budget expires. A
      // timeout alone must not hide repeated collisions behind more retries.
      const originalPlan=agent.controller.plan;
      agent.controller.plan=function(...args){
        try{return originalPlan.apply(this,args);}
        catch(error){
          this.candidateFailures??=[];
          this.candidateFailures.push({parameters:args.slice(1),reason:error.message,detail:error.detail??null});
          if(this.candidateFailures.length>6)this.candidateFailures.shift();
          throw error;
        }
      };
      agents.set(id,agent);
    }
  }catch(error){for(const agent of agents.values())agent.dispose();data?.delete();model?.delete();throw error;}
  finally{vfs.delete();}
  return {
    plan(job){
      const agent=agents.get(job.robotId);if(!agent)throw new Error('未知协作机器人');
      restorePrivateSnapshot(api,model,data,job.physics);
      for(const [id,other]of agents){
        const context=job.robots[id];
        if(!context)throw new Error('缺少本体状态');
        for(const [key,value]of Object.entries(context.controller))other.controller[key]=structuredClone(value);
        for(const [key,value]of Object.entries(context.dynamics))other.dynamics[key]=structuredClone(value);
        other.controller.running=false;other.dynamics.syncRobot();
      }
      const c=agent.controller;
      if(job.kind==='posture')return {ok:true,posture:agent.planPosture(job.posture)};
      if(job.kind==='resume-posture'){
        const check=agent.validateResume({state:job.resume.state,posture:job.posture});
        return {ok:check.safe,...check};
      }
      if(job.kind==='mobile-empty'){
        if(job.carry||job.target)throw new Error('Empty home navigation cannot carry a payload');
        const check=agent.validateMobileRoute({fromMount:job.fromMount,toMount:job.toMount});
        return {ok:check.safe,check,reason:check.safe?null:'空载返航路线存在碰撞'};
      }
      agent.configureTask({target:job.target,arm:job.arm,supportBody:job.supportBody});
      if(job.kind==='resume-manipulation'){
        c.planData=decodePlanningValue(job.planData);
        const check=agent.validateResume({state:job.resume.state,elapsed:c.elapsed,pickOnly:job.pickOnly});
        return {ok:check.safe,...check};
      }
      if(job.kind==='mobile'){
        const check=agent.validateMobileRoute({fromMount:job.fromMount,toMount:job.toMount,objectLocalPose:job.carry?c.objectPose():undefined});
        return {ok:check.safe,check,reason:check.safe?null:'底盘路线存在碰撞'};
      }
      if(job.kind==='loaded-placement'){
        c.planData=decodePlanningValue(job.planData);
        const result=agent.replanHeldPlacement({worldDrop:job.worldDrop,supportBody:job.supportBody});
        return {...result,ok:result.safe,planData:result.safe?encodePlanningValue(c.planData):undefined};
      }
      c.cooperativePlanning=true;c.task='putaway';c.reducedMotion=true;c.planningBudgetMs=10000;c.candidateFailures=[];
      const history=c.history.length;
      if(!c.prepareItem(c.arm,job.target.id,false))return {ok:false,reason:c.message,error:c.planningResult?.error??c.planError};
      let result;
      do{result=c.advancePlanning();}while(result.pending);
      if(!result.ok)return {ok:false,reason:result.reason,error:result.error,detail:{...result.error,candidateFailures:c.candidateFailures}};
      return {ok:true,payload:capturePlanningResult(c,history),steps:result.steps,segmentElapsedMs:result.segmentElapsedMs};
    },
    dispose(){for(const agent of agents.values())agent.dispose();data.delete();model.delete();},
  };
}
