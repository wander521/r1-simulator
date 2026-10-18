import * as THREE from '../vendor/three.module.min.js';
import {CROUCH,SPREAD as R1_SPREAD} from '../r1/grasp-controller.mjs';
import {STAND_POSE as G1_STAND,qToOpening as g1Opening,GRIPPER_Q_MAX as G1_MAX} from '../g1/g1-profile.mjs';
import {qToOpening as c5Opening,GRIPPER_Q_MAX as C5_MAX} from '../c5/c5-profile.mjs';
import {PlanningBudgetError,isPlanningBudgetError,PLANNING_BUDGET_MESSAGE,withPlanningBudget} from './planning-budget.mjs?v=20261018-perf-4';

const now=()=>globalThis.performance?.now?.()??Date.now();
const unreachable='目标超出机器人可达范围，请将物体或放置位置移近';
const budgetMessage=PLANNING_BUDGET_MESSAGE;
export const PLANNING_SEGMENT_BUDGET_MS=10000;
export const G1_PIPE_PLANNING_SEGMENT_BUDGET_MS=30000;
const resultError=(reason,type,detail={})=>({ok:false,reason,error:{type,detail}});

function endpoints(controller,target,robotId){
  if(robotId==='r1')return controller.placementEndpoints(target);
  const start=controller.objectPose(target.id).position,drop=target.drop.clone();drop.y=start.y;
  if(controller.task==='stack'){
    const base=controller.targets.find(item=>item.id==='blue').drop;drop.x=base.x;drop.z=base.z;
    if(controller.itemIndex>0){
      const supportId=['blue','red'][controller.itemIndex-1],support=controller.objectPose(supportId).position;
      const supportTarget=controller.targets.find(item=>item.id===supportId);
      drop.set(support.x,support.y+supportTarget.h+target.h,support.z);
    }
  }
  return {start,drop};
}

/** A triangle-inequality bound from the original kinematic tree. It only
 * rejects points outside every joint configuration, not difficult IK poses. */
export function planningPreflight(controller,{robotId,arm=controller.arm,target}={}){
  if(!target)throw Object.assign(new Error('请选择有效的桌面物体'),{detail:{type:'invalid-target'}});
  const {start,drop}=endpoints(controller,target,robotId);
  const rotation=controller.objectPose(target.id).quaternion;
  if(![rotation.x,rotation.y,rotation.z,rotation.w].every(Number.isFinite)||Math.abs(rotation.lengthSq()-1)>.0001)
    throw Object.assign(new Error('物体姿态无效，请复位后重试'),{detail:{type:'invalid-orientation'}});
  const clearance=robotId==='r1'?new THREE.Vector3(0,.13,0):new THREE.Vector3(.04,robotId==='g1'?.11:.14,0);
  const dropClearance=robotId==='g1'&&controller.pipeLayout?new THREE.Vector3(0,.08,0):clearance;
  const points=[start,drop,start.clone().add(clearance),drop.clone().add(dropClearance)];
  if(points.some(point=>![point.x,point.y,point.z].every(Number.isFinite)))
    throw Object.assign(new Error('物体或放置位置无效，请复位后重试'),{detail:{type:'invalid-position'}});
  // Keep the established task workspace checks, once per search rather than
  // throwing their identical errors for every seed/tilt combination.
  if(robotId==='g1'&&(start.x>.65||Math.hypot(start.x,start.z)>.66||drop.x>.65))
    throw Object.assign(new Error('目标超出站立抓取范围，请将物体移近桌面中央'),{detail:{type:'workspace'}});
  if(robotId==='c5'&&(start.x<.25||start.x>.68||Math.hypot(start.x,start.z)>.72))
    throw Object.assign(new Error('目标超出当前作业范围，请移回桌面中央'),{detail:{type:'workspace'}});
  const robot=controller.robot;
  if(robotId==='r1')robot.setJointValues({...CROUCH,...R1_SPREAD.right,...R1_SPREAD.left});
  else if(robotId==='g1')robot.setJointValues(G1_STAND);
  robot.updateMatrixWorld(true);
  const pivot=robot.joints[robotId==='r1'?'torso_joint4':robotId==='g1'?'waist_yaw_joint':'shoulder_joint'];
  const link=robot.getObjectByName(robotId==='r1'?`${arm}_gripper_link`:`${robotId==='c5'?'left':arm}_dex1_base_link`);
  if(!pivot||!link)throw new Error('机器人运动链不完整，请重新加载');
  const tipScale=link.getWorldScale(new THREE.Vector3());
  let radius=(robotId==='r1'?.068:.12)*Math.max(Math.abs(tipScale.x),Math.abs(tipScale.y),Math.abs(tipScale.z)),node=link;
  // All production source joints are revolute on this ancestor chain. Their
  // rotations preserve each local segment length. Include world scale to
  // keep the bound conservative if a source tree has a fixed scale.
  while(node&&node!==pivot){
    const parentScale=node.parent?.getWorldScale(new THREE.Vector3())??new THREE.Vector3(1,1,1);
    radius+=node.position.length()*Math.max(Math.abs(parentScale.x),Math.abs(parentScale.y),Math.abs(parentScale.z));node=node.parent;
  }
  if(node!==pivot)throw new Error('机器人运动链不完整，请重新加载');
  const center=pivot.getWorldPosition(new THREE.Vector3());
  // Position tolerance remains the existing physical planner tolerance. This
  // bound cannot reject a point the original solver accepts within 3 mm.
  for(const point of points)if(point.distanceTo(center)>radius+.003)
    throw Object.assign(new Error(unreachable),{detail:{type:'reach',point:point.toArray(),center:center.toArray(),radius}});
  return {center:center.toArray(),radius,start:start.toArray(),drop:drop.toArray()};
}

function* route(controller,target,args){
  if(typeof controller.planCandidates==='function')yield* controller.planCandidates(target,...args);
  else controller.plan(target,...args);
}

function* prepare(controller,job,robotId){
  const {arm,id,continuing,before,target}=job;
  let error;
  if(robotId==='r1'){
    let attempts=0,best,valid=0;
    const sign=arm==='right'?1:-1,candidates=controller.waistAssistAngles(target);
    for(const angle of [.3*sign,.4*sign,.2*sign,.1*sign,0])if(!candidates.some(value=>Math.abs(value-angle)<1e-6))candidates.push(angle);
    controller.planData=null;
    for(const wristPitch of [1.35,1.50]){
      search:for(const actualYaw of candidates)for(const yaw of [.35,.7,0,-.35,-.7]){
        attempts++;try{
          yield* route(controller,target,[actualYaw*sign,yaw*sign,{wristPitch}]);valid++;
          if(!best||controller.planData.naturalSelection.score<best.plan.naturalSelection.score)best={plan:controller.planData,crouch:{...controller.crouch}};
          if(valid>=4){yield;break search;}
        }catch(caught){if(isPlanningBudgetError(caught))throw caught;error=caught;}
        yield;
      }
      if(best)break;
    }
    controller.applyPose(before);controller.dynamics?.syncRobot();
    if(!best)throw error??new Error('未找到自然且连续的取放路径');
    controller.planData=best.plan;controller.crouch=best.crouch;
    controller.planData.waistAssist={used:Math.abs(controller.crouch.torso_joint4)>.005,attempts,candidates,actualYaw:controller.crouch.torso_joint4,sharedPickPlaceYaw:true,legacyPriority:false,validNaturalRoutes:valid};
    controller.planData.alignFromYaw=before.torso_joint4??0;
    const turnNeeded=Math.abs(controller.planData.alignFromYaw-controller.crouch.torso_joint4)>.005;
    controller.running=true;controller.opening=.05;controller.evidence={lift:0,maxSpeed:0,dual:false,stable:0};controller.transition(continuing?(turnNeeded?'align_clear':'approach'):'spread');
    return true;
  }
  if(robotId==='g1'){
    search:for(const lateral of [.14,.17,.11])for(const yaw of [0,.15,-.15,.3,-.3])for(let branch=0;branch<6;branch++){
      try{yield* route(controller,target,[yaw,branch,lateral]);error=null;break search;}catch(caught){if(isPlanningBudgetError(caught))throw caught;error=caught;}
      yield;
    }
  }else{
    search:for(const tilt of [20,12,28])for(const yaw of [0,.2,-.2,.4,-.4])for(let branch=0;branch<9;branch++){
      try{yield* route(controller,target,[branch,yaw,tilt*Math.PI/180]);error=null;break search;}catch(caught){if(isPlanningBudgetError(caught))throw caught;error=caught;}
      yield;
    }
  }
  controller.applyPose(before);controller.dynamics?.syncRobot();
  if(error)throw error;
  // These are the original swept opening checks, with the source gripper
  // dimensions and contact tolerances unchanged.
  const open=robotId==='g1'?g1Opening(G1_MAX):c5Opening(C5_MAX);
  if(!controller.setOpenings(robotId==='g1'?{left:open,right:open}:open)){
    controller.running=false;controller.state='failed';throw new Error(controller.message||'夹爪打开路径不可达');
  }
  controller.continuing=continuing;controller.running=true;controller.evidence={lift:0,maxSpeed:0,dual:false,stable:0};controller.transition(robotId==='g1'?(continuing?'recenter':'spread'):'approach');
  return controller.running;
}

/** Runtime-only opt-in adapter. Synchronous standalone controllers and their
 * established physics tests retain the original prepareItem implementation. */
export function planningControllerClass(LegacyController,{robotId}){
  if(!['r1','g1','c5'].includes(robotId))throw new Error(`未知机器人：${robotId}`);
  return class CooperativePlanningController extends LegacyController{
    get planning(){return Boolean(this._planningJob);}
    prepareItem(arm,id,continuing){
      if(!this.cooperativePlanning)return super.prepareItem(arm,id,continuing);
      if(this.planning)return false;
      this.arm=robotId==='c5'?'left':arm;this.selected=id;
      const before={...this.pose},target=this.targets.find(item=>item.id===id);
      this.planningResult=null;this.planError=null;this.planData=null;
      try{planningPreflight(this,{robotId,arm:this.arm,target});}
      catch(error){
        this._restorePlanningPose(before);this.planError=error.detail??null;
        this.planningResult=resultError(error.message,'unreachable',error.detail);
        this.running=false;this.state='failed';this.message=error.message;return false;
      }
      this._restorePlanningPose(before);
      // This measures elapsed time inside computation segments, including OS
      // preemption, and excludes time suspended between browser frames. G1's
      // redundant pipe search retains its existing extra tilt/direction grid.
      const defaultBudget=robotId==='g1'&&this.pipeLayout?G1_PIPE_PLANNING_SEGMENT_BUDGET_MS:PLANNING_SEGMENT_BUDGET_MS;
      const budgetMs=Number.isFinite(this.planningBudgetMs)?Math.max(0,this.planningBudgetMs):defaultBudget;
      const job={arm:this.arm,id,continuing,before,target,segmentElapsedMs:0,steps:0,budgetMs};
      job.iterator=prepare(this,job,robotId);this._planningJob=job;
      this.running=true;this.state='planning';this.message='正在检查自然且连续的抓取路径…';return true;
    }
    _restorePlanningPose(before){
      if(this.dynamics)this.dynamics.syncRobot();
      else{this.robot.setJointValues(before);this.robot.updateMatrixWorld(true);}
    }
    _finishPlanningError(job,error,type='unreachable'){
      this._planningJob=null;job.iterator.return?.();this.applyPose(job.before);this._restorePlanningPose(job.before);
      this.planData=null;this.planError=error.detail??null;this.running=false;this.state='failed';this.reaction=null;this.message=error.message;
      const result={...resultError(error.message,type,error.detail),segmentElapsedMs:job.segmentElapsedMs,steps:job.steps};
      this.planningResult=result;return {pending:false,done:true,...result,result};
    }
    advancePlanning(){
      const job=this._planningJob;
      if(!job){const result=this.planningResult??{ok:!['failed','stopped'].includes(this.state)};return {pending:false,done:true,...result,result};}
      if(job.segmentElapsedMs>=job.budgetMs)return this._finishPlanningError(job,Object.assign(new Error(budgetMessage),{detail:{type:'budget',budgetMs:job.budgetMs,segmentElapsedMs:job.segmentElapsedMs}}),'budget');
      const started=now();let step,budgetFailure;
      const checkpoint=()=>{const segmentElapsedMs=job.segmentElapsedMs+now()-started;if(segmentElapsedMs>=job.budgetMs)throw budgetFailure??=(new PlanningBudgetError({budgetMs:job.budgetMs,segmentElapsedMs}));};
      try{step=withPlanningBudget(this.robot,checkpoint,()=>job.iterator.next(),this.dynamics);}
      catch(error){job.segmentElapsedMs+=now()-started;error=budgetFailure??error;return this._finishPlanningError(job,error,isPlanningBudgetError(error)?'budget':'unreachable');}
      this._restorePlanningPose(job.before);
      job.segmentElapsedMs+=now()-started;job.steps++;
      if(step.done){
        this._planningJob=null;const result={ok:Boolean(step.value),segmentElapsedMs:job.segmentElapsedMs,steps:job.steps};this.planningResult=result;
        return {pending:false,done:true,...result,result};
      }
      return {pending:true,done:false,ok:true,segmentElapsedMs:job.segmentElapsedMs,steps:job.steps};
    }
    cancelPlanning(reason='已停止路径检查'){
      const job=this._planningJob;if(!job)return false;
      this._planningJob=null;job.iterator.return?.();this.applyPose(job.before);this._restorePlanningPose(job.before);
      this.planData=null;this.running=false;this.state='stopped';this.reaction=null;this.message=reason;
      this.planningResult={...resultError(reason,'cancelled'),segmentElapsedMs:job.segmentElapsedMs,steps:job.steps};return true;
    }
    stop(...args){this.cancelPlanning();return super.stop(...args);}
    reset(...args){this.cancelPlanning();this.planningResult=null;return super.reset(...args);}
  };
}
