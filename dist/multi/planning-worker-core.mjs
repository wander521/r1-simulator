import {createRobotRuntimeFromParts,loadPhysicsAPI,ROBOT_CONFIGS} from './robot-runtime.mjs?v=20261018-perf-4';
import {parsePlanningRobot} from './planning-robot.mjs?v=20261018-perf-4';
import {decodePlanningValue,restorePlanningJob,capturePlanningResult} from './planning-serialization.mjs?v=20261018-perf-4';
import {createPlanningCheckCache} from './planning-check-cache.mjs?v=20261018-perf-4';
import {enablePlanningWorkerIK} from './planning-ik-context.mjs?v=20261018-perf-4';

/** Shared by the browser worker and its file-loading-only Node test bridge.
 * The live/standalone parsers retain their original default behavior. */
export function createPlanningWorkerRobot(source,{rootHeight=0}={}){
  return enablePlanningWorkerIK(parsePlanningRobot(source,{rootHeight,reuseUnchangedJoints:true}));
}

async function createWorkerRuntime(job){
  const config=ROBOT_CONFIGS[job.robotId];if(!config)throw new Error('未知的后台机器人模型');
  const [api,response]=await Promise.all([loadPhysicsAPI(),fetch(config.urdf)]);
  if(!response.ok)throw new Error(`后台运动链加载失败（${response.status}）`);
  return compilePlanningRuntime(job,{api,source:await response.text()});
}

export function compilePlanningRuntime(job,{api,source}){
  const config=ROBOT_CONFIGS[job.robotId];if(!config)throw new Error('未知的后台机器人模型');
  const robot=createPlanningWorkerRobot(source,{rootHeight:config.rootHeight}),vfs=new api.MjVFS();let model,data;
  try{
    model=api.MjModel.from_xml_string(job.modelXml,vfs);data=new api.MjData(model);
    return createRobotRuntimeFromParts({id:job.robotId,api,model,data,robot,sourceXml:job.sourceXml,modelXml:job.modelXml,
      initialTask:job.taskMode==='pipes'?{mode:'pipes',modelSeed:job.modelPipeSeed}:null,cooperativePlanning:true});
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}

/** Isolated planning service: creates an independent MuJoCo world, restores
 * the exact snapshot and checks original paths. It never calls runtime.step,
 * mj_step or sends back qpos/qvel/contacts as an execution substitute. */
export function createPlanningWorkerService({createRuntime=createWorkerRuntime}={}){
  let runtime=null,key=null,running=false;
  return{
    async run(encoded,{onProgress=()=>{}}={}){
      if(running)throw new Error('后台规划器仍在工作');running=true;
      let checkCache=null;
      try{
        // Exact immutable fields still define reuse, including both original
        // and compiled layout seeds. Comparing them does not allocate another
        // 20 MB escaped XML string on every G1 item.
        const job=decodePlanningValue(encoded),nextKey=[job.robotId,job.sourceXml,job.modelXml,job.modelPipeSeed,job.taskMode];
        onProgress({stage:'loading',steps:0,attempts:0,segmentElapsedMs:0});
        if(!runtime||!key||nextKey.some((value,index)=>!Object.is(value,key[index]))){runtime?.dispose();runtime=await createRuntime(job);key=nextKey;
          if(job.taskMode==='pipes'){
            // Idle randomization preserves the live MjModel's original body,
            // joint and qpos order. Compile the exact modelXml once, build its
            // original metadata, then reset only the current randomized layout.
            if(runtime.taskMode!=='pipes'||runtime.modelXml!==job.modelXml||runtime.modelPipeSeed!==job.modelPipeSeed)
              throw new Error('后台圆管模型拓扑不匹配');
          }}
        if(job.taskMode==='pipes'&&runtime.controller.pipeLayout?.seed!==job.pipeSeed)runtime.reset({seed:job.pipeSeed});
        // A previous job may leave completed command metadata in this private
        // mirror; it must never override the new exact live-world snapshot.
        runtime.controller.cancelPlanning?.('更新后台规划快照');
        restorePlanningJob(runtime,encoded);const controller=runtime.controller,historyLength=controller.history.length;
        // A fresh bounded cache belongs only to this immutable G1 worker job.
        // It reuses exact scratch-check results, never a pose/plan/physics step.
        if(job.robotId==='g1')checkCache=createPlanningCheckCache(runtime.dynamics);
        const accepted=controller.prepareItem(job.request.arm,job.request.id,job.request.continuing);
        if(!accepted)return{pending:false,done:true,ok:false,reason:controller.message,error:controller.planningResult?.error??{type:'unreachable'}};
        let result;do{
          result=controller.advancePlanning();
          onProgress({stage:'planning',steps:result.steps??0,attempts:result.steps??0,segmentElapsedMs:result.segmentElapsedMs??0});
        }while(result.pending);
        if(!result.ok)return result;
        return{pending:false,done:true,ok:true,steps:result.steps,segmentElapsedMs:result.segmentElapsedMs,payload:capturePlanningResult(controller,historyLength)};
      }catch(error){
        runtime?.dispose();runtime=null;key=null;
        return{pending:false,done:true,ok:false,reason:error.message,error:{type:'worker',detail:error.detail??null}};
      }finally{checkCache?.dispose();running=false;}
    },
    dispose(){runtime?.dispose();runtime=null;key=null;},
  };
}
