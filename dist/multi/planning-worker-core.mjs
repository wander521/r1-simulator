import {createRobotRuntimeFromParts,loadPhysicsAPI,ROBOT_CONFIGS} from './robot-runtime.mjs?v=20261016-worker-1';
import {parsePlanningRobot} from './planning-robot.mjs?v=20261016-worker-1';
import {decodePlanningValue,restorePlanningJob,capturePlanningResult} from './planning-serialization.mjs?v=20261016-worker-1';

async function createWorkerRuntime(job){
  const config=ROBOT_CONFIGS[job.robotId];if(!config)throw new Error('未知的后台机器人模型');
  const [api,response]=await Promise.all([loadPhysicsAPI(),fetch(config.urdf)]);
  if(!response.ok)throw new Error(`后台运动链加载失败（${response.status}）`);
  const robot=parsePlanningRobot(await response.text(),{rootHeight:config.rootHeight}),vfs=new api.MjVFS();let model,data;
  try{
    model=api.MjModel.from_xml_string(job.sourceXml,vfs);data=new api.MjData(model);
    return createRobotRuntimeFromParts({id:job.robotId,api,model,data,robot,sourceXml:job.sourceXml,cooperativePlanning:true});
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
      try{
        const job=decodePlanningValue(encoded),nextKey=JSON.stringify([job.robotId,job.sourceXml,job.modelXml,job.modelPipeSeed,job.taskMode,job.pipeSeed]);
        onProgress({stage:'loading',steps:0,attempts:0,segmentElapsedMs:0});
        if(!runtime||key!==nextKey){runtime?.dispose();runtime=await createRuntime(job);key=nextKey;
          if(job.taskMode==='pipes'){
            // Idle randomization preserves the live MjModel's original body,
            // joint and qpos order. Rebuilding from the *current* layout seed
            // would permute the eight randomized pipe definitions and copy
            // authoritative qpos into the wrong bodies. Compile the exact
            // original scene first, then reset only its current layout.
            const configured=runtime.configureTask('pipes',{seed:job.modelPipeSeed});if(!configured.ok)throw new Error(configured.reason);
            if(job.pipeSeed!==job.modelPipeSeed)runtime.reset({seed:job.pipeSeed});
          }}
        // A previous job may leave completed command metadata in this private
        // mirror; it must never override the new exact live-world snapshot.
        runtime.controller.cancelPlanning?.('更新后台规划快照');
        restorePlanningJob(runtime,encoded);const controller=runtime.controller,historyLength=controller.history.length;
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
      }finally{running=false;}
    },
    dispose(){runtime?.dispose();runtime=null;key=null;},
  };
}
