import {parentPort} from 'node:worker_threads';
import {readFile} from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {createPlanningWorkerService,createPlanningWorkerRobot} from '../dist/multi/planning-worker-core.mjs';

// The browser worker uses fetch for Wasm/URDF. Only that file-loading boundary
// is adapted here; the service, parser, controller, IK and collision gates are
// the actual shipped implementation, on a genuine node:worker_threads thread.
let apiPromise;
async function createRuntime(job){
  apiPromise??=createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
  const [api,source]=await Promise.all([apiPromise,readFile(new URL(ROBOT_CONFIGS[job.robotId].urdf),'utf8')]);
  const robot=createPlanningWorkerRobot(source,{rootHeight:ROBOT_CONFIGS[job.robotId].rootHeight}),vfs=new api.MjVFS();let model,data;
  try{
    model=api.MjModel.from_xml_string(job.sourceXml,vfs);data=new api.MjData(model);
    return createRobotRuntimeFromParts({id:job.robotId,api,model,data,robot,sourceXml:job.sourceXml,cooperativePlanning:true});
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
const service=createPlanningWorkerService({createRuntime});
parentPort.on('message',async message=>{
  if(message?.type!=='plan')return;
  const{jobId,generation,job}=message;
  const result=await service.run(job,{onProgress:progress=>parentPort.postMessage({type:'progress',jobId,generation,progress})});
  parentPort.postMessage({type:'result',jobId,generation,result});
});
