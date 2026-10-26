// Node counterpart of the browser worker transport; the production planning
// service and serialization stay identical. No physics fixture is restored.
import {parentPort} from 'node:worker_threads';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createSharedPlanningService} from '../dist/collab/planner.mjs';
import {VERSION} from '../dist/collab/state.mjs';
let service;
parentPort.on('message',async message=>{
  const {generation,jobId}=message;
  try{
    if(message.type==='initialize'){
      const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
      service=createSharedPlanningService({api,...message.model});
      parentPort.postMessage({version:VERSION,type:'ready',generation});
    }else if(message.type==='plan'){
      const result=service.plan(message.job);
      parentPort.postMessage({version:VERSION,type:'result',generation,jobId,result});
    }
  }catch(error){parentPort.postMessage({version:VERSION,type:'error',generation,jobId,reason:error.message});}
});
