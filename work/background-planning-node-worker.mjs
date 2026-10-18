import {parentPort} from 'node:worker_threads';
import {readFile} from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {createPlanningWorkerService,compilePlanningRuntime} from '../dist/multi/planning-worker-core.mjs';
import {createPlanningWorkerEndpoint} from '../dist/multi/planning-worker-protocol.mjs';

// The browser worker uses fetch for Wasm/URDF. Only that file-loading boundary
// is adapted here; the service, parser, controller, IK and collision gates are
// the actual shipped implementation, on a genuine node:worker_threads thread.
let apiPromise;
async function createRuntime(job){
  apiPromise??=createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
  const [api,source]=await Promise.all([apiPromise,readFile(new URL(ROBOT_CONFIGS[job.robotId].urdf),'utf8')]);
  return compilePlanningRuntime(job,{api,source});
}
const service=createPlanningWorkerService({createRuntime});
const endpoint=createPlanningWorkerEndpoint({service,postMessage:message=>parentPort.postMessage(message)});
parentPort.on('message',message=>endpoint.handle(message));
