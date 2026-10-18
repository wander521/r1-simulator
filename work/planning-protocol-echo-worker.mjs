import {parentPort} from 'node:worker_threads';
import {createHash} from 'node:crypto';
import {createPlanningWorkerEndpoint} from '../dist/multi/planning-worker-protocol.mjs';

// Message-boundary evidence only: no IK, model compilation or simulated grasp
// occurs here. Full production planning is covered by the physical Worker suite.
const modelFields=['robotId','sourceXml','modelXml','modelPipeSeed','taskMode'];
const sha=value=>createHash('sha256').update(value).digest('hex');
let calls=0;
const service={
  async run(job,{onProgress}){
    calls++;onProgress({stage:'planning',steps:calls});
    const dynamic={...job};for(const key of modelFields)delete dynamic[key];
    return{pending:false,done:true,ok:true,payload:{calls,robotId:job.robotId,sourceXmlHash:sha(job.sourceXml),modelXmlHash:sha(job.modelXml),
      modelPipeSeed:job.modelPipeSeed,taskMode:job.taskMode,pipeSeed:job.pipeSeed,dynamicHash:sha(JSON.stringify(dynamic))}};
  },
};
const endpoint=createPlanningWorkerEndpoint({service,postMessage:message=>parentPort.postMessage(message)});
parentPort.on('message',message=>endpoint.handle(message));
