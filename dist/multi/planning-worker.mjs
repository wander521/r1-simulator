import {createPlanningWorkerService} from './planning-worker-core.mjs?v=20261016-worker-1';

const service=createPlanningWorkerService();
self.onmessage=async event=>{
  const message=event.data;if(message?.type!=='plan')return;
  const {jobId,generation,job}=message;
  const result=await service.run(job,{onProgress:progress=>self.postMessage({type:'progress',jobId,generation,progress})});
  self.postMessage({type:'result',jobId,generation,result});
};
