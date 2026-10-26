import {createSharedPlannerClient} from './planning-client.mjs';

/** Bounded independent work queues. Planning is never a simulation clock
 * barrier. At most one request per robot and two expensive planners at once;
 * a failed G1 request cannot cancel an executing/queued C5 request. */
export function createPipelinePlannerPool(model,{concurrency=2,clientFactory=()=>createSharedPlannerClient(model)}={}){
  if(!Number.isInteger(concurrency)||concurrency<1||concurrency>3)throw new Error('规划并发数必须为 1–3');
  const queue=[],requests=new Map(),slots=Array.from({length:concurrency},()=>({client:null,request:null}));let disposed=false;
  function pump(){
    if(disposed)return;
    for(const slot of slots){
      if(slot.request)continue;const request=queue.shift();if(!request)break;
      if(request.cancelled)continue;
      slot.request=request;
      Promise.resolve().then(()=>{if(request.cancelled)return;slot.client??=clientFactory();return slot.client.plan(request.job);}).then(
        result=>{if(!request.cancelled)request.resolve(result);},
        error=>{if(!request.cancelled)request.reject(error);},
      ).finally(()=>{
        if(requests.get(request.job.robotId)===request)requests.delete(request.job.robotId);
        if(slot.request===request)slot.request=null;pump();
      });
    }
  }
  function cancel(robotId,reason=new Error('该工位规划已取消')){
    const matching=robotId?[requests.get(robotId)].filter(Boolean):[...requests.values()];
    for(const request of matching){
      request.cancelled=true;request.reject(reason);requests.delete(request.job.robotId);
      const queued=queue.indexOf(request);if(queued>=0)queue.splice(queued,1);
      const slot=slots.find(s=>s.request===request);
      if(slot){slot.client?.cancel(reason);/* Keep the slot until its finally runs. */}
    }
    pump();
  }
  return {
    plan(job){
      if(disposed)return Promise.reject(new Error('流水线规划池已关闭'));
      if(!['r1','g1','c5'].includes(job?.robotId))return Promise.reject(new Error('未知规划工位'));
      if(requests.has(job.robotId))return Promise.reject(new Error('该机器人已有待处理路径，拒绝重复规划'));
      return new Promise((resolve,reject)=>{const request={job,resolve,reject,cancelled:false};requests.set(job.robotId,request);queue.push(request);pump();});
    },cancel,
    get status(){return {running:slots.filter(s=>s.request).map(s=>s.request.job.robotId),queued:queue.map(r=>r.job.robotId),disposed};},
    dispose(){disposed=true;cancel();for(const slot of slots)slot.client?.dispose();},
  };
}
