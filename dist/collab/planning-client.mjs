import {VERSION} from './state.mjs';

/** One model registration per Worker generation. Stops terminate computation,
 * not just its progress label; old replies cannot enter the new session. */
export function createSharedPlannerClient(model,{workerFactory=()=>new Worker(new URL('./planner-worker.mjs',import.meta.url),{type:'module'})}={}){
  let worker=null,generation=0,ready=null,pending=null,sequence=0,disposed=false,initializationTimer=null,rejectInitialization=null;
  function boot(){
    if(disposed)throw new Error('共享规划器已关闭');if(ready)return ready;
    const epoch=++generation;worker=workerFactory();
    ready=new Promise((resolve,reject)=>{
      rejectInitialization=reject;
      initializationTimer=setTimeout(()=>{if(epoch===generation){reject(new Error('共享规划器初始化超时'));cancel();}},35000);
      worker.onmessage=({data:message})=>{
        if(message?.version!==VERSION||message.generation!==generation||epoch!==generation)return;
        if(message.type==='ready'){clearTimeout(initializationTimer);initializationTimer=null;rejectInitialization=null;resolve();return;}
        if(message.type==='result'&&pending?.jobId===message.jobId){const job=pending;pending=null;clearTimeout(job.timer);job.resolve(message.result);return;}
        if(message.type==='error'){
          const error=new Error(message.reason||'共享路径检查失败');
          if(pending?.jobId===message.jobId){const job=pending;pending=null;clearTimeout(job.timer);job.reject(error);}else {reject(error);cancel(error);}
        }
      };
      worker.onerror=event=>{const error=new Error(event.message||'共享规划进程异常');reject(error);if(pending){clearTimeout(pending.timer);pending.reject(error);pending=null;}cancel();};
      worker.postMessage({version:VERSION,type:'initialize',generation:epoch,model});
    });
    // Cancellation during initialization is expected and never becomes an
    // unhandled rejection when no task is awaiting this generation yet.
    ready.catch(()=>{});return ready;
  }
  function cancel(reason=new Error('共享路径检查已取消')){
    generation++;clearTimeout(initializationTimer);initializationTimer=null;worker?.terminate();worker=null;ready=null;
    rejectInitialization?.(reason);rejectInitialization=null;
    if(pending){const job=pending;pending=null;clearTimeout(job.timer);job.reject(reason);}
  }
  return {initialize:boot,
    async plan(job){
      const initialization=boot(),requestedGeneration=generation;
      await initialization;if(disposed)throw new Error('共享规划器已关闭');
      if(requestedGeneration!==generation||!worker)throw new Error('共享路径检查已取消');
      if(pending)throw new Error('公共作业区已有待处理路径');
      const jobId=++sequence,epoch=generation;
      return new Promise((resolve,reject)=>{
        const timer=setTimeout(()=>{if(pending?.jobId===jobId){reject(new Error('共享路径检查超时，已终止计算'));pending=null;cancel();}},15000);
        pending={jobId,resolve,reject,timer};worker.postMessage({version:VERSION,type:'plan',generation:epoch,jobId,job});
      });
    },cancel,
    dispose(){cancel();disposed=true;},
  };
}
