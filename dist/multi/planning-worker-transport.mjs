import {PLANNING_PROTOCOL_VERSION,createPlanningMessageEncoder} from './planning-worker-protocol.mjs?v=20261018-perf-4';

export const PLANNING_WORKER_URL=new URL('./planning-worker.mjs?v=20261018-perf-4',import.meta.url);
const nowDefault=()=>globalThis.performance?.now?.()??Date.now();
const failed=(reason,type='worker')=>({pending:false,done:true,ok:false,reason,error:{type}});

/** One reusable worker per station. A cancellation terminates synchronous IK
 * immediately; generation checks also reject messages queued before teardown. */
export function createPlanningWorkerTransport({workerFactory=()=>new Worker(PLANNING_WORKER_URL,{type:'module',name:'robot-path-planner'}),now=nowDefault,timeoutMs=90000}={}){
  let worker=null,generation=0,jobId=0,current=null,result=null,disposed=false;
  const encoder=createPlanningMessageEncoder();
  const terminate=()=>{worker?.terminate();worker=null;encoder.reset();};
  const fail=(reason,type)=>{if(current){result=failed(reason,type);current=null;}terminate();};
  const attach=()=>{
    if(worker)return;
    worker=workerFactory();const activeWorker=worker,activeGeneration=generation;
    worker.onmessage=event=>{
      const message=event.data;
      if(disposed||activeGeneration!==generation||worker!==activeWorker||!current||message?.jobId!==current.jobId||message?.generation!==generation)return;
      // A hidden tab may stop polling while its Worker keeps running. Check
      // the same finite deadline on arrival, before a late success clears it.
      if(now()-current.startedAt>timeoutMs){fail('后台路径检查等待超时，请将物体移近桌面中央后重试','timeout');return;}
      if(message.protocol!==PLANNING_PROTOCOL_VERSION||message.modelRevision!==current.modelRevision){fail('后台路径结果模型版本不匹配，请复位后重试','protocol');return;}
      if(message.type==='progress'){
        if(!message.progress||typeof message.progress!=='object'||Array.isArray(message.progress)){fail('后台路径进度读取失败，请复位后重试','protocol');return;}
        current.progress={...current.progress,...message.progress,elapsedMs:now()-current.startedAt,startedAt:current.startedAt};return;
      }
      if(message.type==='result'){
        const value=message.result;
        if(!value||typeof value!=='object'||typeof value.ok!=='boolean'||value.pending===true||value.done===false){fail('后台路径结果协议无效，请复位后重试','protocol');return;}
        if(value.error?.type==='protocol'){fail(value.reason||'后台模型注册失败，请复位后重试','protocol');return;}
        result={...value,pending:false,done:true};current=null;return;
      }
      fail('后台路径消息类型无效，请复位后重试','protocol');
    };
    worker.onerror=event=>{event.preventDefault?.();if(worker===activeWorker&&activeGeneration===generation)fail('后台路径检查暂时不可用，请复位后重试','worker');};
    worker.onmessageerror=()=>{if(worker===activeWorker&&activeGeneration===generation)fail('后台路径结果读取失败，请复位后重试','message');};
  };
  return{
    submit(job){
      if(disposed)throw new Error('后台路径检查已关闭');if(current)throw new Error('路径检查仍在进行');
      result=null;const startedAt=now();current={jobId:++jobId,startedAt,progress:{stage:'loading',steps:0,attempts:0,segmentElapsedMs:0,elapsedMs:0,startedAt}};
      try{
        attach();const message=encoder.encode(job,{jobId,generation});current.modelRevision=message.modelRevision;worker.postMessage(message);
      }catch(error){fail(`后台路径检查启动失败：${error.message}`,'worker');}
      return jobId;
    },
    poll(){
      if(current){if(now()-current.startedAt>timeoutMs)fail('后台路径检查等待超时，请将物体移近桌面中央后重试','timeout');
        else return{pending:true,done:false,ok:true,progress:{...current.progress,elapsedMs:now()-current.startedAt}};}
      return result??{pending:false,done:true,ok:true};
    },
    get progress(){return current?{...current.progress,elapsedMs:now()-current.startedAt}:null;},
    cancel(reason='已停止后台路径检查'){
      const pending=Boolean(current);generation++;current=null;result=failed(reason,'cancelled');terminate();return pending;
    },
    dispose(){if(disposed)return;this.cancel('机器人已关闭');disposed=true;},
  };
}
