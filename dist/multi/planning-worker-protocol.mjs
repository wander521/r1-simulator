export const PLANNING_PROTOCOL_VERSION=3;
const modelFields=Object.freeze(['robotId','sourceXml','modelXml','modelPipeSeed','taskMode']);
const owns=(object,key)=>Object.prototype.hasOwnProperty.call(object,key);
const record=value=>value!==null&&typeof value==='object'&&!Array.isArray(value);
const integer=(value,min=0)=>Number.isSafeInteger(value)&&value>=min;
const seed=value=>integer(value)&&value<=0xffffffff;
const protocolError=reason=>({pending:false,done:true,ok:false,reason,error:{type:'protocol'}});
const requireValid=(value,reason)=>{if(!value)throw new Error(reason);};

function validateModel(model){
  requireValid(record(model)&&Object.keys(model).length===modelFields.length&&modelFields.every(key=>owns(model,key)),'后台模型注册字段不完整');
  requireValid(['r1','g1','c5'].includes(model.robotId),'后台模型机器人无效');
  requireValid(typeof model.sourceXml==='string'&&model.sourceXml.length>0&&typeof model.modelXml==='string'&&model.modelXml.length>0,'后台模型内容无效');
  requireValid(model.taskMode==='legacy'||model.taskMode==='pipes','后台模型任务无效');
  requireValid(model.taskMode==='pipes'?seed(model.modelPipeSeed):model.modelPipeSeed===null,'后台模型编译布局无效');
  return model;
}
function validateDynamic(job,model){
  requireValid(record(job)&&job.version===2&&!modelFields.some(key=>owns(job,key)),'后台路径快照协议无效');
  requireValid(!['__proto__','constructor','prototype'].some(key=>owns(job,key)),'后台路径快照字段无效');
  requireValid(model.taskMode==='pipes'?seed(job.pipeSeed):job.pipeSeed===null,'后台路径当前布局无效');
  return job;
}
function modelOf(job){
  requireValid(record(job),'后台路径快照无效');
  return validateModel(Object.fromEntries(modelFields.map(key=>[key,job[key]])));
}
const sameModel=(left,right)=>Boolean(left)&&modelFields.every(key=>Object.is(left[key],right[key]));
function validateIdentity(message){
  requireValid(record(message)&&message.type==='plan'&&message.protocol===PLANNING_PROTOCOL_VERSION,'后台路径消息版本无效');
  requireValid(integer(message.jobId,1)&&integer(message.generation)&&integer(message.modelRevision,1),'后台路径消息标识无效');
}

/** One model registration belongs to one live Worker session. The first plan
 * carries its exact model atomically; later plans retain only dynamic state.
 * No XML hash, JSON cache key, approximate identity or unbounded model map is
 * needed. A replacement Worker must start again with revision one. */
export function createPlanningMessageEncoder(){
  let model=null,revision=0;
  return{
    encode(job,{jobId,generation}){
      const next=modelOf(job),dynamic={...job};for(const key of modelFields)delete dynamic[key];
      validateDynamic(dynamic,next);
      const changed=!sameModel(model,next);
      if(changed){requireValid(revision<Number.MAX_SAFE_INTEGER,'后台模型版本已耗尽');model=Object.freeze(next);revision++;}
      const message={type:'plan',protocol:PLANNING_PROTOCOL_VERSION,jobId,generation,modelRevision:revision,job:dynamic};
      validateIdentity(message);if(changed)message.model={...model};return message;
    },
    reset(){model=null;revision=0;},
  };
}

/** Browser and node:worker_threads share this exact message boundary. Model
 * order, compilation and source-bound snapshot restoration still belong to the
 * original service; the endpoint only reconstructs its unchanged full input. */
export function createPlanningWorkerEndpoint({service,postMessage}){
  requireValid(service&&typeof service.run==='function'&&typeof postMessage==='function','后台路径服务无效');
  let model=null,revision=0,generation=null,lastJobId=0,busy=false,disposed=false;
  const reply=(message,type,value)=>postMessage({type,protocol:PLANNING_PROTOCOL_VERSION,jobId:message?.jobId,generation:message?.generation,modelRevision:message?.modelRevision,[type==='progress'?'progress':'result']:value});
  return{
    async handle(message){
      if(disposed||message?.type!=='plan')return;
      let acquired=false;
      try{
        validateIdentity(message);requireValid(!busy,'后台路径服务仍在工作');
        requireValid(generation===null||message.generation===generation,'后台路径会话不匹配');
        requireValid(message.jobId>lastJobId,'后台路径任务顺序无效');
        let registered=model,nextRevision=revision;
        if(owns(message,'model')){
          requireValid(message.modelRevision===revision+1,'后台模型注册顺序无效');
          registered=Object.freeze({...validateModel(message.model)});nextRevision=message.modelRevision;
        }
        requireValid(registered&&message.modelRevision===nextRevision,'后台模型尚未注册或已经过期');
        const dynamic=validateDynamic(message.job,registered),job={...dynamic,...registered};
        // Commit registration only with a valid, ordered plan. A malformed
        // message cannot replace the model used by a subsequent valid plan.
        model=registered;revision=nextRevision;generation=message.generation;lastJobId=message.jobId;busy=true;acquired=true;
        const result=await service.run(job,{onProgress:progress=>reply(message,'progress',progress)});
        if(!disposed)reply(message,'result',result);
      }catch(error){
        // A closed/failed message channel may reject even the failure reply.
        // Consume it here; the transport's finite watchdog still fails closed.
        if(!disposed)try{reply(message,'result',protocolError(error?.message??'后台路径消息无效'));}catch{}
      }
      finally{if(acquired)busy=false;}
    },
    dispose(){if(disposed)return;disposed=true;model=null;service.dispose?.();},
  };
}
