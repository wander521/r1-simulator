import {loadPhysicsAPI} from '../multi/robot-runtime.mjs';
import {createSharedPlanningService} from './planner.mjs';
import {VERSION} from './state.mjs';

let service=null,busy=false;
self.onmessage=async({data:message})=>{
  if(message?.version!==VERSION)return;
  const {jobId,generation}=message;
  try{
    if(busy)throw new Error('共享路径规划器正在处理上一任务');busy=true;
    if(message.type==='initialize'){
      service?.dispose();service=createSharedPlanningService({api:await loadPhysicsAPI(),...message.model});
      self.postMessage({version:VERSION,type:'ready',generation});
    }else if(message.type==='plan'){
      if(!service)throw new Error('共享路径规划器尚未初始化');
      const result=service.plan(message.job);self.postMessage({version:VERSION,type:'result',jobId,generation,result});
    }
  }catch(error){self.postMessage({version:VERSION,type:'error',jobId,generation,reason:error.message});}
  finally{busy=false;}
};
