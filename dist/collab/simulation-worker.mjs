import {loadPhysicsAPI,ROBOT_CONFIGS} from '../multi/robot-runtime.mjs';
import {buildBinCollaborationModel} from './bin-model.mjs?v=20261026-load-2';
import {BinCollaborationCore} from './bin-core.mjs';
import {createPipelinePlannerPool} from './planning-pool.mjs';
import {binPresentationSnapshot} from './bin-snapshot.mjs';
import {IDS,VERSION} from './state.mjs';

let core=null,planner=null,initializing=false;
const send=(type,values={})=>self.postMessage({version:VERSION,type,...values});
const resource=async url=>{const response=await fetch(url);if(!response.ok)throw new Error(`场景资源加载失败（${response.status}）`);return response.text();};
self.onmessage=async({data:message})=>{
  if(message?.version!==VERSION)return;
  try{
    if(message.type==='initialize'){
      if(initializing||core)return;initializing=true;
      send('progress',{progress:{value:.05,label:'加载共享物理运行库与三台本体'}});
      const [api,sources,urdfs]=await Promise.all([loadPhysicsAPI(),Promise.all(IDS.map(async id=>[id,await resource(ROBOT_CONFIGS[id].xml)])),Promise.all(IDS.map(async id=>[id,await resource(ROBOT_CONFIGS[id].urdf)]))]);
      const model=buildBinCollaborationModel(Object.fromEntries(sources),{binCount:4}),description={xml:model.xml,metadata:model.metadata,urdfs:Object.fromEntries(urdfs)};
      send('progress',{progress:{value:.45,label:'建立物料墙、四个实物料盒与独立缓冲区'}});
      planner=createPipelinePlannerPool(description);
      core=new BinCollaborationCore({api,...description,plan:job=>planner.plan(job),cancelPlan:id=>planner.cancel(id)});
      send('progress',{progress:{value:.75,label:'独立工位与后台路径队列已就绪'}});
      initializing=false;send('snapshot',{snapshot:binPresentationSnapshot(core)});return;
    }
    if(!core)throw new Error('共享场景尚未加载完成');
    if(message.type==='advance')core.advance(message.wallNow);
    else if(message.type==='start')core.start({durationSeconds:message.durationSeconds});
    else if(message.type==='stop')core.stop();
    else if(message.type==='reset')core.reset();
    send('snapshot',{snapshot:binPresentationSnapshot(core)});
  }catch(error){
    if(initializing){initializing=false;planner?.dispose();core?.dispose();core=null;}
    if(core)send('snapshot',{snapshot:{...binPresentationSnapshot(core),commandError:error.message}});
    else send('error',{reason:error.message});
  }
};
