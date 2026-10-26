import {VERSION} from './state.mjs';

/** Browser facade: parsing, full-scene physics and IK stay off the UI thread. */
export async function createCollaborationEngine({onProgress=()=>{},workerFactory=()=>new Worker(new URL('./simulation-worker.mjs?v=20261026-load-2',import.meta.url),{type:'module'})}={}){
  const worker=workerFactory();let disposed=false,pendingFrame=false,lastSent=0,snapshot={state:'loading',robots:[],parts:[],trays:[],events:[],elapsedSeconds:0,durationSeconds:300,completedOrders:0,completedParts:0};
  let resolveReady,rejectReady;const ready=new Promise((resolve,reject)=>{resolveReady=resolve;rejectReady=reject;});
  const command=(type,value={})=>{if(disposed)throw new Error('协作场景已关闭');worker.postMessage({version:VERSION,type,...value});};
  worker.onmessage=({data:message})=>{
    if(disposed||message?.version!==VERSION)return;
    if(message.type==='progress')onProgress(message.progress);
    if(message.type==='snapshot'){snapshot=message.snapshot;pendingFrame=false;if(snapshot.state!=='loading')resolveReady();}
    if(message.type==='error'){
      pendingFrame=false;snapshot={...snapshot,state:'failed',error:message.reason};rejectReady(new Error(message.reason));
    }
  };
  worker.onerror=event=>{pendingFrame=false;const error=new Error(event.message||'协作仿真进程意外退出');snapshot={...snapshot,state:'failed',error:error.message};rejectReady(error);};
  command('initialize');try{await ready;}catch(error){disposed=true;worker.terminate();throw error;}
  return {get ready(){return !disposed&&snapshot.state!=='loading';},get snapshot(){return snapshot;},
    start({durationSeconds=300}={}){command('start',{durationSeconds});},
    stop(){command('stop');},reset(){command('reset');},
    advance(now){if(disposed||pendingFrame||now-lastSent<33)return;lastSent=now;pendingFrame=true;command('advance',{wallNow:now});},
    dispose(){if(disposed)return;disposed=true;worker.terminate();},
  };
}
