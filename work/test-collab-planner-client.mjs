import assert from 'node:assert/strict';
import {createSharedPlannerClient} from '../dist/collab/planning-client.mjs';
import {VERSION} from '../dist/collab/state.mjs';

class FakeWorker{
  sent=[];terminated=false;
  postMessage(message){assert(!this.terminated);this.sent.push(message);}
  terminate(){this.terminated=true;}
  reply(message){this.onmessage?.({data:{version:VERSION,generation:this.sent[0].generation,...message}});}
}
const workers=[],client=createSharedPlannerClient({xml:'private-model'},{workerFactory:()=>{const worker=new FakeWorker();workers.push(worker);return worker;}});
const tick=()=>new Promise(resolve=>setTimeout(resolve,0));
let passed=0;
try{
  const init=client.initialize();client.cancel();
  await assert.rejects(init,/取消/);assert(workers[0].terminated);passed++;
  const early=client.plan({part:'a'});client.cancel();
  await assert.rejects(early,/取消/);assert(workers[1].terminated);passed++;
  const running=client.plan({part:'b'});workers[2].reply({type:'ready'});await tick();
  const firstJob=workers[2].sent.at(-1);assert.equal(firstJob.type,'plan');
  await assert.rejects(client.plan({part:'overlap'}),/已有/);passed++;
  workers[2].reply({type:'result',jobId:firstJob.jobId+1,result:{ok:true,stale:true}});
  client.cancel();await assert.rejects(running,/取消/);passed++;
  const next=client.plan({part:'c'});workers[3].reply({type:'ready'});await tick();
  // Replies from an old/terminated epoch never resolve the next session.
  workers[2].reply({type:'result',jobId:firstJob.jobId,result:{ok:true,stale:true}});
  const nextJob=workers[3].sent.at(-1);workers[3].reply({type:'result',jobId:nextJob.jobId,result:{ok:true,marker:'current'}});
  assert.deepEqual(await next,{ok:true,marker:'current'});passed++;
  client.dispose();await assert.rejects(client.plan({}),/关闭/);passed++;
  console.log(`PASS ${passed}/6 shared planner lifecycle contracts`);
}finally{client.dispose();}
