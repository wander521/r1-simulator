import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {performance} from 'node:perf_hooks';
import {createCameraFeedQueue} from '../dist/multi/visuals/camera-feed-queue.mjs';

// Controlled promises test the production queue, not a fake GPU speedup.
const root=new URL('../',import.meta.url),args=process.argv.slice(2);
const reportPath=args.find(value=>value.startsWith('--report='))?.slice(9)??'work/validation/performance-v2/camera-feed-queue-contracts.json';
const paths=['dist/multi/visuals/camera-feed-queue.mjs','dist/multi/visuals/camera-feeds.mjs','dist/multi/app.mjs','work/test-camera-feed-queue-perf-v2.mjs'];
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
const sourceHashes=await hashes(),startedAt=new Date().toISOString(),started=performance.now(),results=[],unhandled=[];
const observe=error=>unhandled.push(error);process.on('unhandledRejection',observe);
function deferred(){let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no;});return{promise,resolve,reject};}
const flush=()=>new Promise(resolve=>setImmediate(resolve));
async function bounded(promise){let timer;try{return await Promise.race([promise,new Promise((_,reject)=>{timer=setTimeout(()=>reject(new Error('Queue contract failed to settle within 250ms')),250);})]);}finally{clearTimeout(timer);}}
async function check(name,fn){try{await fn();results.push({name,pass:true});}catch(error){results.push({name,pass:false,error:{message:error.message,stack:error.stack}});}}

await check('Initial idle lifetime is immediately fulfilled without rendering',async()=>{
  const queue=createCameraFeedQueue({render:()=>assert.fail('Unexpected render'),present:()=>assert.fail('Unexpected present')});
  assert.equal(queue.pending,false);await bounded(queue.whenIdle());queue.invalidate();assert.equal(queue.pending,false);await bounded(queue.whenIdle());
});
await check('One in-flight request owns its buffer, rejects overlap, then permits exact buffer reuse',async()=>{
  const reads=[],presents=[],pixels=new Uint8Array([1,2,3,4]);
  const queue=createCameraFeedQueue({render:context=>{const read=deferred();reads.push({read,context});return read.promise;},present:(buffer,context)=>presents.push({buffer,context,copy:[...buffer]})});
  const context={index:0};assert.equal(queue.request(context),true);assert.equal(queue.pending,true);const idle=queue.whenIdle();
  assert.equal(queue.request({index:1}),false);assert.equal(reads.length,1);let settled=false;idle.then(()=>settled=true);await flush();assert.equal(settled,false);
  reads[0].read.resolve(pixels);await bounded(idle);assert.equal(queue.pending,false);assert.equal(presents[0].buffer,pixels);assert.equal(presents[0].context,context);assert.deepEqual(presents[0].copy,[1,2,3,4]);
  assert.equal(queue.request({index:1}),true);pixels.set([5,6,7,8]);reads[1].read.resolve(pixels);await bounded(queue.whenIdle());assert.equal(presents[1].buffer,pixels);assert.deepEqual(presents[1].copy,[5,6,7,8]);
});
await check('Invalidation suppresses stale pixels but never releases shared readback memory early',async()=>{
  const read=deferred(),pixels=new Uint8Array(4);let draws=0,presents=0,predicate=0;
  const queue=createCameraFeedQueue({render:()=>{draws++;return read.promise;},present:()=>presents++,isCurrent:()=>{predicate++;return true;}});
  queue.request({});queue.invalidate();queue.invalidate();assert.equal(queue.pending,true);assert.equal(queue.request({}),false);assert.equal(draws,1);
  read.resolve(pixels);await bounded(queue.whenIdle());assert.equal(queue.pending,false);assert.equal(presents,0);assert.equal(predicate,0,'A generation-stale frame must not even consult its context');
});
await check('Stale readback rejection is consumed without presenting an obsolete error',async()=>{
  const read=deferred();let errors=0;const queue=createCameraFeedQueue({render:()=>read.promise,present:()=>assert.fail(),onError:()=>errors++});
  queue.request({});const idle=queue.whenIdle();queue.invalidate();read.reject(new Error('stale GPU readback'));await bounded(idle);assert.equal(errors,0);assert.equal(queue.pending,false);
});
await check('Focus, controller, arm, theme and visibility snapshot changes independently suppress presentation',async()=>{
  for(const field of ['focus','controller','arm','theme','visible']){
    const read=deferred(),station={controller:{},arm:'left'},state={focus:station,theme:'space',visible:true};let presents=0;
    const context={station,controller:station.controller,arm:station.arm,theme:state.theme};
    const queue=createCameraFeedQueue({render:()=>read.promise,present:()=>presents++,
      isCurrent:record=>state.visible&&state.focus===record.station&&record.station.controller===record.controller&&record.station.arm===record.arm&&state.theme===record.theme});
    queue.request(context);
    if(field==='focus')state.focus={};else if(field==='controller')station.controller={};else if(field==='arm')station.arm='right';else if(field==='theme')state.theme='orbit';else state.visible=false;
    read.resolve(new Uint8Array(4));await bounded(queue.whenIdle());assert.equal(presents,0,field);assert.equal(queue.pending,false);
  }
});
await check('A context predicate that invalidates reentrantly cannot accept its now-stale frame',async()=>{
  let presents=0;const queue=createCameraFeedQueue({render:()=>new Uint8Array(4),present:()=>presents++,isCurrent:()=>{queue.invalidate();return true;}});
  queue.request({});await bounded(queue.whenIdle());assert.equal(presents,0);assert.equal(queue.pending,false);
});
await check('Synchronous submission errors report once, settle idle, and allow recovery',async()=>{
  const errors=[],pixels=new Uint8Array(4);let fail=true,presents=0;
  const queue=createCameraFeedQueue({render:()=>{if(fail)throw new Error('sync submit');return pixels;},present:()=>presents++,onError:error=>errors.push(error.message)});
  assert.equal(queue.request({}),false);assert.equal(queue.pending,false);await bounded(queue.whenIdle());assert.deepEqual(errors,['sync submit']);
  fail=false;assert.equal(queue.request({}),true);await bounded(queue.whenIdle());assert.equal(presents,1);
});
await check('Asynchronous submission rejection is consumed, reported once, and does not poison reuse',async()=>{
  const errors=[];let fail=true,presents=0;
  const queue=createCameraFeedQueue({render:()=>fail?Promise.reject(new Error('async read')):new Uint8Array(4),present:()=>presents++,onError:error=>errors.push(error.message)});
  assert.equal(queue.request({}),true);await bounded(queue.whenIdle());assert.equal(queue.pending,false);assert.deepEqual(errors,['async read']);
  fail=false;queue.request({});await bounded(queue.whenIdle());assert.equal(presents,1);
});
await check('A synchronously throwing presentation reports once and releases pending ownership',async()=>{
  const errors=[];let fail=true,presents=0;
  const queue=createCameraFeedQueue({render:()=>new Uint8Array(4),present:()=>{if(fail)throw new Error('canvas present');presents++;},onError:error=>errors.push(error.message)});
  queue.request({});await bounded(queue.whenIdle());assert.equal(queue.pending,false);assert.deepEqual(errors,['canvas present']);
  fail=false;queue.request({});await bounded(queue.whenIdle());assert.equal(presents,1);
});
await check('Asynchronous presentation owns the pixel buffer until fulfilled or rejected',async()=>{
  for(const reject of [false,true]){
    const presentation=deferred(),errors=[];let presents=0;
    const pixels=new Uint8Array([1,2,3,4]);const queue=createCameraFeedQueue({render:()=>pixels,present:buffer=>{assert.equal(buffer,pixels);presents++;return presentation.promise;},onError:error=>errors.push(error.message)});
    queue.request({});const idle=queue.whenIdle();await flush();assert.equal(presents,1);assert.equal(queue.pending,true);assert.equal(queue.request({}),false);
    if(reject)presentation.reject(new Error('async present'));else presentation.resolve();await bounded(idle);assert.equal(queue.pending,false);assert.deepEqual(errors,reject?['async present']:[]);
  }
});
await check('Throwing context validation is handled once for both fulfilled and rejected readbacks',async()=>{
  for(const rejected of [false,true]){
    const errors=[];let predicate=0;const queue=createCameraFeedQueue({render:()=>rejected?Promise.reject(new Error('read reject')):new Uint8Array(4),present:()=>assert.fail('Invalid context presented'),
      isCurrent:()=>{predicate++;throw new Error('context unavailable');},onError:error=>errors.push(error.message)});
    queue.request({});await bounded(queue.whenIdle());assert.equal(predicate,1);assert.deepEqual(errors,['context unavailable']);assert.equal(queue.pending,false);
  }
});
await check('Error-reporting exceptions, including a failed logger, cannot hang the queue',async()=>{
  const log=console.error;let reports=0;console.error=()=>{throw new Error('logger failed');};
  try{const queue=createCameraFeedQueue({render:()=>Promise.reject(new Error('read error')),present:()=>assert.fail(),onError:()=>{reports++;throw new Error('reporter failed');}});
    queue.request({});await bounded(queue.whenIdle());assert.equal(reports,1);assert.equal(queue.pending,false);
  }finally{console.error=log;}
});
await check('Reentrant requests cannot overlap and whenIdle is already valid inside render/present',async()=>{
  const read=deferred();let fromRender,fromPresent,queue;
  queue=createCameraFeedQueue({render:()=>{assert.equal(queue.pending,true);assert.equal(queue.request({nested:true}),false);fromRender=queue.whenIdle();return read.promise;},
    present:()=>{assert.equal(queue.pending,true);assert.equal(queue.request({nested:true}),false);fromPresent=queue.whenIdle();}});
  queue.request({});const idle=queue.whenIdle();assert.equal(fromRender,idle);let done=false;fromRender.then(()=>done=true);await flush();assert.equal(done,false);
  read.resolve(new Uint8Array(4));await bounded(idle);assert.equal(fromPresent,idle);assert.equal(done,true);assert.equal(queue.pending,false);
});
await check('Synchronous compatibility readback is presented asynchronously without copying its supplied bytes',async()=>{
  const pixels=new Uint8Array([9,8,7,6]);let presents=0;
  const queue=createCameraFeedQueue({render:()=>pixels,present:buffer=>{assert.equal(buffer,pixels);assert.deepEqual([...buffer],[9,8,7,6]);presents++;}});
  assert.equal(queue.request({}),true);assert.equal(queue.pending,true);assert.equal(presents,0);await bounded(queue.whenIdle());assert.equal(presents,1);assert.equal(queue.pending,false);
});
await check('A thenable with a throwing then accessor is consumed as a failed readback',async()=>{
  const errors=[];const queue=createCameraFeedQueue({render:()=>({get then(){throw new Error('broken thenable');}}),present:()=>assert.fail(),onError:error=>errors.push(error.message)});
  queue.request({});await bounded(queue.whenIdle());assert.deepEqual(errors,['broken thenable']);assert.equal(queue.pending,false);
});
await flush();process.off('unhandledRejection',observe);
results.push({name:'All controlled failure paths leave zero unhandled Promise rejections',pass:unhandled.length===0,errors:unhandled.map(error=>String(error))});
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
const report={pass:sourceFrozen&&results.every(result=>result.pass),startedAt,finishedAt:new Date().toISOString(),wallSeconds:(performance.now()-started)/1000,
  checks:results.length,results,sourceFrozen,sourceHashes,endSourceHashes,gpuExecuted:false,browserExecuted:false,physicsExecuted:false,
  scope:'Production queue with controlled read/present Promises, exact buffer identity, single in-flight ownership, stale generation/context rejection, error recovery and reentrant idle lifetime. No UI, pixel color, GPU latency, browser FPS or 72-pipe physical result is claimed.',
  review:'App integration snapshots station/controller/arm/theme; invalidation protects reset/start/stop/task/focus/theme/hide changes. camera-feeds renderer-state/color contracts are separately covered by test-camera-feed-async-perf-v2.mjs and test-multi-camera-feeds.mjs.'};
const destination=new URL(reportPath,root);await mkdir(new URL('.',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));if(!report.pass)process.exitCode=1;
