import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {gunzipSync} from 'node:zlib';
import {performance} from 'node:perf_hooks';
import {Worker} from 'node:worker_threads';
import {PLANNING_PROTOCOL_VERSION,createPlanningMessageEncoder,createPlanningWorkerEndpoint} from '../dist/multi/planning-worker-protocol.mjs';
import {createPlanningWorkerTransport} from '../dist/multi/planning-worker-transport.mjs';

const root=new URL('../',import.meta.url),args=process.argv.slice(2);
const output=args.find(value=>value.startsWith('--report='))?.slice(9)??'work/validation/performance-v4/worker-protocol-final.json';
const paths=['work/test-planning-worker-protocol.mjs','work/planning-protocol-echo-worker.mjs','work/background-planning-node-worker.mjs',
  'dist/multi/planning-worker-protocol.mjs','dist/multi/planning-worker-transport.mjs','dist/multi/planning-worker.mjs','dist/multi/planning-worker-core.mjs',
  'dist/multi/planning-serialization.mjs','dist/multi/planning-robot.mjs','dist/multi/planning-check-cache.mjs','dist/multi/planning-ik-context.mjs','dist/multi/planning-budget.mjs',
  'dist/g1/models/g1-dynamic-scene.xml','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/g1-profile.mjs',
  'work/validation/performance-v2/g1-pipe592-snapshots.json.gz'];
const sha=value=>createHash('sha256').update(value).digest('hex');
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,sha(await readFile(new URL(path,root)))])));
const sourceHashes=await hashes(),startedAt=new Date().toISOString(),started=performance.now(),checks=[],measurements=[],unhandled=[];
const observe=error=>unhandled.push(String(error));process.on('unhandledRejection',observe);
async function check(name,run){try{const details=await run();checks.push({name,pass:true,...(details?{details}:{})});}catch(error){checks.push({name,pass:false,error:{message:error.message,stack:error.stack}});}}
const modelFields=['robotId','sourceXml','modelXml','modelPipeSeed','taskMode'];
const dynamicOf=job=>{const dynamic={...job};for(const key of modelFields)delete dynamic[key];return dynamic;};
function job(overrides={}){return{version:2,robotId:'r1',sourceXml:'<mujoco original="1"/>',modelXml:'<mujoco compiled="1"/>',modelPipeSeed:null,taskMode:'legacy',pipeSeed:null,
  request:{arm:'right',id:'blue',continuing:false},physics:{time:.001,qpos:[-0,.125],qvel:[.01,0]},derived:{xpos:[0,1,2],xquat:[1,0,0,0]},controller:{pose:{joint:0}},targets:[],dynamics:{},...overrides};}
function endpointFixture(run=async value=>({ok:true,payload:value})){
  const messages=[],calls=[];let disposed=0;
  const endpoint=createPlanningWorkerEndpoint({service:{run:async(value,options)=>{calls.push(value);return run(value,options);},dispose(){disposed++;}},postMessage:value=>messages.push(structuredClone(value))});
  return{endpoint,messages,calls,get disposed(){return disposed;}};
}
class FakeWorker{
  constructor(){this.messages=[];this.terminated=0;this.throwPost=false;}
  postMessage(value){if(this.throwPost)throw new Error('clone failed');this.messages.push(structuredClone(value));}
  terminate(){this.terminated++;}
  emit(value){this.onmessage?.({data:structuredClone(value)});}
  reply(result={ok:true,payload:{valid:true}}){const request=this.messages.at(-1);this.emit({type:'result',protocol:PLANNING_PROTOCOL_VERSION,jobId:request.jobId,generation:request.generation,modelRevision:request.modelRevision,result});}
}
function transportFixture(options={}){
  const workers=[];let clock=0;
  const transport=createPlanningWorkerTransport({workerFactory:()=>{const worker=new FakeWorker();workers.push(worker);return worker;},now:()=>clock,...options});
  return{transport,workers,setClock:value=>{clock=value;}};
}
const flush=()=>new Promise(resolve=>setImmediate(resolve));

await check('First plan registers one exact model and later same-model plans omit both XML strings',async()=>{
  const encoder=createPlanningMessageEncoder(),fixture=endpointFixture(),first=job(),second=job({physics:{time:.002,qpos:[-.1,-0],qvel:[.02,0]}});
  const initial=encoder.encode(first,{jobId:1,generation:0}),next=encoder.encode(second,{jobId:2,generation:0});
  assert.equal(initial.modelRevision,1);assert(initial.model);assert(!Object.hasOwn(next,'model'));assert.equal(next.modelRevision,1);
  for(const message of [initial,next])for(const key of modelFields)assert(!Object.hasOwn(message.job,key));
  await fixture.endpoint.handle(structuredClone(initial));await fixture.endpoint.handle(structuredClone(next));
  assert.deepEqual(fixture.calls,[first,second]);assert(Object.is(fixture.calls[1].physics.qpos[1],-0));assert.equal(fixture.messages.at(-1).result.ok,true);
});
for(const [field,value]of [['robotId','c5'],['sourceXml','<mujoco original="changed"/>'],['modelXml','<mujoco compiled="changed"/>'],['modelPipeSeed',521],['taskMode','pipes']])
  await check(`Changing model field ${field} registers an ordered fresh revision`,async()=>{
    const encoder=createPlanningMessageEncoder(),fixture=endpointFixture(),initial=job({taskMode:'pipes',modelPipeSeed:1,pipeSeed:1});
    let altered={...initial,[field]:value};if(field==='taskMode')altered={...job(),taskMode:value,modelPipeSeed:521,pipeSeed:521};
    const baseline=field==='taskMode'?job():initial;
    await fixture.endpoint.handle(encoder.encode(baseline,{jobId:1,generation:7}));const message=encoder.encode(altered,{jobId:2,generation:7});
    assert.equal(message.modelRevision,2);assert(message.model);await fixture.endpoint.handle(message);assert.deepEqual(fixture.calls[1],altered);
  });
await check('Current randomized seed remains dynamic without changing the original compiled-model registration',async()=>{
  const encoder=createPlanningMessageEncoder(),fixture=endpointFixture(),first=job({taskMode:'pipes',modelPipeSeed:521,pipeSeed:558}),second={...first,pipeSeed:592};
  await fixture.endpoint.handle(encoder.encode(first,{jobId:1,generation:0}));const message=encoder.encode(second,{jobId:2,generation:0});
  assert(!Object.hasOwn(message,'model'));assert.equal(message.job.pipeSeed,592);await fixture.endpoint.handle(message);assert.deepEqual(fixture.calls[1],second);
});
await check('One model is retained and returning to an earlier model requires registration again',async()=>{
  const encoder=createPlanningMessageEncoder(),fixture=endpointFixture(),original=job(),changed=job({modelXml:'<mujoco other="1"/>'});
  for(const[index,input]of[original,changed,original].entries()){const message=encoder.encode(input,{jobId:index+1,generation:0});assert(message.model);assert.equal(message.modelRevision,index+1);await fixture.endpoint.handle(message);}
  assert.deepEqual(fixture.calls,[original,changed,original]);
});
await check('Unregistered, duplicate, stale, future and wrong-generation requests cannot run the service',async()=>{
  const encoder=createPlanningMessageEncoder(),initial=encoder.encode(job(),{jobId:1,generation:0}),fixture=endpointFixture();
  await fixture.endpoint.handle({...initial,model:undefined});assert.equal(fixture.calls.length,0);assert.equal(fixture.messages.at(-1).result.error.type,'protocol');
  const omitted={...initial};delete omitted.model;await fixture.endpoint.handle(omitted);assert.equal(fixture.calls.length,0);
  await fixture.endpoint.handle(initial);assert.equal(fixture.calls.length,1);
  for(const invalid of [initial,{...omitted,jobId:2,modelRevision:2},{...omitted,jobId:2,modelRevision:0},{...omitted,jobId:2,generation:1},
    {...initial,jobId:2,modelRevision:3},{...omitted,jobId:0},{...omitted,jobId:2,protocol:2}]){
    await fixture.endpoint.handle(invalid);assert.equal(fixture.calls.length,1);assert.equal(fixture.messages.at(-1).result.ok,false);
  }
  await fixture.endpoint.handle({...omitted,jobId:2});assert.equal(fixture.calls.length,2);
});
await check('Malformed registration, dynamic model injection, illegal seeds and empty jobs fail closed',async()=>{
  const encoder=createPlanningMessageEncoder();for(const input of [{},job({version:1}),job({robotId:'unknown'}),job({sourceXml:''}),job({modelXml:null}),job({modelPipeSeed:1}),job({taskMode:'pipes',modelPipeSeed:-1,pipeSeed:1}),job({pipeSeed:1})])assert.throws(()=>encoder.encode(input,{jobId:1,generation:0}));
  for(const mutate of [message=>message.job.sourceXml='override',message=>message.job.robotId='g1',message=>message.model.extra=true,
    message=>message.model.modelPipeSeed=NaN,message=>message.job.version=1,message=>message.job.pipeSeed=NaN,
    message=>message.job=JSON.parse('{"version":2,"pipeSeed":null,"__proto__":{"polluted":true}}')]){
    const local=endpointFixture(),message=createPlanningMessageEncoder().encode(job(),{jobId:1,generation:0});mutate(message);await local.endpoint.handle(message);
    assert.equal(local.calls.length,0);assert.equal(local.messages.at(-1).result.error.type,'protocol');assert.equal({}.polluted,undefined);
  }
});
await check('A malformed new model never replaces the previously registered exact model',async()=>{
  const encoder=createPlanningMessageEncoder(),fixture=endpointFixture(),original=job(),initial=encoder.encode(original,{jobId:1,generation:0});await fixture.endpoint.handle(initial);
  const invalid=encoder.encode(job({modelXml:'<new/>'}),{jobId:2,generation:0});invalid.job.sourceXml='illegal';await fixture.endpoint.handle(invalid);assert.equal(fixture.calls.length,1);
  const restored=createPlanningMessageEncoder().encode(original,{jobId:3,generation:0});delete restored.model;await fixture.endpoint.handle(restored);assert.deepEqual(fixture.calls[1],original);
});
await check('An overlapping message cannot run or replace an active model',async()=>{
  let resolve;const pending=new Promise(yes=>{resolve=yes;}),fixture=endpointFixture(async()=>pending),encoder=createPlanningMessageEncoder();
  const first=fixture.endpoint.handle(encoder.encode(job(),{jobId:1,generation:0}));await flush();
  await fixture.endpoint.handle(encoder.encode(job({modelXml:'<other/>'}),{jobId:2,generation:0}));assert.equal(fixture.calls.length,1);assert.equal(fixture.messages.at(-1).result.error.type,'protocol');
  resolve({ok:true,payload:{done:true}});await first;assert.equal(fixture.messages.at(-1).jobId,1);
});
await check('Endpoint errors and closed reply channels settle without unhandled promises',async()=>{
  const message=createPlanningMessageEncoder().encode(job(),{jobId:1,generation:0});
  const fixture=endpointFixture(async()=>{throw new Error('service failed');});await fixture.endpoint.handle(message);assert.equal(fixture.messages.at(-1).result.ok,false);
  const endpoint=createPlanningWorkerEndpoint({service:{run:async()=>({ok:true})},postMessage(){throw new Error('closed channel');}});await endpoint.handle(message);
  fixture.endpoint.dispose();fixture.endpoint.dispose();assert.equal(fixture.disposed,1);await fixture.endpoint.handle({...message,jobId:2});assert.equal(fixture.calls.length,1);
});
await check('Transport emits one atomic registration and reuses it after a successful item',()=>{
  const f=transportFixture();f.transport.submit(job());assert.equal(f.workers[0].messages.length,1);assert(f.workers[0].messages[0].model);f.workers[0].reply();assert.equal(f.transport.poll().ok,true);
  f.transport.submit(job({physics:{time:2,qpos:[1],qvel:[0]}}));assert.equal(f.workers.length,1);assert.equal(f.workers[0].messages.length,2);assert(!Object.hasOwn(f.workers[0].messages[1],'model'));f.workers[0].reply();assert.equal(f.transport.poll().ok,true);f.transport.dispose();
});
await check('Cancellation and a queued old result cannot bypass fresh-session model registration',()=>{
  const f=transportFixture();f.transport.submit(job());const first=f.workers[0],old=first.messages[0];f.transport.cancel();assert.equal(first.terminated,1);
  f.transport.submit(job());const next=f.workers[1].messages[0];assert(next.model);assert.equal(next.modelRevision,1);assert.equal(next.generation,old.generation+1);
  first.reply();assert.equal(f.transport.poll().pending,true);f.workers[1].reply();assert.equal(f.transport.poll().ok,true);f.transport.dispose();
});
await check('Worker crashes, idle crashes, message errors, failed posts and watchdogs all clear registration',()=>{
  for(const kind of ['crash','idle-crash','message','post','watchdog']){
    const f=transportFixture({timeoutMs:100});f.transport.submit(job());const original=f.workers[0];
    if(kind==='idle-crash'){original.reply();assert.equal(f.transport.poll().ok,true);original.onerror({preventDefault(){}});}
    else if(kind==='crash')original.onerror({preventDefault(){}});
    else if(kind==='message')original.onmessageerror({});
    else if(kind==='watchdog'){f.setClock(101);assert.equal(f.transport.poll().error.type,'timeout');}
    else{original.reply();assert.equal(f.transport.poll().ok,true);original.throwPost=true;f.transport.submit(job());assert.equal(f.transport.poll().ok,false);}
    assert.equal(original.terminated,1);f.transport.submit(job());const replacement=f.workers[1];assert(replacement.messages[0].model);assert.equal(replacement.messages[0].modelRevision,1);replacement.reply();assert.equal(f.transport.poll().ok,true);f.transport.dispose();
  }
});
await check('Wrong job/generation is ignored while a mismatched active revision or version fails closed',()=>{
  for(const change of [{modelRevision:2},{protocol:2},{protocol:undefined}]){
    const f=transportFixture();f.transport.submit(job());const worker=f.workers[0],request=worker.messages[0],valid={type:'result',protocol:3,jobId:request.jobId,generation:request.generation,modelRevision:request.modelRevision,result:{ok:true,payload:{}}};
    for(const wrong of [{jobId:request.jobId+1},{generation:request.generation+1}]){worker.emit({...valid,...wrong});assert.equal(f.transport.poll().pending,true);}
    worker.emit({...valid,...change});assert.equal(f.transport.poll().error.type,'protocol');assert.equal(worker.terminated,1);f.transport.dispose();
  }
});
await check('A late result or progress fails the deadline even when no poll ran; its exact boundary remains valid',()=>{
  for(const type of ['result','progress']){
    const f=transportFixture({timeoutMs:100});f.transport.submit(job());const worker=f.workers[0],request=worker.messages[0];f.setClock(101);
    const response={type,protocol:3,jobId:request.jobId,generation:request.generation,modelRevision:request.modelRevision,result:{ok:true,payload:{}},progress:{stage:'planning',steps:1}};
    // Filtering a stale identity first must not make it affect a newer request.
    worker.emit({...response,jobId:request.jobId+1});assert.equal(worker.terminated,0);
    worker.emit({...response,generation:request.generation+1});assert.equal(worker.terminated,0);
    worker.emit(response);assert.equal(worker.terminated,1);assert.equal(f.transport.poll().error.type,'timeout');f.transport.dispose();
  }
  const f=transportFixture({timeoutMs:100});f.transport.submit(job());f.setClock(100);f.workers[0].reply();assert.equal(f.transport.poll().ok,true);f.transport.dispose();
});
await check('Malformed active progress/result/type and Worker protocol failures never install a plan',()=>{
  for(const change of [{type:'progress',progress:null},{type:'result',result:null},{type:'result',result:{ok:true,pending:true}},
    {type:'result',result:{ok:true,done:false}},{type:'result',result:{ok:'yes'}},{type:'unexpected'},
    {type:'result',result:{ok:false,reason:'model rejected',error:{type:'protocol'}}}]){
    const f=transportFixture();f.transport.submit(job());const worker=f.workers[0],request=worker.messages[0];worker.emit({type:'result',protocol:3,jobId:request.jobId,generation:request.generation,modelRevision:request.modelRevision,...change});
    assert.equal(f.transport.poll().ok,false);assert.equal(worker.terminated,1);f.transport.dispose();
  }
});
await check('The service keeps exact layout/model identity without serializing large XML cache keys',async()=>{
  const source=await readFile(new URL('dist/multi/planning-worker-core.mjs',root),'utf8');
  assert(!/JSON\.stringify\s*\(/.test(source));assert(source.includes('job.sourceXml,job.modelXml,job.modelPipeSeed,job.taskMode]'));
  assert(!source.includes('job.taskMode,job.pipeSeed]'),'A new dynamic layout must reuse the original compiled-model registration');
  assert(source.includes('Object.is(value,key[index])'));assert(source.includes('api.MjModel.from_xml_string(job.modelXml,vfs)'));assert(!source.includes("runtime.configureTask('pipes'"));assert(source.includes('runtime.reset({seed:job.pipeSeed})'));
});

await check('Four genuine source-bound G1 snapshots survive the actual Worker thread registration protocol with tiny later jobs',async()=>{
  const compressed=await readFile(new URL('work/validation/performance-v2/g1-pipe592-snapshots.json.gz',root));
  const captured=JSON.parse(gunzipSync(compressed).toString('utf8')),rows=captured.rows;assert.equal(rows.length,4);
  const currentXml=await readFile(new URL('dist/g1/models/g1-dynamic-scene.xml',root),'utf8'),encoder=createPlanningMessageEncoder(),wire=rows.map((row,index)=>encoder.encode(row.job,{jobId:index+1,generation:0}));
  for(const row of rows){assert.equal(row.job.sourceXml,currentXml);assert.equal(row.job.modelPipeSeed,521);assert.equal(row.job.pipeSeed,592);assert.equal(row.job.robotId,'g1');}
  assert.equal(sha(currentXml),captured.sourceHashes['dist/g1/models/g1-dynamic-scene.xml']);
  const sizes=wire.map((message,index)=>({item:index+1,registration:Object.hasOwn(message,'model'),legacyBytes:Buffer.byteLength(JSON.stringify({type:'plan',jobId:index+1,generation:0,job:rows[index].job})),wireBytes:Buffer.byteLength(JSON.stringify(message))}));
  for(const row of sizes.slice(1)){assert(!row.registration);assert(row.wireBytes<row.legacyBytes*.01);}
  const cloneMeasurements=[];for(const index of [0,1]){
    const legacy={type:'plan',jobId:index+1,generation:0,job:rows[index].job},current=wire[index],legacyMs=[],currentMs=[];
    for(let run=0;run<5;run++)for(const [value,times]of run%2?[[current,currentMs],[legacy,legacyMs]]:[[legacy,legacyMs],[current,currentMs]]){const start=performance.now();const result=structuredClone(value);times.push(performance.now()-start);assert.equal(result.job.version,2);}
    cloneMeasurements.push({item:index+1,legacyMs,currentMs});
  }
  const worker=new Worker(new URL('./planning-protocol-echo-worker.mjs',import.meta.url),{type:'module'});let timer;
  try{
    const replies=[],pending=new Map();worker.on('message',message=>{replies.push(message);if(message.type==='result')pending.get(message.jobId)?.(message);});
    for(const [index,message]of wire.entries()){
      const completed=new Promise((resolve,reject)=>{pending.set(message.jobId,resolve);timer=setTimeout(()=>reject(new Error('Protocol thread exceeded 10s deadline')),10000);worker.once('error',reject);});
      worker.postMessage(message);const response=await completed;clearTimeout(timer);pending.delete(message.jobId);worker.removeAllListeners('error');
      assert.equal(response.protocol,3);assert.equal(response.modelRevision,1);assert.equal(response.result.ok,true);
      assert.deepEqual(response.result.payload,{calls:index+1,robotId:'g1',sourceXmlHash:sha(rows[index].job.sourceXml),modelXmlHash:sha(rows[index].job.modelXml),
        modelPipeSeed:521,taskMode:'pipes',pipeSeed:592,dynamicHash:sha(JSON.stringify(dynamicOf(rows[index].job)))});
    }
    assert.equal(replies.filter(reply=>reply.type==='progress').length,4);
    const legacyBytes=sizes.reduce((total,row)=>total+row.legacyBytes,0),wireBytes=sizes.reduce((total,row)=>total+row.wireBytes,0);
    measurements.push({scenario:'four-genuine-g1-message-roundtrips',fixture:'performance-v2/g1-pipe592-snapshots.json.gz',fixtureScope:captured.scope,
      originalModelHash:sha(currentXml),jobCount:4,sizes,legacyTotalBytes:legacyBytes,currentTotalBytes:wireBytes,reductionPercent:(legacyBytes-wireBytes)/legacyBytes*100,cloneMeasurements,
      scope:'Actual node:worker_threads transport with production endpoint; exact source/model hashes and all dynamic snapshot fields verified. Echo service only, no IK/model compilation/physical execution. Clone timings are Node observations, not browser FPS or guaranteed gains.'});
    return{jobs:4,laterMessagesUnderOnePercent:true,legacyTotalBytes:legacyBytes,currentTotalBytes:wireBytes};
  }finally{clearTimeout(timer);await worker.terminate();}
});
await flush();process.removeListener('unhandledRejection',observe);await check('Protocol errors leave no unhandled promise rejections',()=>assert.deepEqual(unhandled,[]));
const finalHashes=await hashes(),sourceFrozen=JSON.stringify(finalHashes)===JSON.stringify(sourceHashes),pass=checks.every(item=>item.pass)&&sourceFrozen;
const report={pass,releaseCohort:'20261018-perf-4',protocolVersion:PLANNING_PROTOCOL_VERSION,startedAt,wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,checks,measurements,unhandled,
  scope:'Versioned Worker model registration and full snapshot equivalence. No planning budget, IK candidate order, precision, collision sample, robot model or physical state gate is changed or bypassed by these tests.'};
await mkdir(new URL('.',new URL(output,root)),{recursive:true});await writeFile(new URL(output,root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass,sourceFrozen,checks:checks.length,failed:checks.filter(item=>!item.pass),report:output}));if(!pass)process.exitCode=1;
