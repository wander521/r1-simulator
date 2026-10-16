import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';

// These are orchestration-contract tests, not physics substitutes. The separate
// test-multi-physical suite uses the three production MuJoCo controllers.
const ids=['r1','g1','c5'],checks=[];
const root=new URL('../',import.meta.url);
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)??null;
const sourcePaths=['work/test-multi-coordinator.mjs','dist/multi/coordinator.mjs'];
const sourceHashes=Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,
  createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));

function station(id,{ready=true,accept=true}={}){
  const calls={start:[],stop:0,reset:0,step:[],sync:0},controller={running:false,state:'idle',message:'',
    evidence:{dual:false,lift:0},data:{time:0}};
  const runtime={id,ready,controller,group:{visible:true},calls,objectPosition:[.5,.75,0],
    start(options){
      calls.start.push({...options,time:controller.data.time});
      // An existing controller can enter its failure reaction before returning
      // false, so rollback must stop the rejected station as well.
      controller.running=true;controller.state=accept?'approach':'shy';
      if(!accept)controller.message='测试目标不可达';
      return accept;
    },
    stop(){calls.stop++;controller.running=false;controller.state='stopped';controller.message='已停止';},
    reset(){calls.reset++;controller.running=false;controller.state='idle';controller.data.time=0;},
    step(dt=.001){calls.step.push(dt);controller.data.time+=dt;},
    syncVisuals(){calls.sync++;},
    snapshot(){return {id,ready:runtime.ready,running:controller.running,state:controller.state,
      message:controller.message,time:controller.data.time,progress:controller.state==='complete'?1:0,
      evidence:{...controller.evidence}};},
  };
  return runtime;
}
function fixture(options={}){
  const stations=new Map(ids.map(id=>[id,station(id,options[id])]));
  const c=new MultiRobotCoordinator({stations,selectedIds:options.selectedIds??ids});
  return {stations,c};
}
function snapshot(c){const value=typeof c.snapshot==='function'?c.snapshot():c.snapshot;assert(Array.isArray(value.stations));return value;}
function chosen(c){return snapshot(c).selectedIds;}
function totalStarts(stations){return [...stations.values()].reduce((n,s)=>n+s.calls.start.length,0);}
function run(name,fn){fn();checks.push({name,pass:true});}
const startOptions={objectId:'blue',task:'sort',armByRobot:{r1:'right',g1:'right',c5:'left'}};

run('initial selection is all three independent robot stations',()=>{
  const {c,stations}=fixture(),s=snapshot(c);
  assert.deepEqual(s.selectedIds,ids);assert.equal(s.focusId,'r1');assert.equal(s.running,false);
  assert.deepEqual(s.stations.map(item=>item.id),ids);
  assert.equal(new Set(s.stations.map(item=>item.id)).size,3);
  assert.equal(totalStarts(stations),0);
});
run('array stations and production-style snapshot getters use the same contract',()=>{
  const stations=ids.map(id=>station(id));
  for(const runtime of stations){const read=runtime.snapshot;Object.defineProperty(runtime,'snapshot',{get:read});}
  const c=new MultiRobotCoordinator({stations,selectedIds:['g1','c5']});
  assert.deepEqual(chosen(c),['g1','c5']);assert.equal(c.start(startOptions).ok,true);
  c.step(.001);assert(snapshot(c).stations.every(s=>Math.abs(s.time-.001)<1e-12));
});
for(const selectedIds of [['g1'],['r1','c5'],ids])run(`selection ${selectedIds.join('+')} changes visibility only`,()=>{
  const {c,stations}=fixture();c.select(selectedIds);
  assert.deepEqual(chosen(c),selectedIds);
  for(const [id,s]of stations){assert.equal(s.group.visible,selectedIds.includes(id));assert.equal(s.calls.reset,0);assert.equal(s.calls.stop,0);}
});
run('empty and unknown selections preserve last valid selection',()=>{
  const {c}=fixture();c.select(['g1']);const prior=chosen(c);
  for(const selection of [[],['unknown'],['g1','unknown']]){
    try{c.select(selection);}catch(error){assert(error instanceof Error);}
    assert.deepEqual(chosen(c),prior);
  }
});
run('not-ready station cannot enter selection or start a partial run',()=>{
  const {c,stations}=fixture({selectedIds:['r1'],g1:{ready:false}});
  try{c.select(['r1','g1']);}catch(error){assert(error instanceof Error);}
  assert.deepEqual(chosen(c),['r1']);assert.equal(totalStarts(stations),0);
  // Loading can fail after a previous selection but before the button click.
  stations.get('r1').ready=false;
  const result=c.start(startOptions);assert.equal(result.ok,false);assert.equal(totalStarts(stations),0);
});
run('focus tracks a selected station without selecting new robots',()=>{
  const {c}=fixture();c.select(['g1','c5']);c.setFocus('c5');
  assert.equal(snapshot(c).focusId,'c5');assert.deepEqual(chosen(c),['g1','c5']);
});
run('three starts occur synchronously at one tick before any physics step',()=>{
  const {c,stations}=fixture();for(let n=0;n<17;n++)c.step(.001);
  const before=snapshot(c),result=c.start(startOptions),after=snapshot(c);
  assert.equal(result.ok,true);assert.deepEqual(result.runIds,ids);
  assert.equal(result.startTick,before.tick);assert.deepEqual(after.runIds,ids);assert.equal(after.running,true);
  assert.equal(after.tick,before.tick);
  const times=[];
  for(const [id,s]of stations){
    assert.equal(s.calls.start.length,1);const call=s.calls.start[0];times.push(call.time);
    assert.equal(call.objectId,'blue');assert.equal(call.task,'sort');assert.equal(call.arm,startOptions.armByRobot[id]);
    assert.equal(s.calls.step.length,17);
  }
  assert(Math.max(...times)-Math.min(...times)<1e-12);
});
for(const selectedIds of [['r1'],['g1','c5'],ids])run(`only selected ${selectedIds.join('+')} receive task starts`,()=>{
  const {c,stations}=fixture();c.select(selectedIds);const result=c.start(startOptions);
  assert.equal(result.ok,true);assert.deepEqual(result.runIds,selectedIds);
  for(const [id,s]of stations)assert.equal(s.calls.start.length,selectedIds.includes(id)?1:0);
});
run('busy selection and repeated start cannot mutate an active run',()=>{
  const {c,stations}=fixture();assert.equal(c.start(startOptions).ok,true);
  try{c.select(['c5']);}catch(error){assert(error instanceof Error);}
  assert.deepEqual(chosen(c),ids);const result=c.start(startOptions);
  assert.equal(result.ok,false);assert.equal(totalStarts(stations),3);assert.deepEqual(snapshot(c).runIds,ids);
});
run('every ready world advances once per shared one-millisecond tick, even if hidden',()=>{
  const {c,stations}=fixture();c.select(['c5']);
  for(let n=0;n<125;n++)c.step(.001);
  const s=snapshot(c);assert.equal(s.tick,125);assert(Math.abs(s.simTime-.125)<1e-12);
  for(const runtime of stations.values()){
    assert.equal(runtime.calls.step.length,125);assert(runtime.calls.step.every(value=>value===.001));
    assert(Math.abs(runtime.controller.data.time-.125)<1e-12);
  }
});
run('stop affects this run only and never resets task objects',()=>{
  const {c,stations}=fixture();c.select(['r1','c5']);assert.equal(c.start(startOptions).ok,true);c.stop();
  assert.equal(snapshot(c).running,false);
  for(const [id,s]of stations){assert.equal(s.calls.stop,['r1','c5'].includes(id)?1:0);assert.equal(s.calls.reset,0);assert.deepEqual(s.objectPosition,[.5,.75,0]);}
});
run('reset stops running tasks then resets all ready stations, including hidden ones',()=>{
  const {c,stations}=fixture();c.select(['r1']);assert.equal(c.start(startOptions).ok,true);c.step(.001);c.reset();
  assert.equal(snapshot(c).running,false);
  for(const s of stations.values()){assert.equal(s.calls.reset,1);assert.equal(s.controller.state,'idle');}
  assert(stations.get('r1').calls.stop>=1);
});
run('rejected partial start cancels every attempted controller without teleporting objects',()=>{
  const {c,stations}=fixture({g1:{accept:false}}),before=[...stations.values()].map(s=>[...s.objectPosition]);
  const result=c.start(startOptions);assert.equal(result.ok,false);
  assert(result.reason.includes('测试目标不可达'),'Rollback must preserve the original planner rejection reason, not replace it with a stop message');
  assert.equal(stations.get('r1').calls.start.length,1);assert.equal(stations.get('g1').calls.start.length,1);
  assert.equal(stations.get('c5').calls.start.length,0);
  assert.equal(stations.get('r1').controller.running,false);assert.equal(stations.get('g1').controller.running,false);
  assert([...stations.values()].every(s=>s.calls.reset===0));
  assert.deepEqual([...stations.values()].map(s=>s.objectPosition),before);assert.equal(snapshot(c).running,false);
});
run('an externally busy selected controller prevents a new partial run',()=>{
  const {c,stations}=fixture();stations.get('g1').controller.running=true;stations.get('g1').controller.state='transfer';
  const result=c.start(startOptions);assert.equal(result.ok,false);assert.equal(totalStarts(stations),0);
});
run('one failed station is never aggregated as an all-robot success',()=>{
  const {c,stations}=fixture();assert.equal(c.start(startOptions).ok,true);
  for(const s of stations.values()){s.controller.running=false;s.controller.state='complete';s.controller.evidence={dual:true,lift:.14};}
  stations.get('g1').controller.state='failed';stations.get('g1').controller.message='夹持失败';c.step(.001);
  const s=snapshot(c);assert.notEqual(s.state,'complete');assert.equal(s.running,false);
  assert(s.stations.some(item=>item.id==='g1'&&item.state==='failed'));
});
run('all completed selected controllers aggregate to complete and can be reselected',()=>{
  const {c,stations}=fixture();c.select(['r1','c5']);assert.equal(c.start(startOptions).ok,true);
  for(const id of ['r1','c5']){const s=stations.get(id);s.controller.running=false;s.controller.state='complete';s.controller.evidence={dual:true,lift:.14};}
  c.step(.001);assert.equal(snapshot(c).state,'complete');assert.equal(snapshot(c).running,false);
  c.select(['g1']);assert.deepEqual(chosen(c),['g1']);
});

const finalHashes=Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,
  createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
assert.deepEqual(finalHashes,sourceHashes,'Orchestrator source changed during contract verification');
const report={pass:true,checks:checks.length,results:checks,sourceHashes,sourceFrozen:true};
if(reportPath){const destination=new URL(reportPath,root);await mkdir(new URL('./',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');}
console.log(JSON.stringify(reportPath?{pass:true,checks:checks.length,sourceFrozen:true,report:reportPath}:report));
