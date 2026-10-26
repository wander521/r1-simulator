import test from 'node:test';
import assert from 'node:assert/strict';
import {BinCollaborationCore,SIMULATION_RATE} from '../dist/collab/bin-core.mjs';
const turn=()=>new Promise(resolve=>setImmediate(resolve));
test('2.4x advances 20 percent faster than 2x, not the real admission clock',()=>{
  const core=Object.create(BinCollaborationCore.prototype),scheduled=[];let steps=0;
  Object.assign(core,{lastWall:null,accumulator:0,stats:{droppedWallSeconds:0},schedule:t=>scheduled.push(t),stepPhysics:()=>steps++});
  core.advance(1000,{budgetMs:100});core.advance(1050,{budgetMs:100});
  assert.equal(SIMULATION_RATE,2*1.2);assert.ok(steps>=119&&steps<=120);
  assert.deepEqual(scheduled,[1,1,1.05,1.05]);assert.equal(core.stats.droppedWallSeconds,0);
});
test('2.4x catch-up remains bounded after an inactive browser tab',()=>{
  const core=Object.create(BinCollaborationCore.prototype);let steps=0;
  Object.assign(core,{lastWall:0,accumulator:0,stats:{droppedWallSeconds:0},schedule:()=>{},stepPhysics:()=>steps++});
  core.advance(60000,{maxSteps:12,budgetMs:100});
  assert.equal(steps,12);assert.ok(core.accumulator<=.3);assert.ok(core.stats.droppedWallSeconds>=59.9);
});
test('R1 home uses an empty checked aisle route before its home posture',()=>{
  const core=Object.create(BinCollaborationCore.prototype),calls=[];
  Object.assign(core,{agents:new Map([['r1',{controller:{}}]]),metadata:{robots:{r1:{origin:[.38,.4,.208],yaw:-Math.PI/2}},mobility:{aisleY:.85}},data:{xpos:[1.12,.5,.208]},mountBody:0,
    _navigate:(a,dock,carry,next)=>calls.push({dock,carry,next})});
  const action={job:{robot:'r1',type:'home'},phase:'home-outbound'};
  for(const phase of ['home-outbound','home-travel','home-dock']){action.phase=phase;core._progress(action);}
  assert.deepEqual(calls.map(c=>c.dock.position),[[1.12,.85,.208],[.38,.85,.208],[.38,.4,.208]]);
  assert.deepEqual(calls.map(c=>c.next),['home-travel','home-dock','need-posture']);
  assert.ok(calls.every(c=>!c.carry));assert.equal(core._target(action),undefined);assert.equal(core._support(action),undefined);
});
test('stale plans back off in simulated time while preserving the retry bound',async()=>{
  const core=Object.create(BinCollaborationCore.prototype),action={job:{id:'a',robot:'g1'}},controller={};
  let calls=0,accepted=0;
  Object.assign(core,{generation:0,disposed:false,data:{time:4,qpos:[],qvel:[],ctrl:[]},agents:new Map([['g1',{controller}]]),actions:new Map([['g1',action]]),stats:{planningJobs:0,stalePlans:0},pipeline:{job:()=>({status:'executing'})},_context:()=>({}),_guard:()=>false,_fail:(a)=>{a.failed=true;},plan:async()=>{calls++;return {ok:true};}});
  const submit=()=>core._submit(action,'manipulation',{},()=>accepted++);
  submit();await turn();assert.equal(calls,1);assert.equal(action.nextPlanTime,4.25);
  submit();await turn();assert.equal(calls,1,'A busy loop immediately retried stale physics');
  for(let attempt=2;attempt<=4;attempt++){core.data.time=action.nextPlanTime;submit();await turn();assert.equal(calls,attempt);}
  assert.equal(action.failed,true);assert.equal(accepted,0);assert.equal(core.stats.stalePlans,4);
});
test('a cancelled generation cannot apply a delayed plan or increment retries',async()=>{
  const core=Object.create(BinCollaborationCore.prototype),action={job:{id:'a',robot:'g1'}},controller={};let resolve,accepted=0;
  Object.assign(core,{generation:0,disposed:false,data:{time:4,qpos:[],qvel:[],ctrl:[]},agents:new Map([['g1',{controller}]]),actions:new Map([['g1',action]]),stats:{planningJobs:0,stalePlans:0},pipeline:{job:()=>({status:'executing'})},_context:()=>({}),_guard:()=>true,plan:()=>new Promise(r=>resolve=r)});
  core._submit(action,'manipulation',{},()=>accepted++);await turn();core.generation++;
  resolve({ok:true});await turn();assert.equal(accepted,0);assert.equal(core.stats.stalePlans,0);
});
test('aisle travel never reserves a distant station; entry waits for its local lease',()=>{
  const core=Object.create(BinCollaborationCore.prototype),calls=[];
  Object.assign(core,{agents:new Map([['r1',{controller:{}}]]),metadata:{mobility:{docks:{src:{position:[1,0,0]},dst:{position:[2,0,0]}}}},data:{xpos:[.5,0,0]},mountBody:0,
    _aisleDock:x=>({position:[x,1,0]}),_navigate:(_a,dock,carry,next)=>calls.push({dock,carry,next}),_slot:id=>({kind:id==='src'?'full':'input'}),_acquire:()=>{throw new Error('Premature lease');}});
  const action={job:{id:'a',robot:'r1',type:'deliver',source:'src',destination:'dst'},phase:'await-source'};
  for(const phase of ['await-source','need-source-travel','await-destination']){action.phase=phase;core._progress(action);}
  assert.deepEqual(calls.map(c=>c.next),['need-source-travel','await-source-zone','await-destination-zone']);
  core._acquire=()=>false;
  for(const phase of ['await-source-zone','await-destination-zone']){action.phase=phase;core._progress(action);assert.equal(action.phase,phase);}
  assert.equal(calls.length,3);
  const zones=[];core._acquire=(_a,zone)=>{zones.push(zone);return true;};
  action.phase='await-source-zone';core._progress(action);assert.equal(action.phase,'need-source-dock');
  action.phase='await-destination-zone';core._progress(action);assert.equal(action.phase,'need-destination-dock');
  assert.deepEqual(zones,['g1-out','c5-in']);
});
test('resume remains held until an independent async check; a second stop preserves the phase',async()=>{
  const core=Object.create(BinCollaborationCore.prototype),controller={state:'descend',running:true,arm:'right',planData:{}},action={job:{id:'a',robot:'g1'},phase:'manipulation'};
  let resolve,paused=false;
  const agent={controller,pausePosture:v=>paused=v,measure:()=>({count:0})};
  const pipeline={state:'running',pause(){this.state='paused';},resume(){this.state='running';},job:()=>({status:'executing'})};
  Object.assign(core,{generation:0,disposed:false,data:{time:4,qpos:[],qvel:[],ctrl:[]},agents:new Map([['g1',agent]]),actions:new Map([['g1',action]]),stats:{planningJobs:0,stalePlans:0},pipeline,cancelPlan:()=>{},_context:()=>({}),_guard:()=>true,_target:()=>({}),_support:()=>'',plan:()=>new Promise(r=>resolve=r)});
  core.stop();core.resume();core._progress(action);await turn();
  assert.equal(controller.running,false);assert.equal(paused,true);assert.equal(action.pendingKind,'resume-manipulation');
  core.stop();assert.equal(action.resume.state,'descend');
  resolve({ok:true});await turn();assert.equal(controller.running,false,'Cancelled resume unexpectedly moved');
  core.resume();core._progress(action);await turn();resolve({ok:true});await turn();
  assert.equal(controller.state,'descend');assert.equal(controller.running,true);assert.equal(paused,false);
});
test('a delayed plan cannot revive a failed action',async()=>{
  const core=Object.create(BinCollaborationCore.prototype),action={job:{id:'a',robot:'g1'}},controller={};let resolve,accepted=0;
  Object.assign(core,{generation:0,disposed:false,data:{time:4,qpos:[],qvel:[],ctrl:[]},agents:new Map([['g1',{controller}]]),actions:new Map([['g1',action]]),stats:{planningJobs:0,stalePlans:0},pipeline:{job:()=>({status:'executing'})},_context:()=>({}),_guard:()=>true,plan:()=>new Promise(r=>resolve=r)});
  core._submit(action,'manipulation',{},()=>accepted++);await turn();action.failed=true;
  resolve({ok:true});await turn();assert.equal(accepted,0);
});
