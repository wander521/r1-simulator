import test from 'node:test';
import assert from 'node:assert/strict';
import {BinPipeline} from '../dist/collab/pipeline.mjs';

const make=(count=6,options={})=>new BinPipeline({bins:Array.from({length:count},(_,i)=>({id:`bin-${i}`,parts:[`pin-${i}`,`sleeve-${i}`]})),...options});
function finish(p,job){p.executing(job.id);p.complete(job.id,{ready:true,objectId:job.partId??job.binId,destination:job.destination,dualContact:true,lifted:true,supported:true});}
function ready(p){p.start();for(const job of p.tick(0))finish(p,job);}

test('initial home is separate from ready and motion cannot skip ready evidence',()=>{
  const p=make();p.start();const jobs=p.tick(0);assert.deepEqual(jobs.map(j=>j.type),['ready','ready','ready']);
  p.executing(jobs[0].id);assert.throws(()=>p.complete(jobs[0].id,{}),/准备位/);assert.equal(p.robots.g1.state,'ready');
});
test('independent robots overlap across different batches',()=>{
  const p=make();ready(p);
  const first=p.tick(1).find(j=>j.type==='fetch');finish(p,first);
  let jobs=p.tick(2);assert.deepEqual(jobs.map(j=>j.type),['fetch','deliver']);jobs.forEach(j=>finish(p,j));
  jobs=p.tick(3);assert.deepEqual(jobs.map(j=>j.type),['fetch','deliver','pack']);assert.equal(new Set(jobs.map(j=>j.binId)).size,3);
  // A planner can remain pending indefinitely without blocking another robot.
  p.planning(jobs[0].id);finish(p,jobs[1]);finish(p,jobs[2]);
  assert.ok(p.tick(4).some(j=>j.robot==='c5'));assert.equal(p.job(jobs[0].id).status,'planning');
});
test('handoff mutex covers actual local occupancy, not the entire mobile route',()=>{
  const p=make();ready(p);finish(p,p.tick(1)[0]);const jobs=p.tick(2),a=jobs.find(j=>j.robot==='g1'),b=jobs.find(j=>j.robot==='r1');
  assert.ok(p.enterZone(b.id,'g1-out'));assert.equal(p.enterZone(a.id,'g1-out'),false);
  assert.throws(()=>p.leaveZone(b.id,'g1-out',{clear:false}),/退出交接区/);
  p.executing(b.id);p.sourceDeparted(b.id,{objectId:b.binId,clear:true,dualContact:true,lifted:true});
  p.leaveZone(b.id,'g1-out',{clear:true});assert.ok(p.enterZone(a.id,'g1-out'));
  assert.equal(p.job(b.id).status,'executing');
});
test('material identity/contact proof required; no optimistic stock transition',()=>{
  const p=make();ready(p);const job=p.tick(1)[0];p.executing(job.id);
  assert.throws(()=>p.complete(job.id,{ready:true,objectId:'copy',destination:job.destination}),/身份/);
  assert.throws(()=>p.complete(job.id,{ready:true,objectId:job.binId,destination:job.destination,dualContact:false}),/真实/);
  assert.equal(p.bins.get(job.binId).location,'wall:bin-0');assert.ok(p.leases.has(`bin:${job.binId}`));
});
test('finite buffer backpressure, output and return capacity reserved at admission',()=>{
  const p=make(6,{outputSlots:['finished'],returnSlots:['empty']});ready(p);finish(p,p.tick(1)[0]);
  const jobs=p.tick(2);assert.equal(jobs.some(j=>j.robot==='g1'),false);assert.equal(jobs[0].type,'deliver');
  assert.equal([...p.bins.values()].filter(b=>b.admitted).length,1);
});
test('failure retains load and leases; retry explicit, other stations continue',()=>{
  const p=make();ready(p);finish(p,p.tick(1)[0]);const jobs=p.tick(2),r=jobs.find(j=>j.robot==='r1');
  p.fail(r.id,'route unavailable');assert.equal(p.job(r.id).status,'failed');assert.ok(p.leases.has(`bin:${r.binId}`));
  finish(p,jobs.find(j=>j.robot==='g1'));p.tick(3);assert.equal(p.robots.r1.jobId,r.id);
  assert.equal(p.retry(r.id).status,'reserved');finish(p,p.job(r.id));
});
test('pause does not advance active duration; resume retains task IDs',()=>{
  const p=make();ready(p);const jobs=p.tick(1);p.pause();assert.deepEqual(p.tick(101),[]);assert.equal(p.elapsedSeconds,1);
  p.resume();p.tick(102);assert.equal(p.elapsedSeconds,1);assert.equal(p.robots.g1.jobId,jobs[0].id);
});
test('300 second admission cutoff drains every admitted bin, then homes once',()=>{
  const p=make(20);ready(p);let jobs=p.tick(1),t=1,maxConcurrent=0,home=[];
  // Test logistics only; these completions are mock port evidence, NOT physics.
  while(p.state!=='complete'&&t<1000){
    maxConcurrent=Math.max(maxConcurrent,jobs.length);
    jobs.forEach(j=>{if(j.type==='home')home.push(j.robot);finish(p,j);});t+=20;jobs=p.tick(t);
  }
  assert.equal(p.state,'complete');assert.ok(maxConcurrent===3);assert.deepEqual(home.sort(),['c5','g1','r1']);
  const admitted=[...p.bins.values()].filter(b=>b.admitted);assert.ok(admitted.length<20);assert.ok(admitted.every(b=>p.slots.get(b.location).kind==='returned'&&b.packed===2));
  assert.equal([...p.parts.values()].filter(x=>x.packed).length,admitted.length*2);assert.equal(p.leases.size,0);
  assert.ok(p.elapsedSeconds>=300);assert.ok(p.assertInvariants());
});
test('arbitrary slot names and serialized snapshots preserve ownership',()=>{
  const p=make(2,{fullSlots:['shelf-out'],inputSlots:['machine-feed']});ready(p);
  for(let t=1;t<20;t++)for(const job of p.tick(t))finish(p,job);
  assert.ok([...p.bins.values()].every(b=>p.slots.get(b.location)?.kind==='returned'));
  const snapshot=p.snapshot();snapshot.bins[0].packed=99;assert.equal(p.bins.get('bin-0').packed,2);
});
test('a loaded departure frees only its source; completing delivery cannot erase the next bin',()=>{
  const p=make(3,{fullSlots:['full-1']});ready(p);finish(p,p.tick(1)[0]);
  const mobile=p.tick(2).find(j=>j.type==='deliver');p.executing(mobile.id);
  assert.throws(()=>p.sourceDeparted(mobile.id,{objectId:mobile.binId,clear:true,dualContact:false,lifted:true}),/实物证据/);
  assert.equal(p.slots.get('full-1').binId,mobile.binId);
  p.sourceDeparted(mobile.id,{objectId:mobile.binId,clear:true,dualContact:true,lifted:true});
  assert.equal(p.bins.get(mobile.binId).location,'transport:r1');assert.equal(p.slots.get('full-1').binId,null);
  assert.ok(p.leases.has(`bin:${mobile.binId}`));assert.ok(p.leases.has(`slot:${mobile.destination}`));
  assert.throws(()=>p.sourceDeparted(mobile.id,{}),/重复/);
  const next=p.tick(3).find(j=>j.type==='fetch');finish(p,next);
  p.complete(mobile.id,{ready:true,objectId:mobile.binId,destination:mobile.destination,dualContact:true,lifted:true,supported:true});
  assert.equal(p.slots.get('full-1').binId,next.binId);assert.ok(p.assertInvariants());
});
test('shared output envelope waits for physical departure, not just a free placement mark',()=>{
  const p=make();ready(p);finish(p,p.tick(1)[0]);const jobs=p.tick(2),fetch=jobs.find(j=>j.type==='fetch'),mobile=jobs.find(j=>j.type==='deliver');
  assert.equal(p.enterZone(fetch.id,'g1-out'),false);
  assert.ok(p.enterZone(mobile.id,'g1-out'));p.executing(mobile.id);
  p.sourceDeparted(mobile.id,{objectId:mobile.binId,clear:true,dualContact:true,lifted:true});
  assert.equal(p.enterZone(fetch.id,'g1-out'),false,'Releasing the slot does not prematurely release the volume');
  p.leaveZone(mobile.id,'g1-out',{clear:true});assert.ok(p.enterZone(fetch.id,'g1-out'));
  p.fail(mobile.id,'deliberate transport failure');assert.ok(p.assertInvariants());assert.ok(p.leases.has(`bin:${mobile.binId}`));
});
