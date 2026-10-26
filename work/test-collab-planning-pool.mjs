import test from 'node:test';
import assert from 'node:assert/strict';
import {createPipelinePlannerPool} from '../dist/collab/planning-pool.mjs';
const turn=()=>new Promise(resolve=>setImmediate(resolve));
function setup(){const clients=[];const pool=createPipelinePlannerPool({}, {clientFactory:()=>{const client={calls:[],plan(job){this.calls.push(job);return new Promise((resolve,reject)=>Object.assign(this,{resolve,reject}));},cancel(error){this.reject?.(error);},dispose(){this.cancel(new Error('disposed'));}};clients.push(client);return client;}});return {pool,clients};}
test('two plans run independently, third waits bounded FIFO; duplicate rejected',async()=>{
  const {pool,clients}=setup(),g=pool.plan({robotId:'g1'}),r=pool.plan({robotId:'r1'}),c=pool.plan({robotId:'c5'});await turn();
  assert.deepEqual(pool.status,{running:['g1','r1'],queued:['c5'],disposed:false});assert.equal(clients.length,2);
  await assert.rejects(pool.plan({robotId:'g1'}),/重复/);
  clients[1].resolve({ok:true});assert.deepEqual(await r,{ok:true});await turn();assert.equal(clients[1].calls[1].robotId,'c5');
  clients[1].resolve({ok:true});await c;clients[0].resolve({ok:true});await g;await turn();assert.deepEqual(pool.status.running,[]);pool.dispose();
});
test('cancelling G1 terminates only its worker job; R1 remains pending and C5 starts',async()=>{
  const {pool,clients}=setup(),g=pool.plan({robotId:'g1'}),r=pool.plan({robotId:'r1'}),c=pool.plan({robotId:'c5'});await turn();
  const cancelled=assert.rejects(g,/取消/);pool.cancel('g1');await cancelled;await turn();
  assert.equal(clients[0].calls[1].robotId,'c5');assert.ok(pool.status.running.includes('r1'));
  clients[0].resolve({ok:true});clients[1].resolve({ok:true});await Promise.all([r,c]);pool.dispose();
});
test('queued cancellation never launches a worker and disposal rejects all outstanding work',async()=>{
  const {pool,clients}=setup(),g=pool.plan({robotId:'g1'}),r=pool.plan({robotId:'r1'}),c=pool.plan({robotId:'c5'});await turn();
  const cancelled=assert.rejects(c,/取消/);pool.cancel('c5');await cancelled;assert.equal(pool.status.queued.length,0);
  const stopped=[assert.rejects(g),assert.rejects(r)];pool.dispose();await Promise.all(stopped);await turn();
  assert.equal(clients.flatMap(c=>c.calls).length,2);await assert.rejects(pool.plan({robotId:'g1'}),/关闭/);
});
test('same-tick cancellation never starts computation; initialization errors free the slot',async()=>{
  const {pool,clients}=setup(),pending=pool.plan({robotId:'g1'}),rejected=assert.rejects(pending);pool.cancel('g1');await rejected;await turn();assert.equal(clients.length,0);pool.dispose();
  const broken=createPipelinePlannerPool({}, {clientFactory:()=>{throw new Error('worker unavailable');}});
  await assert.rejects(broken.plan({robotId:'c5'}),/unavailable/);await turn();assert.deepEqual(broken.status.running,[]);broken.dispose();
});
