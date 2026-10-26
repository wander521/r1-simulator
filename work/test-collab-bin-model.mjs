import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {buildBinCollaborationModel,CARRIER} from '../dist/collab/bin-model.mjs';
import {createRobotAgent} from '../dist/collab/robot-adapter.mjs';
import {robotFrame,physicalSnapshot,bodyPose,toThree} from '../dist/collab/state.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {BinPipeline} from '../dist/collab/pipeline.mjs';
import {createPlanningWorkerRobot} from '../dist/multi/planning-worker-core.mjs';

const root=new URL('../',import.meta.url),ids=['r1','g1','c5'];
const sources=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${id}/models/${id}-dynamic-scene.xml`,root),'utf8')])));
const paths={r1:'r1/assets/r1.urdf',g1:'g1/assets/g1_29dof_with_dex1_1.urdf',c5:'c5/assets/c5-dex1.urdf'};
const urdfs=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${paths[id]}`,root),'utf8')])));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const {xml,metadata}=buildBinCollaborationModel(sources,{binCount:4}),vfs=new api.MjVFS();
const model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
const name=(type,value)=>api.mj_name2id(model,api.mjtObj[`mjOBJ_${type}`].value,value);
const agents=ids.map(id=>createRobotAgent({api,model,data,id,metadata:robotFrame(metadata.robots[id],metadata.workbench),robot:createPlanningWorkerRobot(urdfs[id],{rootHeight:ROBOT_CONFIGS[id].rootHeight}),targets:[]}));
api.mj_forward(model,data);

test('four visually distinct materials have physical cylinder and block geometry',()=>{
  assert.deepEqual([...new Set(metadata.parts.map(p=>p.shape))].sort(),['box','cylinder']);
  assert.equal(new Set(metadata.parts.map(p=>p.color)).size,4);
  for(const shape of ['box','cylinder'])assert.equal(new Set(metadata.parts.filter(p=>p.shape===shape).map(p=>p.color)).size,2);
  for(const bin of metadata.bins)assert.deepEqual(metadata.parts.filter(p=>p.binId===bin.id).map(p=>p.shape),['cylinder','box']);
});
test('four category regions route both bins of each material to distinct supported positions',()=>{
  const slots=metadata.slots.filter(s=>s.kind==='output');
  assert.equal(slots.length,4);assert.equal(new Set(slots.map(s=>s.category)).size,4);
  for(const slot of slots){
    const members=metadata.parts.filter(p=>p.outputSlot===slot.id);
    assert.equal(members.length,2);assert.equal(new Set(members.map(p=>p.binId)).size,2);
    assert.deepEqual(members.map(p=>p.outputIndex),[0,1]);
    for(const part of members){
      assert.equal(part.category,slot.category);assert.equal(part.color,slot.color);assert.equal(part.shape,slot.shape);
      const target=slot.partSlots[part.outputIndex];
      assert.ok(Math.abs(target[0]-slot.position[0])+part.halfX<.09);
      assert.ok(Math.abs(target[1]-slot.position[1])+part.halfZ<.065);
      assert.equal(model.geom_contype[name('GEOM',`fixture_${slot.id}_floor`)],1);
    }
    assert.ok(Math.hypot(...slot.partSlots[0].map((v,i)=>v-slot.partSlots[1][i]))>.06);
  }
  const options={bins:metadata.bins,outputSlots:slots.map(s=>s.id),partDestinations:Object.fromEntries(metadata.parts.map(p=>[p.id,{destination:p.outputSlot,partIndex:p.outputIndex}]))};
  const p=new BinPipeline(options);p.start();
  for(let t=0;t<350&&p.state!=='complete';t++)for(const job of p.tick(t)){
    if(job.type==='pack'){
      const part=metadata.parts.find(x=>x.id===job.partId);
      assert.equal(job.destination,part.outputSlot);assert.equal(job.partIndex,part.outputIndex);
    }
    p.executing(job.id);p.complete(job.id,{ready:true,objectId:job.partId??job.binId,destination:job.destination,dualContact:true,lifted:true,supported:true});
  }
  assert.equal(p.state,'complete');
  for(const part of metadata.parts)assert.equal(p.parts.get(part.id).location,part.outputSlot);
  const duplicate=structuredClone(options);duplicate.partDestinations[metadata.parts[1].id]=duplicate.partDestinations[metadata.parts[0].id];
  assert.throws(()=>new BinPipeline(duplicate),/落点重复/);
});
test('rack backing stops at 1.04 metres and has one shared visual/physical definition',()=>{
  const back=metadata.rackStructure.find(g=>g.name==='wall_rack_back'),g=name('GEOM',back.name);
  assert.ok(Math.abs(back.position[2]+back.half[2]-1.04)<1e-9);
  assert.equal(model.geom_size[g*3+2],back.half[2]);
  assert.ok(metadata.rack.every(s=>s.tableHeight<=1.04));
});
test('cabinet visuals use the same collidable fixtures, within existing table footprints',()=>{
  assert.equal(metadata.fixtures.length,metadata.supports.length);
  for(const [i,f]of metadata.fixtures.entries()){
    const g=name('GEOM',f.name),s=metadata.supports[i];assert.ok(g>=0);
    assert.equal(model.geom_contype[g],1);
    for(let axis=0;axis<2;axis++)assert.ok(Math.abs(f.position[axis]-s.center[axis])+f.half[axis]<=s.halfExtents[axis]);
    assert.ok(f.position[2]+f.half[2]<=s.tableHeight-.065+1e-9);
  }
});
test('each rack shelf physically joins the backing without changing its pickup edge or height',()=>{
  const back=metadata.rackStructure.find(g=>g.name==='wall_rack_back');
  for(const s of metadata.rack){
    assert.ok(Math.abs(s.center[0]-s.halfExtents[0]-.4)<1e-9);
    assert.ok(s.center[0]+s.halfExtents[0]>=back.position[0]-back.half[0]);
    assert.ok(Math.abs(s.center[1]-back.position[1])+s.halfExtents[1]<=back.half[1]);
    assert.ok(Math.abs(s.center[2]+s.halfExtents[2]-s.tableHeight)<1e-9);
    const g=name('GEOM',s.geom);assert.equal(model.geom_size[g*3],s.halfExtents[0]);
    assert.equal(model.geom_contype[g],1);
  }
});

test('single shared world: four free carriers, eight separate loose parts, no attachment or carrier motors',()=>{
  assert.equal(metadata.bins.length,4);assert.equal(metadata.parts.length,8);assert.equal(model.neq,0);
  assert.equal(model.nu,70);assert(!/mocap=|<weld|<equality|tray.*motor/.test(xml));
  for(const object of [...metadata.bins,...metadata.parts]){const j=name('JOINT',object.joint);assert.equal(model.jnt_type[j],0);assert.equal(model.body_jntnum[name('BODY',object.body)],1);}
  assert.equal(new Set([...metadata.bins,...metadata.parts].map(x=>x.body)).size,12);
  assert.throws(()=>buildBinCollaborationModel(sources,{binCount:6}),/two reachable wall levels/);
  assert.equal(model.opt.impratio,10);assert.equal(metadata.contactSolver.sourceFrictionAndForceLimitsUnchanged,true);
  for(const side of ['right','left'])for(const i of [1,2]){
    const motor=name('ACTUATOR',`r1__${side}_motor${i}`);assert.equal(model.actuator_forcerange[motor*2],-5);assert.equal(model.actuator_forcerange[motor*2+1],5);
  }
});
test('all physical slot footprints are supported and outside the robot travel lane',()=>{
  for(const slot of metadata.slots){
    const support=metadata.supports.find(x=>x.body===slot.supportBody),p=slot.position;
    const bounds=slot.kind==='output'?{minX:-.09,maxX:.09,y:.065}:{minX:CARRIER.floorOffset[0]-CARRIER.floorHalf[0]-.004,maxX:CARRIER.floorOffset[0]+CARRIER.floorHalf[0]+.004,y:.079};
    assert.ok(p[0]+bounds.minX>=support.center[0]-support.halfExtents[0]-1e-9,`${slot.id} left overhang`);
    assert.ok(p[0]+bounds.maxX<=support.center[0]+support.halfExtents[0]+1e-9,`${slot.id} right overhang`);
    assert.ok(Math.abs(p[1]-support.center[1])+bounds.y<=support.halfExtents[1]+1e-9,`${slot.id} lateral overhang`);
    assert.ok(support.center[1]+support.halfExtents[1]<metadata.mobility.platform.center[1]-metadata.mobility.platform.halfExtents[1]);
  }
});
test('G1 entry to ready has a fully collision-checked three-stage path with all four wall bins present',()=>{
  const agent=agents.find(a=>a.id==='g1'),c=agent.controller;let from={...c.pose};
  try{for(const phase of ['spread','raise','ready']){const to=c.phaseTarget(phase);try{c.validateSegment(from,to,{ignoreSelected:false});}catch(error){console.log('ENTRY_COLLISION',phase,JSON.stringify(error.detail));throw error;}from=to;}}
  finally{agent.dynamics.syncRobot();}
});
test('finite wall inventory settles through contact; no bin or cargo position writes during two seconds',()=>{
  const objects=[...metadata.bins,...metadata.parts].map(object=>{const b=name('BODY',object.body),j=model.body_jntadr[b];return {object,b,q:model.jnt_qposadr[j],d:model.jnt_dofadr[j]};});
  let worstDrift=0,worstResidual=0;
  for(let step=0;step<2000;step++){
    const previous=objects.map(o=>Array.from(data.qpos.slice(o.q,o.q+3)));
    for(const agent of agents)agent.stepControls(.001);api.mj_step(model,data);
    if(step%10===0)for(const agent of agents)agent.afterWorldStep();
    assert.ok(data.qpos.every(Number.isFinite));
    for(const [i,o]of objects.entries()){
      const residual=Math.hypot(...previous[i].map((p,k)=>data.qpos[o.q+k]-p-data.qvel[o.d+k]*.001));worstResidual=Math.max(worstResidual,residual);
      worstDrift=Math.max(worstDrift,Math.hypot(...o.object.start.map((p,k)=>data.qpos[o.q+k]-p)));
    }
  }
  assert.ok(worstResidual<1e-5,`integration residual ${worstResidual}`);assert.ok(worstDrift<.008,`inventory drift ${worstDrift}`);
  const supports=new Map(),list=data.contact;
  try{for(let i=0;i<Math.min(data.ncon,list.size());i++){const c=list.get(i);try{if(c.efc_address>=0&&data.efc_force[c.efc_address]>.005){const a=model.geom_bodyid[c.geom1],b=model.geom_bodyid[c.geom2];supports.set(`${a}:${b}`,true);supports.set(`${b}:${a}`,true);}}finally{c.delete();}}}finally{list.delete();}
  for(const object of [...metadata.bins,...metadata.parts])assert.ok(supports.has(`${name('BODY',object.body)}:${name('BODY',object.sourceSupport)}`),`${object.id} unsupported`);
  console.log(JSON.stringify({scope:'Two-second static real MuJoCo settling only; not grasp/transport validation',worstDrift,worstResidual,steps:2000}));
});
test('carried-bin collision prediction includes loose cargo and never edits the live world',()=>{
  const agent=agents.find(a=>a.id==='g1'),bin=metadata.bins[0],slot=metadata.slots.find(s=>s.id==='full-1');
  agent.configureTask({target:{...bin,start:bodyPose(data,name('BODY',bin.body)).position,drop:toThree(slot.position)},arm:'right',supportBody:slot.supportBody});
  const actual=agent.controller.objectPose(bin.id),before=physicalSnapshot(data),parent=name('BODY',bin.body);
  const check=position=>agent.dynamics.checkPose(agent.controller.pose,{ignoreObjectBodies:[parent],objectPoses:[{body:parent,position,quaternion:actual.quaternion.toArray()}]});
  const supported=check(actual.position.toArray());assert.ok(supported.safe,JSON.stringify(supported.contacts));
  const blocked=check(actual.position.clone().add({x:.19,y:0,z:0}).toArray());assert.equal(blocked.safe,false);
  assert.ok(blocked.contacts.some(c=>bin.payloadBodies.includes(c.body1)||bin.payloadBodies.includes(c.body2)),'Prediction omitted loose cargo collisions');
  assert.deepEqual(physicalSnapshot(data),before,'Private payload prediction changed live bodies');
});
test('bin-cell stop and failure never open fingers or replace motor/pose targets',()=>{
  for(const agent of agents){
    const c=agent.controller,pose=structuredClone(c.pose),opening=structuredClone(c.openings??c.opening),before=physicalSnapshot(data);
    c.stop();assert.equal(c.state,'stopped');assert.equal(c.running,false);
    c.fail('deliberate fault contract test');assert.equal(c.state,'failed');assert.equal(c.running,false);
    assert.deepEqual(c.pose,pose);assert.deepEqual(c.openings??c.opening,opening);assert.deepEqual(physicalSnapshot(data),before);
  }
});
test('R1 released-bin prediction is independent of retreating empty wrist orientation',()=>{
  const agent=agents.find(a=>a.id==='r1'),bin=metadata.bins[0],slot=metadata.slots.find(s=>s.id==='full-1');
  agent.configureTask({target:{...bin,start:bodyPose(data,name('BODY',bin.body)).position,drop:toThree(slot.position)},arm:'right',supportBody:slot.supportBody});
  const c=agent.controller,actual=c.objectPose(bin.id),before=physicalSnapshot(data);
  const plan={drop:c.targets[0].drop.clone(),orientation:actual.quaternion.clone()};
  try{for(const angle of [-.5,.5]){
    const pose={...c.pose,right_arm_joint6:angle},prediction=c.predictedObject(pose,plan,'retreat')[0];
    assert.deepEqual(prediction.position,plan.drop.toArray());
    assert.deepEqual(prediction.quaternion,actual.quaternion.toArray());
  }}finally{agent.dynamics.syncRobot();}
  assert.deepEqual(physicalSnapshot(data),before);
});
test('empty carrier remains collision-protected without loose payload members',()=>{
  const agent=agents.find(a=>a.id==='r1'),bin=metadata.bins[0],slot=metadata.slots.find(s=>s.id==='full-1');
  agent.configureTask({target:{...bin,payloadBodies:[],start:bodyPose(data,name('BODY',bin.body)).position,drop:toThree(slot.position)},arm:'right',supportBody:slot.supportBody});
  const c=agent.controller,actual=c.objectPose(bin.id),before=physicalSnapshot(data),body=name('BODY',bin.body);
  const result=agent.dynamics.checkPose(c.pose,{ignoreObjectBodies:[body],objectPoses:[{body,position:actual.position.clone().add({x:0,y:-.03,z:0}).toArray(),quaternion:actual.quaternion.toArray()}]});
  assert.equal(result.safe,false);assert.ok(result.contacts.some(contact=>contact.kind==='carried-object-environment'));
  assert.deepEqual(physicalSnapshot(data),before);
});
test('independent ready and batch-home motor routes settle without per-item homing',()=>{
  for(const kind of ['ready','home']){
    const plans=agents.map(agent=>agent.planPosture(kind));
    for(const [i,agent]of agents.entries())agent.beginPosture(plans[i]);
    for(let i=0;i<20000&&agents.some(a=>a.postureMotion);i++){
      for(const agent of agents)agent.stepControls(.001);api.mj_step(model,data);if(i%10===0)for(const agent of agents)agent.afterWorldStep();
    }
    for(const agent of agents){assert.equal(agent.postureMotion,null);assert.equal(agent.controller.state,kind,`${agent.id}: ${agent.controller.message}`);assert.equal(agent.controller.sharedAtReady,kind==='ready');}
  }
});
test.after(()=>{for(const agent of agents)agent.dispose();data.delete();model.delete();});
