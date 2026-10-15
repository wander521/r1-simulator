import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';
import {R1_FINGER_MOTION_PROFILE} from '../dist/multi/r1-safe-dynamics.mjs';
import {loadTestRobot as loadR1} from './r1-test-robot.mjs';
import {loadG1TestRobot as loadG1} from './g1-test-robot.mjs';
import {loadC5TestRobot as loadC5} from './c5-test-robot.mjs';

// A single shared physics clock advances three genuinely separate production
// MuJoCo worlds. This verifies orchestration and local/render-frame isolation;
// the existing per-robot suites retain their deeper source-model validations.
const root=new URL('../',import.meta.url),ids=['r1','g1','c5'],results=[];
const args=process.argv.slice(2),filter=args.find(arg=>arg.startsWith('--case='))?.slice(7);
const paths=[
  'work/test-multi-physical.mjs','work/r1-test-robot.mjs','work/g1-test-robot.mjs','work/c5-test-robot.mjs',
  'dist/multi/coordinator.mjs','dist/multi/robot-runtime.mjs','dist/multi/r1-safe-dynamics.mjs',
  ...ids.flatMap(id=>[`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,
    `dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs',
  'work/test-r1-dynamics.mjs','work/validation/browser-regressions.json',
];
async function hashes(){return Object.fromEntries(await Promise.all(paths.map(async path=>[path,
  createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));}
const sourceHashes=await hashes();
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const loaders={r1:loadR1,g1:loadG1,c5:loadC5},stations=new Map();
const armByRobot={r1:'right',g1:'right',c5:'left'};
// R1's retained MJCF has default soft slide limits, 350 N/m motors and no
// Dex1-style solreflimit tightening. The old R1 tests did not impose a 0.2 mm
// stroke bound; its saved browser evidence already records 0.542 mm static
// aperture tracking error. Keep that legacy scene unchanged, add an explicit
// 1 mm numerical guard here, and report its actual excess separately. This is
// not a claim that the R1 physical gripper has 1 mm calibration accuracy.
const fingerStrokeTolerance={r1:.001,g1:.0002,c5:.0002};

function objectStates(runtime){return runtime.targets.map(target=>{
  const body=runtime.controller.bodies.get(target.id),joint=runtime.model.body_jntadr[body];
  const q=runtime.model.jnt_qposadr[joint],d=runtime.model.jnt_dofadr[joint];
  return {id:target.id,q,d,position:[...runtime.data.qpos.subarray(q,q+3)],velocity:[...runtime.data.qvel.subarray(d,d+3)]};
});}
const snapshot=runtime=>typeof runtime.snapshot==='function'?runtime.snapshot():runtime.snapshot;
const maxDelta=(a,b)=>Math.max(0,...a.map((value,index)=>Math.abs(value-b[index])));
const compact=runtime=>{
  const c=runtime.controller,m=c.measure();return {id:runtime.id,state:c.state,history:[...c.history],
    time:runtime.data.time,evidence:{...c.evidence},measurement:{count:m.count,forces:[...m.forces],
      position:m.position.toArray(),distance:m.distance,speed:m.speed,table:m.table,support:m.support}};
};
let coordinator,stats;
const batchSnapshot=()=>typeof coordinator.snapshot==='function'?coordinator.snapshot():coordinator.snapshot;
function freshStats(){return Object.fromEntries(ids.map(id=>[id,{millisecondSamples:0,contactSamples:0,
  actualDualForce:false,maxObjectTranslationResidual:0,objectEditsDuringTask:0,maxTorqueRatio:0,
  maxFingerLimitViolation:0,fingerStrokeToleranceM:fingerStrokeTolerance[id],maxIllegalPenetration:0,states:[]} ]));}
function inspectContacts(runtime){
  const c=runtime.controller,s=stats[runtime.id],m=c.measure();s.contactSamples++;
  if(m.count===2&&m.forces.every(force=>force>.015))s.actualDualForce=true;
  if(!s.states.includes(c.state))s.states.push(c.state);
  // R1 deliberately retains its supplied fixed-size finger collision model;
  // full shell/self collision coverage exists only for the G1 and C5 scenes.
  const dynamics=runtime.dynamics;if(!dynamics.robotBodyIds)return;
  const selected=c.bodies.get(c.selected),fingers=new Set(c.fingers[c.arm].map(finger=>finger.body));
  const contacts=runtime.data.contact;
  try{for(let index=0;index<Math.min(runtime.data.ncon,contacts.size());index++){
    const contact=contacts.get(index);
    try{
      const a=runtime.model.geom_bodyid[contact.geom1],b=runtime.model.geom_bodyid[contact.geom2];
      if(!dynamics.robotBodyIds.has(a)&&!dynamics.robotBodyIds.has(b))continue;
      const footFloor=(dynamics.footBodyIds?.has(a)&&contact.geom2===dynamics.floorGeom)
        ||(dynamics.footBodyIds?.has(b)&&contact.geom1===dynamics.floorGeom);
      const mounting=(a===dynamics.baseBody&&b===dynamics.plinthBody)||(b===dynamics.baseBody&&a===dynamics.plinthBody);
      const selectedFinger=(fingers.has(a)&&b===selected)||(fingers.has(b)&&a===selected);
      if(footFloor||mounting||selectedFinger)continue;
      const penetration=Math.max(0,-contact.dist);s.maxIllegalPenetration=Math.max(s.maxIllegalPenetration,penetration);
      assert(penetration<=.001,`${runtime.id}: illegal physical contact ${penetration} m in ${c.state}`);
    }finally{contact.delete();}
  }}finally{contacts.delete();}
}
function tick(count=1){
  for(let n=0;n<count;n++){
    const before=new Map([...stations].map(([id,runtime])=>[id,objectStates(runtime)]));
    coordinator.step(.001);
    const times=[...stations.values()].map(runtime=>runtime.data.time);
    assert(Math.max(...times)-Math.min(...times)<1e-10,'Independent worlds diverged from the common 1 ms clock');
    for(const [id,runtime]of stations){
      const s=stats[id];s.millisecondSamples++;
      for(const previous of before.get(id)){
        const position=[...runtime.data.qpos.subarray(previous.q,previous.q+3)],velocity=[...runtime.data.qvel.subarray(previous.d,previous.d+3)];
        assert([...position,...velocity].every(Number.isFinite),`${id}/${previous.id}: non-finite physical state`);
        const displacement=Math.hypot(...position.map((value,i)=>value-previous.position[i]));
        const residual=Math.max(0,displacement-.001*Math.max(Math.hypot(...previous.velocity),Math.hypot(...velocity)));
        s.maxObjectTranslationResidual=Math.max(s.maxObjectTranslationResidual,residual);
        assert(residual<1e-5,`${id}/${previous.id}: task object moved without physical velocity (${residual} m)`);
      }
      for(const joint of runtime.dynamics.map.values()){
        const actual=runtime.data.qpos[joint.q],effort=runtime.data.qfrc_actuator[joint.d];
        assert(Number.isFinite(actual)&&Number.isFinite(effort));
        assert(Math.abs(effort)<=joint.effort+1e-6,`${id}/${joint.name}: motor effort exceeds original scene limit`);
        if(joint.effort>0)s.maxTorqueRatio=Math.max(s.maxTorqueRatio,Math.abs(effort)/joint.effort);
        if(joint.finger){const excess=Math.max(0,joint.lower-actual,actual-joint.upper);
          s.maxFingerLimitViolation=Math.max(s.maxFingerLimitViolation,excess);
          assert(excess<=fingerStrokeTolerance[id],`${id}/${joint.name}: finger physical stroke excess ${excess} m`);}
      }
      if(s.millisecondSamples%10===0)inspectContacts(runtime);
    }
    if(stats.r1.millisecondSamples%100===0)for(const runtime of stations.values())runtime.syncVisuals();
  }
}
function reset(){coordinator.reset();stats=freshStats();for(const runtime of stations.values()){
  assert.equal(runtime.controller.running,false);assert.equal(runtime.controller.state,'idle');assert.equal(runtime.data.time,0);
}tick(300);}
function verifyVisualFrames(){for(const runtime of stations.values()){
  runtime.syncVisuals();assert.equal(runtime.robot.parent,null,'Canonical IK tree must not receive the visible station parent');
  assert.equal(runtime.visualRobot.parent,runtime.group);assert.notEqual(runtime.robot,runtime.visualRobot);
  const offset=runtime.group.position;
  for(const target of runtime.targets){
    const actual=runtime.controller.objectPose(target.id).position,world=target.visualMesh.getWorldPosition(new THREE.Vector3());
    assert(world.distanceTo(actual.clone().add(offset))<1e-10,'Visible task object does not match its own local physics frame plus station offset');
    assert(runtime.worldToLocal(world).distanceTo(actual)<1e-10);
    assert.equal(target.visualMesh.userData.robotId,runtime.id);assert.equal(target.zone.children.length,4);
  }
  assert(runtime.tcpWorld().distanceTo(runtime.localToWorld(runtime.config.tcp(runtime.robot,runtime.controller.arm).position))<1e-12);
}}
async function run(name,fn){if(filter&&!name.includes(filter))return;
  reset();try{await fn();verifyVisualFrames();results.push({name,pass:true,stats:structuredClone(stats),robots:[...stations.values()].map(compact)});
    console.error(JSON.stringify({scenario:name,pass:true}));
  }catch(error){results.push({name,pass:false,error:error.message,stats:structuredClone(stats),robots:[...stations.values()].map(compact)});
    console.error(JSON.stringify({scenario:name,pass:false,error:error.message}));}
}
function start(objectId,selectedIds=ids){coordinator.select(selectedIds);const before=[...stations.values()].map(runtime=>runtime.data.time);
  const result=coordinator.start({objectId,task:'sort',armByRobot});assert.equal(result.ok,true,result.reason);
  assert.deepEqual(result.runIds,selectedIds);assert.deepEqual([...stations.values()].map(runtime=>runtime.data.time),before,
    'Planning/starting must not advance any one world ahead of the common clock');
  return result;
}
function finish(selectedIds=ids){for(let step=0;step<45000&&coordinator.running;step++)tick();
  assert.equal(batchSnapshot().state,'complete',JSON.stringify(batchSnapshot().stations.map(s=>({id:s.id,state:s.state,message:s.message}))));
  for(const id of selectedIds){const runtime=stations.get(id),c=runtime.controller,m=c.measure();
    assert.equal(c.state,'complete',`${id}: ${c.message}`);assert(c.evidence.dual&&stats[id].actualDualForce,`${id}: no actual dual-finger grasp`);
    assert(c.evidence.lift>.035,`${id}: object was not physically lifted`);assert(m.table&&m.distance<.02,`${id}: released object did not reach its color frame`);
    assert.equal(stats[id].objectEditsDuringTask,0,`${id}: object edits during task`);
  }
  tick(1000); // Objects stay supported even after all task controllers finish.
  for(const id of selectedIds){const m=stations.get(id).controller.measure();assert(m.table&&m.distance<.02);}
}

try{
  for(const [index,id]of ids.entries()){
    const vfs=new api.MjVFS();let model,data;
    try{model=api.MjModel.from_xml_string(await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),vfs);data=new api.MjData(model);
      const robot=await loaders[id](),runtime=createRobotRuntimeFromParts({id,api,model,data,robot,offset:[0,0,(1-index)*2.2]});
      const original=runtime.controller.setObject.bind(runtime.controller);
      runtime.controller.setObject=(...parameters)=>{if(runtime.controller.running){stats[id].objectEditsDuringTask++;assert.fail(`${id}: task edited object state`);}return original(...parameters);};
      stations.set(id,runtime);
      assert.equal(model.nmocap,0);assert.equal(model.neq,0,'No hidden object attachment/weld allowed');
    }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
  }
  assert.equal(new Set([...stations.values()].map(runtime=>runtime.model)).size,3);
  assert.equal(new Set([...stations.values()].map(runtime=>runtime.data)).size,3);
  assert.equal(new Set([...stations.values()].map(runtime=>runtime.controller)).size,3);
  assert.equal(new Set([...stations.values()].map(runtime=>runtime.targets)).size,3);
  coordinator=new MultiRobotCoordinator({stations,selectedIds:ids});stats=freshStats();
  await run('row-layout-and-local-world-frame-isolation',()=>{
    const offsets=[...stations.values()].map(runtime=>runtime.group.position.toArray());
    assert(offsets.every(offset=>offset[0]===0&&offset[1]===0));assert.deepEqual(offsets.map(offset=>offset[2]),[2.2,0,-2.2]);
    verifyVisualFrames();const before=new Map([...stations].map(([id,runtime])=>[id,[...runtime.data.qpos]]));
    const r1=stations.get('r1'),blue=r1.targets.find(target=>target.id==='blue');
    assert(r1.moveObject('blue',blue.mesh.position.clone().add(new THREE.Vector3(.012,0,.012)),blue.mesh.quaternion));
    assert(maxDelta(before.get('r1'),[...r1.data.qpos])>.005);
    for(const id of ['g1','c5'])assert.deepEqual([...stations.get(id).data.qpos],before.get(id),'Editing same-color R1 object modified another robot world');
  });
  for(const objectId of ['red','blue','yellow'])await run(`three-way-simultaneous-${objectId}`,()=>{start(objectId);finish();});
  await run('single-selected-G1-leaves-other-tasks-idle',()=>{
    const resting=new Map(['r1','c5'].map(id=>[id,objectStates(stations.get(id)).map(object=>object.position)]));
    start('blue',['g1']);finish(['g1']);
    for(const id of ['r1','c5']){const runtime=stations.get(id);assert.equal(runtime.controller.state,'idle');assert.equal(runtime.controller.history.length,0);
      objectStates(runtime).forEach((object,index)=>assert(maxDelta(object.position,resting.get(id)[index])<.001));}
  });
  await run('two-selected-R1-C5-leaves-G1-task-idle',()=>{start('blue',['r1','c5']);finish(['r1','c5']);
    assert.equal(stations.get('g1').controller.state,'idle');assert.equal(stations.get('g1').controller.history.length,0);});
  await run('stop-three-robots-mid-grasp-without-object-teleport-and-reset-all',()=>{
    start('blue');for(let n=0;n<30000&&!ids.every(id=>stations.get(id).controller.evidence.lift>.035);n++)tick();
    assert(ids.every(id=>stations.get(id).controller.evidence.lift>.035),'Stop fixture never reached three actual lifts');
    const before=new Map([...stations].map(([id,runtime])=>[id,objectStates(runtime).map(object=>object.position)]));
    const r1Opening=stations.get('r1').controller.opening;coordinator.stop();
    assert.equal(stations.get('r1').controller.opening,r1Opening,'Stopping the R1 station must not inject a sudden full-opening target');
    assert.equal(batchSnapshot().running,false);
    for(const [id,runtime]of stations){assert.equal(runtime.controller.running,false);assert.equal(runtime.controller.state,'stopped');
      assert.deepEqual(objectStates(runtime).map(object=>object.position),before.get(id),'Stop changed actual task-object positions');}
    tick(300);coordinator.select(['c5']);coordinator.reset();
    for(const runtime of stations.values()){assert.equal(runtime.controller.state,'idle');assert.equal(runtime.data.time,0);
      for(const target of runtime.targets)assert(runtime.controller.objectPose(target.id).position.distanceTo(target.start)<1e-9);}
  });
  await run('unreachable-middle-station-aborts-every-attempted-start-without-reset',()=>{
    const g1=stations.get('g1');assert(g1.controller.setObject('blue',new THREE.Vector3(1.8,g1.config.tableHeight+.019,0)));
    const before=new Map([...stations].map(([id,runtime])=>[id,objectStates(runtime).map(object=>object.position)]));
    coordinator.select(ids);const result=coordinator.start({objectId:'blue',task:'sort',armByRobot});assert.equal(result.ok,false);
    assert.equal(batchSnapshot().running,false);
    for(const [id,runtime]of stations){assert.equal(runtime.controller.running,false,`${id}: failed start left a task/reaction running`);
      assert.deepEqual(objectStates(runtime).map(object=>object.position),before.get(id),'Aborted start reset/teleported an object');}
    assert.equal(stations.get('c5').controller.history.length,0,'A later station started after the middle rejection');
    coordinator.reset();for(const runtime of stations.values())assert.equal(runtime.controller.state,'idle');
  });
}finally{for(const runtime of stations.values())runtime.dispose();}

assert(results.length>0,'No physical scenario selected');
const finalHashes=await hashes(),sourceFrozen=JSON.stringify(finalHashes)===JSON.stringify(sourceHashes);
const failures=results.filter(result=>!result.pass);assert(sourceFrozen,'Multi-robot source changed during the physical regression');
console.log(JSON.stringify({pass:failures.length===0,cases:results.length,physicalSorts:results.filter(result=>result.pass&&result.name.includes('simultaneous')).length*3
  +results.filter(result=>result.pass&&result.name.startsWith('single-selected')).length
  +results.filter(result=>result.pass&&result.name.startsWith('two-selected')).length*2,
  clockStepSeconds:.001,renderOffsets:[[0,0,2.2],[0,0,0],[0,0,-2.2]],
  fingerStrokeToleranceM:fingerStrokeTolerance,
  r1CommandProfile:{...R1_FINGER_MOTION_PROFILE,scope:'Multi-station finger motor commands only; original R1 source scene, forces, gains, joint ranges and inertia are unchanged. Actual qpos/qvel are not clamped.'},
  r1LimitPolicy:'Original R1 default-soft-limit MJCF is preserved. Additional numerical guard is 1 mm; original R1 regressions did not specify a Dex1 0.2 mm bound. Saved original browser evidence includes 0.5419235 mm static finger tracking error. Actual range excess is reported, not hidden or clamped.',
  collisionScope:'Original per-robot scenes; G1/C5 full source collision meshes, R1 original finger contacts. Separate, non-overlapping station worlds do not collide across worlds.',
  results,sourceHashes,sourceFrozen}));
assert.equal(failures.length,0,`${failures.length} multi-robot physical cases failed`);
