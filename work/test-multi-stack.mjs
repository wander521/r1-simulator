import assert from 'node:assert/strict';
import {readFile, mkdir, writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {ROBOT_CONFIGS, createRobotRuntimeFromParts} from '../dist/multi/robot-runtime.mjs';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';
import {loadTestRobot as loadR1} from './r1-test-robot.mjs';
import {loadG1TestRobot as loadG1} from './g1-test-robot.mjs';
import {loadC5TestRobot as loadC5} from './c5-test-robot.mjs';

// Independent end-to-end stack regression: the original three controllers and
// their source-built MuJoCo scenes advance together, without kinematic carries,
// synthetic contacts, weakened collision geometry or injected object poses.
const root=new URL('../',import.meta.url),ids=['r1','g1','c5'],order=['blue','red','yellow'];
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)||'work/validation/performance/stack-physical-final.json';
const sourcePaths=[
  'work/test-multi-stack.mjs','work/r1-test-robot.mjs','work/g1-test-robot.mjs','work/c5-test-robot.mjs',
  'dist/multi/robot-runtime.mjs','dist/multi/coordinator.mjs','dist/multi/r1-safe-dynamics.mjs',
  'dist/multi/planning.mjs','dist/multi/pipe-controller.mjs',
  ...ids.flatMap(id=>[`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,
    `dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
  'dist/c5/assets/aubo_C5.urdf','dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs',
  'dist/r1/mujoco-runtime.mjs','dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',
];
async function hashes(){return Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,
  createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));}
const sourceHashes=await hashes(),startedAt=new Date().toISOString(),wallStart=performance.now();
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const stations=new Map(),loaders={r1:loadR1,g1:loadG1,c5:loadC5};
const armByRobot={r1:'right',g1:'right',c5:'left'};
// Retain the already-declared legacy R1 numerical guard. Its soft-limit MJCF
// has not been modified or represented as the tighter Dex1 mechanism.
const fingerTolerance={r1:.001,g1:.0002,c5:.0002};
const results=[];let coordinator,stats,trackedObjects,planningStats,taskWindow=false,activeCase='initialization';

function compact(runtime){
  const c=runtime.controller;
  return {id:runtime.id,state:c.state,running:c.running,time:runtime.data.time,message:c.message,
    itemIndex:c.itemIndex,selected:c.selected,evidence:{...c.evidence},stackEvidence:structuredClone(c.stackEvidence),
    stackStableSeconds:c.stackStableSeconds,stackResultVerified:c.stackResultVerified,
    reaction:c.reaction,reactionFinished:c.reactionFinished,stackCheck:c.stackCheck(),history:[...c.history],planError:c.planError};
}
function objectState(runtime){return runtime.targets.map(target=>{
  const body=runtime.controller.bodies.get(target.id),joint=runtime.model.body_jntadr[body],q=runtime.model.jnt_qposadr[joint],d=runtime.model.jnt_dofadr[joint];
  return {id:target.id,body,q,d,previous:[...runtime.data.qpos.subarray(q,q+3)],previousSpeed:Math.hypot(...runtime.data.qvel.subarray(d,d+3))};
});}
function freshStats(){return Object.fromEntries(ids.map(id=>[id,{
  millisecondSamples:0,contactSamples:0,supportSamples:0,actualDualByObject:Object.fromEntries(order.map(objectId=>[objectId,false])),
  objectEditsDuringTask:0,maxObjectTranslationResidual:0,maxTorqueRatio:0,maxFingerForce:0,maxFingerLimitViolation:0,
  fingerStrokeToleranceM:fingerTolerance[id],maxRevoluteLimitViolation:0,maxActualVelocityRatio:0,
  maxIllegalPenetration:0,illegalContacts:[],transitions:[],lastTransition:null,
  independentStableSeconds:0,maxIndependentStableSeconds:0,extraStableSamples:0,
  supportMinNormalForce:Object.fromEntries(order.map(objectId=>[objectId,null])),maxStackSpeed:0,maxStackDropError:0,
}]));}
function currentSupport(runtime){
  const c=runtime.controller,forces=Object.fromEntries(order.map(id=>[id,0]));
  const bodies=[c.id('mjOBJ_BODY','task_table'),...order.map(id=>c.bodies.get(id))],contacts=runtime.data.contact;
  try{for(let index=0;index<Math.min(runtime.data.ncon,contacts.size());index++){
    const contact=contacts.get(index);
    try{
      const a=runtime.model.geom_bodyid[contact.geom1],b=runtime.model.geom_bodyid[contact.geom2];
      // The bundled elliptic-cone Wasm binding copies mj_contactForce output
      // arrays. efc_address reads the same solver normal directly, as in the
      // production controllers, rather than silently inspecting an all-zero JS array.
      const force=contact.efc_address>=0?Math.abs(runtime.data.efc_force[contact.efc_address]):0;
      for(let layer=0;layer<order.length;layer++)if((a===bodies[layer]&&b===bodies[layer+1])||(b===bodies[layer]&&a===bodies[layer+1]))forces[order[layer]]+=force;
    }finally{contact.delete();}
  }}finally{contacts.delete();}
  const check=c.stackCheck();
  return {stable:check.stable&&order.every(id=>forces[id]>.005),forces,states:check.states};
}
function inspectContacts(runtime,{requireStable=false}={}){
  const id=runtime.id,c=runtime.controller,s=stats[id],measurement=c.measure();s.contactSamples++;
  if(measurement.count===2&&measurement.forces.every(force=>force>.015))s.actualDualByObject[c.selected]=true;
  const transition=`${c.itemIndex}/${c.selected}/${c.state}`;
  if(transition!==s.lastTransition){s.transitions.push({time:runtime.data.time,key:transition});s.lastTransition=transition;}
  if(c.stackEvidence.length===3){
    const support=currentSupport(runtime);s.supportSamples++;
    s.independentStableSeconds=support.stable?s.independentStableSeconds+.01:0;
    s.maxIndependentStableSeconds=Math.max(s.maxIndependentStableSeconds,s.independentStableSeconds);
    if(support.stable)for(const state of support.states){
      s.supportMinNormalForce[state.id]=Math.min(s.supportMinNormalForce[state.id]??Infinity,support.forces[state.id]);
      s.maxStackSpeed=Math.max(s.maxStackSpeed,state.speed);s.maxStackDropError=Math.max(s.maxStackDropError,state.error);
    }
    if(requireStable){assert(support.stable,`${id}: stack lost true force-bearing support during extra 3 s: ${JSON.stringify(support)}`);s.extraStableSamples++;}
  }else assert(!requireStable,`${id}: missing stack evidence during extra stability window`);
  const dynamics=runtime.dynamics;if(!dynamics.robotBodyIds)return; // Retained R1 scene has finger contacts, not full shells.
  const selected=c.bodies.get(c.selected),fingers=new Set(c.fingers[c.arm].map(finger=>finger.body)),contacts=runtime.data.contact;
  try{for(let index=0;index<Math.min(runtime.data.ncon,contacts.size());index++){
    const contact=contacts.get(index);
    try{
      const a=runtime.model.geom_bodyid[contact.geom1],b=runtime.model.geom_bodyid[contact.geom2];
      if(!dynamics.robotBodyIds.has(a)&&!dynamics.robotBodyIds.has(b))continue;
      const footFloor=(dynamics.footBodyIds?.has(a)&&contact.geom2===dynamics.floorGeom)||(dynamics.footBodyIds?.has(b)&&contact.geom1===dynamics.floorGeom);
      const mounting=(a===dynamics.baseBody&&b===dynamics.plinthBody)||(b===dynamics.baseBody&&a===dynamics.plinthBody);
      const selectedFinger=(fingers.has(a)&&b===selected)||(fingers.has(b)&&a===selected);
      if(footFloor||mounting||selectedFinger)continue;
      const penetration=Math.max(0,-contact.dist);s.maxIllegalPenetration=Math.max(s.maxIllegalPenetration,penetration);
      if(penetration>.001){const detail={time:runtime.data.time,state:c.state,itemIndex:c.itemIndex,selected:c.selected,penetration,
        body1:dynamics.bodyNames[a],body2:dynamics.bodyNames[b],geom1:dynamics.geomNames[contact.geom1],geom2:dynamics.geomNames[contact.geom2]};
        if(s.illegalContacts.length<12)s.illegalContacts.push(detail);assert.fail(`${id}: illegal actual robot collision ${JSON.stringify(detail)}`);}
    }finally{contact.delete();}
  }}finally{contacts.delete();}
}
function drainPlanning(){
  if(!coordinator.planning)return;
  planningStats.barriers++;
  const tick=coordinator.tick,before=new Map([...stations].map(([id,runtime])=>[id,{time:runtime.data.time,qpos:[...runtime.data.qpos],qvel:[...runtime.data.qvel]}]));
  let slices=0;
  while(coordinator.planning){
    const start=performance.now(),result=coordinator.advancePlanning(),wallMs=performance.now()-start;
    planningStats.slices++;planningStats.wallMs+=wallMs;planningStats.maxSliceWallMs=Math.max(planningStats.maxSliceWallMs,wallMs);
    assert.equal(coordinator.tick,tick,'Planning advanced the global 1 ms clock');
    for(const [id,runtime]of stations){const previous=before.get(id);
      assert.equal(runtime.data.time,previous.time,`${id}: planning advanced a physical world`);
      assert.deepEqual([...runtime.data.qpos],previous.qpos,`${id}: planning edited actual qpos`);
      assert.deepEqual([...runtime.data.qvel],previous.qvel,`${id}: planning edited actual qvel`);
    }
    assert(++slices<10000,'Planning failed to return within a bounded number of route candidates');
    assert(result.ok!==false,result.reason||'Cooperative path planning failed');
  }
}
function tick(count=1,{requireStable=false}={}){
  for(let n=0;n<count;n++){
    drainPlanning();
    coordinator.step(.001);
    drainPlanning();
    const times=[...stations.values()].map(runtime=>runtime.data.time);
    assert(Math.max(...times)-Math.min(...times)<1e-10,'Three worlds diverged from their shared one-millisecond clock');
    for(const [id,runtime]of stations){
      const s=stats[id];s.millisecondSamples++;
      for(const object of trackedObjects.get(id)){
        const p=runtime.data.qpos.subarray(object.q,object.q+3),v=runtime.data.qvel.subarray(object.d,object.d+3),speed=Math.hypot(...v);
        assert([...p,...v].every(Number.isFinite),`${id}/${object.id}: non-finite free body state`);
        const distance=Math.hypot(...p.map((value,index)=>value-object.previous[index]));
        const residual=Math.max(0,distance-.001*Math.max(object.previousSpeed,speed));
        s.maxObjectTranslationResidual=Math.max(s.maxObjectTranslationResidual,residual);
        assert(residual<1e-5,`${id}/${object.id}: non-physical object position write, residual ${residual} m`);
        object.previous=[...p];object.previousSpeed=speed;
      }
      for(const joint of runtime.dynamics.map.values()){
        const actual=runtime.data.qpos[joint.q],effort=runtime.data.qfrc_actuator[joint.d],velocity=runtime.data.qvel[joint.d];
        assert([actual,effort,velocity].every(Number.isFinite),`${id}/${joint.name}: invalid actual motor state`);
        assert(Math.abs(effort)<=joint.effort+1e-6,`${id}/${joint.name}: actual effort exceeds source motor limit`);
        if(joint.effort>0)s.maxTorqueRatio=Math.max(s.maxTorqueRatio,Math.abs(effort)/joint.effort);
        const excess=Math.max(0,joint.lower-actual,actual-joint.upper);
        if(joint.finger){s.maxFingerForce=Math.max(s.maxFingerForce,Math.abs(effort));s.maxFingerLimitViolation=Math.max(s.maxFingerLimitViolation,excess);
          assert(excess<=fingerTolerance[id],`${id}/${joint.name}: actual finger range excess ${excess} m`);
        }else{s.maxRevoluteLimitViolation=Math.max(s.maxRevoluteLimitViolation,excess);assert(excess<=.001,`${id}/${joint.name}: revolute range excess ${excess} rad`);}
        if(id==='c5'&&Number.isFinite(joint.velocityLimit)&&joint.velocityLimit>0){
          s.maxActualVelocityRatio=Math.max(s.maxActualVelocityRatio,Math.abs(velocity)/joint.velocityLimit);
          assert(Math.abs(velocity)<=joint.velocityLimit*1.01+.0001,`${id}/${joint.name}: actual source velocity limit exceeded`);
        }
      }
      if(s.millisecondSamples%10===0)inspectContacts(runtime,{requireStable});
    }
    if(stats.r1.millisecondSamples%100===0)for(const runtime of stations.values())runtime.syncVisuals();
  }
}
function reset(){
  taskWindow=false;coordinator.reset();stats=freshStats();planningStats={barriers:0,slices:0,wallMs:0,maxSliceWallMs:0};trackedObjects=new Map([...stations].map(([id,runtime])=>[id,objectState(runtime)]));
  for(const runtime of stations.values()){assert.equal(runtime.data.time,0);assert.equal(runtime.controller.state,'idle');}
  tick(300);
}
function startStack(){
  coordinator.select(ids);const before=new Map([...stations].map(([id,runtime])=>[id,{time:runtime.data.time,objects:objectState(runtime).map(object=>[...runtime.data.qpos.subarray(object.q,object.q+7)])}]));
  taskWindow=true;
  const accepted=coordinator.start({task:'stack',objectId:'blue',armByRobot});assert.equal(accepted.ok,true,accepted.reason);
  drainPlanning();
  const result={...accepted,preparing:false,startTick:coordinator.startTick,startTimes:{...coordinator.startTimes}};
  assert.deepEqual(result.runIds,ids);assert.equal(result.startTick,coordinator.tick);
  assert.equal(new Set(Object.values(result.startTimes)).size,1,'All stack tasks must begin at the same simulation time');
  for(const [id,runtime]of stations){assert.equal(runtime.data.time,before.get(id).time,'Planning advanced a world ahead of the common clock');
    assert.deepEqual(objectState(runtime).map(object=>[...runtime.data.qpos.subarray(object.q,object.q+7)]),before.get(id).objects,'Planning moved a real task object');
    assert(runtime.controller.running);assert.equal(runtime.controller.task,'stack');}
  return result;
}
async function run(name,fn){
  activeCase=name;reset();const began=performance.now();
  try{const detail=await fn();results.push({name,pass:true,wallSeconds:(performance.now()-began)/1000,...detail,planning:structuredClone(planningStats),stats:structuredClone(stats),robots:[...stations.values()].map(compact)});
    console.error(JSON.stringify({scenario:name,pass:true,simTime:coordinator.tick*.001}));return true;
  }catch(error){results.push({name,pass:false,error:error.message,wallSeconds:(performance.now()-began)/1000,planning:structuredClone(planningStats),stats:structuredClone(stats),robots:[...stations.values()].map(compact)});
    console.error(JSON.stringify({scenario:name,pass:false,error:error.message}));return false;}
}

let initializationError;
try{
  for(const [index,id]of ids.entries()){
    const vfs=new api.MjVFS();let model,data;
    try{
      model=api.MjModel.from_xml_string(await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),vfs);data=new api.MjData(model);
      const runtime=createRobotRuntimeFromParts({id,api,model,data,robot:await loaders[id](),offset:[0,0,(1-index)*2.2],cooperativePlanning:true});
      const original=runtime.controller.setObject.bind(runtime.controller);
      runtime.controller.setObject=(...parameters)=>{
        if(taskWindow||runtime.controller.running){stats[id].objectEditsDuringTask++;assert.fail(`${id}: setObject called inside the task/extra-stability window`);}
        return original(...parameters);
      };
      assert.equal(model.nmocap,0);assert.equal(model.neq,0,'No object attachment, equality or weld is permitted');stations.set(id,runtime);
    }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
  }
  assert.equal(new Set([...stations.values()].map(runtime=>runtime.model)).size,3);
  assert.equal(new Set([...stations.values()].map(runtime=>runtime.data)).size,3);
  coordinator=new MultiRobotCoordinator({stations,selectedIds:ids});
  const completed=await run('three-way-simultaneous-blue-red-yellow-stack',()=>{
    const start=startStack();
    for(let n=0;n<150000&&coordinator.running;n++){
      tick();
      if(n>0&&n%20000===0)console.error(JSON.stringify({scenario:activeCase,simTime:coordinator.tick*.001,robots:[...stations.values()].map(runtime=>({id:runtime.id,state:runtime.controller.state,itemIndex:runtime.controller.itemIndex}))}));
    }
    assert.equal(coordinator.state,'complete',JSON.stringify([...stations.values()].map(compact)));
    for(const [id,runtime]of stations){
      const c=runtime.controller;assert.equal(c.state,'complete',`${id}: ${c.message}`);assert(!c.running);
      assert(c.stackResultVerified,`${id}: stack completion was not physically verified`);assert(c.stackStableSeconds>=3,`${id}: original 3 s stack stability not reached`);
      assert.deepEqual(c.stackEvidence.map(item=>item.id),order);
      for(const item of c.stackEvidence){assert(item.dual&&item.lift>.035,`${id}/${item.id}: missing independent lift/contact evidence`);assert(stats[id].actualDualByObject[item.id],`${id}/${item.id}: actual dual-finger normal force never exceeded .015 N per side`);}
      assert(stats[id].maxIndependentStableSeconds>=3,`${id}: independent force-bearing stack did not remain stable for 3 s`);
      assert(currentSupport(runtime).stable,`${id}: blue/red/yellow final support order is invalid`);
      assert.equal(stats[id].objectEditsDuringTask,0);
    }
    tick(3000,{requireStable:true});
    for(const [id,runtime]of stations){assert.equal(runtime.controller.state,'complete');assert.equal(stats[id].extraStableSamples,300);assert(currentSupport(runtime).stable);}
    return {start,extraStableSeconds:3,physicalStacks:3,physicallyGraspedItems:9};
  });
  if(completed)await run('stop-three-way-stack-after-real-lifts-never-reports-completion',()=>{
    const start=startStack();
    const lifted=()=>[...stations.values()].every(runtime=>runtime.controller.evidence.lift>.035||runtime.controller.stackEvidence.some(item=>item.lift>.035));
    for(let n=0;n<30000&&coordinator.running&&!lifted();n++)tick();
    assert(lifted(),'Stop fixture did not reach actual lifted objects on all three robots');
    assert([...stations.values()].every(runtime=>!runtime.controller.stackResultVerified),'Stop fixture unexpectedly completed a stack');
    const before=new Map([...stations].map(([id,runtime])=>[id,{objects:objectState(runtime).map(object=>[...runtime.data.qpos.subarray(object.q,object.q+7)]),pose:{...runtime.controller.pose},evidenceCount:runtime.controller.stackEvidence.length}]));
    const r1Opening=stations.get('r1').controller.opening;coordinator.stop();assert.equal(coordinator.state,'stopped');assert(!coordinator.running);
    assert.equal(stations.get('r1').controller.opening,r1Opening,'Stop injected an abrupt R1 full-open command');
    for(const [id,runtime]of stations){assert.equal(runtime.controller.state,'stopped');assert(!runtime.controller.running);
      assert.deepEqual(objectState(runtime).map(object=>[...runtime.data.qpos.subarray(object.q,object.q+7)]),before.get(id).objects,'Stop teleported an object');}
    tick(3000);
    assert.equal(coordinator.state,'stopped');
    for(const [id,runtime]of stations){const c=runtime.controller;assert.equal(c.state,'stopped');assert(!c.running);assert(!c.stackResultVerified);
      assert.deepEqual(c.pose,before.get(id).pose,'Stopped controller continued an arm trajectory');assert.equal(c.stackEvidence.length,before.get(id).evidenceCount);assert.equal(stats[id].objectEditsDuringTask,0);}
    return {start,postStopPhysicsSeconds:3};
  });
}catch(error){initializationError=error.message;console.error(error);}
finally{for(const runtime of stations.values())runtime.dispose();}

const finalHashes=await hashes(),sourceFrozen=JSON.stringify(finalHashes)===JSON.stringify(sourceHashes);
const report={pass:!initializationError&&results.length===2&&results.every(result=>result.pass)&&sourceFrozen,
  startedAt,wallSeconds:(performance.now()-wallStart)/1000,cases:results.length,physicalStacks:results[0]?.pass?3:0,
  physicallyGraspedStackItems:results[0]?.pass?9:0,clockStepSeconds:.001,fingerStrokeToleranceM:fingerTolerance,
  r1LimitPolicy:'Original R1 soft-limit scene retained; explicit 1 mm finger numerical guard, not Dex1 calibration accuracy. G1/C5 Dex1 guard stays 0.2 mm. No solver or stroke threshold was widened.',
  collisionScope:'Actual solver contacts every 10 ms. Original G1/C5 full source collisions; R1 original finger-contact scene only. Exemptions are exact foot/floor, exact C5 base/plinth, and active fingers/current selected object. Original source adjacency exclusions remain in the untouched MJCF. Separate stations cannot physically collide across worlds.',
  supportScope:'Independent table→blue→red→yellow force-bearing contact normals >0.005 N; source stack speed <0.0175 m/s and saved drop error <0.012 m. Per-item actual dual normal force >0.015 N per finger and lift >0.035 m. Original 3 s controller verification plus an independent stable window and a further 3 s of common-clock physical support.',
  motionIntegrity:'All nine objects checked every 1 ms for finite physical state and displacement-minus-dt*max(oldSpeed,newSpeed) <1e-5 m. setObject prohibited from start preparation through the extra stability/stop window, including after individual controllers finish. Every model has zero mocap and equality constraints.',
  planningScope:'Production cooperative planning enabled. Initial and subsequent stack-item planning drains between complete global 1 ms ticks before evidence sampling. Every planning slice asserts unchanged world time/qpos/qvel.',
  results,initializationError:initializationError??null,sourceFrozen,sourceHashes};
await mkdir(new URL('.',new URL(reportPath,root)),{recursive:true});await writeFile(new URL(reportPath,root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,cases:report.cases,physicalStacks:report.physicalStacks,physicallyGraspedStackItems:report.physicallyGraspedStackItems,
  sourceFrozen,report:reportPath,failures:results.filter(result=>!result.pass).map(result=>({name:result.name,error:result.error})),initializationError:report.initializationError},null,2));
if(!report.pass)process.exitCode=1;
