import assert from 'node:assert/strict';
import {readFile, readdir, mkdir, writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {Worker} from 'node:worker_threads';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts, ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';
import {PLANNING_SEGMENT_BUDGET_MS,G1_PIPE_PLANNING_SEGMENT_BUDGET_MS} from '../dist/multi/planning.mjs';
import {createPlanningWorkerTransport} from '../dist/multi/planning-worker-transport.mjs';
import {loadTestRobot as loadR1} from './r1-test-robot.mjs';
import {loadG1TestRobot as loadG1} from './g1-test-robot.mjs';
import {loadC5TestRobot as loadC5} from './c5-test-robot.mjs';
import {LEG_JOINT_NAMES as G1_LEGS,ROOT_HEIGHT as G1_ROOT_HEIGHT} from '../dist/g1/g1-profile.mjs';

// Independent task acceptance, not an animation or mocked-contact test. This
// advances the production task/controller/scene using the original URDF trees.
// No test may rescue a failed grasp by writing a task object's physical state.
const root = new URL('../', import.meta.url);
const robotIds = ['r1', 'g1', 'c5'];
const loaders = {r1:loadR1, g1:loadG1, c5:loadC5};
const args = process.argv.slice(2);
const quick = args.includes('--quick');
const staticOnly = args.includes('--static-only');
const backgroundPlanning=args.includes('--background-planning');
const filter = args.find(arg => arg.startsWith('--case='))?.slice(7);
const mode=staticOnly?'static-only':quick?'quick':filter?'filtered':'full';
const selectedSeed = Number(args.find(arg => arg.startsWith('--seed='))?.slice(7) ?? 521);
const reportPath = args.find(arg => arg.startsWith('--report='))?.slice(9)
  ?? `work/validation/multi-pipes/${mode==='full'?'physical-final':mode==='static-only'?'layout-final':`${mode}-development`}.json`;
assert(Number.isSafeInteger(selectedSeed)&&selectedSeed>=0&&selectedSeed+139<=0xffffffff,
  'Seed must be an unsigned 32-bit integer with room for the regression seed offsets');
const fingerTolerance = {r1:.001, g1:.0002, c5:.0002};
const startedAt = new Date().toISOString(), wallStart = performance.now();
const results = [], stations = new Map();
const sourceVelocities=new Map();
let coordinator, stats, tracked, stance, taskWindow = false;

const taskSources = (await readdir(new URL('dist/multi/', root)))
  .filter(name => name.endsWith('.mjs') && !['app.mjs','presentation.mjs','workcell.mjs'].includes(name))
  .map(name => `dist/multi/${name}`);
const sourcePaths = ['work/test-multi-pipes.mjs', ...taskSources,
  ...(backgroundPlanning?['work/background-planning-node-worker.mjs']:[]),
  ...robotIds.flatMap(id => [`work/${id === 'r1' ? 'r1' : id}-test-robot.mjs`,
    `dist/${id}/grasp-controller.mjs`, `dist/${id}/grasp-kinematics.mjs`,
    `dist/${id}/${id}-dynamics.mjs`, `dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/r1/mujoco-runtime.mjs',
  'dist/vendor/three.module.min.js',
  'dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];
assert(sourcePaths.includes('dist/multi/planning-check-cache.mjs'),'Worker safety-cache source must be bound to physical evidence');
async function hashes() { return Object.fromEntries(await Promise.all(sourcePaths.map(async path =>
  [path, createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')]))); }
const sourceHashes = await hashes();
console.error(JSON.stringify({physicalRunStarted:startedAt,mode,report:reportPath,
  planningMode:backgroundPlanning?'worker':'cooperative',coordinatorSha256:sourceHashes['dist/multi/coordinator.mjs'],planningSha256:sourceHashes['dist/multi/planning.mjs']}));
const api = staticOnly ? null : await createMuJoCo({locateFile:() =>
  new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const snapshot = runtime => typeof runtime.snapshot === 'function' ? runtime.snapshot() : runtime.snapshot;
const distance = (a,b) => Math.hypot(...a.map((value,index) => value-b[index]));
const arraysEqual = (a,b) => JSON.stringify(a) === JSON.stringify(b);
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
function createNodeWorker(){
  const worker=new Worker(new URL('./background-planning-node-worker.mjs',import.meta.url),{type:'module'});
  const adapter={postMessage:message=>worker.postMessage(message),terminate:()=>worker.terminate(),onmessage:null,onerror:null,onmessageerror:null};
  worker.on('message',message=>adapter.onmessage?.({data:message}));
  worker.on('error',error=>adapter.onerror?.({message:error.message,preventDefault(){}}));
  worker.on('messageerror',()=>adapter.onmessageerror?.({}));return adapter;
}

async function productionLayout(id, seed) {
  const {generatePipeLayout}=await import('../dist/multi/pipe-layout.mjs');
  return generatePipeLayout(ROBOT_CONFIGS[id],{seed});
}

function frameFor(target,bins) {
  assert(Array.isArray(bins),'Production destination bins must be an array');
  const bin=bins.find(item=>item.colorKey===target.colorKey);
  assert(bin,`${target.id}: corresponding color bin missing`);
  return {colorKey:bin.colorKey,minX:bin.center.x-bin.halfX,maxX:bin.center.x+bin.halfX,
    minZ:bin.center.z-bin.halfZ,maxZ:bin.center.z+bin.halfZ};
}

function validateLayout(id, seed, layout) {
  const config=ROBOT_CONFIGS[id], targets=layout.targets;
  assert(Array.isArray(targets) && targets.length===8, `${id}/${seed}: expected eight pipes`);
  assert.equal(new Set(targets.map(target=>target.id)).size,8, `${id}: duplicate pipe IDs`);
  assert(Array.isArray(layout.bins)&&layout.bins.length===2,`${id}: expected one red and one yellow destination frame`);
  for(const color of ['yellow','red']) assert.equal(targets.filter(target=>target.colorKey===color).length,4,
    `${id}/${seed}: expected four ${color} pipes`);
  for(const target of targets) {
    assert.equal(target.shape,'cylinder', `${id}/${target.id}: wrong physical/visual shape`);
    assert(target.start instanceof THREE.Vector3 && target.drop instanceof THREE.Vector3);
    assert(target.start.x < target.drop.x, `${id}/${target.id}: pipes must begin nearer the robot than their color frames`);
    for(const point of [target.start,target.drop]) {
      assert(point.x-target.halfX>=config.tableCenter-config.tableHalfX,
        `${id}/${target.id}: pipe exceeds near table edge`);
      assert(point.x+target.halfX<=config.tableCenter+config.tableHalfX,
        `${id}/${target.id}: pipe exceeds far table edge`);
      assert(Math.abs(point.z)+target.halfZ<=config.tableHalfZ, `${id}/${target.id}: pipe exceeds lateral table edge`);
      assert(Math.abs(point.y-(config.tableHeight+target.h))<1e-8, `${id}/${target.id}: wrong tabletop height`);
    }
    const frame=frameFor(target,layout.bins);assert.equal(frame.colorKey,target.colorKey);
    assertInFrame(target.drop.toArray(),target,frame,`${id}/${target.id} planned drop`);
  }
  for(let a=0;a<targets.length;a++) for(let b=a+1;b<targets.length;b++) {
    assert(targets[a].start.distanceTo(targets[b].start)>targets[a].halfX+targets[b].halfX+.005,
      `${id}/${seed}: initial pipes overlap`);
    assert(targets[a].drop.distanceTo(targets[b].drop)>targets[a].halfX+targets[b].halfX+.005,
      `${id}/${seed}: final pipes overlap`);
  }
  const nearestFrame=Math.min(...layout.bins.map(bin=>bin.center.x-bin.halfX));
  assert(Math.max(...targets.map(target=>target.start.x+target.halfX))<nearestFrame,
    `${id}/${seed}: near-side pipe area overlaps the far-side classification frames`);
  for(const bin of layout.bins) {
    assert(bin.center.x-bin.halfX>=config.tableCenter-config.tableHalfX
      &&bin.center.x+bin.halfX<=config.tableCenter+config.tableHalfX
      &&Math.abs(bin.center.z)+bin.halfZ<=config.tableHalfZ,
      `${id}/${seed}/${bin.colorKey}: destination frame exceeds tabletop`);
    assert.equal(bin.slots.length,4,`${id}: each color frame needs four reserved pipe slots`);
  }
  assert(Array.isArray(layout.queue)&&layout.queue.length===8,`${id}/${seed}: expected eight queued object IDs`);
  assert.equal(new Set(layout.queue).size,8);assert(layout.queue.every(objectId=>targets.some(target=>target.id===objectId)));
  return targets.map(target=>({id:target.id,color:target.colorKey,start:target.start.toArray(),drop:target.drop.toArray(),
    frame:frameFor(target,layout.bins)}));
}
function assertInFrame(position,target,frame,label,quaternion=new THREE.Quaternion()) {
  const axis=new THREE.Vector3(0,1,0).applyQuaternion(quaternion);
  const extent=component=>target.halfX*Math.sqrt(Math.max(0,1-component*component))+target.h*Math.abs(component);
  const extentX=extent(axis.x),extentZ=extent(axis.z);
  assert(frame && [frame.minX,frame.maxX,frame.minZ,frame.maxZ].every(Number.isFinite), `${label}: destination bounds missing`);
  assert(position[0]-extentX>=frame.minX-.003 && position[0]+extentX<=frame.maxX+.003
    && position[2]-extentZ>=frame.minZ-.003 && position[2]+extentZ<=frame.maxZ+.003,
    `${label}: actual pipe is not wholly inside its corresponding color frame`);
}

function objectState(runtime) {
  const c=runtime.controller;
  return runtime.targets.map(target=>{
    const body=c.bodies.get(target.id), joint=runtime.model.body_jntadr[body];
    assert(Number.isInteger(body) && body>0, `${runtime.id}/${target.id}: missing actual physical body`);
    assert.equal(runtime.model.jnt_type[joint], api.mjtJoint.mjJNT_FREE.value);
    const geom=runtime.model.body_geomadr[body];assert.equal(runtime.model.body_geomnum[body],1);
    assert.equal(runtime.model.geom_type[geom],api.mjtGeom.mjGEOM_CYLINDER.value,`${runtime.id}/${target.id}: actual physics shape is not a cylinder`);
    assert(Math.abs(runtime.model.geom_size[geom*3]-.02)<1e-12&&Math.abs(runtime.model.geom_size[geom*3+1]-.026)<1e-12);
    assert(Math.abs(runtime.model.body_mass[body]-.042)<1e-12,`${runtime.id}/${target.id}: source cylinder mass changed`);
    assert.equal(runtime.model.geom_contype[geom],1);assert.equal(runtime.model.geom_conaffinity[geom],3);
    const q=runtime.model.jnt_qposadr[joint], d=runtime.model.jnt_dofadr[joint];
    return {id:target.id,target,frame:frameFor(target,c.classification.bins),body,q,d,previous:[...runtime.data.qpos.subarray(q,q+3)],
      previousQuaternion:[...runtime.data.qpos.subarray(q+3,q+7)],previousAngularSpeed:Math.hypot(...runtime.data.qvel.subarray(d+3,d+6)),
      previousSpeed:Math.hypot(...runtime.data.qvel.subarray(d,d+3)),initialY:c.objectPose(target.id).position.y};
  });
}
function freshStats(runtime) {
  return {millisecondSamples:0,contactSamples:0,objectEditsDuringTask:0,maxTranslationResidualM:0,maxRotationResidualRad:0,
    maxTorqueRatio:0,maxFingerLimitViolationM:0,fingerToleranceM:fingerTolerance[runtime.id],
    maxRevoluteLimitViolationRad:0,maxActualVelocityRatio:0,maxIllegalPenetrationM:0,maxPipePipePenetrationM:0,maxOffTargetFingerForceN:0,
    maxG1LegAngleRad:0,maxG1FootDriftM:0,visualFramesChecked:0,
    unratedWheelNames:[],maxUnratedWheelVelocityRadps:0,
    planningRejectedCandidates:0,planningRejections:[],criticalVelocityExtrema:{},
    planning:{cooperative:!backgroundPlanning,worker:backgroundPlanning,candidateSlices:0,polls:0,hostPollMs:0,maxHostPollMs:0,accumulatedSegmentMs:0,maxSliceMs:0,maxPreparationSegmentMs:0,completedPreparations:0,failedPreparations:0,slowPreparations:[]},
    sequence:[],lastSelected:null,extraStableSamples:0,
    objects:Object.fromEntries(runtime.targets.map(target=>[target.id,{id:target.id,color:target.colorKey,
      start:target.start.toArray(),drop:target.drop.toArray(),frame:frameFor(target,runtime.controller.classification.bins),arms:[],states:[],actualDual:false,
      maxLiftM:0,maxSpeedMps:0,maxFingerForcesN:[0,0],supportNormalN:0,final:null,
      carry:{samplePeriodSeconds:.01,samples:0,dualSamples:0,dualFraction:0,longestGripLossSeconds:0,currentGripLossSeconds:0,
        maxTcpDistanceM:0,maxRelativePositionDriftM:0,maxRelativeAngleDriftRad:0,baseline:null,phases:{},trace:[]}}]))};
}
function observeCarry(runtime,observed,pose,forces) {
  const c=runtime.controller,phase=c.state;
  if(!c.running||c.selected!==observed.id||!['lift','transfer'].includes(phase))return;
  const tcp=runtime.config.tcp(runtime.robot,c.arm),inverse=tcp.quaternion.clone().invert();
  const relativePosition=pose.position.clone().sub(tcp.position).applyQuaternion(inverse);
  const relativeQuaternion=inverse.clone().multiply(pose.quaternion).normalize();
  const dual=forces.every(force=>force>.015),distance=pose.position.distanceTo(tcp.position),carry=observed.carry;
  const baseline={position:relativePosition.toArray(),quaternion:relativeQuaternion.toArray()};
  carry.baseline??=baseline;carry.samples++;if(dual)carry.dualSamples++;carry.dualFraction=carry.dualSamples/carry.samples;
  carry.currentGripLossSeconds=dual?0:carry.currentGripLossSeconds+.01;
  carry.longestGripLossSeconds=Math.max(carry.longestGripLossSeconds,carry.currentGripLossSeconds);
  carry.maxTcpDistanceM=Math.max(carry.maxTcpDistanceM,distance);
  carry.maxRelativePositionDriftM=Math.max(carry.maxRelativePositionDriftM,relativePosition.distanceTo(new THREE.Vector3().fromArray(carry.baseline.position)));
  carry.maxRelativeAngleDriftRad=Math.max(carry.maxRelativeAngleDriftRad,relativeQuaternion.angleTo(new THREE.Quaternion().fromArray(carry.baseline.quaternion)));
  const part=carry.phases[phase]??={samples:0,dualSamples:0,dualFraction:0,currentGripLossSeconds:0,longestGripLossSeconds:0,maxTcpDistanceM:0};
  part.samples++;if(dual)part.dualSamples++;part.dualFraction=part.dualSamples/part.samples;
  part.currentGripLossSeconds=dual?0:part.currentGripLossSeconds+.01;
  part.longestGripLossSeconds=Math.max(part.longestGripLossSeconds,part.currentGripLossSeconds);part.maxTcpDistanceM=Math.max(part.maxTcpDistanceM,distance);
  const previous=carry.trace.at(-1);
  if(!previous||previous.phase!==phase||previous.dual!==dual||carry.samples%10===0)
    carry.trace.push({time:runtime.data.time,phase,dual,forces:[...forces],tcpDistanceM:distance,
      relativePosition:relativePosition.toArray(),relativeQuaternion:relativeQuaternion.toArray()});
}
function robotSummary(runtime) {
  const c=runtime.controller,classification=c.classification??snapshot(runtime).classification;
  const plainClassification=classification?{
    total:classification.total,completed:classification.completed,activeIndex:classification.activeIndex,
    started:classification.started,
    seed:classification.seed,queue:[...(classification.queue??[])],armSequence:[...classification.armSequence],
    verified:classification.verified,finalStableSeconds:classification.finalStableSeconds,
    items:classification.items.map(item=>({...item})),check:classification.check?{
      stable:classification.check.stable,states:classification.check.states.map(item=>({...item}))}:null,
    bins:classification.bins.map(bin=>({id:bin.id,colorKey:bin.colorKey,center:bin.center.toArray(),
      halfX:bin.halfX,halfZ:bin.halfZ,slots:bin.slots.map(point=>point.toArray())})),
  }:null;
  return {id:runtime.id,state:c.state,running:c.running,time:runtime.data.time,message:c.message,
    arm:c.arm,selected:c.selected,classification:plainClassification,
    history:[...c.history],planError:c.planError??null};
}
function scanContacts(runtime,{requireStable=false}={}) {
  // Carry observations use actual FK at this physical sample. Routine visual
  // synchronization is deferred by the production multi-robot runtime.
  runtime.dynamics.syncRobot();
  const c=runtime.controller,s=stats[runtime.id],table=c.id('mjOBJ_BODY','task_table');
  const byBody=new Map(tracked.get(runtime.id).map(object=>[object.body,object]));
  const support=Object.fromEntries(runtime.targets.map(target=>[target.id,{normal:0,finger:[0,0],allFingerNormal:0}]));
  const selectedBody=c.bodies.get(c.selected), activeFingers=c.fingers[c.arm]??[];
  const activeFingerBodies=new Set(activeFingers.map(finger=>finger.body));
  const everyFingerBody=new Set(Object.values(c.fingers).flat().map(finger=>finger.body)),contacts=runtime.data.contact;
  try { for(let index=0;index<Math.min(runtime.data.ncon,contacts.size());index++) {
    const contact=contacts.get(index);
    try {
      const a=runtime.model.geom_bodyid[contact.geom1],b=runtime.model.geom_bodyid[contact.geom2];
      const normal=contact.efc_address>=0?Math.abs(runtime.data.efc_force[contact.efc_address]):0;
      for(const [body,other] of [[a,b],[b,a]]) {
        const object=byBody.get(body); if(!object)continue;
        if(other===table) support[object.id].normal+=normal;
        if(everyFingerBody.has(other))support[object.id].allFingerNormal+=normal;
        const finger=activeFingers.findIndex(item=>item.body===other);
        if(finger>=0) support[object.id].finger[finger]+=normal;
      }
      if(taskWindow&&c.running&&(everyFingerBody.has(a)||everyFingerBody.has(b))) {
        const intended=(activeFingerBodies.has(a)&&b===selectedBody)||(activeFingerBodies.has(b)&&a===selectedBody);
        if(!intended) {
          s.maxOffTargetFingerForceN=Math.max(s.maxOffTargetFingerForceN,normal);
          assert(normal<=.015,`${runtime.id}: a gripper pushed a non-target body with ${normal} N in ${c.state}`);
        }
      }
      if(byBody.has(a)&&byBody.has(b)) {
        const penetration=Math.max(0,-contact.dist);s.maxPipePipePenetrationM=Math.max(s.maxPipePipePenetrationM,penetration);
        assert(penetration<=.001,`${runtime.id}: actual pipe/pipe interpenetration exceeds 1 mm in ${c.state}`);
      }
      const dynamics=runtime.dynamics;
      if(!dynamics.robotBodyIds || (!dynamics.robotBodyIds.has(a)&&!dynamics.robotBodyIds.has(b)))continue;
      const footFloor=(dynamics.footBodyIds?.has(a)&&contact.geom2===dynamics.floorGeom)
        ||(dynamics.footBodyIds?.has(b)&&contact.geom1===dynamics.floorGeom);
      const mounting=(a===dynamics.baseBody&&b===dynamics.plinthBody)||(b===dynamics.baseBody&&a===dynamics.plinthBody);
      const selectedFinger=(activeFingerBodies.has(a)&&b===selectedBody)||(activeFingerBodies.has(b)&&a===selectedBody);
      if(footFloor||mounting||selectedFinger)continue;
      const penetration=Math.max(0,-contact.dist);s.maxIllegalPenetrationM=Math.max(s.maxIllegalPenetrationM,penetration);
      assert(penetration<=.001,`${runtime.id}: illegal physical contact in ${c.state}: ${dynamics.bodyNames[a]} / ${dynamics.bodyNames[b]} ${penetration} m`);
    } finally { contact.delete(); }
  }} finally { contacts.delete(); }
  s.contactSamples++;
  if(runtime.id==='g1')for(const [body,reference]of stance.feet) {
    const drift=distance(runtime.data.xpos.subarray(body*3,body*3+3),reference);
    s.maxG1FootDriftM=Math.max(s.maxG1FootDriftM,drift);assert(drift<.0005,`G1 fixed foot moved ${drift} m`);
  }
  if(c.running && s.objects[c.selected]) {
    const observed=s.objects[c.selected];
    if(s.lastSelected!==c.selected) {s.sequence.push({id:c.selected,arm:c.arm,time:runtime.data.time});s.lastSelected=c.selected;}
    if(!observed.arms.includes(c.arm))observed.arms.push(c.arm);
    if(!observed.states.includes(c.state))observed.states.push(c.state);
  }
  for(const object of tracked.get(runtime.id)) {
    const observed=s.objects[object.id],pose=c.objectPose(object.id),p=pose.position,speed=Math.hypot(...runtime.data.qvel.subarray(object.d,object.d+3));
    observed.maxLiftM=Math.max(observed.maxLiftM,p.y-object.initialY);observed.maxSpeedMps=Math.max(observed.maxSpeedMps,speed);
    observed.maxFingerForcesN=observed.maxFingerForcesN.map((force,index)=>Math.max(force,support[object.id].finger[index]));
    if(support[object.id].finger.every(force=>force>.015))observed.actualDual=true;
    observeCarry(runtime,observed,pose,support[object.id].finger);
    if(requireStable) {
      assert(support[object.id].normal>.005,`${runtime.id}/${object.id}: no true table support after completion`);
      assert(speed<.025,`${runtime.id}/${object.id}: released pipe is still moving`);
      assert(support[object.id].allFingerNormal<.015,`${runtime.id}/${object.id}: pipe still held by either hand after task completion`);
      assertInFrame(p.toArray(),object.target,object.frame,`${runtime.id}/${object.id}`,pose.quaternion);
      assert(p.distanceTo(object.target.drop)<.02,`${runtime.id}/${object.id}: pipe missed its reserved non-overlapping color slot`);
      observed.supportNormalN=support[object.id].normal;observed.final={position:p.toArray(),speedMps:speed,
        quaternion:pose.quaternion.toArray(),slotErrorM:p.distanceTo(object.target.drop),fingerForcesN:support[object.id].finger,allFingerNormalN:support[object.id].allFingerNormal};
    }
  }
  if(requireStable)s.extraStableSamples++;
}
async function drainPlanning() {
  let result={ok:true};
  while(coordinator.planning) {
    const before=new Map([...stations].map(([id,runtime])=>[id,{time:runtime.data.time,qpos:[...runtime.data.qpos],qvel:[...runtime.data.qvel],ctrl:[...runtime.data.ctrl]}]));
    const tickBefore=coordinator.tick;
    result=coordinator.advancePlanning();
    assert.equal(coordinator.tick,tickBefore,'Planning advanced the shared physical tick');
    for(const [id,runtime]of stations){
      assert.equal(runtime.data.time,before.get(id).time,'Planning advanced one physical world');
      assert.deepEqual([...runtime.data.qpos],before.get(id).qpos,`${id}: a planning candidate edited physical qpos`);
      assert.deepEqual([...runtime.data.qvel],before.get(id).qvel,`${id}: a planning candidate edited physical qvel`);
      assert.deepEqual([...runtime.data.ctrl],before.get(id).ctrl,`${id}: a planning candidate edited physical controls`);
    }
    if(result.ok===false)return result;
    if(backgroundPlanning&&coordinator.planning)await wait(10);
  }
  return result;
}
async function tick(count=1,{stableIds=[]}={}) {
  for(let n=0;n<count;n++) {
    const preparation=await drainPlanning();assert.equal(preparation.ok,true,`${preparation.reason}; ${JSON.stringify([...stations.values()].map(robotSummary))}`);
    coordinator.step(.001);
    const times=[...stations.values()].map(runtime=>runtime.data.time);
    assert(Math.max(...times)-Math.min(...times)<1e-10,'Selected/hidden physical worlds diverged from the shared 1 ms clock');
    for(const [id,runtime]of stations) {
      const s=stats[id];s.millisecondSamples++;
      for(const object of tracked.get(id)) {
        const p=runtime.data.qpos.subarray(object.q,object.q+3),v=runtime.data.qvel.subarray(object.d,object.d+3),speed=Math.hypot(...v);
        assert([...p,...v].every(Number.isFinite),`${id}/${object.id}: non-finite real free body state`);
        const residual=Math.max(0,distance(p,object.previous)-.001*Math.max(speed,object.previousSpeed));
        s.maxTranslationResidualM=Math.max(s.maxTranslationResidualM,residual);
        assert(residual<1e-5,`${id}/${object.id}: task object moved without physical velocity (${residual} m)`);
        object.previous=[...p];object.previousSpeed=speed;
        const q=runtime.data.qpos.subarray(object.q+3,object.q+7),angular=runtime.data.qvel.subarray(object.d+3,object.d+6),angularSpeed=Math.hypot(...angular);
        assert([...q,...angular].every(Number.isFinite)&&Math.abs(Math.hypot(...q)-1)<1e-5,`${id}/${object.id}: invalid real object orientation`);
        const delta=2*Math.acos(Math.min(1,Math.abs(q.reduce((sum,value,index)=>sum+value*object.previousQuaternion[index],0))));
        const rotationResidual=Math.max(0,delta-.001*Math.max(angularSpeed,object.previousAngularSpeed));
        s.maxRotationResidualRad=Math.max(s.maxRotationResidualRad,rotationResidual);
        assert(rotationResidual<1e-5,`${id}/${object.id}: task object rotated without physical angular velocity (${rotationResidual} rad)`);
        object.previousQuaternion=[...q];object.previousAngularSpeed=angularSpeed;
      }
      for(const joint of runtime.dynamics.map.values()) {
        const actual=runtime.data.qpos[joint.q],effort=runtime.data.qfrc_actuator[joint.d],velocity=runtime.data.qvel[joint.d];
        assert([actual,effort,velocity].every(Number.isFinite));assert(Math.abs(effort)<=joint.effort+1e-6,`${id}/${joint.name}: actual effort exceeds original motor limit`);
        if(joint.effort>0)s.maxTorqueRatio=Math.max(s.maxTorqueRatio,Math.abs(effort)/joint.effort);
        const excess=Math.max(0,joint.lower-actual,actual-joint.upper);
        if(joint.finger) {s.maxFingerLimitViolationM=Math.max(s.maxFingerLimitViolationM,excess);
          assert(excess<=fingerTolerance[id],`${id}/${joint.name}: physical finger stroke excess ${excess} m`);}
        else {s.maxRevoluteLimitViolationRad=Math.max(s.maxRevoluteLimitViolationRad,excess);
          assert(excess<=.001,`${id}/${joint.name}: physical revolute range excess ${excess} rad`);}
        const sourceVelocity=sourceVelocities.get(id)?.get(joint.name);
        if(Number.isFinite(sourceVelocity)&&sourceVelocity>0) {
          if(Number.isFinite(joint.velocityLimit))assert(Math.abs(joint.velocityLimit-sourceVelocity)<1e-12,`${id}/${joint.name}: dynamics/source velocity limits disagree`);
          s.maxActualVelocityRatio=Math.max(s.maxActualVelocityRatio,Math.abs(velocity)/sourceVelocity);
          if(Math.abs(velocity)/sourceVelocity>.9&&Math.abs(velocity)>(s.criticalVelocityExtrema[joint.name]?.absoluteVelocity??0))
            s.criticalVelocityExtrema[joint.name]={time:runtime.data.time,phase:runtime.controller.state,selected:runtime.controller.selected,
              absoluteVelocity:Math.abs(velocity),velocity,sourceVelocity,position:actual,ctrl:runtime.data.ctrl[joint.actuator],
              target:runtime.dynamics.targets[joint.name],effort};
          assert(Math.abs(velocity)<=sourceVelocity*1.01+.0001,`${id}/${joint.name}: actual source velocity limit exceeded (${velocity} vs ${sourceVelocity} at ${runtime.data.time}s/${runtime.controller.state})`);
        } else {
          // The supplied R1 URDF's three continuous wheel joints do not declare
          // velocity limits. Do not invent a rated value: require zero targets
          // and a stationary base instead, and disclose this separate guard.
          assert(id==='r1'&&/^wheel_motor_joint[123]$/.test(joint.name),`${id}/${joint.name}: original URDF velocity metadata missing`);
          assert.equal(runtime.dynamics.targets[joint.name],0,'R1 base wheel target changed');
          if(!s.unratedWheelNames.includes(joint.name))s.unratedWheelNames.push(joint.name);
          s.maxUnratedWheelVelocityRadps=Math.max(s.maxUnratedWheelVelocityRadps,Math.abs(velocity));
          assert(Math.abs(velocity)<.001,'R1 base wheels must remain stationary during classification');
        }
        if(id==='g1'&&G1_LEGS.includes(joint.name)) {
          assert.equal(runtime.dynamics.targets[joint.name],0,'G1 leg targets must remain in fixed stance');
          s.maxG1LegAngleRad=Math.max(s.maxG1LegAngleRad,Math.abs(actual));assert(Math.abs(actual)<.001,'G1 legs must not execute a squat');
        }
      }
      if(id==='g1')assert(Math.abs(runtime.data.xpos[stance.pelvis*3+2]-G1_ROOT_HEIGHT)<1e-12,'G1 fixed pelvis height changed');
      if(s.millisecondSamples%10===0)scanContacts(runtime,{requireStable:stableIds.includes(id)});
    }
    if(coordinator.tick%100===0)for(const runtime of stations.values())runtime.syncVisuals();
  }
}

async function initialize() {
  for(const [index,id]of robotIds.entries()) {
    const sourceXml=await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),vfs=new api.MjVFS();let model,data;
    try {
      const urdf=await readFile(new URL(ROBOT_CONFIGS[id].urdf),'utf8'),velocities=new Map();
      for(const [,attributes,body]of urdf.matchAll(/<joint\b([^>]*)>([\s\S]*?)<\/joint>/g)) {
        const name=attributes.match(/\bname="([^"]+)"/)?.[1],velocity=Number(body.match(/<limit\b[^>]*\bvelocity="([^"]+)"/)?.[1]);
        if(name&&Number.isFinite(velocity)&&velocity>0)velocities.set(name,velocity);
      }
      sourceVelocities.set(id,velocities);
      model=api.MjModel.from_xml_string(sourceXml,vfs);data=new api.MjData(model);
      const runtime=createRobotRuntimeFromParts({id,api,model,data,sourceXml,robot:await loaders[id](),offset:[0,0,(1-index)*2.2],cooperativePlanning:true,backgroundPlanning,
        planningTransportFactory:()=>createPlanningWorkerTransport({workerFactory:createNodeWorker,timeoutMs:90000})});
      const advance=runtime.advancePlanning.bind(runtime);
      runtime.advancePlanning=(...parameters)=>{
        const active=runtime.planning,started=performance.now();
        try{return advance(...parameters);}
        finally{if(active){
          // The coordinator can poll several Workers in one drain. Attribute
          // each actual call/result to its station, never the first pending id.
          const elapsed=performance.now()-started,observed=stats[id].planning;
          observed.polls++;observed.hostPollMs+=elapsed;observed.maxHostPollMs=Math.max(observed.maxHostPollMs,elapsed);
          if(!backgroundPlanning){observed.candidateSlices++;observed.accumulatedSegmentMs+=elapsed;observed.maxSliceMs=Math.max(observed.maxSliceMs,elapsed);}
          if(!runtime.planning){
            const prepared=runtime.controller.planningResult;
            if(backgroundPlanning){observed.candidateSlices+=prepared?.steps??0;observed.accumulatedSegmentMs+=prepared?.segmentElapsedMs??0;}
            if(prepared?.ok)observed.completedPreparations++;else observed.failedPreparations++;
            observed.maxPreparationSegmentMs=Math.max(observed.maxPreparationSegmentMs,prepared?.segmentElapsedMs??0);
            if((prepared?.segmentElapsedMs??0)>5000||prepared?.ok===false){
              const detail={id,seed:runtime.controller.classification.seed,selected:runtime.selected,
                completed:runtime.controller.classification.completed,segmentElapsedMs:prepared?.segmentElapsedMs,steps:prepared?.steps,
                ok:prepared?.ok,error:prepared?.error??null};
              observed.slowPreparations.push(detail);console.error(JSON.stringify({planningPreparation:detail}));
            }
          }
        }}
      };
      stations.set(id,runtime);
    } catch(error) {data?.delete();model?.delete();throw error;} finally {vfs.delete();}
  }
}
async function configure(seed) {
  taskWindow=false;
  for(const runtime of stations.values()) {
    assert.equal(typeof runtime.configureTask,'function','Runtime pipe task API unavailable');
    const accepted=runtime.configureTask('pipes',{seed});assert.notEqual(accepted,false,`${runtime.id}: pipe task configuration rejected`);
    assert.notEqual(accepted?.ok,false,accepted?.reason??`${runtime.id}: pipe task configuration rejected`);
    assert.equal(runtime.targets.length,8);assert.equal(runtime.model.nmocap,0);assert.equal(runtime.model.neq,0);
    assert.equal([...runtime.model.jnt_type].filter(type=>type===api.mjtJoint.mjJNT_FREE.value).length,8,
      `${runtime.id}: pipe scene must contain exactly eight independent free bodies, not hidden legacy objects`);
  }
  coordinator=new MultiRobotCoordinator({stations,selectedIds:robotIds});
  stats=Object.fromEntries([...stations].map(([id,runtime])=>[id,freshStats(runtime)]));
  tracked=new Map([...stations].map(([id,runtime])=>[id,objectState(runtime)]));
  const g1=stations.get('g1');stance={pelvis:g1.controller.id('mjOBJ_BODY','pelvis'),
    feet:new Map([...g1.dynamics.footBodyIds].map(body=>[body,[...g1.data.xpos.subarray(body*3,body*3+3)]]))};
  for(const runtime of stations.values()) {
    const original=runtime.controller.setObject.bind(runtime.controller);
    runtime.controller.setObject=(...parameters)=>{
      if(taskWindow||runtime.controller.running){stats[runtime.id].objectEditsDuringTask++;assert.fail(`${runtime.id}: setObject inside a task/extra-support window`);}
      return original(...parameters);
    };
    const rejectPlan=(target,parameters,error)=>{const s=stats[runtime.id];s.planningRejectedCandidates++;
      if(s.planningRejections.length<12)s.planningRejections.push({id:target.id,parameters,
        message:error.message,detail:JSON.parse(JSON.stringify(error.detail??null))});};
    const originalPlan=runtime.controller.plan.bind(runtime.controller);
    runtime.controller.plan=(target,...parameters)=>{
      try{return originalPlan(target,...parameters);}
      catch(error){rejectPlan(target,parameters,error);throw error;}
    };
    const originalCandidates=runtime.controller.planCandidates.bind(runtime.controller);
    runtime.controller.planCandidates=function*(target,...parameters){
      try{return yield* originalCandidates(target,...parameters);}
      catch(error){rejectPlan(target,parameters,error);throw error;}
    };
  }
  await tick(300);
  for(const [id,runtime]of stations)for(const object of tracked.get(id))object.initialY=runtime.controller.objectPose(object.id).position.y;
}
async function start(ids) {
  coordinator.select(ids);
  const before=new Map([...stations].map(([id,runtime])=>[id,{time:runtime.data.time,qpos:[...runtime.data.qpos]}]));
  taskWindow=true;let result=coordinator.start({task:'pipes'});
  assert.equal(result.ok,true,result.reason);
  if(result.preparing)result=await drainPlanning();
  assert.equal(result.ok,true,result.reason);assert.deepEqual(result.runIds,ids);assert.equal(new Set(Object.values(result.startTimes)).size,1);
  for(const [id,runtime]of stations){assert.equal(runtime.data.time,before.get(id).time,'Planning advanced one physical world');
    for(const object of tracked.get(id))assert(arraysEqual([...runtime.data.qpos.subarray(object.q,object.q+7)],before.get(id).qpos.slice(object.q,object.q+7)),`${id}: starting wrote a pipe pose`);}
  return result;
}
function rejectConsumedBatch(label) {
  const before=new Map([...stations].map(([id,runtime])=>[id,{objects:tracked.get(id).map(object=>[...runtime.data.qpos.subarray(object.q,object.q+7)]),
    state:runtime.controller.state,history:[...runtime.controller.history],
    started:runtime.controller.classification.started,completed:runtime.controller.classification.completed,
    verified:runtime.controller.classification.verified,items:JSON.stringify(runtime.controller.classification.items),
    arms:[...runtime.controller.classification.armSequence]}]));
  const rejected=coordinator.start({task:'pipes'});assert.equal(rejected.ok,false,`${label}: consumed batch was started again without reset`);
  assert(!coordinator.running,`${label}: rejected restart left a task running`);
  for(const [id,runtime]of stations) {
    const c=runtime.controller,old=before.get(id);assert.equal(c.classification.started,old.started);
    assert.equal(c.state,old.state,`${label}: rejected restart mutated controller state`);assert.deepEqual(c.history,old.history);
    if(c.classification.started)assert.equal(runtime.moveObject(runtime.targets[0].id,c.objectPose(runtime.targets[0].id).position.clone().add(new THREE.Vector3(.003,0,0))),false,
      `${label}: a consumed classification batch still allowed idle object editing`);
    assert.equal(c.classification.completed,old.completed,`${label}: rejected restart cleared completed count`);
    assert.equal(c.classification.verified,old.verified);assert.equal(JSON.stringify(c.classification.items),old.items);
    assert.deepEqual(c.classification.armSequence,old.arms);
    assert.deepEqual(tracked.get(id).map(object=>[...runtime.data.qpos.subarray(object.q,object.q+7)]),old.objects,`${label}: rejected restart moved a pipe`);
  }
  return rejected;
}
async function finish(ids) {
  for(let n=0;n<360000&&coordinator.running;n++) {
    await tick();if(n>0&&n%60000===0)console.error(JSON.stringify({progressSeconds:coordinator.tick*.001,
      robots:[...stations.values()].map(runtime=>({id:runtime.id,state:runtime.controller.state,arm:runtime.controller.arm,
        selected:runtime.controller.selected,completed:runtime.controller.classification.completed}))}));
  }
  assert.equal(coordinator.state,'complete',JSON.stringify([...stations.values()].map(robotSummary)));
  for(const id of ids) {
    const runtime=stations.get(id),s=stats[id];assert.equal(runtime.controller.state,'complete');assert(!runtime.controller.running);
    const classification=runtime.controller.classification;
    assert.equal(classification.started,true,`${id}: completed task forgot that its batch was consumed`);
    assert.equal(classification.total,8);assert.equal(classification.completed,8);assert.equal(classification.items.length,8);
    assert.equal(classification.verified,true,`${id}: classification completion is not physically verified`);
    assert.equal(s.sequence.length,8,`${id}: each pipe must receive one sequential grasp`);
    assert.equal(new Set(s.sequence.map(item=>item.id)).size,8,`${id}: one pipe was repeated and another omitted`);
    const expected=id==='c5'?Array(8).fill('left'):Array.from({length:8},(_,index)=>index%2?'left':'right');
    assert.deepEqual(s.sequence.map(item=>item.arm),expected,`${id}: incorrect automatic arm alternation`);
    assert.deepEqual(classification.armSequence,expected,`${id}: production sequence disagrees with observed arm alternation`);
    assert.deepEqual(classification.items.map(item=>item.id),s.sequence.map(item=>item.id),`${id}: saved item evidence disagrees with physical observation`);
    for(const item of classification.items) {
      assert(item.dual&&item.lift>.035&&item.tableSupport,`${id}/${item.id}: source item evidence lacks real lift/contact/support`);
      assert(item.placementError<.02&&item.speed<.025,`${id}/${item.id}: source release evidence is not stable within its slot`);
    }
    for(const object of Object.values(s.objects)) {
      assert(object.actualDual,`${id}/${object.id}: no actual dual-finger normal force`);assert(object.maxLiftM>.035,`${id}/${object.id}: no genuine physical lift`);
      assert(object.maxSpeedMps<1,`${id}/${object.id}: pipe was flung`);
      for(const phase of ['approach','descend','closing','lift','transfer','place','release','retreat','settling'])assert(object.states.includes(phase),`${id}/${object.id}: missing actual ${phase} phase`);
      assert.equal(object.arms.length,1,`${id}/${object.id}: arm changed while manipulating one pipe`);
      assert(object.carry.samples>0&&object.carry.phases.lift?.samples>0&&object.carry.phases.transfer?.samples>0,
        `${id}/${object.id}: real transport contact/relative-pose observations absent`);
    }
    assert.equal(s.objectEditsDuringTask,0);
  }
  await tick(3000,{stableIds:ids});
  for(const id of ids) {
    assert.equal(stats[id].extraStableSamples,300);
    const runtime=stations.get(id);runtime.syncVisuals();assert.equal(runtime.robot.parent,null,'Canonical pipe IK tree received a station parent');
    assert.equal(runtime.solidMeshes.length,8);assert.equal(new Set(runtime.targets.map(target=>target.zone)).size,2,'Expected exactly two shared color frames');
    for(const target of runtime.targets) {
      const actual=runtime.controller.objectPose(target.id).position,world=target.visualMesh.getWorldPosition(new THREE.Vector3());
      assert(world.distanceTo(runtime.localToWorld(actual))<1e-10,`${id}/${target.id}: visible pipe is detached from its actual physical pose`);
      assert.equal(target.visualMesh.userData.robotId,id);assert.equal(target.visualMesh.userData.taskId,target.id);
      assert.equal(target.visualMesh.geometry.type,'CylinderGeometry');assert.equal(target.zone.children.length,4);
    }
    stats[id].visualFramesChecked=8;
  }
  for(const id of robotIds.filter(id=>!ids.includes(id))) {
    const runtime=stations.get(id);assert.equal(runtime.controller.state,'idle');assert.equal(runtime.controller.history.length,0);
    assert.equal(stats[id].sequence.length,0);assert(Object.values(stats[id].objects).every(object=>!object.actualDual));
  }
  return {restartAfterComplete:rejectConsumedBatch('completed task')};
}
async function run(name,seed,fn) {
  if(filter&&!name.includes(filter))return;
  const began=performance.now();
  try {await configure(seed);const detail=await fn();results.push({name,seed,pass:true,wallSeconds:(performance.now()-began)/1000,...detail,
    stats:structuredClone(stats),robots:[...stations.values()].map(robotSummary)});console.error(JSON.stringify({scenario:name,seed,pass:true}));}
  catch(error){results.push({name,seed,pass:false,error:error.message,wallSeconds:(performance.now()-began)/1000,
    stats:structuredClone(stats),robots:[...stations.values()].map(robotSummary)});console.error(JSON.stringify({scenario:name,seed,pass:false,error:error.message}));}
  finally{taskWindow=false;coordinator?.stop();}
}

let initializationError=null,layoutResult=null;
try {
  const layoutRows=[],fingerprints={};
  for(const id of robotIds) {
    const distinct=new Set();
    for(let n=0;n<100;n++) {
      const seed=selectedSeed+n,layout=await productionLayout(id,seed),again=await productionLayout(id,seed);
      const row=validateLayout(id,seed,layout);assert.deepEqual(row,validateLayout(id,seed,again),`${id}/${seed}: seed was not reproducible`);
      distinct.add(JSON.stringify(row.map(target=>target.start)));if(n<2)layoutRows.push({id,seed,targets:row});
    }
    assert.equal(distinct.size,100,`${id}: distinct random seeds produce identical layouts`);fingerprints[id]=distinct.size;
  }
  layoutResult={pass:true,seedsPerRobot:100,totalLayouts:300,distinctLayouts:fingerprints,samples:layoutRows};
  if(!staticOnly) {
    await initialize();
    const simultaneousSeeds=quick?[selectedSeed]:[selectedSeed,selectedSeed+37];
    for(const seed of simultaneousSeeds)await run(`three-way-eight-pipe-classification-seed-${seed}`,seed,async()=>{const initial=await start(robotIds),restart=await finish(robotIds);return {start:initial,...restart,selectedIds:robotIds,physicallySortedPipes:24,extraStableSeconds:3};});
    if(!quick) {
      for(const id of robotIds)await run(`single-${id}-eight-pipe-classification`,selectedSeed+71,async()=>{const initial=await start([id]),restart=await finish([id]);return {start:initial,...restart,selectedIds:[id],physicallySortedPipes:8,extraStableSeconds:3};});
      await run('stop-three-way-pipe-task-after-genuine-lifts',selectedSeed+99,async()=>{
        const initial=await start(robotIds),hasLifted=()=>robotIds.every(id=>Object.values(stats[id].objects).some(object=>object.actualDual&&object.maxLiftM>.035));
        for(let n=0;n<60000&&coordinator.running&&!hasLifted();n++)await tick();
        assert(hasLifted(),'Stop fixture never reached genuine lifts at all three stations');
        const before=new Map([...stations].map(([id,runtime])=>[id,{qpos:[...runtime.data.qpos],
          completed:runtime.controller.classification.completed,selected:runtime.controller.selected}]));
        coordinator.stop();assert.equal(coordinator.state,'stopped');assert(!coordinator.running);
        for(const [id,runtime]of stations){assert(!runtime.controller.running);assert.equal(runtime.controller.state,'stopped');
          for(const object of tracked.get(id))assert.deepEqual([...runtime.data.qpos.subarray(object.q,object.q+7)],before.get(id).qpos.slice(object.q,object.q+7),'Stop wrote a task object pose');}
        await tick(3000);
        for(const [id,runtime]of stations){assert.equal(runtime.controller.state,'stopped');assert(!runtime.controller.classification.verified);
          assert.equal(runtime.controller.classification.started,true);
          assert.equal(runtime.controller.classification.completed,before.get(id).completed,'Stopped queue continued processing');
          assert.equal(runtime.controller.selected,before.get(id).selected,'Stopped queue switched to another pipe');assert.equal(stats[id].objectEditsDuringTask,0);}
        const restartAfterStop=rejectConsumedBatch('stopped task');
        return {start:initial,restartAfterStop,selectedIds:robotIds,postStopSeconds:3,physicallySortedPipes:0};
      });
      await run('unreachable-pipe-preparation-aborts-all-attempted-stations',selectedSeed+117,async()=>{
        const g1=stations.get('g1'),layout=g1.controller.classification,first=g1.targets.find(target=>target.id===(layout.queue?.[0]??g1.targets[0].id));
        taskWindow=false;assert.notEqual(g1.controller.setObject(first.id,new THREE.Vector3(1.8,g1.config.tableHeight+first.h,0)),false);
        const state=runtime=>({time:runtime.data.time,qpos:[...runtime.data.qpos],qvel:[...runtime.data.qvel],ctrl:[...runtime.data.ctrl],
          armTargets:Object.fromEntries([...runtime.dynamics.map].filter(([,joint])=>!joint.finger).map(([name])=>[name,runtime.dynamics.targets[name]])),
          history:[...runtime.controller.history],opening:runtime.controller.opening});
        const before=new Map([...stations].map(([id,runtime])=>[id,state(runtime)]));
        const clock=()=>({tick:coordinator.tick,startTick:coordinator.startTick,startTimes:{...coordinator.startTimes},batchNumber:coordinator.batchNumber});
        const coordinatorBefore=clock();
        coordinator.select(robotIds);taskWindow=true;let initial=coordinator.start({task:'pipes'});
        if(initial.preparing)initial=await drainPlanning();
        assert.equal(initial.ok,false,'Unreachable pipe must not enter a partially running batch');assert(!coordinator.running);assert.equal(coordinator.state,'failed');
        const worlds=[];
        for(const [id,runtime]of stations){assert(!runtime.controller.running);assert(!runtime.controller.classification.verified);
          for(const object of tracked.get(id))assert.deepEqual([...runtime.data.qpos.subarray(object.q,object.q+7)],before.get(id).qpos.slice(object.q,object.q+7),'Rejected start teleported a task object');
          const prior=before.get(id),after={...state(runtime),planning:Boolean(runtime.planning),running:Boolean(runtime.controller.running),
            verified:Boolean(runtime.controller.classification.verified),completed:runtime.controller.classification.completed,objectEditsDuringTask:stats[id].objectEditsDuringTask};
          assert.equal(after.time,prior.time,`${id}: rejected preparation advanced a physical clock`);
          for(const name of ['qpos','qvel','ctrl'])assert.deepEqual(after[name],prior[name],`${id}: rejected preparation changed actual ${name}`);
          assert.deepEqual(after.armTargets,prior.armTargets,`${id}: rejected preparation changed arm motor targets`);
          assert.equal(after.planning,false,`${id}: rejected batch left a Worker active`);assert.equal(after.completed,0);assert.equal(after.objectEditsDuringTask,0);
          worlds.push({id,before:prior,after});
        }
        // Faster Workers may install private ready-state metadata before the
        // middle Worker rejects. History is not a physical start/commit signal.
        // Require the stronger exact world/command/clock invariants instead of
        // the old FIFO-polling assumption that C5 history must remain empty.
        const coordinatorAfter={...clock(),pendingStart:coordinator.pendingStart,planning:Boolean(coordinator.planning)};
        assert.deepEqual(clock(),coordinatorBefore,'Rejected preparation committed or advanced the batch');
        assert.equal(coordinatorAfter.startTick,null);assert.deepEqual(coordinatorAfter.startTimes,{});
        assert.equal(coordinatorAfter.pendingStart,null);assert.equal(coordinatorAfter.planning,false);
        const preparationSafety={noPhysicalAdvance:true,noBatchCommit:true,allWorkersCancelled:true,unchangedPhysicsArrays:true,unchangedArmTargets:true,
          coordinatorBefore,coordinatorAfter,worlds};
        const restartAfterFailure=rejectConsumedBatch('failed preparation');
        return {start:initial,restartAfterFailure,selectedIds:robotIds,physicallySortedPipes:0,preparationSafety};
      });
      await run('pipe-task-reset-randomizes-and-explicit-seed-reproduces-all-eight-starts',selectedSeed+139,async()=>{
        const resetSeed=selectedSeed+139,before=new Map([...stations].map(([id,runtime])=>[id,runtime.targets.map(target=>target.start.toArray())]));
        const initial=await start(robotIds);await tick(500);taskWindow=false;coordinator.reset();
        assert.equal(coordinator.state,'idle');assert.equal(coordinator.tick,0);assert(!coordinator.running);
        for(const [id,runtime]of stations) {
          const c=runtime.controller;assert.equal(runtime.data.time,0);assert.equal(c.state,'idle');assert(!c.running);
          assert.equal(c.classification.seed,resetSeed+1,`${id}: reset did not advance the random layout seed`);
          assert(!arraysEqual(runtime.targets.map(target=>target.start.toArray()),before.get(id)),`${id}: reset did not generate new positions`);
          assert.equal(c.classification.total,8);assert.equal(c.classification.completed,0);assert(!c.classification.verified);
          assert.equal(c.classification.started,false,`${id}: reset did not clear the consumed-batch flag`);
          assert.equal(c.classification.items.length,0);assert.equal(c.classification.armSequence.length,0);
          for(const target of runtime.targets)assert(c.objectPose(target.id).position.distanceTo(target.start)<1e-9,`${id}/${target.id}: reset missed its new seeded start`);
        }
        const restartAfterReset=await start(robotIds);assert.equal(restartAfterReset.ok,true);coordinator.stop();taskWindow=false;
        for(const [id,runtime]of stations) {
          runtime.reset({seed:resetSeed});assert.deepEqual(runtime.targets.map(target=>target.start.toArray()),before.get(id),`${id}: explicit seed failed to reproduce the same random layout`);
          assert.equal(runtime.controller.classification.completed,0);assert(!runtime.controller.classification.verified);assert.equal(runtime.controller.classification.started,false);
        }
        return {start:initial,restartAfterReset,selectedIds:robotIds,physicallySortedPipes:0};
      });
    }
  }
} catch(error) {initializationError=error.message;console.error(error);}
finally {for(const runtime of stations.values())runtime.dispose();}

const sourceFrozen=arraysEqual(await hashes(),sourceHashes),expectedCases=staticOnly?0:filter?results.length:quick?1:8;
const pass=!initializationError&&layoutResult?.pass===true&&(staticOnly||results.length>0)&&results.length===expectedCases&&results.every(result=>result.pass)&&sourceFrozen;
const report={pass,startedAt,wallSeconds:(performance.now()-wallStart)/1000,mode,
  cases:results.length,expectedCases,physicallySortedPipes:results.filter(result=>result.pass).reduce((sum,result)=>sum+(result.physicallySortedPipes??0),0),
  clockStepSeconds:.001,fingerToleranceM:fingerTolerance,layout:layoutResult,results,initializationError,sourceFrozen,sourceHashes,
  scope:'Actual original-source URDF dynamics, production pipe task and independent MjModel/MjData per station. No mocap/equality attachments. Every pipe monitored at 1 ms for finite motion/integration; actual contacts monitored at 10 ms. Original R1 finger-contact model only; G1/C5 full robot collisions. Independent stations do not physically collide across worlds.',
  releaseScope:'All eight pipes, not only the last selected pipe, must have actual dual-finger forces >0.015 N per side, actual lift >35 mm, correct color/slot, actual table normal >0.005 N, released fingers and speed <0.025 m/s for a further 3 seconds.'};
report.planningExecution={cooperative:!backgroundPlanning,worker:backgroundPlanning,productionBudgetOverride:false,
  ordinarySegmentBudgetMs:PLANNING_SEGMENT_BUDGET_MS,g1PipeSegmentBudgetMs:G1_PIPE_PLANNING_SEGMENT_BUDGET_MS,
  scope:backgroundPlanning?'Production background controller, real worker_threads transport and shipped worker service/parser/serialization. Only browser file fetch is adapted to Node readFile. All pending workers are polled asynchronously between complete shared 1 ms ticks; actual qpos/qvel/ctrl/time stay unchanged while planning. Worker computation uses the unchanged cooperative candidate iterator and original gates. No task motion executes in a worker.':'Production cooperative candidate iterator; all pending planning is drained between complete shared 1 ms ticks. Every candidate is checked for unchanged shared clock and physical qpos/qvel/ctrl. 累计计算片段耗时 includes elapsed candidate/runtime synchronization time and can include OS preemption; waiting between segments is excluded.'};
report.carryObservationScope='Actual lift/transfer dual-finger retention, longest contact-loss span and object-to-TCP relative pose are aggregated every 10 ms. Saved keyframes are every 100 ms plus every phase/contact change. Retention/drift values are observational, not silently thresholded or inferred from successful endpoints; materially ballistic transport requires explicit review and cannot be advertised as a grasp.';
report.planningObservationScope='Per-station runtime.advancePlanning wrappers attribute each actual Worker poll and completed result to that same runtime. Global drain assertions still require every complete qpos/qvel/ctrl array and every world clock/tick to remain unchanged throughout preparation. Ready-state history is private command metadata, not a committed physical task start.';
await mkdir(new URL('.',new URL(reportPath,root)),{recursive:true});
await writeFile(new URL(reportPath,root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass,mode:report.mode,cases:report.cases,physicallySortedPipes:report.physicallySortedPipes,
  sourceFrozen,report:reportPath,initializationError,failures:results.filter(result=>!result.pass).map(({name,error})=>({name,error}))},null,2));
if(!pass)process.exitCode=1;
