import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController,REST,REST_TORSO,SPREAD,CROUCH,TABLE_HEIGHT} from '../dist/r1/grasp-controller.mjs';
import {tcpPose} from '../dist/r1/grasp-kinematics.mjs';
import {R1SafeDynamics,R1_FINGER_MOTION_PROFILE} from '../dist/multi/r1-safe-dynamics.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';

const root=new URL('../',import.meta.url),results=[],cases=[];
const paths=new Set(['work/test-r1-waist-assist.mjs','work/r1-test-robot.mjs','dist/r1/grasp-controller.mjs',
  'dist/r1/grasp-kinematics.mjs','dist/r1/r1-dynamics.mjs','dist/multi/r1-safe-dynamics.mjs',
  'dist/r1/assets/r1.urdf','dist/r1/models/r1-dynamic-scene.xml']);
const urdf=await readFile(new URL('dist/r1/assets/r1.urdf',root),'utf8');
const xml=await readFile(new URL('dist/r1/models/r1-dynamic-scene.xml',root),'utf8');
const attr=(text,key,fallback='')=>text.match(new RegExp(`\\b${key}="([^"]*)"`))?.[1]??fallback;
const bodyNames=new Set(['base_link','torso_link1','torso_link2','torso_link3','torso_link4','zed_link']);
const visualSpecs=[];
for(const [,attrs,body]of urdf.matchAll(/<link\b([^>]*)>([\s\S]*?)<\/link>/g)){
  const name=attr(attrs,'name');if(!bodyNames.has(name)&&!/^(right|left)_(arm_link[2-6]|gripper|realsense)/.test(name))continue;
  for(const [,visual]of body.matchAll(/<visual\b[^>]*>([\s\S]*?)<\/visual>/g)){
    const meshTag=visual.match(/<mesh\b[^>]*>/)?.[0]??'',filename=attr(meshTag,'filename');if(!filename)continue;
    assert(filename.startsWith('package://r1_v2_1_0/'));
    const path='dist/r1/assets/'+filename.slice('package://r1_v2_1_0/'.length);paths.add(path);
    visualSpecs.push({name,path,meshTag,origin:visual.match(/<origin\b[^>]*>/)?.[0]??''});
  }
}
async function hashes(){return Object.fromEntries(await Promise.all([...paths].sort().map(async path=>[path,
  createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));}
const sourceHashes=await hashes();
for(const spec of visualSpecs){
  const bytes=await readFile(new URL(spec.path,root)),view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),count=view.getUint32(80,true);
  assert.equal(84+count*50,bytes.length,'Expected original binary STL');spec.local=new THREE.Box3();
  const point=new THREE.Vector3();for(let i=0;i<count;i++)for(let j=0;j<3;j++){
    const offset=84+i*50+12+j*12;spec.local.expandByPoint(point.set(view.getFloat32(offset,true),view.getFloat32(offset+4,true),view.getFloat32(offset+8,true)));
  }
  const numbers=(tag,key,fallback)=>attr(tag,key,fallback).trim().split(/\s+/).map(Number);
  spec.originMatrix=new THREE.Matrix4().compose(new THREE.Vector3(...numbers(spec.origin,'xyz','0 0 0')),
    new THREE.Quaternion().setFromEuler(new THREE.Euler(...numbers(spec.origin,'rpy','0 0 0'),'ZYX')),new THREE.Vector3(...numbers(spec.meshTag,'scale','1 1 1')));
}
const tableSpec=xml.match(/<body name="task_table" pos="([^"]+)"[^>]*>\s*<geom[^>]*size="([^"]+)"/);assert(tableSpec);
const tp=tableSpec[1].split(/\s+/).map(Number),th=tableSpec[2].split(/\s+/).map(Number);
const tableBox=new THREE.Box3(new THREE.Vector3(tp[0]-th[0],tp[2]-th[2],-tp[1]-th[1]),new THREE.Vector3(tp[0]+th[0],tp[2]+th[2],-tp[1]+th[1]));
const velocityLimits=Object.fromEntries([...urdf.matchAll(/<joint\b[^>]*name="([^"]+)"[^>]*>([\s\S]*?)<\/joint>/g)]
  .map(([,name,body])=>[name,Number(body.match(/<limit\b[^>]*velocity="([^"]+)"/)?.[1])||Infinity]));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
const robot=await loadTestRobot(),dynamics=new R1SafeDynamics(api,model,data,robot);
const targets=[['red','red_cube',.0225,.14],['blue','blue_block',.019,0],['yellow','yellow_cylinder',.026,-.14]]
  .map(([id,body,h,z])=>({id,body,h,start:new THREE.Vector3(.78,TABLE_HEIGHT+h,z),drop:new THREE.Vector3(.9,TABLE_HEIGHT+h,z),rotation:new THREE.Quaternion()}));
const c=new GraspController(api,model,data,robot,targets,dynamics),blue=targets[1],idle={...REST_TORSO,...REST.right,...REST.left};
const records=visualSpecs.map(spec=>({...spec,link:robot.getObjectByName(spec.name)}));
const arms=records.filter(record=>!bodyNames.has(record.name)),bodies=records.filter(record=>bodyNames.has(record.name));
const id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name),pads=new Set(['left_pad1','left_pad2','right_pad1','right_pad2'].map(name=>id('mjOBJ_BODY',name)));
const identity=new THREE.Matrix4(),gap=(a,b)=>Math.hypot(...['x','y','z'].map(k=>Math.max(0,a.min[k]-b.max[k],b.min[k]-a.max[k])));
const box=(record,reference)=>record.local.clone().applyMatrix4(new THREE.Matrix4().copy(reference).multiply(record.link.matrixWorld).multiply(record.originMatrix));
function reset(x=.78,z=0){blue.start.set(x,TABLE_HEIGHT+blue.h,z);c.reset();for(let i=0;i<300;i++)c.step();}
function checkPlan(){
  const p=c.planData,above=p.start.clone().add(new THREE.Vector3(0,.13,0)),aboveDrop=p.drop.clone().add(new THREE.Vector3(0,.13,0));
  const spans={descend:[above,p.start],lift:[p.start,above],transfer:[above,aboveDrop],place:[aboveDrop,p.drop],retreat:[p.drop,aboveDrop]};
  let samples=0,maxPositionError=0,maxAngleError=0,maxJointJump=0;
  try{for(const [phase,[from,to]]of Object.entries(spans))for(let index=0;index<p[phase].length;index++){
    const pose=p[phase][index];robot.setJointValues(pose);robot.updateMatrixWorld(true);
    const actual=tcpPose(robot,c.arm),target=from.clone().lerp(to,index/(p[phase].length-1));
    const pe=actual.position.distanceTo(target),ae=actual.quaternion.angleTo(p.orientation);
    assert(pe<=.003&&ae<=.035,'All natural path samples must satisfy original position AND orientation bounds');
    assert(Math.abs(pose.torso_joint4-p.waistAssist.actualYaw)<1e-12,'No extra carrying-time waist turn');
    for(const [name,value]of Object.entries(pose)){const limit=robot.joints[name]?.limit;if(limit)assert(value>=limit.lower&&value<=limit.upper);}
    if(index){const previous=p[phase][index-1],jump=Math.max(...Object.keys(pose).map(name=>Math.abs(pose[name]-(previous[name]??pose[name]))));assert(jump<=.30);maxJointJump=Math.max(maxJointJump,jump);}
    samples++;maxPositionError=Math.max(maxPositionError,pe);maxAngleError=Math.max(maxAngleError,ae);
  }}finally{dynamics.syncRobot();}
  return{samples,maxPositionErrorM:maxPositionError,maxOrientationErrorRad:maxAngleError,maxAdjacentJointJumpRad:maxJointJump};
}
function geometry(stats){
  robot.updateMatrixWorld(true);stats.geometrySamples++;
  for(const a of arms){
    const tabletopGap=gap(box(a,identity),tableBox);assert(tabletopGap>0,`Source STL/table conservative overlap: ${a.name}`);
    stats.minimumTableGapM=Math.min(stats.minimumTableGapM,tabletopGap);
    for(const b of bodies){
      // Rotating both separated shapes enlarges their world-aligned boxes.
      // Use each source body frame instead: both transformed boxes still
      // conservatively contain every original STL vertex, without that false
      // world-axis overlap during a common waist rotation.
      const reference=b.link.matrixWorld.clone().invert(),distance=gap(box(a,reference),box(b,reference));
      assert(distance>0,`Source STL/body conservative overlap: ${a.name}/${b.name}`);stats.minimumBodyGapM=Math.min(stats.minimumBodyGapM,distance);
    }
  }
}
function objectState(){const b=c.bodies.get('blue'),joint=model.body_jntadr[b],q=model.jnt_qposadr[joint],d=model.jnt_dofadr[joint];return{position:[...data.qpos.subarray(q,q+3)],velocity:[...data.qvel.subarray(d,d+3)]};}
let continuationPose;
try{
  reset();assert(c.start('right','blue'));assert.equal(c.planData.waistAssist.legacyPriority,false);assert(c.planData.waistAssist.validNaturalRoutes>=1);
  assert(c.planData.waistAssist.used,'Natural wrist margin can choose waist assistance before a position-only failure');
  assert(c.planData.naturalSelection.maxWristPitch<=c.planData.naturalSelection.wristPitchLimit);
  assert(c.planData.naturalSelection.limits.right_arm_joint4.upper<0,'The old opposite forearm-roll branch is not a natural solution');c.stop();c.reset();
  results.push({name:'Natural continuous branch and wrist margin replace historical 25-branch priority',pass:true});
  for(const arm of ['right','left']){
    reset();c.arm=arm;
    const before=[...data.qpos];assert(c.start(arm,'blue'));assert.deepEqual([...data.qpos],before);
    const assist={...c.planData.waistAssist};assert(assist.used&&!assist.legacyPriority&&assist.sharedPickPlaceYaw);assert(Math.abs(assist.actualYaw)<=1);
    assert(assist.candidates.some(value=>Math.abs(value-assist.actualYaw)<1e-12),'Selected yaw is derived in the true source waist frame');
    const path=checkPlan(),stats={millisecondSamples:0,contactScans:0,maxEffortRatio:0,maxActualVelocityRatio:0,maxFingerRangeExcessM:0,maxIllegalPadPenetrationM:0,
      maxObjectTranslationResidualM:0,actualDualForce:false,geometrySamples:0,minimumBodyGapM:Infinity,minimumTableGapM:Infinity,maxCarryingWaistTrackingErrorRad:0};
    let alignPose=null;
    for(let index=0;index<45000&&c.running;index++){
      const previous=objectState();c.step();stats.millisecondSamples++;
      const now=objectState(),displacement=Math.hypot(...now.position.map((value,i)=>value-previous.position[i]));
      const residual=Math.max(0,displacement-.001*Math.max(Math.hypot(...previous.velocity),Math.hypot(...now.velocity)));
      assert(residual<1e-5,'Objects must not jump/attach during task motion');stats.maxObjectTranslationResidualM=Math.max(stats.maxObjectTranslationResidualM,residual);
      for(const [name,joint]of dynamics.map){
        const value=data.qpos[joint.q],velocity=data.qvel[joint.d],effort=Math.abs(data.qfrc_actuator[joint.d]);assert([value,velocity,effort].every(Number.isFinite));assert(effort<=joint.effort+1e-6);stats.maxEffortRatio=Math.max(stats.maxEffortRatio,effort/joint.effort);
        if(joint.finger){const excess=Math.max(0,joint.lower-value,value-joint.upper);assert(excess<=.001);stats.maxFingerRangeExcessM=Math.max(stats.maxFingerRangeExcessM,excess);assert(Math.abs(dynamics.targetVelocity[name])<=R1_FINGER_MOTION_PROFILE.commandVelocityMps+1e-10);}
        else{assert(value>=joint.lower-.001&&value<=joint.upper+.001);const ratio=Math.abs(velocity)/velocityLimits[name];assert(ratio<=1);stats.maxActualVelocityRatio=Math.max(stats.maxActualVelocityRatio,ratio);}
      }
      if(c.state==='crouch')assert.equal(c.pose.torso_joint4,0,'Crouch must finish before horizontal alignment');
      if(c.state==='align'){
        alignPose??={...c.pose};for(const [name,value]of Object.entries(c.pose))if(name!=='torso_joint4')assert(Math.abs(value-alignPose[name])<1e-12,'Alignment must turn only the waist');
      }
      if(['lift','transfer','place'].includes(c.state)){
        assert(Math.abs(c.pose.torso_joint4-assist.actualYaw)<1e-12);const waist=dynamics.map.get('torso_joint4');
        stats.maxCarryingWaistTrackingErrorRad=Math.max(stats.maxCarryingWaistTrackingErrorRad,Math.abs(data.qpos[waist.q]-assist.actualYaw));
      }
      if(index%10===0){const m=c.measure();if(m.count===2&&m.forces.every(force=>force>.015))stats.actualDualForce=true;
        const contacts=data.contact;try{for(let i=0;i<Math.min(data.ncon,contacts.size());i++){const contact=contacts.get(i);try{
          if(pads.has(model.geom_bodyid[contact.geom1])||pads.has(model.geom_bodyid[contact.geom2])){const penetration=Math.max(0,-contact.dist);assert(penetration<=.001);stats.maxIllegalPadPenetrationM=Math.max(stats.maxIllegalPadPenetrationM,penetration);}
        }finally{contact.delete();}}}finally{contacts.delete();}stats.contactScans++;}
      if(index%20===0&&['crouch','align','align_clear'].includes(c.state))geometry(stats);
    }
    assert.equal(c.state,'complete',c.message);assert(c.evidence.dual&&stats.actualDualForce&&c.evidence.lift>.035&&c.evidence.maxSpeed<1);
    const measurement=c.measure();assert(measurement.table&&measurement.support&&measurement.distance<.02);assert(stats.maxCarryingWaistTrackingErrorRad<.01);
    const order=['spread','crouch','align','approach'].map(phase=>c.history.indexOf(phase));assert(order.every((value,index)=>value>=0&&(!index||value>order[index-1])));
    if(arm==='right')continuationPose={...c.planData.retreat.at(-1)};
    cases.push({arm,pass:true,assist,path,stats,evidence:{...c.evidence},placementErrorM:measurement.distance,history:[...c.history]});
    results.push({name:`${arm}: source-FK waist alignment precedes naturally continuous real grasp/release`,pass:true});
  }
  reset();assert(c.start('right','blue'));for(let i=0;i<10000&&(c.state!=='align'||c.elapsed<.35);i++)c.step();assert.equal(c.state,'align');
  c.stop();const stopped={...c.pose};for(let i=0;i<1000;i++)c.step();assert.equal(c.state,'stopped');assert.deepEqual(c.pose,stopped);c.reset();
  assert.equal(c.state,'idle');for(const [name,value]of Object.entries(idle)){const joint=dynamics.map.get(name);assert(Math.abs(data.qpos[joint.q]-value)<1e-12);}
  results.push({name:'Stopping during actual waist alignment holds targets and reset restores camera-facing natural REST',pass:true});

  // An explicitly initialized released-object continuation fixture exercises
  // the stack-only clear-before-realignment branch, without pretending this
  // short fixture is a complete three-item physical stack test.
  reset();assert(continuationPose);const previousYaw=continuationPose.torso_joint4;
  c.applyPose(continuationPose);dynamics.reset({...continuationPose,...Object.fromEntries(['right','left'].flatMap(side=>[1,2].map(i=>[`${side}_gripper_finger_joint${i}`,.05])))});
  assert(c.prepareItem('left','blue',true));assert.equal(c.state,'align_clear');let sawAlign=false;
  const continuationGeometry={geometrySamples:0,minimumBodyGapM:Infinity,minimumTableGapM:Infinity};
  for(let i=0;i<8000&&c.state!=='approach';i++){
    c.step();if(c.state==='align_clear')assert(Math.abs(c.pose.torso_joint4-previousYaw)<1e-12);if(c.state==='align')sawAlign=true;
    if(i%20===0&&['align_clear','align'].includes(c.state))geometry(continuationGeometry);
  }
  assert(sawAlign&&c.state==='approach');c.stop();c.reset();
  results.push({name:'Changed-yaw continuing-item entry retracts at old yaw before a waist-only alignment',pass:true,geometry:continuationGeometry});
  blue.start.x=1.8;c.reset();const before=[...data.qpos];assert.equal(c.start('right','blue'),false);assert.equal(c.state,'failed');assert.deepEqual([...data.qpos],before);assert(c.planError);
  results.push({name:'Unreachable full pick/place paths are rejected without moving physical state',pass:true});
}finally{data.delete();model.delete();}
const sourceFrozen=JSON.stringify(await hashes())===JSON.stringify(sourceHashes);assert(sourceFrozen);
console.log(JSON.stringify({pass:true,checks:results.length,results,cases,sourceFrozen,sourceHashes,
  collisionScope:'Original R1 fixed-size fingertip contacts only, with unchanged 1 mm legacy numerical guard. Source shell/self collision geoms remain absent. Source-STL conservative body-frame and world-table boxes are sampled every 20 ms during crouch/waist alignment; this is not full-body collision dynamics or arbitrary-workspace certification.',
  limits:{positionErrorM:.003,orientationErrorRad:.035,naturalAdjacentJointJumpRad:.30,naturalWaistYawAbsRad:1,requiredLiftM:.035,finalPlacementErrorM:.02,fingerStrokeToleranceM:.001,illegalFingertipPenetrationM:.001},
  placementStrategy:'One common waist yaw must solve the complete original pick-and-place trajectory; no carrying-time extra waist rotation.'},null,2));
