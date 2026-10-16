import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController,REST,REST_TORSO,SPREAD,CROUCH,TABLE_HEIGHT} from '../dist/r1/grasp-controller.mjs';
import {solvePose,tcpPose,naturalArmLimits} from '../dist/r1/grasp-kinematics.mjs';
import {R1SafeDynamics} from '../dist/multi/r1-safe-dynamics.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';
import {loadR1SourceGeometry} from './r1-source-geometry.mjs';

const root=new URL('../',import.meta.url),robot=await loadTestRobot(),geometry=await loadR1SourceGeometry(robot),results=[],cases=[];
const paths=[...geometry.files,'work/test-r1-natural-grasp.mjs','work/r1-test-robot.mjs','dist/r1/grasp-controller.mjs','dist/r1/grasp-kinematics.mjs','dist/r1/r1-dynamics.mjs','dist/multi/r1-safe-dynamics.mjs'];
const hashes=async()=>Object.fromEntries(await Promise.all(paths.sort().map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')]))),sourceHashes=await hashes();
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(await readFile(new URL('dist/r1/models/r1-dynamic-scene.xml',root),'utf8'),vfs),data=new api.MjData(model);vfs.delete();
const d=new R1SafeDynamics(api,model,data,robot),targets=[['red','red_cube',.0225,.14],['blue','blue_block',.019,0],['yellow','yellow_cylinder',.026,-.14]]
  .map(([id,body,h,z])=>({id,body,h,start:new THREE.Vector3(.78,TABLE_HEIGHT+h,z),drop:new THREE.Vector3(.9,TABLE_HEIGHT+h,z),rotation:new THREE.Quaternion()})),c=new GraspController(api,model,data,robot,targets,d);
const idle={...REST_TORSO,...REST.right,...REST.left};
const armNames=arm=>Array.from({length:6},(_,i)=>`${arm}_arm_joint${i+1}`);
const optical=arm=>new THREE.Vector3(0,0,1).applyQuaternion(robot.getObjectByName(`${arm}_realsense_link`).getWorldQuaternion(new THREE.Quaternion()));
function clearTable(phase,stats){
  const hits=geometry.tableIntersections();assert.deepEqual(hits,[],`Original source STL/table intersects at ${phase}: ${JSON.stringify(hits)}`);
  const bodyGap=geometry.conservativeBodyGap();assert(bodyGap>0,`Source non-mount arm/body conservative boxes overlap at ${phase}`);
  stats.geometrySamples++;stats.minimumConservativeBodyGapM=Math.min(stats.minimumConservativeBodyGapM,bodyGap);
}
function staticPlan(arm,phaseLabel){
  const p=c.planData,s=arm==='right'?1:-1,names=armNames(arm),limits=naturalArmLimits(arm,{wristPitch:p.naturalSelection.wristPitchLimit});
  assert.equal(p.waistAssist.legacyPriority,false);assert(p.waistAssist.validNaturalRoutes>=1);assert(Math.abs(p.waistAssist.actualYaw)<=1);
  const stats={samples:0,geometrySamples:0,minimumConservativeBodyGapM:Infinity,maxPositionErrorM:0,maxAngleErrorRad:0,maxAdjacentJointJumpRad:0};
  const above=p.start.clone().add(new THREE.Vector3(0,.13,0)),aboveDrop=p.drop.clone().add(new THREE.Vector3(0,.13,0));
  const spans={descend:[above,p.start],lift:[p.start,above],transfer:[above,aboveDrop],place:[aboveDrop,p.drop],retreat:[p.drop,aboveDrop]};
  const prepare=[idle,{...REST_TORSO,...SPREAD.right,...SPREAD.left},{...CROUCH,...SPREAD.right,...SPREAD.left},{...c.crouch,...SPREAD.right,...SPREAD.left},p.approach];
  for(let segment=1;segment<prepare.length;segment++)for(let i=0;i<=120;i++){
    const from=prepare[segment-1],to=prepare[segment],pose=Object.fromEntries(Object.keys(to).map(n=>[n,THREE.MathUtils.lerp(from[n],to[n],i/120)]));
    robot.setJointValues(pose);clearTable(`${phaseLabel}/prepare${segment}/${i}`,stats);
    // REST/SPREAD retain the same native axial branch instead of unwinding
    // the forearm by ~160 degrees before starting a grasp.
    assert(pose[`${arm}_arm_joint4`]*s<0);
  }
  let previous=p.approach;
  for(const [phase,[from,to]]of Object.entries(spans)){
    for(let i=0;i<p[phase].length;i++){
      const pose=p[phase][i];robot.setJointValues(pose);robot.updateMatrixWorld(true);
      const actual=tcpPose(robot,arm),target=from.clone().lerp(to,i/(p[phase].length-1)),pe=actual.position.distanceTo(target),ae=actual.quaternion.angleTo(p.orientation);
      assert(pe<=.003&&ae<=.035);assert.equal(pose.torso_joint4,c.crouch.torso_joint4,'Every complete pick/place path retains its selected waist');
      const jump=Math.max(...names.map(n=>Math.abs(pose[n]-previous[n])));assert(jump<=.30);previous=pose;
      for(const n of names){assert(pose[n]>=limits[n].lower&&pose[n]<=limits[n].upper);const original=robot.joints[n].limit;assert(pose[n]>=original.lower&&pose[n]<=original.upper);}
      const front=optical(arm);assert(front.x>0&&front.y<0,'The actual source camera lens must look front/down, not up from below the wrist');
      const jaw=new THREE.Vector3(0,1,0).applyQuaternion(actual.quaternion),desiredJaw=new THREE.Vector3(0,1,0).applyQuaternion(p.orientation);
      assert(Math.abs(desiredJaw.y)<1e-10,'Requested original parallel-jaw closing axis is horizontal');
      assert(Math.abs(jaw.y)<=Math.sin(ae)+1e-8,'Actual jaw respects the unchanged orientation-error bound');
      clearTable(`${phaseLabel}/${phase}/${i}`,stats);stats.samples++;stats.maxPositionErrorM=Math.max(stats.maxPositionErrorM,pe);stats.maxAngleErrorRad=Math.max(stats.maxAngleErrorRad,ae);stats.maxAdjacentJointJumpRad=Math.max(stats.maxAdjacentJointJumpRad,jump);
    }
  }
  d.syncRobot();return stats;
}
try{
  // A full controller reference must never overwrite the already selected
  // torso or non-executing arm inside a six-joint IK solve.
  const sourceBefore=JSON.stringify([...data.qpos]);
  for(const arm of ['right','left']){
    robot.setJointValues({...CROUCH,torso_joint4:.27,...SPREAD.right,...SPREAD.left});robot.updateMatrixWorld(true);
    const waistQuaternion=robot.joints.torso_joint4.quaternion.toArray();
    const desired=tcpPose(robot,arm),solved=solvePose(robot,arm,desired.position,desired.quaternion,idle,300);
    assert.deepEqual(Object.keys(solved.pose).sort(),armNames(arm).sort());assert.deepEqual(robot.joints.torso_joint4.quaternion.toArray(),waistQuaternion);
  }
  assert.equal(JSON.stringify([...data.qpos]),sourceBefore);d.syncRobot();results.push({name:'IK trims full-pose seeds to exactly six executing-arm joints, preserving torso and real state',pass:true});
  for(const arm of ['right','left'])for(const id of ['red','blue','yellow']){
    c.reset();for(let i=0;i<300;i++)c.step();assert(c.start(arm,id),`${arm}/${id}: ${c.message}`);
    const stats=staticPlan(arm,`${arm}/${id}`);cases.push({arm,id,type:'source-planned-route',pass:true,assist:c.planData.waistAssist,natural:c.planData.naturalSelection,stats});c.stop();
  }
  results.push({name:'Both arms: all three default object routes and preparation are naturally continuous and original-STL/table clear',pass:true});
  for(const arm of ['right','left']){
    c.reset();for(let i=0;i<300;i++)c.step();assert(c.start(arm,'blue'));
    const stats={geometrySamples:0,minimumConservativeBodyGapM:Infinity,maxEffortRatio:0,maxJointErrorRad:0,contactOptical:[]};
    for(let i=0;i<45000&&c.running;i++){
      c.step();for(const j of d.map.values()){const effort=Math.abs(data.qfrc_actuator[j.d])/j.effort;assert(effort<=1+1e-7);stats.maxEffortRatio=Math.max(stats.maxEffortRatio,effort);if(!j.finger)stats.maxJointErrorRad=Math.max(stats.maxJointErrorRad,Math.abs(d.targets[j.name]-data.qpos[j.q]));}
      if(i%20===0){clearTable(`${arm}/actual/${c.state}/${i}`,stats);if(c.state==='closing'){const f=optical(arm);assert(f.x>0&&f.y<0);stats.contactOptical.push(f.toArray());}}
    }
    assert.equal(c.state,'complete',c.message);assert(c.evidence.dual&&c.evidence.lift>.035&&c.evidence.maxSpeed<1);assert(c.measure().table&&c.measure().distance<.02);assert(stats.contactOptical.length>0);assert(stats.maxJointErrorRad<.01);
    cases.push({arm,id:'blue',type:'complete-real-MuJoCo-route',pass:true,stats,evidence:{...c.evidence},placementErrorM:c.measure().distance,history:[...c.history]});
  }
  results.push({name:'Both arms: complete real blue grasp/release with original STL triangles checked every 20 ms',pass:true});
  for(const arm of ['right','left'])for(const index of [1,2]){
    c.reset();for(let i=0;i<300;i++)c.step();
    // Explicit initial fixtures for the additional stacking heights, not a
    // claim that this short geometric check executed a physical full stack.
    c.setObject('blue',new THREE.Vector3(.9,TABLE_HEIGHT+.019,0));
    if(index===2)c.setObject('red',new THREE.Vector3(.9,TABLE_HEIGHT+.019*2+.0225,0));
    c.task='stack';c.itemIndex=index;assert(c.prepareItem(arm,index===1?'red':'yellow',false),c.message);
    const stats=staticPlan(arm,`${arm}/stack-fixture-${index}`);cases.push({arm,index,type:'explicit-stack-height-source-geometry',pass:true,stats});c.stop();
  }
  results.push({name:'Both arms: original-source table clearance for red/yellow stack-height preparation and complete planned routes',pass:true});
}finally{data.delete();model.delete();}
const sourceFrozen=JSON.stringify(await hashes())===JSON.stringify(sourceHashes);assert(sourceFrozen);
console.log(JSON.stringify({pass:true,checks:results.length,results,cases,sourceFrozen,sourceHashes,
  collisionScope:'Original MuJoCo fingertip contact dynamics only: shell/self collision geoms remain absent. Independent unchanged source-STL triangle-vs-original-table SAT/contained-vertex checks sample planned paths and actual blue trajectories every 20 ms. Body-frame boxes are conservative gap witnesses, not full-body contact physics or arbitrary-workspace certification.',
  opticalScope:'Camera direction uses original Realsense source-frame +Z inferred from STL lens geometry, not calibrated optical intrinsics/extrinsics.',
  limits:{positionErrorM:.003,orientationErrorRad:.035,adjacentJointJumpRad:.30,toolDownTiltRad:Math.PI/9,requiredLiftM:.035,placementErrorM:.02,originalEffortRatio:1}},null,2));
