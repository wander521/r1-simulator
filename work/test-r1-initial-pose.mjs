import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController,REST,REST_TORSO,SPREAD,TABLE_HEIGHT} from '../dist/r1/grasp-controller.mjs';
import {R1Dynamics} from '../dist/r1/r1-dynamics.mjs';
import {R1SafeDynamics,R1_FINGER_MOTION_PROFILE} from '../dist/multi/r1-safe-dynamics.mjs';
import {R1_WRIST_OPTICS,createWristVision} from '../dist/r1/r1-workcell.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';

// Presentation-pose regression, with unchanged original motors and limits.
// R1's source MJCF intentionally has no shell/self-collision geoms. The STL
// check below is a separate conservative, sampled non-overlap check, not an
// assertion that the physical scene implements whole-body collision response.
const root=new URL('../',import.meta.url),results=[],sourcePaths=new Set([
  'work/test-r1-initial-pose.mjs','work/r1-test-robot.mjs',
  'dist/r1/grasp-controller.mjs','dist/r1/grasp-kinematics.mjs',
  'dist/r1/r1-dynamics.mjs','dist/multi/r1-safe-dynamics.mjs','dist/r1/r1-workcell.mjs',
  'dist/r1/assets/r1.urdf','dist/r1/models/r1-dynamic-scene.xml',
]);
const attribute=(text,key,fallback='')=>text.match(new RegExp(`\\b${key}="([^"]*)"`))?.[1]??fallback;
const urdf=await readFile(new URL('dist/r1/assets/r1.urdf',root),'utf8');
const scene=await readFile(new URL('dist/r1/models/r1-dynamic-scene.xml',root),'utf8');
const bodyNames=new Set(['base_link','torso_link1','torso_link2','torso_link3','torso_link4','zed_link']);
const relevant=name=>bodyNames.has(name)||/^(right|left)_(arm_link[2-6]|gripper|realsense)/.test(name);
const visuals=[];
for(const [,attrs,body]of urdf.matchAll(/<link\b([^>]*)>([\s\S]*?)<\/link>/g)){
  const name=attribute(attrs,'name');if(!relevant(name))continue;
  for(const [,visual]of body.matchAll(/<visual\b[^>]*>([\s\S]*?)<\/visual>/g)){
    const meshTag=visual.match(/<mesh\b[^>]*>/)?.[0]??'',file=attribute(meshTag,'filename');
    if(!file)continue;
    assert(file.startsWith('package://r1_v2_1_0/'));
    const path='dist/r1/assets/'+file.slice('package://r1_v2_1_0/'.length);
    sourcePaths.add(path);visuals.push({name,path,meshTag,origin:visual.match(/<origin\b[^>]*>/)?.[0]??''});
  }
}
async function hashes(){return Object.fromEntries(await Promise.all([...sourcePaths].sort().map(async path=>[
  path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));}
const sourceHashes=await hashes();
function stlBounds(bytes){
  const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),count=bytes.length>=84?view.getUint32(80,true):0;
  const bounds=new THREE.Box3(),p=new THREE.Vector3();let vertices=0;
  if(84+count*50===bytes.length){
    for(let triangle=0;triangle<count;triangle++)for(let vertex=0;vertex<3;vertex++){
      const offset=84+triangle*50+12+vertex*12;
      bounds.expandByPoint(p.set(view.getFloat32(offset,true),view.getFloat32(offset+4,true),view.getFloat32(offset+8,true)));vertices++;
    }
  }else for(const match of bytes.toString('utf8').matchAll(/vertex\s+(\S+)\s+(\S+)\s+(\S+)/g)){
    bounds.expandByPoint(p.fromArray(match.slice(1).map(Number)));vertices++;
  }
  assert(vertices>0,'Source STL contains no vertices');return bounds;
}
for(const visual of visuals){
  const xyz=attribute(visual.origin,'xyz','0 0 0').trim().split(/\s+/).map(Number);
  const rpy=attribute(visual.origin,'rpy','0 0 0').trim().split(/\s+/).map(Number);
  const scale=attribute(visual.meshTag,'scale','1 1 1').trim().split(/\s+/).map(Number);
  visual.local=stlBounds(await readFile(new URL(visual.path,root)));
  visual.originMatrix=new THREE.Matrix4().compose(new THREE.Vector3().fromArray(xyz),
    new THREE.Quaternion().setFromEuler(new THREE.Euler(...rpy,'ZYX')),new THREE.Vector3().fromArray(scale));
}
const velocityLimits=Object.fromEntries([...urdf.matchAll(/<joint\b[^>]*name="([^"]+)"[^>]*>([\s\S]*?)<\/joint>/g)]
  .map(([,name,body])=>[name,Number(body.match(/<limit\b[^>]*velocity="([^"]+)"/)?.[1])||Infinity]));
const idle={...REST_TORSO,...REST.right,...REST.left},spread={...REST_TORSO,...SPREAD.right,...SPREAD.left};
const open=Object.fromEntries(['right','left'].flatMap(arm=>[1,2].map(i=>[`${arm}_gripper_finger_joint${i}`,.05])));
const poses={idle:{...idle,...open},spread:{...spread,...open}};
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const runs=[];
for(const [name,Dynamics]of [['original-single-robot',R1Dynamics],['conservative-multi-robot',R1SafeDynamics]]){
  const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(scene,vfs),data=new api.MjData(model);vfs.delete();
  try{
    const robot=await loadTestRobot(),dynamics=new Dynamics(api,model,data,robot);
    const feedCamera=new THREE.PerspectiveCamera(),wristVision=createWristVision({THREE,camera:feedCamera});
    const targets=[['red','red_cube',.0225,.14],['blue','blue_block',.019,0],['yellow','yellow_cylinder',.026,-.14]]
      .map(([id,body,h,z])=>({id,body,h,start:new THREE.Vector3(.78,TABLE_HEIGHT+h,z),drop:new THREE.Vector3(.9,TABLE_HEIGHT+h,z),rotation:new THREE.Quaternion()}));
    const controller=new GraspController(api,model,data,robot,targets,dynamics);
    const stats={millisecondSamples:0,contactScans:0,maxStaticErrorRad:0,maxMovingErrorRad:0,maxEffortRatio:0,
      maxActualVelocityRatio:0,maxArmJointRangeExcess:0,maxFingerRangeExcess:0,maxIllegalPadPenetrationM:0,maxFKPositionErrorM:0};
    const geometry={samples:0,minimumAABBSeparationM:Infinity,closestPair:null,
      scope:'Source STL transformed AABBs; non-mount arm parts versus torso/base and opposite arm; 20 ms samples during standing REST/SPREAD only. No whole-body MuJoCo collision geoms or arbitrary-trajectory guarantee.'};
    const records=visuals.map(v=>({...v,link:robot.getObjectByName(v.name),world:new THREE.Box3()}));
    const id=(type,label)=>api.mj_name2id(model,api.mjtObj[type].value,label);
    const pads=new Set(['right_pad1','right_pad2','left_pad1','left_pad2'].map(n=>id('mjOBJ_BODY',n)));
    function initial(){
      assert.equal(controller.state,'idle');assert.equal(controller.running,false);assert.deepEqual(controller.pose,idle);
      for(const [jointName,joint]of dynamics.map){
        const expected=poses.idle[jointName]??0;
        assert(Math.abs(data.qpos[joint.q]-expected)<1e-12,`${name}/${jointName}: reset qpos is not source REST`);
        assert.equal(data.qvel[joint.d],0);
        assert(expected>=joint.lower&&expected<=joint.upper,`${jointName}: source joint range exceeded`);
        const expectedCtrl=joint.finger?expected:joint.kp*expected+data.qfrc_bias[joint.d];
        assert(Math.abs(data.ctrl[joint.actuator]-expectedCtrl)<1e-10,`${name}/${jointName}: ctrl not initialized from actual source REST`);
      }
    }
    function points(){
      const result={};
      for(const arm of ['right','left']){
        const suffixes=['arm_link2','arm_link3','arm_link4','arm_link5','gripper_link','realsense_link'];
        const positions=suffixes.map(suffix=>robot.getObjectByName(`${arm}_${suffix}`).getWorldPosition(new THREE.Vector3()));
        const down=new THREE.Vector3(0,-1,0),upperAngle=positions[1].clone().sub(positions[0]).angleTo(down)*180/Math.PI;
        const lowerAngle=positions[3].clone().sub(positions[2]).angleTo(down)*180/Math.PI;
        assert(upperAngle<10&&lowerAngle<5,`${arm}: both upper and lower arm must point down, not merely the gripper`);
        assert(positions[0].y-positions[1].y>.3&&positions[1].y-positions[3].y>.2);
        assert(Math.abs(positions[4].z)>.15&&Math.abs(positions[4].z)<.35,'Hand should rest beside torso');
        assert((arm==='right'?1:-1)*positions[4].z>0,'Hands crossed sides');
        for(const [index,suffix]of suffixes.entries()){
          const body=id('mjOBJ_BODY',`${arm}_${suffix}`);
          stats.maxFKPositionErrorM=Math.max(stats.maxFKPositionErrorM,positions[index].distanceTo(new THREE.Vector3(data.xpos[body*3],data.xpos[body*3+2],-data.xpos[body*3+1])));
        }
        // Measure the real composite wrist orientation relative to the
        // pre-roll forearm frame. Pitch and roll do not commute: simply
        // adding the two joint commands would not prove the resulting twist.
        const forearmQuaternion=robot.getObjectByName(`${arm}_arm_link3`).getWorldQuaternion(new THREE.Quaternion());
        const handQuaternion=robot.getObjectByName(`${arm}_gripper_link`).getWorldQuaternion(new THREE.Quaternion());
        const relative=forearmQuaternion.clone().invert().multiply(handQuaternion);
        const twist=2*Math.atan2(relative.x,relative.w),wrappedTwist=Math.atan2(Math.sin(twist),Math.cos(twist));
        assert(Math.abs(Math.abs(wrappedTwist)-Math.PI)<1e-10,'Real wrist quaternion must retain a half-turn longitudinal twist');
        const cameraLink=robot.getObjectByName(`${arm}_realsense_link`),cameraQuaternion=cameraLink.getWorldQuaternion(new THREE.Quaternion());
        const cameraFront=new THREE.Vector3(0,0,1).applyQuaternion(cameraQuaternion).normalize();
        assert(positions[5].x>0&&cameraFront.x>0,'Native source camera must sit in front and its STL +Z face must look forward');
        const visual=records.find(record=>record.name===`${arm}_realsense_link`);
        assert(visual,'Original source camera STL must be present');
        const cameraBounds=visual.local.clone().applyMatrix4(new THREE.Matrix4().multiplyMatrices(cameraLink.matrixWorld,visual.originMatrix));
        assert(cameraBounds.min.x>0,'The whole conservative source-camera box must be on the front side');
        assert.deepEqual(R1_WRIST_OPTICS.forwardLocal,[0,0,1]);assert.equal(R1_WRIST_OPTICS.calibrated,false);
        assert(wristVision.update(cameraLink));assert.equal(wristVision.model,cameraLink,'No replacement housing or remounted model');
        const lensCenter=cameraLink.localToWorld(new THREE.Vector3(...R1_WRIST_OPTICS.lensLocal));
        assert(feedCamera.position.distanceTo(lensCenter)<1e-12,'Simulated optical center must follow the native frame');
        assert(feedCamera.getWorldDirection(new THREE.Vector3()).distanceTo(cameraFront)<1e-12,'Feed must follow the real STL +Z face, not an invented front-facing extrinsic');
        const tableMatch=scene.match(/<body name="task_table" pos="([^"]+)"[^>]*>\s*<geom[^>]*size="([^"]+)"/);
        assert(tableMatch);const tablePos=tableMatch[1].split(/\s+/).map(Number),tableHalf=tableMatch[2].split(/\s+/).map(Number);
        const tableY=tablePos[2]+tableHalf[2],rayT=(tableY-lensCenter.y)/cameraFront.y;
        const tabletopRayPoint=lensCenter.clone().addScaledVector(cameraFront,rayT);
        const idleCenterRayHitsTabletop=rayT>0&&Math.abs(tabletopRayPoint.x-tablePos[0])<=tableHalf[0]&&Math.abs(tabletopRayPoint.z+tablePos[1])<=tableHalf[1];
        assert.equal(idleCenterRayHitsTabletop,false,'Do not claim this natural rest pose aims its feed at the tabletop');
        result[arm]={shoulder:positions[0].toArray(),elbow:positions[1].toArray(),wrist:positions[3].toArray(),gripper:positions[4].toArray(),upperVerticalAngleDeg:upperAngle,lowerVerticalAngleDeg:lowerAngle,
          longitudinalTwistRad:wrappedTwist,handFromDownAngleDeg:new THREE.Vector3(1,0,0).applyQuaternion(handQuaternion).angleTo(down)*180/Math.PI,
          nativeCamera:{framePosition:positions[5].toArray(),stlFrontAxisLocal:[0,0,1],stlFrontAxisWorld:cameraFront.toArray(),sourceBounds:{min:cameraBounds.min.toArray(),max:cameraBounds.max.toArray()},simulatedLensCenter:lensCenter.toArray(),opticsCalibrated:false,
            idleCenterRayHitsTabletop,centerHeightAboveTableM:lensCenter.y-tableY,rayTablePlaneParameter:rayT,rayTablePlanePoint:tabletopRayPoint.toArray(),
            limitation:'Source STL lens-face direction and approximate double-lens centre, not hardware calibration. Standing wrists are near/below table height; a forward-facing native camera does not imply a tabletop view at idle. Centre-ray check is not a rendered-pixel visibility test.'}};
      }
      assert(stats.maxFKPositionErrorM<1e-10);return result;
    }
    function inspectGeometry(phase){
      robot.updateMatrixWorld(true);
      for(const record of records)record.world.copy(record.local).applyMatrix4(new THREE.Matrix4().multiplyMatrices(record.link.matrixWorld,record.originMatrix));
      const bodies=records.filter(record=>bodyNames.has(record.name)),arms=records.filter(record=>!bodyNames.has(record.name));
      const pairs=arms.flatMap(a=>bodies.map(b=>[a,b]));
      for(const right of arms.filter(a=>a.name.startsWith('right_')))for(const left of arms.filter(a=>a.name.startsWith('left_')))pairs.push([right,left]);
      for(const [a,b]of pairs){
        const gap=Math.hypot(...['x','y','z'].map(k=>Math.max(0,a.world.min[k]-b.world.max[k],b.world.min[k]-a.world.max[k])));
        assert(gap>0,`${name}/${phase}: source STL AABBs overlap for ${a.name}/${b.name}`);
        if(gap<geometry.minimumAABBSeparationM){geometry.minimumAABBSeparationM=gap;geometry.closestPair={phase,links:[a.name,b.name]};}
      }
      geometry.samples++;
    }
    function tick(target,staticPhase,phase){
      controller.applyPose(target);dynamics.beforeStep(.001);api.mj_step(model,data);dynamics.afterStep();stats.millisecondSamples++;
      for(const [jointName,joint]of dynamics.map){
        const value=data.qpos[joint.q],velocity=data.qvel[joint.d],effort=Math.abs(data.qfrc_actuator[joint.d]);
        assert([value,velocity,effort].every(Number.isFinite));assert(effort<=joint.effort+1e-8);
        stats.maxEffortRatio=Math.max(stats.maxEffortRatio,effort/joint.effort);
        const excess=Math.max(0,joint.lower-value,value-joint.upper);
        if(joint.finger){stats.maxFingerRangeExcess=Math.max(stats.maxFingerRangeExcess,excess);assert(excess<=.001);}
        else{
          const error=Math.abs(dynamics.targets[jointName]-value),key=staticPhase?'maxStaticErrorRad':'maxMovingErrorRad';
          stats[key]=Math.max(stats[key],error);stats.maxActualVelocityRatio=Math.max(stats.maxActualVelocityRatio,Math.abs(velocity)/velocityLimits[jointName]);
          stats.maxArmJointRangeExcess=Math.max(stats.maxArmJointRangeExcess,excess);assert(excess<=.001);
        }
        if(dynamics instanceof R1SafeDynamics&&joint.finger)assert(Math.abs(dynamics.targetVelocity[jointName])<=R1_FINGER_MOTION_PROFILE.commandVelocityMps+1e-10);
      }
      if(stats.millisecondSamples%10===0){
        const contacts=data.contact;
        try{for(let index=0;index<Math.min(data.ncon,contacts.size());index++){
          const contact=contacts.get(index);
          try{if(pads.has(model.geom_bodyid[contact.geom1])||pads.has(model.geom_bodyid[contact.geom2])){
            const penetration=Math.max(0,-contact.dist);stats.maxIllegalPadPenetrationM=Math.max(stats.maxIllegalPadPenetrationM,penetration);assert(penetration<=.001);
          }}finally{contact.delete();}
        }}finally{contacts.delete();}stats.contactScans++;
      }
      if(stats.millisecondSamples%20===0)inspectGeometry(phase);
    }
    initial();const restPoints=points();inspectGeometry('initial');
    for(let index=0;index<3000;index++)tick(poses.idle,true,'rest');
    for(const [phase,from,to,duration]of [['spread',poses.idle,poses.spread,1.4],['hold-spread',poses.spread,poses.spread,.5],['return',poses.spread,poses.idle,2.6],['hold-rest',poses.idle,poses.idle,1]]){
      const steps=Math.round(duration*1000);
      for(let index=0;index<steps;index++){
        const t=(index+1)/steps,smooth=t*t*(3-2*t),pose=Object.fromEntries(Object.keys(to).map(key=>[key,from[key]+(to[key]-from[key])*smooth]));
        tick(pose,false,phase);
      }
    }
    assert(stats.maxStaticErrorRad<1e-5);assert(stats.maxMovingErrorRad<.01);assert(stats.maxActualVelocityRatio<=1);
    controller.reset();initial();points();assert.equal(controller.history.length,0);assert.equal(data.time,0);
    wristVision.dispose();
    for(const kind of ['both upper and lower arms point down in source FK','controller initialization and reset set genuine qpos/ctrl','original-limit static and REST/SPREAD return dynamics','sampled source-STL non-mount arm/body separation',
      'real wrist quaternion has half-turn longitudinal twist within original limits','native Realsense STL and simulated lens face forward without remount','idle centre ray is not misreported as a tabletop view'])results.push({name:`${name}: ${kind}`,pass:true});
    runs.push({name,restPoints,stats,geometry});
  }finally{data.delete();model.delete();}
}
const sourceFrozen=JSON.stringify(await hashes())===JSON.stringify(sourceHashes);assert(sourceFrozen);
console.log(JSON.stringify({pass:true,checks:results.length,results,initialPose:idle,runs,
  collisionScope:'R1 original fixed-size finger-contact MJCF only; source shell/self contacts are omitted. Independent STL AABB tests do not add collision geoms, modify the physical model, or claim real-robot safety.',
  sourceFrozen,sourceHashes},null,2));
