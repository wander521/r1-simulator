import assert from 'node:assert/strict';
import {readFile,mkdir,appendFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController} from '../dist/g1/grasp-controller.mjs';
import {G1Dynamics} from '../dist/g1/g1-dynamics.mjs';
import {ROOT_HEIGHT,TABLE_HEIGHT,OBJECT_START_X,OBJECT_DROP_X,OBJECT_LATERAL_OFFSET} from '../dist/g1/g1-profile.mjs';
import {loadG1TestRobot} from './g1-test-robot.mjs';

const root=new URL('../',import.meta.url),args=process.argv.slice(2),filter=args.find(a=>!a.startsWith('--')),repeat=Number(args.find(a=>a.startsWith('--repeat='))?.split('=')[1]||1);
const reportPath=args.find(a=>a.startsWith('--report='))?.slice(9)||`work/validation/g1/dynamic-grasp-${Date.now()}.jsonl`;
await mkdir(new URL('work/validation/g1/',root),{recursive:true});
const sourceHashes=Object.fromEntries(await Promise.all(['work/test-g1-dynamic-grasp.mjs','work/build-g1-dynamics.mjs','dist/g1/grasp-controller.mjs','dist/g1/grasp-kinematics.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/models/g1-dynamic-scene.xml','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/assets/dex1_1/meshes/dex1_col_1.stl','dist/g1/assets/dex1_1/meshes/dex1_col_2.stl'].map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
const report=async record=>{const line=JSON.stringify({...record,sourceHashes});console.log(line);await appendFile(new URL(reportPath,root),line+'\n');};
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
const xml=await readFile(new URL('../dist/g1/models/g1-dynamic-scene.xml',import.meta.url),'utf8');
const targetsFor=()=>[{id:'red',body:'red_cube',z:OBJECT_LATERAL_OFFSET,h:.0225},{id:'blue',body:'blue_block',z:0,h:.019},{id:'yellow',body:'yellow_cylinder',z:-OBJECT_LATERAL_OFFSET,h:.026}].map(t=>({...t,start:new THREE.Vector3(OBJECT_START_X,TABLE_HEIGHT+t.h,t.z),drop:new THREE.Vector3(OBJECT_DROP_X,TABLE_HEIGHT+t.h,t.z),rotation:new THREE.Quaternion()}));
async function scene(targets){const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),robot=await loadG1TestRobot();vfs.delete();const dynamics=new G1Dynamics(api,model,data,robot),controller=new GraspController(api,model,data,robot,targets,dynamics);return {model,data,robot,dynamics,c:controller};}
const compact=m=>({count:m.count,force:m.force,forces:m.forces,table:m.table,support:m.support,position:m.position.toArray(),speed:m.speed,distance:m.distance});
function createMonitor({model,data,robot,dynamics,c}){
  const feet=['left','right'].map(side=>robot.links[`${side}_ankle_roll_link`].getWorldPosition(new THREE.Vector3())),pelvis=api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,'pelvis');
  const stats={samples:0,fingerLimitSamples:0,maxFingerLimitViolation:0,maxTorque:0,maxFingerForce:0,maxJointError:0,maxFootDrift:0,maxLegAngle:0,maxWaistTilt:0,maxWaistYaw:0,actualDualForce:false,maxWristRoll:{left:0,right:0},jointExtrema:{},trackingByPhase:{},collision:{samples:0,maxIllegalPenetration:0,illegalContacts:[],kinds:{},phases:[]}};
  const fingerJoints=[...dynamics.map.values()].filter(j=>j.finger);
  const inspectFingerLimits=()=>{
    stats.fingerLimitSamples++;
    for(const j of fingerJoints){const actual=data.qpos[j.q],violation=Math.max(0,j.lower-actual,actual-j.upper);stats.maxFingerLimitViolation=Math.max(stats.maxFingerLimitViolation,violation);assert(violation<=.0002,`${j.name} actual physical stroke exceeded by ${violation*1000}mm at ${data.time}s/${c.state}: ${actual} outside [${j.lower}, ${j.upper}]`);}
  };
  const inspect=()=>{
    stats.samples++;
    const telemetry=dynamics.telemetry(c.arm);assert.equal(telemetry.joints.length,7);assert.equal(telemetry.allJoints.length,33);
    for(const j of telemetry.allJoints){
      assert(Number.isFinite(j.actual)&&Number.isFinite(j.torque)&&Number.isFinite(j.error));assert(Math.abs(j.torque)<=j.effortLimit+1e-7,`${j.name} effort limit`);
      assert(j.actual>=j.lower-(j.unit==='m'?.0002:.001)&&j.actual<=j.upper+(j.unit==='m'?.0002:.001),`${j.name} actual joint position must remain inside its source limits (0.2mm/1mrad solver tolerance)`);
      const range=stats.jointExtrema[j.name]??={min:j.actual,max:j.actual,targetMin:j.target,targetMax:j.target,minimumLimitMargin:Infinity,lower:j.lower,upper:j.upper};
      range.min=Math.min(range.min,j.actual);range.max=Math.max(range.max,j.actual);range.targetMin=Math.min(range.targetMin,j.target);range.targetMax=Math.max(range.targetMax,j.target);range.minimumLimitMargin=Math.min(range.minimumLimitMargin,j.actual-j.lower,j.upper-j.actual);
      if(j.unit==='rad'){stats.maxTorque=Math.max(stats.maxTorque,Math.abs(j.torque));stats.maxJointError=Math.max(stats.maxJointError,Math.abs(j.error));}else {assert(j.effortLimit===20);stats.maxFingerForce=Math.max(stats.maxFingerForce,Math.abs(j.torque));}
      if(j.name==='waist_yaw_joint')stats.maxWaistYaw=Math.max(stats.maxWaistYaw,Math.abs(j.actual));
      for(const side of ['left','right'])if(j.name===`${side}_wrist_roll_joint`)stats.maxWristRoll[side]=Math.max(stats.maxWristRoll[side],Math.abs(j.actual));
    }
    for(const j of telemetry.legs){assert.equal(j.target,0,'Leg targets must stay in fixed zero-angle stance');stats.maxLegAngle=Math.max(stats.maxLegAngle,Math.abs(j.actual));}
    for(const j of telemetry.torso.filter(j=>j.name!=='waist_yaw_joint')){assert.equal(j.target,0,'No waist roll/pitch squat movement');stats.maxWaistTilt=Math.max(stats.maxWaistTilt,Math.abs(j.actual));}
    for(const [i,side]of ['left','right'].entries())stats.maxFootDrift=Math.max(stats.maxFootDrift,robot.links[`${side}_ankle_roll_link`].getWorldPosition(new THREE.Vector3()).distanceTo(feet[i]));
    assert(Math.abs(data.xpos[pelvis*3+2]-ROOT_HEIGHT)<1e-12,'Pelvis fixed at calibrated standing height');
    const m=c.measure();if(m.count===2&&m.forces.every(force=>force>.015))stats.actualDualForce=true;
    const phase=stats.trackingByPhase[c.state]??={jointError:0,wristError:0};phase.jointError=Math.max(phase.jointError,...telemetry.joints.map(j=>Math.abs(j.error)));phase.wristError=Math.max(phase.wristError,c.wristError());
    const selected=c.bodies.get(c.selected),activeFingers=new Set(c.fingers[c.arm].map(f=>f.body)),illegal=[],list=data.contact;
    stats.collision.samples++;if(!stats.collision.phases.includes(c.state))stats.collision.phases.push(c.state);
    // These are actual solver contacts, not a second check of commanded poses.
    // Only normal foot/floor and the active gripper's selected object are legal
    // robot contacts. Task/table and object/support pairs do not involve robot.
    try{for(let i=0;i<Math.min(data.ncon,list.size());i++){
      const contact=list.get(i);
      try{
        const b1=model.geom_bodyid[contact.geom1],b2=model.geom_bodyid[contact.geom2],r1=dynamics.robotBodyIds.has(b1),r2=dynamics.robotBodyIds.has(b2);
        if(!r1&&!r2)continue;
        const footFloor=(dynamics.footBodyIds.has(b1)&&contact.geom2===dynamics.floorGeom)||(dynamics.footBodyIds.has(b2)&&contact.geom1===dynamics.floorGeom);
        const selectedFinger=(activeFingers.has(b1)&&b2===selected)||(activeFingers.has(b2)&&b1===selected);
        if(footFloor||selectedFinger)continue;
        const penetration=Math.max(0,-contact.dist),kind=r1&&r2?'robot-self':'robot-environment';
        stats.collision.maxIllegalPenetration=Math.max(stats.collision.maxIllegalPenetration,penetration);
        if(penetration>.001){const record={time:data.time,phase:c.state,selected:c.selected,kind,dist:contact.dist,body1:dynamics.bodyNames[b1],body2:dynamics.bodyNames[b2],geom1:dynamics.geomNames[contact.geom1],geom2:dynamics.geomNames[contact.geom2],worldPosition:[contact.pos[0],contact.pos[2],-contact.pos[1]]};illegal.push(record);stats.collision.kinds[kind]=(stats.collision.kinds[kind]||0)+1;if(stats.collision.illegalContacts.length<12)stats.collision.illegalContacts.push(record);}
      }finally{contact.delete();}
    }}finally{list.delete();}
    assert.equal(illegal.length,0,`Actual collision penetration > 1mm: ${JSON.stringify(illegal.slice(0,3))}`);
  };
  let steps=0;
  return {stats,inspect(){inspectFingerLimits();inspect();},step(){c.step();inspectFingerLimits();if(steps++%100===0)inspect();},snapshot(){return {...stats,naturalWristRollPreference:Object.values(stats.maxWristRoll).every(angle=>angle<.5)};}};
}
function assertHealthy(monitor){
  assert(monitor.stats.maxJointError<.07,'Actual joint trajectory must track the command without blocked/saturated motion');
  assert(monitor.stats.maxFootDrift<.0005&&monitor.stats.maxLegAngle<.001&&monitor.stats.maxWaistTilt<.003,'Fixed legs/waist must not squat');
  assert(monitor.stats.maxWaistYaw<1.001,'Horizontal waist alignment must remain within the permitted ±1 rad range');
}
const scenarios=[];for(const arm of ['right','left'])for(const id of ['red','blue','yellow'])scenarios.push({arm,id});
scenarios.push({arm:'right',id:'blue',offset:[-.015,.012]},{arm:'left',id:'yellow',offset:[.012,-.012]},{arm:'right',id:'blue',yaw:.4},{arm:'left',id:'yellow',roll:Math.PI/2},{arm:'right',stack:true},{arm:'left',stack:true});
const failures=[];
for(const scenario of scenarios.filter(s=>!filter||JSON.stringify(s).includes(filter)).flatMap(s=>Array.from({length:repeat},(_,i)=>({...s,repeat:i+1})))){
  const targets=targetsFor(),target=targets.find(t=>t.id===scenario.id);
  if(scenario.offset){target.start.x+=scenario.offset[0];target.start.z+=scenario.offset[1];}
  if(scenario.yaw)target.rotation.setFromAxisAngle(new THREE.Vector3(0,1,0),scenario.yaw);
  if(scenario.roll){target.rotation.setFromAxisAngle(new THREE.Vector3(1,0,0),scenario.roll);target.start.y=TABLE_HEIGHT+.02;}
  const current=await scene(targets),{model,data,dynamics,c}=current,monitor=createMonitor(current);
  let previous='';const transitions=[];
  try{
    for(let n=0;n<300;n++)monitor.step();assert(scenario.stack?c.startStack(scenario.arm):c.start(scenario.arm,scenario.id),`${c.message} ${JSON.stringify(c.planError)}`);
    for(let n=0;n<(scenario.stack?120000:30000)&&c.running;n++){
      monitor.step();const key=`${c.itemIndex}/${c.selected}/${c.state}`;if(key!==previous){transitions.push({time:data.time,key,measurement:compact(c.measure())});previous=key;}
    }
    monitor.inspect();assert.equal(c.state,'complete',c.message);assert(monitor.stats.actualDualForce,'Actual solver normal forces must exist on both fingers');
    if(scenario.stack){assert(c.stackResultVerified);assert(c.stackStableSeconds>=3);assert.equal(c.stackEvidence.length,3);assert(c.stackEvidence.every(e=>e.dual&&e.lift>.035));for(let n=0;n<3000;n++)monitor.step();monitor.inspect();assert(c.stackCheck().stable,'Released stack must stay supported for an additional 3 seconds');}
    else {assert(c.evidence.dual&&c.evidence.lift>.035);assert(c.evidence.maxSpeed<1);assert(c.measure().distance<.02&&c.measure().table);}
    assertHealthy(monitor);
    const result={scenario,pass:true,evidence:c.evidence,measurement:compact(c.measure()),stackStableSeconds:c.stackStableSeconds,stackEvidence:c.stackEvidence,stackcheck:scenario.stack?c.stackCheck():undefined,...monitor.snapshot(),transitions};
    c.reset();assert.equal(c.state,'idle');assert.equal(c.running,false);assert.equal(c.reaction,null);assert(!c.stackResultVerified);assert(!c.reactionFinished);assert.equal(c.history.length,0);await report(result);
  }catch(error){const failure={scenario,pass:false,error:error.message,state:c.state,message:c.message,selected:c.selected,itemIndex:c.itemIndex,planError:c.planError,evidence:c.evidence,measurement:compact(c.measure()),stackEvidence:c.stackEvidence,stackcheck:scenario.stack?c.stackCheck():undefined,...monitor.snapshot(),transitions};failures.push(failure);await report(failure);}
  dynamics.dispose();data.delete();model.delete();
}
if(!filter||filter==='recovery'){
  const targets=targetsFor();targets.find(t=>t.id==='blue').start.x=1.8;const current=await scene(targets),{model,data,dynamics,c}=current;let monitor=createMonitor(current);
  try{
    for(let n=0;n<300;n++)monitor.step();assert.equal(c.startStack('right'),false);assert(c.running&&c.reaction==='shy');const reason=c.message;
    for(let n=0;n<15000&&c.running;n++)monitor.step();monitor.inspect();assert.equal(c.state,'failed');assert.equal(c.message,reason);assert(c.history.includes('shy'));assert(!c.history.includes('happy'));assert(!c.stackResultVerified);assertHealthy(monitor);await report({scenario:{recovery:'unreachable_stack'},pass:true,reason,history:c.history,...monitor.snapshot()});
    targets.find(t=>t.id==='blue').start.x=OBJECT_START_X;c.reset();monitor=createMonitor(current);for(let n=0;n<300;n++)monitor.step();assert(c.start('right','blue'));for(let n=0;n<30000&&c.running;n++)monitor.step();monitor.inspect();assert.equal(c.state,'complete');assert(c.evidence.dual&&c.evidence.lift>.035&&c.measure().table);assertHealthy(monitor);await report({scenario:{recovery:'failed_reset_then_sort'},pass:true,evidence:c.evidence,measurement:compact(c.measure()),...monitor.snapshot()});
    c.reset();monitor=createMonitor(current);for(let n=0;n<300;n++)monitor.step();assert(c.startStack('left'));for(let n=0;n<25000&&c.running&&c.state!=='transfer';n++)monitor.step();assert.equal(c.state,'transfer');assert(c.evidence.lift>.035);c.stop();const stoppedPose={...c.pose};for(let n=0;n<4000;n++)monitor.step();monitor.inspect();assert.equal(c.state,'stopped');assert(!c.running);assert.equal(c.reaction,null);assert.deepEqual(c.pose,stoppedPose);assert(!c.stackResultVerified);assertHealthy(monitor);await report({scenario:{recovery:'stop_stack_mid_transfer'},pass:true,state:c.state,measurement:compact(c.measure()),...monitor.snapshot()});
    c.reset();monitor=createMonitor(current);for(let n=0;n<300;n++)monitor.step();assert(c.startStack('right'));for(let n=0;n<100000&&c.running&&c.state!=='happy';n++)monitor.step();assert.equal(c.state,'happy');assert(c.stackResultVerified);for(let n=0;n<100;n++)monitor.step();c.stop();for(let n=0;n<4000;n++)monitor.step();monitor.inspect();assert.equal(c.state,'stopped');assert(!c.running);assert.equal(c.reaction,null);assert.notEqual(c.reactionFinished,'happy');assert(c.stackCheck().stable);assertHealthy(monitor);await report({scenario:{recovery:'stop_happy_mid_gesture'},pass:true,state:c.state,stackcheck:c.stackCheck(),...monitor.snapshot()});
    c.reset();assert.equal(c.state,'idle');assert(!c.stackResultVerified&&!c.reactionFinished);assert.equal(c.history.length,0);assert(dynamics.telemetry().legs.every(j=>j.target===0));
  }catch(error){const failure={scenario:{recovery:true},pass:false,error:error.message,state:c.state,message:c.message,history:c.history,measurement:compact(c.measure()),planError:c.planError,...monitor.snapshot()};failures.push(failure);await report(failure);}
  dynamics.dispose();data.delete();model.delete();
}
console.log(JSON.stringify({report:reportPath,failures:failures.length}));assert.equal(failures.length,0,`${failures.length} G1 dynamics acceptance cases failed`);
