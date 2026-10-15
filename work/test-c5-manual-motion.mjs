import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController} from '../dist/c5/grasp-controller.mjs';
import {C5Dynamics} from '../dist/c5/c5-dynamics.mjs';
import {tcpPose,solvePose} from '../dist/c5/grasp-kinematics.mjs';
import {HOME_POSE,TABLE_HEIGHT,OBJECT_START_X,OBJECT_DROP_X,OBJECT_LATERAL_OFFSET,armJointNames,fingerJointNames,qToOpening,GRIPPER_Q_MIN,GRIPPER_Q_MAX} from '../dist/c5/c5-profile.mjs';
import {loadC5TestRobot} from './c5-test-robot.mjs';

const root=new URL('../',import.meta.url),names=armJointNames(),OPEN=qToOpening(GRIPPER_Q_MAX),CLOSED=qToOpening(GRIPPER_Q_MIN);
const hashPaths=['work/test-c5-manual-motion.mjs','work/build-c5-dynamics.mjs','work/c5-test-robot.mjs','dist/c5/grasp-controller.mjs','dist/c5/grasp-kinematics.mjs','dist/c5/c5-dynamics.mjs','dist/c5/c5-profile.mjs','dist/c5/models/c5-dynamic-scene.xml','dist/c5/assets/aubo_C5.urdf','dist/c5/assets/c5-dex1.urdf','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/assets/g1_29dof_with_dex1_1.urdf',...Array.from({length:7},(_,i)=>`dist/c5/assets/meshes/aubo_C5/collision/link${i}.STL`),'dist/g1/assets/dex1_1/meshes/Dex1_base_link.STL',...[1,2].map(i=>`dist/g1/assets/dex1_1/meshes/dex1_col_${i}.stl`)];
const hashes=async()=>Object.fromEntries(await Promise.all(hashPaths.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')]))),sourceHashes=await hashes();
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname}),vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(await readFile(new URL('dist/c5/models/c5-dynamic-scene.xml',root),'utf8'),vfs);vfs.delete();
const targetsFor=()=>[{id:'red',body:'red_cube',h:.0225,z:OBJECT_LATERAL_OFFSET},{id:'blue',body:'blue_block',h:.019,z:0},{id:'yellow',body:'yellow_cylinder',h:.026,z:-OBJECT_LATERAL_OFFSET}].map(t=>({...t,start:new THREE.Vector3(OBJECT_START_X,TABLE_HEIGHT+t.h,t.z),drop:new THREE.Vector3(OBJECT_DROP_X,TABLE_HEIGHT+t.h,t.z),rotation:new THREE.Quaternion()}));
const results=[];
async function scene(){const data=new api.MjData(model),robot=await loadC5TestRobot(),dynamics=new C5Dynamics(api,model,data,robot),c=new GraspController(api,model,data,robot,targetsFor(),dynamics);return {data,robot,dynamics,c};}
const clone=value=>JSON.parse(JSON.stringify(value));
function monitorFor({data,dynamics,c}){
  const base=api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,'base_link'),plinth=api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,'robot_plinth');
  const stats={samples:0,collisionSamples:0,noActualRewriteChecks:0,maxVelocityRatio:0,maxFingerLimitViolation:0,maxIllegalPenetration:0,maxPositionIntegrationResidual:0,perJoint:{},illegalContacts:[]};
  const actual=()=>({qpos:[...data.qpos],qvel:[...data.qvel],ctrl:[...data.ctrl],xpos:[...data.xpos],xquat:[...data.xquat],time:data.time});
  const command=()=>({pose:{...c.pose},targets:{...dynamics.targets},manualMotion:clone(c.manualMotion),openings:{...c.openings},fingerDesired:{...dynamics.fingerDesiredTargets},fingerRamped:{...dynamics.fingerRampedTargets}});
  const inspect=()=>{
    stats.samples++;const telemetry=dynamics.telemetry();assert.equal(telemetry.allJoints.length,8);
    for(const j of telemetry.allJoints){
      assert([j.actual,j.target,j.velocity,j.torque,j.targetVelocity].every(Number.isFinite),`${j.name} finite actual telemetry`);
      const row=stats.perJoint[j.name]??={min:j.actual,max:j.actual,maxActualVelocity:0,maxTargetVelocity:0,maxTorque:0,maxRangeViolation:0,lower:j.lower,upper:j.upper,velocityLimit:j.velocityLimit,effortLimit:j.effortLimit,unit:j.unit};
      const violation=Math.max(0,j.lower-j.actual,j.actual-j.upper),ratio=Math.abs(j.velocity)/j.velocityLimit;
      row.min=Math.min(row.min,j.actual);row.max=Math.max(row.max,j.actual);row.maxActualVelocity=Math.max(row.maxActualVelocity,Math.abs(j.velocity));row.maxTargetVelocity=Math.max(row.maxTargetVelocity,Math.abs(j.targetVelocity));row.maxTorque=Math.max(row.maxTorque,Math.abs(j.torque));row.maxRangeViolation=Math.max(row.maxRangeViolation,violation);stats.maxVelocityRatio=Math.max(stats.maxVelocityRatio,ratio);
      if(j.unit==='m')stats.maxFingerLimitViolation=Math.max(stats.maxFingerLimitViolation,violation);
      assert(violation<=(j.unit==='m'?.0002:.001),`${j.name} actual range violation ${violation} at ${data.time}/${c.state}`);
      assert(Math.abs(j.torque)<=j.effortLimit+1e-7,`${j.name} actual torque ${j.torque} > ${j.effortLimit}`);
      assert(Math.abs(j.velocity)<=j.velocityLimit*1.01+.0001,`${j.name} actual velocity ${j.velocity} > source ${j.velocityLimit} +1% at ${data.time}/${c.state}`);
    }
    stats.collisionSamples++;const contacts=data.contact,illegal=[];
    try{for(let i=0;i<Math.min(data.ncon,contacts.size());i++){
      const contact=contacts.get(i);try{
        const b1=model.geom_bodyid[contact.geom1],b2=model.geom_bodyid[contact.geom2],r1=dynamics.robotBodyIds.has(b1),r2=dynamics.robotBodyIds.has(b2);if(!r1&&!r2)continue;
        if((b1===base&&b2===plinth)||(b2===base&&b1===plinth))continue;
        const selected=c.bodies.get(c.selected),fingers=c.fingers.left.map(f=>f.body),selectedFinger=(fingers.includes(b1)&&b2===selected)||(fingers.includes(b2)&&b1===selected);
        if(c.running&&selectedFinger)continue;
        const penetration=Math.max(0,-contact.dist);stats.maxIllegalPenetration=Math.max(stats.maxIllegalPenetration,penetration);
        if(penetration>.001){const record={time:data.time,state:c.state,dist:contact.dist,body1:dynamics.bodyNames[b1],body2:dynamics.bodyNames[b2],geom1:dynamics.geomNames[contact.geom1],geom2:dynamics.geomNames[contact.geom2]};illegal.push(record);if(stats.illegalContacts.length<12)stats.illegalContacts.push(record);}
      }finally{contact.delete();}
    }}finally{contacts.delete();}
    assert.equal(illegal.length,0,`Actual collision >1mm: ${JSON.stringify(illegal.slice(0,3))}`);
  };
  const step=(n=1)=>{for(let i=0;i<n;i++){
    const previous=Object.fromEntries([...dynamics.map].map(([name,j])=>[name,{q:data.qpos[j.q],velocity:data.qvel[j.d]}]));c.step(.001);
    for(const [name,j]of dynamics.map){const displacement=Math.abs(data.qpos[j.q]-previous[name].q),bound=.001*Math.max(Math.abs(previous[name].velocity),Math.abs(data.qvel[j.d])),residual=Math.max(0,displacement-bound);stats.maxPositionIntegrationResidual=Math.max(stats.maxPositionIntegrationResidual,residual);assert(residual<1e-8,`${name} qpos moved without physical qvel: ${residual}`);}
    inspect();
  }};
  const request=(pose,expected=true)=>{
    const beforeActual=actual(),beforeCommand=command(),accepted=c.applyManualPose(pose);assert.equal(accepted,expected,`${c.message}: ${JSON.stringify(c.planError)}`);assert.deepEqual(actual(),beforeActual,'Manual target must never rewrite real solver state');
    if(expected){assert.deepEqual(c.pose,beforeCommand.pose,'Accepted manual target is queued, not a pose jump');assert.deepEqual(dynamics.targets,beforeCommand.targets,'Accepted manual target does not step actuator targets before the physical clock');}
    else assert.deepEqual(command(),beforeCommand,'Rejected target must preserve current commands and pending motion');
    stats.noActualRewriteChecks++;return accepted;
  };
  const finish=()=>{for(let n=0;n<15000&&c.manualMotion;n++)step();assert.equal(c.manualMotion,null,'Manual trajectory must finish');step(500);};
  return {stats,actual,command,inspect,step,request,finish};
}
async function run(name,body){
  const current=await scene(),monitor=monitorFor(current);let detail={};
  try{detail=await body(current,monitor)||{};results.push({name,pass:true,...detail,...monitor.stats});console.log(JSON.stringify({name,pass:true,samples:monitor.stats.samples,maxVelocityRatio:monitor.stats.maxVelocityRatio,maxIllegalPenetration:monitor.stats.maxIllegalPenetration}));}
  catch(error){results.push({name,pass:false,error:error.message,state:current.c.state,planError:current.c.planError,...detail,...monitor.stats});console.log(JSON.stringify({name,pass:false,error:error.message,planError:current.c.planError}));}
  finally{current.dynamics.dispose();current.data.delete();}
}
for(const [name,amount]of [['shoulder_joint',1],['upperArm_joint',.5],['foreArm_joint',.5],['wrist1_joint',.5],['wrist2_joint',.5],['wrist3_joint',1]])await run('large-step-'+name,({c},m)=>{
  m.step(500);const target={...c.pose,[name]:amount};m.request(target);const duration=c.manualMotion.duration;m.finish();assert(Math.abs(c.dynamics.telemetry().joints.find(j=>j.name===name).actual-amount)<.003);return {commandedJoint:name,commandedAngle:amount,duration};
});
await run('rapid-reversals',({c},m)=>{m.step(300);const commands=[];for(const [goal,steps]of [[1,240],[-1,160],[.8,200],[-.75,170],[.6,250],[0,0]]){const target={...c.pose,shoulder_joint:goal};m.request(target);commands.push({time:c.data.time,goal,from:c.pose.shoulder_joint,duration:c.manualMotion.duration});m.step(steps);}m.finish();return {commands};});
await run('continuous-tcp-edits',({c,robot,dynamics},m)=>{
  m.step(300);const commands=[];
  for(let i=0;i<10;i++){
    const goal={...c.pose,shoulder_joint:c.pose.shoulder_joint+(i%2?-.025:.04),upperArm_joint:c.pose.upperArm_joint+.012,foreArm_joint:c.pose.foreArm_joint+.01,wrist3_joint:c.pose.wrist3_joint+(i%2?-.025:.04)};
    robot.setJointValues(goal);robot.updateMatrixWorld(true);const tcp=tcpPose(robot);dynamics.syncRobot();const solution=solvePose(robot,'left',tcp.position,tcp.quaternion,c.pose,350);dynamics.syncRobot();assert(solution.positionError<.0025&&solution.angleError<.025);
    m.request(solution.pose);commands.push({time:c.data.time,position:tcp.position.toArray(),positionError:solution.positionError,angleError:solution.angleError});m.step(45);
  }m.finish();return {commands};
});
await run('pending-stop-clears-motion',({c},m)=>{m.step(300);m.request({...c.pose,shoulder_joint:1});m.step(220);assert(c.manualMotion);const before=m.actual();c.stop();assert.equal(c.manualMotion,null);assert.deepEqual(m.actual(),before);const stopped={...c.pose};m.step(1800);assert.deepEqual(c.pose,stopped);assert.equal(c.manualMotion,null);return {state:c.state};});
await run('pending-reset-clears-motion',({c},m)=>{m.step(300);m.request({...c.pose,wrist3_joint:1});m.step(220);assert(c.manualMotion);c.reset();assert.equal(c.manualMotion,null);assert.deepEqual(c.pose,HOME_POSE);m.step(1600);assert.deepEqual(c.pose,HOME_POSE);assert.equal(c.manualMotion,null);return {explicitResetAllowed:true};});
await run('pending-start-clears-motion',({c},m)=>{m.step(300);m.request({...c.pose,wrist3_joint:.5});m.step(180);assert(c.manualMotion);const before=m.actual();assert(c.start('left','blue'),`${c.message}: ${JSON.stringify(c.planError)}`);assert.equal(c.manualMotion,null);assert.deepEqual(m.actual(),before,'Starting task may plan but must not rewrite actual state');m.step(3000);assert.equal(c.manualMotion,null);assert(c.running);c.stop();m.step(1000);assert.equal(c.manualMotion,null);return {taskStarted:true,finalState:c.state};});
await run('pending-failure-clears-motion',({c},m)=>{m.step(300);m.request({...c.pose,shoulder_joint:1});m.step(180);assert(c.manualMotion);const before=m.actual();c.fail('manual pending failure fixture');assert.equal(c.manualMotion,null);assert.deepEqual(m.actual(),before);const stopped={...c.pose};m.step(1200);assert.equal(c.manualMotion,null);assert.deepEqual(c.pose,stopped);assert.equal(c.state,'failed');return {failureReason:c.message};});
const A={shoulder_joint:-.2663934747688472,upperArm_joint:.6815151846967638,foreArm_joint:.24314724490977824,wrist1_joint:.8481733128428459,wrist2_joint:1.8638996023219079,wrist3_joint:0};
const B={shoulder_joint:-1.0092515707947314,upperArm_joint:-.878724385984242,foreArm_joint:2.4717594438698143,wrist1_joint:1.568647266831249,wrist2_joint:-1.2278378510382026,wrist3_joint:0};
await run('actual-departure-A-B-rejected-without-state-write',({c},m)=>{m.request(B,false);assert.equal(c.manualMotion,null);c.reset();m.request(A);const queued=clone(c.manualMotion);m.request(B,false);assert.deepEqual(c.manualMotion,queued);assert.deepEqual(c.pose,HOME_POSE);m.step(3000);assert.equal(c.manualMotion,null);return {A,B,fromResetBRejected:true,immediateAThenBRejected:true,queuedAPreserved:true};});
await run('near-table-arm-and-gripper-interaction',({c,robot,dynamics,data},m)=>{
  assert(c.start('left','blue'),`${c.message}: ${JSON.stringify(c.planError)}`);let fixture;
  for(const index of [40,36,32,28,24,20])for(const roll of [-Math.PI/2,Math.PI/2,-1.1,1.1,-.7,.7]){
    const pose={...c.planData.descend[index],wrist3_joint:c.planData.descend[index].wrist3_joint+roll},closed=dynamics.checkPose(pose,{opening:CLOSED,ignoreObjectBodies:[c.bodies.get('blue')]}),opened=dynamics.checkPose(pose,{opening:OPEN,ignoreObjectBodies:[c.bodies.get('blue')]});
    if(!fixture&&closed.safe&&!opened.safe&&opened.contacts.some(contact=>contact.dist<-.003&&[contact.body1,contact.body2].includes('task_table')&&[contact.body1,contact.body2].some(name=>name.includes('finger'))))fixture={pose,index,roll,contacts:opened.contacts};
  }assert(fixture,'Must find genuine source-mesh tabletop opening interference');
  c.reset();assert(c.setObject('blue',new THREE.Vector3(.65,TABLE_HEIGHT+.019,.25)));c.opening=CLOSED;c.pose={...fixture.pose};dynamics.reset({...fixture.pose,...Object.fromEntries(fingerJointNames().map(name=>[name,GRIPPER_Q_MIN]))});
  const initial=dynamics.checkPose(c.pose,{opening:CLOSED});assert(initial.safe,JSON.stringify(initial));m.step(300);
  const before=m.actual(),command=m.command();assert.equal(c.setManualOpening(OPEN),false);assert.deepEqual(m.actual(),before);assert.deepEqual(m.command(),command);m.stats.noActualRewriteChecks++;
  const tcp=tcpPose(robot),point=tcp.position.clone().add(new THREE.Vector3(0,.10,0)),solution=solvePose(robot,'left',point,tcp.quaternion,c.pose,350);dynamics.syncRobot();assert(solution.positionError<.0025&&solution.angleError<.025);m.request(solution.pose);const pending=m.command(),pendingActual=m.actual();assert.equal(c.setManualOpening(OPEN),false);assert.deepEqual(m.command(),pending);assert.deepEqual(m.actual(),pendingActual);m.stats.noActualRewriteChecks++;m.finish();
  const lifted=m.actual();assert(c.setManualOpening(OPEN));assert.deepEqual(m.actual(),lifted);m.stats.noActualRewriteChecks++;m.step(900);assert(dynamics.telemetry().fingers.every(j=>Math.abs(j.actual-GRIPPER_Q_MAX)<.001));
  return {fixture:{index:fixture.index,roll:fixture.roll,pose:fixture.pose,contacts:fixture.contacts.slice(0,4)},explicitFixtureInitialization:true,selectedObjectMovedOnlyDuringSetup:true,unsafeOpeningRejectedAtRestAndPending:true,safeLiftThenOpeningAccepted:true};
});
const sourceFrozen=JSON.stringify(await hashes())===JSON.stringify(sourceHashes),pass=sourceFrozen&&results.every(result=>result.pass),report={pass,cases:results.length,results,sourceHashes,sourceFrozen,scope:'Production C5 controller/dynamics, official C5 + Dex1 meshes. Actual qpos/qvel/force/contact every 1ms. Explicit reset/near-table fixture setup is disclosed and excluded from no-rewrite assertions.'};
await mkdir(new URL('work/validation/c5/',root),{recursive:true});await writeFile(new URL('work/validation/c5/manual-motion-final.json',root),JSON.stringify(report,null,2)+'\n');model.delete();console.log(JSON.stringify({pass,cases:results.length,failed:results.filter(result=>!result.pass).map(result=>result.name),sourceFrozen}));assert(pass,'Manual-motion regression failed; see recorded real-state diagnostics');
