import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts} from '../dist/multi/robot-runtime.mjs';
import {R1SafeDynamics,R1_FINGER_MOTION_PROFILE as profile} from '../dist/multi/r1-safe-dynamics.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';

const root=new URL('../',import.meta.url),paths=['work/test-multi-r1-gripper.mjs','dist/multi/r1-safe-dynamics.mjs',
  'dist/multi/robot-runtime.mjs','dist/r1/r1-dynamics.mjs','dist/r1/grasp-controller.mjs',
  'dist/r1/assets/r1.urdf','dist/r1/models/r1-dynamic-scene.xml','work/r1-test-robot.mjs'];
async function hashes(){return Object.fromEntries(await Promise.all(paths.map(async path=>[path,
  createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));}
const sourceHashes=await hashes(),urdf=await readFile(new URL('dist/r1/assets/r1.urdf',root),'utf8');
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(await readFile(new URL('dist/r1/models/r1-dynamic-scene.xml',root),'utf8'),vfs),data=new api.MjData(model);vfs.delete();
const runtime=createRobotRuntimeFromParts({id:'r1',api,model,data,robot:await loadTestRobot()}),dynamics=runtime.dynamics;
const fingers=[...dynamics.map.values()].filter(joint=>joint.finger),results=[];
const stats={millisecondSamples:0,maxCommandVelocity:0,maxActualVelocity:0,maxActualFingerRangeExcess:0,maxEffortRatio:0};
function pass(name){results.push({name,pass:true});}
function tick(count){for(let index=0;index<count;index++){
  const qpos=[...data.qpos],qvel=[...data.qvel];dynamics.beforeStep(.001);
  assert.deepEqual([...data.qpos],qpos,'Command smoothing changed actual qpos');assert.deepEqual([...data.qvel],qvel,'Command smoothing changed actual qvel');
  for(const joint of fingers){const speed=Math.abs(dynamics.targetVelocity[joint.name]);stats.maxCommandVelocity=Math.max(stats.maxCommandVelocity,speed);
    assert(speed<=profile.commandVelocityMps+1e-10,'Finger command velocity exceeded conservative profile');
    assert.equal(data.ctrl[joint.actuator],dynamics.fingerRampedTargets[joint.name]);assert.equal(data.ctrl[joint.actuator],dynamics.targets[joint.name]);}
  api.mj_step(model,data);dynamics.afterStep();stats.millisecondSamples++;
  for(const joint of fingers){const actual=data.qpos[joint.q],excess=Math.max(0,joint.lower-actual,actual-joint.upper),effort=Math.abs(data.qfrc_actuator[joint.d]);
    assert(Number.isFinite(actual)&&Number.isFinite(data.qvel[joint.d]));assert(excess<=.001,'R1 range excess passed the unchanged 1 mm numerical guard');
    assert(effort<=joint.effort+1e-6);stats.maxActualVelocity=Math.max(stats.maxActualVelocity,Math.abs(data.qvel[joint.d]));
    stats.maxActualFingerRangeExcess=Math.max(stats.maxActualFingerRangeExcess,excess);stats.maxEffortRatio=Math.max(stats.maxEffortRatio,effort/joint.effort);}
}}
try{
  assert(dynamics instanceof R1SafeDynamics);assert.equal(fingers.length,4);
  for(const joint of fingers){const body=[...urdf.matchAll(/<joint\b([^>]*)>([\s\S]*?)<\/joint>/g)].find(([,attrs])=>attrs.includes(`name="${joint.name}"`))?.[2];
    assert(body);assert.equal(Number(body.match(/<limit\b[^>]*\bvelocity="([^"]+)"/)?.[1]),profile.sourceVelocityMps);}
  assert.equal(profile.commandFraction,.4);assert.equal(profile.commandVelocityMps,profile.sourceVelocityMps*profile.commandFraction);
  pass('All four finger source velocities are 0.25 m/s and conservative commands use 40%');

  dynamics.setTargets(Object.fromEntries(fingers.map(joint=>[joint.name,0])));tick(600);
  for(const joint of fingers){assert.equal(dynamics.fingerDesiredTargets[joint.name],0);assert.equal(dynamics.fingerRampedTargets[joint.name],0);}
  dynamics.setTargets(Object.fromEntries(fingers.map(joint=>[joint.name,.05])));tick(600);
  for(const joint of fingers){assert.equal(dynamics.fingerDesiredTargets[joint.name],.05);assert.equal(dynamics.fingerRampedTargets[joint.name],.05);}
  pass('One-shot close/open requests persist while independent ramp history advances through real motors');

  for(let cycle=0;cycle<8;cycle++){
    for(const joint of fingers)data.ctrl[joint.actuator]=0;tick(75);
    for(const joint of fingers)data.ctrl[joint.actuator]=.05;tick(75);
  }
  pass('Rapid reversals preserve the command slew, actual state integration and original force/range bounds');

  dynamics.setTargets(Object.fromEntries(fingers.map(joint=>[joint.name,0])));tick(50);
  runtime.reset();
  for(const joint of fingers){assert.equal(dynamics.fingerDesiredTargets[joint.name],.05);assert.equal(dynamics.fingerRampedTargets[joint.name],.05);
    assert.equal(data.ctrl[joint.actuator],.05);assert.equal(dynamics.targetVelocity[joint.name],0);}
  tick(600);for(const joint of fingers)assert.equal(dynamics.fingerRampedTargets[joint.name],.05);
  pass('Reset clears both desired/ramped histories and cannot resume a stale closing request');

  runtime.controller.opening=.013;const opening=runtime.controller.opening,pose={...runtime.controller.pose};runtime.stop();
  assert.equal(runtime.controller.opening,opening);assert.deepEqual(runtime.controller.pose,pose);
  pass('Batch stop still retains the R1 aperture and arm targets');
}finally{runtime.dispose();}
const sourceFrozen=JSON.stringify(await hashes())===JSON.stringify(sourceHashes);assert(sourceFrozen);
console.log(JSON.stringify({pass:true,checks:results.length,results,profile,stats,sourceFrozen,sourceHashes},null,2));
