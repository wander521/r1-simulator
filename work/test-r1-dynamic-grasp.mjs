import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController,TABLE_HEIGHT} from '../dist/r1/grasp-controller.mjs';
import {R1Dynamics} from '../dist/r1/r1-dynamics.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
const xml=await readFile(new URL('../dist/r1/models/r1-dynamic-scene.xml',import.meta.url),'utf8');
const sourceHashes=Object.fromEntries(await Promise.all(['dist/r1/grasp-controller.mjs','dist/r1/grasp-kinematics.mjs','dist/r1/r1-dynamics.mjs','dist/r1/models/r1-dynamic-scene.xml','dist/r1/assets/r1.urdf'].map(async path=>[path,createHash('sha256').update(await readFile(new URL('../'+path,import.meta.url))).digest('hex')])));
const report=record=>console.log(JSON.stringify({...record,sourceHashes}));
const scenarios=[];
for(const arm of ['right','left'])for(const id of ['red','blue','yellow'])scenarios.push({arm,id});
scenarios.push({arm:'right',id:'blue',offset:[-.025,.035]},{arm:'left',id:'yellow',offset:[.02,-.025]});
scenarios.push({arm:'right',id:'blue',yaw:.4},{arm:'left',id:'yellow',roll:Math.PI/2});
scenarios.push({arm:'right',stack:true},{arm:'left',stack:true});
const filter=process.argv.slice(2).find(arg=>!arg.startsWith('--'));
const repeats=Number(process.argv.find(arg=>arg.startsWith('--repeat='))?.split('=')[1]||1);
const failures=[];
for(const scenario of scenarios.filter(s=>!filter||JSON.stringify(s).includes(filter)).flatMap(s=>Array.from({length:repeats},(_,repeat)=>({...s,repeat:repeat+1})))){
  const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),robot=await loadTestRobot();vfs.delete();
  const targets=[{id:'red',body:'red_cube',x:.78,z:.14,h:.0225},{id:'blue',body:'blue_block',x:.78,z:0,h:.019},{id:'yellow',body:'yellow_cylinder',x:.78,z:-.14,h:.026}].map(t=>({...t,start:new THREE.Vector3(t.x,TABLE_HEIGHT+t.h,t.z),drop:new THREE.Vector3(.90,TABLE_HEIGHT+t.h,t.z),rotation:new THREE.Quaternion()}));
  if(scenario.offset){const t=targets.find(t=>t.id===scenario.id);t.start.x+=scenario.offset[0];t.start.z+=scenario.offset[1];}
  if(scenario.yaw){const t=targets.find(t=>t.id===scenario.id);t.rotation.setFromAxisAngle(new THREE.Vector3(0,1,0),scenario.yaw);}
  if(scenario.roll){const t=targets.find(t=>t.id===scenario.id);t.rotation.setFromAxisAngle(new THREE.Vector3(1,0,0),scenario.roll);t.start.y=TABLE_HEIGHT+.02;}
  const dynamics=new R1Dynamics(api,model,data,robot),c=new GraspController(api,model,data,robot,targets,dynamics);
  let maxTorque=0,maxJointError=0,maxWristError=0;const transitions=[],trackingByPhase={};
  try{
    for(let n=0;n<300;n++)c.step();
    assert(scenario.stack?c.startStack(scenario.arm):c.start(scenario.arm,scenario.id),`${c.message} ${JSON.stringify(c.planError)}`);
    let last='';
    for(let n=0;n<(scenario.stack?120000:30000)&&c.running;n++){
      c.step();if(n%100===0){const t=dynamics.telemetry(scenario.arm),phase=trackingByPhase[c.state]??={torque:0,jointError:0,wristError:0};for(const j of [...t.joints,...t.torso]){assert(Number.isFinite(j.torque)&&Number.isFinite(j.error));assert(Math.abs(j.torque)<=j.effortLimit+1e-7);maxTorque=Math.max(maxTorque,Math.abs(j.torque));maxJointError=Math.max(maxJointError,Math.abs(j.error));phase.torque=Math.max(phase.torque,Math.abs(j.torque));phase.jointError=Math.max(phase.jointError,Math.abs(j.error));}const wristError=c.wristError();maxWristError=Math.max(maxWristError,wristError);phase.wristError=Math.max(phase.wristError,wristError);}
      const key=`${c.itemIndex}/${c.selected}/${c.state}`;if(key!==last){transitions.push({time:data.time,key,measurement:c.measure(),stackcheck:scenario.stack?c.stackCheck():undefined});last=key;}
    }
    assert.equal(c.state,'complete',c.message);
    if(scenario.stack){assert(c.stackResultVerified);assert(c.stackStableSeconds>=3);assert.equal(c.stackEvidence.length,3);assert(c.stackEvidence.every(e=>e.dual&&e.lift>.035));for(let n=0;n<3000;n++)c.step();assert(c.stackCheck().stable,'Stack must remain physically supported for another 3 s');}
    else {assert(c.evidence.dual);assert(c.evidence.lift>.035);assert(c.evidence.maxSpeed<1);assert(c.measure().distance<.02);assert(c.measure().table);}
    const result={evidence:{...c.evidence},measurement:c.measure(),stackStableSeconds:c.stackStableSeconds,stackEvidence:c.stackEvidence,stackcheck:scenario.stack?c.stackCheck():undefined};
    c.reset();assert.equal(c.state,'idle');assert.equal(c.running,false);for(let n=0;n<100;n++)c.step();assert(c.start(scenario.arm,scenario.id||'blue'));for(let n=0;n<200;n++)c.step();c.stop();assert.equal(c.running,false);const command={...c.pose};for(let n=0;n<100;n++)c.step();assert.deepEqual(c.pose,command);c.reset();assert.equal(c.state,'idle');
    report({scenario,pass:true,maxTorque,maxJointError,maxWristError,trackingByPhase,...result,transitions:transitions.map(t=>t.key)});
  }catch(error){const failure={scenario,pass:false,error:error.message,state:c.state,message:c.message,selected:c.selected,itemIndex:c.itemIndex,evidence:c.evidence,measurement:c.measure(),stackEvidence:c.stackEvidence,stackcheck:scenario.stack?c.stackCheck():undefined,transitions,maxTorque,maxJointError,maxWristError};failures.push(failure);report(failure);}
  data.delete();model.delete();
}
assert.equal(failures.length,0,`${failures.length} dynamic task scenarios failed`);

// State recovery checks use the same physical scene/controller repeatedly.
// No simulation qpos changes occur except documented reset / editable start.
if(!filter||filter==='recovery'){
  const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),robot=await loadTestRobot();vfs.delete();
  const targets=[{id:'red',body:'red_cube',x:.78,z:.14,h:.0225},{id:'blue',body:'blue_block',x:1.8,z:0,h:.019},{id:'yellow',body:'yellow_cylinder',x:.78,z:-.14,h:.026}].map(t=>({...t,start:new THREE.Vector3(t.x,TABLE_HEIGHT+t.h,t.z),drop:new THREE.Vector3(.9,TABLE_HEIGHT+t.h,t.z),rotation:new THREE.Quaternion()}));
  const dynamics=new R1Dynamics(api,model,data,robot),c=new GraspController(api,model,data,robot,targets,dynamics);
  for(let n=0;n<300;n++)c.step();
  const accepted=c.startStack('right');assert.equal(accepted,false,'Unreachable physical object must not plan a valid stack');assert(c.running&&c.reaction==='shy');const reason=c.message;
  for(let n=0;n<15000&&c.running;n++)c.step();
  assert.equal(c.state,'failed');assert.equal(c.message,reason);assert(c.history.includes('shy'));assert(!c.history.includes('happy'));assert(!c.stackResultVerified);assert.equal(c.stackEvidence.length,0);
  report({scenario:{recovery:'unreachable_stack'},pass:true,reason,history:c.history});
  targets.find(t=>t.id==='blue').start.x=.78;c.reset();for(let n=0;n<300;n++)c.step();assert(c.start('right','blue'));
  for(let n=0;n<30000&&c.running;n++)c.step();assert.equal(c.state,'complete');assert(c.evidence.dual&&c.evidence.lift>.035);assert(c.measure().table);
  report({scenario:{recovery:'failure_reset_then_sort'},pass:true,evidence:c.evidence,distance:c.measure().distance});
  c.reset();for(let n=0;n<300;n++)c.step();assert(c.startStack('left'));
  for(let n=0;n<20000&&c.running&&c.state!=='transfer';n++)c.step();assert.equal(c.state,'transfer');assert(c.evidence.lift>.035);c.stop();const stoppedPose={...c.pose};
  for(let n=0;n<4000;n++)c.step();assert.equal(c.state,'stopped');assert(!c.running);assert.equal(c.reaction,null);assert.deepEqual(c.pose,stoppedPose);assert(!c.stackResultVerified);
  report({scenario:{recovery:'stop_stack_mid_transfer'},pass:true,state:c.state,object:c.measure().position.toArray(),speed:c.measure().speed});
  c.reset();for(let n=0;n<300;n++)c.step();assert(c.startStack('right'));
  for(let n=0;n<100000&&c.running&&c.state!=='happy';n++)c.step();assert.equal(c.state,'happy');assert(c.stackResultVerified);for(let n=0;n<100;n++)c.step();c.stop();
  for(let n=0;n<4000;n++)c.step();assert.equal(c.state,'stopped');assert(!c.running);assert.equal(c.reaction,null);assert.notEqual(c.reactionFinished,'happy');assert(c.stackCheck().stable);
  report({scenario:{recovery:'stop_happy_mid_gesture'},pass:true,state:c.state,history:c.history,stackcheck:c.stackCheck()});
  c.reset();assert.equal(c.state,'idle');assert(!c.running);assert.equal(c.reaction,null);assert.equal(c.history.length,0);assert(!c.stackResultVerified);assert(!c.reactionFinished);data.delete();model.delete();
}
