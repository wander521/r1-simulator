import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController,TABLE_HEIGHT} from '../dist/r1/grasp-controller.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
const xml=await readFile(new URL('../dist/r1/models/r1-contact-scene.xml',import.meta.url),'utf8');
const scenarios=[];
for(const arm of ['right','left'])for(const id of ['red','blue','yellow'])scenarios.push({arm,id});
scenarios.push({arm:'right',id:'blue',offset:[-.025,.035]},{arm:'left',id:'yellow',offset:[.02,-.025]});
scenarios.push({arm:'right',id:'blue',yaw:.4},{arm:'left',id:'yellow',roll:Math.PI/2});
for(const scenario of scenarios){
  const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),robot=await loadTestRobot();vfs.delete();
  const targets=[{id:'red',body:'red_cube',x:.78,z:.14,h:.0225},{id:'blue',body:'blue_block',x:.78,z:0,h:.019},{id:'yellow',body:'yellow_cylinder',x:.78,z:-.14,h:.026}].map(t=>({...t,start:new THREE.Vector3(t.x,TABLE_HEIGHT+t.h,t.z),drop:new THREE.Vector3(.90,TABLE_HEIGHT+t.h,t.z),rotation:new THREE.Quaternion()}));
  if(scenario.offset){const t=targets.find(t=>t.id===scenario.id);t.start.x+=scenario.offset[0];t.start.z+=scenario.offset[1];}
  if(scenario.yaw){const t=targets.find(t=>t.id===scenario.id);t.rotation.setFromAxisAngle(new THREE.Vector3(0,1,0),scenario.yaw);}
  if(scenario.roll){const t=targets.find(t=>t.id===scenario.id);t.rotation.setFromAxisAngle(new THREE.Vector3(1,0,0),scenario.roll);t.start.y=TABLE_HEIGHT+.02;}
  const c=new GraspController(api,model,data,robot,targets);
  for(let n=0;n<300;n++)c.step();
  assert(c.start(scenario.arm,scenario.id),`${JSON.stringify(scenario)}: ${c.message} ${JSON.stringify(c.planError)}`);
  for(let n=0;n<30000&&c.running;n++)c.step();
  console.log(JSON.stringify({scenario,state:c.state,message:c.message,stages:c.history,evidence:c.evidence,distance:c.measure().distance,speed:c.measure().speed}));
  assert.equal(c.state,'complete');assert(c.evidence.dual);assert(c.evidence.lift>.035);assert(c.evidence.maxSpeed<1);assert(c.measure().distance<.02);assert(c.measure().table);
  data.delete();model.delete();
}
