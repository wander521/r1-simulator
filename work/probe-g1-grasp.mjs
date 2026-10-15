import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController} from '../dist/g1/grasp-controller.mjs';
import {G1Dynamics} from '../dist/g1/g1-dynamics.mjs';
import {TABLE_HEIGHT,OBJECT_START_X,OBJECT_DROP_X,OBJECT_LATERAL_OFFSET,armJointNames} from '../dist/g1/g1-profile.mjs';
import {loadG1TestRobot} from './g1-test-robot.mjs';
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
const xml=await readFile(new URL('../dist/g1/models/g1-dynamic-scene.xml',import.meta.url),'utf8'),vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),robot=await loadG1TestRobot();vfs.delete();
const targets=[{id:'red',body:'red_cube',z:OBJECT_LATERAL_OFFSET,h:.0225},{id:'blue',body:'blue_block',z:0,h:.019},{id:'yellow',body:'yellow_cylinder',z:-OBJECT_LATERAL_OFFSET,h:.026}].map(t=>({...t,start:new THREE.Vector3(OBJECT_START_X,TABLE_HEIGHT+t.h,t.z),drop:new THREE.Vector3(OBJECT_DROP_X,TABLE_HEIGHT+t.h,t.z),rotation:new THREE.Quaternion()}));
const d=new G1Dynamics(api,model,data,robot),c=new GraspController(api,model,data,robot,targets,d),arm=process.argv[2]||'right',id=process.argv[3]||'blue';
for(let n=0;n<300;n++)c.step();
if(process.argv.includes('--plan-only')){
  c.arm=arm;c.selected=id;
  for(let branch=0;branch<6;branch++)try{c.plan(targets.find(t=>t.id===id),0,branch,.14);console.log(JSON.stringify({branch,pass:true}));}catch(error){console.log(JSON.stringify({branch,error:error.message,detail:error.detail}));}
  d.dispose();data.delete();model.delete();process.exit(0);
}
console.log('START',id==='stack'?c.startStack(arm):c.start(arm,id),c.message,c.planError);
let last='';
for(let n=0;n<120000&&c.running;n++){
  c.step();const key=`${c.itemIndex}/${c.selected}/${c.state}`;
  if(key!==last||(['closing','place'].includes(c.state)&&n%500===0)){
    console.log(JSON.stringify({t:data.time,key,evidence:c.evidence,m:c.measure(),wristError:c.wristError(),arm:armJointNames(arm).map(n=>d.targets[n]),fingers:d.telemetry(arm).fingers.map(j=>({actual:j.actual,target:j.target,force:j.torque}))}));last=key;
  }
}
console.log('RESULT',c.state,c.message,c.planError,JSON.stringify(c.evidence),JSON.stringify(c.stackCheck()));d.dispose();data.delete();model.delete();
