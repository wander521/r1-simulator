import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController} from '../dist/c5/grasp-controller.mjs';
import {C5Dynamics} from '../dist/c5/c5-dynamics.mjs';
import {TABLE_HEIGHT,OBJECT_START_X,OBJECT_DROP_X,OBJECT_LATERAL_OFFSET,armJointNames} from '../dist/c5/c5-profile.mjs';
import {loadC5TestRobot} from './c5-test-robot.mjs';
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
const xml=await readFile(new URL('../dist/c5/models/c5-dynamic-scene.xml',import.meta.url),'utf8'),vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),robot=await loadC5TestRobot();vfs.delete();
const targets=[{id:'red',body:'red_cube',z:OBJECT_LATERAL_OFFSET,h:.0225},{id:'blue',body:'blue_block',z:0,h:.019},{id:'yellow',body:'yellow_cylinder',z:-OBJECT_LATERAL_OFFSET,h:.026}].map(t=>({...t,start:new THREE.Vector3(OBJECT_START_X,TABLE_HEIGHT+t.h,t.z),drop:new THREE.Vector3(OBJECT_DROP_X,TABLE_HEIGHT+t.h,t.z),rotation:new THREE.Quaternion()}));
const d=new C5Dynamics(api,model,data,robot),c=new GraspController(api,model,data,robot,targets,d),id=process.argv[2]||'blue';
for(let n=0;n<300;n++)c.step();
if(process.argv.includes('--plan-only')){
  c.selected=id;
  for(let branch=0;branch<9;branch++)try{c.plan(targets.find(t=>t.id===id),branch);console.log(JSON.stringify({branch,pass:true,approach:c.planData.approach,descend:c.planData.descend.at(-1)}));}catch(e){console.log(JSON.stringify({branch,error:e.message,detail:e.detail}));}
}else{
  console.log(JSON.stringify({start:id==='stack'?c.startStack('left'):c.start('left',id),message:c.message,planError:c.planError}));let last='';
  for(let n=0;n<120000&&c.running;n++){
    c.step();const key=c.itemIndex+'/'+c.selected+'/'+c.state;
    if(key!==last||(['closing','place'].includes(c.state)&&n%500===0)){const m=c.measure();console.log(JSON.stringify({t:data.time,key,evidence:c.evidence,m:{count:m.count,forces:m.forces,support:m.support,p:m.position.toArray(),speed:m.speed,distance:m.distance},wristError:c.wristError(),arm:armJointNames().map(n=>d.targets[n])}));last=key;}
  }
  console.log(JSON.stringify({result:c.state,message:c.message,planError:c.planError,evidence:c.evidence,stack:c.stackCheck()}));
}
d.dispose();data.delete();model.delete();
