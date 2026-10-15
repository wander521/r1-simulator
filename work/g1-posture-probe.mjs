// Read-only pose research. Does not alter production IK/controller/model files.
import * as THREE from '../dist/vendor/three.module.min.js';
import {loadG1TestRobot} from './g1-test-robot.mjs';
import {solvePose,tcpPose,naturalArmLimits} from '../dist/g1/grasp-kinematics.mjs';
import {armJointNames,STAND_POSE,TABLE_HEIGHT,OBJECT_START_X,OBJECT_DROP_X} from '../dist/g1/g1-profile.mjs';
import {readFile} from 'node:fs/promises';

const sign=side=>side==='left'?1:-1;
const probeTilt=Number(process.argv.find(a=>a.startsWith('--tilt='))?.split('=')[1]??0)*Math.PI/180;
const hoverForward=Number(process.argv.find(a=>a.startsWith('--hover-forward='))?.split('=')[1]??0);
const tiltEase=Number(process.argv.find(a=>a.startsWith('--tilt-ease='))?.split('=')[1]??1);
const options=side=>process.argv.includes('--natural-limits')?{limits:naturalArmLimits(side)}:{};
const mirror=(side,values)=>Object.fromEntries(armJointNames(side).map((name,i)=>[name,values[i]*([1,2,4,6].includes(i)?sign(side):1)]));
export const READY={...STAND_POSE,...mirror('left',[0,.28,.10,-.70,0,.15,0]),...mirror('right',[0,.28,.10,-.70,0,.15,0])};
const REST={...STAND_POSE,...mirror('left',[0,.025,0,1.50,0,0,0]),...mirror('right',[0,.025,0,1.50,0,0,0])};
export const waistFor=(side,target,lateral=.14)=>THREE.MathUtils.clamp(Math.atan2(-target.z,target.x)-Math.asin(THREE.MathUtils.clamp(sign(side)*lateral/Math.hypot(target.x,target.z),-.99,.99)),-.85,.85);
export const naturalOrientation=(yaw,tilt=probeTilt)=>new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),yaw).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2)).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),tilt));
const r=await loadG1TestRobot(),point=name=>r.links[name].getWorldPosition(new THREE.Vector3());
const round=values=>values.map(value=>+value.toFixed(6));
function inspect(pose,side){
  r.setJointValues(pose);r.updateMatrixWorld(true);
  const center=point('torso_link'),shoulder=point(`${side}_shoulder_roll_link`),elbow=point(`${side}_elbow_link`),wrist=point(`${side}_wrist_yaw_link`),tcp=tcpPose(r,side).position;
  const inv=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-(pose.waist_yaw_joint??0));
  const local=v=>v.clone().sub(center).applyQuaternion(inv);
  const e=local(elbow),w=local(wrist);
  return {shoulder:round(shoulder.toArray()),elbow:round(elbow.toArray()),wrist:round(wrist.toArray()),tcp:round(tcp.toArray()),localElbow:round(e.toArray()),localWrist:round(w.toArray()),outwardElbow:-sign(side)*e.z,outwardWrist:-sign(side)*w.z};
}
const seeds=[[0,.25,.15,-.35,0,.10,0],[0,.2,0,-.45,0,0,0],[-.5,.3,.2,.5,0,0,0],[-.8,.25,.3,.8,0,0,0],[-1,.5,.3,1,0,0,0],[-.2,.4,.2,0,.2,0,0],[.2,.4,.4,-.6,0,0,0]];
const items=[{id:'red',z:.09,h:.0225},{id:'blue',z:0,h:.019},{id:'yellow',z:-.09,h:.026}];
const report={tilt:probeTilt,ready:{right:inspect(READY,'right'),left:inspect(READY,'left')},solutions:[],restToReady:{}};
const recordedPoses=[];
for(const side of ['right','left']){
  let minimum={height:Infinity};
  for(let k=0;k<=100;k++){
    const p=Object.fromEntries(Object.keys(READY).map(n=>[n,(REST[n]??0)+(READY[n]-(REST[n]??0))*k/100]));
    const m=inspect(p,side),[x,y,z]=m.tcp;
    if(x>=.22&&x<=.78&&Math.abs(z)<.35&&y<minimum.height)minimum={height:y,fraction:k/100,tcp:m.tcp};
  }
  report.restToReady[side]=minimum;
  for(const item of items)for(const x of [OBJECT_START_X,OBJECT_DROP_X])for(const lift of [0,.11]){
    const target=new THREE.Vector3(x+(lift?hoverForward:0),TABLE_HEIGHT+item.h+lift,item.z),yaw=waistFor(side,target),q=naturalOrientation(yaw,process.argv.includes('--level-hover')&&lift?0:probeTilt);
    const results=seeds.map(seed=>{
      const result=solvePose(r,side,target,q,{...READY,waist_yaw_joint:yaw,...mirror(side,seed)},240,options(side)),m=inspect(result.pose,side),angles=armJointNames(side).map(n=>result.pose[n]);
      // Rank task-valid solutions first; penalise an elbow across the torso,
      // inward upper arm and unnecessary yaw/roll in the redundant chain.
      const roll=angles[1]*sign(side),score=result.positionError+result.angleError*.18+Math.max(0,.13-m.outwardElbow)*4+Math.max(0,.06-roll)*.2+Math.abs(angles[2])*.001+Math.abs(angles[4])*.002;
      return {result,m,angles,score};
    }).sort((a,b)=>a.score-b.score);
    const a=results[0];report.solutions.push({side,item:item.id,x,lift,yaw,positionError:a.result.positionError,angleError:a.result.angleError,angles:round(a.angles),...a.m});
    recordedPoses.push({side,item:item.id,x,target:target.toArray(),lift,pose:a.result.pose,candidates:results.map(c=>({pose:c.result.pose,angles:round(c.angles),positionError:c.result.positionError,angleError:c.result.angleError,score:c.score}))});
  }
}
if(process.argv.includes('--physics')){
  const [{default:createMuJoCo},{G1Dynamics}]=await Promise.all([import('../dist/r1/mujoco-runtime.mjs'),import('../dist/g1/g1-dynamics.mjs')]);
  const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
  const xml=await readFile(new URL('../dist/g1/models/g1-dynamic-scene.xml',import.meta.url),'utf8'),vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),d=new G1Dynamics(api,model,data,r);vfs.delete();
  const poses={REST,WIDE:{...STAND_POSE,...mirror('left',[0,1.15,0,1.50,0,0,0]),...mirror('right',[0,1.15,0,1.50,0,0,0])},FOLD:{...STAND_POSE,...mirror('left',[0,1.15,.15,-.35,0,.1,0]),...mirror('right',[0,1.15,.15,-.35,0,.1,0])},READY};
  const compact=p=>{const pairs=new Map();for(const c of p.contacts??[]){const key=`${c.body1}/${c.body2}`;if(!pairs.has(key)||pairs.get(key).dist>c.dist)pairs.set(key,{kind:c.kind,dist:c.dist,a:c.body1,b:c.body2});}return {safe:p.safe,minDistance:p.minDistance,limitViolations:p.limitViolations,contacts:[...pairs.values()]};};
  report.collision={poses:Object.fromEntries(Object.entries(poses).map(([name,pose])=>[name,compact(d.checkPose(pose,{opening:.094}))])),routes:[],taskPoses:[]};
  for(const [a,b]of [['REST','WIDE'],['WIDE','FOLD'],['FOLD','READY'],['REST','READY']]){
    let unsafe=0,first=null,worst=null;
    for(let k=0;k<=100;k++){
      const pose=Object.fromEntries(Object.keys(poses[a]).map(n=>[n,poses[a][n]+(poses[b][n]-poses[a][n])*k/100])),p=d.checkPose(pose,{opening:.094});
      if(!p.safe){unsafe++;first??={t:k/100,...compact(p)};if(!worst||p.minDistance<worst.minDistance)worst={t:k/100,...compact(p)};}
    }
    report.collision.routes.push({a,b,unsafe,first,worst});
  }
  const objNames={red:'red_cube',blue:'blue_block',yellow:'yellow_cylinder'};
  const initial=data.qpos.slice();
  const chosen=[];
  for(const t of recordedPoses){
    data.qpos.set(initial);const item=items.find(i=>i.id===t.item),body=api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,objNames[t.item]),q=model.jnt_qposadr[model.body_jntadr[body]];
    data.qpos.set([t.target[0],-item.z,TABLE_HEIGHT+item.h+t.lift,1,0,0,0],q);api.mj_forward(model,data);
    const yaw=t.pose.waist_yaw_joint,width=t.item==='yellow'?.04:t.item==='red'?.045*(Math.abs(Math.cos(yaw))+Math.abs(Math.sin(yaw))):.032*Math.abs(Math.cos(yaw))+.052*Math.abs(Math.sin(yaw));
    const candidates=t.candidates.map(c=>({...c,check:compact(d.checkPose(c.pose,{opening:Math.min(.094,width+.018),ignoreObjectBodies:[objNames[t.item]]}))})),selected=candidates.find(c=>c.check.safe&&c.positionError<.001&&c.angleError<.01)??candidates[0];
    report.collision.taskPoses.push({...t,pose:undefined,candidates:undefined,angles:selected.angles,positionError:selected.positionError,angleError:selected.angleError,safeCandidates:candidates.filter(c=>c.check.safe&&c.positionError<.001&&c.angleError<.01).length,check:selected.check});
    chosen.push({...t,pose:selected.pose});
  }
  if(process.argv.includes('--paths')){
    report.collision.paths=[];
    for(const side of ['right','left'])for(const item of items){
      const key=(x,lift)=>chosen.find(p=>p.side===side&&p.item===item.id&&p.x===x&&p.lift===lift);
      const grasp=key(OBJECT_START_X,0),above=key(OBJECT_START_X,.11),aboveDrop=key(OBJECT_DROP_X,.11),drop=key(OBJECT_DROP_X,0);
      let seed=above.pose;const phases=[];
      const aligned={...READY,waist_yaw_joint:above.pose.waist_yaw_joint};
      for(const [name,pa,pb]of [['ready-turn',READY,aligned],['approach',aligned,above.pose]]){
        let unsafe=0,first=null;data.qpos.set(initial);api.mj_forward(model,data);
        for(let k=0;k<=60;k++){
          const f=k/60,pose=Object.fromEntries(Object.keys(pa).map(n=>[n,pa[n]+(pb[n]-pa[n])*f])),check=d.checkPose(pose,{opening:.094});
          if(!check.safe){unsafe++;first??={t:f,check:compact(check)};}
        }
        phases.push({name,unsafe,first});
      }
      for(const [name,a,b,ta,tb]of [['descend',above,grasp,0,probeTilt],['lift',grasp,above,probeTilt,0],['transfer',above,aboveDrop,0,0],['place',aboveDrop,drop,0,probeTilt],['retreat',drop,aboveDrop,probeTilt,0]]){
        let unsafe=0,ikFail=0,maxJump=0,maxWristRoll=0,maxShoulderYaw=0,first=null;
        for(let k=0;k<=60;k++){
          const f=k/60,target=new THREE.Vector3().fromArray(a.target).lerp(new THREE.Vector3().fromArray(b.target),f),yaw=waistFor(side,target),tiltF=name==='descend'?Math.min(1,f*tiltEase):name==='retreat'?Math.max(0,(f-(1-1/tiltEase))*tiltEase):f,quat=naturalOrientation(yaw,ta+(tb-ta)*tiltF),old=seed;
          const result=solvePose(r,side,target,quat,{...seed,waist_yaw_joint:yaw},100,options(side));seed=result.pose;
          const body=api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,objNames[item.id]),q=model.jnt_qposadr[model.body_jntadr[body]];
          // Unheld objects remain on the table before closure / after release.
          // Only a hypothetical held-object path follows TCP during carry.
          const objectPoint=name==='descend'?grasp.target:name==='retreat'?drop.target:target.toArray();
          data.qpos.set(initial);data.qpos.set([objectPoint[0],-objectPoint[2],objectPoint[1],1,0,0,0],q);api.mj_forward(model,data);
          const width=item.id==='yellow'?.04:item.id==='red'?.045*(Math.abs(Math.cos(yaw))+Math.abs(Math.sin(yaw))):.032*Math.abs(Math.cos(yaw))+.052*Math.abs(Math.sin(yaw));
          const check=d.checkPose(seed,{opening:width+.018,ignoreObjectBodies:[objNames[item.id]]});
          if(!check.safe){unsafe++;first??={t:f,check:compact(check)};}if(result.positionError>.001||result.angleError>.01)ikFail++;
          maxJump=Math.max(maxJump,...armJointNames(side).map(n=>Math.abs(seed[n]-old[n])));maxWristRoll=Math.max(maxWristRoll,Math.abs(seed[`${side}_wrist_roll_joint`]));maxShoulderYaw=Math.max(maxShoulderYaw,Math.abs(seed[`${side}_shoulder_yaw_joint`]));
        }
        phases.push({name,unsafe,ikFail,maxJump,maxWristRoll,maxShoulderYaw,first});
      }
      report.collision.paths.push({side,item:item.id,phases});
    }
  }
  d.dispose?.();data.delete();model.delete();
}
console.log(JSON.stringify(process.argv.includes('--summary')?{tilt:report.tilt,ready:report.ready,collision:report.collision}:report,null,2));
