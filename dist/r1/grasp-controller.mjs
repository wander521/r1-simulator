import * as THREE from '../vendor/three.module.min.js';
import {solvePose} from './grasp-kinematics.mjs?v=20261015-3';

export const TABLE_HEIGHT = .692;
export const REST_TORSO = {torso_joint1:0,torso_joint2:.06,torso_joint3:0,torso_joint4:0};
export const CROUCH = {torso_joint1:1.45,torso_joint2:-2.35,torso_joint3:-.8,torso_joint4:0};
export const REST = {}, SPREAD = {};
for (const arm of ['right','left']) {
  const sign = arm === 'right' ? 1 : -1;
  REST[arm] = Object.fromEntries([-1.57*sign,2.20,-1.40,0,0,0].map((v,i)=>[`${arm}_arm_joint${i+1}`,v]));
  SPREAD[arm] = Object.fromEntries([.64*sign,1.04,-1.35,.1*sign,.3*sign,.02*sign].map((v,i)=>[`${arm}_arm_joint${i+1}`,v]));
}
const idlePose = {...REST_TORSO,...REST.right,...REST.left};
const smooth = t => {t=THREE.MathUtils.clamp(t,0,1);return t*t*(3-2*t);};
const mix = (a,b,t) => Object.fromEntries(Object.keys(b).map(k=>[k,THREE.MathUtils.lerp(a[k]??b[k],b[k],t)]));
const toMJ = p => [p.x,-p.z,p.y];
const basis = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI/2);
const gripRotation = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,1),new THREE.Vector3(0,-1,0)));

export class GraspController {
  constructor(api,model,data,robot,targets) {
    Object.assign(this,{api,model,data,robot,targets}); this.state='idle'; this.running=false;this.time=0;this.elapsed=0;this.arm='right';this.selected='blue';this.opening=.05;this.pose={...idlePose};this.evidence={};this.history=[];
    this.id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name);
    this.bodies=new Map(targets.map(t=>[t.id,this.id('mjOBJ_BODY',t.body)]));
    this.fingers={}; this.anchors={};
    for(const arm of ['right','left']) {
      this.anchors[arm]=model.body_mocapid[this.id('mjOBJ_BODY',`${arm}_anchor`)];
      this.fingers[arm]=[1,2].map(i=>{const j=this.id('mjOBJ_JOINT',`${arm}_finger${i}`);return {q:model.jnt_qposadr[j],d:model.jnt_dofadr[j],body:this.id('mjOBJ_BODY',`${arm}_pad${i}`),actuator:this.id('mjOBJ_ACTUATOR',`${arm}_motor${i}`)};});
    }
    this.reset();
  }
  objectPose(id=this.selected) {
    const b=this.bodies.get(id),p=this.data.xpos.subarray(b*3,b*3+3),q=this.data.xquat.subarray(b*4,b*4+4);
    // Shapes are authored Y-up in Three; this conjugation preserves that local
    // basis (unlike the URDF wrist, whose local coordinates remain Z-up).
    const rotation=basis.clone().invert().multiply(new THREE.Quaternion(q[1],q[2],q[3],q[0])).multiply(basis);
    return {position:new THREE.Vector3(p[0],p[2],-p[1]),quaternion:rotation};
  }
  setObject(id,position,quaternion=new THREE.Quaternion()) {
    const b=this.bodies.get(id),j=this.model.body_jntadr[b],q=this.model.jnt_qposadr[j],d=this.model.jnt_dofadr[j];
    const rotation=basis.clone().multiply(quaternion).multiply(basis.clone().invert());
    this.data.qpos.set([...toMJ(position),rotation.w,rotation.x,rotation.y,rotation.z],q);this.data.qvel.fill(0,d,d+6);this.api.mj_forward(this.model,this.data);
  }
  applyPose(pose) {
    this.pose={...pose}; this.robot.setJointValues(pose);this.robot.updateMatrixWorld(true);
    for(const arm of ['right','left']) {
      const link=this.robot.getObjectByName(`${arm}_gripper_link`),p=link.getWorldPosition(new THREE.Vector3()),q=basis.clone().multiply(link.getWorldQuaternion(new THREE.Quaternion())),m=this.anchors[arm];
      this.data.mocap_pos.set(toMJ(p),m*3); this.data.mocap_quat.set([q.w,q.x,q.y,q.z],m*4);
    }
  }
  syncFingers() {
    for(const arm of ['right','left']) this.fingers[arm].forEach((f,i)=>this.robot.setJointValues({[`${arm}_gripper_finger_joint${i+1}`]:this.data.qpos[f.q]}));
    this.robot.updateMatrixWorld(true);
  }
  reset() {
    this.api.mj_resetData(this.model,this.data);this.running=false;this.state='idle';this.message='双臂垂直待命';this.elapsed=0;this.opening=.05;this.evidence={};this.history=[];
    for(const t of this.targets)this.setObject(t.id,t.start,t.rotation);
    for(const arm of ['right','left'])for(const f of this.fingers[arm]){this.data.qpos[f.q]=.05;this.data.ctrl[f.actuator]=.05;}
    this.applyPose(idlePose);
    for(const arm of ['right','left']) {
      const b=this.id('mjOBJ_BODY',`${arm}_palm`),q=this.model.jnt_qposadr[this.model.body_jntadr[b]],m=this.anchors[arm];
      this.data.qpos.set(this.data.mocap_pos.subarray(m*3,m*3+3),q);this.data.qpos.set(this.data.mocap_quat.subarray(m*4,m*4+4),q+3);
    }
    this.api.mj_forward(this.model,this.data);this.syncFingers();
  }
  measure() {
    const b=this.bodies.get(this.selected),target=this.targets.find(t=>t.id===this.selected),fingers=this.fingers[this.arm],hits=new Set(),forces=[0,0],contacts=[];
    let table=false;
    const list=this.data.contact;
    for(let i=0;i<Math.min(this.data.ncon,list.size());i++) {
      const c=list.get(i),a=this.model.geom_bodyid[c.geom1],d=this.model.geom_bodyid[c.geom2];if(a!==b&&d!==b){c.delete();continue;}
      const other=a===b?d:a;const finger=fingers.findIndex(f=>f.body===other);
      // This WASM build copies JS output arrays instead of writing them back.
      // With the elliptic cone, efc_address directly indexes normal force.
      const force=c.efc_address>=0?Math.abs(this.data.efc_force[c.efc_address]):0;
      if(finger>=0&&force>.015){hits.add(finger);forces[finger]+=force;}
      if(other===this.id('mjOBJ_BODY','task_table'))table=true;
      contacts.push({position:new THREE.Vector3(c.pos[0],c.pos[2],-c.pos[1]),force});
      c.delete();
    }
    list.delete();
    const p=this.objectPose().position,j=this.model.body_jntadr[b],d=this.model.jnt_dofadr[j];
    return {count:hits.size,force:forces.reduce((s,v)=>s+v,0),forces,table,position:p,speed:Math.hypot(...this.data.qvel.subarray(d,d+3)),distance:p.distanceTo(target.drop),contacts};
  }
  plan(target,twist=.3,yaw=0) {
    this.crouch={...CROUCH,torso_joint4:this.arm==='right'?twist:-twist};
    this.robot.setJointValues({...this.crouch,...SPREAD.right,...SPREAD.left});
    const seedValues=this.arm==='right'?[-.55,3.13,-1.93,1.09,1.24,1.74]:[.55,3.13,-1.93,-1.09,-1.24,-1.74];
    let seed=Object.fromEntries(seedValues.map((v,i)=>[`${this.arm}_arm_joint${i+1}`,v]));
    const orientation=gripRotation.clone().premultiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),yaw));
    const start=this.objectPose(target.id).position,drop=target.drop.clone();
    // Keep the measured resting height when an object has been rotated onto
    // another face; the drop frame denotes XY, not a forced object rotation.
    drop.y=start.y;
    // Side-on pinch keeps the jaws horizontal and the wrist clear of the
    // tabletop with the user's existing crouch. The original position-only
    // IK had no such constraint and often pointed the jaws upward.
    const solve=p=>{let result=solvePose(this.robot,this.arm,p,orientation,seed);
      if(result.positionError>.003||result.angleError>.035) {
        for(let k=0;k<12;k++) {
          const guess=Object.fromEntries([Math.sin(k)*1.5,1.5+Math.cos(k)*1.3,-1.7,Math.sin(k*2)*2,0,Math.cos(k)*2].map((v,i)=>[`${this.arm}_arm_joint${i+1}`,v]));
          const candidate=solvePose(this.robot,this.arm,p,orientation,guess);
          if(candidate.positionError+.18*candidate.angleError<result.positionError+.18*result.angleError)result=candidate;
          if(result.positionError<.001&&result.angleError<.01)break;
        }
      }
      if(result.positionError>.003||result.angleError>.035){const error=new Error('目标超出当前蹲姿可稳定抓取的范围，请向桌面中央移动');error.detail={position:p.toArray(),positionError:result.positionError,angleError:result.angleError};throw error;}seed=result.pose;return {...this.crouch,...SPREAD.right,...SPREAD.left,...seed};};
    const above=start.clone().add(new THREE.Vector3(0,.13,0)),aboveDrop=drop.clone().add(new THREE.Vector3(0,.13,0));
    const approach=solve(above),line=(from,to)=>{const samples=[];for(let i=0;i<=24;i++){const p=from.clone().lerp(to,i/24);samples.push(solve(p));}return samples;};
    const descend=line(above,start),lift=line(start,above),transfer=line(above,aboveDrop),place=line(aboveDrop,drop),retreat=line(drop,aboveDrop);
    this.planData={start,drop,approach,descend,lift,transfer,place,retreat};
  }
  start(arm,id) {
    if(this.running)return false;this.arm=arm;this.selected=id;
    const before={...this.pose};
    let error;
    search:for(const yaw of [0,.35,-.35,.7,-.7])for(const twist of [.3,.4,.2,.1,0]) {
      try{this.plan(this.targets.find(t=>t.id===id),twist,yaw*(arm==='right'?1:-1));error=null;break search;}catch(e){error=e;}
    }
    if(error){this.applyPose(before);this.state='failed';this.message=error.message;this.planError=error.detail;return false;}
    this.applyPose(before);this.running=true;this.opening=.05;this.evidence={lift:0,maxSpeed:0,dual:false,stable:0};this.history=[];this.transition('spread');return true;
  }
  transition(state) {this.state=state;this.elapsed=0;this.from={...this.pose};this.evidence.stable=0;this.history.push(state);}
  stop(message='已停止；物体继续按重力落桌') {this.running=false;this.state='stopped';this.opening=.05;this.message=message;}
  fail(message) {this.stop(message);this.state='failed';}
  updateMotion() {
    const t=this.elapsed,p=this.planData;
    const joint=(to,duration)=>this.applyPose(mix(this.from,to,smooth(t/duration)));
    const path=(samples,duration)=>{const v=smooth(t/duration)*(samples.length-1),i=Math.min(samples.length-2,Math.floor(v));this.applyPose(mix(samples[i],samples[i+1],v-i));};
    if(this.state==='spread')joint({...REST_TORSO,...SPREAD.right,...SPREAD.left},1.4);
    if(this.state==='crouch')joint({...this.crouch,...SPREAD.right,...SPREAD.left},1.5);
    if(this.state==='approach')joint(p.approach,2.4);
    if(this.state==='descend')path(p.descend,1.7);
    if(this.state==='lift')path(p.lift,1.8);
    if(this.state==='transfer')path(p.transfer,2.0);
    if(this.state==='place')path(p.place,1.8);
    if(this.state==='retreat')path(p.retreat,1.6);
    if(this.state==='return')joint(idlePose,2.6);
  }
  step(dt=.001) {
    this.time+=dt;
    if(this.running) {
      this.elapsed+=dt;this.updateMotion();
      if(this.state==='closing')this.opening=THREE.MathUtils.lerp(.05,0,smooth(this.elapsed/1.0));
      if(this.state==='release')this.opening=THREE.MathUtils.lerp(0,.05,smooth(this.elapsed/.6));
    }
    for(const arm of ['right','left'])for(const f of this.fingers[arm])this.data.ctrl[f.actuator]=arm===this.arm?this.opening:.05;
    this.api.mj_step(this.model,this.data);
    // State transitions and telemetry run at 100 Hz; all dynamics and wrist
    // interpolation still run at the same fixed 1 ms step regardless of FPS.
    if(Math.round(this.time/dt)%10!==0)return;
    const m=this.measure();this.measurement=m;
    if(!this.running)return;
    this.evidence.maxSpeed=Math.max(this.evidence.maxSpeed,m.speed);const t=this.elapsed;
    if(this.state==='spread'&&t>=1.4)this.transition('crouch');
    else if(this.state==='crouch'&&t>=1.5)this.transition('approach');
    else if(this.state==='approach'&&t>=2.4)this.transition('descend');
    else if(this.state==='descend'&&t>=1.7)this.transition('closing');
    else if(this.state==='closing') {
      this.evidence.stable=m.count===2?this.evidence.stable+.01:0;
      if(t>1&&this.evidence.stable>.18){this.evidence.dual=true;this.transition('lift');}
      else if(t>2.2)this.fail('夹持失败：双指未建立稳定接触，请调整物体位置后重试');
    }else if(this.state==='lift') {
      this.evidence.lift=Math.max(this.evidence.lift,m.position.y-this.planData.start.y);
      if(t>=1.8){if(this.evidence.lift>.035&&m.count===2)this.transition('transfer');else this.fail('夹持滑脱：目标未稳定抬离桌面');}
    }else if(this.state==='transfer'&&t>=2)this.transition('place');
    else if(this.state==='place') {
      this.evidence.stable=m.table&&m.distance<.012&&m.speed<.035?this.evidence.stable+.01:0;
      if(t>=1.8&&this.evidence.stable>.12)this.transition('release');else if(t>3)this.fail('放置未稳定：目标尚未在对应颜色框内落桌');
    }else if(this.state==='release'&&t>.75)this.transition('retreat');
    else if(this.state==='retreat'&&t>=1.6)this.transition('settling');
    else if(this.state==='settling'&&t>.4) {
      if(m.table&&m.distance<.02&&m.speed<.025)this.transition('return');else if(t>1.5)this.fail('物体松爪后未在放置框内稳定');
    }else if(this.state==='return'&&t>=2.6){this.running=false;this.state='complete';this.message='已夹取并放入对应颜色框';}
    if(['lift','transfer','place'].includes(this.state)&&m.position.y<TABLE_HEIGHT-.03)this.fail('目标已滑落，请复位后重试');
  }
}
