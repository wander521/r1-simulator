import * as THREE from '../vendor/three.module.min.js';
import {solvePose,tcpPose,naturalArmLimits,naturalArmPose} from './grasp-kinematics.mjs';
import {TABLE_HEIGHT,STAND_POSE,armJointNames,fingerJointNames,fingerBodyNames,fingerActuatorNames,openingToQ,qToOpening,GRIPPER_Q_MIN,GRIPPER_Q_MAX} from './g1-profile.mjs';
export {TABLE_HEIGHT} from './g1-profile.mjs';

export const REST_TORSO={...STAND_POSE};
export const REST={},SPREAD={},RAISED={},READY={};
for(const side of ['right','left']){
  const s=side==='left'?1:-1;
  REST[side]=Object.fromEntries(armJointNames(side).map((name,i)=>[name,[0,.025*s,0,1.50,0,0,0][i]]));
  SPREAD[side]=Object.fromEntries(armJointNames(side).map((name,i)=>[name,[0,1.15*s,0,1.50,0,0,0][i]]));
  RAISED[side]=Object.fromEntries(armJointNames(side).map((name,i)=>[name,[0,1.15*s,.15*s,-.35,0,.10,0][i]]));
  READY[side]=Object.fromEntries(armJointNames(side).map((name,i)=>[name,[0,.28*s,.10*s,-.70,0,.15,0][i]]));
}
const idlePose={...STAND_POSE,...REST.right,...REST.left};
const OPEN=qToOpening(GRIPPER_Q_MAX),CLOSED=qToOpening(GRIPPER_Q_MIN);
const smooth=t=>{t=THREE.MathUtils.clamp(t,0,1);return t*t*(3-2*t);};
const mix=(a,b,t)=>Object.fromEntries(Object.keys(b).map(k=>[k,THREE.MathUtils.lerp(a[k]??b[k],b[k],t)]));
const toMJ=p=>[p.x,-p.z,p.y];
const basis=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI/2);
// G1/Dex1's native +Z is the palm back, not the upside-down R1 tool frame.
const sidePinch=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2);

export class GraspController{
  get opening(){return this.openings[this.arm];}
  set opening(value){this.openings[this.arm]=value;}
  constructor(api,model,data,robot,targets,dynamics){
    Object.assign(this,{api,model,data,robot,targets,dynamics});this.arm='right';this.selected='blue';
    this.id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name);
    this.bodies=new Map(targets.map(t=>[t.id,this.id('mjOBJ_BODY',t.body)]));
    this.fingers={};for(const arm of ['right','left'])this.fingers[arm]=fingerJointNames(arm).map((name,i)=>{
      const j=this.id('mjOBJ_JOINT',name),body=this.id('mjOBJ_BODY',fingerBodyNames(arm)[i]),actuator=this.id('mjOBJ_ACTUATOR',fingerActuatorNames(arm)[i]);
      if(j<0||body<0||actuator<0)throw new Error(`Dex1物理夹指缺失：${name}`);
      return {q:model.jnt_qposadr[j],d:model.jnt_dofadr[j],body,actuator};
    });this.reset();
  }
  objectPose(id=this.selected){
    const b=this.bodies.get(id),p=this.data.xpos.subarray(b*3,b*3+3),q=this.data.xquat.subarray(b*4,b*4+4);
    return {position:new THREE.Vector3(p[0],p[2],-p[1]),quaternion:basis.clone().invert().multiply(new THREE.Quaternion(q[1],q[2],q[3],q[0])).multiply(basis)};
  }
  // Object coordinates can be set only by reset or explicit idle editing.
  // Task motion never writes an object's position/velocity or attaches it.
  setObject(id,position,quaternion=new THREE.Quaternion()){
    if(this.running)return false;
    const b=this.bodies.get(id),j=this.model.body_jntadr[b],q=this.model.jnt_qposadr[j],d=this.model.jnt_dofadr[j],rotation=basis.clone().multiply(quaternion).multiply(basis.clone().invert());
    this.data.qpos.set([...toMJ(position),rotation.w,rotation.x,rotation.y,rotation.z],q);this.data.qvel.fill(0,d,d+6);this.api.mj_forward(this.model,this.data);return true;
  }
  applyPose(pose){this.pose={...STAND_POSE,...pose};this.dynamics.setTargets(this.pose);}
  validateSegment(from,to,{opening=this.openings,openingFrom=null,ignoreSelected=true,objectPoseFor=null}={}){
    const count=Math.max(2,Math.ceil(Math.max(...Object.keys(to).map(n=>Math.abs((to[n]??0)-(from[n]??to[n]))))/.045));
    for(let i=0;i<=count;i++){
      const pose=mix(from,to,i/count),gaps=typeof opening==='number'?{left:opening,right:opening}:opening;
      const gapSteps=openingFrom?Math.max(1,...['left','right'].map(side=>Math.ceil(Math.abs(gaps[side]-openingFrom[side])/.002))):0;
      for(let k=0;k<=gapSteps;k++){
        // Check the complete gap envelope at each swept arm pose, not only
        // the final open/closed fingers. Opening is itself a robot motion.
        const gap=openingFrom?mix(openingFrom,gaps,k/gapSteps):gaps;
        const check=this.dynamics.checkPose(pose,{opening:gap,ignoreObjectBodies:ignoreSelected?[this.bodies.get(this.selected)]:[],objectPoses:objectPoseFor?.(pose)});
        if(!check.safe){const error=new Error('手臂或夹爪路径与身体、桌面或邻近物体干涉，请调整目标位置');error.detail={sample:i,count,contacts:check.contacts.slice(0,3),limits:check.limitViolations};throw error;}
      }
    }return true;
  }
  selectArm(side){
    if(this.running||!['left','right'].includes(side))return false;
    // Switching the UI must never silently open the previously selected hand.
    this.arm=side;return true;
  }
  setOpenings(gaps){
    const to={...this.openings,...gaps};
    try{
      for(const side of ['left','right']){
        if(!Number.isFinite(to[side]))throw new Error('夹爪开口数值无效');
        to[side]=THREE.MathUtils.clamp(to[side],CLOSED,OPEN);
      }
      const actual=this.dynamics.telemetry(this.arm).allJoints;
      const actualPose=Object.fromEntries(actual.filter(j=>j.unit==='rad').map(j=>[j.name,j.actual]));
      const actualGaps=Object.fromEntries(['left','right'].map(side=>[side,qToOpening(this.fingers[side].reduce((sum,f)=>sum+this.data.qpos[f.q],0)/2)]));
      this.validateSegment(this.pose,this.pose,{opening:to,openingFrom:this.openings});
      this.validateSegment(actualPose,actualPose,{opening:to,openingFrom:actualGaps});
      this.openings=to;return true;
    }catch(error){this.message=`夹爪动作已拒绝：${error.message}；请先将手臂移离障碍`;this.planError=error.detail;return false;}
  }
  setManualOpening(value){return !this.running&&this.setOpenings({[this.arm]:value});}
  predictedObject(pose,plan,phase){
    // Prediction belongs only to the independent collision-check data. Never
    // writes the running scene: the actual object's motion is contact-driven.
    let position,quaternion;
    if(phase==='retreat'){
      position=plan.drop.clone();quaternion=plan.orientation.clone().multiply(plan.graspOrientation.clone().invert()).multiply(plan.objectRotation);
    }else{
      this.robot.setJointValues(pose);this.robot.updateMatrixWorld(true);const tcp=tcpPose(this.robot,this.arm);
      position=tcp.position;quaternion=tcp.quaternion.multiply(plan.graspOrientation.clone().invert()).multiply(plan.objectRotation);
    }
    return [{body:this.bodies.get(this.selected),position:position.toArray(),quaternion:quaternion.toArray()}];
  }
  applyManualPose(pose){
    if(this.running)return false;
    try{this.validateSegment(this.pose,pose,{ignoreSelected:false});this.applyPose(pose);return true;}
    catch(error){this.message=`手动目标已拒绝：${error.message}`;return false;}
  }
  syncFingers(){this.dynamics.syncRobot();}
  reset(){
    this.api.mj_resetData(this.model,this.data);this.running=false;this.state='idle';this.message='固定站位 · 双臂垂下待命';this.elapsed=0;this.time=0;this.tick=0;this.openings={left:OPEN,right:OPEN};this.history=[];this.task='putaway';this.itemIndex=0;this.stackEvidence=[];this.stackStableSeconds=0;this.stackResultVerified=false;this.reaction=null;this.reactionFinished=null;this.planData=null;this.planError=null;this.measurement=null;this.evidence={lift:0,maxSpeed:0,dual:false,stable:0};
    for(const t of this.targets)this.setObject(t.id,t.start,t.rotation);
    const fingers=Object.fromEntries(['right','left'].flatMap(a=>fingerJointNames(a).map(name=>[name,GRIPPER_Q_MAX])));
    this.pose={...idlePose};this.dynamics.reset({...idlePose,...fingers});this.stackSnapshot=null;this.syncFingers();
  }
  measure(){
    const b=this.bodies.get(this.selected),target=this.targets.find(t=>t.id===this.selected),hits=new Set(),forces=[0,0],contacts=[];
    const tableBody=this.id('mjOBJ_BODY','task_table'),required=this.task==='stack'&&this.itemIndex>0?this.bodies.get(['blue','red','yellow'][this.itemIndex-1]):tableBody;
    let table=false,support=false;const list=this.data.contact;
    for(let i=0;i<Math.min(this.data.ncon,list.size());i++){
      const c=list.get(i),a=this.model.geom_bodyid[c.geom1],d=this.model.geom_bodyid[c.geom2];if(a!==b&&d!==b){c.delete();continue;}
      const other=a===b?d:a,finger=this.fingers[this.arm].findIndex(f=>f.body===other),force=c.efc_address>=0?Math.abs(this.data.efc_force[c.efc_address]):0;
      if(finger>=0&&force>.015){hits.add(finger);forces[finger]+=force;}
      if(other===tableBody)table=true;if(other===required&&force>.005)support=true;
      contacts.push({position:new THREE.Vector3(c.pos[0],c.pos[2],-c.pos[1]),force,other});c.delete();
    }list.delete();
    const position=this.objectPose().position,j=this.model.body_jntadr[b],d=this.model.jnt_dofadr[j];
    return {count:hits.size,force:forces.reduce((s,v)=>s+v,0),forces,table,support,position,speed:Math.hypot(...this.data.qvel.subarray(d,d+3)),distance:position.distanceTo(this.planData?.drop||target.drop),contacts};
  }
  plan(target,yawOffset=0,seedIndex=0,lateral=.14){
    const start=this.objectPose(target.id).position,drop=target.drop.clone();drop.y=start.y;
    if(this.task==='stack'){
      const base=this.targets.find(t=>t.id==='blue').drop;drop.x=base.x;drop.z=base.z;
      if(this.itemIndex>0){const id=['blue','red'][this.itemIndex-1],support=this.objectPose(id).position,t=this.targets.find(t=>t.id===id);drop.x=support.x;drop.z=support.z;drop.y=support.y+t.h+target.h;}
    }
    const names=armJointNames(this.arm),sign=this.arm==='left'?1:-1;
    if(start.x>.65||Math.hypot(start.x,start.z)>.66||drop.x>.65)throw new Error('目标超出站立抓取范围，请将物体移近桌面中央');
    const align=point=>THREE.MathUtils.clamp(Math.atan2(-point.z,point.x)-Math.asin(THREE.MathUtils.clamp(sign*lateral/Math.hypot(point.x,point.z),-.9,.9)),-1.0,1.0);
    const startYaw=align(start),dropYaw=align(drop),tilt=20*Math.PI/180;
    const quaternion=(yaw,down=0)=>new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),yaw+yawOffset).multiply(sidePinch).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),down));
    const orientation=quaternion(startYaw,tilt),dropOrientation=quaternion(dropYaw,tilt),stand={...STAND_POSE,waist_yaw_joint:startYaw};
    let seed={...stand,...READY.right,...READY.left};
    const seeds=[
      naturalArmPose(this.arm),[.15,.5*sign,.2*sign,-.5,0,.15,0],[-.6,.3*sign,0,.2,0,.1,0],
      [-.27,.15*sign,.03*sign,-.85,-.24*sign,1.10,.24*sign],[-.3,.7*sign,-.3*sign,-.4,.4*sign,.3,0],
      [-.7,.45*sign,.4*sign,-.8,-.3*sign,.7,0]
    ];
    Object.assign(seed,Object.fromEntries(names.map((n,i)=>[n,seeds[seedIndex][i]])));
    const options={limits:naturalArmLimits(this.arm)};
    const solve=(p,continuous=false,down=0)=>{
      const yaw=align(p);
      seed.waist_yaw_joint=yaw;
      const result=solvePose(this.robot,this.arm,p,quaternion(yaw,down),seed,continuous?140:240,options);
      // A valid isolated IK solution is not necessarily a valid trajectory.
      // Reject branch jumps instead of interpolating a wrist through a table
      // or throwing an object when the redundant elbow flips configuration.
      if(result.positionError>.003||result.angleError>.035){const error=new Error('目标超出当前站姿可稳定抓取的范围，请向桌面中央移动');error.detail={position:p.toArray(),positionError:result.positionError,angleError:result.angleError};throw error;}
      const jump=Math.max(...names.map(n=>Math.abs(result.pose[n]-(seed[n]??0))));
      if(continuous&&jump>.30){const error=new Error('当前位置的连续抓取路径不可达，请将物体稍微移近');error.detail={position:p.toArray(),jointJump:jump};throw error;}
      seed={...stand,...READY.right,...READY.left,...result.pose};return {...seed};
    };
    const above=start.clone().add(new THREE.Vector3(.04,.11,0)),aboveDrop=drop.clone().add(new THREE.Vector3(.04,.11,0));
    const approach=solve(above),line=(from,to,ta=0,tb=ta)=>Array.from({length:41},(_,i)=>solve(from.clone().lerp(to,i/40),true,THREE.MathUtils.lerp(ta,tb,i/40)));
    const closingAxis=new THREE.Vector3(0,1,0).applyQuaternion(orientation),objectQ=this.objectPose(target.id).quaternion;
    const axes=[new THREE.Vector3(1,0,0),new THREE.Vector3(0,1,0),new THREE.Vector3(0,0,1)].map(a=>Math.abs(a.applyQuaternion(objectQ).dot(closingAxis)));
    const half=target.id==='blue'?[.026,.019,.016]:[.0225,.0225,.0225];
    const width=target.id==='yellow'?2*(.02*Math.sqrt(Math.max(0,1-axes[1]**2))+.026*axes[1]):2*half.reduce((s,v,i)=>s+v*axes[i],0);
    const plan={start,drop,startYaw,dropYaw,stand,objectRotation:objectQ.clone(),orientation:dropOrientation,graspOrientation:orientation,preOpening:Math.min(OPEN,width+.018),approach,descend:line(above,start,0,tilt),lift:line(start,above,tilt,0),transfer:line(above,aboveDrop),place:line(aboveDrop,drop,0,tilt),retreat:line(drop,aboveDrop,tilt,0)};
    const ignore=[this.bodies.get(this.selected)];
    for(const phase of ['descend','lift','transfer','place','retreat']){const path=plan[phase];for(let i=0;i<path.length;i++){
      const objectPoseFor=phase==='descend'?null:pose=>this.predictedObject(pose,plan,phase);
      const check=this.dynamics.checkPose(path[i],{opening:{[this.arm]:plan.preOpening,[this.arm==='right'?'left':'right']:OPEN},ignoreObjectBodies:ignore,objectPoses:objectPoseFor?.(path[i])});
      if(!check.safe){const error=new Error('抓取路径会与身体、桌面或相邻物体相撞，请调整物体位置');error.detail={phase,sample:i,contacts:check.contacts.slice(0,3),limits:check.limitViolations};throw error;}
      if(i)this.validateSegment(path[i-1],path[i],{opening:{[this.arm]:plan.preOpening,[this.arm==='right'?'left':'right']:OPEN},objectPoseFor});
    }}
    this.planData=plan;
  }
  start(arm,id){if(this.running)return false;this.task='putaway';this.history=[];this.stackEvidence=[];this.reaction=null;this.planError=null;return this.prepareItem(arm,id,false);}
  startStack(arm,{observerYaw=0,reducedMotion=false}={}){
    if(this.running)return false;
    Object.assign(this,{task:'stack',itemIndex:0,stackEvidence:[],stackStableSeconds:0,stackResultVerified:false,history:[],reaction:null,reactionFinished:null,planError:null,observerYaw:THREE.MathUtils.clamp(observerYaw,-1,1),reducedMotion});return this.prepareItem(arm,'blue',false);
  }
  prepareItem(arm,id,continuing){
    this.arm=arm;this.selected=id;const before={...this.pose};let error;
    search:for(const lateral of [.14,.17,.11])for(const yaw of [0,.15,-.15,.3,-.3])for(let branch=0;branch<6;branch++){try{this.plan(this.targets.find(t=>t.id===id),yaw,branch,lateral);error=null;break search;}catch(e){error=e;}}
    this.applyPose(before);this.dynamics.syncRobot();
    if(error){this.planError=error.detail;this.fail(error.message);return false;}
    if(!this.setOpenings({left:OPEN,right:OPEN})){this.running=false;this.state='failed';return false;}
    this.continuing=continuing;this.running=true;this.evidence={lift:0,maxSpeed:0,dual:false,stable:0};this.transition(continuing?'recenter':'spread');return this.running;
  }
  phaseTarget(state){
    const yaw=this.pose.waist_yaw_joint??0,standing={...STAND_POSE,waist_yaw_joint:yaw},p=this.planData;
    if(state==='align')return {...this.pose,...STAND_POSE,waist_yaw_joint:p.startYaw};
    if(state==='spread')return {...standing,...SPREAD.right,...SPREAD.left};
    if(state==='raise')return {...standing,...RAISED.right,...RAISED.left};
    if(state==='ready')return {...standing,...READY.right,...READY.left};
    if(['recenter','return_clear','reaction_clear'].includes(state))return {...standing,...READY.right,...READY.left};
    if(['return_align','reaction_align'].includes(state))return {...STAND_POSE,...READY.right,...READY.left};
    if(['return_fold','reaction_fold'].includes(state))return {...standing,...RAISED.right,...RAISED.left};
    if(['return_wide','reaction_wide'].includes(state))return {...standing,...SPREAD.right,...SPREAD.left};
    if(['return','reaction_home'].includes(state))return {...idlePose,waist_yaw_joint:yaw};
    if(state==='return_center')return {...idlePose};
    if(state==='approach')return p.approach;
    if(state==='face_observer')return this.reactionCenter;
    if(state==='happy'||state==='shy')return this.reactionTarget();
    return null;
  }
  transition(state){
    const to=this.phaseTarget(state);
    const step=['happy','shy'].includes(state)?this.reactionSteps[this.reactionIndex]:null;
    const gapGoal=state==='closing'?CLOSED:['approach','release'].includes(state)?this.planData.preOpening:step?(step.opening??OPEN):null;
    try{
      if(to)this.validateSegment(this.pose,to,{opening:gapGoal===null?this.openings:{...this.openings,[this.arm]:gapGoal},openingFrom:gapGoal===null?null:this.openings});
      if(gapGoal!==null&&!to)this.validateSegment(this.pose,this.pose,{opening:{...this.openings,[this.arm]:gapGoal},openingFrom:this.openings});
    }catch(error){this.message=error.message;this.planError=error.detail;this.running=false;this.state='failed';return;}
    this.state=state;this.elapsed=0;this.from={...this.pose};this.phaseGoal=to;this.evidence.stable=0;if(state==='place'){this.placeLowering=0;this.placePose=null;}this.history.push(state);
  }
  stop(message='已停止；松爪后物体按重力落桌'){this.running=false;this.state='stopped';const released=this.setOpenings({left:OPEN,right:OPEN});if(released)this.message=message;else this.message=`已停止并保持安全开口。${this.message}`;this.reaction=null;}
  fail(message){const released=this.setOpenings({left:OPEN,right:OPEN});if(!released){this.message=`${message}；已保持夹爪开口，请先移离障碍`;this.running=false;this.state='failed';return;}this.message=message;if(this.task==='stack'&&!this.reaction){this.beginReaction('shy');return;}this.running=false;this.state='failed';}
  beginReaction(kind){
    this.reaction=kind;this.running=true;this.reactionCenter={...idlePose,waist_yaw_joint:THREE.MathUtils.clamp(this.observerYaw??0,-1,1)};
    this.reactionSteps=kind==='happy'?[{joint:7,offset:-.45,hold:.48,timeout:1},{joint:7,offset:.45,hold:.48,timeout:1},{hold:.35,timeout:.85},{joint:5,offset:.30,hold:.65,timeout:1.15},{opening:CLOSED,hold:.4,timeout:1},{opening:OPEN,hold:.48,timeout:1.1}]:[{joint:4,offset:.12,hold:.75,timeout:1.25},{joint:4,offset:.20,hold:.75,timeout:1.25},{hold:.5,timeout:1}];
    this.reactionIndex=0;this.transition(['idle','complete','stopped'].includes(this.state)?'face_observer':'reaction_clear');
  }
  reactionTarget(){
    const step=this.reactionSteps[this.reactionIndex]||{},pose={...this.reactionCenter};
    if(step.joint)for(const arm of ['right','left']){const name=armJointNames(arm)[step.joint-1],sign=step.joint===7?(arm==='right'?1:-1):1;pose[name]+=sign*step.offset*(this.reducedMotion?.2:1);}
    for(const [name,value] of Object.entries(pose)){const limit=this.robot.joints[name]?.limit;if(limit)pose[name]=THREE.MathUtils.clamp(value,limit.lower+.001,limit.upper-.001);}return pose;
  }
  jointError(){const t=this.dynamics.telemetry(this.arm);return Math.max(...t.joints.map(j=>Math.abs(j.error)),...t.torso.map(j=>Math.abs(j.error)));}
  wristError(){const actual=tcpPose(this.robot,this.arm).position;this.robot.setJointValues(this.pose);this.robot.updateMatrixWorld(true);const desired=tcpPose(this.robot,this.arm).position;this.dynamics.syncRobot();return actual.distanceTo(desired);}
  stackCheck(){
    let stable=true,supportBody=this.id('mjOBJ_BODY','task_table');const states=[],list=this.data.contact;
    for(const id of ['blue','red','yellow']){
      const b=this.bodies.get(id),p=this.objectPose(id).position,j=this.model.body_jntadr[b],d=this.model.jnt_dofadr[j];let supported=false;
      for(let i=0;i<Math.min(this.data.ncon,list.size());i++){const c=list.get(i),a=this.model.geom_bodyid[c.geom1],o=this.model.geom_bodyid[c.geom2];if((a===b&&o===supportBody)||(o===b&&a===supportBody))supported=true;c.delete();}
      const target=this.stackEvidence.find(s=>s.id===id)?.drop,speed=Math.hypot(...this.data.qvel.subarray(d,d+3)),error=target?p.distanceTo(new THREE.Vector3(...target)):Infinity;
      const ok=supported&&speed<.0175&&error<.012;stable&&=ok;states.push({id,supported,speed,error,position:p.toArray()});supportBody=b;
    }list.delete();return {stable,states};
  }
  finishItem(){
    if(this.task!=='stack'){this.transition('return_clear');return;}
    this.stackEvidence.push({id:this.selected,drop:this.planData.drop.toArray(),...this.evidence});
    if(this.itemIndex<2){this.itemIndex++;this.prepareItem(this.arm,['blue','red','yellow'][this.itemIndex],true);}else this.transition('stack_settle');
  }
  updateMotion(){
    const t=this.elapsed,p=this.planData,joint=(to,duration)=>this.applyPose(mix(this.from,to,smooth(t/duration))),path=(samples,duration)=>{const v=smooth(t/duration)*(samples.length-1),i=Math.min(samples.length-2,Math.floor(v));this.applyPose(mix(samples[i],samples[i+1],v-i));};
    const durations={align:1.3,recenter:1.4,spread:1.4,raise:1.5,ready:1,approach:2.4,return_clear:1.6,return_align:1.2,return_fold:1,return_wide:1.5,return:1.4,return_center:1.3,reaction_clear:1.6,reaction_align:1.2,reaction_fold:1,reaction_wide:1.5,reaction_home:1.4,face_observer:1.6,happy:.5,shy:.5};
    if(this.phaseGoal)joint(this.phaseGoal,durations[this.state]??1);
    if(this.state==='descend')path(p.descend,1.7);if(this.state==='lift')path(p.lift,1.8);if(this.state==='transfer')path(p.transfer,2);if(this.state==='place'){if(this.placePose)this.applyPose(this.placePose);else path(p.place,1.8);}if(this.state==='retreat')path(p.retreat,1.6);
    if(this.state==='happy'||this.state==='shy')this.opening=this.reactionSteps[this.reactionIndex].opening??OPEN;
  }
  step(dt=.001){
    this.time+=dt;if(this.running){this.elapsed+=dt;this.updateMotion();if(this.state==='approach')this.opening=THREE.MathUtils.lerp(OPEN,this.planData.preOpening,smooth(this.elapsed/1.2));if(this.state==='closing')this.opening=THREE.MathUtils.lerp(this.planData.preOpening,CLOSED,smooth(this.elapsed));if(this.state==='release')this.opening=THREE.MathUtils.lerp(CLOSED,this.planData.preOpening,smooth(this.elapsed/.6));}
    for(const arm of ['right','left'])for(const f of this.fingers[arm])this.data.ctrl[f.actuator]=openingToQ(this.openings[arm]);
    this.dynamics.beforeStep(dt);this.api.mj_step(this.model,this.data);this.dynamics.afterStep();this.tick++;if(this.tick%10!==0)return;
    const m=this.measure();this.measurement=m;if(!this.running)return;this.evidence.maxSpeed=Math.max(this.evidence.maxSpeed,m.speed);const t=this.elapsed;
    if(this.state==='recenter'&&t>=1.4)this.transition('align');
    else if(this.state==='align'&&t>=1.3)this.transition('approach');
    else if(this.state==='spread'&&t>=1.4)this.transition('raise');
    else if(this.state==='raise'&&t>=1.5)this.transition('ready');
    else if(this.state==='ready'&&t>=1)this.transition('align');
    else if(this.state==='return_clear'&&t>=1.6)this.transition('return_align');
    else if(this.state==='return_align'&&t>=1.2)this.transition('return_fold');
    else if(this.state==='return_fold'&&t>=1)this.transition('return_wide');
    else if(this.state==='return_wide'&&t>=1.5)this.transition('return');
    else if(this.state==='return'&&t>=1.4)this.transition('return_center');
    else if(this.state==='reaction_clear'&&t>=1.6)this.transition('reaction_align');
    else if(this.state==='reaction_align'&&t>=1.2)this.transition('reaction_fold');
    else if(this.state==='reaction_fold'&&t>=1)this.transition('reaction_wide');
    else if(this.state==='reaction_wide'&&t>=1.5)this.transition('reaction_home');
    else if(this.state==='reaction_home'&&t>=1.4)this.transition('face_observer');
    else if(this.state==='approach'&&t>=2.4)this.transition('descend');
    else if(this.state==='descend'&&t>=1.7)this.transition('closing');
    else if(this.state==='closing'){this.evidence.stable=m.count===2?this.evidence.stable+.01:0;if(t>1&&this.evidence.stable>.18){this.evidence.dual=true;this.transition('lift');}else if(t>2.2)this.fail('夹持失败：双指未建立稳定接触，请调整物体位置后重试');}
    else if(this.state==='lift'){this.evidence.lift=Math.max(this.evidence.lift,m.position.y-this.planData.start.y);if(t>=1.8){if(this.evidence.lift>.035&&m.count===2)this.transition('transfer');else this.fail('夹持滑脱：目标未稳定抬离桌面');}}
    else if(this.state==='transfer'&&t>=2)this.transition('place');
    else if(this.state==='place'){
      // Contact feedback, not an object pose assignment: an object may settle
      // a few mm above the nominal center between sloped Dex1 pads. Lower the
      // actual arm slowly until the required real support contact is present.
      if(t>=1.8&&!m.support&&this.placeLowering<.009){
        this.placeLowering+=.00007;
        const position=this.planData.drop.clone();position.y-=this.placeLowering;
        const correction=solvePose(this.robot,this.arm,position,this.planData.orientation,this.pose,20,{limits:naturalArmLimits(this.arm)});
        if(correction.positionError<.002&&correction.angleError<.025&&Math.max(...armJointNames(this.arm).map(n=>Math.abs(correction.pose[n]-this.pose[n])))<.05){
          try{this.validateSegment(this.pose,correction.pose,{ignoreSelected:true});this.placePose=correction.pose;}catch{/* Hold the last safe arm target; never force the palm through support. */}
        }
        this.dynamics.syncRobot();
      }
      this.evidence.stable=m.support&&m.distance<.012&&m.speed<.035?this.evidence.stable+.01:0;
      if(t>=1.8&&this.evidence.stable>.12)this.transition('release');else if(t>3.4)this.fail('放置未稳定：目标尚未在对应颜色框内落桌');
    }
    else if(this.state==='release'&&t>.75)this.transition('retreat');else if(this.state==='retreat'&&t>=1.6)this.transition('settling');
    else if(this.state==='settling'&&t>.4){if(m.support&&m.distance<.02&&m.speed<.025)this.finishItem();else if(t>1.5)this.fail('物体松爪后未在放置框内稳定');}
    else if(this.state==='stack_settle'){const check=this.stackCheck();this.stackSnapshot=check;this.stackStableSeconds=check.stable?this.stackStableSeconds+.01:0;if(this.stackStableSeconds>=3){this.stackResultVerified=true;this.beginReaction('happy');}else if(t>5)this.fail('叠放未稳定：三件物体的支撑或速度判定未通过');}
    else if(this.state==='face_observer'&&t>=1.6)this.transition(this.reaction);
    else if(this.state==='happy'||this.state==='shy'){const step=this.reactionSteps[this.reactionIndex],grip=this.fingers[this.arm].every(f=>Math.abs(this.data.qpos[f.q]-openingToQ(this.opening))<.006);if((t>=step.hold&&this.jointError()<.07&&grip)||t>=step.timeout){this.reactionIndex++;if(this.reactionIndex>=this.reactionSteps.length){this.running=false;this.state=this.reaction==='happy'?'complete':'failed';if(this.reaction==='happy')this.message='叠放完成：蓝色、红色、黄色已逐件释放并稳定支撑';this.reactionFinished=this.reaction;}else this.transition(this.reaction);}}
    else if(this.state==='return_center'&&t>=1.3){this.running=false;this.state='complete';this.message='已夹取并放入对应颜色框';}
    if(['lift','transfer','place'].includes(this.state)&&m.position.y<TABLE_HEIGHT-.03)this.fail('目标已滑落，请复位后重试');
  }
}
