import * as THREE from '../vendor/three.module.min.js';
import {solvePose,tcpPose} from './grasp-kinematics.mjs';
import {TABLE_HEIGHT,ROOT_HEIGHT,HOME_POSE,JOINT_VELOCITY_LIMITS,armJointNames,fingerJointNames,fingerBodyNames,fingerActuatorNames,openingToQ,qToOpening,GRIPPER_Q_MIN,GRIPPER_Q_MAX} from './c5-profile.mjs';
export {TABLE_HEIGHT} from './c5-profile.mjs';

export const REST={left:HOME_POSE},REST_TORSO={};
const OPEN=qToOpening(GRIPPER_Q_MAX),CLOSED=qToOpening(GRIPPER_Q_MIN);
const smooth=t=>{t=THREE.MathUtils.clamp(t,0,1);return t*t*(3-2*t);};
const mix=(a,b,t)=>Object.fromEntries(Object.keys(b).map(k=>[k,THREE.MathUtils.lerp(a[k]??b[k],b[k],t)]));
const toMJ=p=>[p.x,-p.z,p.y];
const basis=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI/2);
const sidePinch=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2);
const names=armJointNames('left');
const fromArray=a=>Object.fromEntries(names.map((n,i)=>[n,a[i]]));
const planningLimits=[[-Math.PI,Math.PI],[-2.9,2.9],[-2.9,2.9],[-Math.PI,Math.PI],[-2.9,2.9],[-Math.PI,Math.PI]];

export class GraspController{
  get opening(){return this.openings.left;}
  set opening(value){this.openings.left=value;}
  constructor(api,model,data,robot,targets,dynamics){
    Object.assign(this,{api,model,data,robot,targets,dynamics});this.arm='left';this.selected='blue';
    this.id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name);
    this.bodies=new Map(targets.map(t=>[t.id,this.id('mjOBJ_BODY',t.body)]));
    this.fingers={left:fingerJointNames().map((name,i)=>{
      const j=this.id('mjOBJ_JOINT',name),body=this.id('mjOBJ_BODY',fingerBodyNames()[i]),actuator=this.id('mjOBJ_ACTUATOR',fingerActuatorNames()[i]);
      if(j<0||body<0||actuator<0)throw new Error('Dex1物理夹指缺失：'+name);
      return {q:model.jnt_qposadr[j],d:model.jnt_dofadr[j],body,actuator};
    })};this.reset();
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
  stackCheck(){
    let stable=true,supportBody=this.id('mjOBJ_BODY','task_table');const states=[],list=this.data.contact;
    for(const id of ['blue','red','yellow']){
      const b=this.bodies.get(id),p=this.objectPose(id).position,j=this.model.body_jntadr[b],d=this.model.jnt_dofadr[j];let supported=false;
      for(let i=0;i<Math.min(this.data.ncon,list.size());i++){const c=list.get(i),a=this.model.geom_bodyid[c.geom1],o=this.model.geom_bodyid[c.geom2];if((a===b&&o===supportBody)||(o===b&&a===supportBody))supported=true;c.delete();}
      const target=this.stackEvidence.find(s=>s.id===id)?.drop,speed=Math.hypot(...this.data.qvel.subarray(d,d+3)),error=target?p.distanceTo(new THREE.Vector3(...target)):Infinity;
      const ok=supported&&speed<.0175&&error<.012;stable&&=ok;states.push({id,supported,speed,error,position:p.toArray()});supportBody=b;
    }list.delete();return {stable,states};
  }
  applyPose(pose){this.pose={...pose};this.dynamics.setTargets(this.pose);}
  validateSegment(from,to,{opening=this.opening,openingFrom=null,ignoreSelected=true,objectPoseFor=null}={}){
    const count=Math.max(2,Math.ceil(Math.max(...names.map(n=>Math.abs((to[n]??0)-(from[n]??to[n]))))/.035));
    const gapSteps=openingFrom===null?0:Math.max(1,Math.ceil(Math.abs(opening-openingFrom)/.002));
    for(let i=0;i<=count;i++){
      const pose=mix(from,to,i/count);
      for(let k=0;k<=gapSteps;k++){
        const gap=openingFrom===null?opening:THREE.MathUtils.lerp(openingFrom,opening,k/gapSteps);
        const check=this.dynamics.checkPose(pose,{opening:gap,ignoreObjectBodies:ignoreSelected?[this.bodies.get(this.selected)]:[],objectPoses:objectPoseFor?.(pose)});
        if(!check.safe){const error=new Error('机械臂或夹爪路径与桌面、支柱或邻近物体干涉，请调整目标位置');error.detail={sample:i,count,contacts:check.contacts.slice(0,3),limits:check.limitViolations};throw error;}
      }
    }return true;
  }
  selectArm(){return !this.running;}
  setOpenings(gaps){
    const value=typeof gaps==='number'?gaps:gaps.left;
    if(!Number.isFinite(value)){this.message='夹爪开口无效';return false;}
    const next=THREE.MathUtils.clamp(value,CLOSED,OPEN),actual=this.dynamics.telemetry().allJoints;
    const pose=Object.fromEntries(actual.filter(j=>j.unit==='rad').map(j=>[j.name,j.actual]));
    const gap=qToOpening(this.fingers.left.reduce((sum,f)=>sum+this.data.qpos[f.q],0)/2);
    try{this.validateSegment(this.pose,this.pose,{opening:next,openingFrom:this.opening});this.validateSegment(pose,pose,{opening:next,openingFrom:gap});if(this.manualMotion)this.validateSegment(this.pose,this.manualMotion.to,{opening:next,openingFrom:this.opening});this.opening=next;return true;}
    catch(error){this.message='夹爪动作已拒绝：'+error.message+'；请先移离障碍';this.planError=error.detail;return false;}
  }
  setManualOpening(value){return !this.running&&this.setOpenings(value);}
  applyManualPose(pose){
    if(this.running)return false;
    try{
      this.validateSegment(this.pose,pose,{ignoreSelected:false});
      const actual=Object.fromEntries(this.dynamics.telemetry().joints.map(j=>[j.name,j.actual]));
      const gap=qToOpening(this.fingers.left.reduce((sum,f)=>sum+this.data.qpos[f.q],0)/2);
      this.validateSegment(actual,pose,{opening:this.opening,openingFrom:gap,ignoreSelected:false});
      // A collision-free endpoint is not permission to apply a position step.
      // Time-scale the same checked joint-space segment, with one shared clock
      // and 35% velocity headroom, so coordinated axes keep its swept path.
      const duration=Math.max(.18,...names.map(name=>1.5*Math.abs(pose[name]-this.pose[name])/(JOINT_VELOCITY_LIMITS[name]*.65)));
      this.manualMotion={from:{...this.pose},to:{...pose},elapsed:0,duration};return true;
    }
    catch(error){this.message='手动目标已拒绝：'+error.message;this.planError=error.detail;return false;}
  }
  syncFingers(){this.dynamics.syncRobot();}
  reset(){
    this.api.mj_resetData(this.model,this.data);
    Object.assign(this,{running:false,state:'idle',message:'C5 六轴机械臂待命',elapsed:0,time:0,tick:0,openings:{left:OPEN},manualMotion:null,history:[],task:'putaway',itemIndex:0,stackEvidence:[],stackStableSeconds:0,stackResultVerified:false,reaction:null,reactionFinished:null,planData:null,planError:null,measurement:null,evidence:{lift:0,maxSpeed:0,dual:false,stable:0}});
    for(const t of this.targets)this.setObject(t.id,t.start,t.rotation);
    this.pose={...HOME_POSE};this.dynamics.reset({...this.pose,...Object.fromEntries(fingerJointNames().map(name=>[name,GRIPPER_Q_MAX]))});this.stackSnapshot=null;this.syncFingers();
  }
  plan(target,seedIndex=0,yawOffset=0,tilt=20*Math.PI/180){
    const start=this.objectPose(target.id).position,drop=target.drop.clone();drop.y=start.y;
    if(start.x<.25||start.x>.68||Math.hypot(start.x,start.z)>.72)throw new Error('目标超出当前作业范围，请移回桌面中央');
    if(this.task==='stack'){
      const base=this.targets.find(t=>t.id==='blue').drop;drop.x=base.x;drop.z=base.z;
      if(this.itemIndex>0){const id=['blue','red'][this.itemIndex-1],p=this.objectPose(id).position,t=this.targets.find(t=>t.id===id);drop.set(p.x,p.y+t.h+target.h,p.z);}
    }
    const aim=p=>Math.atan2(-p.z,p.x)+yawOffset;
    const quaternion=(p,down=0)=>new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),aim(p)).multiply(sidePinch).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),down));
    const seeds=[
      [0,-.7,1.4,-.7,1.5,0],[0,.7,-1.4,.7,-1.5,0],
      [Math.PI,-.7,1.4,-.7,-1.5,0],[-Math.PI,.7,-1.4,.7,1.5,0],
      [1.5,-1.2,1.8,-1,1.5,0],[-1.5,1.2,-1.8,1,-1.5,0],
      [0,-1.5,1.5,0,1.5,0],[0,1.5,-1.5,0,-1.5,0],Object.values(this.pose)
    ];
    let seed=fromArray(seeds[seedIndex]);
    const solve=(point,continuous=false,down=0)=>{
      const result=solvePose(this.robot,'left',point,quaternion(point,down),seed,continuous?160:350,{limits:planningLimits});
      if(result.positionError>.0025||result.angleError>.025){const e=new Error('目标姿态不可稳定到达，请移动或转动物体');e.detail={point:point.toArray(),positionError:result.positionError,angleError:result.angleError};throw e;}
      const jump=Math.max(...names.map(n=>Math.abs(result.pose[n]-(seed[n]??0))));
      if(continuous&&jump>.28){const e=new Error('连续抓取路径跨越关节分支，请调整目标位置');e.detail={jointJump:jump};throw e;}
      seed={...result.pose};return {...seed};
    };
    const above=start.clone().add(new THREE.Vector3(.04,.14,0)),aboveDrop=drop.clone().add(new THREE.Vector3(.04,.14,0));
    const approach=solve(above),line=(a,b,ta=0,tb=ta)=>Array.from({length:41},(_,i)=>solve(a.clone().lerp(b,i/40),true,THREE.MathUtils.lerp(ta,tb,i/40)));
    const orientation=quaternion(drop,tilt),graspOrientation=quaternion(start,tilt),objectRotation=this.objectPose(target.id).quaternion;
    const axis=new THREE.Vector3(0,1,0).applyQuaternion(graspOrientation);
    const axes=[new THREE.Vector3(1,0,0),new THREE.Vector3(0,1,0),new THREE.Vector3(0,0,1)].map(v=>Math.abs(v.applyQuaternion(objectRotation).dot(axis)));
    const half=target.id==='blue'?[.026,.019,.016]:[.0225,.0225,.0225];
    const width=target.id==='yellow'?2*(.02*Math.sqrt(Math.max(0,1-axes[1]**2))+.026*axes[1]):2*half.reduce((sum,v,i)=>sum+v*axes[i],0);
    const plan={start,drop,orientation,graspOrientation,objectRotation:objectRotation.clone(),preOpening:Math.min(OPEN,width+.018),approach,descend:line(above,start,0,tilt),lift:line(start,above,tilt,0),transfer:line(above,aboveDrop),place:line(aboveDrop,drop,0,tilt),retreat:line(drop,aboveDrop,tilt,0)};
    this.validateSegment(this.pose,approach,{opening:OPEN});
    for(const phase of ['descend','lift','transfer','place','retreat']){
      const path=plan[phase],objectPoseFor=phase==='descend'?null:pose=>this.predictedObject(pose,plan,phase);
      for(let i=0;i<path.length;i++)this.validateSegment(i?path[i-1]:path[i],path[i],{opening:plan.preOpening,objectPoseFor});
    }
    this.planData=plan;
  }
  start(arm,id){if(this.running)return false;this.manualMotion=null;this.task='putaway';this.history=[];this.stackEvidence=[];this.reaction=null;this.planError=null;return this.prepareItem('left',id,false);}
  startStack(arm,{reducedMotion=false}={}){
    if(this.running)return false;
    Object.assign(this,{manualMotion:null,task:'stack',itemIndex:0,stackEvidence:[],stackStableSeconds:0,stackResultVerified:false,history:[],reaction:null,reactionFinished:null,planError:null,reducedMotion});
    return this.prepareItem('left','blue',false);
  }
  prepareItem(arm,id,continuing){
    this.selected=id;const before={...this.pose};let error;
    search:for(const tilt of [20,12,28])for(const yaw of [0,.2,-.2,.4,-.4])for(let branch=0;branch<9;branch++){try{this.plan(this.targets.find(t=>t.id===id),branch,yaw,tilt*Math.PI/180);error=null;break search;}catch(e){error=e;}}
    this.applyPose(before);this.dynamics.syncRobot();
    if(error){this.planError=error.detail;this.fail(error.message);return false;}
    if(!this.setOpenings(OPEN)){this.running=false;this.state='failed';return false;}
    this.continuing=continuing;this.running=true;this.evidence={lift:0,maxSpeed:0,dual:false,stable:0};this.transition('approach');return this.running;
  }
  phaseTarget(state){
    if(state==='approach')return this.planData.approach;
    if(['return','reaction_home'].includes(state))return {...HOME_POSE};
    if(['happy','shy'].includes(state)){
      const pose={...HOME_POSE},step=this.reactionSteps[this.reactionIndex];
      if(step.joint)pose[names[step.joint]]+=step.offset*(this.reducedMotion?.2:1);return pose;
    }return null;
  }
  transition(state){
    const to=this.phaseTarget(state),step=['happy','shy'].includes(state)?this.reactionSteps[this.reactionIndex]:null;
    const goal=state==='closing'?CLOSED:['approach','release'].includes(state)?this.planData.preOpening:step?(step.opening??OPEN):null;
    try{
      if(to)this.validateSegment(this.pose,to,{opening:goal??this.opening,openingFrom:goal===null?null:this.opening});
      else if(goal!==null)this.validateSegment(this.pose,this.pose,{opening:goal,openingFrom:this.opening});
    }catch(error){this.message=error.message;this.planError=error.detail;this.running=false;this.state='failed';return;}
    this.state=state;this.elapsed=0;this.from={...this.pose};this.phaseGoal=to;this.history.push(state);this.evidence.stable=0;
    if(state==='place'){this.placeLowering=0;this.placePose=null;}
  }
  stop(message='已停止；物体按重力落桌'){this.manualMotion=null;this.running=false;this.state='stopped';const released=this.setOpenings(OPEN);if(released)this.message=message;else this.message='已停止并保持安全开口；请先移离障碍';this.reaction=null;}
  fail(message){
    this.manualMotion=null;
    const released=this.setOpenings(OPEN);this.message=message;
    if(!released){this.message+='；夹爪已保持安全开口';this.running=false;this.state='failed';return;}
    if(this.task==='stack'&&!this.reaction){this.beginReaction('shy');return;}this.running=false;this.state='failed';
  }
  beginReaction(kind){
    this.reaction=kind;this.running=true;this.reactionIndex=0;
    this.reactionSteps=kind==='happy'?[{joint:5,offset:-.2,hold:.6,timeout:1},{joint:5,offset:.2,hold:.6,timeout:1},{hold:.4,timeout:.8},{opening:CLOSED,hold:.5,timeout:1},{opening:OPEN,hold:.5,timeout:1}]:[{joint:4,offset:.10,hold:.8,timeout:1.2},{joint:4,offset:.18,hold:.8,timeout:1.2},{hold:.5,timeout:.8}];
    this.transition('reaction_home');
  }
  jointError(){return Math.max(...this.dynamics.telemetry().joints.map(j=>Math.abs(j.error)));}
  wristError(){const actual=tcpPose(this.robot).position;this.robot.setJointValues(this.pose);this.robot.updateMatrixWorld(true);const desired=tcpPose(this.robot).position;this.dynamics.syncRobot();return actual.distanceTo(desired);}
  finishItem(){
    if(this.task!=='stack'){this.transition('return');return;}
    this.stackEvidence.push({id:this.selected,drop:this.planData.drop.toArray(),...this.evidence});
    if(this.itemIndex<2){this.itemIndex++;this.prepareItem('left',['blue','red','yellow'][this.itemIndex],true);}else this.transition('stack_settle');
  }
  updateMotion(){
    const t=this.elapsed,p=this.planData,path=(samples,duration)=>{const v=smooth(t/duration)*(samples.length-1),i=Math.min(samples.length-2,Math.floor(v));this.applyPose(mix(samples[i],samples[i+1],v-i));};
    if(this.phaseGoal)this.applyPose(mix(this.from,this.phaseGoal,smooth(t/(['approach','return','reaction_home'].includes(this.state)?2.5:.6))));
    if(this.state==='descend')path(p.descend,1.7);
    if(this.state==='lift')path(p.lift,1.8);
    if(this.state==='transfer')path(p.transfer,2);
    if(this.state==='place'){if(this.placePose)this.applyPose(this.placePose);else path(p.place,1.8);}
    if(this.state==='retreat')path(p.retreat,1.6);
    if(['happy','shy'].includes(this.state))this.opening=this.reactionSteps[this.reactionIndex].opening??OPEN;
  }
  step(dt=.001){
    if(!this.running&&this.manualMotion){
      const motion=this.manualMotion;motion.elapsed+=dt;
      this.applyPose(mix(motion.from,motion.to,smooth(motion.elapsed/motion.duration)));
      if(motion.elapsed>=motion.duration){this.applyPose(motion.to);this.manualMotion=null;}
    }
    this.time+=dt;if(this.running){
      this.elapsed+=dt;this.updateMotion();
      if(this.state==='approach')this.opening=THREE.MathUtils.lerp(OPEN,this.planData.preOpening,smooth(this.elapsed/1.2));
      if(this.state==='closing')this.opening=THREE.MathUtils.lerp(this.planData.preOpening,CLOSED,smooth(this.elapsed));
      if(this.state==='release')this.opening=THREE.MathUtils.lerp(CLOSED,this.planData.preOpening,smooth(this.elapsed/.6));
    }
    for(const f of this.fingers.left)this.data.ctrl[f.actuator]=openingToQ(this.opening);
    this.dynamics.beforeStep(dt);this.api.mj_step(this.model,this.data);this.dynamics.afterStep();this.tick++;if(this.tick%10)return;
    const m=this.measure();this.measurement=m;if(!this.running)return;
    this.evidence.maxSpeed=Math.max(this.evidence.maxSpeed,m.speed);const t=this.elapsed;
    if(this.state==='approach'&&t>=2.5)this.transition('descend');
    else if(this.state==='descend'&&t>=1.7)this.transition('closing');
    else if(this.state==='closing'){
      this.evidence.stable=m.count===2?this.evidence.stable+.01:0;
      if(t>1&&this.evidence.stable>.18){this.evidence.dual=true;this.transition('lift');}
      else if(t>2.2)this.fail('夹持失败：双指未建立持续接触，请调整物体位置后重试');
    }else if(this.state==='lift'){
      this.evidence.lift=Math.max(this.evidence.lift,m.position.y-this.planData.start.y);
      if(t>=1.8){if(this.evidence.lift>.035&&m.count===2)this.transition('transfer');else this.fail('夹持滑脱：目标未稳定抬离桌面');}
    }else if(this.state==='transfer'&&t>=2)this.transition('place');
    else if(this.state==='place'){
      if(t>=1.8&&!m.support&&this.placeLowering<.009){
        this.placeLowering+=.00007;const point=this.planData.drop.clone();point.y-=this.placeLowering;
        const correction=solvePose(this.robot,'left',point,this.planData.orientation,this.pose,25,{limits:planningLimits});
        if(correction.positionError<.002&&correction.angleError<.025&&Math.max(...names.map(n=>Math.abs(correction.pose[n]-this.pose[n])))<.05){
          try{this.validateSegment(this.pose,correction.pose);this.placePose=correction.pose;}catch{}
        }this.dynamics.syncRobot();
      }
      this.evidence.stable=m.support&&m.distance<.012&&m.speed<.035?this.evidence.stable+.01:0;
      if(t>=1.8&&this.evidence.stable>.12)this.transition('release');else if(t>3.4)this.fail('放置未稳定：目标尚未接触正确支撑');
    }else if(this.state==='release'&&t>.75)this.transition('retreat');
    else if(this.state==='retreat'&&t>=1.6)this.transition('settling');
    else if(this.state==='settling'&&t>.4){if(m.support&&m.distance<.02&&m.speed<.025)this.finishItem();else if(t>1.5)this.fail('松爪后目标未在放置框内稳定');}
    else if(this.state==='stack_settle'){
      const check=this.stackCheck();this.stackSnapshot=check;this.stackStableSeconds=check.stable?this.stackStableSeconds+.01:0;
      if(this.stackStableSeconds>=3){this.stackResultVerified=true;this.beginReaction('happy');}else if(t>5)this.fail('叠放支撑或稳定判定未通过');
    }else if(this.state==='reaction_home'&&t>=2.5)this.transition(this.reaction);
    else if(['happy','shy'].includes(this.state)){
      const step=this.reactionSteps[this.reactionIndex];
      if((t>=step.hold&&this.jointError()<.07)||t>=step.timeout){
        this.reactionIndex++;if(this.reactionIndex>=this.reactionSteps.length){
          // The last phase target was already swept and accepted. Finish its
          // motor command exactly; never leave a fractional wrist gesture as
          // the next task's supposedly stationary home target.
          if(this.phaseGoal)this.applyPose(this.phaseGoal);
          this.running=false;this.state=this.reaction==='happy'?'complete':'failed';this.reactionFinished=this.reaction;
          if(this.reaction==='happy')this.message='叠放完成：三件物体已逐件释放并稳定支撑';
        }else this.transition(this.reaction);
      }
    }else if(this.state==='return'&&t>=2.5){this.running=false;this.state='complete';this.message='已夹取并放入对应颜色框';}
    if(['lift','transfer','place'].includes(this.state)&&m.position.y<TABLE_HEIGHT-.03)this.fail('目标已滑落，请复位后重试');
  }
}

