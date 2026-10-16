import * as THREE from '../vendor/three.module.min.js';
import {solvePose as solveG1Pose,naturalArmLimits,naturalArmPose} from '../g1/grasp-kinematics.mjs';
import {STAND_POSE,armJointNames,qToOpening,GRIPPER_Q_MAX} from '../g1/g1-profile.mjs';
import {READY as G1_READY} from '../g1/grasp-controller.mjs';

const G1_OPEN=qToOpening(GRIPPER_Q_MAX);
export const PIPE_GRIPPER_PROFILE=Object.freeze({r1PreopenJointM:.024,dex1HoldJointM:-.0037,dex1NominalCylinderDiameterM:.040,c5FingerCommandFraction:.75,g1FingerSourceVelocityMps:.2,g1FingerCommandFraction:.75});
export function pipeDynamicsClass(LegacyDynamics,{robotId}){
  return class PipeGraspDynamics extends LegacyDynamics{
    constructor(...args){super(...args);this.pipeFingerDesired=Object.fromEntries([...this.map].filter(([,joint])=>joint.finger).map(([name])=>[name,this.targets[name]]));this.pipeFingerRamped={...this.pipeFingerDesired};}
    command(value){return robotId==='r1'?Math.min(PIPE_GRIPPER_PROFILE.r1PreopenJointM,value):Math.max(PIPE_GRIPPER_PROFILE.dex1HoldJointM,value);}
    reset(pose={}){
      const safe={...pose};for(const[name,joint]of this.map)if(joint.finger&&Number.isFinite(safe[name]))safe[name]=this.command(safe[name]);
      for(const[name,joint]of this.map)if(joint.finger){const target=safe[name]??this.command(this.targets[name]);this.pipeFingerDesired[name]=target;this.pipeFingerRamped[name]=target;}
      super.reset(safe);
    }
    beforeStep(dt=.001){
      // This is a task aperture/preload command, not a model or actual-state
      // clamp. Keep the supplied slide ranges, motor forces and PD unchanged.
      for(const[name,joint]of this.map)if(joint.finger){
        const incoming=this.command(this.data.ctrl[joint.actuator]);
        if(robotId==='c5'||robotId==='g1'){
          const previous=this.pipeFingerRamped[name];if(Math.abs(incoming-previous)>1e-12)this.pipeFingerDesired[name]=incoming;
          const velocity=robotId==='c5'?joint.velocityLimit*PIPE_GRIPPER_PROFILE.c5FingerCommandFraction:PIPE_GRIPPER_PROFILE.g1FingerSourceVelocityMps*PIPE_GRIPPER_PROFILE.g1FingerCommandFraction;
          const step=velocity*dt,target=Math.max(previous-step,Math.min(previous+step,this.pipeFingerDesired[name]));
          this.data.ctrl[joint.actuator]=target;this.pipeFingerRamped[name]=target;
        }else this.data.ctrl[joint.actuator]=incoming;
      }
      super.beforeStep(dt);
    }
  };
}
const sidePinch=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2);
function planG1Pipe(controller,target,yawOffset=0,seedIndex=0,lateral=.14,tiltDegrees=20,reverse=false){
  const c=controller,start=c.objectPose(target.id).position,drop=target.drop.clone();drop.y=start.y;
  if(start.x>.65||Math.hypot(start.x,start.z)>.66||drop.x>.65)throw new Error('圆管超出固定站姿的抓取范围');
  const names=armJointNames(c.arm),sign=c.arm==='left'?1:-1;
  const align=p=>THREE.MathUtils.clamp(Math.atan2(-p.z,p.x)-Math.asin(THREE.MathUtils.clamp(sign*lateral/Math.hypot(p.x,p.z),-.9,.9)),-1,1);
  const startYaw=align(start),dropYaw=align(drop),tilt=tiltDegrees*Math.PI/180;
  const quaternion=(yaw,down=0)=>new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),yaw+yawOffset).multiply(sidePinch).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),down));
  const orientation=quaternion(startYaw,tilt),dropOrientation=quaternion(dropYaw,tilt),stand={...STAND_POSE,waist_yaw_joint:startYaw,waist_pitch_joint:.10};
  const seeds=[naturalArmPose(c.arm),[.15,.5*sign,.2*sign,-.5,0,.15,0],[-.6,.3*sign,0,.2,0,.1,0],[-.27,.15*sign,.03*sign,-.85,-.24*sign,1.10,.24*sign],[-.3,.7*sign,-.3*sign,-.4,.4*sign,.3,0],[-.7,.45*sign,.4*sign,-.8,-.3*sign,.7,0]];
  let seed={...stand,...G1_READY.right,...G1_READY.left,...Object.fromEntries(names.map((n,i)=>[n,seeds[seedIndex][i]]))};
  const solve=(point,continuous=false,down=0)=>{
    const yaw=align(point);seed.waist_yaw_joint=yaw;const result=solveG1Pose(c.robot,c.arm,point,quaternion(yaw,down),seed,continuous?280:400,{limits:naturalArmLimits(c.arm)});
    if(result.positionError>.003||result.angleError>.035){const error=new Error('圆管轨迹超出当前站姿可稳定到达的范围');error.detail={position:point.toArray(),positionError:result.positionError,angleError:result.angleError};throw error;}
    const jump=Math.max(...names.map(n=>Math.abs(result.pose[n]-(seed[n]??0))));
    if(continuous&&jump>.30){const error=new Error('圆管连续抓取轨迹不可达');error.detail={position:point.toArray(),jointJump:jump};throw error;}
    seed={...stand,...G1_READY.right,...G1_READY.left,...result.pose};return{...seed};
  };
  // The near-side lift retains 110 mm clearance; the destination retreat uses
  // 80 mm vertical clearance without another forward lunge. All original
  // swept collision and natural joint-limit checks are retained.
  const above=start.clone().add(new THREE.Vector3(.04,.11,0)),aboveDrop=drop.clone().add(new THREE.Vector3(0,.08,0));
  const line=(a,b,ta=0,tb=ta)=>Array.from({length:41},(_,i)=>solve(a.clone().lerp(b,i/40),true,THREE.MathUtils.lerp(ta,tb,i/40)));
  // Solve backwards from the constrained destination. A redundant 7-axis
  // chain can have a valid grasp branch that cannot continue to the far bin;
  // destination-first continuation finds a continuous branch for both ends.
  let approach,descend,lift,transfer,place,retreat;
  if(reverse){
    solve(drop,false,tilt);const reversePlace=line(drop,aboveDrop,tilt,0),reverseTransfer=line(aboveDrop,above);descend=line(above,start,0,tilt);approach=descend[0];
    lift=[...descend].reverse();transfer=[...reverseTransfer].reverse();place=[...reversePlace].reverse();retreat=reversePlace;
  }else{approach=solve(above);descend=line(above,start,0,tilt);lift=line(start,above,tilt,0);transfer=line(above,aboveDrop);place=line(aboveDrop,drop,0,tilt);retreat=line(drop,aboveDrop,tilt,0);}
  const objectRotation=c.objectPose(target.id).quaternion,closingAxis=new THREE.Vector3(0,1,0).applyQuaternion(orientation),vertical=Math.abs(new THREE.Vector3(0,1,0).applyQuaternion(objectRotation).dot(closingAxis));
  const width=2*(target.halfX*Math.sqrt(Math.max(0,1-vertical*vertical))+target.h*vertical);
  const plan={start,drop,startYaw,dropYaw,stand,objectRotation:objectRotation.clone(),orientation:dropOrientation,graspOrientation:orientation,preOpening:Math.min(G1_OPEN,width+.018),approach,
    descend,lift,transfer,place,retreat};
  const ignore=[c.bodies.get(c.selected)],opening={[c.arm]:plan.preOpening,[c.arm==='right'?'left':'right']:G1_OPEN};
  for(const phase of ['descend','lift','transfer','place','retreat'])for(let i=0;i<plan[phase].length;i++){
    const path=plan[phase],objectPoseFor=phase==='descend'?null:pose=>c.predictedObject(pose,plan,phase);
    const check=c.dynamics.checkPose(path[i],{opening,ignoreObjectBodies:ignore,objectPoses:objectPoseFor?.(path[i])});
    if(!check.safe){const error=new Error('圆管轨迹与身体、桌面或邻管发生干涉');error.detail={phase,sample:i,contacts:check.contacts.slice(0,3),limits:check.limitViolations};throw error;}
    if(i)c.validateSegment(path[i-1],path[i],{opening,objectPoseFor});
  }
  c.planData=plan;
  // A collision-free Cartesian path is insufficient if the transition from
  // READY enters it through the table. Check the same staged entry/return
  // routes the source controller will execute before accepting this branch.
  const savedPose=c.pose;
  try{
    let from={...savedPose};
    for(const state of ['spread','raise','ready','align','approach']){
      c.pose=from;const to=c.phaseTarget(state),gaps=state==='approach'?opening:{left:G1_OPEN,right:G1_OPEN};
      c.validateSegment(from,to,{opening:gaps,openingFrom:state==='approach'?{left:G1_OPEN,right:G1_OPEN}:null});from={...to};
    }
    from={...plan.retreat.at(-1)};
    for(const state of ['return_clear','return_align','return_fold','return_wide','return','return_center']){
      c.pose=from;const to=c.phaseTarget(state);c.validateSegment(from,to,{opening,objectPoseFor:pose=>c.predictedObject(pose,plan,'retreat')});from={...to};
    }
  }finally{c.pose=savedPose;}
}

/** Extend the validated single-object controller, never replace its contacts
 * with carries, welds, mocap bodies, synthetic forces or object pose writes.
 */
export function pipeControllerClass(LegacyController,{robotId}){
  return class PipeClassificationController extends LegacyController{
    constructor(...args){super(...args);this.robotId=robotId;this.pipeLayout=null;this.selected=this.targets[0].id;}
    reset(){
      super.reset();this.selected=this.targets[0]?.id;this.task='pipes';
      this.classification={total:8,completed:0,activeIndex:0,seed:this.pipeLayout?.seed??null,queue:this.pipeLayout?.queue??[],started:false,armSequence:[],items:[],bins:this.pipeLayout?.bins??[],verified:false,finalStableSeconds:0,check:null};
      this.message=robotId==='c5'?'圆管分类待命 · C5 单臂自动执行':'圆管分类待命 · 双臂自动交替';
    }
    setLayout(layout){this.pipeLayout=layout;this.reset();}
    setObject(...args){if(this.running)return false;return super.setObject(...args);}
    phaseTarget(state){
      // Retract vertically first, then turn away from the table with the safe
      // arm shape unchanged. Folding at the bin-facing waist yaw sweeps the
      // elbow through the table edge even when both endpoint poses are clear.
      if(robotId==='g1'&&state==='return_clear'&&this.planData?.retreat)return{...this.planData.retreat.at(-1),waist_yaw_joint:0};
      const goal=super.phaseTarget?.(state);
      if(robotId==='g1'&&goal&&this.planData?.stand&&state!=='return_center')return{...goal,waist_pitch_joint:this.planData.stand.waist_pitch_joint??0};
      return goal;
    }
    objectPose(id=this.selected){
      // The old planners use id="yellow" as their cylinder shape descriptor.
      // Scope that legacy descriptor to planning only; actual target IDs,
      // contact body IDs and collision-ignore sets remain unique for all eight.
      return super.objectPose(this._pipeCylinderShape&&id==='yellow'?this.selected:id);
    }
    *planCandidates(target,...args){
      if(robotId==='g1'){
        let error;for(const reverse of [false,true])for(const tilt of [0,12,20,-12,28]){try{return planG1Pipe(this,target,...args,tilt,reverse);}catch(caught){error=caught;}yield;}throw error;
      }
      this._pipeCylinderShape=target.shape==='cylinder';
      try{return super.plan(this._pipeCylinderShape?{...target,id:'yellow'}:target,...args);}
      finally{this._pipeCylinderShape=false;}
    }
    plan(target,...args){const candidates=this.planCandidates(target,...args);let step;do{step=candidates.next();}while(!step.done);return step.value;}
    startClassification(){
      if(this.running||!this.pipeLayout)return false;
      if(this.classification.started){this.message='请先复位或重新随机摆放，再开始新批次';return false;}
      this.classification={total:8,completed:0,activeIndex:0,seed:this.pipeLayout.seed,queue:[...this.pipeLayout.queue],started:false,armSequence:[],items:[],bins:this.pipeLayout.bins,verified:false,finalStableSeconds:0,check:null};
      const accepted=this.startPipeItem();if(accepted)this.classification.started=true;return accepted;
    }
    startPipeItem(){
      const index=this.classification.completed,id=this.pipeLayout.queue[index],arm=this.robotId==='c5'?'left':index%2===0?'right':'left';
      if(!id)return false;
      this.classification.activeIndex=index;const history=this.history??[];
      const accepted=super.start(arm,id);this.task='pipes';
      this.history=[...history,...this.history];
      if(accepted){this.classification.armSequence.push(arm);this.message=`正在分类第 ${index+1}/8 根圆管`;}
      return accepted;
    }
    allPipesCheck(){
      const states=[],table=this.id('mjOBJ_BODY','task_table'),contacts=this.data.contact,fingerBodies=new Set(Object.values(this.fingers).flat().map(finger=>finger.body));
      try{for(const target of this.targets){
        const body=this.bodies.get(target.id),joint=this.model.body_jntadr[body],d=this.model.jnt_dofadr[joint],position=this.objectPose(target.id).position;
        const bin=this.pipeLayout.bins.find(item=>item.id===target.colorKey);let supported=false,held=false;
        for(let i=0;i<Math.min(this.data.ncon,contacts.size());i++){const c=contacts.get(i);try{
          const a=this.model.geom_bodyid[c.geom1],b=this.model.geom_bodyid[c.geom2],force=c.efc_address>=0?Math.abs(this.data.efc_force[c.efc_address]):0;
          if(((a===body&&b===table)||(b===body&&a===table))&&force>.005)supported=true;
          if(((a===body&&fingerBodies.has(b))||(b===body&&fingerBodies.has(a)))&&force>.005)held=true;
        }finally{c.delete();}}
        const pose=this.objectPose(target.id),axis=new THREE.Vector3(0,1,0).applyQuaternion(pose.quaternion);
        const extents={x:target.halfX*Math.sqrt(Math.max(0,1-axis.x*axis.x))+target.h*Math.abs(axis.x),z:target.halfZ*Math.sqrt(Math.max(0,1-axis.z*axis.z))+target.h*Math.abs(axis.z)};
        const speed=Math.hypot(...this.data.qvel.subarray(d,d+3)),inside=Math.abs(position.x-bin.center.x)+extents.x<=bin.halfX&&Math.abs(position.z-bin.center.z)+extents.z<=bin.halfZ,distance=position.distanceTo(target.drop);
        states.push({id:target.id,colorKey:target.colorKey,position:position.toArray(),supported,released:!held,inside,speed,distance,stable:supported&&!held&&inside&&distance<.02&&speed<.025});
      }}finally{contacts.delete();}
      return{stable:states.every(state=>state.stable),states};
    }
    step(dt=.001){
      // Slow the task's phase clock, never the 1 ms physics clock. This gives
      // the C5's original velocity-limited motors room for physical tracking
      // without increasing a source velocity, torque or collision threshold.
      if(robotId==='c5'&&this.running&&this.state!=='pipe_settle')this.elapsed-=dt*(1-1/(this.state==='lift'?2:1.35));
      const before=this.state;super.step(dt);
      if(this.state==='complete'&&before!=='complete'&&this.classification.completed<8){
        const target=this.targets.find(t=>t.id===this.selected),m=this.measure();
        this.classification.items.push({id:this.selected,colorKey:target.colorKey,arm:this.arm,drop:this.planData.drop.toArray(),position:m.position.toArray(),...this.evidence,tableSupport:m.support,placementError:m.distance,speed:m.speed});
        this.classification.completed++;
        if(this.classification.completed<8){if(!this.startPipeItem()){this.running=false;this.state='failed';}return;}
        this.running=true;this.state='pipe_settle';this.elapsed=0;this.phaseGoal=null;this.from={...this.pose};this.history.push(this.state);this.message='8 根圆管已放置，正在检查全部框内支撑';
      }
      if(this.state==='pipe_settle'){
        // This check is expensive; state evidence shares the source 100 Hz
        // cadence. The physical worlds still step at exactly 1 ms.
        if(this.tick%10)return;const check=this.allPipesCheck();this.classification.check=check;
        this.classification.finalStableSeconds=check.stable?this.classification.finalStableSeconds+.01:0;
        if(this.classification.finalStableSeconds>=3){this.classification.verified=true;this.running=false;this.state='complete';this.message='分类完成：4 根黄色圆管和 4 根红色圆管已在对应框内稳定落桌';}
        else if(this.elapsed>8){this.running=false;this.state='failed';this.message='分类未通过：部分圆管未在对应颜色框内稳定落桌';}
      }
    }
  };
}
