import * as THREE from '../vendor/three.module.min.js';
import {GraspController} from '../c5/grasp-controller.mjs';
import {solvePose} from '../c5/grasp-kinematics.mjs';
import {TABLE_HEIGHT,armJointNames,openingToQ,qToOpening,GRIPPER_Q_MIN,GRIPPER_Q_MAX} from '../c5/c5-profile.mjs';

const OPEN=qToOpening(GRIPPER_Q_MAX),CLOSED=qToOpening(GRIPPER_Q_MIN);
const smooth=t=>{t=THREE.MathUtils.clamp(t,0,1);return t*t*(3-2*t);};
const mix=(a,b,t)=>Object.fromEntries(Object.keys(b).map(k=>[k,THREE.MathUtils.lerp(a[k]??b[k],b[k],t)]));
const names=armJointNames('left');
const planningLimits=[[-Math.PI,Math.PI],[-2.9,2.9],[-2.9,2.9],[-Math.PI,Math.PI],[-2.9,2.9],[-Math.PI,Math.PI]];

// Shared-bin execution uses an object-to-TCP offset and a scene-specific carry
// floor. Keep this step variant here so standalone C5 retains its original
// source/controller and independent acceptance evidence. All other behavior
// is inherited. When the native step changes, review this variant with it.
export class CollaborationC5Controller extends GraspController{
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
        this.placeLowering+=.00007;const point=this.planData.drop.clone();if(this.planData.tcpOffset)point.add(this.planData.tcpOffset);point.y-=this.placeLowering;
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
    if(['lift','transfer','place'].includes(this.state)&&m.position.y<(this.planData?.minimumCarryHeight??TABLE_HEIGHT-.03))this.fail('目标已滑落，请复位后重试');
  }
}
