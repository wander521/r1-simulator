import {G1Dynamics} from '../g1/g1-dynamics.mjs';
import {armJointNames,fingerJointNames,BASE_LINK,JOINT_VELOCITY_LIMITS} from './c5-profile.mjs';

// Share the already-tested, state-isolated MuJoCo mechanics, not the G1 chain:
// this model/map contains only six C5 axes and the two original left Dex1 axes.
export class C5Dynamics extends G1Dynamics {
  constructor(api,model,data,robot){
    super(api,model,data,robot);this.footBodyIds.clear();
    this.baseBody=api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,BASE_LINK);this.plinthBody=api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,'robot_plinth');
    for(const [name,j]of this.map)j.velocityLimit=JOINT_VELOCITY_LIMITS[name];
    this.fingerDesiredTargets=Object.fromEntries(fingerJointNames().map(name=>[name,this.targets[name]]));this.fingerRampedTargets={...this.fingerDesiredTargets};
  }
  setTargets(pose){
    super.setTargets(pose);
    if(this.fingerDesiredTargets)for(const name of fingerJointNames())if(Number.isFinite(pose[name])){this.fingerDesiredTargets[name]=this.targets[name];this.targets[name]=this.fingerRampedTargets[name];}
  }
  reset(pose={}){
    if(this.fingerDesiredTargets)for(const name of fingerJointNames()){
      const j=this.map.get(name),target=Math.max(j.lower,Math.min(j.upper,pose[name]??this.targets[name]));this.fingerDesiredTargets[name]=target;this.fingerRampedTargets[name]=target;this.data.ctrl[j.actuator]=target;
    }
    super.reset(pose);
  }
  beforeStep(dt=.001){
    for(const [name,j]of this.map){
      if(j.finger){
        const incoming=Math.max(j.lower,Math.min(j.upper,this.data.ctrl[j.actuator])),previous=this.fingerRampedTargets[name];
        // The controller writes desired ctrl every millisecond. Keep a separate
        // ramp state: the incoming desired must not overwrite the ramp history.
        // A setTargets request also persists if the caller does not rewrite ctrl.
        if(Math.abs(incoming-previous)>1e-12)this.fingerDesiredTargets[name]=incoming;
        const step=j.velocityLimit*dt,target=Math.max(previous-step,Math.min(previous+step,this.fingerDesiredTargets[name]));
        this.data.ctrl[j.actuator]=target;this.fingerRampedTargets[name]=target;this.targetVelocity[name]=(target-previous)/dt;this.targets[name]=target;this.previous[name]=target;continue;
      }
      const target=this.targets[name],raw=(target-this.previous[name])/dt,velocity=Number.isFinite(j.velocityLimit)&&j.velocityLimit>0?j.velocityLimit:8;
      this.targetVelocity[name]+=.2*(Math.max(-velocity,Math.min(velocity,raw))-this.targetVelocity[name]);
      // The implicit PD motor includes gravity/Coriolis feedforward in ctrl;
      // displayed qfrc_actuator is therefore the actual complete motor torque.
      this.data.ctrl[j.actuator]=j.kp*target+j.kv*this.targetVelocity[name]+this.data.qfrc_bias[j.d];this.previous[name]=target;
    }
  }
  checkPose(pose,options={}){
    // Parent's opening helper supports two Dex1s. This assembly has only left;
    // accepting a scalar keeps the single-gripper controller independent of it.
    const opening=options.opening===undefined?undefined:{left:typeof options.opening==='number'?options.opening:options.opening.left??options.opening.single??options.opening.right};
    const check=super.checkPose(pose,{...options,opening}),contacts=check.contacts.filter(c=>!((c.body1Id===this.baseBody&&c.body2Id===this.plinthBody)||(c.body2Id===this.baseBody&&c.body1Id===this.plinthBody)));
    return {...check,contacts,safe:check.limitViolations.length===0&&contacts.every(c=>c.dist>=0),minDistance:contacts.length?Math.min(...contacts.map(c=>c.dist)):null};
  }
  telemetry(){
    const read=name=>{const j=this.map.get(name);return {name,target:this.targets[name],desiredTarget:j.finger?this.fingerDesiredTargets[name]:this.targets[name],actual:this.data.qpos[j.q],error:this.targets[name]-this.data.qpos[j.q],velocity:this.data.qvel[j.d],targetVelocity:this.targetVelocity[name],velocityLimit:j.velocityLimit,torque:this.data.qfrc_actuator[j.d],biasTorque:this.data.qfrc_bias[j.d],lower:j.lower,upper:j.upper,effortLimit:j.effort,unit:j.unit,effortUnit:j.finger?'N':'N·m'};};
    const joints=armJointNames().map(read),fingers=fingerJointNames().map(read),allJoints=[...this.map.keys()].map(read),rms=Math.sqrt(joints.reduce((sum,j)=>sum+j.error*j.error,0)/joints.length);
    return {joints,torso:[],legs:[],fingers,allJoints,all:allJoints,rms,bodyRms:rms,time:this.data.time};
  }
}
