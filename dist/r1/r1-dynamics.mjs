// The simulation remains Z-up in the URDF frame; Three displays that same tree
// with its root rotated -PI/2 around X. Scalar joint coordinates need no remap.
export class R1Dynamics {
  constructor(api,model,data,robot){
    Object.assign(this,{api,model,data,robot});this.map=new Map();this.targets={};this.previous={};this.targetVelocity={};
    const id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name);
    for(const name of Object.keys(robot.joints)){
      const physical=name.replace(/_gripper_finger_joint(\d)/,'_finger$1'),joint=id('mjOBJ_JOINT',physical);
      if(joint<0)continue;
      const finger=name.includes('_gripper_finger_joint'),index=Number(name.at(-1));
      const motor=finger?`${name.split('_')[0]}_motor${index}`:`${name}_motor`,actuator=id('mjOBJ_ACTUATOR',motor);
      if(actuator<0)continue;
      const kp=-model.actuator_biasprm[actuator*10+1],kv=-model.actuator_biasprm[actuator*10+2];
      // This runtime does not bind memory_view<bool>; finite URDF limits have
      // a nonzero span while the continuous wheel joints keep a zero range.
      const limited=model.jnt_range[joint*2+1]>model.jnt_range[joint*2],lower=limited?model.jnt_range[joint*2]:-Infinity,upper=limited?model.jnt_range[joint*2+1]:Infinity;
      const effort=Math.max(Math.abs(model.actuator_forcerange[actuator*2]),Math.abs(model.actuator_forcerange[actuator*2+1]));
      this.map.set(name,{name,joint,q:model.jnt_qposadr[joint],d:model.jnt_dofadr[joint],actuator,lower,upper,effort,kp,kv,finger,unit:finger?'m':'rad'});
      this.targets[name]=data.qpos[model.jnt_qposadr[joint]];this.previous[name]=this.targets[name];this.targetVelocity[name]=0;
    }
  }
  setTargets(pose){
    for(const [name,value] of Object.entries(pose)){const j=this.map.get(name);if(j&&Number.isFinite(value)){this.targets[name]=Math.min(j.upper,Math.max(j.lower,value));if(j.finger)this.data.ctrl[j.actuator]=this.targets[name];}}
  }
  reset(pose={}){
    this.setTargets(pose);
    for(const [name,j] of this.map){const value=this.targets[name];this.data.qpos[j.q]=value;this.data.qvel[j.d]=0;this.previous[name]=value;this.targetVelocity[name]=0;}
    this.api.mj_forward(this.model,this.data);this.beforeStep();this.syncRobot();
  }
  beforeStep(dt=.001){
    for(const [name,j] of this.map){
      if(j.finger){
        // Aperture motors are commanded by the task/manual controller, but
        // telemetry must still compare against that actual motor command.
        const target=Math.max(j.lower,Math.min(j.upper,this.data.ctrl[j.actuator]));
        this.targetVelocity[name]=(target-this.previous[name])/dt;this.targets[name]=target;this.previous[name]=target;continue;
      }
      const target=this.targets[name],raw=(target-this.previous[name])/dt;
      // Command paths are smooth; filtering keeps drag-event target changes
      // from producing unbounded derivative impulses. Motor limits still apply.
      this.targetVelocity[name]+=.2*(Math.max(-8,Math.min(8,raw))-this.targetVelocity[name]);
      const bias=this.data.qfrc_bias[j.d];
      // The affine motor contributes -Kp*q-Kd*qvel inside implicitfast.
      // Gravity/Coriolis feedforward is included in ctrl, and consequently in
      // the solver's actual qfrc_actuator, rather than a displayed estimate.
      this.data.ctrl[j.actuator]=j.kp*target+j.kv*this.targetVelocity[name]+bias;
      this.previous[name]=target;
    }
  }
  afterStep(){this.syncRobot();}
  syncRobot(){
    const actual={};for(const [name,j] of this.map)actual[name]=this.data.qpos[j.q];
    this.robot.setJointValues(actual);this.robot.updateMatrixWorld(true);return actual;
  }
  telemetry(arm='right'){
    const read=([name,j])=>({name,target:this.targets[name],actual:this.data.qpos[j.q],error:this.targets[name]-this.data.qpos[j.q],velocity:this.data.qvel[j.d],targetVelocity:this.targetVelocity[name],torque:this.data.qfrc_actuator[j.d],biasTorque:this.data.qfrc_bias[j.d],effortLimit:j.effort,unit:j.unit,effortUnit:j.finger?'N':'N·m'});
    const joints=[...this.map].filter(([name])=>name.startsWith(`${arm}_arm_joint`)).map(read);
    const torso=[...this.map].filter(([name])=>name.startsWith('torso_joint')).map(read);
    const fingers=[...this.map].filter(([name])=>name.startsWith(`${arm}_gripper_finger_joint`)).map(read);
    return {joints,torso,fingers,rms:Math.sqrt(joints.reduce((sum,j)=>sum+j.error*j.error,0)/Math.max(1,joints.length)),time:this.data.time};
  }
}
