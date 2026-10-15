import {R1Dynamics} from '../r1/r1-dynamics.mjs';

// A conservative multi-station command profile, not a different robot model.
// All four original R1 URDF fingers specify 0.25 m/s. Use 40% for this browser
// demonstration; original masses, motor forces, gains and soft limits stay put.
export const R1_FINGER_MOTION_PROFILE=Object.freeze({sourceVelocityMps:.25,commandFraction:.4,commandVelocityMps:.1});

export class R1SafeDynamics extends R1Dynamics {
  constructor(api,model,data,robot){
    super(api,model,data,robot);
    this.fingerDesiredTargets=Object.fromEntries([...this.map].filter(([,joint])=>joint.finger).map(([name])=>[name,this.targets[name]]));
    this.fingerRampedTargets={...this.fingerDesiredTargets};
  }
  setTargets(pose){
    super.setTargets(pose);
    if(this.fingerDesiredTargets)for(const [name,joint]of this.map)if(joint.finger&&Number.isFinite(pose[name])){
      this.fingerDesiredTargets[name]=this.targets[name];this.targets[name]=this.fingerRampedTargets[name];
    }
  }
  reset(pose={}){
    // Explicit scene reset may set actual state, as in the retained runtime.
    // Reset both command histories before the parent's reset/beforeStep calls;
    // a previous task's pending close must never resume after reset.
    for(const [name,joint]of this.map)if(joint.finger){
      const target=Math.max(joint.lower,Math.min(joint.upper,pose[name]??this.targets[name]));
      this.fingerDesiredTargets[name]=target;this.fingerRampedTargets[name]=target;this.data.ctrl[joint.actuator]=target;
    }
    super.reset(pose);
  }
  beforeStep(dt=.001){
    for(const [name,joint]of this.map)if(joint.finger){
      const previous=this.fingerRampedTargets[name],incoming=Math.max(joint.lower,Math.min(joint.upper,this.data.ctrl[joint.actuator]));
      // Controller writes are desired apertures, not ramp history. A one-shot
      // setTargets request also persists when ctrl still equals the last ramp.
      if(Number.isFinite(incoming)&&Math.abs(incoming-previous)>1e-12)this.fingerDesiredTargets[name]=incoming;
      const step=R1_FINGER_MOTION_PROFILE.commandVelocityMps*dt;
      const target=Math.max(previous-step,Math.min(previous+step,this.fingerDesiredTargets[name]));
      this.data.ctrl[joint.actuator]=target;this.fingerRampedTargets[name]=target;
    }
    // Parent reads the actual ramped motor command for target/velocity telemetry
    // and retains its original arm feedforward. No actual qpos/qvel is clamped.
    super.beforeStep(dt);
  }
  telemetry(arm='right'){
    const telemetry=super.telemetry(arm);
    telemetry.fingers=telemetry.fingers.map(joint=>({...joint,desiredTarget:this.fingerDesiredTargets[joint.name],
      velocityLimit:R1_FINGER_MOTION_PROFILE.sourceVelocityMps,commandVelocityLimit:R1_FINGER_MOTION_PROFILE.commandVelocityMps}));
    return telemetry;
  }
}
