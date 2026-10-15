import {armJointNames,fingerJointNames,WAIST_JOINT_NAMES,LEG_JOINT_NAMES,openingToQ} from './g1-profile.mjs';

export class G1Dynamics {
  constructor(api,model,data,robot){
    Object.assign(this,{api,model,data,robot});this.map=new Map();this.targets={};this.previous={};this.targetVelocity={};
    const id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name);
    for(const name of Object.keys(robot.joints)){
      const joint=id('mjOBJ_JOINT',name);if(joint<0)continue;
      const finger=name.includes('_dex1_finger_joint_'),motor=finger?`${name.split('_')[0]}_motor${name.at(-1)}`:`${name}_motor`,actuator=id('mjOBJ_ACTUATOR',motor);if(actuator<0)continue;
      const lower=model.jnt_range[joint*2],upper=model.jnt_range[joint*2+1],kp=-model.actuator_biasprm[actuator*10+1],kv=-model.actuator_biasprm[actuator*10+2],effort=Math.max(Math.abs(model.actuator_forcerange[actuator*2]),Math.abs(model.actuator_forcerange[actuator*2+1]));
      this.map.set(name,{name,joint,q:model.jnt_qposadr[joint],d:model.jnt_dofadr[joint],actuator,lower,upper,effort,kp,kv,finger,unit:finger?'m':'rad'});
      this.targets[name]=data.qpos[model.jnt_qposadr[joint]];this.previous[name]=this.targets[name];this.targetVelocity[name]=0;
    }
    this.scratch=new api.MjData(model);this.bodyNames=Array.from({length:model.nbody},(_,i)=>api.mj_id2name(model,api.mjtObj.mjOBJ_BODY.value,i)||`body_${i}`);this.geomNames=Array.from({length:model.ngeom},(_,i)=>api.mj_id2name(model,api.mjtObj.mjOBJ_GEOM.value,i)||`geom_${i}`);
    this.robotBodyIds=new Set(Object.keys(robot.links||{}).map(name=>id('mjOBJ_BODY',name)).filter(i=>i>=0));this.footBodyIds=new Set(['left_ankle_roll_link','right_ankle_roll_link'].map(name=>id('mjOBJ_BODY',name)));this.floorGeom=id('mjOBJ_GEOM','floor');
  }
  setTargets(pose){for(const [name,value]of Object.entries(pose)){const j=this.map.get(name);if(j&&Number.isFinite(value)){this.targets[name]=Math.min(j.upper,Math.max(j.lower,value));if(j.finger)this.data.ctrl[j.actuator]=this.targets[name];}}}
  setTarget(name,value){this.setTargets({[name]:value});}
  reset(pose={}){
    this.setTargets(pose);for(const [name,j]of this.map){this.data.qpos[j.q]=this.targets[name];this.data.qvel[j.d]=0;this.previous[name]=this.targets[name];this.targetVelocity[name]=0;}
    this.api.mj_forward(this.model,this.data);this.beforeStep();this.syncRobot();
  }
  beforeStep(dt=.001){
    for(const [name,j]of this.map){
      if(j.finger){const target=Math.max(j.lower,Math.min(j.upper,this.data.ctrl[j.actuator]));this.targetVelocity[name]=(target-this.previous[name])/dt;this.targets[name]=target;this.previous[name]=target;continue;}
      const target=this.targets[name],raw=(target-this.previous[name])/dt;this.targetVelocity[name]+=.2*(Math.max(-8,Math.min(8,raw))-this.targetVelocity[name]);
      // Implicit affine motor PD, with gravity/Coriolis feedforward in ctrl.
      // qfrc_actuator therefore is the true total motor torque, including that
      // feedforward; no angle-based displayed torque estimate is used.
      this.data.ctrl[j.actuator]=j.kp*target+j.kv*this.targetVelocity[name]+this.data.qfrc_bias[j.d];this.previous[name]=target;
    }
  }
  afterStep(){this.syncRobot();}
  checkPose(pose,{opening,ignoreObjectBodies=[],objectPoses=[]}={}){
    const scratch=this.scratch;this.api.mj_resetData(this.model,scratch);scratch.qpos.set(this.data.qpos);scratch.qvel.fill(0);scratch.ctrl.set(this.data.ctrl);const limitViolations=[];
    for(const [name,value]of Object.entries(pose)){
      const j=this.map.get(name);if(!j)continue;if(!Number.isFinite(value)||value<j.lower||value>j.upper)limitViolations.push({name,value,lower:j.lower,upper:j.upper});if(Number.isFinite(value))scratch.qpos[j.q]=value;
    }
    if(opening!==undefined)for(const side of ['left','right']){
      const gap=typeof opening==='number'?opening:opening[side];if(!Number.isFinite(gap))continue;
      for(const name of fingerJointNames(side))scratch.qpos[this.map.get(name).q]=openingToQ(gap);
    }
    if(!Array.isArray(objectPoses))limitViolations.push({type:'invalid-object',reason:'objectPoses must be an array'});
    else for(const object of objectPoses){
      const invalid=reason=>limitViolations.push({type:'invalid-object',body:object?.body,reason});
      if(!object||typeof object!=='object'){invalid('Object pose must contain body, position and quaternion');continue;}
      const body=typeof object.body==='string'?this.api.mj_name2id(this.model,this.api.mjtObj.mjOBJ_BODY.value,object.body):object.body;
      if(!Number.isInteger(body)||body<=0||body>=this.model.nbody){invalid('Unknown body');continue;}
      const joint=this.model.body_jntadr[body],freeType=this.api.mjtJoint?.mjJNT_FREE?.value??0;
      if(this.model.body_jntnum[body]!==1||joint<0||this.model.jnt_type[joint]!==freeType){invalid('Body does not have a single free joint');continue;}
      const p=object.position,q=object.quaternion;
      if(!Array.isArray(p)||p.length!==3||!p.every(Number.isFinite)){invalid('Position must be a finite Three-space [x,y,z] array');continue;}
      if(!Array.isArray(q)||q.length!==4||!q.every(Number.isFinite)||Math.abs(Math.hypot(...q)-1)>1e-4){invalid('Quaternion must be a finite unit Three-space [x,y,z,w] array');continue;}
      const address=this.model.jnt_qposadr[joint];
      // For task shapes authored Y-up, Rx(PI/2) * q * Rx(-PI/2)
      // maps quaternion vector [x,y,z] to [x,-z,y], while w is unchanged.
      // This is the same conjugation used by controller.setObject.
      scratch.qpos.set([p[0],-p[2],p[1],q[3],q[0],-q[2],q[1]],address);
    }
    this.api.mj_forward(this.model,scratch);
    const ignore=new Set(ignoreObjectBodies.map(body=>typeof body==='number'?body:this.api.mj_name2id(this.model,this.api.mjtObj.mjOBJ_BODY.value,body))),contacts=[],list=scratch.contact;
    for(let i=0;i<Math.min(scratch.ncon,list.size());i++){
      const c=list.get(i),body1=this.model.geom_bodyid[c.geom1],body2=this.model.geom_bodyid[c.geom2],robot1=this.robotBodyIds.has(body1),robot2=this.robotBodyIds.has(body2),name1=this.bodyNames[body1],name2=this.bodyNames[body2];
      const footFloor=(this.footBodyIds.has(body1)&&c.geom2===this.floorGeom)||(this.footBodyIds.has(body2)&&c.geom1===this.floorGeom);
      const allowedFingerObject=(name1.includes('_dex1_finger_link_')&&ignore.has(body2))||(name2.includes('_dex1_finger_link_')&&ignore.has(body1));
      if((robot1||robot2)&&!footFloor&&!allowedFingerObject)contacts.push({dist:c.dist,body1:name1,body2:name2,body1Id:body1,body2Id:body2,geom1:this.geomNames[c.geom1],geom2:this.geomNames[c.geom2],geom1Id:c.geom1,geom2Id:c.geom2,position:Array.from(c.pos),worldPosition:[c.pos[0],c.pos[2],-c.pos[1]],kind:robot1&&robot2?'robot-self':'robot-environment'});
      c.delete();
    }
    list.delete();return {safe:limitViolations.length===0&&contacts.every(c=>c.dist>=0),contacts,minDistance:contacts.length?Math.min(...contacts.map(c=>c.dist)):null,limitViolations,positionFrame:'mujoco-z-up',worldPositionFrame:'three-y-up'};
  }
  dispose(){if(this.scratch){this.scratch.delete();this.scratch=null;}}
  syncRobot(){const actual={};for(const [name,j]of this.map)actual[name]=this.data.qpos[j.q];this.robot.setJointValues(actual);this.robot.updateMatrixWorld(true);return actual;}
  telemetry(arm='right'){
    const read=name=>{const j=this.map.get(name);return {name,target:this.targets[name],actual:this.data.qpos[j.q],error:this.targets[name]-this.data.qpos[j.q],velocity:this.data.qvel[j.d],targetVelocity:this.targetVelocity[name],torque:this.data.qfrc_actuator[j.d],biasTorque:this.data.qfrc_bias[j.d],lower:j.lower,upper:j.upper,effortLimit:j.effort,unit:j.unit,effortUnit:j.finger?'N':'N·m'};};
    const joints=armJointNames(arm).map(read),torso=WAIST_JOINT_NAMES.map(read),legs=LEG_JOINT_NAMES.map(read),fingers=fingerJointNames(arm).map(read),allJoints=[...this.map.keys()].map(read),body=allJoints.filter(j=>j.unit==='rad');
    const rms=rows=>Math.sqrt(rows.reduce((sum,j)=>sum+j.error*j.error,0)/Math.max(1,rows.length));
    return {joints,torso,legs,fingers,allJoints,all:allJoints,rms:rms(joints),bodyRms:rms(body),time:this.data.time};
  }
}
