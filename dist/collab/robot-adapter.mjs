import * as THREE from '../vendor/three.module.min.js';
import {ROBOT_CONFIGS} from '../multi/robot-runtime.mjs';
import {openingToQ as dexOpeningToQ,qToOpening as dexQToOpening,GRIPPER_Q_MAX as DEX_MAX} from '../g1/g1-profile.mjs';
import {REST as R1_REST,REST_TORSO as R1_TORSO,SPREAD as R1_SPREAD,CROUCH as R1_CROUCH} from '../r1/grasp-controller.mjs';
import {HOME_POSE as C5_HOME} from '../c5/c5-profile.mjs';
import {solvePose as solveC5Posture,topDownQuaternion as c5TopDown} from '../c5/grasp-kinematics.mjs';
import {planG1Task,planC5Task} from './task-plans.mjs';
import {buildHeldPlacementPlan} from './held-placement.mjs';
import {remainingMotionSegments} from './resume-route.mjs';
import {CollaborationC5Controller} from './c5-controller.mjs';

const basis=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI/2);
const inverseBasis=basis.clone().invert();
const vector=value=>value?.isVector3?value.clone():new THREE.Vector3(...value);
const rotation=value=>value?.isQuaternion?value.clone():new THREE.Quaternion(...(value??[0,0,0,1]));
const requireBody=(model,id)=>{if(!Number.isInteger(id)||id<=0||id>=model.nbody)throw new Error(`Unknown shared body: ${id}`);return id;};
const readWorldPosition=(data,id)=>new THREE.Vector3(data.xpos[id*3],data.xpos[id*3+2],-data.xpos[id*3+1]);
const readWorldRotation=(data,id)=>inverseBasis.clone().multiply(new THREE.Quaternion(data.xquat[id*4+1],data.xquat[id*4+2],data.xquat[id*4+3],data.xquat[id*4])).multiply(basis);

/** Canonical task/URDF Three coordinates ↔ the one shared Three world. */
export function createRigidFrame({position=[0,0,0],yaw=0,quaternion}={}){
  const origin=vector(position),q=quaternion?rotation(quaternion):new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),yaw);
  if(!origin.toArray().every(Number.isFinite)||!q.toArray().every(Number.isFinite)||Math.abs(q.lengthSq()-1)>1e-6)throw new Error('Invalid shared robot rigid frame');
  const inverse=q.clone().invert();
  return Object.freeze({position:origin.clone(),quaternion:q.clone(),
    toWorld:point=>vector(point).applyQuaternion(q).add(origin),
    toLocal:point=>vector(point).sub(origin).applyQuaternion(inverse),
    rotationToWorld:value=>q.clone().multiply(rotation(value)),
    rotationToLocal:value=>inverse.clone().multiply(rotation(value)),
    vectorToWorld:value=>vector(value).applyQuaternion(q),
    vectorToLocal:value=>vector(value).applyQuaternion(inverse)});
}

/** A moving mount is read from real MuJoCo body transforms, never a visual
 * group or a desired navigation command. forData also supports worker/scratch
 * snapshots without rebuilding a controller or resetting any joint. */
export function createTrackedFrame({model,data,mountBody}){
  requireBody(model,mountBody);let version=0,last=null;
  const read=source=>createRigidFrame({position:readWorldPosition(source,mountBody),quaternion:readWorldRotation(source,mountBody)});
  const sync=()=>{
    const value=read(data),signature=[...value.position.toArray(),...value.quaternion.toArray()];
    if(!last||signature.some((v,i)=>Math.abs(v-last[i])>1e-12)){last=signature;version++;}
    return {position:value.position.toArray(),quaternion:value.quaternion.toArray(),version,mountBody,tracked:true};
  };
  return Object.freeze({tracked:true,mountBody,sync,forData:read,
    get position(){return read(data).position;},get quaternion(){return read(data).quaternion;},
    toWorld:point=>read(data).toWorld(point),toLocal:point=>read(data).toLocal(point),
    rotationToWorld:q=>read(data).rotationToWorld(q),rotationToLocal:q=>read(data).rotationToLocal(q),
    vectorToWorld:v=>read(data).vectorToWorld(v),vectorToLocal:v=>read(data).vectorToLocal(v)});
}

export function validatePresetRoute(controller,phases,{from=controller.pose,opening,openingFrom=null,objectPoseFor=null}={}){
  let previous={...from};const saved=controller.pose,checked=[];
  try{for(const item of phases){
    controller.pose=previous;
    const next=typeof item==='string'?controller.phaseTarget(item):item.pose,phase=typeof item==='string'?item:item.phase;
    if(!next)continue;
    try{controller.validateSegment(previous,next,{opening,openingFrom,ignoreSelected:false,objectPoseFor});}
    catch(error){error.detail={...error.detail,stage:'preset-route',phase};throw error;}
    previous={...next};checked.push(phase);
  }}finally{controller.pose=saved;}
  return checked;
}

/** IDs are ALWAYS indices in shared model/data. No local qpos/ctrl arrays. */
export function createNamespacedAPI(base,{model,liveData,prefix,aliases={},sharedNames=['floor']}={}){
  if(typeof prefix!=='string'||!prefix)throw new Error('A nonempty robot namespace is required');
  const names=new Map(Object.entries(aliases)),shared=new Set(sharedNames);
  let initializing=0,controlling=0;
  const stats={suppressedInitializationResets:0,controlStepRequests:0};
  const resolve=name=>names.has(name)?names.get(name):shared.has(name)||name.startsWith(prefix)?name:`${prefix}${name}`;
  const wrappers={
    mj_name2id:(source,type,name)=>base.mj_name2id(source,type,resolve(name)),
    mj_id2name:(source,type,id)=>{const name=base.mj_id2name(source,type,id);return name?.startsWith(prefix)?name.slice(prefix.length):name;},
    mj_resetData:(source,target)=>{
      if(target!==liveData)return base.mj_resetData(source,target);
      if(!initializing)throw new Error('Per-robot reset of shared live MjData is forbidden');
      // The scene owner initialized it once. The original constructor resets
      // only its mapped joints after this no-op; other robots keep their state.
      stats.suppressedInitializationResets++;
    },
    mj_step:(source,target)=>{
      if(target!==liveData)throw new Error('Robot adapters never own physics stepping');
      if(!controlling)throw new Error('Call stepControls, then step shared physics exactly once');
      stats.controlStepRequests++;
    }};
  const api=new Proxy(base,{get:(target,key)=>key in wrappers?wrappers[key]:Reflect.get(target,key)});
  return {api,prefix,stats,resolve,
    setAlias:(name,globalName)=>{if(typeof globalName!=='string'||!globalName)throw new Error('Alias must name a shared model object');names.set(name,globalName);},
    removeAlias:name=>names.delete(name),getAlias:name=>names.get(name),
    withInitialization:fn=>{initializing++;try{return fn();}finally{initializing--;}},
    withControl:fn=>{controlling++;try{return fn();}finally{controlling--;}},
    get initializing(){return initializing>0;},model};
}

/** Full-model collision mirror. Writes only scratch qpos, never the live scene. */
export class SharedCollisionChecker{
  constructor({api,model,data,jointMap,robotBodyIds,fingerBodyIds=[],frame,allowedBodyPairs=[],allowedGeomPairs=[],floorGeomIds=[],floorBodyIds=[]}){
    Object.assign(this,{api,model,data,jointMap,frame});this.scratch=new api.MjData(model);
    this.robotBodyIds=new Set(robotBodyIds);this.fingerBodyIds=new Set(fingerBodyIds);
    this.floorGeomIds=new Set(floorGeomIds);this.floorBodyIds=new Set(floorBodyIds);
    this.allowedBodyPairs=new Set(allowedBodyPairs.map(([a,b])=>[a,b].sort((x,y)=>x-y).join(':')));
    this.allowedGeomPairs=new Set(allowedGeomPairs.map(([a,b])=>[a,b].sort((x,y)=>x-y).join(':')));
    this.payloadAssembly=null;
  }
  /** Capture a conservative rigid prediction from actual loose-body poses.
   * This only belongs to the collision mirror. Live cargo remains free and
   * must separately pass contact/retention checks while carrying. */
  setPayloadAssembly({body,payloadBodies=[],supportBodies=[],carrier=false}={}){
    this.payloadAssembly=null;if(!payloadBodies.length&&!carrier)return;
    const id=value=>requireBody(this.model,typeof value==='number'?value:this.api.mj_name2id(this.model,this.api.mjtObj.mjOBJ_BODY.value,value));
    const parent=id(body),p=readWorldPosition(this.data,parent),q=readWorldRotation(this.data,parent),inverse=q.clone().invert();
    const members=payloadBodies.map(value=>{
      const b=id(value);if(b===parent)throw new Error('Payload cannot contain its carrier');
      return {body:b,position:readWorldPosition(this.data,b).sub(p).applyQuaternion(inverse),quaternion:inverse.clone().multiply(readWorldRotation(this.data,b))};
    });
    if(new Set(members.map(m=>m.body)).size!==members.length)throw new Error('Duplicate payload body');
    this.payloadAssembly={body:parent,members,supports:new Set(supportBodies.filter(Boolean).map(id))};
  }
  check(pose,{opening,ignoreObjectBodies=[],objectPoses=[],baseJointPositions={},protectObjects=false}={}){
    const {api,model,data,scratch}=this;api.mj_resetData(model,scratch);scratch.qpos.set(data.qpos);scratch.qvel.fill(0);scratch.ctrl.set(data.ctrl);
    const limitViolations=[];
    for(const [name,value]of Object.entries(pose)){
      const joint=this.jointMap.get(name);if(!joint)continue;
      if(!Number.isFinite(value)||value<joint.lower||value>joint.upper)limitViolations.push({name,value,lower:joint.lower,upper:joint.upper});
      if(Number.isFinite(value))scratch.qpos[joint.q]=value;
    }
    for(const [name,value]of Object.entries(baseJointPositions)){
      const joint=api.mj_name2id(model,api.mjtObj.mjOBJ_JOINT.value,name);
      if(joint<0||![2,3].includes(model.jnt_type[joint])||!Number.isFinite(value)){limitViolations.push({name,value,type:'invalid-base-joint'});continue;}
      const lower=model.jnt_range[joint*2],upper=model.jnt_range[joint*2+1];
      if(upper>lower&&(value<lower||value>upper))limitViolations.push({name,value,lower,upper});
      scratch.qpos[model.jnt_qposadr[joint]]=value;
    }
    for(const [name,joint]of this.jointMap)if(joint.finger&&opening!==undefined){
      const side=name.split('_')[0],gap=typeof opening==='number'?opening:opening[side];if(!Number.isFinite(gap))continue;
      const value=name.includes('_dex1_')?dexOpeningToQ(gap):gap;
      if(value<joint.lower-1e-9||value>joint.upper+1e-9)limitViolations.push({name,value,lower:joint.lower,upper:joint.upper});
      scratch.qpos[joint.q]=value;
    }
    if(this.frame.tracked)api.mj_forward(model,scratch);
    const activeFrame=this.frame.tracked?this.frame.forData(scratch):this.frame,protectedBodies=new Set(),assemblyBodies=new Set();
    if(Array.isArray(objectPoses)&&this.payloadAssembly){
      const assembly=this.payloadAssembly,parent=objectPoses.find(o=>(typeof o.body==='number'?o.body:api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,o.body))===assembly.body);
      if(parent){
        const p=vector(parent.position),q=rotation(parent.quaternion);
        assemblyBodies.add(assembly.body);for(const member of assembly.members)assemblyBodies.add(member.body);
        const payload=assembly.members.map(m=>({body:m.body,position:m.position.clone().applyQuaternion(q).add(p).toArray(),quaternion:q.clone().multiply(m.quaternion).toArray()}));
        // A caller may not provide a conflicting second prediction for cargo.
        objectPoses=[...objectPoses.filter(o=>!assembly.members.some(m=>m.body===(typeof o.body==='number'?o.body:api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,o.body)))),...payload];
      }
    }
    if(!Array.isArray(objectPoses))limitViolations.push({type:'invalid-object',reason:'objectPoses must be an array'});
    else for(const object of objectPoses){
      try{
        const body=requireBody(model,typeof object.body==='number'?object.body:api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,object.body));
        const j=model.body_jntadr[body];if(model.body_jntnum[body]!==1||model.jnt_type[j]!==0)throw new Error('Object must have one free joint');
        const p=vector(object.position),q=rotation(object.quaternion);
        if(!p.toArray().every(Number.isFinite)||!q.toArray().every(Number.isFinite)||Math.abs(q.lengthSq()-1)>1e-4)throw new Error('Invalid object rigid pose');
        const world=activeFrame.toWorld(p),worldQ=basis.clone().multiply(activeFrame.rotationToWorld(q)).multiply(inverseBasis);
        scratch.qpos.set([world.x,-world.z,world.y,worldQ.w,worldQ.x,worldQ.y,worldQ.z],model.jnt_qposadr[j]);
        if(protectObjects||assemblyBodies.has(body))protectedBodies.add(body);
      }catch(error){limitViolations.push({type:'invalid-object',body:object?.body,reason:error.message});}
    }
    api.mj_forward(model,scratch);
    const ignored=new Set(ignoreObjectBodies.map(body=>typeof body==='number'?body:api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,body))),contacts=[],list=scratch.contact;
    try{for(let i=0;i<Math.min(scratch.ncon,list.size());i++){
      const c=list.get(i);try{
        const a=model.geom_bodyid[c.geom1],b=model.geom_bodyid[c.geom2];if(!this.robotBodyIds.has(a)&&!this.robotBodyIds.has(b)&&!protectedBodies.has(a)&&!protectedBodies.has(b))continue;
        const fixed=this.allowedBodyPairs.has([a,b].sort((x,y)=>x-y).join(':'))||this.allowedGeomPairs.has([c.geom1,c.geom2].sort((x,y)=>x-y).join(':'));
        const footFloor=(this.floorBodyIds.has(a)&&this.floorGeomIds.has(c.geom2))||(this.floorBodyIds.has(b)&&this.floorGeomIds.has(c.geom1));
        const fingerObject=(this.fingerBodyIds.has(a)&&ignored.has(b))||(this.fingerBodyIds.has(b)&&ignored.has(a));
        const assemblyInternal=assemblyBodies.has(a)&&assemblyBodies.has(b)&&c.dist>=-.001;
        const carrierSupport=this.payloadAssembly&&assemblyBodies.size&&((a===this.payloadAssembly.body&&this.payloadAssembly.supports.has(b))||(b===this.payloadAssembly.body&&this.payloadAssembly.supports.has(a)))&&c.dist>=-.001&&Math.abs(c.frame[2])>.9;
        if(fixed||footFloor||fingerObject||assemblyInternal||carrierSupport)continue;
        const worldPosition=new THREE.Vector3(c.pos[0],c.pos[2],-c.pos[1]);
        contacts.push({dist:c.dist,body1:api.mj_id2name(model,api.mjtObj.mjOBJ_BODY.value,a),body2:api.mj_id2name(model,api.mjtObj.mjOBJ_BODY.value,b),body1Id:a,body2Id:b,geom1Id:c.geom1,geom2Id:c.geom2,position:Array.from(c.pos),worldPosition:worldPosition.toArray(),localPosition:activeFrame.toLocal(worldPosition).toArray(),kind:this.robotBodyIds.has(a)&&this.robotBodyIds.has(b)?'robot-self':protectedBodies.has(a)||protectedBodies.has(b)?'carried-object-environment':'robot-environment'});
      }finally{c.delete();}
    }}finally{list.delete();}
    return {safe:limitViolations.length===0&&contacts.every(c=>c.dist>=0),contacts,limitViolations,minDistance:contacts.length?Math.min(...contacts.map(c=>c.dist)):null,positionFrame:'mujoco-z-up',worldPositionFrame:'shared-three-y-up',mountPose:this.frame.tracked?{position:activeFrame.position.toArray(),quaternion:activeFrame.quaternion.toArray()}:null};
  }
  dispose(){this.scratch?.delete();this.scratch=null;}
}

function localTarget(target,frame){
  if(!target?.id||!target.body||!target.drop)throw new Error('A task needs a shared body ID/name and a world drop pose');
  return {...target,start:frame.toLocal(target.start??target.drop),drop:frame.toLocal(target.drop),rotation:frame.rotationToLocal(target.rotation??new THREE.Quaternion())};
}

/** Reuses native IK/controllers against a single shared model and live data. */
export function createRobotAgent({api,model,data,robot,id,metadata={},targets=[],Controller,Dynamics,deferRobotSync=true}={}){
  const config=ROBOT_CONFIGS[id];if(!config)throw new Error(`Unknown robot: ${id}`);
  // model.mjs metadata.origin is explicitly MuJoCo Z-up. An explicit frame
  // or position is Three Y-up. Never silently treat the two as interchangeable.
  const frameSpec=metadata.frame??(metadata.origin?{position:[metadata.origin[0],metadata.origin[2],-metadata.origin[1]],yaw:metadata.yaw??0}:metadata);
  const prefix=metadata.prefix??`${id}__`,mobile=Boolean(metadata.mobile||metadata.mobility),mountName=metadata.mountName??metadata.mobility?.mountName??`${prefix}mount`;
  const mountBody=mobile?api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,mountName):-1;
  if(mobile)api.mj_forward(model,data);
  const frame=mobile?createTrackedFrame({model,data,mountBody}):createRigidFrame(frameSpec),namespace=createNamespacedAPI(api,{model,liveData:data,prefix,aliases:{task_table:'shared_workbench',floor:'cell_floor',...metadata.aliases}});
  let mobilityLocked=false,mobilityReason='底盘移动时不能规划或执行机械臂';
  const assertStationary=()=>{if(mobilityLocked)throw Object.assign(new Error(mobilityReason),{detail:{type:'mobility-interlock'}});};
  const localTargets=targets.map(target=>localTarget(target,frame));
  for(const target of localTargets)if(typeof target.body==='string')namespace.setAlias(target.body,target.body);
  const Base=Controller??(id==='c5'?CollaborationC5Controller:config.Controller),Dynamic=Dynamics??config.Dynamics;
  let controller,dynamics,checker,postureMotion=null;
  const normalizeOpening=value=>id==='r1'&&typeof value==='number'?{left:controller?.arm==='left'?value:.05,right:controller?.arm==='right'?value:.05}:value;
  class SharedController extends Base{
    fail(message){
      if(!metadata.binFlow)return super.fail(message);
      this.running=false;this.state='failed';this.message=message; // Keep actual grip command: never drop a carried bin on failure.
    }
    stop(message='已暂停，保持夹持与当前关节目标'){
      if(!metadata.binFlow)return super.stop(message);
      this.running=false;this.state='stopped';this.message=message;
    }
    transition(state){
      // In the independent bin cell, a completed pick/place returns to the
      // work-ready pose, not the standing/rest pose of the standalone demo.
      if(id==='g1'&&metadata.operationReturn==='ready'&&state==='return_fold')state='operation_ready';
      if(['r1','c5'].includes(id)&&metadata.operationReturn==='ready'&&state==='return')state='operation_return';
      return super.transition(state);
    }
    updateMotion(){
      if(['r1','c5'].includes(id)&&metadata.operationReturn==='ready'&&this.state==='operation_return'){
        const u=THREE.MathUtils.clamp(this.elapsed/1.8,0,1),t=u*u*(3-2*u),goal=this.planData.readyPose;
        this.applyPose(Object.fromEntries(Object.entries(goal).map(([name,value])=>[name,THREE.MathUtils.lerp(this.from[name]??value,value,t)])));return;
      }
      return super.updateMotion();
    }
    step(dt=.001){
      super.step(dt);
      if(['r1','c5'].includes(id)&&metadata.operationReturn==='ready'&&this.running&&this.state==='operation_return'&&this.tick%10===0){
        if(this.elapsed>=1.8&&this.jointError()<.03)this.transition('operation_ready');
        else if(this.elapsed>4)this.fail('尚未稳定返回准备位，保持当前关节目标');
      }
      if(metadata.operationReturn==='ready'&&this.running&&this.state==='operation_ready'&&this.tick%10===0){
        const m=this.measure(),settled=this.jointError()<.03&&m.support&&m.speed<.025&&m.distance<.02;
        this.evidence.stable=settled?this.evidence.stable+.01:0;
        if(this.evidence.stable>=.25){this.running=false;this.state='complete';this.sharedAtReady=true;this.message=`${id==='c5'?'零件':'料盒'}已放稳，已返回准备位`;}
        else if(this.elapsed>4){this.running=false;this.state='failed';this.message='准备位或料盒支撑尚未稳定，保持当前关节目标';}
      }
    }
    phaseTarget(state){
      if(id==='c5'&&metadata.operationReturn==='ready'&&state==='operation_return')return this.planData.readyPose;
      const pose=super.phaseTarget?.(state);
      if(id==='g1'&&Number.isFinite(metadata.readyShoulderPitch)&&['ready','recenter','return_align'].includes(state)&&pose){
        return {...pose,left_shoulder_pitch_joint:metadata.readyShoulderPitch,right_shoulder_pitch_joint:metadata.readyShoulderPitch};
      }
      if(id==='g1'&&Number.isFinite(metadata.entryShoulderRoll)&&['spread','raise','return_fold','return_wide'].includes(state)&&pose){
        return {...pose,left_shoulder_roll_joint:metadata.entryShoulderRoll,right_shoulder_roll_joint:-metadata.entryShoulderRoll};
      }
      return pose;
    }
    reset(){if(!namespace.initializing)throw new Error('Reset the shared scene, not an individual robot');return super.reset();}
    setObject(){if(!namespace.initializing)throw new Error('Shared task objects move only through contact and actuators');return false;}
    objectPose(selected=this.selected){
      const body=requireBody(model,this.bodies.get(selected));
      return {position:frame.toLocal(readWorldPosition(data,body)),quaternion:frame.rotationToLocal(readWorldRotation(data,body))};
    }
    placementEndpoints(target){return {start:this.objectPose(target.id).position,drop:target.drop.clone()};}
    start(...args){assertStationary();return super.start(...args);}
    prepareItem(...args){
      assertStationary();this._sharedEntryChecked=null;
      if(metadata.operationReturn==='ready'&&this.sharedAtReady)args[2]=true;
      if(id==='g1'){
        this.arm=args[0];this.selected=args[1];
        const phases=args[2]?['recenter']:['spread','raise','ready'];
        try{
          const opening={left:dexQToOpening(DEX_MAX),right:dexQToOpening(DEX_MAX)};
          const checked=validatePresetRoute(this,phases,{opening});
          this._sharedEntryChecked={phases:checked,terminal:this.phaseTarget(phases.at(-1))};
        }catch(error){
          this.running=false;this.state='failed';this.message=error.message;this.planError=error.detail??null;
          this.planningResult={ok:false,reason:error.message,error:{type:'collision',detail:error.detail??null}};return false;
        }
      }
      return super.prepareItem(...args);
    }
    plan(...args){
      assertStationary();
      const result=id==='g1'?planG1Task.call(this,...args):id==='c5'?planC5Task.call(this,...args):super.plan(...args),plan=this.planData;
      if(id==='g1'){
        const continuing=Boolean(this._planningJob?.continuing),phases=this._sharedEntryChecked?['align','approach']:continuing?['recenter','align','approach']:['spread','raise','ready','align','approach'];
        plan.entryChecks=[...(this._sharedEntryChecked?.phases??[]),...validatePresetRoute(this,phases,{from:this._sharedEntryChecked?.terminal??this.pose,opening:{left:dexQToOpening(DEX_MAX),right:dexQToOpening(DEX_MAX)}})];
        const returning=metadata.operationReturn==='ready'?['return_clear','return_align']:['return_clear','return_align','return_fold','return_wide','return','return_center'];
        plan.returnChecks=validatePresetRoute(this,returning,{from:plan.retreat.at(-1),opening:{...this.openings,[this.arm]:plan.preOpening},objectPoseFor:pose=>this.predictedObject(pose,plan,'retreat')});
        return result;
      }
      if(id==='c5'){
        plan.entryChecks=validatePresetRoute(this,['approach'],{opening:this.opening});
        plan.returnChecks=validatePresetRoute(this,[metadata.operationReturn==='ready'?'operation_return':'return'],{from:plan.retreat.at(-1),opening:plan.preOpening,objectPoseFor:pose=>this.predictedObject(pose,plan,'retreat')});
        return result;
      }
      // Native R1 plans were kinematic-only. Here every swept path is checked
      // in the complete shared scene, including the other two robot bodies.
      for(const phase of ['descend','lift','transfer','place','retreat']){
        const path=plan[phase];for(let i=0;i<path.length;i++)this.validateSegment(i?path[i-1]:path[i],path[i],{opening:.05,objectPoseFor:phase==='descend'?null:pose=>this.predictedObject(pose,plan,phase)});
      }
      plan.entryChecks=validatePresetRoute(this,[
        {phase:'spread',pose:{...R1_TORSO,...R1_SPREAD.right,...R1_SPREAD.left}},
        {phase:'crouch',pose:{...this.crouch,torso_joint4:0,...R1_SPREAD.right,...R1_SPREAD.left}},
        {phase:'align',pose:{...this.crouch,...R1_SPREAD.right,...R1_SPREAD.left}},
        {phase:'approach',pose:plan.approach},
      ],{opening:.05});
      plan.readyPose={...this.crouch,...R1_SPREAD.right,...R1_SPREAD.left};
      plan.returnChecks=validatePresetRoute(this,[{phase:metadata.operationReturn==='ready'?'operation_return':'return',pose:metadata.operationReturn==='ready'?plan.readyPose:{...R1_TORSO,...R1_REST.right,...R1_REST.left}}],{from:plan.retreat.at(-1),opening:.05,objectPoseFor:pose=>this.predictedObject(pose,plan,'retreat')});
      return result;
    }
    validateSegment(from,to,{opening=this.openings??this.opening,openingFrom=null,ignoreSelected=true,objectPoseFor=null}={}){
      if(id!=='r1')return super.validateSegment(from,to,{opening,openingFrom,ignoreSelected,objectPoseFor});
      const names=[...dynamics.map].filter(([,joint])=>!joint.finger).map(([name])=>name),count=Math.max(2,Math.ceil(Math.max(...names.map(name=>Math.abs((to[name]??from[name]??0)-(from[name]??to[name]??0))))/.035));
      for(let i=0;i<=count;i++){
        const pose=Object.fromEntries(names.map(name=>[name,THREE.MathUtils.lerp(from[name]??to[name]??0,to[name]??from[name]??0,i/count)]));
        const check=checker.check(pose,{opening:normalizeOpening(openingFrom===null?opening:THREE.MathUtils.lerp(openingFrom,opening,i/count)),ignoreObjectBodies:ignoreSelected?[this.bodies.get(this.selected)]:[],objectPoses:objectPoseFor?.(pose)});
        if(!check.safe){const error=new Error('Shared-scene R1 path collision');error.detail={sample:i,count,contacts:check.contacts.slice(0,3),limits:check.limitViolations};throw error;}
      }return true;
    }
    predictedObject(pose,plan,phase){
      if(id!=='r1'&&!plan.graspTransform)return super.predictedObject(pose,plan,phase);
      this.robot.setJointValues(pose);this.robot.updateMatrixWorld(true);const tcp=config.tcp(this.robot,this.arm);
      if(plan.graspTransform){
        const released=phase==='retreat',g=plan.graspTransform;
        return [{body:this.bodies.get(this.selected),position:(released?plan.drop:tcp.position.clone().add(g.position.clone().applyQuaternion(tcp.quaternion))).toArray(),quaternion:(released?plan.releaseObjectQuaternion:tcp.quaternion.clone().multiply(g.quaternion)).toArray()}];
      }
      const initial=this.objectPose(this.selected).quaternion,orientation=plan.orientation.clone().invert().multiply(initial);
      // Released objects stay on their support while the empty wrist retreats.
      // Rotating them with the wrist invents a tabletop penetration in scratch
      // collision checking, rejecting otherwise safe return-to-ready paths.
      return [{body:this.bodies.get(this.selected),position:(phase==='retreat'?plan.drop:tcp.position).toArray(),quaternion:(phase==='retreat'?initial:tcp.quaternion.multiply(orientation)).toArray()}];
    }
    measure(){return measure();}
  }
  namespace.withInitialization(()=>{
    dynamics=new Dynamic(namespace.api,model,data,robot);
    controller=new SharedController(namespace.api,model,data,robot,localTargets,dynamics);
  });
  if(localTargets.length)controller.selected=localTargets[0].id;
  const ownBodies=new Set(Object.keys(robot.links??{}).map(name=>namespace.api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,name)).filter(body=>body>=0));
  const fingers=Object.values(controller.fingers).flat().map(f=>f.body),floorGeom=api.mj_name2id(model,api.mjtObj.mjOBJ_GEOM.value,metadata.floorGeom??'cell_floor');
  // R1's MuJoCo fingertip boxes are called *_pad1/2, unlike their URDF
  // *_gripper_finger_link1/2 names. They still belong to this robot.
  for(const body of fingers)if(body>=0)ownBodies.add(body);
  if(mobile)ownBodies.add(mountBody);
  const feet=['left_ankle_roll_link','right_ankle_roll_link'].map(name=>namespace.api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,name)).filter(body=>body>=0);
  const pairs=(metadata.allowedBodyPairs??[]).map(pair=>pair.map(body=>typeof body==='number'?body:namespace.api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,body)));
  const geomPairs=(metadata.allowedGeomPairs??[]).map(pair=>pair.map(geom=>typeof geom==='number'?geom:api.mj_name2id(model,api.mjtObj.mjOBJ_GEOM.value,geom)));
  if(id==='c5'){const body=namespace.api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,'base_link'),plinth=namespace.api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,'robot_plinth');if(body>=0&&plinth>=0)pairs.push([body,plinth]);}
  if(id==='c5'){const base=namespace.api.mj_name2id(model,api.mjtObj.mjOBJ_GEOM.value,'base_link_collision1'),pedestal=api.mj_name2id(model,api.mjtObj.mjOBJ_GEOM.value,'c5_grounded_pedestal');if(base>=0&&pedestal>=0)geomPairs.push([base,pedestal]);}
  if(mobile){
    const proxy=api.mj_name2id(model,api.mjtObj.mjOBJ_GEOM.value,`${prefix}chassis_proxy`),platform=api.mj_name2id(model,api.mjtObj.mjOBJ_GEOM.value,'r1_mobile_platform_top');
    if(proxy>=0){ownBodies.add(model.geom_bodyid[proxy]);if(platform>=0)geomPairs.push([proxy,platform]);}
  }
  checker=new SharedCollisionChecker({api,model,data,jointMap:dynamics.map,robotBodyIds:ownBodies,fingerBodyIds:fingers,frame,allowedBodyPairs:pairs,allowedGeomPairs:geomPairs,floorGeomIds:floorGeom>=0?[floorGeom]:[],floorBodyIds:feet});
  // Dispose the legacy mirror: all three use the stricter shared-model one.
  dynamics.scratch?.delete();dynamics.scratch=null;
  dynamics.checkPose=(pose,options={})=>checker.check(pose,{...options,opening:normalizeOpening(options.opening)});
  // Only FK/display work is deferred. Original motor/force control remains
  // 1 kHz, and the owner calls afterWorldStep at the native 100 Hz boundary.
  if(deferRobotSync)dynamics.afterStep=()=>{};
  const resetDynamics=dynamics.reset.bind(dynamics);dynamics.reset=(...args)=>{if(!namespace.initializing)throw new Error('Robot actual state is immutable outside shared reset initialization');return resetDynamics(...args);};
  function measure(){
    const body=controller?.bodies.get(controller.selected),target=controller?.targets.find(t=>t.id===controller.selected);
    if(!target||!Number.isInteger(body))return {count:0,force:0,forces:[0,0],table:false,support:false,position:new THREE.Vector3(),worldPosition:new THREE.Vector3(),speed:0,distance:Infinity,contacts:[]};
    const fingerIds=controller.fingers[controller.arm].map(f=>f.body),hits=new Set(),forces=[0,0],contacts=[];
    const support=namespace.api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,'task_table'),list=data.contact;let supported=false;
    try{for(let i=0;i<Math.min(data.ncon,list.size());i++){
      const c=list.get(i);try{const a=model.geom_bodyid[c.geom1],b=model.geom_bodyid[c.geom2];if(a!==body&&b!==body)continue;
        const other=a===body?b:a,index=fingerIds.indexOf(other),force=c.efc_address>=0?Math.abs(data.efc_force[c.efc_address]):0;
        if(index>=0&&force>.015){hits.add(index);forces[index]+=force;}if(other===support&&force>.005)supported=true;
        const worldPosition=new THREE.Vector3(c.pos[0],c.pos[2],-c.pos[1]);contacts.push({position:frame.toLocal(worldPosition),worldPosition,force,other});
      }finally{c.delete();}
    }}finally{list.delete();}
    const worldPosition=readWorldPosition(data,body),position=frame.toLocal(worldPosition),joint=model.body_jntadr[body],dof=model.jnt_dofadr[joint];
    return {count:hits.size,force:forces.reduce((sum,value)=>sum+value,0),forces,table:supported,support:supported,position,worldPosition,speed:Math.hypot(...data.qvel.subarray(dof,dof+3)),distance:position.distanceTo(controller.planData?.drop??target.drop),contacts};
  }
  const syncFrame=()=>frame.tracked?frame.sync():{position:frame.position.toArray(),quaternion:frame.quaternion.toArray(),version:0,mountBody:null,tracked:false};
  const mountJointIds=mobile?Array.from({length:model.body_jntnum[mountBody]},(_,index)=>model.body_jntadr[mountBody]+index):[];
  const mountJointNames=new Set(mountJointIds.map(j=>api.mj_id2name(model,api.mjtObj.mjOBJ_JOINT.value,j)));
  return {id,controller,dynamics,frame,namespace,toLocal:frame.toLocal,toWorld:frame.toWorld,syncFrame,
    get postureMotion(){return postureMotion;},
    planPosture(kind){
      assertStationary();if(!['ready','home'].includes(kind))throw new Error('Unknown station posture');
      const home={...R1_TORSO,...R1_REST.right,...R1_REST.left};let poses;
      try{
        if(id==='g1')poses=(kind==='ready'?['spread','raise','ready']:['return_fold','return_wide','return','return_center']).map(phase=>controller.phaseTarget(phase));
        else if(id==='r1')poses=kind==='ready'?[{...R1_TORSO,...R1_SPREAD.right,...R1_SPREAD.left},{...R1_CROUCH,...R1_SPREAD.right,...R1_SPREAD.left}]:[{...R1_TORSO,...R1_SPREAD.right,...R1_SPREAD.left},home];
        else if(kind==='home')poses=[{...C5_HOME}];
        else{
          const result=solveC5Posture(robot,'left',new THREE.Vector3(.38,1.05,0),c5TopDown(0),C5_HOME,350);
          if(result.positionError>.0025||result.angleError>.025)throw new Error('C5 ready posture is unreachable');poses=[result.pose];
        }
        let from=controller.pose;
        const route=poses.map(pose=>{
          controller.validateSegment(from,pose,{ignoreSelected:false});
          const duration=Math.max(1.4,...Object.keys(pose).map(name=>Math.abs(pose[name]-(from[name]??pose[name]))/.8));from=pose;return {pose:{...pose},duration};
        });return {kind,route};
      }finally{dynamics.syncRobot();}
    },
    beginPosture(plan){
      assertStationary();if(controller.running||postureMotion)throw new Error('Robot already has active motion');
      if(!['ready','home'].includes(plan?.kind)||!plan.route?.length||plan.route.some(s=>!Number.isFinite(s.duration)||s.duration<=0||!Object.values(s.pose).every(Number.isFinite)))throw new Error('Invalid checked posture route');
      postureMotion={...structuredClone(plan),index:0,elapsed:0,stable:0,from:{...controller.pose},paused:false};
      controller.manualMotion=null;controller.state=plan.kind==='ready'?'readying':'homing';controller.sharedAtReady=false;
    },
    pausePosture(paused=true){if(postureMotion)postureMotion.paused=Boolean(paused);},
    validateResume({state,elapsed,posture,pickOnly=false}={}){
      assertStationary();
      const actual=Object.fromEntries([...dynamics.map].filter(([,j])=>!j.finger).map(([name,j])=>[name,data.qpos[j.q]]));
      const savedPose=controller.pose;let samples=0;
      try{
        const route=posture?posture.route.slice(posture.index).map(s=>({phase:'posture',pose:s.pose,objectPhase:null})):
          remainingMotionSegments({robotId:id,state,elapsed,plan:controller.planData,pose:savedPose,placePose:controller.placePose,pickOnly,
            phaseTarget:(phase,from)=>{
              controller.pose=from;
              if(id!=='r1')return controller.phaseTarget(phase);
              if(phase==='spread')return {...R1_TORSO,...R1_SPREAD.right,...R1_SPREAD.left};
              if(phase==='crouch')return {...controller.crouch,torso_joint4:0,...R1_SPREAD.right,...R1_SPREAD.left};
              if(phase==='align_clear')return {...controller.crouch,torso_joint4:controller.planData.alignFromYaw,...R1_SPREAD.right,...R1_SPREAD.left};
              if(phase==='align')return {...controller.crouch,...R1_SPREAD.right,...R1_SPREAD.left};
              return phase==='approach'?controller.planData.approach:phase==='operation_return'?controller.planData.readyPose:null;
            }});
        // Include actual motor tracking drift → saved command before the tail.
        const held=['lift','transfer','place'].includes(state),released=['retreat','settling','return_clear','return_align','operation_return','operation_ready'].includes(state);
        route.unshift({phase:'actual-to-command',pose:savedPose,objectPhase:held?state:released?'retreat':null});
        let from=actual;
        for(const segment of route){
          controller.validateSegment(from,segment.pose,{opening:posture?controller.openings??controller.opening:id==='r1'?.05:controller.planData.preOpening,ignoreSelected:false,
            objectPoseFor:segment.objectPhase?pose=>controller.predictedObject(pose,controller.planData,segment.objectPhase):null});
          from=segment.pose;samples++;
        }
        return {safe:true,samples};
      }catch(error){return {safe:false,samples,reason:error.message,detail:error.detail};}
      finally{controller.pose=savedPose;dynamics.syncRobot();}
    },
    get mobilityInterlock(){return mobilityLocked;},
    setMobilityInterlock({moving,reason}={}){
      if(typeof moving!=='boolean')throw new Error('Mobility interlock needs a boolean moving state');
      if(moving&&(controller.running||controller.planning))throw new Error('Pause the native arm controller before moving its base');
      mobilityLocked=moving;if(reason)mobilityReason=reason;return syncFrame();
    },
    configureTask({target,arm=config.defaultArm,supportBody}={}){
      assertStationary();syncFrame();if(controller.running||controller.planning)throw new Error('Cannot reassign an active shared manipulation');
      const local=localTarget(target,frame),body=typeof target.body==='number'?target.body:api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,target.body);requireBody(model,body);
      if(id==='c5'&&target.binId&&target.sourceSupport){
        const parent=requireBody(model,api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,target.sourceSupport));
        const radial=local.start.clone().sub(frame.toLocal(readWorldPosition(data,parent)));
        // Close tangentially to the carrier handle, not through its post.
        local.binGraspYaw=Math.atan2(radial.x,-radial.z);
      }
      controller.targets.splice(0,controller.targets.length,local);controller.bodies=new Map([[local.id,body]]);controller.selected=local.id;controller.arm=id==='c5'?'left':arm;
      if(supportBody)namespace.setAlias('task_table',supportBody);
      checker.setPayloadAssembly({body,payloadBodies:target.payloadBodies??[],supportBodies:[target.sourceSupport,supportBody],carrier:target.carrier===true});
      controller.planData=null;controller.planError=null;controller._sharedEntryChecked=null;return local;
    },
    replanHeldPlacement({worldDrop,supportBody}={}){
      assertStationary();syncFrame();
      if(id!=='r1'||!metadata.binFlow||controller.running)throw new Error('Loaded placement requires a stationary, held R1 bin operation');
      if(measure().count!==2)throw new Error('Loaded placement requires actual dual finger contact');
      const target=controller.targets.find(t=>t.id===controller.selected);
      if(!target?.carrier||!controller.planData)throw new Error('No carrier plan to refine');
      const previous=controller.planData,actualPose=Object.fromEntries([...dynamics.map].filter(([,j])=>!j.finger).map(([name,j])=>[name,data.qpos[j.q]]));
      const body=controller.bodies.get(controller.selected);
      checker.setPayloadAssembly({body,payloadBodies:target.payloadBodies??[],supportBodies:[target.sourceSupport,supportBody],carrier:true});
      try{
        controller.planData=buildHeldPlacementPlan({controller,actualPose,objectPose:controller.objectPose(),drop:frame.toLocal(worldDrop)});
        for(const phase of ['transfer','place','retreat']){
          const path=controller.planData[phase];
          for(let i=0;i<path.length;i++)controller.validateSegment(i?path[i-1]:phase==='transfer'?actualPose:path[0],path[i],{opening:.05,objectPoseFor:pose=>controller.predictedObject(pose,controller.planData,phase)});
        }
        if(metadata.operationReturn==='ready')controller.planData.returnChecks=validatePresetRoute(controller,[{phase:'operation_return',pose:controller.planData.readyPose}],{from:controller.planData.retreat.at(-1),opening:.05,objectPoseFor:pose=>controller.predictedObject(pose,controller.planData,'retreat')});
        return {safe:true,quality:controller.planData.loadedPlacementQuality};
      }catch(error){controller.planData=previous;return {safe:false,reason:error.message,detail:error.detail};}
      finally{dynamics.syncRobot();}
    },
    validateMobileRoute({pose=controller.pose,opening=controller.openings??controller.opening,fromMount,toMount,selectedBody=controller.bodies.get(controller.selected),objectLocalPose}={}){
      if(!mobile)throw new Error('This robot does not have an actuated mobile mount');
      const from=fromMount?.jointPositions,to=toMount?.jointPositions;
      if(!from||!to)throw new Error('Mobile route endpoints must contain explicit shared jointPositions');
      const names=[...mountJointNames];
      if(Object.keys(from).some(name=>!mountJointNames.has(name))||Object.keys(to).some(name=>!mountJointNames.has(name))||names.some(name=>!Number.isFinite(from[name])||!Number.isFinite(to[name])))throw new Error('Mobile route may only command its own finite mount joints');
      const linear=Math.hypot(...mountJointIds.filter(j=>model.jnt_type[j]===2).map(j=>{const name=api.mj_id2name(model,api.mjtObj.mjOBJ_JOINT.value,j);return to[name]-from[name];}));
      const angular=Math.max(0,...mountJointIds.filter(j=>model.jnt_type[j]===3).map(j=>{const name=api.mj_id2name(model,api.mjtObj.mjOBJ_JOINT.value,j);return Math.abs(to[name]-from[name]);}));
      let count=Math.max(1,Math.ceil(linear/.02),Math.ceil(angular/.03));
      const held=objectLocalPose?[{body:selectedBody,position:vector(objectLocalPose.position).toArray(),quaternion:rotation(objectLocalPose.quaternion).toArray()}]:[];
      const actualPose=Object.fromEntries([...dynamics.map].filter(([,joint])=>!joint.finger).map(([name,joint])=>[name,data.qpos[joint.q]]));
      const actualCheck=checker.check(actualPose,{baseJointPositions:from,objectPoses:held,ignoreObjectBodies:held.length?[selectedBody]:[],protectObjects:true});
      if(!actualCheck.safe)return {...actualCheck,samples:1,sample:0,count,stage:'mobile-actual-hold'};
      for(let refinement=0;refinement<6;refinement++){
        let previous=null,tooWide=false;
        for(let i=0;i<=count;i++){
          const baseJointPositions=Object.fromEntries(names.map(name=>[name,THREE.MathUtils.lerp(from[name],to[name],i/count)]));
          const result=checker.check(pose,{opening:normalizeOpening(opening),baseJointPositions,objectPoses:held,ignoreObjectBodies:held.length?[selectedBody]:[],protectObjects:true});
          if(!result.safe)return {...result,samples:i+1,sample:i,count,stage:'mobile-route'};
          if(previous){const displacement=vector(result.mountPose.position).distanceTo(vector(previous.position)),angle=rotation(result.mountPose.quaternion).angleTo(rotation(previous.quaternion));if(displacement>.020000001||angle>.030000001){tooWide=true;break;}}
          previous=result.mountPose;
        }
        if(!tooWide)return {safe:true,samples:count+1,count,maxLinearStep:.02,maxAngularStep:.03,stage:'mobile-route'};
        count*=2;
      }
      return {safe:false,stage:'mobile-route',reason:'Unable to sample mobile route within physical spacing bounds',count};
    },
    validateRemainingPlan({phases=['transfer','place','retreat']}={}){
      assertStationary();const plan=controller.planData;if(!plan)throw new Error('No checked manipulation plan');
      let samples=0;
      try{for(const phase of phases){const path=plan[phase];if(!Array.isArray(path)||!path.length)throw new Error(`Invalid remaining phase: ${phase}`);
        for(let i=0;i<path.length;i++){controller.validateSegment(i?path[i-1]:path[i],path[i],{opening:id==='r1'?.05:plan.preOpening,objectPoseFor:pose=>controller.predictedObject(pose,plan,phase)});samples++;}
      }return {safe:true,samples,phases,frame:syncFrame()};}catch(error){return {safe:false,samples,phases,reason:error.message,error:error.detail??null,frame:syncFrame()};}finally{dynamics.syncRobot();}
    },
    stepControls(dt=.001){
      if(mobilityLocked&&controller.running)throw new Error('Moving base cannot execute a native arm trajectory');
      if(postureMotion&&!postureMotion.paused){
        const m=postureMotion,s=m.route[m.index];m.elapsed+=dt;
        const u=THREE.MathUtils.clamp(m.elapsed/s.duration,0,1),t=u*u*(3-2*u);
        controller.applyPose(Object.fromEntries(Object.entries(s.pose).map(([name,value])=>[name,THREE.MathUtils.lerp(m.from[name]??value,value,t)])));
        m.stable=m.elapsed>=s.duration&&controller.jointError()<.03?m.stable+dt:0;
        if(m.stable>=.15){
          controller.applyPose(s.pose);m.index++;m.elapsed=0;m.stable=0;m.from={...s.pose};
          if(m.index>=m.route.length){controller.state=m.kind==='ready'?'ready':'home';controller.sharedAtReady=m.kind==='ready';postureMotion=null;}
        }else if(m.elapsed>s.duration+5){controller.fail('工位准备/收尾动作未能稳定到达');postureMotion=null;}
      }
      namespace.withControl(()=>controller.step(dt));
    },
    afterWorldStep(){dynamics.syncRobot();controller.measurement=measure();return controller.measurement;},
    measure,
    dispose(){checker.dispose();dynamics.dispose?.();}};
}
