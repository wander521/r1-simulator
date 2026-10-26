import {ROBOT_CONFIGS} from '../multi/robot-runtime.mjs';
import {createPlanningWorkerRobot} from '../multi/planning-worker-core.mjs';
import {applyPlanningResult} from '../multi/planning-serialization.mjs';
import {createRobotAgent} from './robot-adapter.mjs';
import {IDS,LABELS,KIND_LABELS,bodyPose,toThree,robotFrame,plainFields,physicalSnapshot,CONTROL_FIELDS,MOTOR_FIELDS} from './state.mjs';

const STEP=.001;
const now=()=>globalThis.performance?.now?.()??Date.now();
const finite=x=>Number.isFinite(x);
const distance=(a,b)=>Math.hypot(...a.map((v,i)=>v-b[i]));

/** Authoritative live world. This is also the exact core exercised by Node
 * regression tests; only resource loading and Worker messaging differ. */
export class CollaborationCore{
  constructor({api,xml,metadata,urdfs,plan,cancelPlan=()=>{},clock=()=>Date.now()}){
    Object.assign(this,{api,xml,metadata,urdfs,clock,plan,cancelPlan});this.agents=new Map();this.disposed=false;this.generation=0;
    const vfs=new api.MjVFS();
    try{this.model=api.MjModel.from_xml_string(xml,vfs);this.data=new api.MjData(this.model);}
    finally{vfs.delete();}
    this.enabled=Object.fromEntries(IDS.map(id=>[id,true]));this.pendingWithdrawal={};
    this.partBodies=new Map(metadata.parts.map(part=>[part.id,this.id('BODY',part.body)]));
    this.trayBodies=new Map(metadata.trays.map(tray=>[tray.orderId,this.id('BODY',tray.body)]));
    this.trayMotors=new Map(metadata.trays.map(tray=>[tray.orderId,{x:this.id('ACTUATOR',tray.actuatorX),y:this.id('ACTUATOR',tray.actuatorY)}]));
    this.mountBodies=new Map(IDS.map(id=>[id,this.id('BODY',metadata.robots[id].mountName)]));
    this.baseAxes=metadata.mobility?Object.fromEntries(['x','y','yaw'].map(axis=>{const joint=this.id('JOINT',metadata.mobility.joints[axis]);return [axis,{name:metadata.mobility.joints[axis],q:this.model.jnt_qposadr[joint],d:this.model.jnt_dofadr[joint],actuator:this.id('ACTUATOR',metadata.mobility.actuators[axis])}];})):null;
    this._buildAgents();this._resetSession();
    // Settle loose parts using exactly the same shared physics step, not edits.
    for(let i=0;i<300;i++)this.stepPhysics();this.lastWall=null;this.accumulator=0;
  }
  id(type,name){const value=this.api.mj_name2id(this.model,this.api.mjtObj[`mjOBJ_${type}`].value,name);if(value<0)throw new Error(`共享场景缺少 ${type}: ${name}`);return value;}
  _buildAgents(){
    for(const id of IDS){
      const robot=createPlanningWorkerRobot(this.urdfs[id],{rootHeight:ROBOT_CONFIGS[id].rootHeight});
      const agent=createRobotAgent({api:this.api,model:this.model,data:this.data,robot,id,metadata:robotFrame(this.metadata.robots[id],this.metadata.workbench),targets:[]});
      this.agents.set(id,agent);
    }
    this.api.mj_forward(this.model,this.data);
  }
  _resetSession(){
    this.state='idle';this.error=null;this.elapsedSeconds=0;this.durationSeconds=300;this.startedAt=null;this.lastWall=null;this.accumulator=0;
    this.completedOrders=0;this.completedParts=0;this.orderIndex=0;this.active=null;this.conveyor=null;this.navigation=null;this.events=[];this.jobNumber=0;
    this.locations=Object.fromEntries(this.metadata.parts.map(part=>[part.id,'source']));this.evidence=[];this.packAttempts={};this.lastTrayVerification=null;
    this.stats={physicsSteps:0,planningJobs:0,planningMs:0,maxStepMs:0,maxPartIntegrationResidual:0,maxPartSpeed:0,maxTorqueRatio:0,maxFingerViolation:{r1:0,g1:0,c5:0},dualContactSamples:0,sharedHandoffs:0,reassignments:0,droppedWallSeconds:0,mobileTrips:0,mobileDistance:0,mobileCarryTrips:0};
    this._log('共享工作台已就绪：三台本体、同一批零件与托盘。');
  }
  _log(message){this.events.push({time:this.elapsedSeconds,message});if(this.events.length>80)this.events.shift();}
  start({durationSeconds=300}={}){
    if(!finite(durationSeconds)||durationSeconds<1||durationSeconds>900)throw new Error('作业时长必须在 1–900 秒之间');
    if(['running','draining'].includes(this.state))return false;
    if(this.state==='failed'||this.state==='complete')throw new Error('请先复位共享工作台再开始新一轮');
    if(!IDS.some(id=>this.enabled[id]))throw new Error('至少启用一台机器人');
    if(this.state==='stopped')this._validateResume();
    this.durationSeconds=durationSeconds;this.startedAt=this.clock()-this.elapsedSeconds*1000;this.state='running';this.error=null;this.lastWall=null;
    this._log(`开始连续齐套作业，目标 ${durationSeconds} 秒。`);return true;
  }
  stop(){
    if(this.disposed||this.state==='stopped')return;
    const terminalState=['failed','complete'].includes(this.state)?this.state:null;
    this.generation++;
    if(this.active?.planning){
      this.cancelPlan();
      const c=this.agents.get(this.active.owner).controller;c.running=false;c.state='idle';this.active=null;
    }
    // Keep the actual grip and current arm target. A stop must not suddenly
    // release a suspended workpiece or overwrite real joint positions.
    if(this.active){const c=this.agents.get(this.active.owner).controller;this.active.resumeState??=c.state;c.running=false;c.state='stopped';}
    this.state=terminalState??'stopped';this.accumulator=0;this._log('作业已停止，保持当前关节目标和夹爪开口。');
    if(this.baseAxes)for(const axis of Object.values(this.baseAxes))this.data.ctrl[axis.actuator]=this.data.qpos[axis.q];
    if(this.conveyor){const motors=this.trayMotors.get(this.conveyor.tray.orderId);for(const axis of ['x','y']){const joint=this.id('JOINT',axis==='x'?this.conveyor.tray.jointX:this.conveyor.tray.jointY);this.data.ctrl[motors[axis]]=this.data.qpos[this.model.jnt_qposadr[joint]];}}
  }
  _validateResume(){
    const op=this.active;
    if(this.navigation){
      const agent=this.agents.get('r1'),c=agent.controller,nav=this.navigation;
      if(nav.carry&&agent.measure().count!==2)throw new Error('暂停后已失去双指夹持，请复位后重新作业');
      const from=Object.fromEntries(Object.values(this.baseAxes).map(axis=>[axis.name,this.data.qpos[axis.q]])),to=Object.fromEntries(Object.entries(this.baseAxes).map(([key,axis])=>[axis.name,nav.goal[key]]));
      const check=agent.validateMobileRoute({pose:c.pose,opening:c.openings??c.opening,fromMount:{jointPositions:from},toMount:{jointPositions:to},selectedBody:nav.carry?this.partBodies.get(op.part.id):undefined,objectLocalPose:nav.carry?c.objectPose(op.part.id):undefined});
      if(!check.safe)throw new Error('暂停后底盘路线已不再安全，请复位后重试');
      return;
    }
    if(!op)return;
    const agent=this.agents.get(op.owner),c=agent.controller,phase=op.resumeState,m=agent.measure();
    if(['lift','transfer','place'].includes(phase)&&m.count!==2)throw new Error('暂停后已失去双指夹持，请复位后重新作业');
    if(['spread','raise','ready','align','approach','descend','closing','crouch'].includes(phase)&&c.objectPose(op.part.id).position.distanceTo(c.planData.start)>.005)throw new Error('暂停后源物料已移动，旧抓取路径作废，请复位后重试');
    const paths=['descend','lift','transfer','place','retreat'],remaining=paths.includes(phase)?paths.slice(paths.indexOf(phase)):['spread','raise','ready','align','approach','closing','crouch'].includes(phase)?paths:[];
    if(remaining.length){const result=agent.validateRemainingPlan({phases:remaining});if(!result.safe)throw new Error('暂停后剩余机械臂路径未通过共享检查，请复位后重试');}
  }
  reset(){
    this.generation++;this.cancelPlan();for(const agent of this.agents.values())agent.dispose();this.agents.clear();
    this.api.mj_resetData(this.model,this.data);this._buildAgents();this._resetSession();
    for(let i=0;i<300;i++)this.stepPhysics();this.lastWall=null;this.accumulator=0;
  }
  setRobotEnabled(id,enabled){
    if(!IDS.includes(id))throw new Error('未知机器人');
    if(!enabled&&!IDS.some(other=>other!==id&&this.enabled[other]))throw new Error('至少保留一台可用机器人');
    this.enabled[id]=Boolean(enabled);
    if(enabled)for(const part of Object.keys(this.packAttempts))this.packAttempts[part]=this.packAttempts[part].filter(owner=>owner!==id);
    this._log(`${LABELS[id]} ${enabled?'重新加入协作':'退出后续任务分配'}${this.active?.owner===id&&!this.active.planning?'，先安全完成当前操作':''}。`);
    if(!enabled&&this.active?.owner===id&&this.active.planning){this.generation++;this.cancelPlan();this.active=null;}
    if(enabled&&this.state==='waiting'){this.state='running';this.error=null;}
    return true;
  }
  partTarget(part,drop){return {id:part.id,body:part.body,label:KIND_LABELS[part.kind],shape:part.shape,h:part.h,halfX:part.halfX,halfZ:part.halfZ,start:bodyPose(this.data,this.partBodies.get(part.id)).position,drop};}
  _feedDock(orderId){return this.metadata.mobility?.feedByOrder?.find(dock=>dock.orderId===orderId)??this.metadata.mobility?.docks.feed;}
  _atDock(dock){
    if(!this.baseAxes||!dock)return true;
    const p=bodyPose(this.data,this.mountBodies.get('r1')).position,t=toThree(dock.position),tolerance=this.metadata.mobility.dockTolerance;
    return distance(p,t)<tolerance.position&&Math.abs(this.data.qpos[this.baseAxes.yaw.q]-(dock.yaw-this.metadata.robots.r1.yaw))<tolerance.yaw&&Object.values(this.baseAxes).every(axis=>Math.abs(this.data.qvel[axis.d])<tolerance.speed);
  }
  _baseJointGoal(dock){
    // The model's planar slide axes are world-aligned; the mount quaternion
    // only rotates the native robot below those joints. No live qpos writes.
    const origin=this.metadata.robots.r1.origin;
    return {x:dock.position[0]-origin[0],y:dock.position[1]-origin[1],yaw:dock.yaw-this.metadata.robots.r1.yaw};
  }
  _beginNavigation(dock,{carry=false,label='R1 前往供料位'}={}){
    if(!this.baseAxes||!dock)throw new Error('缺少 R1 物理底盘路线');
    const agent=this.agents.get('r1'),goal=this._baseJointGoal(dock),from=Object.fromEntries(Object.values(this.baseAxes).map(axis=>[axis.name,this.data.qpos[axis.q]])),to=Object.fromEntries(Object.entries(this.baseAxes).map(([key,axis])=>[axis.name,goal[key]]));
    const c=agent.controller,selectedBody=carry?this.partBodies.get(this.active.part.id):undefined,objectLocalPose=carry?c.objectPose(this.active.part.id):undefined;
    const check=agent.validateMobileRoute({pose:c.pose,opening:c.openings??c.opening,fromMount:{jointPositions:from},toMount:{jointPositions:to},selectedBody,objectLocalPose});
    if(!check.safe){const error=new Error('R1 底盘路线存在碰撞，已保持夹爪并停止移动');error.detail=check;throw error;}
    if(carry){this.active.navigationResume={state:c.state,elapsed:c.elapsed,from:c.from?{...c.from}:null};c.running=false;c.state='transport_hold';}
    else if(c.running)throw new Error('机械臂执行中不能空载移动底盘');
    agent.setMobilityInterlock({moving:true,reason:label});
    const start=bodyPose(this.data,this.mountBodies.get('r1')).position;
    this.navigation={dock,goal,carry,label,settled:0,lostGrip:0,start,elapsed:0,timeout:distance(start,toThree(dock.position))/this.metadata.mobility.maxSpeed*3+5};this._log(label+'，底盘低速移动，机械臂保持当前位姿。');
  }
  _updateNavigation(){
    if(!this.navigation||!['running','draining'].includes(this.state))return;
    const nav=this.navigation,speed=this.metadata.mobility.maxSpeed;
    nav.elapsed=(nav.elapsed??0)+STEP;if(nav.elapsed>nav.timeout)throw new Error('R1 未在限定时间内停靠，已保持夹爪并停止底盘');
    for(const [key,axis]of Object.entries(this.baseAxes)){
      const limit=(key==='yaw'?.15:speed)*STEP;
      this.data.ctrl[axis.actuator]+=Math.max(-limit,Math.min(limit,nav.goal[key]-this.data.ctrl[axis.actuator]));
    }
    nav.settled=this._atDock(nav.dock)?nav.settled+STEP:0;
    if(nav.carry&&this.stats.physicsSteps%10===0){const m=this.agents.get('r1').measure();nav.lostGrip=m.count===2?0:nav.lostGrip+.01;if(nav.lostGrip>.15)throw new Error('R1 移动供料时双指接触丢失，已停止底盘');}
    if(nav.settled<.4)return;
    const agent=this.agents.get('r1');agent.setMobilityInterlock({moving:false});agent.syncFrame();
    this.stats.mobileTrips++;this.stats.mobileDistance+=distance(nav.start,bodyPose(this.data,this.mountBodies.get('r1')).position);if(nav.carry)this.stats.mobileCarryTrips++;
    this.navigation=null;this._log('R1 已停稳并锁定底盘。');
    if(nav.carry){
      const result=agent.validateRemainingPlan({phases:['transfer','place','retreat']});
      if(result===false||result?.safe===false)throw new Error('停靠后放料路径未通过共享碰撞检查');
      const c=agent.controller,resume=this.active.navigationResume;Object.assign(c,{state:resume.state,elapsed:resume.elapsed,running:true});if(resume.from)c.from=resume.from;delete this.active.navigationResume;this.active.transported=true;
    }
  }
  _nextOperation(){
    const tray=this.metadata.trays[this.orderIndex];if(!tray)return null;
    const parts=this.metadata.parts.filter(part=>part.orderId===tray.orderId),valve=parts.find(part=>part.kind==='valve');
    const next=parts.find(part=>this.locations[part.id]==='handoff')??(this.locations[valve.id]!=='packed'?valve:parts.find(part=>this.locations[part.id]!=='packed'));
    if(!next)return {type:'dispatch',tray};
    const phase=next.kind==='valve'?'load-valve':this.locations[next.id]==='handoff'?'pack':'supply';
    const packOwners=['c5','g1'].filter(id=>this.enabled[id]&&!this.packAttempts[next.id]?.includes(id));
    const station=phase==='load-valve'?'g1':phase==='supply'?'r1':packOwners[0]??null,dock=station?tray.docks?.[station]:null;
    if(dock&&distance(bodyPose(this.data,this.trayBodies.get(tray.orderId)).position,toThree(dock))>.006)return {type:'conveyor',tray,stage:station};
    const feed=this._feedDock(next.orderId);
    if(phase==='supply'&&this.enabled.r1&&feed&&!this._atDock(feed))return {type:'navigate',dock:feed};
    const slot=tray.slots.find(slot=>slot.kind===next.kind&&slot.index===next.index),trayPosition=bodyPose(this.data,this.trayBodies.get(tray.orderId)).position;
    const drop=phase==='supply'&&tray.staging?[trayPosition[0]+tray.staging.offset[0],trayPosition[1]+tray.staging.offset[2],trayPosition[2]-tray.staging.offset[1]]
      :phase==='supply'?toThree([this.metadata.handoff.position[0],this.metadata.handoff.position[1],this.metadata.tableHeight+next.h])
      :[trayPosition[0]+slot.offset[0],trayPosition[1]+slot.offset[2],trayPosition[2]-slot.offset[1]];
    return {type:'manipulate',part:next,phase,drop,tray,station,supportBody:phase==='supply'&&!tray.staging?(this.metadata.handoff.body??this.metadata.workbench.body):tray.body,
      candidates:phase==='supply'?['r1','g1','c5']:phase==='pack'?(station?[station]:[]):['g1','c5','r1'],preferredOwner:phase==='pack'?'c5':phase==='supply'?'r1':'g1',tried:[]};
  }
  _eligible(operation,id){
    if(!this.enabled[id]||operation.tried.includes(id))return false;
    const agent=this.agents.get(id),p=agent.toLocal(bodyPose(this.data,this.partBodies.get(operation.part.id)).position),q=agent.toLocal(operation.drop);
    // Cheap established workspace rejects only; full FK and shared collision
    // checks still run in the private planner before anything can execute.
    if(id==='g1'&&[p,q].some(v=>v.x>.65||Math.hypot(v.x,v.z)>.66))return false;
    if(id==='c5'&&[p,q].some(v=>v.x<.25||v.x>.68||Math.hypot(v.x,v.z)>.72))return false;
    return true;
  }
  async _prepare(operation,retryOwner=null,retryArm=null){
    const owner=retryOwner??operation.candidates.find(id=>this._eligible(operation,id));
    if(!owner){this.state='waiting';this.error='当前零件没有可用且可达的执行机器人；请恢复机器人，或停止后复位。';this._log(this.error);this.active=null;return;}
    const generation=this.generation;
    try{
    operation.owner=owner;operation.planning=true;operation.tried.push(owner);this.active=operation;
    const agent=this.agents.get(owner),source=agent.toLocal(bodyPose(this.data,this.partBodies.get(operation.part.id)).position),destination=agent.toLocal(operation.drop);
    const arm=owner==='c5'?'left':retryArm??((source.z+destination.z)>=0?'right':'left');
    operation.armAttempts??={};operation.armAttempts[owner]??=[];operation.armAttempts[owner].push(arm);
    // Supply is one actual grip: plan native joint motion in the feed frame,
    // pause after lift, drive the physical base holding the same free body,
    // then execute the canonical release motion at the handoff dock.
    let plannedDrop=operation.drop;
    if(owner==='r1'&&operation.phase==='supply'&&this.metadata.mobility){
      const feed=this._feedDock(operation.part.orderId),dock=this.metadata.mobility.docks.handoff;
      if(!this._atDock(feed))throw new Error('R1 未停稳在供料位，拒绝伸臂');
      plannedDrop=operation.drop.map((value,i)=>value-toThree(dock.position)[i]+toThree(feed.position)[i]);operation.mobileSupply=true;
    }
    const target=this.partTarget(operation.part,plannedDrop);
    agent.configureTask({target,arm,supportBody:operation.supportBody});
    const c=agent.controller;c.task='putaway';c.reaction=null;c.reducedMotion=true;
    const jobId=++this.jobNumber;
    const job={jobId,robotId:owner,target,arm,supportBody:operation.supportBody,physics:physicalSnapshot(this.data),
      robots:Object.fromEntries([...this.agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}]))};
    this.stats.planningJobs++;const started=now();this._log(`${LABELS[owner]} 检查${KIND_LABELS[operation.part.kind]}${operation.phase==='supply'?'供料交接':'装盘'}路径。`);
      const result=await this.plan(job);
      if(generation!==this.generation||this.active!==operation)return;
      this.stats.planningMs+=now()-started;
      if(!result.ok){
        this._log(`${LABELS[owner]}${arm==='right'?'右':'左'}臂未找到安全路径，检查其他可达执行方式。`);this.active=null;
        if(owner!=='c5'&&operation.armAttempts[owner].length===1&&this.enabled[owner])await this._prepare(operation,owner,arm==='right'?'left':'right');
        else if(operation.phase==='pack'){
          // A different body cannot reach across the separated stations. Move
          // this same tray (including its staged part) to the alternate dock
          // before computing that robot's route on a fresh physical snapshot.
          this.packAttempts[operation.part.id]??=[];this.packAttempts[operation.part.id].push(owner);
        }else await this._prepare(operation);return;
      }
      // Physics is deliberately held while this shared-region snapshot is
      // planned. Neither another body nor a belt may invalidate the route.
      if(this.data.time!==job.physics.time||this.data.qpos.some((v,i)=>v!==job.physics.qpos[i]))throw new Error('共享世界已改变，拒绝过期路径');
      applyPlanningResult(c,result.payload);c.task='putaway';operation.planning=false;operation.startY=target.start[1];operation.dual=false;operation.maxLift=0;operation.forcePeaks=[0,0];operation.startedAt=this.data.time;
      if(owner!==(operation.preferredOwner??operation.candidates[0])){this.stats.reassignments++;this._log(`${LABELS[owner]} 接替${LABELS[operation.preferredOwner??operation.candidates[0]]}的可达任务。`);}
      this.lastWall=null;this.accumulator=0;
    }catch(error){if(generation!==this.generation)return;this.state='failed';this.error=error.message;this._log(`路径检查失败：${error.message}`);this.active=null;}
  }
  _beginConveyor(tray,stage){
    if(stage==='out')this._verifyTrayPayload(tray,{requireFull:true});
    const target=stage==='out'?toThree(tray.exit):toThree(tray.docks?.[stage]??tray.destination);
    const position=bodyPose(this.data,this.trayBodies.get(tray.orderId)).position;
    this.conveyor={tray,stage,target,settled:0,elapsed:0,timeout:(Math.abs(target[0]-position[0])+Math.abs(target[2]-position[2]))/.08*3+5};this._log(stage==='out'?'齐套核验通过，成品托盘沿输送线离开。':stage==='r1'?'同一托盘前往 R1 待装料交接位。':stage==='c5'?'同一托盘前往 C5 零件装盘位。':'同一托盘前往 G1 装盘位。');
  }
  _updateConveyor(){
    if(!this.conveyor||!['running','draining'].includes(this.state))return;
    const {tray,target,stage}=this.conveyor,body=this.trayBodies.get(tray.orderId),position=bodyPose(this.data,body).position,motors=this.trayMotors.get(tray.orderId);
    this.conveyor.elapsed=(this.conveyor.elapsed??0)+STEP;if(this.conveyor.elapsed>this.conveyor.timeout)throw new Error('托盘未在限定时间内停靠，已停止输送并保留实际物料状态');
    // Empty trays align Y in the separate queue lane before entering G1;
    // moving X first would sweep through the valve source bins. Transfers to
    // C5 align X first to use the narrow connector clear of G1's arm sweep.
    // The dynamic tray and cargo move solely through motors and contact.
    const global=toThree(tray.initialPosition),xDelta=target[0]-position[0],zDelta=target[2]-position[2],speed=.08;
    const xTarget=this.data.ctrl[motors.x]+Math.max(-speed*STEP,Math.min(speed*STEP,target[0]-global[0]-this.data.ctrl[motors.x]));
    const yStep=Math.max(-speed*STEP,Math.min(speed*STEP,-target[2]+global[2]-this.data.ctrl[motors.y]));
    if(stage==='in'||stage==='g1'){
      this.data.ctrl[motors.y]+=yStep;
      if(Math.abs(zDelta)<.015)this.data.ctrl[motors.x]=xTarget;
    }else{
      this.data.ctrl[motors.x]=xTarget;
      if(Math.abs(xDelta)<.015)this.data.ctrl[motors.y]+=yStep;
    }
    this.conveyor.settled=Math.abs(xDelta)<.003&&Math.abs(zDelta)<.003?this.conveyor.settled+STEP:0;
    if(this.conveyor.settled>.4){
      this.conveyor=null;
      this._verifyTrayPayload(tray,{requireFull:stage==='out'});
      if(stage==='out'){
        this.completedOrders++;this.orderIndex++;this._log(`订单 ${tray.orderId} 已完成，五件物料全部真实装盘。`);
        const next=this.metadata.trays[this.orderIndex];if(next&&this.state==='running')this._beginConveyor(next,'in');
        else if(!next){
          if(this.elapsedSeconds>=this.durationSeconds){this.state='complete';this._log('当前来料批次全部完成，作业已安全结束。');}
          else {this.state='awaiting-material';this._log('实体来料批次已完成；等待补料，不虚构后续成品。');}
        }
      }
    }
  }
  _verifyTrayPayload(tray,{requireFull=false}={}){
    const parts=this.metadata.parts.filter(part=>part.orderId===tray.orderId),packed=parts.filter(part=>this.locations[part.id]==='packed'),carried=parts.filter(part=>['packed','handoff'].includes(this.locations[part.id]));
    if(requireFull&&packed.length!==5)throw new Error('托盘缺件，禁止计入齐套成品');
    const trayId=this.trayBodies.get(tray.orderId),trayPose=bodyPose(this.data,trayId).position,supported=new Set(),list=this.data.contact;
    try{for(let i=0;i<Math.min(this.data.ncon,list.size());i++){const c=list.get(i);try{const a=this.model.geom_bodyid[c.geom1],b=this.model.geom_bodyid[c.geom2],force=c.efc_address>=0?Math.abs(this.data.efc_force[c.efc_address]):0;if(force>.005){if(a===trayId)supported.add(b);if(b===trayId)supported.add(a);}}finally{c.delete();}}}finally{list.delete();}
    const measurements=carried.map(part=>{const body=this.partBodies.get(part.id),slot=this.locations[part.id]==='handoff'&&tray.staging?tray.staging:tray.slots.find(slot=>slot.kind===part.kind&&slot.index===part.index),p=bodyPose(this.data,body).position,goal=[trayPose[0]+slot.offset[0],trayPose[1]+slot.offset[2],trayPose[2]-slot.offset[1]],d=this.model.jnt_dofadr[this.model.body_jntadr[body]],speed=Math.hypot(...this.data.qvel.subarray(d,d+3));return {id:part.id,location:this.locations[part.id],supported:supported.has(body),distance:distance(p,goal),speed};});
    if(measurements.some(m=>!m.supported||m.distance>.02||m.speed>.025))throw new Error('托盘物料未通过实际支撑、槽位与稳定性复核，禁止继续输送或计数');
    this.lastTrayVerification={orderId:tray.orderId,simTime:this.data.time,requireFull,parts:measurements};return this.lastTrayVerification;
  }
  _finishOperation(){
    const op=this.active,agent=this.agents.get(op.owner),c=agent.controller,m=agent.measure();
    if(c.state!=='complete'||!op.dual||op.maxLift<.035||!m.support||m.distance>.02||m.speed>.025){
      this.state='failed';this.error=c.message||'零件未通过真实双指夹持、抬升和落盘支撑核验';this._log(`${LABELS[op.owner]}：${this.error}`);return;
    }
    this.evidence.push({partId:op.part.id,body:op.part.body,phase:op.phase,owner:op.owner,arm:c.arm,dual:op.dual,lift:op.maxLift,forces:op.forcePeaks,distance:m.distance,supported:m.support,endedAt:this.data.time});
    this.locations[op.part.id]=op.phase==='supply'?'handoff':'packed';
    if(op.phase==='supply')this.stats.sharedHandoffs++;else this.completedParts++;
    this._log(`${LABELS[op.owner]} 已${op.phase==='supply'?'释放交接':'装入'}${KIND_LABELS[op.part.kind]}，支撑核验通过。`);this.active=null;
  }
  stepPhysics(){
    const started=now(),before=this.metadata.parts.map(part=>{const b=this.partBodies.get(part.id),j=this.model.body_jntadr[b],q=this.model.jnt_qposadr[j],d=this.model.jnt_dofadr[j];return {q,d,p:Array.from(this.data.qpos.subarray(q,q+3)),v:Math.hypot(...this.data.qvel.subarray(d,d+3))};});
    for(const agent of this.agents.values())agent.stepControls(STEP);
    this._updateConveyor();this._updateNavigation();this.api.mj_step(this.model,this.data);
    this.stats.physicsSteps++;
    // Native control interpolation is 1 kHz; FK/render feedback need not be.
    if(this.stats.physicsSteps%10===0)for(const agent of this.agents.values())agent.afterWorldStep();
    for(const previous of before){
      const speed=Math.hypot(...this.data.qvel.subarray(previous.d,previous.d+3)),delta=distance(previous.p,Array.from(this.data.qpos.subarray(previous.q,previous.q+3)));
      if(!finite(speed)||speed>1)throw new Error('检测到零件失稳或异常速度，作业已停止');
      const residual=Math.max(0,delta-STEP*Math.max(speed,previous.v));this.stats.maxPartSpeed=Math.max(this.stats.maxPartSpeed,speed);this.stats.maxPartIntegrationResidual=Math.max(this.stats.maxPartIntegrationResidual,residual);
      if(residual>1e-5)throw new Error('检测到非物理零件位移，作业已停止');
    }
    if(this.stats.physicsSteps%10===0){
      for(const [id,agent]of this.agents)for(const joint of agent.dynamics.map.values()){
        const actual=this.data.qpos[joint.q],effort=this.data.qfrc_actuator[joint.d];if(!finite(actual)||!finite(effort)||Math.abs(effort)>joint.effort+1e-6)throw new Error(`${LABELS[id]}关节状态或力矩超限`);
        if(joint.effort)this.stats.maxTorqueRatio=Math.max(this.stats.maxTorqueRatio,Math.abs(effort)/joint.effort);
        if(joint.finger){const excess=Math.max(0,joint.lower-actual,actual-joint.upper);this.stats.maxFingerViolation[id]=Math.max(this.stats.maxFingerViolation[id],excess);if(excess>(id==='r1'?.001:.0002))throw new Error(`${LABELS[id]}夹爪行程超限`);}
      }
      if(this.active&&!this.active.planning){
        const m=this.agents.get(this.active.owner).measure();this.active.dual||=m.count===2;this.stats.dualContactSamples+=m.count===2?1:0;
        this.active.maxLift=Math.max(this.active.maxLift,m.worldPosition.y-this.active.startY);
        m.forces.forEach((force,i)=>this.active.forcePeaks[i]=Math.max(this.active.forcePeaks[i],force));
      }
    }
    if(this.active?.mobileSupply&&!this.active.transported&&!this.navigation&&this.agents.get('r1').controller.state==='transfer')this._beginNavigation(this.metadata.mobility.docks.handoff,{carry:true,label:'R1 载物前往交接位'});
    this.stats.maxStepMs=Math.max(this.stats.maxStepMs,now()-started);
  }
  advance(wallNow=this.clock(),{maxSteps=100,budgetMs=20}={}){
    if(this.disposed)return;
    // An initialized idle scene is already physically settled. Do not burn a
    // second 1 kHz simulation merely while inspecting the page. A real stop
    // still steps contact physics to hold a suspended part, unlike idle.
    if(this.state==='idle'||this.state==='complete'){this.lastWall=null;this.accumulator=0;return;}
    if(['running','draining','waiting','awaiting-material'].includes(this.state)){
      this.elapsedSeconds=Math.max(0,(wallNow-this.startedAt)/1000);
      if(this.elapsedSeconds>=this.durationSeconds&&['running','waiting','awaiting-material'].includes(this.state)){this.state='draining';this._log('已达到目标时长，停止接新任务并安全收尾。');}
    }
    const delta=this.lastWall===null?0:Math.max(0,(wallNow-this.lastWall)/1000);this.lastWall=wallNow;
    if(this.active?.planning)return;
    try{
    if(this.state==='running'&&!this.active&&!this.conveyor&&!this.navigation){
      const operation=this._nextOperation();
      if(operation?.type==='dispatch')this._beginConveyor(operation.tray,'out');
      else if(operation?.type==='conveyor')this._beginConveyor(operation.tray,operation.stage);
      else if(operation?.type==='navigate')this._beginNavigation(operation.dock);
      else if(operation)this._prepare(operation);
      if(this.active?.planning)return;
    }
    }catch(error){this.stop();this.state='failed';this.error=error.message;this._log(error.message);return;}
    if(this.state==='running'&&this.active?.resumeState){const c=this.agents.get(this.active.owner).controller;c.state=this.active.resumeState;c.running=!this.navigation;delete this.active.resumeState;}
    this.accumulator+=Math.min(delta,.10);if(delta>.1)this.stats.droppedWallSeconds+=delta-.1;
    const started=now();let steps=0;
    try{while(this.accumulator>=STEP&&steps<maxSteps&&now()-started<budgetMs){this.stepPhysics();this.accumulator-=STEP;steps++;}}
    catch(error){this.stop();this.state='failed';this.error=error.message;this._log(error.message);}
    if(this.accumulator>.15){this.stats.droppedWallSeconds+=this.accumulator-.15;this.accumulator=.15;}
    if(this.active&&!this.active.planning&&!this.navigation&&!this.agents.get(this.active.owner).controller.running&&['running','draining'].includes(this.state))this._finishOperation();
    if(this.state==='draining'&&!this.active&&!this.conveyor&&!this.navigation){this.state='complete';this._log('五分钟作业已安全结束；未完成的订单不计入成品。');}
  }
  get snapshot(){
    const active=this.active,staged=this.metadata.parts.find(part=>part.orderId===this.metadata.trays[this.orderIndex]?.orderId&&this.locations[part.id]==='handoff');
    return {state:this.state,error:this.error,elapsedSeconds:this.elapsedSeconds,durationSeconds:this.durationSeconds,simTime:this.data.time,completedOrders:this.completedOrders,completedParts:this.completedParts,
      currentOrder:this.metadata.trays[this.orderIndex]?.orderId??null,activeRobot:active?.owner??(this.navigation?'r1':null),activePart:active?.part.id??(this.conveyor?staged?.id??null:null),stagedPart:staged?.id??null,phase:active?.planning?'planning':this.navigation?'navigation':active?.phase??(this.conveyor?'conveyor':'ready'),phaseLabel:this.navigation?.label??(this.conveyor?`托盘前往${this.conveyor.stage==='g1'?'G1 工位':this.conveyor.stage==='c5'?'C5 工位':this.conveyor.stage==='r1'?'R1 交接位':'出料区'}`:null),
      robots:IDS.map(id=>{const agent=this.agents.get(id),c=agent.controller,telemetry=agent.dynamics.telemetry(c.arm);return {id,label:LABELS[id],enabled:this.enabled[id],state:id==='r1'&&this.navigation?'navigation':active?.owner===id&&active.planning?'planning':c.state,message:id==='r1'&&this.navigation?this.navigation.label:c.message,basePose:bodyPose(this.data,this.mountBodies.get(id)),pose:Object.fromEntries([...agent.dynamics.map].map(([name,j])=>[name,this.data.qpos[j.q]])),force:agent.measure().force,torque:Math.max(0,...telemetry.joints.map(j=>Math.abs(j.torque)))};}),
      parts:this.metadata.parts.map(part=>({...part,...bodyPose(this.data,this.partBodies.get(part.id)),location:this.locations[part.id]})),
      trays:this.metadata.trays.map(tray=>({...tray,id:tray.orderId,...bodyPose(this.data,this.trayBodies.get(tray.orderId))})),
      bom:this.metadata.parts.filter(part=>part.orderId===this.metadata.trays[this.orderIndex]?.orderId).map(part=>({id:part.id,kind:part.kind,label:KIND_LABELS[part.kind],location:this.locations[part.id],packed:this.locations[part.id]==='packed'})),
      events:this.events.slice(-18),evidence:structuredClone(this.evidence),lastTrayVerification:structuredClone(this.lastTrayVerification),stats:structuredClone(this.stats),metadata:this.metadata};
  }
  dispose(){if(this.disposed)return;this.disposed=true;this.generation++;for(const agent of this.agents.values())agent.dispose();this.data.delete();this.model.delete();}
}
