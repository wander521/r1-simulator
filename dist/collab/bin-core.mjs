import {ROBOT_CONFIGS} from '../multi/robot-runtime.mjs';
import {createPlanningWorkerRobot} from '../multi/planning-worker-core.mjs';
import {applyPlanningResult,encodePlanningValue,decodePlanningValue} from '../multi/planning-serialization.mjs';
import {createRobotAgent} from './robot-adapter.mjs';
import {BinPipeline} from './pipeline.mjs';
import {verifyBinInventory,verifyHandoffClearance} from './bin-evidence.mjs';
import {sameFreeBodyPose,guardedBodies} from './planning-guard.mjs';
import {advancePlanarCommand} from './mobile-profile.mjs';
import {requiresHeldContact} from './resume-route.mjs';
import {IDS,bodyPose,toThree,robotFrame,plainFields,physicalSnapshot,CONTROL_FIELDS,MOTOR_FIELDS} from './state.mjs';

const STEP=.001,distance=(a,b)=>Math.hypot(...a.map((v,i)=>v-b[i]));
export const SIMULATION_RATE=2.4;
const zoneFor=slot=>slot?.kind==='full'?'g1-out':slot?.kind==='input'?'c5-in':slot?.kind==='returned'?'empty-return':null;

/** Independent, authoritative bin-cell executor. There is one physical world,
 * but no global planning/motion barrier. Each station owns one action; only
 * local handoff-zone leases serialize work. Planner promises run elsewhere. */
export class BinCollaborationCore{
  constructor({api,xml,metadata,urdfs,plan,cancelPlan=()=>{}}){
    Object.assign(this,{api,xml,metadata,urdfs,plan,cancelPlan});this.generation=0;this.disposed=false;
    const vfs=new api.MjVFS();try{this.model=api.MjModel.from_xml_string(xml,vfs);this.data=new api.MjData(this.model);}finally{vfs.delete();}
    this.objectBodies=new Map([...metadata.bins,...metadata.parts].map(o=>[o.id,this.id('BODY',o.body)]));
    this.baseAxes=Object.fromEntries(['x','y','yaw'].map(axis=>{const j=this.id('JOINT',metadata.mobility.joints[axis]);return[axis,{name:metadata.mobility.joints[axis],q:this.model.jnt_qposadr[j],d:this.model.jnt_dofadr[j],motor:this.id('ACTUATOR',metadata.mobility.actuators[axis])}];}));
    this.mountBody=this.id('BODY',metadata.mobility.mountName);this._initialize();
  }
  id(type,name){const n=this.api.mj_name2id(this.model,this.api.mjtObj[`mjOBJ_${type}`].value,name);if(n<0)throw new Error(`Missing ${type}: ${name}`);return n;}
  _initialize(){
    this.agents=new Map(IDS.map(id=>[id,createRobotAgent({api:this.api,model:this.model,data:this.data,id,metadata:robotFrame(this.metadata.robots[id],this.metadata.workbench),robot:createPlanningWorkerRobot(this.urdfs[id],{rootHeight:ROBOT_CONFIGS[id].rootHeight}),targets:[]})]));
    const slots=kind=>this.metadata.slots.filter(s=>s.kind===kind).map(s=>s.id);
    const partDestinations=Object.fromEntries(this.metadata.parts.map(p=>[p.id,{destination:p.outputSlot,partIndex:p.outputIndex}]));
    this.pipeline=new BinPipeline({bins:this.metadata.bins,fullSlots:slots('full'),inputSlots:slots('input'),returnSlots:slots('returned'),outputSlots:slots('output'),partDestinations});
    this.actions=new Map();this.evidence=[];this.lastWall=null;this.accumulator=0;this.error=null;
    this.stats={physicsSteps:0,planningJobs:0,stalePlans:0,stepsWhilePlanning:0,planningMotionOverlapSteps:0,multiRobotMotionSteps:0,maxMovingRobots:0,droppedWallSeconds:0};
    for(let i=0;i<300;i++)this.stepPhysics();
  }
  start({durationSeconds=300}={}){
    if(this.disposed)throw new Error('Cell disposed');
    if(this.pipeline.state==='paused'){this.resume();return;}
    if(this.pipeline.state!=='idle')throw new Error('Reset the cell before starting a new batch');
    if(!Number.isFinite(durationSeconds)||durationSeconds<=0||durationSeconds>900)throw new Error('Invalid work window');
    this.pipeline.durationSeconds=durationSeconds;this.pipeline.start();this.lastWall=null;
  }
  _slot(id){const slot=this.metadata.slots.find(s=>s.id===id);if(!slot)throw new Error(`Unknown slot ${id}`);return slot;}
  _target(action){
    if(['home','ready'].includes(action.job.type))return undefined;
    const j=action.job,bin=this.metadata.bins.find(b=>b.id===j.binId),slot=this._slot(j.destination);
    if(j.type==='pack'){
      const part=this.metadata.parts.find(p=>p.id===j.partId);
      return {...part,start:bodyPose(this.data,this.objectBodies.get(part.id)).position,drop:toThree(slot.partSlots[j.partIndex])};
    }
    const source=this.metadata.slots.find(s=>s.id===j.source);
    return {...bin,payloadBodies:bin.parts.filter(id=>!this.pipeline.parts.get(id).packed).map(id=>this.metadata.parts.find(p=>p.id===id).body),sourceSupport:source?.supportBody??bin.sourceSupport,start:bodyPose(this.data,this.objectBodies.get(bin.id)).position,drop:toThree(slot.position)};
  }
  _support(action){return ['home','ready'].includes(action.job.type)?undefined:action.job.type==='pack'?`fixture_${action.job.destination}`:this._slot(action.job.destination).supportBody;}
  _context(){return Object.fromEntries([...this.agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}]));}
  _guard(action,job){
    const agent=this.agents.get(action.job.robot),indices=[...agent.dynamics.map.values()].map(j=>j.q);
    if(indices.some(q=>Math.abs(this.data.qpos[q]-job.physics.qpos[q])>.02))return false;
    if(action.job.robot==='r1'&&Object.values(this.baseAxes).some(a=>Math.abs(this.data.qpos[a.q]-job.physics.qpos[a.q])>.004))return false;
    for(const name of guardedBodies(job.target)){
      const b=this.id('BODY',name),j=this.model.body_jntadr[b],q=this.model.jnt_qposadr[j];
      const object=[...this.metadata.bins,...this.metadata.parts].find(o=>o.body===name);
      if(!sameFreeBodyPose(this.data.qpos,job.physics.qpos,q,{axialSymmetry:object?.shape==='cylinder'}))return false;
    }
    return true;
  }
  _submit(action,kind,extra,accept){
    if(action.pending||this.data.time<(action.nextPlanTime??0))return;
    const robotId=action.job.robot,agent=this.agents.get(robotId),c=agent.controller,generation=this.generation;
    const job={kind,robotId,jobId:action.job.id,physics:physicalSnapshot(this.data),robots:this._context(),...extra};
    action.pending=true;action.pendingKind=kind;action.token=(action.token??0)+1;const token=action.token;
    c.running=false;c.state='planning';this.stats.planningJobs++;
    if(this.pipeline.job(action.job.id).status==='reserved')this.pipeline.planning(action.job.id);
    Promise.resolve().then(()=>this.plan(job)).then(result=>{
      if(this.disposed||action.failed||generation!==this.generation||this.actions.get(robotId)!==action||token!==action.token)return;
      action.pending=false;
      if(!this._guard(action,job)){
        this.stats.stalePlans++;action.replans=(action.replans??0)+1;
        // Let real loose contents settle before taking another snapshot. Do
        // not burn the entire retry allowance on consecutive simulation ticks.
        action.nextPlanTime=this.data.time+.25*action.replans;
        if(action.replans>3)this._fail(action,'规划期间本工位物料/姿态持续变化，保持当前夹持');return;
      }
      if(!result.ok)throw Object.assign(new Error(result.reason??'路径检查失败'),{detail:result.detail??result.error??result.check});
      action.replans=0;if(this.pipeline.job(action.job.id).status==='planning')this.pipeline.executing(action.job.id);
      accept(result);
    }).catch(error=>{if(!this.disposed&&generation===this.generation&&this.actions.get(robotId)===action&&token===action.token){action.pending=false;this._fail(action,error.message,error.detail);}});
  }
  _fail(action,message,detail){
    if(action.failed)return;action.failed=true;action.token=(action.token??0)+1;action.pending=false;this.cancelPlan(action.job.robot);action.detail=detail;this.pipeline.fail(action.job.id,message);
    const a=this.agents.get(action.job.robot);a.controller.fail(message);a.pausePosture(true);
    if(action.navigation)for(const axis of Object.values(this.baseAxes))this.data.ctrl[axis.motor]=this.data.qpos[axis.q];
  }
  _acquire(action,zone){return !zone||this.pipeline.enterZone(action.job.id,zone);}
  _clearance(action,zone){return verifyHandoffClearance({api:this.api,model:this.model,data:this.data,metadata:this.metadata,robotId:action.job.robot,zone});}
  _release(action,zone){
    if(zone&&this.pipeline.leases.get(`zone:${zone}`)===action.job.id){
      const result=this._clearance(action,zone);if(!result.clear)throw Object.assign(new Error('机械臂仍占用交接空间，保留区域预约'),{detail:result});
      this.pipeline.leaveZone(action.job.id,zone,{clear:true});
    }
  }
  _dockGoal(dock){const origin=this.metadata.robots.r1.origin;return{x:dock.position[0]-origin[0],y:dock.position[1]-origin[1],yaw:dock.yaw-this.metadata.robots.r1.yaw};}
  _aisleDock(x){return {position:[x,this.metadata.mobility.aisleY,this.metadata.robots.r1.origin[2]],yaw:this.metadata.robots.r1.yaw};}
  _homeDock(){return {position:[...this.metadata.robots.r1.origin],yaw:this.metadata.robots.r1.yaw};}
  _atDock(dock){const goal=this._dockGoal(dock),t=this.metadata.mobility.dockTolerance;return distance(bodyPose(this.data,this.mountBody).position,toThree(dock.position))<t.position&&Object.entries(this.baseAxes).every(([key,a])=>Math.abs(this.data.qvel[a.d])<t.speed&&(key!=='yaw'||Math.abs(this.data.qpos[a.q]-goal[key])<t.yaw));}
  _navigate(action,dock,carry,next){
    const a=this.agents.get('r1'),c=a.controller,goal=this._dockGoal(dock),target=this._target(action);
    const from=Object.fromEntries(Object.values(this.baseAxes).map(v=>[v.name,this.data.qpos[v.q]])),to=Object.fromEntries(Object.entries(this.baseAxes).map(([k,v])=>[v.name,goal[k]]));
    if(this._atDock(dock)){action.phase=next;return;}
    this._submit(action,target?'mobile':'mobile-empty',{target,arm:'right',supportBody:this._support(action),carry,fromMount:{jointPositions:from},toMount:{jointPositions:to}},()=>{
      c.running=false;c.state=carry?'transport_hold':'navigation';a.setMobilityInterlock({moving:true});
      action.navigation={dock,goal,carry,next,velocity:[0,0],elapsed:0,stable:0,lostGrip:0,timeout:distance(bodyPose(this.data,this.mountBody).position,toThree(dock.position))/this.metadata.mobility.maxSpeed*2+6};
    });
  }
  _drive(action){
    const nav=action.navigation;if(!nav||action.failed||action.resume)return;
    nav.elapsed+=STEP;if(nav.elapsed>nav.timeout){this._fail(action,'底盘停靠超时');return;}
    const keys=['x','y'],profile=advancePlanarCommand(keys.map(key=>this.data.ctrl[this.baseAxes[key].motor]),nav.velocity,keys.map(key=>nav.goal[key]),STEP,{maxSpeed:this.metadata.mobility.maxSpeed,maxAcceleration:.18});
    nav.velocity=profile.velocity;keys.forEach((key,i)=>{this.data.ctrl[this.baseAxes[key].motor]=profile.position[i];});
    const yawError=nav.goal.yaw-this.data.ctrl[this.baseAxes.yaw.motor];
    this.data.ctrl[this.baseAxes.yaw.motor]+=Math.max(-.15*STEP,Math.min(.15*STEP,yawError));
    nav.stable=this._atDock(nav.dock)?nav.stable+STEP:0;
    if(nav.stable>=.4){this.agents.get('r1').setMobilityInterlock({moving:false});action.navigation=null;action.phase=nav.next;}
  }
  _manipulate(action,mobile=false){
    const target=this._target(action),agent=this.agents.get(action.job.robot),supportBody=this._support(action);
    if(mobile){const destination=this.metadata.mobility.docks[action.job.destination],base=bodyPose(this.data,this.mountBody).position;target.drop=target.drop.map((v,k)=>v-toThree(destination.position)[k]+base[k]);}
    const arm=action.job.robot==='g1'&&action.job.destination==='full-2'?'left':'right';
    agent.configureTask({target,arm,supportBody});
    action.target=target;action.pickStart=[...target.start];action.dualSamples=0;action.maxLift=0;
    this._submit(action,'manipulation',{target,arm:agent.controller.arm,supportBody},result=>{applyPlanningResult(agent.controller,result.payload);action.phase=mobile?'picking':'manipulation';});
  }
  _finish(action){
    const a=this.agents.get(action.job.robot),j=action.job,c=a.controller,m=a.measure();
    const posture=['ready','home'].includes(j.type);
    if(j.type==='home'&&j.robot==='r1'&&!this._atDock(this._homeDock()))throw new Error('R1 底盘尚未稳定返回初始停靠位');
    const evidence=posture?{ready:j.type==='home'?c.state==='home':c.sharedAtReady===true}:{ready:c.sharedAtReady===true,objectId:j.partId??j.binId,destination:j.destination,dualContact:action.dualSamples>10,lifted:action.maxLift>.035,supported:m.support&&m.distance<.02&&m.speed<.025};
    if(!posture){
      evidence.inventory=verifyBinInventory({metadata:this.metadata,pipeline:this.pipeline,model:this.model,data:this.data,bodyId:name=>this.id('BODY',name),job:j});
      if(!evidence.inventory.valid)throw Object.assign(new Error('实物库存检查失败，保留工位及料盒预约'),{detail:evidence.inventory});
      for(const resource of this.pipeline.job(j.id).resources.filter(r=>r.startsWith('zone:'))){
        const clearance=this._clearance(action,resource.slice(5));if(!clearance.clear)throw Object.assign(new Error('准备位未退出交接空间'),{detail:clearance});
      }
    }
    this.pipeline.complete(j.id,evidence);this.evidence.push({...evidence,jobId:j.id,robot:j.robot,type:j.type,simTime:this.data.time});this.actions.delete(j.robot);
  }
  _progress(action){
    if(action.failed||action.pending)return;
    if(action.resume){this._resumeAction(action);return;}
    if(action.navigation)return;
    const j=action.job,a=this.agents.get(j.robot),c=a.controller;
    if(action.phase==='posture'){if(!a.postureMotion){if(c.state===j.type)this._finish(action);else this._fail(action,c.message);}return;}
    if(action.phase==='need-posture'){this._submit(action,'posture',{posture:j.type},result=>{a.beginPosture(result.posture);action.phase='posture';});return;}
    // Empty base returns through the checked aisle before unfolding to home.
    // Never reset mount joints or declare home at the last return bin.
    if(action.phase==='home-outbound'){this._navigate(action,this._aisleDock(this.data.xpos[this.mountBody*3]),false,'home-travel');return;}
    if(action.phase==='home-travel'){this._navigate(action,this._aisleDock(this.metadata.robots.r1.origin[0]),false,'home-dock');return;}
    if(action.phase==='home-dock'){this._navigate(action,this._homeDock(),false,'need-posture');return;}
    if(action.phase==='await-zone'){
      if(this._acquire(action,j.type==='fetch'?'g1-out':'c5-in'))this._manipulate(action);return;
    }
    if(action.phase==='await-source'){
      this._navigate(action,this._aisleDock(this.data.xpos[this.mountBody*3]),false,'need-source-travel');return;
    }
    if(action.phase==='need-source-travel'){this._navigate(action,this._aisleDock(this.metadata.mobility.docks[j.source].position[0]),false,'await-source-zone');return;}
    if(action.phase==='await-source-zone'){if(this._acquire(action,zoneFor(this._slot(j.source))))action.phase='need-source-dock';return;}
    if(action.phase==='need-source-dock'){this._navigate(action,this.metadata.mobility.docks[j.source],false,'need-pick');return;}
    if(action.phase==='need-pick'){this._manipulate(action,true);return;}
    if(action.phase==='need-outbound'){
      this._navigate(action,this._aisleDock(this.data.xpos[this.mountBody*3]),true,'clear-source');return;
    }
    if(action.phase==='clear-source'){
      const zone=zoneFor(this._slot(j.source)),bin=this.metadata.bins.find(b=>b.id===j.binId);
      const clear=verifyHandoffClearance({api:this.api,model:this.model,data:this.data,metadata:this.metadata,robotId:j.robot,zone,objectBodies:[bin.body,...this._target(action).payloadBodies]});
      if(!clear.clear)throw Object.assign(new Error('料盒尚未退出源工位，保留交接预约'),{detail:clear});
      this.pipeline.sourceDeparted(j.id,{objectId:j.binId,clear:true,dualContact:a.measure().count===2,lifted:action.maxLift>.035});
      this._release(action,zone);action.phase='await-destination';
    }
    if(action.phase==='await-destination'){
      this._navigate(action,this._aisleDock(this.metadata.mobility.docks[j.destination].position[0]),true,'await-destination-zone');return;
    }
    if(action.phase==='await-destination-zone'){if(this._acquire(action,zoneFor(this._slot(j.destination))))action.phase='need-destination-dock';return;}
    if(action.phase==='need-destination-dock'){this._navigate(action,this.metadata.mobility.docks[j.destination],true,'need-place');return;}
    if(action.phase==='need-place'){
      this._submit(action,'loaded-placement',{target:this._target(action),arm:c.arm,supportBody:this._support(action),worldDrop:toThree(this._slot(j.destination).position),planData:encodePlanningValue(c.planData)},result=>{c.planData=decodePlanningValue(result.planData);c.running=true;c.transition('transfer');action.phase='placing';});return;
    }
    if(['manipulation','picking','placing'].includes(action.phase)){
      if(c.state==='failed'){this._fail(action,c.message,c.planError);return;}
      if(action.phase==='picking'&&c.state==='transfer'){c.running=false;c.state='transport_hold';action.phase='need-outbound';return;}
      if(!c.running&&c.state==='complete')this._finish(action);
    }
  }
  schedule(wallSeconds){
    if(!['running','draining','homing'].includes(this.pipeline.state))return;
    for(const job of this.pipeline.tick(wallSeconds))this.actions.set(job.robot,{job,phase:job.type==='home'&&job.robot==='r1'?'home-outbound':['ready','home'].includes(job.type)?'need-posture':['fetch','pack'].includes(job.type)?'await-zone':'await-source',dualSamples:0,maxLift:0});
    // Clear completed upstream work first. G1 must not reserve the handoff
    // volume for its next bin just before R1 can collect the previous one.
    const actions=[...this.actions.values()].sort((a,b)=>Number(b.job.robot==='r1'&&b.phase==='await-source')-Number(a.job.robot==='r1'&&a.phase==='await-source'));
    for(const action of actions)try{this._progress(action);}catch(error){this._fail(action,error.message,error.detail);}
  }
  stepPhysics(){
    const active=['running','draining','homing'].includes(this.pipeline.state),moving=[...this.agents.values()].filter(a=>a.controller.running||a.postureMotion&&!a.postureMotion.paused).length+([...this.actions.values()].some(a=>a.navigation&&!a.failed)&&active?1:0),planning=[...this.actions.values()].some(a=>a.pending);
    this.stats.physicsSteps++;if(planning)this.stats.stepsWhilePlanning++;if(planning&&moving)this.stats.planningMotionOverlapSteps++;if(moving>1)this.stats.multiRobotMotionSteps++;this.stats.maxMovingRobots=Math.max(this.stats.maxMovingRobots,moving);
    if(active)for(const action of this.actions.values())this._drive(action);
    for(const agent of this.agents.values())agent.stepControls(STEP);this.api.mj_step(this.model,this.data);
    if(!this.data.qpos.every(Number.isFinite))throw new Error('Non-finite shared physics state');
    if(this.stats.physicsSteps%10===0){
      for(const agent of this.agents.values())agent.afterWorldStep();
      for(const action of this.actions.values())if(action.pickStart&&!action.failed){
        const agent=this.agents.get(action.job.robot),m=agent.measure();if(m.count===2)action.dualSamples++;action.maxLift=Math.max(action.maxLift,m.worldPosition.y-action.pickStart[1]);
        if(requiresHeldContact(action,action.resume?.state??agent.controller.state)){
          action.lostGrip=m.count===2?0:(action.lostGrip??0)+.01;
          if(action.lostGrip>.15)this._fail(action,'持物期间双指接触丢失，已保持机械臂并停止底盘');
        }else action.lostGrip=0;
      }
    }
  }
  advance(wallNow,{maxSteps=160,budgetMs=20}={}){
    if(!Number.isFinite(wallNow))throw new Error('Invalid monotonic clock');
    const delta=this.lastWall===null?0:Math.max(0,(wallNow-this.lastWall)/1000);this.lastWall=wallNow;
    // Double simulated time, never the integration step, forces or servo gains.
    // The five-minute admission window still uses the real monotonic clock.
    this.schedule(wallNow/1000);this.accumulator+=Math.min(.1,delta)*SIMULATION_RATE;if(delta>.1)this.stats.droppedWallSeconds+=delta-.1;
    const started=performance.now();let steps=0;
    while(this.accumulator>=STEP&&steps<maxSteps&&performance.now()-started<budgetMs){this.stepPhysics();this.accumulator-=STEP;steps++;}
    if(this.accumulator>.3){this.stats.droppedWallSeconds+=(this.accumulator-.3)/SIMULATION_RATE;this.accumulator=.3;}
    this.schedule(wallNow/1000);
  }
  stop(){
    if(!['running','draining','homing'].includes(this.pipeline.state))return;
    this.pipeline.pause();this.generation++;this.cancelPlan();
    for(const action of this.actions.values()){
      const a=this.agents.get(action.job.robot),c=a.controller;action.pending=false;
      // A second stop may cancel a resume check; keep the ORIGINAL phase.
      action.resume??={state:c.state,running:c.running,physics:physicalSnapshot(this.data)};c.running=false;c.state='stopped';a.pausePosture(true);
      if(action.navigation){action.navigation.velocity=[0,0];for(const axis of Object.values(this.baseAxes))this.data.ctrl[axis.motor]=this.data.qpos[axis.q];}
    }
  }
  resume(){
    if(this.pipeline.state!=='paused')throw new Error('Cell is not paused');
    // A stopped live grasp is never silently resumed after contact loss.
    for(const action of this.actions.values())if(requiresHeldContact(action,action.resume?.state)&&this.agents.get(action.job.robot).measure().count!==2)throw new Error('暂停后夹持丢失，请检查物料并复位');
    // Scheduling resumes, but each robot stays held until its own background
    // route check passes. No synchronous IK and no global planning barrier.
    this.pipeline.resume();this.lastWall=null;
  }
  _resumeAction(action){
    const a=this.agents.get(action.job.robot),c=a.controller,saved=action.resume;
    const restore=()=>{c.state=saved.state;c.running=saved.running;delete action.resume;a.pausePosture(false);};
    if(action.navigation){
      const nav=action.navigation,from=Object.fromEntries(Object.values(this.baseAxes).map(v=>[v.name,this.data.qpos[v.q]])),to=Object.fromEntries(Object.entries(this.baseAxes).map(([key,v])=>[v.name,nav.goal[key]]));
      const target=this._target(action);
      this._submit(action,target?'mobile':'mobile-empty',{target,arm:c.arm,supportBody:this._support(action),carry:nav.carry,fromMount:{jointPositions:from},toMount:{jointPositions:to}},restore);return;
    }
    if(a.postureMotion){this._submit(action,'resume-posture',{resume:{state:saved.state},posture:structuredClone(a.postureMotion)},restore);return;}
    if(saved.running){
      const target=this._target(action);
      if(['spread','raise','ready','recenter','crouch','align_clear','align','approach','descend','closing'].includes(saved.state)){
        for(const name of guardedBodies(target)){
          const body=this.id('BODY',name),q=this.model.jnt_qposadr[this.model.body_jntadr[body]],object=[...this.metadata.bins,...this.metadata.parts].find(o=>o.body===name);
          if(!sameFreeBodyPose(this.data.qpos,saved.physics.qpos,q,{axialSymmetry:object?.shape==='cylinder'})){this._fail(action,'暂停期间待抓物料发生移动，请检查并复位');return;}
        }
      }
      this._submit(action,'resume-manipulation',{target,arm:c.arm,supportBody:this._support(action),resume:{state:saved.state},pickOnly:action.phase==='picking',planData:encodePlanningValue(c.planData)},restore);return;
    }
    // Waiting/cancelled planning has no running trajectory to restore. The
    // normal phase will create a fresh plan from the current physical world.
    restore();
  }
  reset(){this.generation++;this.cancelPlan();for(const a of this.agents.values())a.dispose();this.api.mj_resetData(this.model,this.data);this._initialize();}
  get snapshot(){return {ledger:this.pipeline.snapshot(),stats:{...this.stats},simTime:this.data.time,actions:[...this.actions.values()].map(a=>({jobId:a.job.id,robot:a.job.robot,type:a.job.type,phase:a.phase,pending:a.pending??false,failed:a.failed??false,detail:a.detail})),evidence:structuredClone(this.evidence)};}
  dispose(){if(this.disposed)return;this.disposed=true;this.generation++;this.cancelPlan();for(const a of this.agents.values())a.dispose();this.data.delete();this.model.delete();}
}
