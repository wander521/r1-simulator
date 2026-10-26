import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {buildCollaborationModel} from '../dist/collab/model.mjs';
import {CollaborationCore} from '../dist/collab/core.mjs';
import {createSharedPlanningService} from '../dist/collab/planner.mjs';
import {SharedCollisionChecker} from '../dist/collab/robot-adapter.mjs';
import {bodyPose,physicalSnapshot} from '../dist/collab/state.mjs';

const root=new URL('../',import.meta.url),started=performance.now(),checks=[],plans=[],phaseSamples=[],transports=[],runtimeSafetyRejections=[];
const argument=name=>process.argv.find(x=>x.startsWith(`--${name}=`))?.slice(name.length+3);
const operationLimit=Number(argument('operation-limit')??3),full=process.argv.includes('--full-tray'),takeover=process.argv.includes('--takeover'),testCarryStop=!process.argv.includes('--no-carry-stop');
const orderCount=Number(argument('order-count')??1),nextOrder=process.argv.includes('--next-order'),takeoverNextOrder=process.argv.includes('--takeover-next-order');
const diagnoseRejections=process.argv.includes('--diagnose-rejections');
let activePlanDiagnostics=null;
const originalCollisionCheck=SharedCollisionChecker.prototype.check;
if(diagnoseRejections)SharedCollisionChecker.prototype.check=function(...args){
  const result=originalCollisionCheck.apply(this,args);
  if(activePlanDiagnostics&&!result.safe){
    activePlanDiagnostics.count++;
    if(activePlanDiagnostics.first.length<8)activePlanDiagnostics.first.push({contacts:result.contacts,limitViolations:result.limitViolations,minDistance:result.minDistance,opening:args[1]?.opening});
  }
  return result;
};
if(nextOrder&&(!full||orderCount<2))throw new Error('--next-order requires --full-tray and --order-count>=2');
if(takeoverNextOrder&&!nextOrder)throw new Error('--takeover-next-order requires --next-order');
const paths=['work/test-collab-physical.mjs',...['model','core','state','planner','robot-adapter','task-plans'].map(name=>`dist/collab/${name}.mjs`)];
for(const id of['r1','g1','c5'])paths.push(`dist/${id}/models/${id}-dynamic-scene.xml`,`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,`dist/${id}/${id}-dynamics.mjs`,id==='r1'?'dist/r1/assets/r1.urdf':id==='g1'?'dist/g1/assets/g1_29dof_with_dex1_1.urdf':'dist/c5/assets/c5-dex1.urdf');
paths.push('dist/multi/r1-safe-dynamics.mjs','dist/multi/planning.mjs','dist/multi/planning-serialization.mjs','dist/multi/planning-worker-core.mjs');
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
const sourceHashes=await hashes();let core,service,metadata,setupError;
function contacts(){
  if(!core)return[];const result=[],list=core.data.contact;
  try{for(let i=0;i<Math.min(core.data.ncon,list.size());i++){const c=list.get(i);try{result.push({geom1:core.api.mj_id2name(core.model,core.api.mjtObj.mjOBJ_GEOM.value,c.geom1),geom2:core.api.mj_id2name(core.model,core.api.mjtObj.mjOBJ_GEOM.value,c.geom2),dist:c.dist,force:c.efc_address>=0?core.data.efc_force[c.efc_address]:0});}finally{c.delete();}}}finally{list.delete();}return result;
}
function transportState(){
  if(!core)return null;const conveyor=core.conveyor;
  return{conveyor:conveyor?{stage:conveyor.stage,target:conveyor.target,settled:conveyor.settled,position:bodyPose(core.data,core.trayBodies.get(conveyor.tray.orderId)).position,
    axes:Object.fromEntries(['x','y'].map(axis=>{const j=core.id('JOINT',axis==='x'?conveyor.tray.jointX:conveyor.tray.jointY),a=core.trayMotors.get(conveyor.tray.orderId)[axis],q=core.model.jnt_qposadr[j],d=core.model.jnt_dofadr[j];return[axis,{actual:core.data.qpos[q],velocity:core.data.qvel[d],command:core.data.ctrl[a],force:core.data.qfrc_actuator[d]}];}))}:null,
    navigation:core.navigation?{carry:core.navigation.carry,goal:core.navigation.goal,dock:core.navigation.dock,settled:core.navigation.settled,lostGrip:core.navigation.lostGrip}:null};
}
async function check(name,run){const start=performance.now();try{const detail=await run();checks.push({name,pass:true,wallSeconds:(performance.now()-start)/1000,detail});console.log('PASS',name);}catch(error){checks.push({name,pass:false,wallSeconds:(performance.now()-start)/1000,error:error.message,errorDetail:error.detail,stack:error.stack,state:core?.state,coreError:core?.error,active:core?.active?{owner:core.active.owner,phase:core.active.phase,part:core.active.part?.id,planning:core.active.planning}:null,transportState:transportState(),controllers:core?Object.fromEntries([...core.agents].map(([id,a])=>[id,{state:a.controller.state,message:a.controller.message,planError:a.controller.planError}])):null,events:core?.events,contacts:contacts()});console.error('FAIL',name,error.message);}}
try{
  const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
  const sources=Object.fromEntries(await Promise.all(['r1','g1','c5'].map(async id=>[id,await readFile(new URL(`dist/${id}/models/${id}-dynamic-scene.xml`,root),'utf8')])));
  const urdfs=Object.fromEntries(await Promise.all([['r1','dist/r1/assets/r1.urdf'],['g1','dist/g1/assets/g1_29dof_with_dex1_1.urdf'],['c5','dist/c5/assets/c5-dex1.urdf']].map(async([id,path])=>[id,await readFile(new URL(path,root),'utf8')])));
  const built=buildCollaborationModel(sources,{orderCount});metadata=built.metadata;
  service=createSharedPlanningService({api,...built,urdfs});
  core=new CollaborationCore({api,...built,urdfs,clock:()=>0,plan:async job=>{
    const before=physicalSnapshot(core.data),started=performance.now();activePlanDiagnostics=diagnoseRejections?{count:0,first:[]}:null;
    let result;try{result=service.plan(job);}finally{if(!diagnoseRejections)activePlanDiagnostics=null;}
    const record={jobId:job.jobId,robotId:job.robotId,partId:job.target.id,arm:job.arm,supportBody:job.supportBody,ok:result.ok,reason:result.reason,error:result.error,steps:result.steps,wallMs:performance.now()-started,...(activePlanDiagnostics?{rejections:activePlanDiagnostics}:{})};activePlanDiagnostics=null;
    assert.deepEqual(physicalSnapshot(core.data),before,'Planning mutated shared live physics');plans.push(record);console.log('PLAN',JSON.stringify(record));return result;
  }});
  // Observe the authoritative runtime safety result without changing its
  // return value, the trajectory, contact filtering or a physical threshold.
  for(const[id,agent]of core.agents){const validate=agent.validateRemainingPlan.bind(agent);agent.validateRemainingPlan=(options={})=>{
    const result=validate(options);if(result?.safe===false)runtimeSafetyRejections.push({robotId:id,partId:core.active?.part?.id,time:core.data.time,result});return result;
  };}
  const worldIdentity={model:core.model,data:core.data};
  core.start({durationSeconds:300});
  function physicsStep(){
    const previous=core.metadata.parts.map(part=>{const body=core.partBodies.get(part.id),joint=core.model.body_jntadr[body],q=core.model.jnt_qposadr[joint],d=core.model.jnt_dofadr[joint];return{partId:part.id,q,d,p:[...core.data.qpos.slice(q,q+3)],v:Math.hypot(...core.data.qvel.slice(d,d+3))};}),time=core.data.time;
    core.stepPhysics();assert(Math.abs(core.data.time-time-.001)<1e-10,'Shared physics did not advance by exactly 1ms');
    for(const value of previous){const v=Math.hypot(...core.data.qvel.slice(value.d,value.d+3)),delta=Math.hypot(...value.p.map((p,i)=>core.data.qpos[value.q+i]-p)),residual=Math.max(0,delta-.001*Math.max(value.v,v));assert(residual<=1e-5,`${value.partId} nonphysical position jump ${residual}`);}
  }
  async function nextManipulation(){
    for(let action=0;action<12;action++){
      const operation=core._nextOperation();assert(operation,'No remaining operation');if(operation.type==='manipulate')return operation;
      const startTime=core.data.time,record={type:operation.type,stage:operation.stage,startTime,steps:0};transports.push(record);
      if(operation.type==='conveyor'){
        record.tray=operation.tray.body;record.before=bodyPose(core.data,core.trayBodies.get(operation.tray.orderId)).position;record.maxForce={x:0,y:0};record.maxSpeed={x:0,y:0};
        record.cargo=metadata.parts.filter(part=>part.orderId===operation.tray.orderId&&['packed','handoff'].includes(core.locations[part.id])).map(part=>({id:part.id,body:part.body,location:core.locations[part.id],before:bodyPose(core.data,core.partBodies.get(part.id)).position}));
        record.supportSamples=0;record.allCargoSupportedSamples=0;core._beginConveyor(operation.tray,operation.stage);
        let stalled=0;
        for(;record.steps<60000&&core.conveyor;record.steps++){
          physicsStep();const state=transportState().conveyor;if(!state)break;
          for(const axis of['x','y']){record.maxForce[axis]=Math.max(record.maxForce[axis],Math.abs(state.axes[axis].force));record.maxSpeed[axis]=Math.max(record.maxSpeed[axis],Math.abs(state.axes[axis].velocity));assert(Math.abs(state.axes[axis].force)<=60+1e-6);}
          if(record.steps%10===0){const list=core.data.contact,supported=new Set(),trayBody=core.trayBodies.get(operation.tray.orderId);try{for(let i=0;i<Math.min(core.data.ncon,list.size());i++){const c=list.get(i);try{const a=core.model.geom_bodyid[c.geom1],b=core.model.geom_bodyid[c.geom2],force=c.efc_address>=0?Math.abs(core.data.efc_force[c.efc_address]):0;if(force>.005&&[a,b].includes(trayBody))supported.add(a===trayBody?b:a);}finally{c.delete();}}}finally{list.delete();}record.supportSamples++;if(record.cargo.every(part=>supported.has(core.partBodies.get(part.id))))record.allCargoSupportedSamples++;}
          const yFirst=['in','g1'].includes(operation.stage),axis=yFirst?(Math.abs(state.position[2]-state.target[2])>=.015?'y':'x'):(Math.abs(state.position[0]-state.target[0])>=.015?'x':'y'),motor=state.axes[axis],error=Math.abs(state.position[axis==='x'?0:2]-state.target[axis==='x'?0:2]);
          stalled=record.steps>3000&&Math.abs(motor.velocity)<1e-4&&error>.02?stalled+1:0;assert(stalled<2000,`Conveyor statically stalled on ${axis}; force/command/actual recorded in transportState`);
        }
        assert.equal(core.conveyor,null,'Conveyor did not reach a physical dock');record.after=bodyPose(core.data,core.trayBodies.get(operation.tray.orderId)).position;
        for(const cargo of record.cargo){cargo.after=bodyPose(core.data,core.partBodies.get(cargo.id)).position;cargo.relativeTransportError=Math.hypot(...cargo.after.map((value,i)=>value-cargo.before[i]-(record.after[i]-record.before[i])));assert(cargo.relativeTransportError<.015,'Loaded cargo left the moving tray');}if(record.cargo.length)assert(record.allCargoSupportedSamples>=record.supportSamples*.95,'Cargo did not remain physically supported during transfer');
        const dock=operation.tray.docks[operation.stage],expected=[dock[0],dock[2],-dock[1]];assert(Math.hypot(...record.after.map((v,i)=>v-expected[i]))<.006);
      }else if(operation.type==='navigate'){
        record.before=bodyPose(core.data,core.mountBodies.get('r1')).position;core._beginNavigation(operation.dock);
        for(;record.steps<60000&&core.navigation;record.steps++)physicsStep();assert.equal(core.navigation,null,'R1 did not reach its real feed dock');assert(core._atDock(operation.dock));record.after=bodyPose(core.data,core.mountBodies.get('r1')).position;
      }else throw new Error(`Expected manipulation prerequisite, got ${operation.type}`);
      record.endTime=core.data.time;assert.equal(core.state,'running',core.error);
    }
    throw new Error('Too many prerequisite transports before manipulation');
  }
  async function execute(operation,{owner,expectedOwner=owner,label}={}){
    const targetBody=core.partBodies.get(operation.part.id),before=bodyPose(core.data,targetBody),historyIndex=core.evidence.length;
    await core._prepare(operation);assert.equal(core.state,'running',core.error);assert(core.active&&!core.active.planning,'No ready manipulation after planning');
    assert.equal(core.active.part.body,operation.part.body);const active=core.active,agent=core.agents.get(active.owner),controller=agent.controller;
    if(expectedOwner)assert.equal(active.owner,expectedOwner,'The requested takeover did not use the expected physical robot');
    const sample={label,partId:operation.part.id,body:operation.part.body,bodyId:targetBody,owner:active.owner,phase:operation.phase,initialPosition:before.position,worldDrop:operation.drop,physicalDualSamples:0,minDualForces:[Infinity,Infinity],maxLift:0,maxSpeed:0,states:[],steps:0,carryNavigationSamples:0,carryStop:null,mobileCarryTripsBefore:core.stats.mobileCarryTrips};
    phaseSamples.push(sample);let lastState;
    for(let step=0;step<150000&&(controller.running||core.navigation||active.navigationResume);step++){
      physicsStep();sample.steps++;
      const state=core.navigation?'navigation':controller.state;
      if(state!==lastState){sample.states.push({state,time:core.data.time});lastState=state;console.log('PHASE',label,state,core.data.time.toFixed(3));}
      if(step%10===0){const m=agent.measure();sample.maxSpeed=Math.max(sample.maxSpeed,m.speed);sample.maxLift=Math.max(sample.maxLift,m.worldPosition.y-before.position[1]);if(m.count===2&&m.forces.every(f=>f>.015)){sample.physicalDualSamples++;sample.minDualForces=m.forces.map((f,i)=>Math.min(sample.minDualForces[i],f));}}
      if(core.navigation?.carry){
        sample.carryNavigationSamples++;
        if(testCarryStop&&!sample.carryStop&&sample.carryNavigationSamples>=1500){
          const snapshot=physicalSnapshot(core.data),grip=[...controller.fingers[controller.arm]].map(f=>[f.actuator,core.data.ctrl[f.actuator]]),opening=controller.opening;
          core.stop();assert.equal(core.state,'stopped');assert.equal(controller.running,false);assert.deepEqual([...core.data.qpos],snapshot.qpos);assert.deepEqual([...core.data.qvel],snapshot.qvel);assert.equal(core.data.time,snapshot.time);assert.equal(controller.opening,opening);
          for(const[a,value]of grip)assert.equal(core.data.ctrl[a],value,'Stop released the held gripper');
          let heldSamples=0;const pauseStart=bodyPose(core.data,targetBody).position;
          for(let pause=0;pause<500;pause++){physicsStep();if(pause%10===0){const m=agent.measure();if(m.count===2&&m.forces.every(f=>f>.015))heldSamples++;}}
          assert.equal(controller.opening,opening);assert(heldSamples>=45,'Real double-finger grip did not persist through stop');const pauseEnd=bodyPose(core.data,targetBody).position;assert(Math.abs(pauseEnd[1]-pauseStart[1])<.01,'Stopped carry dropped vertically');
          sample.carryStop={physicalStopSeconds:.5,heldSamples,totalSamples:50,opening,pauseStart,pauseEnd,stopQposWrites:0,gripperCommandsRetained:true};
          assert(core.start({durationSeconds:300}));core.advance(0,{maxSteps:0});assert.equal(core.state,'running');assert(core.navigation?.carry,'Carry navigation was lost on resume');
        }
      }
    }
    assert(!core.navigation&&!active.navigationResume,'Manipulation ended before carry navigation docked');core._finishOperation();sample.finalPosition=bodyPose(core.data,targetBody).position;sample.finalMeasurement={...agent.measure(),contacts:undefined};sample.evidence=core.evidence[historyIndex];sample.controllerError=controller.planError;sample.controllerMessage=controller.message;
    sample.worldDropError=Math.hypot(...sample.finalPosition.map((v,i)=>v-operation.drop[i]));
    assert.equal(core.state,'running',core.error);assert.equal(controller.state,'complete',controller.message);assert.equal(core.evidence.length,historyIndex+1);
    assert(sample.physicalDualSamples>0,'No two real finger forces during operation');assert(sample.maxLift>.035,'No physical lift');assert(sample.evidence.dual&&sample.evidence.supported);
    assert(sample.finalMeasurement.distance<.02&&sample.finalMeasurement.speed<.025&&sample.finalMeasurement.support);assert.equal(sample.evidence.body,operation.part.body);assert(sample.worldDropError<.02,`Actual shared-world release error ${sample.worldDropError}`);
    if(active.mobileSupply){assert(sample.carryNavigationSamples>0,'R1 supplied without real navigation');assert(core.stats.mobileCarryTrips>sample.mobileCarryTripsBefore);assert(core._atDock(metadata.mobility.docks.handoff));if(testCarryStop)assert(sample.carryStop);}
    return sample;
  }
  await check('G1 grasps the shared valve and releases it into the shared physical tray',async()=>{
    const operation=await nextManipulation();assert.equal(operation.part.kind,'valve');return execute(operation,{owner:'g1',label:'G1 valve'});
  });
  if(checks.every(c=>c.pass)&&takeover){const before=physicalSnapshot(core.data);core.setRobotEnabled('c5',false);assert.equal(core.enabled.c5,false);assert.deepEqual(physicalSnapshot(core.data),before,'Taking C5 offline changed live physics');}
  if(checks.every(c=>c.pass)&&operationLimit>=2)await check('R1 supplies a shared physical pin to the common handoff area',async()=>{
    const operation=await nextManipulation();assert.equal(operation.phase,'supply');assert.equal(operation.part.kind,'pin');const tray=metadata.trays[0],p=bodyPose(core.data,core.trayBodies.get(tray.orderId)).position;
    assert.equal(operation.supportBody,tray.body,'R1 must hand off onto the real shared tray, not a copied table object');
    const expected=[p[0]+tray.staging.offset[0],p[1]+tray.staging.offset[2],p[2]-tray.staging.offset[1]];assert(Math.hypot(...operation.drop.map((v,i)=>v-expected[i]))<1e-9);
    return execute(operation,{owner:'r1',label:'R1 pin tray staging'});
  });
  if(checks.every(c=>c.pass)&&operationLimit>=3)await check(`${takeover?'G1 takes over C5 and':'A downstream robot'} re-grasps the exact pin body released by R1 and places it in the same tray`,async()=>{
    const supplied=phaseSamples.at(-1),operation=await nextManipulation();assert.equal(operation.phase,'pack');assert.equal(operation.part.id,supplied.partId);assert.equal(core.partBodies.get(operation.part.id),supplied.bodyId);
    const sample=await execute(operation,{expectedOwner:takeover?'g1':undefined,label:`${takeover?'G1 takeover':'Downstream'} same pin`});
    if(takeover){assert.equal(core.enabled.c5,false);assert(core.stats.reassignments>0,'The takeover was not counted as real task reassignment');const moves=transports.filter(t=>t.type==='conveyor'&&t.stage==='g1'&&t.cargo?.some(p=>p.id===supplied.partId&&p.location==='handoff'));assert(moves.length>0,'The exact staged pin was not physically transported back to G1');}
    return sample;
  });
  if(checks.every(c=>c.pass)&&full)await check('All five BOM parts complete real shared handoff and tray placement',async()=>{
    while(core.completedParts<5){const operation=await nextManipulation();await execute(operation,{label:`${operation.phase} ${operation.part.id}`});}
    assert.equal(core.completedParts,5);assert.equal(core.evidence.length,9);assert.equal(core.stats.sharedHandoffs,4);return{completedParts:core.completedParts,evidence:core.evidence};
  });
  if(checks.every(c=>c.pass)&&full)await check('A loaded dynamic tray carries loose cargo through motor/contact transport with no part writes',async()=>{
    const tray=metadata.trays[0],firstPayload=metadata.parts.filter(part=>part.orderId===tray.orderId),initialTray=bodyPose(core.data,core.trayBodies.get(tray.orderId)).position,initial=new Map(firstPayload.map(part=>[part.id,bodyPose(core.data,core.partBodies.get(part.id)).position]));core._beginConveyor(tray,'out');
    for(let step=0;step<30000&&core.conveyor;step++)physicsStep();assert.equal(core.conveyor,null);assert.equal(core.completedOrders,1);assert.equal(core.completedParts,5);assert.equal(core.orderIndex,1);
    // This accelerated single-order fixture has not consumed 300 wall seconds.
    // Production correctly waits for actual material instead of fabricating
    // another order or claiming the five-minute run is already complete.
    assert.equal(core.state,orderCount===1?'awaiting-material':'running');assert.equal(core.active,null);assert.equal(core.navigation,null);
    const trayPosition=bodyPose(core.data,core.trayBodies.get(tray.orderId)).position;assert(Math.abs(trayPosition[0]-tray.exit[0])<.003);
    const displacement=trayPosition.map((v,i)=>v-initialTray[i]);for(const part of firstPayload){const p=bodyPose(core.data,core.partBodies.get(part.id)).position,old=initial.get(part.id);assert(Math.abs(p[0]-old[0]-displacement[0])<.015,'Cargo did not travel with tray');assert(Math.abs(p[2]-old[2]-displacement[2])<.015);assert(Math.abs(p[1]-old[1])<.01);}return{trayPosition,displacement};
  });
  if(checks.every(c=>c.pass)&&nextOrder)await check('The next physical order enters from its queue without resetting the model, data, completed tray or object identities',async()=>{
    const checkpoint={time:core.data.time,steps:core.stats.physicsSteps,evidence:core.evidence.length,completedParts:core.completedParts,reassignments:core.stats.reassignments};
    const firstTray=metadata.trays[0],firstPartPositions=new Map(metadata.parts.filter(part=>part.orderId===firstTray.orderId).map(part=>[part.id,bodyPose(core.data,core.partBodies.get(part.id)).position]));
    assert.equal(core.orderIndex,1);assert.equal(core.completedOrders,1);assert.equal(core.completedParts,5);
    let operation=await nextManipulation();assert.equal(operation.part.orderId,'order_2');assert.equal(operation.phase,'load-valve');const valve=await execute(operation,{expectedOwner:'g1',label:'Second-order G1 valve'});
    if(takeoverNextOrder){const snapshot=physicalSnapshot(core.data);core.setRobotEnabled('c5',false);assert.equal(core.enabled.c5,false);assert.deepEqual(physicalSnapshot(core.data),snapshot,'Taking C5 offline changed second-order live physics');}
    operation=await nextManipulation();assert.equal(operation.part.orderId,'order_2');assert.equal(operation.phase,'supply');const supplied=await execute(operation,{expectedOwner:'r1',label:'Second-order R1 tray staging'});
    operation=await nextManipulation();assert.equal(operation.phase,'pack');assert.equal(operation.part.id,supplied.partId);assert.equal(core.partBodies.get(operation.part.id),supplied.bodyId);
    const packed=await execute(operation,{expectedOwner:takeoverNextOrder?'g1':undefined,label:`Second-order ${takeoverNextOrder?'G1 takeover':'downstream'} same pin`});
    assert.equal(core.model,worldIdentity.model);assert.equal(core.data,worldIdentity.data);assert(core.data.time>checkpoint.time);assert(core.stats.physicsSteps>checkpoint.steps);assert.equal(core.evidence.length,checkpoint.evidence+3);assert.equal(core.completedParts,7);
    assert.notEqual(supplied.bodyId,phaseSamples.find(s=>s.partId==='order_1_pin_1').bodyId,'A new order reused a copied old pin body');
    core._verifyTrayPayload(firstTray,{requireFull:true});
    for(const part of metadata.parts.filter(part=>part.orderId===firstTray.orderId)){const before=firstPartPositions.get(part.id),after=bodyPose(core.data,core.partBodies.get(part.id)).position;assert(Math.hypot(...after.map((v,i)=>v-before[i]))<.001,'Completed first-order cargo was reset or disturbed');}
    if(takeoverNextOrder){assert(core.stats.reassignments>checkpoint.reassignments);assert(transports.some(t=>t.type==='conveyor'&&t.stage==='g1'&&t.cargo?.some(p=>p.id===supplied.partId&&p.location==='handoff')));}
    return{checkpoint,endedAt:core.data.time,modelAndDataIdentityRetained:true,completedOrders:core.completedOrders,completedParts:core.completedParts,secondOrderBodyIds:[valve.bodyId,supplied.bodyId,packed.bodyId],newReassignments:core.stats.reassignments-checkpoint.reassignments};
  });
}catch(error){setupError={message:error.message,stack:error.stack};checks.push({name:'Shared collaboration setup',pass:false,error:error.message,stack:error.stack,contacts:contacts()});console.error(error);}
const stats=core?structuredClone(core.stats):null,evidence=core?structuredClone(core.evidence):[],events=core?structuredClone(core.events):[],snapshot=core?.snapshot;
core?.dispose();service?.dispose();SharedCollisionChecker.prototype.check=originalCollisionCheck;
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes),pass=!setupError&&sourceFrozen&&checks.length>0&&checks.every(c=>c.pass);
const report={pass,scope:'Accelerated execution of the production single-world CollaborationCore, shared full-model planner and 1ms physics, tray-mounted staging, physical dock prerequisites, actual mobile carry and stop/hold/resume. Integration residuals and source bindings detect object jumps; no navigation/part state setters are used by this harness after initialization. This is not proof of 300 seconds real-wall-time continuous work.',fullTray:full,takeover,testCarryStop,operationLimit,orderCount,nextOrder,takeoverNextOrder,total:checks.length,passed:checks.filter(c=>c.pass).length,wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,endSourceHashes,checks,plans,phaseSamples,transports,runtimeSafetyRejections,stats,evidence,events,snapshot};
const destination=new URL(argument('report')??'work/validation/collab/physical-development.json',root);await mkdir(new URL('./',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass,total:report.total,passed:report.passed,sourceFrozen,wallSeconds:report.wallSeconds,report:destination.pathname}));if(!pass)process.exitCode=1;
