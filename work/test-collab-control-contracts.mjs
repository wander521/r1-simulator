import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {CollaborationCore} from '../dist/collab/core.mjs';

// These are production Core methods with deliberately controlled state and
// measurement fixtures. No MuJoCo simulation, collision/grasp success, real
// navigation, browser rendering or five-minute wall-clock run is proved here.
const root=new URL('../',import.meta.url),checks=[],started=performance.now();
const paths=['work/test-collab-control-contracts.mjs','dist/collab/core.mjs','dist/collab/state.mjs'];
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
const sourceHashes=await hashes();
function check(name,run){
  try{const detail=run();checks.push({name,pass:true,detail});console.log('PASS',name);}
  catch(error){checks.push({name,pass:false,error:error.message,stack:error.stack});console.error('FAIL',name,error.message);}
}
function coreStub(fields={}){
  return Object.assign(Object.create(CollaborationCore.prototype),{
    disposed:false,generation:0,state:'idle',elapsedSeconds:0,durationSeconds:300,startedAt:0,lastWall:null,accumulator:0,
    active:null,conveyor:null,navigation:null,baseAxes:null,enabled:{r1:true,g1:true,c5:true},agents:new Map(),
    data:{time:0,qpos:new Float64Array(0),qvel:new Float64Array(0),ctrl:new Float64Array(0)},
    metadata:{parts:[],trays:[]},completedOrders:0,completedParts:0,orderIndex:0,events:[],
    stats:{droppedWallSeconds:0},clock:()=>0,cancelPlan:()=>{},
  },fields);
}

/** Five distinct slot/body/contact measurements in a synthetic arrival state.
 * Positions are MuJoCo Z-up and pass through production bodyPose conversion.
 * Contacts are hand-authored boundary inputs, not simulated evidence. */
function trayFixture(){
  const specs=[['valve',0,0,0,.023],['pin',0,-.07,.065,.030],['pin',1,.07,.065,.030],['sleeve',0,-.07,-.065,.030],['sleeve',1,.07,-.065,.030]];
  const tray={orderId:'order_1',body:'tray__order_1',jointX:'tray_x',jointY:'tray_y',initialPosition:[.53,.31,.904],destination:[.53,.31,.904],exit:[.53,.31,.904],
    slots:specs.map(([kind,index,x,y,z])=>({kind,index,offset:[x,y,z]}))};
  const parts=specs.map(([kind,index],i)=>({id:`fixture_${kind}_${index}`,orderId:tray.orderId,kind,index,body:`fixture_body_${i}`}));
  const data={time:12,xpos:new Float64Array(7*3),xquat:new Float64Array(7*4),qpos:new Float64Array(2),qvel:new Float64Array(32),ctrl:new Float64Array(2),efc_force:new Float64Array([.1,.1,.1,.1,.1]),ncon:5};
  for(let body=0;body<7;body++)data.xquat[body*4]=1;
  data.xpos.set(tray.initialPosition,3);
  parts.forEach((part,i)=>{const slot=tray.slots[i],body=i+2;data.xpos.set(tray.initialPosition.map((value,axis)=>value+slot.offset[axis]),body*3);});
  const model={geom_bodyid:new Int32Array([1,2,3,4,5,6]),body_jntadr:new Int32Array([0,0,2,3,4,5,6]),jnt_dofadr:new Int32Array([0,1,2,8,14,20,26])};
  const lifecycle={lists:0,contacts:0};
  Object.defineProperty(data,'contact',{get(){return{
    size:()=>5,
    get(index){assert(index>=0&&index<5);return{geom1:0,geom2:index+1,efc_address:index,delete(){lifecycle.contacts++;}};},
    delete(){lifecycle.lists++;},
  };}});
  const core=coreStub({state:'running',elapsedSeconds:120,metadata:{trays:[tray],parts},data,model,
    trayBodies:new Map([[tray.orderId,1]]),trayMotors:new Map([[tray.orderId,{x:0,y:1}]]),
    partBodies:new Map(parts.map((part,i)=>[part.id,i+2])),locations:Object.fromEntries(parts.map(part=>[part.id,'packed'])),
  });
  return{core,tray,parts,data,model,lifecycle};
}
function setArrival(core,tray){
  core.conveyor={tray,target:[tray.initialPosition[0],tray.initialPosition[2],-tray.initialPosition[1]],stage:'out',settled:.401};
}
function physicalArrays(data){return Object.fromEntries(['xpos','xquat','qpos','qvel','ctrl'].map(key=>[key,Array.from(data[key])]));}

check('Repeated stop preserves the first native phase and does not release the saved operation',()=>{
  const controller={state:'transfer',running:true},core=coreStub({state:'running',active:{owner:'r1'},agents:new Map([['r1',{controller}]])});
  core.stop();const generation=core.generation;core.stop();
  assert.equal(core.state,'stopped');assert.equal(core.active.resumeState,'transfer');assert.equal(controller.state,'stopped');assert.equal(controller.running,false);assert.equal(core.generation,generation);
  return{resumeState:core.active.resumeState,repeatedStopWasIdempotent:true};
});
check('Stop preserves failed/complete terminal states and cannot bypass reset-before-start',()=>{
  for(const state of['failed','complete']){const core=coreStub({state});core.stop();assert.equal(core.state,state);assert.throws(()=>core.start(),/复位/);}
  return{terminalStates:['failed','complete']};
});
check('Blocked empty navigation fails safely once instead of staying running and retrying each frame',()=>{
  let attempts=0;
  const core=coreStub({state:'running',_nextOperation:()=>({type:'navigate',dock:{position:[0,0,0]}}),_beginNavigation:()=>{attempts++;throw new Error('controlled blocked route');}});
  core.advance(1000);assert.equal(core.state,'failed');assert.equal(core.error,'controlled blocked route');
  core.advance(1033,{maxSteps:0});assert.equal(attempts,1);assert.equal(core.state,'failed');
  return{routeAttempts:attempts,state:core.state};
});
check('Batch exhaustion before 300 seconds waits for physical material; a controlled deadline finishes without admitting work',()=>{
  const{core,tray}=trayFixture();setArrival(core,tray);core._updateConveyor();
  assert.equal(core.completedOrders,1);assert.equal(core.elapsedSeconds,120);assert.equal(core.state,'awaiting-material');
  core.advance(300000,{maxSteps:0});assert.equal(core.elapsedSeconds,300);assert.equal(core.state,'complete');assert.equal(core.completedOrders,1);assert.equal(core.active,null);
  return{beforeDeadline:'awaiting-material',controlledClockSeconds:core.elapsedSeconds,orders:core.completedOrders,realWallClockTest:false};
});
check('Lost bilateral grip rejects both native-arm and loaded-navigation resume before restarting',()=>{
  for(const navigation of[null,{carry:true}]){
    const controller={state:'stopped'},core=coreStub({state:'stopped',navigation,active:{owner:'r1',part:{id:'fixture_pin'},resumeState:'transfer'},agents:new Map([['r1',{controller,measure:()=>({count:0})}]])});
    assert.throws(()=>core.start(),/失去双指/);assert.equal(core.state,'stopped');assert.equal(controller.state,'stopped');
  }
  return{rejectedModes:['native-arm','loaded-navigation']};
});
check('Supported in-slot payload passes production verification at departure and arrival without changing measured poses',()=>{
  const{core,tray,data,lifecycle}=trayFixture(),before=physicalArrays(data);
  const verification=core._verifyTrayPayload(tray,{requireFull:true});assert.equal(verification.parts.length,5);assert(verification.parts.every(part=>part.supported&&part.distance<1e-10&&part.speed===0));
  core._beginConveyor(tray,'out');assert.equal(core.conveyor.stage,'out');setArrival(core,tray);core._updateConveyor();assert.equal(core.completedOrders,1);
  assert.deepEqual(physicalArrays(data),before);assert.equal(lifecycle.contacts,lifecycle.lists*5);
  return{verifiedParts:5,completedOrders:1,measurementFixturesOnly:true,contactListsDisposed:lifecycle.lists};
});
for(let index=0;index<5;index++)for(const anomaly of['slot-drift','support-lost'])check(`Any slot rejects ${anomaly} at both departure and arrival: piece ${index+1}`,()=>{
  for(const boundary of['departure','arrival']){
    const{core,tray,parts,data}=trayFixture();
    if(anomaly==='slot-drift')data.xpos[(index+2)*3]+=.03;
    else data.efc_force[index]=0;
    const before=physicalArrays(data);
    if(boundary==='departure'){
      assert.throws(()=>core._beginConveyor(tray,'out'),/支撑、槽位与稳定性/);assert.equal(core.conveyor,null);
    }else{
      setArrival(core,tray);assert.throws(()=>core._updateConveyor(),/支撑、槽位与稳定性/);
    }
    assert.equal(core.completedOrders,0);assert.equal(core.orderIndex,0);assert.equal(core.locations[parts[index].id],'packed');assert.deepEqual(physicalArrays(data),before);
  }
  return{piece:index+1,anomaly,rejectedBoundaries:['departure','arrival'],ordersCounted:0};
});
check('Unstable measured payload velocity prevents departure and arrival count',()=>{
  for(const boundary of['departure','arrival']){
    const{core,tray,data,model}=trayFixture();data.qvel[model.jnt_dofadr[model.body_jntadr[4]]]=.026;
    if(boundary==='departure')assert.throws(()=>core._beginConveyor(tray,'out'),/稳定性/);
    else{setArrival(core,tray);assert.throws(()=>core._updateConveyor(),/稳定性/);}
    assert.equal(core.completedOrders,0);
  }
  return{measuredSpeed:.026,existingMaximum:.025};
});
check('Historical location flags cannot count an incomplete five-part payload',()=>{
  for(const boundary of['departure','arrival']){
    const{core,tray,parts}=trayFixture();core.locations[parts[4].id]='handoff';
    if(boundary==='departure')assert.throws(()=>core._beginConveyor(tray,'out'),/缺件/);
    else{setArrival(core,tray);assert.throws(()=>core._updateConveyor(),/缺件/);}
    assert.equal(core.completedOrders,0);
  }
  return{requiredParts:5,packedParts:4,ordersCounted:0};
});

const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
const report={pass:sourceFrozen&&checks.every(check=>check.pass),total:checks.length,passed:checks.filter(check=>check.pass).length,wallSeconds:(performance.now()-started)/1000,
  scope:'Production CollaborationCore prototype methods with controlled state, synthetic contact/slot/velocity measurements and a controlled clock. No MuJoCo stepping or actual physical-grasp/navigation proof; no real 300-second/browser/continuous-production proof. Planner initialization/cancellation is covered separately.',
  sourceFrozen,sourceHashes,endSourceHashes,checks};
const destination=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9);
if(destination){const path=new URL(destination,root);await mkdir(new URL('./',path),{recursive:true});await writeFile(path,JSON.stringify(report,null,2)+'\n');}
console.log(JSON.stringify(report,null,2));if(!report.pass)process.exitCode=1;
