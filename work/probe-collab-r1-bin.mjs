// Isolated diagnostic fork of an actual G1 delivery snapshot. This does not
// prove end-to-end or concurrent runtime; no replay enters the browser world.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {buildBinCollaborationModel} from '../dist/collab/bin-model.mjs';
import {createRobotAgent} from '../dist/collab/robot-adapter.mjs';
import {createSharedPlanningService} from '../dist/collab/planner.mjs';
import {robotFrame,physicalSnapshot,restorePrivateSnapshot,plainFields,CONTROL_FIELDS,MOTOR_FIELDS,toThree,bodyPose} from '../dist/collab/state.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {createPlanningWorkerRobot} from '../dist/multi/planning-worker-core.mjs';
import {applyPlanningResult} from '../dist/multi/planning-serialization.mjs';

const root=new URL('../',import.meta.url),ids=['r1','g1','c5'];
const proofSources=['work/probe-collab-r1-bin.mjs','dist/collab/bin-model.mjs','dist/collab/model.mjs','dist/collab/robot-adapter.mjs','dist/collab/held-placement.mjs','dist/collab/state.mjs','dist/collab/planner.mjs','dist/multi/planning.mjs'];
const hashes=async()=>Object.fromEntries(await Promise.all(proofSources.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')]))),sourceHashes=await hashes();
const fixture=JSON.parse(await readFile(new URL(process.env.FIXTURE??'work/validation/collab/g1-delivery-replay-development.json',root),'utf8'));
const sources=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${id}/models/${id}-dynamic-scene.xml`,root),'utf8')])));
const paths={r1:'r1/assets/r1.urdf',g1:'g1/assets/g1_29dof_with_dex1_1.urdf',c5:'c5/assets/c5-dex1.urdf'};
const urdfs=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${paths[id]}`,root),'utf8')])));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const built=buildBinCollaborationModel(sources,{binCount:4}),{xml,metadata}=built;
assert.equal(createHash('sha256').update(xml).digest('hex'),fixture.modelHash,'Replay fixture model changed; generate a new actual G1 delivery first');
const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
const name=(type,value)=>api.mj_name2id(model,api.mjtObj[`mjOBJ_${type}`].value,value);
function supportedParts(parts,parent){
  const records=parts.map(part=>({id:part.id,body:name('BODY',part.body),position:bodyPose(data,name('BODY',part.body)).position,supported:false})),contacts=data.contact;
  try{for(let i=0;i<Math.min(data.ncon,contacts.size());i++){const contact=contacts.get(i);try{
    if(contact.efc_address>=0&&data.efc_force[contact.efc_address]>.005){const a=model.geom_bodyid[contact.geom1],b=model.geom_bodyid[contact.geom2];for(const part of records)if(a===parent&&b===part.body||b===parent&&a===part.body)part.supported=true;}
  }finally{contact.delete();}}}finally{contacts.delete();}return records;
}
const agents=new Map(ids.map(id=>[id,createRobotAgent({api,model,data,id,metadata:robotFrame(metadata.robots[id],metadata.workbench),robot:createPlanningWorkerRobot(urdfs[id],{rootHeight:ROBOT_CONFIGS[id].rootHeight}),targets:[]})]));
let service;
try{
  restorePrivateSnapshot(api,model,data,fixture.physics);
  for(const [id,a]of agents){Object.assign(a.controller,structuredClone(fixture.robots[id].controller));Object.assign(a.dynamics,structuredClone(fixture.robots[id].dynamics));a.controller.running=false;a.dynamics.syncRobot();}
  if(process.env.G1_READY_PITCH!==undefined){
    const g1=agents.get('g1'),pose={...g1.controller.pose};
    for(const side of ['left','right'])pose[`${side}_shoulder_pitch_joint`]=Number(process.env.G1_READY_PITCH);
    g1.controller.validateSegment(g1.controller.pose,pose,{ignoreSelected:false});
    g1.controller.applyPose(pose);
  }
  // Diagnostic docking adjustments use the actual bounded position motors.
  // They never assign robot/base/cargo qpos or qvel after snapshot creation.
  for(const axis of ['x','y'])if(process.env[`BASE_${axis.toUpperCase()}`]!==undefined){
    const joint=name('JOINT',metadata.mobility.joints[axis]),value=Number(process.env[`BASE_${axis.toUpperCase()}`]);
    assert.ok(value>=model.jnt_range[joint*2]&&value<=model.jnt_range[joint*2+1]);
    data.ctrl[name('ACTUATOR',metadata.mobility.actuators[axis])]=value;
  }
  for(let i=0;i<1500;i++){for(const a of agents.values())a.stepControls(.001);api.mj_step(model,data);if(i%10===0)for(const a of agents.values())a.afterWorldStep();}
  const r1=agents.get('r1'),bin=metadata.bins.find(b=>b.id===fixture.binId),source=metadata.slots.find(s=>s.id===fixture.sourceSlot),input=metadata.slots.find(s=>s.id===(process.env.DESTINATION??'input-1')),dock=metadata.mobility.docks[input.id];
  const base=bodyPose(data,name('BODY',metadata.mobility.mountName)).position,futureBase=toThree(dock.position),drop=toThree(input.position).map((v,k)=>v-futureBase[k]+base[k]);
  const target={...bin,sourceSupport:source.supportBody,start:bodyPose(data,name('BODY',bin.body)).position,drop};
  if(process.env.EMPTY==='1')target.payloadBodies=[];
  const packedBefore=process.env.EMPTY==='1'?supportedParts(metadata.parts.filter(p=>p.binId===bin.id),name('BODY','fixture_output-1')):[];
  assert.ok(packedBefore.every(part=>part.supported),'An empty return cannot start while original parts remain unpacked');
  r1.configureTask({target,arm:process.env.ARM??'left',supportBody:input.supportBody});
  console.log('R1_ENDPOINTS',JSON.stringify({base,start:r1.toLocal(target.start).toArray(),drop:r1.toLocal(drop).toArray(),arm:r1.controller.arm}));
  if(process.env.DIRECT==='1'){
    for(const twist of [.3,.4,.2,0])for(const yaw of [0,.35,-.35]){
      const saved={...r1.controller.pose},started=performance.now();
      try{r1.controller.plan(r1.controller.targets[0],twist,yaw);console.log('CANDIDATE_OK',JSON.stringify({twist,yaw,ms:performance.now()-started}));}
      catch(error){console.log('CANDIDATE_FAIL',JSON.stringify({twist,yaw,reason:error.message,detail:error.detail,ms:performance.now()-started}));}
      finally{r1.controller.applyPose(saved);r1.dynamics.syncRobot();}
    }
  }else{
    service=createSharedPlanningService({api,...built,urdfs});
    const physics=physicalSnapshot(data),job={jobId:1,robotId:'r1',target,arm:r1.controller.arm,supportBody:input.supportBody,physics,robots:Object.fromEntries([...agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}]))};
    const result=service.plan(job);assert.deepEqual(physicalSnapshot(data),physics);console.log('PLAN',JSON.stringify({ok:result.ok,reason:result.reason,error:result.error}));assert.ok(result.ok);
    if(process.env.EXECUTE_PICK==='1'||process.env.EXECUTE_TRANSPORT==='1'){
      applyPlanningResult(r1.controller,result.payload);
      let phase=null,dualSamples=0,maxPregraspShift=0;
      for(let i=0;i<45000&&r1.controller.running&&r1.controller.state!=='transfer';i++){
        for(const a of agents.values())a.stepControls(.001);api.mj_step(model,data);
        if(i%10===0){
          for(const a of agents.values())a.afterWorldStep();
          const m=r1.measure();if(m.count===2)dualSamples++;
          if(['spread','crouch','align','approach','descend'].includes(r1.controller.state))maxPregraspShift=Math.max(maxPregraspShift,Math.hypot(...m.worldPosition.toArray().map((v,k)=>v-target.start[k])));
        }
        if(r1.controller.state!==phase){phase=r1.controller.state;console.log('R1_PHASE',phase,data.time.toFixed(3));}
      }
      const m=r1.measure();console.log('PICK_RESULT',JSON.stringify({state:r1.controller.state,message:r1.controller.message,dualSamples,maxPregraspShift,lift:m.worldPosition.y-target.start[1],count:m.count,forces:m.forces}));
      assert.equal(r1.controller.state,'transfer');assert.ok(dualSamples>10&&m.count===2&&m.worldPosition.y-target.start[1]>.035);assert.ok(maxPregraspShift<.005);
      if(process.env.EXECUTE_TRANSPORT==='1'){
        const c=r1.controller,axes=Object.fromEntries(['x','y','yaw'].map(axis=>{const j=name('JOINT',metadata.mobility.joints[axis]);return [axis,{name:metadata.mobility.joints[axis],q:model.jnt_qposadr[j],d:model.jnt_dofadr[j],motor:name('ACTUATOR',metadata.mobility.actuators[axis])}];}));
        const goal={x:dock.position[0]-metadata.robots.r1.origin[0],y:dock.position[1]-metadata.robots.r1.origin[1],yaw:dock.yaw-metadata.robots.r1.yaw};
        const from=Object.fromEntries(Object.values(axes).map(a=>[a.name,data.qpos[a.q]])),to=Object.fromEntries(Object.entries(axes).map(([key,a])=>[a.name,goal[key]]));
        const check=r1.validateMobileRoute({fromMount:{jointPositions:from},toMount:{jointPositions:to},objectLocalPose:c.objectPose(bin.id)});
        console.log('MOBILE_CHECK',JSON.stringify(check));assert.ok(check.safe,'Loaded mobile route rejected');
        c.running=false;c.state='transport_hold';r1.setMobilityInterlock({moving:true});
        let settled=0,lostGrip=0;
        for(let i=0;i<90000&&settled<.4;i++){
          for(const [key,a]of Object.entries(axes)){const step=(key==='yaw'?.15:metadata.mobility.maxSpeed)*.001;data.ctrl[a.motor]+=Math.max(-step,Math.min(step,goal[key]-data.ctrl[a.motor]));}
          for(const a of agents.values())a.stepControls(.001);api.mj_step(model,data);
          if(i%10===0){for(const a of agents.values())a.afterWorldStep();lostGrip=r1.measure().count===2?0:lostGrip+.01;assert.ok(lostGrip<.15,'Lost dual finger contact in transport');}
          const stable=Object.entries(axes).every(([key,a])=>Math.abs(data.qpos[a.q]-goal[key])<.004&&Math.abs(data.qvel[a.d])<.01);settled=stable?settled+.001:0;
        }
        assert.ok(settled>=.4,'Docking timeout');r1.setMobilityInterlock({moving:false});r1.syncFrame();
        const remaining=r1.replanHeldPlacement({worldDrop:toThree(input.position),supportBody:input.supportBody});console.log('DOCKED_PATH',JSON.stringify(remaining));
        const cargo=metadata.parts.filter(p=>target.payloadBodies.includes(p.body)).map(part=>({id:part.id,body:name('BODY',part.body),supported:false,position:bodyPose(data,name('BODY',part.body)).position})),parent=name('BODY',bin.body),contacts=data.contact;
        try{for(let i=0;i<Math.min(data.ncon,contacts.size());i++){const contact=contacts.get(i);try{
          if(contact.efc_address>=0&&data.efc_force[contact.efc_address]>.005){const a=model.geom_bodyid[contact.geom1],b=model.geom_bodyid[contact.geom2];for(const part of cargo)if(a===parent&&b===part.body||b===parent&&a===part.body)part.supported=true;}
        }finally{contact.delete();}}}finally{contacts.delete();}
        const report={scope:'Isolated physical continuation of actual G1 delivery; not concurrent pipeline or 300-second acceptance',modelHash:fixture.modelHash,fixtureSourceHashes:fixture.sourceHashes,sourceHashes,endSourceHashes:await hashes(),binId:bin.id,dualSamples,maxPregraspShift,lift:m.worldPosition.y-target.start[1],navigationValidated:check.safe,docked:true,remainingPath:remaining,cargo,heldPose:c.objectPose(bin.id),status:remaining.safe?'ready-to-place':'placement-safely-rejected'};
        assert.deepEqual(report.sourceHashes,report.endSourceHashes);
        if(process.env.REPORT)await writeFile(process.env.REPORT,JSON.stringify(report,null,2)+'\n');
        console.log('TRANSPORT_CARGO',JSON.stringify(cargo));assert.ok(cargo.every(part=>part.supported),'Transport cargo must remain in the same physical carrier');
        assert.ok(remaining.safe);
        c.running=true;c.transition('transfer');
        for(let i=0;i<30000&&c.running;i++){
          for(const a of agents.values())a.stepControls(.001);api.mj_step(model,data);if(i%10===0)for(const a of agents.values())a.afterWorldStep();
          if(c.state!==phase){phase=c.state;console.log('R1_PHASE',phase,data.time.toFixed(3));}
        }
        const end=r1.measure();console.log('DELIVERY_RESULT',JSON.stringify({state:c.state,message:c.message,support:end.support,distance:end.distance,position:end.worldPosition.toArray()}));
        assert.equal(c.state,'complete');assert.ok(end.support&&end.distance<.02);assert.equal(c.sharedAtReady,true);assert.ok(!c.history.includes('return'));
        const postDeliveryCargo=supportedParts(metadata.parts.filter(p=>target.payloadBodies.includes(p.body)),name('BODY',bin.body)),packedAfter=process.env.EMPTY==='1'?supportedParts(metadata.parts.filter(p=>p.binId===bin.id),name('BODY','fixture_output-1')):[];
        assert.ok(postDeliveryCargo.every(part=>part.supported));
        assert.ok(packedAfter.every((part,i)=>part.supported&&Math.hypot(...part.position.map((v,k)=>v-packedBefore[i].position[k]))<.002),'Packed parts must remain in their original output fixture during empty-bin return');
        Object.assign(report,{status:'delivered-at-ready',empty:process.env.EMPTY==='1',ready:c.sharedAtReady,support:end.support,distance:end.distance,postDeliveryCargo,packedBefore,packedAfter,history:[...c.history],endSourceHashes:await hashes()});
        assert.deepEqual(report.sourceHashes,report.endSourceHashes);
        if(process.env.REPORT)await writeFile(process.env.REPORT,JSON.stringify(report,null,2)+'\n');
        if(process.env.SNAPSHOT_REPORT)await writeFile(process.env.SNAPSHOT_REPORT,JSON.stringify({scope:'Isolated diagnostic fixture after real G1 pickup and R1 transport/release; never restore into a live browser',modelHash:fixture.modelHash,binId:bin.id,sourceSlot:input.id,physics:physicalSnapshot(data),robots:Object.fromEntries([...agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}])),sourceHashes},null,2)+'\n');
      }
    }
  }
}finally{service?.dispose();for(const a of agents.values())a.dispose();data.delete();model.delete();}
