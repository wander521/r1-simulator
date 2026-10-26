// Development probe: exercises the real shared-world planner/controllers.
// No result here proves concurrent operation or the 300-second acceptance run.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {buildBinCollaborationModel} from '../dist/collab/bin-model.mjs';
import {createRobotAgent} from '../dist/collab/robot-adapter.mjs';
import {createSharedPlanningService} from '../dist/collab/planner.mjs';
import {robotFrame,physicalSnapshot,plainFields,CONTROL_FIELDS,MOTOR_FIELDS,toThree,bodyPose} from '../dist/collab/state.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {createPlanningWorkerRobot} from '../dist/multi/planning-worker-core.mjs';
import {applyPlanningResult} from '../dist/multi/planning-serialization.mjs';
import * as THREE from '../dist/vendor/three.module.min.js';

const root=new URL('../',import.meta.url),ids=['r1','g1','c5'];
const proofSources=['work/probe-collab-bin-grasp.mjs','dist/collab/bin-model.mjs','dist/collab/model.mjs','dist/collab/carrier-kinematics.mjs','dist/collab/robot-adapter.mjs','dist/collab/task-plans.mjs','dist/collab/state.mjs','dist/collab/planner.mjs','dist/g1/grasp-controller.mjs','dist/g1/grasp-kinematics.mjs','dist/multi/planning.mjs'];
const hashes=async()=>Object.fromEntries(await Promise.all(proofSources.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')]))),sourceHashes=await hashes();
const sources=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${id}/models/${id}-dynamic-scene.xml`,root),'utf8')])));
const paths={r1:'r1/assets/r1.urdf',g1:'g1/assets/g1_29dof_with_dex1_1.urdf',c5:'c5/assets/c5-dex1.urdf'};
const urdfs=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${paths[id]}`,root),'utf8')])));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const built=buildBinCollaborationModel(sources,{binCount:Number(process.env.BIN_COUNT??1)}),{xml,metadata}=built,vfs=new api.MjVFS();
const model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
const name=(type,value)=>api.mj_name2id(model,api.mjtObj[`mjOBJ_${type}`].value,value);
const agents=new Map(ids.map(id=>[id,createRobotAgent({api,model,data,id,metadata:robotFrame(metadata.robots[id],metadata.workbench),robot:createPlanningWorkerRobot(urdfs[id],{rootHeight:ROBOT_CONFIGS[id].rootHeight}),targets:[]})]));
let service,step=0;
function advance(){for(const agent of agents.values())agent.stepControls(.001);api.mj_step(model,data);if(step++%10===0)for(const agent of agents.values())agent.afterWorldStep();assert.ok(data.qpos.every(Number.isFinite));}
try{
  for(let i=0;i<500;i++)advance();
  service=createSharedPlanningService({api,...built,urdfs});
  const bin=metadata.bins[Number(process.env.BIN_INDEX??0)],dest=metadata.slots.find(s=>s.id===(process.env.DESTINATION??'full-1')),agent=agents.get('g1'),c=agent.controller;
  const target={...bin,start:bodyPose(data,name('BODY',bin.body)).position,drop:toThree(dest.position)};
  if(process.env.CARRIER_TILT!==undefined)target.graspTilt=Number(process.env.CARRIER_TILT);
  agent.configureTask({target,arm:process.env.ARM??'right',supportBody:dest.supportBody});
  if(process.env.PLAN_SCAN==='1'){
    const beforePose={...c.pose};
    const candidates=process.env.PLAN_HINTS?JSON.parse(process.env.PLAN_HINTS):[.14,.17,.11].flatMap(lateral=>[0,.15,-.15,.3,-.3].flatMap(yaw=>[0,1].map(branch=>[yaw,branch,lateral])));
    for(const [yaw,branch,lateral]of candidates){
      try{c.plan(c.targets[0],yaw,branch,lateral);console.log('CANDIDATE_OK',JSON.stringify({yaw,branch,lateral}));}
      catch(error){console.log('CANDIDATE_FAIL',JSON.stringify({yaw,branch,lateral,message:error.message,detail:error.detail}));}
      finally{c.applyPose(beforePose);agent.dynamics.syncRobot();}
    }
    process.exit(0);
  }
  if(process.env.POSE_SCAN==='1'){
    for(const phase of ['ready','recenter'])try{c.validateSegment(c.pose,c.phaseTarget(phase),{ignoreSelected:false});console.log('DIRECT_READY_SAFE',phase);}catch(error){console.log('DIRECT_READY_REJECT',phase,JSON.stringify(error.detail));}
    for(const roll of [1.35,1.5,1.65,1.8]){
      const spread=c.phaseTarget('spread'),raise=c.phaseTarget('raise'),ready=c.phaseTarget('ready');
      for(const pose of [spread,raise]){pose.left_shoulder_roll_joint=roll;pose.right_shoulder_roll_joint=-roll;}
      let from=c.pose,phase=0;
      try{for(const to of [spread,raise,ready]){c.validateSegment(from,to,{ignoreSelected:false});from=to;phase++;}console.log('WIDE_READY_SAFE',roll);}
      catch(error){console.log('WIDE_READY_REJECT',roll,phase,JSON.stringify(error.detail?.contacts?.[0]??error.detail));}
    }
  }
  const before=physicalSnapshot(data),job={jobId:1,robotId:'g1',target,arm:c.arm,supportBody:dest.supportBody,physics:before,robots:Object.fromEntries([...agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}]))};
  const started=performance.now(),result=service.plan(job);assert.deepEqual(physicalSnapshot(data),before);
  console.log('PLAN',JSON.stringify({ok:result.ok,reason:result.reason,error:result.error,ms:performance.now()-started}));assert.ok(result.ok,'No safe bin grasp plan');
  if(process.env.PLAN_ONLY==='1')process.exitCode=0;
  else{
    applyPlanningResult(c,result.payload);let phase=null,dualSamples=0,lift=0,maxCargoOffset=0,maxPregraspShift=0;
    for(let i=0;i<90000&&c.running;i++){
      advance();if(c.state!==phase){phase=c.state;console.log('PHASE',phase,data.time.toFixed(3));}
      if(i%10===0){const m=agent.measure();if(m.count===2)dualSamples++;lift=Math.max(lift,m.worldPosition.y-target.start[1]);
        if(['spread','raise','ready','align','approach','descend'].includes(c.state))maxPregraspShift=Math.max(maxPregraspShift,Math.hypot(...m.worldPosition.toArray().map((v,k)=>v-target.start[k])));
        if(process.env.CONTACT_DEBUG==='1'&&c.state==='closing'&&i%200===0){
          const tcp=ROBOT_CONFIGS.g1.tcp(c.robot,c.arm);
          console.log('CLOSING_CONTACT',JSON.stringify({time:data.time,opening:c.opening,position:m.position.toArray(),tcp:tcp.position.toArray(),target:c.planData.start.toArray(),wristError:c.wristError(),jointError:c.jointError(),count:m.count,forces:m.forces,contacts:m.contacts.map(contact=>({other:name('BODY',api.mj_id2name(model,api.mjtObj.mjOBJ_BODY.value,contact.other)),point:contact.position.toArray(),force:contact.force}))}));
        }
        const bp=bodyPose(data,name('BODY',bin.body));
        for(const part of metadata.parts.filter(p=>p.binId===bin.id)){
          const p=bodyPose(data,name('BODY',part.body)).position,local=new THREE.Vector3(...p).sub(new THREE.Vector3(...bp.position)).applyQuaternion(new THREE.Quaternion(...bp.quaternion).invert()).toArray();
          maxCargoOffset=Math.max(maxCargoOffset,Math.hypot(...local.map((v,k)=>v-(toThree(part.start)[k]-toThree(bin.start)[k]))));
        }
      }
    }
    const parent=name('BODY',bin.body),cargo=metadata.parts.filter(p=>p.binId===bin.id).map(part=>({id:part.id,body:name('BODY',part.body),supported:false}));
    const contacts=data.contact;
    try{for(let i=0;i<Math.min(data.ncon,contacts.size());i++){
      const contact=contacts.get(i);try{
        const a=model.geom_bodyid[contact.geom1],b=model.geom_bodyid[contact.geom2];
        if(contact.efc_address>=0&&data.efc_force[contact.efc_address]>.005)for(const part of cargo)if(a===part.body&&b===parent||b===part.body&&a===parent)part.supported=true;
      }finally{contact.delete();}
    }}finally{contacts.delete();}
    for(const part of cargo){const joint=model.body_jntadr[part.body],d=model.jnt_dofadr[joint];part.speed=Math.hypot(...data.qvel.slice(d,d+3));part.position=bodyPose(data,part.body).position;}
    const m=agent.measure(),report={scope:'Actual MuJoCo single full-bin grasp; not concurrent pipeline or wall-clock 300-second acceptance',bin:bin.id,arm:c.arm,destination:dest.id,state:c.state,message:c.message,dualSamples,lift,maxCargoOffset,maxPregraspShift,distance:m.distance,supported:m.support,cargo};
    Object.assign(report,{modelVersion:metadata.version,modelHash:createHash('sha256').update(xml).digest('hex'),binCount:metadata.bins.length,searchHint:c.planData.searchHint,returnChecks:c.planData.returnChecks,ready:c.sharedAtReady===true,history:[...c.history],sourceHashes,endSourceHashes:await hashes()});
    assert.deepEqual(report.sourceHashes,report.endSourceHashes,'Source changed during the physical regression');
    console.log('RESULT',JSON.stringify({...report,sourceHashes:undefined,endSourceHashes:undefined}));
    if(process.env.REPORT)await writeFile(process.env.REPORT,JSON.stringify(report,null,2)+'\n');
    assert.equal(c.state,'complete');assert.ok(dualSamples>10&&lift>.035&&m.support&&m.distance<.02&&maxCargoOffset<.03);
    assert.ok(maxPregraspShift<.005,`Bin moved ${maxPregraspShift} m before the commanded pinch`);
    assert.ok(cargo.every(part=>part.supported&&part.speed<.025),'All loose cargo must remain physically supported by its original bin');
    assert.equal(c.sharedAtReady,true);assert.ok(!c.history.some(phase=>['return_fold','return_wide','return','return_center'].includes(phase)),'Per-item completion must not home the robot');
    if(process.env.SNAPSHOT_REPORT)await writeFile(process.env.SNAPSHOT_REPORT,JSON.stringify({scope:'Isolated diagnostic replay fixture after actual G1 full-bin delivery; never restore into a running browser scene',modelHash:report.modelHash,binId:bin.id,sourceSlot:dest.id,physics:physicalSnapshot(data),robots:Object.fromEntries([...agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}])),sourceHashes},null,2)+'\n');
    if(process.env.R1_PLAN_ONLY==='1'){
      const r1=agents.get('r1'),input=metadata.slots.find(s=>s.id==='input-1'),dock=metadata.mobility.docks[input.id];
      const actualBase=bodyPose(data,name('BODY',metadata.mobility.mountName)).position,futureBase=toThree(dock.position);
      const drop=toThree(input.position).map((v,k)=>v-futureBase[k]+actualBase[k]);
      const carried={...bin,sourceSupport:dest.supportBody,start:bodyPose(data,parent).position,drop};
      r1.configureTask({target:carried,arm:process.env.R1_ARM??'left',supportBody:input.supportBody});
      const job={jobId:2,robotId:'r1',target:carried,arm:r1.controller.arm,supportBody:input.supportBody,physics:physicalSnapshot(data),robots:Object.fromEntries([...agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}]))};
      const result=service.plan(job);console.log('R1_PLAN',JSON.stringify({ok:result.ok,reason:result.reason,error:result.error}));assert.ok(result.ok,'R1 must plan pickup of the same physically delivered bin');
    }
  }
}finally{service?.dispose();for(const agent of agents.values())agent.dispose();data.delete();model.delete();}
