// Physical continuation of the measured G1 -> R1 handoff, not browser or
// concurrent acceptance. Replaying a fixture is restricted to this test world.
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
const proofSources=['work/probe-collab-c5-bin.mjs','dist/collab/bin-model.mjs','dist/collab/model.mjs','dist/collab/robot-adapter.mjs','dist/collab/task-plans.mjs','dist/collab/state.mjs','dist/collab/planner.mjs','dist/c5/grasp-controller.mjs'];
const hashes=async()=>Object.fromEntries(await Promise.all(proofSources.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')]))),sourceHashes=await hashes();
const fixture=JSON.parse(await readFile(new URL('work/validation/collab/r1-delivery-replay-development.json',root),'utf8'));
const sources=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${id}/models/${id}-dynamic-scene.xml`,root),'utf8')])));
const paths={r1:'r1/assets/r1.urdf',g1:'g1/assets/g1_29dof_with_dex1_1.urdf',c5:'c5/assets/c5-dex1.urdf'};
const urdfs=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${paths[id]}`,root),'utf8')])));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const built=buildBinCollaborationModel(sources,{binCount:4}),{xml,metadata}=built;
assert.equal(createHash('sha256').update(xml).digest('hex'),fixture.modelHash);
const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
const name=(type,value)=>api.mj_name2id(model,api.mjtObj[`mjOBJ_${type}`].value,value);
const agents=new Map(ids.map(id=>[id,createRobotAgent({api,model,data,id,metadata:robotFrame(metadata.robots[id],metadata.workbench),robot:createPlanningWorkerRobot(urdfs[id],{rootHeight:ROBOT_CONFIGS[id].rootHeight}),targets:[]})]));
let service,step=0;
const advance=()=>{for(const a of agents.values())a.stepControls(.001);api.mj_step(model,data);if(step++%10===0)for(const a of agents.values())a.afterWorldStep();assert.ok(data.qpos.every(Number.isFinite));};
try{
  restorePrivateSnapshot(api,model,data,fixture.physics);
  for(const [id,a]of agents){Object.assign(a.controller,structuredClone(fixture.robots[id].controller));Object.assign(a.dynamics,structuredClone(fixture.robots[id].dynamics));a.controller.running=false;a.dynamics.syncRobot();}
  service=createSharedPlanningService({api,...built,urdfs});
  const agent=agents.get('c5'),c=agent.controller,bin=metadata.bins.find(b=>b.id===fixture.binId),output=metadata.slots.find(s=>s.id==='output-1'),results=[];
  for(const [index,part]of metadata.parts.filter(p=>p.binId===bin.id).entries()){
    const target={...part,start:bodyPose(data,name('BODY',part.body)).position,drop:toThree(output.partSlots[index])},supportBody=`fixture_${output.id}`;
    agent.configureTask({target,arm:'left',supportBody});
    const physics=physicalSnapshot(data),job={jobId:index+1,robotId:'c5',target,arm:'left',supportBody,physics,robots:Object.fromEntries([...agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}]))};
    console.log('C5_TARGET',JSON.stringify({part:part.id,start:c.targets[0].start.toArray(),drop:c.targets[0].drop.toArray()}));
    if(process.env.DIRECT==='1'){
      const before={...c.pose};
      for(const yaw of [0,.2,-.2,.4,-.4])for(let branch=0;branch<9;branch++){
        try{c.plan(c.targets[0],branch,yaw,20*Math.PI/180);console.log('DIRECT_OK',JSON.stringify({yaw,branch}));}
        catch(error){console.log('DIRECT_FAIL',JSON.stringify({yaw,branch,reason:error.message,detail:error.detail}));}
        finally{c.applyPose(before);agent.dynamics.syncRobot();}
      }
      break;
    }
    const result=service.plan(job);assert.deepEqual(physicalSnapshot(data),physics);console.log('C5_PLAN',JSON.stringify(result.ok?{ok:true}:{ok:false,reason:result.reason,error:result.error}));assert.ok(result.ok);
    applyPlanningResult(c,result.payload);let phase=null,dualSamples=0,lift=0,maxPregraspShift=0;
    for(let i=0;i<60000&&c.running;i++){
      advance();if(phase!==c.state){phase=c.state;console.log('C5_PHASE',phase,data.time.toFixed(3));}
      if(i%10===0){const m=agent.measure();if(m.count===2)dualSamples++;lift=Math.max(lift,m.worldPosition.y-target.start[1]);if(['approach','descend'].includes(c.state))maxPregraspShift=Math.max(maxPregraspShift,Math.hypot(...m.worldPosition.toArray().map((v,k)=>v-target.start[k])));}
    }
    const m=agent.measure(),record={partId:part.id,state:c.state,message:c.message,dualSamples,lift,maxPregraspShift,distance:m.distance,supported:m.support,ready:c.sharedAtReady===true,history:[...c.history]};results.push(record);console.log('C5_RESULT',JSON.stringify(record));
    const endSourceHashes=await hashes();assert.deepEqual(sourceHashes,endSourceHashes);
    if(process.env.REPORT)await writeFile(process.env.REPORT,JSON.stringify({scope:'Isolated physical continuation of actual delivered bin, not concurrent/browser acceptance',modelHash:fixture.modelHash,sourceHashes,endSourceHashes,results},null,2)+'\n');
    assert.equal(c.state,'complete');assert.ok(dualSamples>10&&lift>.035&&m.support&&m.distance<.02&&maxPregraspShift<.005);assert.equal(c.sharedAtReady,true);assert.ok(!c.history.includes('return'));
  }
  if(process.env.SNAPSHOT_REPORT)await writeFile(process.env.SNAPSHOT_REPORT,JSON.stringify({scope:'Isolated diagnostic fixture after real C5 packing of both loose parts',modelHash:fixture.modelHash,sourceHashes,binId:bin.id,sourceSlot:fixture.sourceSlot,physics:physicalSnapshot(data),robots:Object.fromEntries([...agents].map(([id,a])=>[id,{controller:plainFields(a.controller,CONTROL_FIELDS),dynamics:plainFields(a.dynamics,MOTOR_FIELDS)}]))},null,2)+'\n');
}finally{service?.dispose();for(const a of agents.values())a.dispose();data.delete();model.delete();}
