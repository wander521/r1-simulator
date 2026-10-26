// Fresh-world integration through REAL separate planner threads. Accelerated
// physical stepping is diagnostic only, never the 300-second browser evidence.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {Worker} from 'node:worker_threads';
import {setImmediate as yieldLoop} from 'node:timers/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {buildBinCollaborationModel} from '../dist/collab/bin-model.mjs';
import {BinCollaborationCore} from '../dist/collab/bin-core.mjs';
import {createSharedPlannerClient} from '../dist/collab/planning-client.mjs';
import {createPipelinePlannerPool} from '../dist/collab/planning-pool.mjs';
import {tcpPose} from '../dist/r1/grasp-kinematics.mjs';
import {physicalSnapshot} from '../dist/collab/state.mjs';

const root=new URL('../',import.meta.url),ids=['r1','g1','c5'];
const sources=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${id}/models/${id}-dynamic-scene.xml`,root),'utf8')])));
const paths={r1:'r1/assets/r1.urdf',g1:'g1/assets/g1_29dof_with_dex1_1.urdf',c5:'c5/assets/c5-dex1.urdf'};
const urdfs=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${paths[id]}`,root),'utf8')])));
const proofFiles=['dist/collab/c5-controller.mjs','dist/collab/bin-core.mjs','dist/collab/bin-evidence.mjs','dist/collab/bin-model.mjs','dist/collab/carrier-kinematics.mjs','dist/collab/robot-adapter.mjs','dist/collab/task-plans.mjs','dist/collab/planner.mjs','dist/collab/planning-guard.mjs','dist/collab/mobile-profile.mjs','dist/collab/resume-route.mjs','dist/collab/pipeline.mjs','dist/collab/planning-pool.mjs','work/probe-collab-bin-core.mjs'];
const hashes=async()=>Object.fromEntries(await Promise.all(proofFiles.map(async p=>[p,createHash('sha256').update(await readFile(new URL(p,root))).digest('hex')]))),sourceHashes=await hashes();
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const model={...buildBinCollaborationModel(sources,{binCount:Number(process.env.BIN_COUNT??1)}),urdfs};
const pool=createPipelinePlannerPool(model,{clientFactory:()=>createSharedPlannerClient(model,{workerFactory:()=>{
  const worker=new Worker(new URL('./collab-node-planner-worker.mjs',import.meta.url));
  const bridge={postMessage:message=>worker.postMessage(message),terminate:()=>worker.terminate()};
  worker.on('message',data=>bridge.onmessage?.({data}));worker.on('error',error=>bridge.onerror?.(error));return bridge;
}})});
const lastJobs=new Map();
const core=new BinCollaborationCore({api,...model,plan:job=>{lastJobs.set(job.robotId,job);return pool.plan(job);},cancelPlan:id=>pool.cancel(id)});
const started=performance.now();let previous='',ticks=0,lastProgress=started,diagnostic=null,lastTrace=-1,capturedHold=false;const holdTrace=[];
const pauseCases=process.env.PAUSE_CASES==='1'?[['g1','posture'],['g1','descend'],['r1','loaded-navigation'],['c5','transfer']]:[],pauseEvidence=[];
let pauseUntil=null;
try{
  core.start({durationSeconds:300});
  while(core.pipeline.state!=='complete'&&performance.now()-started<Number(process.env.TIMEOUT_MS??240000)){
    // Admission time is synthetic in this probe; do not label this as a real
    // five-minute run. Planning threads get event-loop turns between bursts.
    core.schedule(core.data.time);
    for(let i=0;i<20;i++)core.stepPhysics();
    core.schedule(core.data.time);
    await yieldLoop();ticks++;
    if(pauseUntil!==null&&core.data.time>=pauseUntil){core.resume();pauseUntil=null;}
    else if(pauseCases.length&&pauseUntil===null&&core.pipeline.state!=='paused'){
      const [id,state]=pauseCases[0],action=core.actions.get(id),agent=core.agents.get(id);
      const matches=state==='posture'?action?.phase==='posture'&&agent.postureMotion?.elapsed>.5:state==='loaded-navigation'?action?.navigation?.carry&&action.navigation.elapsed>1:agent.controller.state===state&&agent.controller.elapsed>.5;
      if(matches&&!action.pending){pauseCases.shift();pauseEvidence.push({id,state,simTime:core.data.time});core.stop();pauseUntil=core.data.time+1;console.log('CORE_PAUSE',JSON.stringify(pauseEvidence.at(-1)));}
    }
    const r1Action=core.actions.get('r1');
    if(process.env.HOLD_SNAPSHOT&&!capturedHold&&r1Action?.job.binId==='bin-2'&&r1Action.phase==='need-outbound'&&!r1Action.pending){
      await writeFile(process.env.HOLD_SNAPSHOT,JSON.stringify({scope:'Offline hold experiment only; never restored into live browser',sourceHashes,model,physics:physicalSnapshot(core.data),robots:core._context(),target:core._target(r1Action),supportBody:core._support(r1Action)})+'\n');capturedHold=true;
      if(process.env.CAPTURE_ONLY==='1')break;
    }
    if(process.env.HOLD_TRACE&&core.data.time-lastTrace>=.2){
      lastTrace=core.data.time;const action=core.actions.get('r1'),a=core.agents.get('r1'),c=a.controller;
      if(action?.pickStart){
        a.dynamics.syncRobot();const m=a.measure(),tcp=tcpPose(c.robot,c.arm),body=c.objectPose();
        holdTrace.push({time:core.data.time,job:action.job.id,bin:action.job.binId,phase:action.phase,state:c.state,pending:action.pending,opening:c.opening,count:m.count,forces:m.forces,speed:m.speed,
          tcp:tcp.position.toArray(),object:body.position.toArray(),relative:body.position.clone().sub(tcp.position).applyQuaternion(tcp.quaternion.clone().invert()).toArray(),quaternion:body.quaternion.toArray(),
          fingers:c.fingers[c.arm].map(f=>({q:core.data.qpos[f.q],ctrl:core.data.ctrl[f.actuator],force:core.data.actuator_force[f.actuator]})),base:Object.fromEntries(Object.entries(core.baseAxes).map(([key,v])=>[key,core.data.qpos[v.q]]))});
      }
    }
    const status=JSON.stringify(core.snapshot.actions.map(a=>[a.robot,a.type,a.phase,a.pending,a.failed,core.agents.get(a.robot).controller.state]));
    if(status!==previous||performance.now()-lastProgress>10000){console.log('CORE',core.data.time.toFixed(2),status);previous=status;lastProgress=performance.now();}
    const failed=core.snapshot.actions.find(a=>a.failed);
    if(failed){
      const agent=core.agents.get(failed.robot),c=agent.controller;
      diagnostic={failed,message:core.pipeline.robots[failed.robot].error,measurement:agent.measure(),pose:c.pose,history:c.history,searchHint:c.planData?.searchHint,drop:c.planData?.drop};
      if(process.env.FAILURE_SNAPSHOT)await writeFile(process.env.FAILURE_SNAPSHOT,JSON.stringify({sourceHashes,binCount:model.metadata.bins.length,job:lastJobs.get(failed.robot)})+'\n');
      console.log('CORE_FAILURE',JSON.stringify(diagnostic));throw new Error(`${failed.robot}: ${diagnostic.message}`);
    }
  }
  console.log('CORE_RESULT',JSON.stringify(core.snapshot));
  if(process.env.CAPTURE_ONLY==='1')assert.ok(capturedHold,'No actual held snapshot was captured');
  else{
    assert.equal(core.pipeline.state,'complete','Fresh-world integration did not complete before diagnostic timeout');
    assert.equal(core.snapshot.ledger.parts.filter(p=>p.packed).length,model.metadata.parts.length);
    assert.ok(core._atDock(core._homeDock()),'R1 arms homed but base did not return to its initial dock');
    assert.equal(pauseCases.length,0,'Not all requested live pause/resume phases were exercised');
  }
  assert.ok(core.stats.stepsWhilePlanning>0);assert.ok(core.stats.multiRobotMotionSteps>0);
  assert.deepEqual(await hashes(),sourceHashes,'Source changed during this diagnostic run');
}finally{
  if(process.env.HOLD_TRACE)await writeFile(process.env.HOLD_TRACE,JSON.stringify({scope:'Read-only actual R1 hold telemetry, no payload pose writes',sourceHashes,trace:holdTrace},null,2)+'\n');
  if(process.env.REPORT)await writeFile(process.env.REPORT,JSON.stringify({scope:'Fresh-world accelerated integration with real independent worker threads; not browser or 300-wall-second acceptance',sourceHashes,endSourceHashes:await hashes(),wallMs:performance.now()-started,ticks,diagnostic,pauseEvidence,baseAtHome:core._atDock(core._homeDock()),snapshot:core.snapshot},null,2)+'\n');
  pool.dispose();core.dispose();
}
