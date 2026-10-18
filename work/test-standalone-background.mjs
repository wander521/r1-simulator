import assert from 'node:assert/strict';
import {Worker} from 'node:worker_threads';
import {readFile,readdir,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {R1Dynamics} from '../dist/r1/r1-dynamics.mjs';
import {createStandalonePlanningController} from '../dist/multi/standalone-planning.mjs';
import {createPlanningWorkerTransport} from '../dist/multi/planning-worker-transport.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';
import {loadG1TestRobot} from './g1-test-robot.mjs';
import {loadC5TestRobot} from './c5-test-robot.mjs';

const root=new URL('../',import.meta.url),checks=[],paths=['work/test-standalone-background.mjs','work/background-planning-node-worker.mjs'];
for(const dir of ['dist/multi','dist/multi/visuals'])for(const name of await readdir(new URL(dir+'/',root)))if(name.endsWith('.mjs'))paths.push(dir+'/'+name);
for(const id of ['r1','g1','c5'])paths.push(`dist/${id}/${id}-app.mjs`,`dist/${id}/index.html`,`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,`dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`,id==='r1'?'dist/r1/assets/r1.urdf':id==='g1'?'dist/g1/assets/g1_29dof_with_dex1_1.urdf':'dist/c5/assets/c5-dex1.urdf');
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async p=>[p,createHash('sha256').update(await readFile(new URL(p,root))).digest('hex')])));
const sourceHashes=await hashes(),started=performance.now(),wait=ms=>new Promise(r=>setTimeout(r,ms));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
function worker(){const w=new Worker(new URL('./background-planning-node-worker.mjs',import.meta.url),{type:'module'}),adapter={postMessage:v=>w.postMessage(v),terminate:()=>w.terminate(),onmessage:null,onerror:null,onmessageerror:null};w.on('message',data=>adapter.onmessage?.({data}));w.on('error',error=>adapter.onerror?.({message:error.message,preventDefault(){}}));w.on('messageerror',()=>adapter.onmessageerror?.({}));return adapter;}
const loaders={r1:loadTestRobot,g1:loadG1TestRobot,c5:loadC5TestRobot};
for(const id of ['r1','g1','c5']){
  const config=ROBOT_CONFIGS[id],sourceXml=await readFile(new URL(config.xml),'utf8'),vfs=new api.MjVFS();
  const model=api.MjModel.from_xml_string(sourceXml,vfs),data=new api.MjData(model),robot=await loaders[id]();vfs.delete();
  const Dynamics=config.Dynamics,dynamics=new Dynamics(api,model,data,robot);
  const targets=[{id:'red',body:'red_cube',z:config.lateral,h:.0225,halfX:.0225,halfZ:.0225},{id:'blue',body:'blue_block',z:0,h:.019,halfX:.026,halfZ:.016},{id:'yellow',body:'yellow_cylinder',z:-config.lateral,h:.026,halfX:.02,halfZ:.02}].map(t=>({...t,start:new THREE.Vector3(config.startX,config.tableHeight+t.h,t.z),drop:new THREE.Vector3(config.dropX,config.tableHeight+t.h,t.z),rotation:new THREE.Quaternion()}));
  const c=createStandalonePlanningController(config.Controller,{robotId:id,api,model,data,robot,targets,dynamics,sourceXml,transportFactory:()=>createPlanningWorkerTransport({workerFactory:worker})});
  const arm=config.defaultArm;
  let physicalSteps=0,planningPolls=0,maxPollMs=0,actualDual=false,maxTorqueRatio=0,maxFingerViolation=0;
  const step=()=>{assert(!c.planning);c.step(.001);physicalSteps++;
    for(const j of dynamics.map.values()){
      const q=data.qpos[j.q],force=data.qfrc_actuator[j.d];assert(Number.isFinite(q)&&Number.isFinite(force));assert(Math.abs(force)<=j.effort+1e-6);
      if(j.effort>0)maxTorqueRatio=Math.max(maxTorqueRatio,Math.abs(force)/j.effort);
      if(j.finger){const excess=Math.max(0,j.lower-q,q-j.upper);maxFingerViolation=Math.max(maxFingerViolation,excess);assert(excess<=(id==='r1'?.001:.0002));}
    }
    if(physicalSteps%10===0){const m=c.measure();if(m.count===2&&m.forces.every(f=>f>.015))actualDual=true;}
  };
  async function drain(){if(!c.planning)return;const before={time:data.time,qpos:[...data.qpos],qvel:[...data.qvel],ctrl:[...data.ctrl]},start=performance.now();
    while(c.planning){const t=performance.now(),r=c.advancePlanning();maxPollMs=Math.max(maxPollMs,performance.now()-t);planningPolls++;
      assert.equal(data.time,before.time);assert.deepEqual([...data.qpos],before.qpos);assert.deepEqual([...data.qvel],before.qvel);assert.deepEqual([...data.ctrl],before.ctrl);
      assert(r.ok!==false,r.reason);assert(performance.now()-start<90000);if(c.planning)await wait(5);}
  }
  async function run(name,fn){try{const detail=await fn();checks.push({name:`${id}: ${name}`,pass:true,...detail});}catch(e){checks.push({name:`${id}: ${name}`,pass:false,error:e.message,state:c.state,message:c.message});}}
  try{
    for(let n=0;n<300;n++)step();
    await run('Worker cancellation and reset reject old queued results',async()=>{
      assert(c.start(arm,'blue'));assert(c.planning);c.stop();assert(!c.planning&&!c.running);c.reset();assert.equal(data.time,0);assert.equal(c.state,'idle');
      await wait(10);assert.equal(c.state,'idle');for(let n=0;n<300;n++)step();
    });
    await run('Unreachable target fails asynchronously and reset permits a normal retry',async()=>{
      const target=targets.find(t=>t.id==='blue');c.setObject('blue',new THREE.Vector3(2,config.tableHeight+target.h,0),new THREE.Quaternion());
      assert(c.start(arm,'blue'));let rejection=false;try{await drain();}catch{rejection=true;}assert(rejection);assert.equal(c.state,'failed');assert(!c.running);
      c.reset();for(let n=0;n<300;n++)step();
    });
    await run('Blue sort has real dual contact lift release and stable table support',async()=>{
      assert(c.start(arm,'blue'));assert(c.planning);await drain();
      for(let n=0;n<60000&&c.running;n++){await drain();step();}assert.equal(c.state,'complete',c.message);
      assert(c.evidence.dual&&c.evidence.lift>.035);assert(actualDual);const m=c.measure();assert(m.table&&m.support&&m.distance<.02&&m.speed<.025);
      for(let n=0;n<3000;n++)step();assert(c.measure().table&&c.measure().distance<.02);
      const detail={evidence:{...c.evidence},actualDual,finalDistance:c.measure().distance};c.reset();for(let n=0;n<300;n++)step();return detail;
    });
    await run('Three-object stack completes only after real support stabilization',async()=>{
      assert(c.startStack(arm,{observerYaw:-.8,reducedMotion:true}));await drain();
      for(let n=0;n<180000&&c.running;n++){await drain();step();}assert.equal(c.state,'complete',c.message);
      assert(c.stackResultVerified&&c.stackStableSeconds>=3&&c.stackEvidence.length===3&&c.stackEvidence.every(e=>e.dual&&e.lift>.035));
      for(let n=0;n<3000;n++)step();assert(c.stackCheck().stable);return {stackEvidence:c.stackEvidence,stableSeconds:c.stackStableSeconds};
    });
  }finally{c.disposePlanning();dynamics.dispose?.();data.delete();model.delete();}
  checks.push({name:`${id}: observation`,pass:true,physicalSteps,planningPolls,maxPollMs,maxTorqueRatio,maxFingerViolation});
}
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes),pass=sourceFrozen&&checks.every(c=>c.pass);
const report={pass,releaseCohort:'20261018-perf-4',wallSeconds:(performance.now()-started)/1000,total:checks.length,passed:checks.filter(c=>c.pass).length,checks,sourceFrozen,sourceHashes,endSourceHashes,scope:'Actual original standalone Dynamics + Controller wrapped by the production shared Worker adapter. Real worker_threads changes only fetch/readFile. Same 1 ms physics and physical success gates; not a rendered-browser test.'};
const output=new URL('work/validation/performance-v4/standalone-background-final.json',root);await mkdir(new URL('.',output),{recursive:true});await writeFile(output,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({pass,total:report.total,passed:report.passed,sourceFrozen,failures:checks.filter(c=>!c.pass)},null,2));if(!pass)process.exitCode=1;
