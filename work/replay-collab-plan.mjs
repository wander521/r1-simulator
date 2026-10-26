// Offline diagnostic only: never writes a snapshot into the live scene.
import {readFile} from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {buildBinCollaborationModel} from '../dist/collab/bin-model.mjs';
import {createSharedPlanningService} from '../dist/collab/planner.mjs';
const root=new URL('../',import.meta.url),ids=['r1','g1','c5'];
const saved=JSON.parse(await readFile(process.argv[2]??new URL('work/validation/collab/bin-core-failed-job.json',root),'utf8'));
const sources=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${id}/models/${id}-dynamic-scene.xml`,root),'utf8')])));
const paths={r1:'r1/assets/r1.urdf',g1:'g1/assets/g1_29dof_with_dex1_1.urdf',c5:'c5/assets/c5-dex1.urdf'};
const urdfs=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(`dist/${paths[id]}`,root),'utf8')])));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const service=createSharedPlanningService({api,...buildBinCollaborationModel(sources,{binCount:saved.binCount}),urdfs});
try{
  const started=performance.now(),result=service.plan(saved.job);
  console.log(JSON.stringify({scope:'Offline replay of a failed private planning snapshot; not execution proof',robot:saved.job.robotId,target:saved.job.target?.id,elapsedMs:performance.now()-started,ok:result.ok,reason:result.reason,detail:result.detail,error:result.error,steps:result.steps},null,2));
  if(!result.ok)process.exitCode=1;
}finally{service.dispose();}
