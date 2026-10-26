// Controlled offline contact-model experiment. All runs start from the SAME
// actual grasp snapshot; only friction/normal impedance ratio differs. No
// snapshot or parameter mutation is ever sent to the browser/live executor.
import {readFile,writeFile} from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotAgent} from '../dist/collab/robot-adapter.mjs';
import {restorePrivateSnapshot,robotFrame} from '../dist/collab/state.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {createPlanningWorkerRobot} from '../dist/multi/planning-worker-core.mjs';
import {tcpPose} from '../dist/r1/grasp-kinematics.mjs';
const root=new URL('../',import.meta.url),fixture=JSON.parse(await readFile(new URL(process.env.FIXTURE??'work/validation/collab/r1-live-hold-fixture-development.json',root),'utf8'));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname}),reports=[];
for(const ratio of (process.env.RATIOS??'1,10,100').split(',').map(Number)){
  if(!Number.isFinite(ratio)||ratio<1||ratio>1000)throw new Error('Invalid experiment ratio');
  const xml=fixture.model.xml.replace(/ impratio="[^"]*"/g,'').replace('<option ','<option impratio="'+ratio+'" '),vfs=new api.MjVFS();
  const model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
  const agents=Object.keys(fixture.model.urdfs).map(id=>createRobotAgent({api,model,data,id,metadata:robotFrame(fixture.model.metadata.robots[id],fixture.model.metadata.workbench),robot:createPlanningWorkerRobot(fixture.model.urdfs[id],{rootHeight:ROBOT_CONFIGS[id].rootHeight}),targets:[]}));
  try{
    restorePrivateSnapshot(api,model,data,fixture.physics);
    for(const agent of agents){Object.assign(agent.controller,structuredClone(fixture.robots[agent.id].controller));Object.assign(agent.dynamics,structuredClone(fixture.robots[agent.id].dynamics));agent.controller.running=false;agent.dynamics.syncRobot();}
    const r1=agents.find(a=>a.id==='r1');r1.configureTask({target:fixture.target,arm:'right',supportBody:fixture.supportBody});
    const relative=()=>{r1.dynamics.syncRobot();const tool=tcpPose(r1.controller.robot,'right');return r1.controller.objectPose().position.sub(tool.position).applyQuaternion(tool.quaternion.clone().invert());};
    let settled,maximumSlip=0,dual=0,samples=0,firstLoss=null;const trace=[],wall=performance.now(),start=data.time;
    for(let step=0;step<63000;step++){
      for(const agent of agents)agent.stepControls(.001);api.mj_step(model,data);
      if(step%10!==0)continue;
      for(const agent of agents)agent.afterWorldStep();
      const t=data.time-start,m=r1.measure(),p=relative();
      if(t>=3){settled??=p.clone();maximumSlip=Math.max(maximumSlip,p.distanceTo(settled));samples++;if(m.count===2)dual++;else firstLoss??=t;}
      if(step%1000===0)trace.push({time:t,relative:p.toArray(),count:m.count,force:m.force,opening:r1.controller.opening});
    }
    const report={ratio,scope:'63 simulated seconds static hold from one actual grasp; first 3 seconds settling, NOT end-to-end or wall-clock proof',maximumSlip,dualFraction:dual/samples,firstLoss,wallMs:performance.now()-wall,trace};reports.push(report);
    console.log(JSON.stringify({...report,trace:undefined}));
  }finally{for(const agent of agents)agent.dispose();data.delete();model.delete();}
}
if(process.env.REPORT)await writeFile(process.env.REPORT,JSON.stringify({fixtureSourceHashes:fixture.sourceHashes,reports},null,2)+'\n');
