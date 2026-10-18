import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';

// Production orchestration with explicit deterministic station/Worker-result
// fixtures. These contracts do not certify real Worker, browser or physics speed.
const root=new URL('../',import.meta.url),ids=['r1','g1','c5'],checks=[],errors=[];
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)
  ??'work/validation/performance-v4/coordinator-polling-final.json';
const sourcePaths=['dist/multi/coordinator.mjs','work/test-coordinator-polling-perf-v3.mjs'];
const hashes=async()=>Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,
  createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
const sourceHashes=await hashes();

function fixture(options={}){
  let wall=0;const events=[];
  const stations=new Map(ids.map(id=>{
    const {mode='worker',slices=2,deadline=Infinity,declaredBackground=mode==='worker'}=options[id]??{};
    let active=null,generation=0;
    const s={id,ready:true,backgroundPlanning:declaredBackground,config:{label:id,defaultArm:'right'},group:{visible:true},
      controller:{running:false,state:'idle',message:'',backgroundPlanning:declaredBackground},
      data:{time:0,qpos:[0,1,2],qvel:[0,0,0],ctrl:[.1,.2]},objectPosition:[.5,.75,0],
      calls:{start:0,poll:0,candidate:0,stop:0,reset:0,step:0},
      get planning(){return Boolean(active);},
      get planningProgress(){return active&&mode==='worker'?{mode:'worker'}:null;},
      get status(){return {time:this.data.time,state:this.controller.state,message:this.controller.message};},
      get snapshot(){return {...this.status,running:this.controller.running};},
      prepare(){
        active={ticket:++generation,remaining:slices,startedAt:wall,outcome:null};
        this.controller.running=true;this.controller.state='planning';
        this.controller[mode==='worker'?'_backgroundJob':'_planningJob']=active;return generation;
      },
      start(){this.calls.start++;this.prepare();return true;},
      get ticket(){return generation;},
      settle(ticket,outcome={pending:false,done:true,ok:true}){
        if(!active||ticket!==active.ticket)return false;active.outcome=outcome;return true;
      },
      advancePlanning(){
        events.push(`${mode}:${id}`);if(mode==='worker')this.calls.poll++;else this.calls.candidate++;
        assert(active,'Coordinator polled an already completed station');
        let result=active.outcome;
        if(result?.throws)throw new Error(result.throws);
        if(mode==='worker'&&wall-active.startedAt>deadline)result={pending:false,done:true,ok:false,reason:'fixture Worker deadline',error:{type:'timeout'}};
        if(mode==='cooperative'&&--active.remaining<=0)result={pending:false,done:true,ok:true};
        if(!result)return {pending:true,done:false,ok:true};
        active=null;this.controller._backgroundJob=null;this.controller._planningJob=null;
        this.controller.state=result.ok?'approach':'failed';this.controller.running=result.ok;
        this.controller.message=result.reason??'';return result;
      },
      step(dt){this.calls.step++;this.data.time+=dt;},
      stop(){this.calls.stop++;active=null;generation++;this.controller._backgroundJob=null;this.controller._planningJob=null;
        this.controller.running=false;this.controller.state='stopped';},
      reset(){this.calls.reset++;active=null;generation++;this.data.time=0;this.controller.running=false;this.controller.state='idle';},
    };return[id,s];
  }));
  const c=new MultiRobotCoordinator({stations});return{c,stations,events,setWall:value=>{wall=value;}};
}
const physical=stations=>JSON.stringify([...stations].map(([id,s])=>({id,data:s.data,objectPosition:s.objectPosition,step:s.calls.step})));
const candidates=stations=>[...stations.values()].reduce((sum,s)=>sum+s.calls.candidate,0);
function advanceFrozen(c,stations){
  const before=physical(stations),tick=c.tick,work=candidates(stations),result=c.advancePlanning();
  assert.equal(c.tick,tick,'Planning advanced the shared clock');assert.equal(physical(stations),before,'Planning mutated a physical snapshot');
  assert(candidates(stations)-work<=1,'A coordinator frame advanced multiple expensive cooperative candidates');return result;
}
function check(name,fn){try{fn();checks.push({name,pass:true});}catch(error){checks.push({name,pass:false,error:error.message});errors.push({name,message:error.message});}}
function start(c){const result=c.start({task:'pipes'});assert.equal(result.ok,true);assert.equal(result.preparing,true);assert.equal(c.batchNumber,0);}
function failAll(c,stations,result){
  assert.equal(result.ok,false);assert.equal(result.done,true);assert.equal(c.state,'failed');assert.equal(c.planning,false);
  assert.equal(c.running,false);assert.equal(c.batchNumber,0);assert.equal(c.startTick,null);assert.deepEqual(c.startTimes,{});
  assert([...stations.values()].every(s=>s.calls.stop===1&&s.calls.reset===0));
}

for(const failedId of ['g1','c5'])check(`${failedId} Worker failure is consumed in the same frame while the first Worker is pending`,()=>{
  const {c,stations}=fixture();start(c);stations.get(failedId).settle(stations.get(failedId).ticket,{pending:false,done:true,ok:false,reason:`${failedId} failed`});
  const result=advanceFrozen(c,stations);failAll(c,stations,result);assert.match(result.reason,new RegExp(`${failedId} failed`));
  assert.equal(stations.get('r1').calls.poll,1);assert.equal(stations.get(failedId).calls.poll,1);
  for(const s of stations.values())assert.equal(s.settle(s.ticket-1),false,'A late cancelled completion revived a failed batch');
  advanceFrozen(c,stations);assert.equal(c.batchNumber,0);
});
check('A third Worker timeout is polled promptly rather than waiting for the first Worker',()=>{
  const {c,stations,setWall}=fixture({c5:{deadline:100}});start(c);advanceFrozen(c,stations);setWall(101);
  const result=advanceFrozen(c,stations);failAll(c,stations,result);assert.match(result.reason,/deadline/);
  assert.deepEqual([...stations.values()].map(s=>s.calls.poll),[2,2,2]);
});
check('Every pending Worker is polled once per frame without advancing a candidate or physical time',()=>{
  const {c,stations}=fixture();start(c);
  for(let frame=0;frame<5;frame++){assert.equal(advanceFrozen(c,stations).pending,true);assert.throws(()=>c.step(),/规划/);}
  assert.deepEqual([...stations.values()].map(s=>s.calls.poll),[5,5,5]);assert.equal(candidates(stations),0);assert.equal(c.batchNumber,0);
});
check('Simultaneous Worker results commit exactly once at the shared tick and identical start times',()=>{
  const {c,stations}=fixture();for(let i=0;i<7;i++)c.step();start(c);
  for(const s of stations.values())s.settle(s.ticket);
  const result=advanceFrozen(c,stations);assert.equal(result.ok,true);assert.equal(result.done,true);assert.equal(c.state,'running');
  assert.equal(c.batchNumber,1);assert.equal(c.startTick,7);assert.deepEqual(c.startTimes,Object.fromEntries(ids.map(id=>[id,.007])));
  for(let i=0;i<3;i++)advanceFrozen(c,stations);assert.equal(c.batchNumber,1);assert.deepEqual([...stations.values()].map(s=>s.calls.poll),[1,1,1]);
  c.step();assert.equal(c.tick,8);assert([...stations.values()].every(s=>s.calls.step===8&&s.data.time===.008));
});
check('A ready Worker waits at the barrier while later Workers continue being polled',()=>{
  const {c,stations}=fixture();start(c);stations.get('r1').settle(stations.get('r1').ticket);
  for(let i=0;i<3;i++){assert.equal(advanceFrozen(c,stations).pending,true);assert.equal(c.batchNumber,0);}
  assert.deepEqual([...stations.values()].map(s=>s.calls.poll),[1,3,3]);
  for(const id of ['g1','c5'])stations.get(id).settle(stations.get(id).ticket);advanceFrozen(c,stations);assert.equal(c.batchNumber,1);
});
check('Mixed batches poll Workers first and fairly alternate at most one cooperative candidate per frame',()=>{
  const {c,stations,events}=fixture({g1:{mode:'cooperative'},c5:{mode:'cooperative'}});start(c);
  for(let frame=0;frame<4;frame++)assert.equal(advanceFrozen(c,stations).pending,true);
  assert.deepEqual(events,['worker:r1','cooperative:g1','worker:r1','cooperative:c5','worker:r1','cooperative:g1','worker:r1','cooperative:c5']);
  assert.deepEqual([...stations.values()].map(s=>s.calls.candidate),[0,2,2]);assert.equal(c.batchNumber,0);
  stations.get('r1').settle(stations.get('r1').ticket);advanceFrozen(c,stations);assert.equal(c.batchNumber,1);
});
check('A Worker error is consumed before an expensive cooperative candidate in the same mixed frame',()=>{
  const {c,stations,events}=fixture({r1:{mode:'cooperative'},c5:{mode:'cooperative'}});start(c);
  stations.get('g1').settle(stations.get('g1').ticket,{pending:false,done:true,ok:false,reason:'mixed failure'});
  const result=advanceFrozen(c,stations);failAll(c,stations,result);assert.deepEqual(events,['worker:g1']);assert.equal(candidates(stations),0);
});
check('A configured background station with an actual cooperative fallback cannot bypass candidate fairness',()=>{
  const {c,stations,events}=fixture({r1:{mode:'cooperative',declaredBackground:true},g1:{mode:'cooperative',declaredBackground:true}});start(c);
  for(let i=0;i<4;i++)advanceFrozen(c,stations);
  assert.deepEqual(events,['worker:c5','cooperative:r1','worker:c5','cooperative:g1','worker:c5','cooperative:r1','worker:c5','cooperative:g1']);
  assert.equal(stations.get('r1').calls.candidate,2);assert.equal(stations.get('g1').calls.candidate,2);
});
check('Pure cooperative batches use round-robin with exactly one candidate per planning frame',()=>{
  const {c,stations,events}=fixture(Object.fromEntries(ids.map(id=>[id,{mode:'cooperative'}])));start(c);
  for(let i=0;i<6;i++)advanceFrozen(c,stations);
  assert.deepEqual(events,[...ids,...ids].map(id=>`cooperative:${id}`));assert.equal(c.batchNumber,1);assert.equal(c.state,'running');
});
check('A thrown later Worker error follows the same fail-fast batch cancellation path',()=>{
  const {c,stations}=fixture();start(c);stations.get('c5').settle(stations.get('c5').ticket,{throws:'fixture Worker poll exception'});
  const result=advanceFrozen(c,stations);failAll(c,stations,result);assert.match(result.reason,/poll exception/);
});
check('Stop discards old completions, prevents late commit, and permits a clean fresh batch',()=>{
  const {c,stations}=fixture();start(c);advanceFrozen(c,stations);const tickets=new Map([...stations].map(([id,s])=>[id,s.ticket]));c.stop();
  for(const [id,s]of stations)assert.equal(s.settle(tickets.get(id)),false);advanceFrozen(c,stations);assert.equal(c.state,'stopped');assert.equal(c.batchNumber,0);
  start(c);for(const [id,s]of stations){assert.equal(s.settle(tickets.get(id)),false);s.settle(s.ticket);}
  advanceFrozen(c,stations);assert.equal(c.batchNumber,1);assert.equal(c.state,'running');
});
check('Reset invalidates every pending result before selecting and starting a different subset',()=>{
  const {c,stations}=fixture();start(c);const tickets=new Map([...stations].map(([id,s])=>[id,s.ticket]));c.reset();
  for(const [id,s]of stations)assert.equal(s.settle(tickets.get(id)),false);assert.equal(c.select(['c5']).ok,true);start(c);
  stations.get('c5').settle(stations.get('c5').ticket);advanceFrozen(c,stations);assert.equal(c.batchNumber,1);assert.deepEqual(c.runIds,['c5']);assert.deepEqual(c.startTimes,{c5:0});
});
check('A later-item Worker barrier preserves the original batch metadata and physical tick',()=>{
  const {c,stations}=fixture();start(c);for(const s of stations.values())s.settle(s.ticket);advanceFrozen(c,stations);c.step();
  const started={batch:c.batchNumber,tick:c.startTick,times:{...c.startTimes}};stations.get('r1').prepare();stations.get('g1').prepare();
  stations.get('g1').settle(stations.get('g1').ticket);assert.equal(advanceFrozen(c,stations).pending,true);assert.equal(c.tick,1);
  stations.get('r1').settle(stations.get('r1').ticket);advanceFrozen(c,stations);
  assert.deepEqual({batch:c.batchNumber,tick:c.startTick,times:c.startTimes},started);assert.equal(c.tick,1);
});

const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
if(!sourceFrozen)errors.push({name:'source freeze',message:'Coordinator contract sources changed during verification'});
const report={pass:errors.length===0,releaseCohort:'20261018-perf-4',total:checks.length,passed:checks.filter(check=>check.pass).length,checks,errors,sourceHashes,endSourceHashes,sourceFrozen,
  scope:'Exact production coordinator with explicit Worker-result, deadline, cancellation and cooperative-iterator fixtures. No native browser, real Worker execution, physical contacts or FPS is certified.'};
await mkdir(new URL('./',new URL(reportPath,root)),{recursive:true});await writeFile(new URL(reportPath,root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,total:report.total,passed:report.passed,sourceFrozen,errors,report:reportPath},null,2));
if(!report.pass)process.exitCode=1;
