import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {verifyPipeAcceptance} from './verify-pipe-acceptance.mjs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';

// Exercise the real release verifier using in-memory mutations of a completed
// physical report. These are report-rejection tests, not substitute grasp runs.
const root=new URL('../',import.meta.url),reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)
  ??'work/validation/performance-v3/pipes-physical-compiled-model-final.json';
const bytes=await readFile(new URL(reportPath,root)),baseline=JSON.parse(bytes),results=[];
const requireBackground=process.argv.includes('--require-background');
const reportDirectory=process.argv.find(arg=>arg.startsWith('--report-dir='))?.slice(13)??reportPath.slice(0,reportPath.lastIndexOf('/'));
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const ownSources=['work/test-pipe-acceptance-verifier.mjs','work/verify-pipe-acceptance.mjs','work/performance-v3-contracts.mjs'];
const hashSources=async()=>Object.fromEntries(await Promise.all(ownSources.map(async path=>[path,sha(await readFile(new URL(path,root)))])));
const sourceHashes=await hashSources();let acceptedBaseline;
const row=(report,prefix)=>report.results.find(item=>item.name.startsWith(prefix));
const firstObject=(report,id='r1')=>Object.values(report.results[0].stats[id].objects)[0];
async function run(name,mutate,pattern,{contractTarget,contractMutate,missing=false}={}) {
  const report=structuredClone(baseline);if(mutate)mutate(report);
  const readEvidence=async(path,...options)=>{
    const target=contractTarget&&resolve(fileURLToPath(root),reportDirectory,contractTarget);
    if(path===target&&missing)throw new Error('ENOENT simulated missing release report');
    const bytes=await readFile(path,...options);if(path!==target)return bytes;
    const contract=JSON.parse(bytes.toString());contractMutate(contract);return Buffer.from(JSON.stringify(contract));
  };
  const result=await verifyPipeAcceptance(report,{requireBackground,reportDirectory,readEvidence});
  if(!mutate&&!contractTarget){assert.equal(result.pass,true,`${name}: ${JSON.stringify(result.errors)}`);acceptedBaseline=result;}
  else {assert.equal(result.pass,false,`${name}: invalid evidence was accepted`);
    assert(result.errors.some(error=>pattern.test(error)),`${name}: wrong rejection: ${JSON.stringify(result.errors)}`);}
  results.push({name,pass:true});
}
await run('Current full seventy-two-item physical evidence is accepted');
await run('Quick mode cannot masquerade as full acceptance',report=>{report.mode='quick';},/quick\/static-only/);
await run('Fewer than seventy-two physical items are rejected',report=>{report.physicallySortedPipes=64;},/seventy-two genuine/);
await run('A required case cannot be omitted',report=>{report.results.pop();report.cases=7;report.expectedCases=7;},/eight complete|required .* case is missing/);
await run('Duplicate cases cannot replace a required scenario',report=>{report.results[2].name=report.results[3].name;},/duplicate|required .* case is missing/);
await run('Independent simultaneous seeds are required',report=>{report.results[1].seed=report.results[0].seed;},/distinct simultaneous/);
await run('Explicitly unfrozen source cannot pass',report=>{report.sourceFrozen=false;},/source was not frozen/);
await run('Old-source SHA cannot pass',report=>{report.sourceHashes['dist/multi/pipe-controller.mjs']='0'.repeat(64);},/stale source/);
await run('The cooperative planning module must be bound to the current physical run',report=>{delete report.sourceHashes['dist/multi/planning.mjs'];},/source binding missing: dist\/multi\/planning/);
if(requireBackground){
  await run('Cooperative execution cannot replace the required real worker run',report=>{report.planningExecution.worker=false;},/real background worker execution/);
  await run('Worker-core source must be bound to the actual physical run',report=>{delete report.sourceHashes['dist/multi/planning-worker-core.mjs'];},/source binding missing: dist\/multi\/planning-worker-core/);
  await run('The exact collision-cache source must be bound to the actual physical run',report=>{delete report.sourceHashes['dist/multi/planning-check-cache.mjs'];},/source binding missing: dist\/multi\/planning-check-cache/);
  await run('A changed worker source cannot reuse saved physical success',report=>{report.sourceHashes['dist/multi/planning-worker-core.mjs']='0'.repeat(64);},/stale source/);
  await run('A planning budget override cannot claim production worker behavior',report=>{report.planningExecution.productionBudgetOverride=true;},/real background worker execution/);
}
await run('Three hundred distinct layouts are required',report=>{report.layout.distinctLayouts.g1=99;},/not reproducible\/distinct/);
await run('Fake grasp without real dual contact is rejected',report=>{firstObject(report).actualDual=false;},/no actual dual-finger force/);
await run('One force-free finger is rejected',report=>{firstObject(report).maxFingerForcesN[0]=0;},/no actual dual-finger force/);
await run('Insufficient true lift is rejected',report=>{firstObject(report).maxLiftM=.01;},/no actual 35 mm physical lift/);
await run('Endpoint success cannot omit actual transport observations',report=>{delete firstObject(report).carry;},/100 Hz transport contact observations absent/);
await run('R1 must alternate actual arms',report=>{report.results[0].stats.r1.sequence.forEach(item=>{item.arm='right';});},/did not alternate automatically/);
await run('G1 must alternate its production arm sequence',report=>{report.results[0].robots.find(robot=>robot.id==='g1').classification.armSequence.fill('right');},/did not alternate automatically/);
await run('Saved item support cannot be missing',report=>{report.results[0].robots[0].classification.items[0].tableSupport=false;},/no actual lift\/contact\/table support/);
await run('All eight pipes require final force-bearing table support',report=>{firstObject(report).supportNormalN=0;},/no actual table support/);
await run('Held pipes cannot count as released',report=>{firstObject(report).final.allFingerNormalN=.1;},/not stably released/);
await run('The production final check must verify every pipe is released',report=>{report.results[0].robots[0].classification.check.states[0].released=false;},/final all-eight support check absent/);
await run('A pipe outside its reserved slot is rejected',report=>{firstObject(report).final.slotErrorM=.03;},/not stably released/);
await run('Wrong-color destination frames are rejected',report=>{const object=firstObject(report);object.frame.colorKey=object.color==='red'?'yellow':'red';},/actual final color\/position absent/);
await run('Non-physical object translation is rejected',report=>{report.results[0].stats.r1.maxTranslationResidualM=.001;},/teleported or rotated/);
await run('Non-physical object rotation is rejected',report=>{report.results[0].stats.g1.maxRotationResidualRad=.001;},/teleported or rotated/);
await run('Task-time object writes are rejected',report=>{report.results[0].stats.r1.objectEditsDuringTask=1;},/teleported or rotated/);
await run('Original R1 finger guard is preserved',report=>{report.results[0].stats.r1.maxFingerLimitViolationM=.00101;},/finger guard exceeded/);
await run('Original Dex1 finger guard is preserved',report=>{report.results[0].stats.c5.maxFingerLimitViolationM=.00021;},/finger guard exceeded/);
await run('Aggregate velocity summaries cannot bypass actual joint traces',report=>{report.results[0].stats.c5.maxActualVelocityRatio=1.015;report.results[0].stats.c5.criticalVelocityExtrema={};},/no actual per-joint trace/);
await run('Actual unsafe robot contact is rejected',report=>{report.results[0].stats.g1.maxIllegalPenetrationM=.0011;},/illegal actual physical penetration/);
await run('A gripper cannot push neighboring pipes',report=>{report.results[0].stats.r1.maxOffTargetFingerForceN=1;},/pushed a non-target body/);
await run('G1 must keep its feet fixed',report=>{report.results[0].stats.g1.maxG1FootDriftM=.001;},/fixed legs\/feet moved/);
await run('A stopped task cannot falsely claim completion',report=>{const robot=row(report,'stop-').robots[0];robot.state='complete';robot.classification.verified=true;},/stopped task falsely reported completion/);
await run('A stopped batch requires reset before restart',report=>{row(report,'stop-').restartAfterStop.ok=true;},/stopped batch accepted a restart/);
await run('A completed batch requires reset before restart',report=>{report.results[0].restartAfterComplete.ok=true;},/completed batch accepted a restart/);
await run('Reset must clear consumed-batch state',report=>{row(report,'pipe-task-reset-').robots[0].classification.started=true;},/reset left a task\/evidence active/);
await run('Unselected stations cannot secretly execute a task',report=>{row(report,'single-g1-').robots.find(robot=>robot.id==='r1').state='complete';},/unselected station received a task/);
await run('An unreachable batch requires raw preparation-safety evidence',report=>{delete row(report,'unreachable-').preparationSafety;},/rejected preparation: unchanged physical snapshot proof/);
await run('A rejected start cannot hide a physical-advance flag',report=>{row(report,'unreachable-').preparationSafety.noPhysicalAdvance=false;},/rejected preparation: unchanged physical snapshot proof/);
await run('A rejected start cannot advance its shared physical tick',report=>{row(report,'unreachable-').preparationSafety.coordinatorAfter.tick++;},/rejected preparation: shared physical tick advanced/);
await run('A rejected start cannot advance an individual physical world',report=>{row(report,'unreachable-').preparationSafety.worlds[0].after.time+=.001;},/rejected preparation\/r1: physical time advanced/);
for(const name of ['qpos','qvel','ctrl'])await run(`A rejected start cannot change its full ${name} array`,report=>{row(report,'unreachable-').preparationSafety.worlds[0].after[name][0]+=.001;},new RegExp(`rejected preparation/r1: unchanged physical ${name} array`));
await run('Empty before/after arrays cannot masquerade as unchanged physical state',report=>{const world=row(report,'unreachable-').preparationSafety.worlds[0];world.before.qpos=[];world.after.qpos=[];},/rejected preparation\/r1: unchanged physical qpos array/);
await run('Equally truncated physical snapshots cannot pass as complete before/after arrays',report=>{const world=row(report,'unreachable-').preparationSafety.worlds[0];world.before.qpos.pop();world.after.qpos.pop();},/rejected preparation\/r1: unchanged physical qpos array/);
await run('A rejected start cannot commit the batch behind a true summary flag',report=>{row(report,'unreachable-').preparationSafety.coordinatorAfter.batchNumber++;},/rejected preparation: batch was committed/);
await run('A rejected start cannot retain a physical start marker',report=>{const safety=row(report,'unreachable-').preparationSafety;safety.coordinatorAfter.startTick=safety.coordinatorBefore.tick;},/rejected preparation: batch was committed/);
await run('A rejected start cannot retain a pending batch',report=>{row(report,'unreachable-').preparationSafety.coordinatorAfter.pendingStart={};},/rejected preparation: pending batch or Worker planning/);
await run('A rejected start must cancel every pending Worker',report=>{row(report,'unreachable-').preparationSafety.worlds[2].after.planning=true;},/rejected preparation\/c5: a Worker, task or classification remained active/);
await run('An unreachable batch cannot change any non-finger arm target',report=>{const targets=row(report,'unreachable-').preparationSafety.worlds[0].after.armTargets;targets[Object.keys(targets)[0]]+=.01;},/rejected preparation\/r1: non-finger arm targets changed/);
await run('An unreachable batch cannot falsely claim completed items',report=>{row(report,'unreachable-').preparationSafety.worlds[0].after.completed=1;},/rejected preparation\/r1: a Worker, task or classification remained active/);
await run('An unreachable batch must retain all three raw world snapshots',report=>{row(report,'unreachable-').preparationSafety.worlds.pop();},/rejected preparation: three independent raw world snapshots/);
for(const contractTarget of ['coordinator-polling-final.json','worker-protocol-final.json','planning-ik-budget-contracts.json']){
  await run(`${contractTarget}: missing v3 report is rejected`,null,/v3 .*required report unavailable/,{contractTarget,missing:true});
  await run(`${contractTarget}: old v2 evidence is rejected`,null,/v3 .*current performance-v3 cohort/,{contractTarget,contractMutate:r=>{r.releaseCohort='20261018-perf-2';}});
  await run(`${contractTarget}: critical named check cannot be omitted`,null,/v3 .*critical named contracts/,{contractTarget,contractMutate:r=>{r.checks.pop();if(r.total!==undefined)r.total=r.passed=r.checks.length;}});
  await run(`${contractTarget}: false named contract cannot be hidden by report pass`,null,/v3 .*critical named contracts/,{contractTarget,contractMutate:r=>{r.checks[0].pass=false;}});
  await run(`${contractTarget}: source must be frozen`,null,/v3 .*frozen source/,{contractTarget,contractMutate:r=>{r.sourceFrozen=false;}});
}
await run('A stale protocol source cannot accompany saved pipe success',null,/v3 workerProtocol: stale source.*planning-worker-protocol/,{contractTarget:'worker-protocol-final.json',contractMutate:r=>{r.sourceHashes['dist/multi/planning-worker-protocol.mjs']='0'.repeat(64);}});
await run('A stale Worker IK context cannot accompany saved pipe success',null,/v3 planningIKBudget: stale source.*planning-ik-context/,{contractTarget:'planning-ik-budget-contracts.json',contractMutate:r=>{r.sourceHashes['dist/multi/planning-ik-context.mjs']='0'.repeat(64);}});
await run('A missing Worker protocol binding is rejected',null,/v3 workerProtocol: source binding missing.*planning-worker-protocol/,{contractTarget:'worker-protocol-final.json',contractMutate:r=>{delete r.sourceHashes['dist/multi/planning-worker-protocol.mjs'];}});
await run('A missing planning-budget binding is rejected',null,/v3 planningIKBudget: source binding missing.*planning-budget/,{contractTarget:'planning-ik-budget-contracts.json',contractMutate:r=>{delete r.sourceHashes['dist/multi/planning-budget.mjs'];}});
await run('Version two protocol cannot stand in for the new message boundary',null,/protocol version 3/,{contractTarget:'worker-protocol-final.json',contractMutate:r=>{r.protocolVersion=2;}});
await run('Absent actual Worker protocol roundtrips cannot pass',null,/four exact real-thread messages/,{contractTarget:'worker-protocol-final.json',contractMutate:r=>{r.measurements=[];}});
await run('Missing paired planning evidence is rejected',null,/v3 planningIKPaired: required report unavailable/,{contractTarget:'planning-ik-paired.json',missing:true});
await run('Stale paired Worker context source cannot reuse physical success',null,/v3 planningIKPaired: stale source.*planning-ik-context/,{contractTarget:'planning-ik-paired.json',contractMutate:r=>{r.sourceHashes['dist/multi/planning-ik-context.mjs']='0'.repeat(64);}});
await run('A mismatched paired safety trace is rejected',null,/safety trace or unchanged physics gate/,{contractTarget:'planning-ik-paired.json',contractMutate:r=>{r.rows[0].optimized.safetyTraceHash='0'.repeat(64);}});
await run('A changed paired physics array is rejected',null,/safety trace or unchanged physics gate/,{contractTarget:'planning-ik-paired.json',contractMutate:r=>{r.rows[0].optimized.unchangedAllPhysicsArrays=false;}});
await run('Changed original candidate steps are rejected',null,/exact original steps/,{contractTarget:'planning-ik-paired.json',contractMutate:r=>{r.rows[0].optimized.steps++;}});
await run('Non-reversed paired trials are rejected',null,/complete reversed-order pairs/,{contractTarget:'planning-ik-paired.json',contractMutate:r=>{r.rows[1].order=['worker-v2','worker-v3'];}});
await run('Non-finite paired timing is rejected',null,/finite positive observations/,{contractTarget:'planning-ik-paired.json',contractMutate:r=>{r.rows[0].optimized.wallMs=null;}});
await run('A substituted paired input archive is rejected',null,/preserved input archive binding/,{contractTarget:'planning-ik-paired.json',contractMutate:r=>{r.compressedInputHash='0'.repeat(64);}});
const historical=JSON.parse(await readFile(new URL('work/validation/multi-pipes/physical-final.json',root),'utf8'));
await run('Historical physical pipe evidence cannot replace the fresh cooperative run',report=>{Object.assign(report,structuredClone(historical));},/stale source|source binding missing/);
assert.deepEqual(await hashSources(),sourceHashes,'Pipe verifier sources changed during positive/negative validation');
const report={pass:true,scope:'Positive/negative validation of a real saved physical report; not new physics, browser or hardware evidence',
  checks:results.length,results,physicalReport:reportPath,physicalReportSha256:sha(bytes),sourceHashes,sourceFrozen:true,reportHashes:acceptedBaseline.reportHashes,
  evidenceReports:Object.keys(acceptedBaseline.reportHashes)};
const outputPath=process.argv.find(arg=>arg.startsWith('--output='))?.slice(9)??'work/validation/performance-v3/pipe-verifier-final.json';
const destination=new URL(outputPath,root);await mkdir(new URL('./',destination),{recursive:true});
await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
