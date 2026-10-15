import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {verifyPipeAcceptance} from './verify-pipe-acceptance.mjs';

// Exercise the real release verifier using in-memory mutations of a completed
// physical report. These are report-rejection tests, not substitute grasp runs.
const root=new URL('../',import.meta.url),reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)
  ??'work/validation/multi-pipes/physical-final.json';
const bytes=await readFile(new URL(reportPath,root)),baseline=JSON.parse(bytes),results=[];
const row=(report,prefix)=>report.results.find(item=>item.name.startsWith(prefix));
const firstObject=(report,id='r1')=>Object.values(report.results[0].stats[id].objects)[0];
async function run(name,mutate,pattern) {
  const report=structuredClone(baseline);if(mutate)mutate(report);
  const result=await verifyPipeAcceptance(report);
  if(!mutate)assert.equal(result.pass,true,`${name}: ${JSON.stringify(result.errors)}`);
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
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const sourceHashes=Object.fromEntries(await Promise.all(['work/test-pipe-acceptance-verifier.mjs','work/verify-pipe-acceptance.mjs'].map(async path=>[path,sha(await readFile(new URL(path,root)))])));
const report={pass:true,scope:'Positive/negative validation of a real saved physical report; not new physics, browser or hardware evidence',
  checks:results.length,results,physicalReport:reportPath,physicalReportSha256:sha(bytes),sourceHashes};
await mkdir(new URL('work/validation/multi-pipes/',root),{recursive:true});
await writeFile(new URL('work/validation/multi-pipes/verifier-final.json',root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
