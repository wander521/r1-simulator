import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve,relative,isAbsolute} from 'node:path';
import {fileURLToPath} from 'node:url';
import {verifyPerformanceV3Contracts} from './performance-v3-contracts.mjs';

// Execute the actual verifier with a read-only, in-memory report reader. No
// production source or saved report is edited to exercise rejection paths.
const root=fileURLToPath(new URL('../',import.meta.url));
const verifierPath='work/verify-multi-acceptance.mjs';
const args=process.argv.slice(2),reportDirectory=args.find(arg=>arg.startsWith('--report-dir='))?.slice(13)??'work/validation/performance-v3',requireBackground=args.includes('--require-background');
const physicalSuffix=args.find(arg=>arg.startsWith('--physical-suffix='))?.slice(18)??'-compiled-model';
const evidenceReports={
  'coordinator-final':'coordinator-final','physical-final':`sort-physical${physicalSuffix}-final`,'stack-final':`stack-physical${physicalSuffix}-final`,
  'r1-gripper-final':'r1-gripper-final','ui-static-final':'ui-static-final','pages-final':'pages-final',
  'responsiveness-final':'responsiveness-contracts-final','framing-final':'framing-final',
  'themes-final':'theme-contracts-final','release-cache-final':'release-cache-final',
  'background-final':'contracts-final',
  'camera-async-final':'camera-feed-async-contracts',
  'planning-cache-final':'planning-check-cache-contracts',
  'camera-queue-final':'camera-feed-queue-contracts',
  'coordinator-polling-final':'coordinator-polling-final',
  'worker-protocol-final':'worker-protocol-final',
  'planning-ik-budget-final':'planning-ik-budget-contracts',
  'planning-ik-paired-final':'planning-ik-paired',
};
const raw=await readFile(resolve(root,verifierPath),'utf8');
const ownSources=['work/test-multi-acceptance-verifier.mjs',verifierPath,'work/performance-v3-contracts.mjs'];
const hashSources=async()=>Object.fromEntries(await Promise.all(ownSources.map(async path=>[path,createHash('sha256').update(await readFile(resolve(root,path))).digest('hex')])));
const sourceHashes=await hashSources();
const source=raw.replace(/^import .*;\r?\n/gm,'').replace(/^const root=.*;$/m,'const root=rootOverride;');
const AsyncFunction=Object.getPrototypeOf(async function(){}).constructor;
const execute=new AsyncFunction('readFile','createHash','resolve','relative','isAbsolute','rootOverride','console','process','verifyPerformanceV3Contracts',source);
const results=[];let acceptedBaseline;
async function run(name,{target,mutate,missing=false,errorPattern}={}){
  let output;
  const reader=async(path,...options)=>{
    if(missing&&path===resolve(root,`${reportDirectory}/${evidenceReports[target]}.json`))throw new Error('ENOENT simulated missing release report');
    const bytes=await readFile(path,...options);
    if(!target||path!==resolve(root,`${reportDirectory}/${evidenceReports[target]}.json`))return bytes;
    const report=JSON.parse(bytes.toString());mutate(report);return Buffer.from(JSON.stringify(report));
  };
  const processFixture={exitCode:0,argv:[`--report-dir=${reportDirectory}`,`--physical-suffix=${physicalSuffix}`,...(requireBackground?['--require-background']:[])]};
  await execute(reader,createHash,resolve,relative,isAbsolute,root,{log:line=>{output=JSON.parse(line);}},processFixture,verifyPerformanceV3Contracts);
  assert(output,`${name}: verifier emitted no result`);
  if(!target){assert.equal(output.pass,true,JSON.stringify(output.errors));assert.equal(processFixture.exitCode,0);acceptedBaseline=output;}
  else {assert.equal(output.pass,false,`${name}: invalid evidence was accepted`);assert.equal(processFixture.exitCode,1);
    assert(output.errors.some(error=>errorPattern.test(error)),`${name}: wrong rejection: ${JSON.stringify(output.errors)}`);}
  results.push({name,pass:true});
}
await run('Current complete release reports are accepted');
await run('Explicitly unfrozen coordinator evidence is rejected',{target:'coordinator-final',mutate:r=>{r.sourceFrozen=false;},errorPattern:/source was not frozen/});
await run('Old-source coordinator SHA cannot pass',{target:'coordinator-final',mutate:r=>{r.sourceHashes['dist/multi/coordinator.mjs']='0'.repeat(64);},errorPattern:/stale source/});
await run('A duplicated physical case cannot replace the required scenario',{target:'physical-final',mutate:r=>{r.results[1].name=r.results[2].name;},errorPattern:/missing\/duplicate\/unexpected cases/});
await run('A completed animation without actual two-finger forces is rejected',{target:'physical-final',mutate:r=>{r.results[1].stats.r1.actualDualForce=false;},errorPattern:/no completed real two-finger grasp/});
await run('A claimed success with insufficient physical object lift is rejected',{target:'physical-final',mutate:r=>{r.results[1].robots[0].evidence.lift=.01;},errorPattern:/not lifted 35 mm/});
await run('R1 finger stroke beyond the explicit legacy 1 mm guard is rejected',{target:'physical-final',mutate:r=>{r.results[1].stats.r1.maxFingerLimitViolation=.00101;},errorPattern:/actual finger range exceeds/});
await run('Dex1 finger stroke beyond 0.2 mm is independently rejected',{target:'physical-final',mutate:r=>{r.results[1].stats.c5.maxFingerLimitViolation=.00021;},errorPattern:/actual finger range exceeds/});
const rejectedSort=report=>report.results.find(row=>row.name==='unreachable-middle-station-aborts-every-attempted-start-without-reset');
const rejectedSortSafety=report=>rejectedSort(report).preparationSafety;
await run('An ordinary rejected batch requires raw preparation-safety evidence',{target:'physical-final',mutate:r=>{delete rejectedSort(r).preparationSafety;},errorPattern:/rejected sort preparation: unchanged physical snapshot proof/});
await run('An ordinary rejected batch cannot hide a physical-advance flag',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).noPhysicalAdvance=false;},errorPattern:/rejected sort preparation: unchanged physical snapshot proof/});
await run('An ordinary rejected batch cannot hide a changed-array flag',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).unchangedPhysicsArrays=false;},errorPattern:/rejected sort preparation: unchanged physical snapshot proof/});
await run('An ordinary rejected batch cannot advance its shared physical tick',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).coordinatorAfter.tick++;},errorPattern:/rejected sort preparation: shared physical tick advanced/});
await run('An ordinary rejected batch cannot advance an individual physical world',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).worlds[0].after.time+=.001;},errorPattern:/rejected sort preparation\/r1: physical time advanced/});
for(const name of ['qpos','qvel','ctrl'])await run(`An ordinary rejected batch cannot change its full ${name} array`,{target:'physical-final',mutate:r=>{rejectedSortSafety(r).worlds[0].after[name][0]+=.001;},errorPattern:new RegExp(`rejected sort preparation/r1: unchanged physical ${name} array`)});
await run('Empty ordinary before/after arrays cannot masquerade as unchanged state',{target:'physical-final',mutate:r=>{const world=rejectedSortSafety(r).worlds[0];world.before.qpos=[];world.after.qpos=[];},errorPattern:/rejected sort preparation\/r1: unchanged physical qpos array/});
await run('Equally truncated ordinary arrays cannot pass as complete physical snapshots',{target:'physical-final',mutate:r=>{const world=rejectedSortSafety(r).worlds[0];world.before.qvel.pop();world.after.qvel.pop();},errorPattern:/rejected sort preparation\/r1: unchanged physical qvel array/});
await run('An ordinary rejected batch cannot hide a false no-commit flag',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).noBatchCommit=false;},errorPattern:/rejected sort preparation: no-batch-commit proof/});
await run('An ordinary rejected batch cannot increment an existing batch number',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).coordinatorAfter.batchNumber++;},errorPattern:/rejected sort preparation: batch was committed/});
await run('An ordinary rejected batch cannot retain a physical start marker',{target:'physical-final',mutate:r=>{const safety=rejectedSortSafety(r);safety.coordinatorAfter.startTick=safety.coordinatorBefore.tick;},errorPattern:/rejected sort preparation: batch was committed/});
await run('An ordinary rejected batch cannot retain committed start times',{target:'physical-final',mutate:r=>{const safety=rejectedSortSafety(r);safety.coordinatorAfter.startTimes={r1:safety.worlds[0].after.time};},errorPattern:/rejected sort preparation: batch was committed/});
await run('An ordinary rejected batch cannot retain a pending batch',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).coordinatorAfter.pendingStart={};},errorPattern:/rejected sort preparation: pending batch or Worker planning/});
await run('An ordinary rejected batch cannot leave coordinator planning active',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).coordinatorAfter.planning=true;},errorPattern:/rejected sort preparation: pending batch or Worker planning/});
await run('An ordinary rejected batch requires an explicit pending-Worker cancellation proof',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).allWorkersCancelled=false;},errorPattern:/rejected sort preparation: all-Worker cancellation proof/});
await run('An ordinary rejected batch must cancel every pending Worker',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).worlds[2].after.planning=true;},errorPattern:/rejected sort preparation\/c5: a Worker or task remained active/});
await run('An ordinary rejected batch must stop every attempted controller',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).worlds[1].after.running=true;},errorPattern:/rejected sort preparation\/g1: a Worker or task remained active/});
await run('An ordinary rejected batch cannot change any non-finger arm target',{target:'physical-final',mutate:r=>{const targets=rejectedSortSafety(r).worlds[0].after.armTargets;targets[Object.keys(targets)[0]]+=.01;},errorPattern:/rejected sort preparation\/r1: non-finger arm targets changed/});
await run('An ordinary rejected batch requires a complete non-finger target map',{target:'physical-final',mutate:r=>{const world=rejectedSortSafety(r).worlds[0],key=Object.keys(world.before.armTargets)[0];delete world.before.armTargets[key];delete world.after.armTargets[key];},errorPattern:/rejected sort preparation\/r1: non-finger arm targets changed/});
await run('An ordinary rejected batch must retain all three raw world snapshots',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).worlds.pop();},errorPattern:/rejected sort preparation: three independent raw world snapshots/});
await run('An ordinary rejected batch cannot hide an object edit behind reset',{target:'physical-final',mutate:r=>{rejectedSortSafety(r).worlds[0].after.objectEditsDuringTask=1;},errorPattern:/rejected sort preparation\/r1: a Worker or task remained active/});
await run('An ordinary rejected batch must separately verify the final reset',{target:'physical-final',mutate:r=>{delete rejectedSort(r).resetVerified;},errorPattern:/rejected sort preparation: final all-station reset/});
await run('Raw preparation safety cannot replace the original reset-state gate',{target:'physical-final',mutate:r=>{rejectedSort(r).robots[0].state='stopped';},errorPattern:/unreachable rollback\/reset left a controller active/});
await run('Publication reports explicitly marked unfrozen are rejected',{target:'pages-final',mutate:r=>{r.sourceFrozen=false;},errorPattern:/explicitly unfrozen report/});
await run('One fake per-item grasp cannot pass a nine-item stack',{target:'stack-final',mutate:r=>{r.results[0].stats.g1.actualDualByObject.yellow=false;},errorPattern:/no real per-item dual forces\/lift/});
await run('Contact existence without force-bearing stack support is rejected',{target:'stack-final',mutate:r=>{r.results[0].stats.c5.supportMinNormalForce.red=0;},errorPattern:/stack support was not force-bearing/});
await run('A stopped stack falsely labelled complete is rejected',{target:'stack-final',mutate:r=>{r.results[1].robots[0].state='complete';},errorPattern:/stopped task falsely completed/});
await run('The new planning module requires an explicit physical-source binding',{target:'physical-final',mutate:r=>{delete r.sourceHashes['dist/multi/planning.mjs'];},errorPattern:/missing current-source binding dist\/multi\/planning/});
await run('The pipe controller requires an explicit sort physical-source binding',{target:'physical-final',mutate:r=>{delete r.sourceHashes['dist/multi/pipe-controller.mjs'];},errorPattern:/missing current-source binding dist\/multi\/pipe-controller/});
await run('The pipe controller requires an explicit stack physical-source binding',{target:'stack-final',mutate:r=>{delete r.sourceHashes['dist/multi/pipe-controller.mjs'];},errorPattern:/missing current-source binding dist\/multi\/pipe-controller/});
await run('An incomplete responsiveness report cannot certify the new scheduler',{target:'responsiveness-final',mutate:r=>{r.passed=19;},errorPattern:/twenty bounded-planning/});
await run('A changed scheduler invalidates saved responsiveness evidence',{target:'responsiveness-final',mutate:r=>{r.sourceHashes['dist/multi/visuals/frame-scheduler.mjs']='0'.repeat(64);},errorPattern:/stale source/});
await run('A missing framing projection cannot replace the original numeric gates',{target:'framing-final',mutate:r=>{r.projectionCases=97;},errorPattern:/98 projections/});
await run('An independent native cache cohort cannot be rewritten by the multi release',{target:'release-cache-final',mutate:r=>{r.releases['r1/index.html']='20261016-responsive-1';},errorPattern:/independent native cohorts/});
await run('An omitted Worker entry cannot pass as the complete publication graph',{target:'pages-final',mutate:r=>{r.workerEntries=[];},errorPattern:/independent worker entry/});
await run('Document imports cannot stand in for the isolated Worker graph',{target:'pages-final',mutate:r=>{r.workerModules=[];},errorPattern:/independent worker entry/});
await run('Missing Worker stale-cache negative fixtures are rejected',{target:'release-cache-final',mutate:r=>{r.negativeFixtures=r.negativeFixtures.filter(row=>row.name!=='unversioned-worker-entry-is-rejected');},errorPattern:/worker stale-cache\/import-map/});
await run('A prior multi cohort cannot replace the Worker release',{target:'release-cache-final',mutate:r=>{r.releases['multi/index.html']='20261016-responsive-1';},errorPattern:/multi worker cohort/});
await run('The exact planning cache source cannot be omitted from the Worker graph',{target:'pages-final',mutate:r=>{r.workerModules=r.workerModules.filter(path=>!path.endsWith('/planning-check-cache.mjs'));},errorPattern:/independent worker entry/});
await run('The camera queue source is required in current UI evidence',{target:'ui-static-final',mutate:r=>{delete r.sourceHashes['dist/multi/visuals/camera-feed-queue.mjs'];},errorPattern:/missing current-source binding.*camera-feed-queue/});
await run('Failed async camera restoration contracts cannot pass release acceptance',{target:'camera-async-final',mutate:r=>{r.results[0].pass=false;},errorPattern:/async camera: complete/});
await run('Missing camera-queue stale-frame protection cannot pass release acceptance',{target:'camera-queue-final',mutate:r=>{r.results=r.results.filter(row=>!row.name.startsWith('Invalidation suppresses'));r.checks=r.results.length;},errorPattern:/camera queue: complete/});
await run('Missing planning-cache invalidation contracts cannot pass release acceptance',{target:'planning-cache-final',mutate:r=>{r.checks=[];},errorPattern:/planning cache: exact/});
for(const target of ['coordinator-polling-final','worker-protocol-final','planning-ik-budget-final']){
  await run(`${target}: required v3 report cannot be missing`,{target,missing:true,errorPattern:/v3 .*required report unavailable/});
  await run(`${target}: v2 cohort cannot replace current evidence`,{target,mutate:r=>{r.releaseCohort='20261018-perf-2';},errorPattern:/v3 .*current performance-v3 cohort/});
  await run(`${target}: omitted critical named contract cannot be hidden by corrected counts`,{target,mutate:r=>{r.checks.pop();if(r.total!==undefined)r.total=r.passed=r.checks.length;},errorPattern:/v3 .*critical named contracts/});
  await run(`${target}: a failed contract cannot pass behind a true report flag`,{target,mutate:r=>{r.checks[0].pass=false;},errorPattern:/v3 .*critical named contracts/});
  await run(`${target}: explicitly unfrozen contracts are rejected`,{target,mutate:r=>{r.sourceFrozen=false;},errorPattern:/v3 .*frozen source/});
}
await run('Old Worker protocol version cannot replace the v3 message boundary',{target:'worker-protocol-final',mutate:r=>{r.protocolVersion=2;},errorPattern:/protocol version 3/});
await run('An unchanged-name protocol report cannot reuse an old protocol source',{target:'worker-protocol-final',mutate:r=>{r.sourceHashes['dist/multi/planning-worker-protocol.mjs']='0'.repeat(64);},errorPattern:/v3 workerProtocol: stale source.*planning-worker-protocol/});
await run('An unchanged-name IK report cannot reuse an old Worker context source',{target:'planning-ik-budget-final',mutate:r=>{r.sourceHashes['dist/multi/planning-ik-context.mjs']='0'.repeat(64);},errorPattern:/v3 planningIKBudget: stale source.*planning-ik-context/});
await run('The Worker registration source requires an explicit protocol binding',{target:'worker-protocol-final',mutate:r=>{delete r.sourceHashes['dist/multi/planning-worker-protocol.mjs'];},errorPattern:/v3 workerProtocol: source binding missing.*planning-worker-protocol/});
await run('The Worker IK scope requires an explicit budget binding',{target:'planning-ik-budget-final',mutate:r=>{delete r.sourceHashes['dist/multi/planning-budget.mjs'];},errorPattern:/v3 planningIKBudget: source binding missing.*planning-budget/});
await run('The actual protocol roundtrip measurement cannot be replaced by an absent echo',{target:'worker-protocol-final',mutate:r=>{r.measurements=[];},errorPattern:/four exact real-thread messages/});
await run('The paired planning evidence is mandatory',{target:'planning-ik-paired-final',missing:true,errorPattern:/v3 planningIKPaired: required report unavailable/});
await run('Old-source paired planning evidence is rejected',{target:'planning-ik-paired-final',mutate:r=>{r.sourceHashes['dist/multi/planning-ik-context.mjs']='0'.repeat(64);},errorPattern:/v3 planningIKPaired: stale source.*planning-ik-context/});
await run('A different safety trace cannot be hidden behind a passing paired summary',{target:'planning-ik-paired-final',mutate:r=>{r.rows[0].optimized.safetyTraceHash='0'.repeat(64);},errorPattern:/safety trace or unchanged physics gate/});
await run('Changed physical arrays cannot pass paired planning acceptance',{target:'planning-ik-paired-final',mutate:r=>{r.rows[0].optimized.unchangedAllPhysicsArrays=false;},errorPattern:/safety trace or unchanged physics gate/});
await run('Original candidate steps cannot be changed by the paired optimization',{target:'planning-ik-paired-final',mutate:r=>{r.rows[0].optimized.steps++;},errorPattern:/exact original steps/});
await run('Repeated execution order cannot replace reversed paired trials',{target:'planning-ik-paired-final',mutate:r=>{r.rows[1].order=['worker-v2','worker-v3'];},errorPattern:/complete reversed-order pairs/});
await run('Non-finite paired timing observations are rejected',{target:'planning-ik-paired-final',mutate:r=>{r.rows[0].optimized.wallMs=null;},errorPattern:/finite positive observations/});
await run('A different paired input archive cannot reuse the timing summary',{target:'planning-ik-paired-final',mutate:r=>{r.compressedInputHash='0'.repeat(64);},errorPattern:/preserved input archive binding/});
if(requireBackground){
  await run('A cooperative physical run cannot masquerade as background execution',{target:'physical-final',mutate:r=>{r.planningExecution.worker=false;},errorPattern:/real background worker execution/});
  await run('A cooperative stack run cannot masquerade as background execution',{target:'stack-final',mutate:r=>{r.planningExecution.worker=false;},errorPattern:/real background worker execution/});
  await run('Worker physical evidence requires the actual shipped core source',{target:'physical-final',mutate:r=>{delete r.sourceHashes['dist/multi/planning-worker-core.mjs'];},errorPattern:/missing current-source binding.*planning-worker-core/});
  await run('Worker physical evidence requires the exact per-job collision cache source',{target:'physical-final',mutate:r=>{delete r.sourceHashes['dist/multi/planning-check-cache.mjs'];},errorPattern:/missing current-source binding.*planning-check-cache/});
  await run('Contracts-only evidence cannot replace actual worker computations',{target:'background-final',mutate:r=>{r.mode='contracts-only';},errorPattern:/complete required real-worker checks/});
  await run('Worker-only exact joint reuse equivalence cannot be omitted',{target:'background-final',mutate:r=>{r.checks=r.checks.filter(row=>row.name!=='Worker-only exact joint reuse preserves default parser behavior and every world matrix');r.checkCount=r.checks.length;},errorPattern:/complete required real-worker checks/});
  await run('Moving derived-state equivalence cannot be omitted from worker acceptance',{target:'background-final',mutate:r=>{r.checks=r.checks.filter(row=>!row.name.startsWith('g1: a moving snapshot'));r.checkCount=r.checks.length;},errorPattern:/complete required real-worker checks/});
  await run('Cross-seed compiled-model reuse equivalence cannot be omitted',{target:'background-final',mutate:r=>{r.checks=r.checks.filter(row=>!row.name.startsWith('g1: reused compiled pipe worlds'));r.checkCount=r.checks.length;},errorPattern:/complete required real-worker checks/});
  await run('A missing actual host heartbeat cannot certify event-loop isolation',{target:'background-final',mutate:r=>{r.measurements=r.measurements.filter(row=>row.scenario!=='g1-pipes-592-first-route');},errorPattern:/actual worker event-loop isolation/});
}
const historicalSort=JSON.parse(await readFile(resolve(root,'work/validation/multi/physical-final.json'),'utf8'));
await run('Historical physical evidence cannot replace this fresh run',{target:'physical-final',mutate:r=>{Object.assign(r,structuredClone(historicalSort));},errorPattern:/stale source|missing current-source binding|real background worker execution/});
assert.deepEqual(await hashSources(),sourceHashes,'Release verifier sources changed during positive/negative validation');
const report={pass:true,checks:results.length,results,sourceHashes,sourceFrozen:true,reportHashes:acceptedBaseline.reportHashes,verifierSha256:createHash('sha256').update(raw).digest('hex'),
  evidenceReports:Object.fromEntries(Object.entries(evidenceReports).map(([key,value])=>[key,`${reportDirectory}/${value}.json`]))};
const outputPath=process.argv.find(arg=>arg.startsWith('--output='))?.slice(9)??null;
if(outputPath){const destination=resolve(root,outputPath);await mkdir(resolve(destination,'..'),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');}
console.log(JSON.stringify(report,null,2));
