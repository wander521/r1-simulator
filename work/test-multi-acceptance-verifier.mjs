import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve,relative,isAbsolute} from 'node:path';
import {fileURLToPath} from 'node:url';

// Execute the actual verifier with a read-only, in-memory report reader. No
// production source or saved report is edited to exercise rejection paths.
const root=fileURLToPath(new URL('../',import.meta.url));
const verifierPath='work/verify-multi-acceptance.mjs';
const args=process.argv.slice(2),reportDirectory=args.find(arg=>arg.startsWith('--report-dir='))?.slice(13)??'work/validation/performance-v2',requireBackground=args.includes('--require-background');
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
};
const raw=await readFile(resolve(root,verifierPath),'utf8');
const source=raw.replace(/^import .*;\r?\n/gm,'').replace(/^const root=.*;$/m,'const root=rootOverride;');
const AsyncFunction=Object.getPrototypeOf(async function(){}).constructor;
const execute=new AsyncFunction('readFile','createHash','resolve','relative','isAbsolute','rootOverride','console','process',source);
const results=[];
async function run(name,{target,mutate,errorPattern}={}){
  let output;
  const reader=async(path,...options)=>{
    const bytes=await readFile(path,...options);
    if(!target||path!==resolve(root,`${reportDirectory}/${evidenceReports[target]}.json`))return bytes;
    const report=JSON.parse(bytes.toString());mutate(report);return Buffer.from(JSON.stringify(report));
  };
  const processFixture={exitCode:0,argv:[`--report-dir=${reportDirectory}`,`--physical-suffix=${physicalSuffix}`,...(requireBackground?['--require-background']:[])]};
  await execute(reader,createHash,resolve,relative,isAbsolute,root,{log:line=>{output=JSON.parse(line);}},processFixture);
  assert(output,`${name}: verifier emitted no result`);
  if(!target){assert.equal(output.pass,true,JSON.stringify(output.errors));assert.equal(processFixture.exitCode,0);}
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
const report={pass:true,checks:results.length,results,verifierSha256:createHash('sha256').update(raw).digest('hex'),
  evidenceReports:Object.fromEntries(Object.entries(evidenceReports).map(([key,value])=>[key,`${reportDirectory}/${value}.json`]))};
const outputPath=process.argv.find(arg=>arg.startsWith('--output='))?.slice(9)??null;
if(outputPath){const destination=resolve(root,outputPath);await mkdir(resolve(destination,'..'),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');}
console.log(JSON.stringify(report,null,2));
