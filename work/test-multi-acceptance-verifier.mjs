import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve,relative,isAbsolute} from 'node:path';
import {fileURLToPath} from 'node:url';

// Execute the actual verifier with a read-only, in-memory report reader. No
// production source or saved report is edited to exercise rejection paths.
const root=fileURLToPath(new URL('../',import.meta.url));
const verifierPath='work/verify-multi-acceptance.mjs';
const evidenceReports={
  'coordinator-final':'coordinator-final','physical-final':'sort-physical-final','stack-final':'stack-physical-final',
  'r1-gripper-final':'r1-gripper-final','ui-static-final':'ui-static-final','pages-final':'pages-final',
  'responsiveness-final':'responsiveness-contracts-final','framing-final':'framing-final',
  'themes-final':'theme-contracts-final','release-cache-final':'release-cache-final',
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
    if(!target||path!==resolve(root,`work/validation/performance/${evidenceReports[target]}.json`))return bytes;
    const report=JSON.parse(bytes.toString());mutate(report);return Buffer.from(JSON.stringify(report));
  };
  const processFixture={exitCode:0,argv:[]};
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
const historicalSort=JSON.parse(await readFile(resolve(root,'work/validation/multi/physical-final.json'),'utf8'));
await run('Historical physical evidence cannot replace this fresh cooperative run',{target:'physical-final',mutate:r=>{Object.assign(r,structuredClone(historicalSort));},errorPattern:/stale source|missing current-source binding/});
const report={pass:true,checks:results.length,results,verifierSha256:createHash('sha256').update(raw).digest('hex'),
  evidenceReports:Object.fromEntries(Object.entries(evidenceReports).map(([key,value])=>[key,`work/validation/performance/${value}.json`]))};
const outputPath=process.argv.find(arg=>arg.startsWith('--output='))?.slice(9)??null;
if(outputPath){const destination=resolve(root,outputPath);await mkdir(resolve(destination,'..'),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');}
console.log(JSON.stringify(report,null,2));
