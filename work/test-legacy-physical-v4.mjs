import assert from 'node:assert/strict';
import {execFileSync} from 'node:child_process';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';

const root=new URL('../',import.meta.url),checks=[],sourceHashes={},reportHashes={};
const sha=b=>createHash('sha256').update(b).digest('hex');
for(const path of ['work/test-legacy-physical-v4.mjs','work/verify-acceptance.mjs','work/verify-g1-acceptance.mjs','work/verify-c5-acceptance.mjs'])sourceHashes[path]=sha(await readFile(new URL(path,root)));
function execute(file,args=[]){
  try{return {exitCode:0,result:JSON.parse(execFileSync(process.execPath,[file,...args],{cwd:fileURLToPath(root),encoding:'utf8',maxBuffer:20e6}))};}
  catch(error){if(typeof error.status!=='number')throw error;return {exitCode:error.status,result:JSON.parse(error.stdout)};}
}
const r1=execute('work/verify-acceptance.mjs');
assert.equal(r1.exitCode,0);assert.equal(r1.result.passed,true);assert.equal(r1.result.checksPassed,r1.result.checksTotal);assert(r1.result.checks.every(c=>c.pass));
checks.push({name:'R1 retained physical evidence passes the original read-only verifier',pass:true,result:r1.result});
for(const row of r1.result.checks.find(row=>row.name==='final_physics_source_hashes_match_current').evidence){assert(row.match);sourceHashes[row.path]=row.actual;}
for(const id of ['g1','c5']){
  const path=`work/verify-${id}-acceptance.mjs`,physical=execute(path,['--physics-only']);
  assert.equal(physical.exitCode,0);assert.equal(physical.result.pass,true);assert.equal(physical.result.physicsOnly,true);
  assert(physical.result.historicalUI.length>=2);assert(physical.result.historicalUI.every(row=>[`dist/${id}/${id}-app.mjs`,`dist/${id}/index.html`].includes(row.path)));
  Object.assign(sourceHashes,physical.result.sourceHashes);Object.assign(reportHashes,physical.result.reportHashes);
  checks.push({name:`${id}: unchanged physical model and original scenario evidence remain valid`,pass:true,result:physical.result});
  const strict=execute(path);assert.notEqual(strict.exitCode,0);assert.equal(strict.result.pass,false);
  assert(strict.result.errors.length>0&&strict.result.errors.every(error=>/stale source dist\/(?:g1|c5)\/(?:index.html|(?:g1|c5)-app.mjs)|rendered release cache differs/.test(error)),JSON.stringify(strict.result.errors));
  checks.push({name:`${id}: old UI evidence cannot falsely certify the new app and entry`,pass:true,errors:strict.result.errors});
}
let sourceFrozen=true;for(const [path,hash]of Object.entries(sourceHashes))if(sha(await readFile(new URL(path,root)))!==hash)sourceFrozen=false;
assert(sourceFrozen);const report={pass:true,releaseCohort:'20261018-perf-4',total:checks.length,passed:checks.length,sourceFrozen,sourceHashes,reportHashes,checks,
  scope:'Revalidates retained original physical/model evidence only, with explicit app/index UI exclusions for G1/C5. Negative strict runs must reject old UI bindings. Current Worker execution and browser rendering are separately validated in performance-v4; historical reports are not rewritten.'};
const out=new URL('work/validation/performance-v4/legacy-physical-final.json',root);await mkdir(new URL('.',out),{recursive:true});await writeFile(out,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:true,total:checks.length,sourceFrozen,sourceBindings:Object.keys(sourceHashes).length,reportBindings:Object.keys(reportHashes).length}));
