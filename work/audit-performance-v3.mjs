// Byte-binding audit only. The semantic acceptance verifiers must also pass.
import {readFile,readdir,mkdir,writeFile,realpath} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {resolve,relative,isAbsolute} from 'node:path';
import {fileURLToPath} from 'node:url';

const root=fileURLToPath(new URL('../',import.meta.url)),directory='work/validation/performance-v3';
const preservationBaseline='3949cdc3c156221f3db6268f6191b603ec546245',outputPath=`${directory}/final-hash-audit.json`,rootReal=await realpath(root);
const hash=bytes=>createHash('sha256').update(bytes).digest('hex'),errors=[],bindings=[];
const sourceHashes={},reportHashes={};let sourceBindings=0,reportBindings=0;
const required=['multi-acceptance-final.json','pipes-acceptance-final.json','multi-verifier-final.json','pipe-verifier-final.json'];
const record=value=>value!==null&&typeof value==='object'&&!Array.isArray(value),nonempty=value=>record(value)&&Object.keys(value).length>0;
const safe=path=>{
  if(typeof path!=='string'||!path||isAbsolute(path)||path.includes('\0'))throw new Error(`Invalid binding path: ${path}`);
  const normalized=relative(root,resolve(root,path));
  if(!normalized||normalized==='..'||normalized.startsWith('../'))throw new Error(`Binding outside checkout: ${path}`);
  if(normalized===outputPath)throw new Error('Audit output cannot bind itself, directly or through an input report');
  return normalized;
};
const bindingsInCheckout=async path=>{
  const key=safe(path),actualPath=await realpath(resolve(root,key)),inside=relative(rootReal,actualPath);
  if(!inside||isAbsolute(inside)||inside==='..'||inside.startsWith('../'))throw new Error(`Binding resolves outside checkout: ${path}`);
  return key;
};
let changedProtected=[],currentCommit=null;
try{
const entries=await readdir(resolve(root,directory));
for(const name of required)if(!entries.includes(name))errors.push(`Missing final acceptance: ${name}`);
const paths=entries.filter(name=>name!=='final-hash-audit.json'&&(/-final\.json$|-contracts\.json$/.test(name)||name==='planning-ik-paired.json')).map(name=>`${directory}/${name}`);
for(const name of await readdir(resolve(root,directory,'browser')))if(name.endsWith('.json'))paths.push(`${directory}/browser/${name}`);
for(const path of paths.sort()){
  const bytes=await readFile(resolve(root,path)),report=JSON.parse(bytes);reportHashes[path]=hash(bytes);
  if(!record(report))throw new Error(`Invalid report object: ${path}`);
  const requiredFinal=required.includes(path.slice(directory.length+1));
  if(requiredFinal&&(report.pass!==true||!nonempty(report.sourceHashes)||!nonempty(report.reportHashes)))errors.push(`Missing passing acceptance or nonempty bindings: ${path}`);
  if(Object.hasOwn(report,'pass')&&report.pass!==true)errors.push(`Failed or invalid pass flag: ${path}`);
  if(Object.hasOwn(report,'sourceFrozen')&&report.sourceFrozen!==true)errors.push(`Unfrozen or invalid freeze flag: ${path}`);
  for(const [kind,hashes]of [['sourceHashes',report.sourceHashes],['endSourceHashes',report.endSourceHashes],['reportHashes',report.reportHashes]]){
    if(hashes!==undefined&&!record(hashes)){errors.push(`${path}: invalid ${kind}`);continue;}
    for(const [boundPath,expected]of Object.entries(hashes??{})){
      try{
        const key=await bindingsInCheckout(boundPath),actual=hash(await readFile(resolve(root,key)));
        if(kind==='reportHashes')reportBindings++;else sourceBindings++;
        if(typeof expected!=='string'||!/^[a-f0-9]{64}$/.test(expected)||actual!==expected)errors.push(`${path}: stale ${kind} ${key}`);
        if(kind!=='reportHashes')sourceHashes[key]=actual;
      }catch(error){errors.push(`${path}: ${error.message}`);}
    }
  }
  bindings.push(path);
}
const protectedPaths=['dist/r1','dist/g1','dist/c5','dist/vendor','dist/Rebot_Arm_AGV','work/validation/performance-v2','work/validation/background-planning'];
changedProtected=execFileSync('git',['diff','--name-only',preservationBaseline,'--',...protectedPaths],{cwd:root,encoding:'utf8'}).trim().split('\n').filter(Boolean);
if(changedProtected.length)errors.push(`Unexpected edits to preserved trees: ${changedProtected.join(', ')}`);
currentCommit=execFileSync('git',['rev-parse','HEAD'],{cwd:root,encoding:'utf8'}).trim();
}catch(error){errors.push(error.message);}
const auditSource='work/audit-performance-v3.mjs';sourceHashes[auditSource]=hash(await readFile(resolve(root,auditSource)));
const report={pass:errors.length===0,releaseCohort:'20261018-perf-3',generatedAt:new Date().toISOString(),
  currentCommit,preservationBaseline,sourceBindings,reportBindings,
  reports:bindings,sourceHashes,reportHashes,changedProtected,errors,
  scope:'Current final/contracts/paired/browser report byte bindings only; counts include end-source and repeated bindings, not unique files. Semantic physical acceptance is independently required. Historical v2 and original standalone/reference/vendor tracked trees must match the fixed preservation baseline.'};
await mkdir(resolve(root,directory),{recursive:true});await writeFile(resolve(root,outputPath),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,reports:bindings.length,sourceBindings,reportBindings,errors}));
if(!report.pass)process.exitCode=1;
