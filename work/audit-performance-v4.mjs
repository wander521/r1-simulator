// Byte-binding audit only. The semantic acceptance verifiers must also pass.
import {readFile,readdir,mkdir,writeFile,realpath} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';
import {resolve,relative,isAbsolute} from 'node:path';
import {fileURLToPath} from 'node:url';

const root=fileURLToPath(new URL('../',import.meta.url)),directory='work/validation/performance-v4';
const preservationBaseline='bf09f5dc369945fe9fec5a1df9be7c59d9cf3320',outputPath=`${directory}/final-hash-audit.json`,rootReal=await realpath(root);
const hash=bytes=>createHash('sha256').update(bytes).digest('hex'),errors=[],bindings=[];
const sourceHashes={},reportHashes={};let sourceBindings=0,reportBindings=0;
const browserImages=[];
const required=['multi-acceptance-final.json','pipes-acceptance-final.json','multi-verifier-final.json','pipe-verifier-final.json'];
const record=value=>value!==null&&typeof value==='object'&&!Array.isArray(value),nonempty=value=>record(value)&&Object.keys(value).length>0;
function jpegSize(bytes){
  if(bytes[0]!==255||bytes[1]!==216)throw new Error('Invalid screenshot JPEG');
  for(let i=2;i+4<bytes.length;){
    if(bytes[i++]!==255)continue;while(bytes[i]===255)i++;const marker=bytes[i++];
    if(marker===217||marker===218)break;if(marker>=208&&marker<=215)continue;
    const size=bytes.readUInt16BE(i);if(size<2||i+size>bytes.length)throw new Error('Invalid JPEG segment');
    if([192,193,194,195,197,198,199,201,202,203,205,206,207].includes(marker))return {width:bytes.readUInt16BE(i+5),height:bytes.readUInt16BE(i+3)};
    i+=size;
  }throw new Error('Screenshot dimensions unavailable');
}
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
for(const name of await readdir(resolve(root,directory,'browser')))if(name.endsWith('.jpg')){
  const path=`${directory}/browser/${name}`,bytes=await readFile(resolve(root,path)),size=jpegSize(bytes),sha256=hash(bytes);
  if(name.startsWith('mobile-')&&(size.width!==390||size.height!==844))errors.push(`Wrong phone screenshot dimensions: ${name}`);
  if(name.startsWith('desktop-')&&(size.width!==1280||size.height!==720))errors.push(`Wrong desktop screenshot dimensions: ${name}`);
  browserImages.push({path,...size,sha256});reportHashes[path]=sha256;
}
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
const protectedPaths=['dist/r1','dist/g1','dist/c5','dist/vendor','dist/Rebot_Arm_AGV','work/validation/performance-v2','work/validation/performance-v3','work/validation/background-planning'];
changedProtected=execFileSync('git',['diff','--name-only',preservationBaseline,'--',...protectedPaths],{cwd:root,encoding:'utf8'}).trim().split('\n').filter(Boolean);
changedProtected=changedProtected.filter(path=>!['dist/r1/r1-app.mjs','dist/r1/index.html','dist/g1/g1-app.mjs','dist/g1/index.html','dist/c5/c5-app.mjs','dist/c5/index.html'].includes(path));
if(changedProtected.length)errors.push(`Unexpected edits to preserved trees: ${changedProtected.join(', ')}`);
currentCommit=execFileSync('git',['rev-parse','HEAD'],{cwd:root,encoding:'utf8'}).trim();
}catch(error){errors.push(error.message);}
const auditSource='work/audit-performance-v4.mjs';sourceHashes[auditSource]=hash(await readFile(resolve(root,auditSource)));
const report={pass:errors.length===0,releaseCohort:'20261018-perf-4',generatedAt:new Date().toISOString(),
  currentCommit,preservationBaseline,sourceBindings,reportBindings,
  reports:bindings,sourceHashes,reportHashes,browserImages,changedProtected,errors,
  scope:'Current final/contracts/paired/browser report byte bindings only; counts include end-source and repeated bindings, not unique files. Semantic physical acceptance is independently required. Historical v2/v3 evidence and original physical/reference/vendor tracked files must match the fixed preservation baseline; only the six explicitly authorized standalone app/index files may differ.'};
await mkdir(resolve(root,directory),{recursive:true});await writeFile(resolve(root,outputPath),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,reports:bindings.length,sourceBindings,reportBindings,errors}));
if(!report.pass)process.exitCode=1;
