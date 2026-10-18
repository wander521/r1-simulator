import {spawnSync} from 'node:child_process';
import {readFile,mkdir,mkdtemp,readdir,writeFile} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {fileURLToPath} from 'node:url';

const root=new URL('../',import.meta.url),existing=process.argv.find(arg=>arg.startsWith('--from-profile='))?.slice(15);
const benchmarkReport=process.argv.find(arg=>arg.startsWith('--benchmark-report='))?.slice(19)
  ??`work/validation/performance-v2/${existing?'':'rerun/'}profile-experiment-run.json`;
let profileFile=existing;
if(!profileFile){
  const directory=await mkdtemp(join(tmpdir(),'r1-runtime-perf-v2-'));profileFile=join(directory,'runtime.cpuprofile');
  const result=spawnSync(process.execPath,['--cpu-prof',`--cpu-prof-dir=${directory}`,'--cpu-prof-name=runtime.cpuprofile',
    'work/test-multi-runtime-perf-v2.mjs',`--report=${benchmarkReport}`],
    {cwd:fileURLToPath(root),encoding:'utf8',maxBuffer:8*1024*1024});
  if(result.status!==0)throw new Error(`Profile fixture failed (${result.status}): ${result.stderr}\n${result.stdout.slice(0,1000)}`);
}
const profile=JSON.parse(await readFile(profileFile,'utf8'));
const nodes=new Map(profile.nodes.map(node=>[node.id,node])),parents=new Map();
for(const node of profile.nodes)for(const child of node.children??[])parents.set(child,node.id);
const stacks=new Map();
function stack(id){if(!stacks.has(id))stacks.set(id,[nodes.get(id),...(parents.has(id)?stack(parents.get(id)):[])]);return stacks.get(id);}
const summary={},duringStep={},leaves=new Map();let totalUs=0;
function categorize(frames){
  const at=(expression,fn)=>frames.some(node=>expression.test(node.callFrame.url??'')&&fn.test(node.callFrame.functionName??''));
  if(at(/grasp-(?:controller|kinematics)\.mjs|multi\/planning\.mjs|multi\/pipe-controller\.mjs/,/^(?:plan|planCandidates|prepare|prepareItem|advancePlanning|solvePose|validateSegment|route|planG1Pipe)$/))return'route-search-and-safety-preparation';
  if(at(/(?:\/-dynamics|\/r1-dynamics|\/g1-dynamics|\/c5-dynamics|\/r1-safe-dynamics|\/pipe-controller)\.mjs/,/^beforeStep$/))return'motor-command-beforeStep';
  if(at(/\/dist\//,/^(?:syncRobot|syncVisuals|updateMatrixWorld)$/))return'canonical-or-display-FK-sync';
  if(at(/\/dist\//,/^(?:get snapshot|telemetry|measure|wristError)$/))return'live-state-and-telemetry-read';
  if(at(/grasp-controller\.mjs|multi\/robot-runtime\.mjs/,/^step$/))return frames.some(node=>/wasm:/.test(node.callFrame.url??''))?'physical-step-Wasm':'controller-step-other-JS';
  if(at(/\/work\/test-multi-runtime-perf-v2\.mjs/,/^compare$/)||frames.some(node=>/node:.*assert|node:internal\/util\/comparisons/.test(node.callFrame.url??'')))return'fixture-assertions';
  if(frames.some(node=>node.callFrame.functionName==='(garbage collector)'))return'garbage-collection';
  return'initialization-or-unclassified';
}
for(let index=0;index<profile.samples.length;index++){
  const id=profile.samples[index],duration=profile.timeDeltas[index]??0,frames=stack(id),category=categorize(frames),node=nodes.get(id);
  totalUs+=duration;summary[category]=(summary[category]??0)+duration;
  if(frames.some(frame=>/grasp-controller\.mjs|multi\/robot-runtime\.mjs/.test(frame.callFrame.url??'')&&frame.callFrame.functionName==='step'))duringStep[category]=(duringStep[category]??0)+duration;
  const frame=node.callFrame,url=(frame.url??'').replace(fileURLToPath(root),'./'),key=`${frame.functionName} ${url}:${(frame.lineNumber??-1)+1}`;
  const record=leaves.get(key)??{functionName:frame.functionName,url,line:(frame.lineNumber??-1)+1,selfUs:0,samples:0};record.selfUs+=duration;record.samples++;leaves.set(key,record);
}
const productionStepUs=Object.values(duringStep).reduce((sum,value)=>sum+value,0);
const benchmark=JSON.parse(await readFile(new URL(benchmarkReport,root),'utf8'));
const report={pass:benchmark.pass&&benchmark.sourceFrozen,profileFile,profileTotalMs:totalUs/1000,
  bucketMs:Object.fromEntries(Object.entries(summary).map(([key,value])=>[key,value/1000])),
  productionStepSampleShares:Object.fromEntries(Object.entries(duringStep).map(([key,value])=>[key,value/productionStepUs])),
  topSelfFunctions:[...leaves.values()].sort((a,b)=>b.selfUs-a.selfUs).slice(0,35).map(({selfUs,...row})=>({...row,selfMs:selfUs/1000})),
  sourceFrozen:benchmark.sourceFrozen,sourceHashes:benchmark.sourceHashes,endSourceHashes:benchmark.endSourceHashes,
  limitations:'V8 sampling of the paired baseline/test-only view-cache fixture, not a browser profile. Route preparation, explicit display/snapshot probes and strict assertion overhead are separated from step samples; sample shares are estimates, not exact nested timers or FPS. The experiment includes a test-only object-swap wrapper; production sources remain unchanged.'};
const destination=new URL('work/validation/performance-v2/rerun/runtime-cpu-profile.json',root);await mkdir(new URL('.',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));if(!report.pass)process.exitCode=1;
