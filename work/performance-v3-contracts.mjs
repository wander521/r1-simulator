import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve,relative,isAbsolute} from 'node:path';

export const PERFORMANCE_V3_COHORT='20261018-perf-3';
const ids=['r1','g1','c5'];
export const PERFORMANCE_V3_CONTRACTS=[
  {key:'coordinatorPolling',file:'coordinator-polling-final.json',names:[
    ...['g1','c5'].map(id=>`${id} Worker failure is consumed in the same frame while the first Worker is pending`),
    'A third Worker timeout is polled promptly rather than waiting for the first Worker',
    'Every pending Worker is polled once per frame without advancing a candidate or physical time',
    'Simultaneous Worker results commit exactly once at the shared tick and identical start times',
    'A ready Worker waits at the barrier while later Workers continue being polled',
    'Mixed batches poll Workers first and fairly alternate at most one cooperative candidate per frame',
    'A Worker error is consumed before an expensive cooperative candidate in the same mixed frame',
    'A configured background station with an actual cooperative fallback cannot bypass candidate fairness',
    'Pure cooperative batches use round-robin with exactly one candidate per planning frame',
    'A thrown later Worker error follows the same fail-fast batch cancellation path',
    'Stop discards old completions, prevents late commit, and permits a clean fresh batch',
    'Reset invalidates every pending result before selecting and starting a different subset',
    'A later-item Worker barrier preserves the original batch metadata and physical tick',
  ],sources:['work/test-coordinator-polling-perf-v3.mjs','dist/multi/coordinator.mjs']},
  {key:'workerProtocol',file:'worker-protocol-final.json',names:[
    'First plan registers one exact model and later same-model plans omit both XML strings',
    ...['robotId','sourceXml','modelXml','modelPipeSeed','taskMode'].map(field=>`Changing model field ${field} registers an ordered fresh revision`),
    'Current randomized seed remains dynamic without changing the original compiled-model registration',
    'One model is retained and returning to an earlier model requires registration again',
    'Unregistered, duplicate, stale, future and wrong-generation requests cannot run the service',
    'Malformed registration, dynamic model injection, illegal seeds and empty jobs fail closed',
    'A malformed new model never replaces the previously registered exact model',
    'An overlapping message cannot run or replace an active model',
    'Endpoint errors and closed reply channels settle without unhandled promises',
    'Transport emits one atomic registration and reuses it after a successful item',
    'Cancellation and a queued old result cannot bypass fresh-session model registration',
    'Worker crashes, idle crashes, message errors, failed posts and watchdogs all clear registration',
    'Wrong job/generation is ignored while a mismatched active revision or version fails closed',
    'A late result or progress fails the deadline even when no poll ran; its exact boundary remains valid',
    'Malformed active progress/result/type and Worker protocol failures never install a plan',
    'The service keeps exact layout/model identity without serializing large XML cache keys',
    'Four genuine source-bound G1 snapshots survive the actual Worker thread registration protocol with tiny later jobs',
    'Protocol errors leave no unhandled promise rejections',
  ],sources:['work/test-planning-worker-protocol.mjs','work/planning-protocol-echo-worker.mjs','work/background-planning-node-worker.mjs',
    ...['planning-worker-protocol','planning-worker-transport','planning-worker','planning-worker-core','planning-serialization','planning-robot','planning-check-cache','planning-ik-context','planning-budget'].map(name=>`dist/multi/${name}.mjs`),
    'dist/g1/models/g1-dynamic-scene.xml','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/g1-profile.mjs',
    'work/validation/performance-v2/g1-pipe592-snapshots.json.gz']},
  {key:'planningIKBudget',file:'planning-ik-budget-contracts.json',names:[
    ...ids.map(id=>`${id}: standalone default and Worker opt-in preserve every solver pose/FK/result against frozen 3949cdc source`),
    'Worker node context is explicit, idempotent, non-enumerable, and does not alter default robot serialization',
    'Worker FK node reads fall back correctly after a direct joint mutation and match original nonuniform-scale transforms',
    'Budget scopes restore nested callbacks and collision methods after success and typed failure',
    'Budget installation failures restore exact own descriptors, prototype methods, and previous hooks',
    ...ids.map(id=>`${id}: active budget interrupts within a solver, then removes all scoped hooks`),
    ...ids.map(id=>`${id}: production candidate catches preserve budget type and restore pose without physics stepping`),
    'Budget expiry cannot be disguised by a legacy catch that wraps the typed error',
    'Budget expiry cannot be hidden by a legacy catch that returns normally',
    'G1 pipe tilt/direction candidate catches never convert a typed inner-IK budget error into another yielded candidate',
  ],sources:['work/test-planning-ik-budget-perf-v3.mjs',
    ...['planning-budget','planning-ik-context','planning','pipe-controller','planning-robot'].map(name=>`dist/multi/${name}.mjs`),
    ...ids.flatMap(id=>[`dist/${id}/grasp-kinematics.mjs`,`dist/${id}/grasp-controller.mjs`,`work/fixtures/performance-v3/${id}-kinematics-3949cdc.txt`]),
    'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
    'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/vendor/three.module.min.js','dist/vendor/three.core.min.js']},
  {key:'planningIKPaired',file:'planning-ik-paired.json',sources:[
    'work/test-planning-ik-performance-v3.mjs','work/planning-cache-experiment.mjs',
    ...['robot-runtime','planning','pipe-controller','planning-budget','planning-ik-context','planning-check-cache','planning-robot','planning-serialization'].map(name=>`dist/multi/${name}.mjs`),
    ...['planning','pipe-controller','robot-runtime','g1-kinematics'].map(name=>`work/fixtures/performance-v3/${name}-3949cdc.txt`),
    ...['grasp-kinematics','grasp-controller','g1-dynamics','g1-profile'].map(name=>`dist/g1/${name}.mjs`),
    'dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/models/g1-dynamic-scene.xml','dist/vendor/three.module.min.js','dist/vendor/three.core.min.js',
    'dist/r1/mujoco-runtime.mjs','dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm']},
];
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const same=(left,right)=>JSON.stringify(left)===JSON.stringify(right);

/** Mandatory current-source contracts and paired planning evidence shared by
 * both release verifiers. Timing observations are never universal FPS gates.
 * The optional report reader exists for read-only negative fixtures; source
 * bytes are always read directly from the actual checkout, never injected. */
export async function verifyPerformanceV3Contracts({rootPath,reportDirectory='work/validation/performance-v3',readEvidence=readFile}){
  const errors=[],sourceHashes={},reportHashes={},counts={},hashes=new Map();
  const check=(condition,message)=>{if(!condition)errors.push(message);};
  const canonical=path=>{
    if(typeof path!=='string'||!path||isAbsolute(path)||path.includes('\0'))throw new Error(`invalid source path: ${path}`);
    const local=relative(rootPath,resolve(rootPath,path));
    if(!local||local==='..'||local.startsWith('../'))throw new Error(`source outside checkout: ${path}`);return local;
  };
  const currentHash=path=>{const local=canonical(path);if(!hashes.has(local))hashes.set(local,readFile(resolve(rootPath,local)).then(sha));return hashes.get(local);};
  for(const spec of PERFORMANCE_V3_CONTRACTS){
    const label=`v3 ${spec.key}`;
    try{
      const path=canonical(`${reportDirectory}/${spec.file}`),bytes=await readEvidence(resolve(rootPath,path)),report=JSON.parse(bytes.toString());reportHashes[path]=sha(bytes);
      const rows=report?.checks,names=Array.isArray(rows)?rows.map(row=>row?.name):[];
      check(report?.pass===true&&report?.sourceFrozen===true,`${label}: report must pass with frozen source`);
      check(report?.releaseCohort===PERFORMANCE_V3_COHORT,`${label}: current performance-v3 cohort is required`);
      if(spec.names){
        check(Array.isArray(rows)&&rows.length===spec.names.length&&rows.every(row=>row?.pass===true)
          &&new Set(names).size===names.length&&same([...names].sort(),[...spec.names].sort()),`${label}: missing/duplicate/failed critical named contracts or incorrect count`);
        if(spec.key!=='workerProtocol')check(report?.total===spec.names.length&&report?.passed===spec.names.length,`${label}: total/passed counts do not match required contracts`);
      }
      if(report?.errors!==undefined)check(Array.isArray(report.errors)&&report.errors.length===0,`${label}: contract errors were recorded`);
      const bindings=report?.sourceHashes;
      check(bindings&&typeof bindings==='object'&&!Array.isArray(bindings),`${label}: source hash manifest absent`);
      for(const path of spec.sources)check(Object.hasOwn(bindings??{},path),`${label}: source binding missing: ${path}`);
      for(const [path,expected]of Object.entries(bindings??{})){
        try{check(typeof expected==='string'&&/^[a-f0-9]{64}$/.test(expected),`${label}: invalid source SHA: ${path}`);
          const actual=await currentHash(path);sourceHashes[canonical(path)]=actual;check(actual===expected,`${label}: stale source: ${path}`);
        }catch(error){errors.push(`${label}: ${error.message}`);}
      }
      if(spec.key==='workerProtocol'){
        check(report.protocolVersion===3,`${label}: protocol version 3 is required`);
        check(Array.isArray(report.unhandled)&&report.unhandled.length===0,`${label}: unhandled Worker protocol rejection`);
        const measurements=report.measurements?.filter(row=>row.scenario==='four-genuine-g1-message-roundtrips')??[],sample=measurements[0],sizes=sample?.sizes;
        check(measurements.length===1&&sample?.jobCount===4&&Array.isArray(sizes)&&sizes.length===4
          &&sizes.every((row,index)=>row.item===index+1&&row.registration===(index===0)&&Number.isSafeInteger(row.legacyBytes)&&row.legacyBytes>0
            &&Number.isSafeInteger(row.wireBytes)&&row.wireBytes>0&&(index===0||row.wireBytes<row.legacyBytes*.01)),`${label}: four exact real-thread messages and tiny follow-up jobs are required`);
        if(Array.isArray(sizes))check(sample.legacyTotalBytes===sizes.reduce((sum,row)=>sum+row.legacyBytes,0)
          &&sample.currentTotalBytes===sizes.reduce((sum,row)=>sum+row.wireBytes,0)
          &&Math.abs(sample.reductionPercent-(sample.legacyTotalBytes-sample.currentTotalBytes)/sample.legacyTotalBytes*100)<1e-10,`${label}: message-byte totals/reduction do not match observations`);
        check(sample?.originalModelHash===bindings?.['dist/g1/models/g1-dynamic-scene.xml']&&sample?.fixture==='performance-v2/g1-pipe592-snapshots.json.gz'
          &&sample?.scope?.includes('Echo service only')&&sample.scope.includes('not browser FPS'),`${label}: source-bound echo-only measurement scope is missing`);
      }
      if(spec.key==='planningIKBudget')for(const id of ids){
        const row=rows?.find(row=>row.name===spec.names[ids.indexOf(id)]);
        check(row?.detail?.comparisons===(id==='c5'?6:12)&&row.detail.frozenSourceCommit==='3949cdc3c156221f3db6268f6191b603ec546245',`${label}: ${id} frozen-reference solver comparison absent`);
      }
      if(spec.key==='planningIKPaired'){
        const input='work/validation/performance-v2/g1-pipe592-snapshots.json.gz',archiveHash=await currentHash(input);sourceHashes[input]=archiveHash;
        check(report.inputPath===input&&report.inputHash==='eb613c832587d06a5d5083112dfcf476cc1823f426cfb1b7a884b49ed4414be8'
          &&report.compressedInputHash===archiveHash&&archiveHash==='8459ef7f8d11f684c1976454106fdb261aa415500ba9734e839adb97c9f99eb9',`${label}: genuine preserved input archive binding is missing or stale`);
        const frozen={planning:'9cfa8681783ec616ea24aa27b2991e30a1574cea74f84252540301fd96c39810','pipe-controller':'788ae5dc4dd5495816d5cb8b6934b382a11d4731e788cd53ba8577848b2e441e',
          'robot-runtime':'80174f074646f9cff6494424e0b0dc26b98666010c8057ffd11a8781dd37d3ab','g1-kinematics':'9e3ebf66e4a865a2f8e95500d4df44c80433cdd2100e28e7769873b57ea379a1'};
        check(report.baselineCommit==='3949cdc3c156221f3db6268f6191b603ec546245'&&Object.entries(frozen).every(([name,hash])=>bindings?.[`work/fixtures/performance-v3/${name}-3949cdc.txt`]===hash),`${label}: frozen 3949cdc reference binding changed`);
        check(same(report.endSourceHashes,bindings),`${label}: start/end source manifests disagree`);
        const pairs=report.rows??[],steps=[69,137,201,267],selected=['pipe-yellow-1','pipe-red-1','pipe-yellow-2','pipe-red-2'];
        const positive=value=>typeof value==='number'&&Number.isFinite(value)&&value>0,hash=value=>typeof value==='string'&&/^[a-f0-9]{64}$/.test(value);
        check(report.items===4&&report.trials===2&&Array.isArray(pairs)&&pairs.length===8
          &&new Set(pairs.map(row=>`${row.index}/${row.trial}`)).size===8
          &&pairs.every(row=>Number.isInteger(row.index)&&row.index>=0&&row.index<4&&[1,2].includes(row.trial)
            &&row.selected===selected[row.index]&&same(row.order,row.trial===1?['worker-v2','worker-v3']:['worker-v3','worker-v2'])),`${label}: four complete reversed-order pairs are required`);
        let baseline=0,optimized=0;
        for(const row of pairs){
          const before=row.baseline,after=row.optimized;
          check(before?.optimized===false&&after?.optimized===true&&before.steps===steps[row.index]&&after.steps===steps[row.index]
            &&Number.isSafeInteger(before.safetyChecks)&&before.safetyChecks>0&&before.safetyChecks===after.safetyChecks
            &&hash(before.safetyTraceHash)&&before.safetyTraceHash===after.safetyTraceHash
            &&hash(before.planHash)&&before.planHash===after.planHash&&row.exactEverySafetyRequestAndResult===true
            &&before.exactPlanAndCommand===true&&after.exactPlanAndCommand===true
            &&before.unchangedAllPhysicsArrays===true&&after.unchangedAllPhysicsArrays===true,`${label}: exact original steps, plan/command, safety trace or unchanged physics gate failed`);
          check(positive(before?.wallMs)&&positive(after?.wallMs)&&positive(row.speedup)
            &&Math.abs(row.speedup-before.wallMs/after.wallMs)<1e-10,`${label}: paired timings must be finite positive observations`);
          baseline+=before?.wallMs??NaN;optimized+=after?.wallMs??NaN;
        }
        check(positive(report.summary?.baselineMs)&&positive(report.summary?.optimizedMs)&&Math.abs(report.summary.baselineMs-baseline)<1e-6
          &&Math.abs(report.summary.optimizedMs-optimized)<1e-6&&Math.abs(report.summary.speedup-baseline/optimized)<1e-10
          &&Math.abs(report.summary.reductionFraction-(1-optimized/baseline))<1e-10,`${label}: timing summary does not match paired observations`);
        check(report.scope?.includes('Node CPU timings')&&report.scope.includes('not native browser responsiveness/FPS or physical execution')
          &&report.scope.includes('10/30s budgets'),`${label}: bounded Node-only measurement scope is missing`);
      }
      counts[spec.key]=spec.names?(Array.isArray(rows)?rows.length:0):(Array.isArray(report.rows)?report.rows.length:0);
    }catch(error){errors.push(`${label}: required report unavailable or invalid: ${error.message}`);}
  }
  const helper='work/performance-v3-contracts.mjs';sourceHashes[helper]=await currentHash(helper);
  return{pass:errors.length===0,counts,sourceHashes,reportHashes,errors};
}
