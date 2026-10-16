// Verify explicit release evidence against the current checkout. This does not
// run missing tests, accept development probes, or certify unseen browser pixels.
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve,relative,isAbsolute} from 'node:path';
import {fileURLToPath} from 'node:url';

const root=fileURLToPath(new URL('../',import.meta.url));
const errors=[],sourceHashes={},reportHashes={},cache=new Map();
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const check=(condition,message)=>{if(!condition)errors.push(message);};
const finite=(value,label)=>check(typeof value==='number'&&Number.isFinite(value),`${label}: missing/non-finite number`);
const equal=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
const robotIds=['r1','g1','c5'];
const evidenceReports={
  'coordinator-final':'work/validation/performance/coordinator-final.json',
  'physical-final':'work/validation/performance/sort-physical-final.json',
  'stack-final':'work/validation/performance/stack-physical-final.json',
  'r1-gripper-final':'work/validation/performance/r1-gripper-final.json',
  'ui-static-final':'work/validation/performance/ui-static-final.json',
  'pages-final':'work/validation/performance/pages-final.json',
  'responsiveness-final':'work/validation/performance/responsiveness-contracts-final.json',
  'framing-final':'work/validation/performance/framing-final.json',
  'themes-final':'work/validation/performance/theme-contracts-final.json',
  'release-cache-final':'work/validation/performance/release-cache-final.json',
};
const expectedOffsets=[[0,0,2.2],[0,0,0],[0,0,-2.2]];
const fingerTolerances={r1:.001,g1:.0002,c5:.0002};
const coordinatorNames=[
  'initial selection is all three independent robot stations',
  'array stations and production-style snapshot getters use the same contract',
  'selection g1 changes visibility only','selection r1+c5 changes visibility only',
  'selection r1+g1+c5 changes visibility only','empty and unknown selections preserve last valid selection',
  'not-ready station cannot enter selection or start a partial run',
  'focus tracks a selected station without selecting new robots',
  'three starts occur synchronously at one tick before any physics step',
  'only selected r1 receive task starts','only selected g1+c5 receive task starts',
  'only selected r1+g1+c5 receive task starts',
  'busy selection and repeated start cannot mutate an active run',
  'every ready world advances once per shared one-millisecond tick, even if hidden',
  'stop affects this run only and never resets task objects',
  'reset stops running tasks then resets all ready stations, including hidden ones',
  'rejected partial start cancels every attempted controller without teleporting objects',
  'an externally busy selected controller prevents a new partial run',
  'one failed station is never aggregated as an all-robot success',
  'all completed selected controllers aggregate to complete and can be reselected',
];
const physicalNames=[
  'row-layout-and-local-world-frame-isolation',
  'three-way-simultaneous-red','three-way-simultaneous-blue','three-way-simultaneous-yellow',
  'single-selected-G1-leaves-other-tasks-idle',
  'two-selected-R1-C5-leaves-G1-task-idle',
  'stop-three-robots-mid-grasp-without-object-teleport-and-reset-all',
  'unreachable-middle-station-aborts-every-attempted-start-without-reset',
];
const selectedFor={
  'three-way-simultaneous-red':robotIds,'three-way-simultaneous-blue':robotIds,
  'three-way-simultaneous-yellow':robotIds,
  'single-selected-G1-leaves-other-tasks-idle':['g1'],
  'two-selected-R1-C5-leaves-G1-task-idle':['r1','c5'],
};
const stackNames=['three-way-simultaneous-blue-red-yellow-stack',
  'stop-three-way-stack-after-real-lifts-never-reports-completion'];
const physicalSources=[
  'work/test-multi-physical.mjs','work/r1-test-robot.mjs','work/g1-test-robot.mjs','work/c5-test-robot.mjs',
  'dist/multi/coordinator.mjs','dist/multi/robot-runtime.mjs','dist/multi/planning.mjs','dist/multi/pipe-controller.mjs','dist/multi/r1-safe-dynamics.mjs',
  ...robotIds.flatMap(id=>[`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,
    `dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','work/test-r1-dynamics.mjs',
  'work/validation/browser-regressions.json',
];
const uiSources=['work/test-multi-ui.mjs',...['index.html','styles.css','app.mjs','presentation.mjs',
  'workcell.mjs','robot-runtime.mjs','coordinator.mjs','planning.mjs','pipe-controller.mjs',
  'visuals/theme-controller.mjs','visuals/scene-environment.mjs','visuals/camera-feeds.mjs','visuals/frame-scheduler.mjs'].map(file=>`dist/multi/${file}`)];
const pageSources=['work/test-multi-pages.mjs','dist/index.html',...uiSources.filter(path=>path.startsWith('dist/')),
  'dist/multi/r1-safe-dynamics.mjs','dist/vendor/URDFLoader.js','dist/vendor/URDFClasses.js','dist/vendor/three.module.min.js',
  'dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];

function canonical(path){
  if(typeof path!=='string'||!path||isAbsolute(path))throw new Error(`Invalid source path: ${path}`);
  const result=relative(root,resolve(root,path));
  if(!result||result==='..'||result.startsWith('../')||path.includes('\0'))throw new Error(`Source outside checkout: ${path}`);
  return result;
}
async function currentHash(path){
  const key=canonical(path);if(!cache.has(key))cache.set(key,readFile(resolve(root,key)).then(sha));
  return cache.get(key);
}
async function bind(report,label,required){
  const hashes=report?.sourceHashes;
  check(hashes&&typeof hashes==='object'&&!Array.isArray(hashes),`${label}: source hashes absent`);
  if(!hashes||typeof hashes!=='object'||Array.isArray(hashes))return;
  for(const path of required)check(Object.hasOwn(hashes,path),`${label}: missing current-source binding ${path}`);
  for(const [path,expected]of Object.entries(hashes)){
    try{
      check(typeof expected==='string'&&/^[a-f0-9]{64}$/.test(expected),`${label}: malformed SHA-256 for ${path}`);
      const key=canonical(path),actual=await currentHash(path);sourceHashes[key]=actual;
      check(expected===actual,`${label}: stale source ${path}`);
    }catch(error){errors.push(`${label}: ${error.message}`);}
  }
}
async function readReport(name){
  const path=evidenceReports[name];if(!path)throw new Error(`Unknown release evidence: ${name}`);
  const bytes=await readFile(resolve(root,path));
  reportHashes[path]=sha(bytes);return JSON.parse(bytes);
}
function casesExactly(rows,expected,label){
  check(Array.isArray(rows)&&rows.length===expected.length,`${label}: incorrect number of cases`);
  check(equal((rows??[]).map(row=>row.name).sort(),[...expected].sort()),`${label}: missing/duplicate/unexpected cases`);
  check((rows??[]).every(row=>row.pass===true),`${label}: a case did not pass`);
}

const summary={coordinatorChecks:0,physicalCases:0,physicalSorts:0,stackCases:0,physicalStacks:0,physicallyGraspedStackItems:0,
  r1GripperChecks:0,staticUIChecks:0,publicationChecks:0,responsivenessChecks:0,framingChecks:0,themeChecks:0,releaseCacheChecks:0,stackMillisecondSamples:0,stackContactScans:0,
  millisecondSamples:0,contactScans:0,minActualLiftM:null,maxReleasedFrameErrorM:0,
  maxObjectTranslationResidualM:0,maxFingerLimitViolationM:{r1:0,g1:0,c5:0},maxG1C5IllegalPenetrationM:0};

async function main(){
  const [coordinator,physical,stack,gripper,ui,pages,responsiveness,framing,themes,releaseCache]=await Promise.all([
    readReport('coordinator-final'),readReport('physical-final'),readReport('stack-final'),readReport('r1-gripper-final'),readReport('ui-static-final'),readReport('pages-final'),
    readReport('responsiveness-final'),readReport('framing-final'),readReport('themes-final'),readReport('release-cache-final')]);
  for(const [label,report]of [['coordinator',coordinator],['physical',physical],['stack',stack],['r1 gripper',gripper],['ui',ui],
    ['responsiveness',responsiveness],['framing',framing],['themes',themes],['release cache',releaseCache]]){
    check(report.pass===true,`${label}: report did not pass`);
    check(report.sourceFrozen===true,`${label}: source was not frozen during verification`);
  }
  await bind(coordinator,'coordinator',['work/test-multi-coordinator.mjs','dist/multi/coordinator.mjs']);
  check(coordinator.checks===20,'coordinator: expected the final twenty-check suite');
  casesExactly(coordinator.results,coordinatorNames,'coordinator');summary.coordinatorChecks=coordinator.checks;

  await bind(physical,'physical',physicalSources);
  check(physical.cases===8&&physical.physicalSorts===12,'physical: expected eight cases/twelve real sorts');
  casesExactly(physical.results,physicalNames,'physical');
  check(physical.clockStepSeconds===.001,'physical: shared clock is not 1 ms');
  check(equal(physical.renderOffsets,expectedOffsets),'physical: fixed three-robot row changed');
  check(equal(physical.fingerStrokeToleranceM,fingerTolerances),'physical: original R1/Dex1 numerical boundaries changed');
  check(physical.r1LimitPolicy?.includes('default-soft-limit')&&physical.r1LimitPolicy.includes('1 mm')
    &&physical.r1LimitPolicy.includes('not hidden or clamped'),'physical: R1 legacy soft-limit limitation omitted');
  check(physical.collisionScope?.includes('R1 original finger contacts')
    &&physical.collisionScope.includes('do not collide across worlds'),'physical: collision-scope limitation omitted');
  summary.physicalCases=physical.cases;
  const lifts=[];
  for(const row of physical.results??[]){
    const label=`physical/${row.name}`,robots=row.robots??[];
    check(equal(Object.keys(row.stats??{}).sort(),[...robotIds].sort()),`${label}: three worlds not monitored`);
    check(equal(robots.map(robot=>robot.id).sort(),[...robotIds].sort()),`${label}: three actual robot results missing`);
    const times=robots.map(robot=>robot.time);times.forEach(time=>finite(time,`${label}/time`));
    check(Math.max(...times)-Math.min(...times)<1e-10,`${label}: independent worlds left the common clock`);
    const counts=robotIds.map(id=>row.stats?.[id]?.millisecondSamples);
    check(counts.every(count=>Number.isInteger(count)&&count>0)&&new Set(counts).size===1,`${label}: unequal/missing 1 ms sampling`);
    for(const id of robotIds){
      const stats=row.stats?.[id],robot=robots.find(item=>item.id===id);if(!stats||!robot)continue;
      for(const key of ['millisecondSamples','contactSamples','maxObjectTranslationResidual','objectEditsDuringTask',
        'maxTorqueRatio','maxFingerLimitViolation','fingerStrokeToleranceM','maxIllegalPenetration'])finite(stats[key],`${label}/${id}/${key}`);
      check(stats.contactSamples===Math.floor(stats.millisecondSamples/10)&&stats.contactSamples>0,`${label}/${id}: 10 ms real-contact sampling absent`);
      check(stats.objectEditsDuringTask===0&&stats.maxObjectTranslationResidual<1e-5,`${label}/${id}: task object edited or teleported`);
      check(stats.maxTorqueRatio<=1+1e-6,`${label}/${id}: actual effort exceeds the original motor limit`);
      check(stats.fingerStrokeToleranceM===fingerTolerances[id]&&stats.maxFingerLimitViolation<=fingerTolerances[id],`${label}/${id}: actual finger range exceeds its explicit numerical guard`);
      if(id!=='r1'){
        check(stats.maxIllegalPenetration<=.001,`${label}/${id}: robot self/environment penetration exceeds 1 mm`);
        summary.maxG1C5IllegalPenetrationM=Math.max(summary.maxG1C5IllegalPenetrationM,stats.maxIllegalPenetration);
      }
      summary.millisecondSamples+=stats.millisecondSamples;summary.contactScans+=stats.contactSamples;
      summary.maxObjectTranslationResidualM=Math.max(summary.maxObjectTranslationResidualM,stats.maxObjectTranslationResidual);
      summary.maxFingerLimitViolationM[id]=Math.max(summary.maxFingerLimitViolationM[id],stats.maxFingerLimitViolation);
      const selected=selectedFor[row.name];
      if(selected?.includes(id)){
        summary.physicalSorts++;const measurement=robot.measurement;
        check(robot.state==='complete'&&robot.evidence?.dual===true&&stats.actualDualForce===true,`${label}/${id}: no completed real two-finger grasp`);
        finite(robot.evidence?.lift,`${label}/${id}/lift`);check(robot.evidence?.lift>.035,`${label}/${id}: object was not lifted 35 mm`);
        finite(robot.evidence?.maxSpeed,`${label}/${id}/maxSpeed`);check(robot.evidence?.maxSpeed<1,`${label}/${id}: object flung during manipulation`);
        finite(measurement?.distance,`${label}/${id}/frame error`);finite(measurement?.speed,`${label}/${id}/released speed`);
        check(measurement?.count===0&&measurement.table===true&&measurement.support===true
          &&measurement.distance<.02&&measurement.speed<.025,`${label}/${id}: released object is not physically supported inside its color frame`);
        check(equal(measurement?.forces,[0,0]),`${label}/${id}: fingers did not release`);
        for(const phase of ['approach','descend','closing','lift','transfer','place','release','retreat','settling'])
          check(robot.history?.includes(phase)&&stats.states?.includes(phase),`${label}/${id}: missing actual phase ${phase}`);
        lifts.push(robot.evidence.lift);summary.maxReleasedFrameErrorM=Math.max(summary.maxReleasedFrameErrorM,measurement?.distance??Infinity);
      }else if(selected){
        check(robot.state==='idle'&&robot.history?.length===0&&robot.evidence?.dual===false
          &&stats.actualDualForce===false&&equal(stats.states,['idle']),`${label}/${id}: an unselected robot received a task`);
      }
      if(row.name===physicalNames[6]){
        check(stats.actualDualForce===true&&stats.states?.includes('lift')&&stats.states.includes('stopped'),`${label}/${id}: stop did not occur after a genuine grasp/lift`);
        check(robot.state==='idle'&&robot.time===0&&robot.history?.length===0,`${label}/${id}: reset did not restore every station`);
      }
      if(row.name===physicalNames[7])check(robot.state==='idle'&&robot.time===0&&robot.history?.length===0,`${label}/${id}: unreachable rollback/reset left a controller active`);
    }
  }
  check(summary.physicalSorts===12,'physical: twelve selected-robot successes not individually verified');
  summary.minActualLiftM=lifts.length?Math.min(...lifts):null;

  await bind(stack,'stack',['work/test-multi-stack.mjs',...physicalSources.filter(path=>
    !['work/test-multi-physical.mjs','work/test-r1-dynamics.mjs','work/validation/browser-regressions.json'].includes(path)),
    'dist/c5/assets/aubo_C5.urdf','dist/r1/mujoco-runtime.mjs','dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm']);
  check(stack.cases===2&&stack.physicalStacks===3&&stack.physicallyGraspedStackItems===9
    &&stack.initializationError===null,'stack: complete three-robot/nine-item positive and stop cases absent');
  casesExactly(stack.results,stackNames,'stack');
  check(stack.clockStepSeconds===.001&&equal(stack.fingerStrokeToleranceM,fingerTolerances),
    'stack: shared clock or original finger numerical guards changed');
  check(stack.supportScope?.includes('>0.005 N')&&stack.supportScope.includes('>0.015 N')
    &&stack.motionIntegrity?.includes('zero mocap and equality'),'stack: real contact/integrity acceptance scope missing');
  summary.stackCases=stack.cases;
  for(const row of stack.results??[]){
    const label=`stack/${row.name}`,positive=row.name===stackNames[0];
    check(row.start?.ok===true&&equal(row.start.runIds,robotIds)&&Number.isInteger(row.start.startTick),`${label}: all three tasks did not start on one tick`);
    const startTimes=Object.values(row.start?.startTimes??{});startTimes.forEach(time=>finite(time,`${label}/startTime`));
    check(equal(Object.keys(row.start?.startTimes??{}),robotIds)&&new Set(startTimes).size===1,`${label}: start simulation times differ`);
    const robots=row.robots??[];
    check(equal(robots.map(robot=>robot.id).sort(),[...robotIds].sort()),`${label}: three robot results missing`);
    const times=robots.map(robot=>robot.time);times.forEach(time=>finite(time,`${label}/final time`));
    check(Math.max(...times)-Math.min(...times)<1e-10,`${label}: worlds left the shared clock`);
    check(equal(Object.keys(row.stats??{}).sort(),[...robotIds].sort()),`${label}: three physical monitors missing`);
    const counts=robotIds.map(id=>row.stats?.[id]?.millisecondSamples);
    check(counts.every(count=>Number.isInteger(count)&&count>0)&&new Set(counts).size===1,`${label}: uneven/missing 1 ms sampling`);
    check(positive?row.extraStableSeconds===3&&row.physicalStacks===3&&row.physicallyGraspedItems===9:row.postStopPhysicsSeconds===3,
      `${label}: extra physical support/stop window missing`);
    for(const id of robotIds){
      const stats=row.stats?.[id],robot=robots.find(item=>item.id===id);if(!stats||!robot)continue;
      for(const key of ['millisecondSamples','contactSamples','supportSamples','objectEditsDuringTask','maxObjectTranslationResidual',
        'maxTorqueRatio','maxFingerForce','maxFingerLimitViolation','fingerStrokeToleranceM','maxRevoluteLimitViolation',
        'maxActualVelocityRatio','maxIllegalPenetration','maxIndependentStableSeconds','extraStableSamples','maxStackSpeed','maxStackDropError'])finite(stats[key],`${label}/${id}/${key}`);
      check(stats.contactSamples===Math.floor(stats.millisecondSamples/10)&&stats.contactSamples>0,`${label}/${id}: actual 10 ms contact scans absent`);
      check(stats.objectEditsDuringTask===0&&stats.maxObjectTranslationResidual<1e-5,`${label}/${id}: stack task edited or teleported a real object`);
      check(stats.maxTorqueRatio<=1+1e-6&&stats.maxFingerForce<=(id==='r1'?5:20)+1e-6,
        `${label}/${id}: actual original motor effort/finger-force bound exceeded`);
      check(stats.fingerStrokeToleranceM===fingerTolerances[id]&&stats.maxFingerLimitViolation<=fingerTolerances[id]
        &&stats.maxRevoluteLimitViolation<=.001,`${label}/${id}: actual source range exceeded the stated guard`);
      if(id!=='r1'){
        check(stats.maxIllegalPenetration<=.001&&stats.illegalContacts?.length===0,`${label}/${id}: non-exempt actual robot collision`);
        summary.maxG1C5IllegalPenetrationM=Math.max(summary.maxG1C5IllegalPenetrationM,stats.maxIllegalPenetration);
      }
      if(id==='c5')check(stats.maxActualVelocityRatio<=1.0105,`${label}/c5: source motor speed exceeded the numerical allowance`);
      summary.stackMillisecondSamples+=stats.millisecondSamples;summary.stackContactScans+=stats.contactSamples;
      summary.maxFingerLimitViolationM[id]=Math.max(summary.maxFingerLimitViolationM[id],stats.maxFingerLimitViolation);
      summary.maxObjectTranslationResidualM=Math.max(summary.maxObjectTranslationResidualM,stats.maxObjectTranslationResidual);
      if(positive){
        summary.physicalStacks++;
        check(robot.state==='complete'&&robot.running===false&&robot.stackResultVerified===true&&robot.stackStableSeconds>=3,
          `${label}/${id}: stack completion was not physically verified`);
        check(equal(robot.stackEvidence?.map(item=>item.id),['blue','red','yellow']),`${label}/${id}: stack order changed`);
        check(stats.supportSamples>=300&&stats.maxIndependentStableSeconds>=3&&stats.extraStableSamples===300
          &&stats.maxStackSpeed<.0175&&stats.maxStackDropError<.012,`${label}/${id}: independent released-stack stability missing`);
        for(const item of robot.stackEvidence??[]){
          summary.physicallyGraspedStackItems++;
          check(item.dual===true&&stats.actualDualByObject?.[item.id]===true&&item.lift>.035,
            `${label}/${id}/${item.id}: no real per-item dual forces/lift`);
          const force=stats.supportMinNormalForce?.[item.id];finite(force,`${label}/${id}/${item.id}/support force`);
          check(force>.005,`${label}/${id}/${item.id}: stack support was not force-bearing`);
        }
        check(robot.stackCheck?.stable===true&&equal(robot.stackCheck.states?.map(item=>item.id),['blue','red','yellow']),
          `${label}/${id}: final three-level stack is not stable`);
        for(const item of robot.stackCheck?.states??[])check(item.supported===true&&Number.isFinite(item.error)&&item.error<.012
          &&Number.isFinite(item.speed)&&item.speed<.0175,`${label}/${id}/${item.id}: final released support/speed/drop error failed`);
      }else{
        check(robot.state==='stopped'&&robot.running===false&&robot.stackResultVerified===false
          &&!robot.history?.includes('complete'),`${label}/${id}: stopped task falsely completed`);
        check(stats.actualDualByObject?.blue===true&&stats.transitions?.some(transition=>transition.key==='0/blue/lift')
          &&stats.transitions.some(transition=>transition.key.endsWith('/stopped')),`${label}/${id}: stop was not exercised after a real grasp/lift`);
      }
    }
  }
  check(summary.physicalStacks===3&&summary.physicallyGraspedStackItems===9,'stack: three complete physical stacks/nine genuine grasps not individually verified');

  await bind(gripper,'r1 gripper',['work/test-multi-r1-gripper.mjs','dist/multi/r1-safe-dynamics.mjs',
    'dist/multi/robot-runtime.mjs','dist/multi/planning.mjs','dist/r1/r1-dynamics.mjs','dist/r1/grasp-controller.mjs',
    'dist/r1/assets/r1.urdf','dist/r1/models/r1-dynamic-scene.xml','work/r1-test-robot.mjs']);
  check(gripper.checks===5,'r1 gripper: five smoothing/reset/motion-integrity cases missing');
  casesExactly(gripper.results,[
    'All four finger source velocities are 0.25 m/s and conservative commands use 40%',
    'One-shot close/open requests persist while independent ramp history advances through real motors',
    'Rapid reversals preserve the command slew, actual state integration and original force/range bounds',
    'Reset clears both desired/ramped histories and cannot resume a stale closing request',
    'Batch stop still retains the R1 aperture and arm targets',
  ],'r1 gripper');
  check(equal(gripper.profile,{sourceVelocityMps:.25,commandFraction:.4,commandVelocityMps:.1}),
    'r1 gripper: original source velocity or conservative command profile changed');
  for(const key of ['millisecondSamples','maxCommandVelocity','maxActualVelocity','maxActualFingerRangeExcess','maxEffortRatio'])finite(gripper.stats?.[key],`r1 gripper/${key}`);
  check(gripper.stats?.millisecondSamples>=3000&&gripper.stats.maxCommandVelocity<=.1000000001
    &&gripper.stats.maxActualFingerRangeExcess<=.001&&gripper.stats.maxEffortRatio<=1+1e-6,
    'r1 gripper: real smoothed motor commands exceed the unchanged range/force guard');
  summary.r1GripperChecks=gripper.checks;

  await bind(ui,'ui',uiSources);
  check(ui.total===28&&ui.passed===28&&ui.checks?.length===28&&ui.checks.every(result=>result.pass===true)
    &&new Set(ui.checks.map(result=>result.name)).size===28&&ui.errors?.length===0,'ui: final twenty-eight static checks incomplete');
  check(ui.scope?.includes('Node-only')&&ui.scope.includes('No browser rendering')&&ui.limitations?.length>=3,
    'ui: static evidence must not claim browser rendering or native input certification');
  for(const id of robotIds)check(Array.isArray(ui.stateCoverage?.[id])&&ui.stateCoverage[id].includes('complete')
    &&ui.stateCoverage[id].includes('failed')&&ui.stateCoverage[id].includes('stopped'),`ui: ${id} terminal-state labels not covered`);
  summary.staticUIChecks=ui.total;

  await bind(pages,'publication',pageSources);
  check(pages.pass===true&&pages.sourceFrozen!==false,'publication: failed or explicitly unfrozen report');
  check(pages.checks===16&&pages.results?.length===16&&pages.results.every(result=>result.pass===true)
    &&new Set(pages.results.map(result=>result.name)).size===16,'publication: final sixteen static checks incomplete');
  const freeze=pages.results?.find(result=>result.name==='All inspected source and asset bytes stayed frozen during the check');
  check(freeze?.pass===true&&freeze.files===pages.checkedFiles&&pages.checkedFiles>=137,'publication: source/asset freeze not demonstrated');
  check(equal(pages.bases,['http://127.0.0.1:4173/','https://wander521.github.io/r1-simulator/']),
    'publication: local and GitHub project-relative bases not checked');
  check(equal(pages.meshCounts,{r1:34,g1:38,c5:19})&&pages.moduleCount>=30&&pages.wasmBytes===10125422,
    'publication: complete original robot/module/WASM paths not covered');
  check(pages.scope?.includes('Static publication')&&pages.scope.includes('No browser'),
    'publication: static paths must not be misrepresented as browser/runtime acceptance');
  summary.publicationChecks=pages.checks;

  await bind(responsiveness,'responsiveness',['work/test-multi-responsiveness.mjs','dist/multi/coordinator.mjs',
    'dist/multi/app.mjs','dist/multi/visuals/frame-scheduler.mjs']);
  check(responsiveness.total===20&&responsiveness.passed===20&&responsiveness.checks?.length===20
    &&responsiveness.checks.every(row=>row.pass===true)&&new Set(responsiveness.checks.map(row=>row.name)).size===20
    &&responsiveness.errors?.length===0,'responsiveness: twenty bounded-planning, cancellation, clock, feed and camera contracts incomplete');
  check(responsiveness.scope?.includes('explicit fake stations')&&responsiveness.scope.includes('No real contacts'),
    'responsiveness: declared fixtures must not claim real physical or native-browser success');
  summary.responsivenessChecks=responsiveness.total;

  await bind(framing,'framing',['work/test-multi-framing.mjs','dist/multi/app.mjs','dist/multi/workcell.mjs',
    'dist/multi/visuals/frame-scheduler.mjs',evidenceReports['physical-final'],evidenceReports['stack-final'],
    'work/validation/performance/pipes-physical-final.json']);
  check(framing.total===113&&framing.passed===113&&framing.checks?.length===113&&framing.checks.every(row=>row.pass===true)
    &&framing.errors?.length===0&&framing.projectionCases===98&&framing.idleProjectionCases===49&&framing.motionProjectionCases===49,
    'framing: complete original numeric gates, 98 projections and camera ownership checks absent');
  check(framing.comparisons?.historical?.minimumAverageHeightRatio===1.15
    &&framing.comparisons.historical.magnifications?.length===4
    &&framing.comparisons.historical.magnifications.every(row=>row.averageHeightRatio>=1.15),
    'framing: original fifteen-percent historical camera gate changed');
  summary.framingChecks=framing.total;

  await bind(themes,'themes',['work/test-multi-themes.mjs','dist/multi/app.mjs','dist/multi/index.html','dist/multi/workcell.mjs',
    'dist/multi/visuals/theme-controller.mjs','dist/multi/visuals/frame-scheduler.mjs']);
  check(themes.total===15&&themes.passed===15&&themes.checks?.length===15&&themes.checks.every(row=>row.pass===true)
    &&themes.errors?.length===0,'themes: complete actual theme-controller contracts absent');
  summary.themeChecks=themes.total;

  await bind(releaseCache,'release cache',['work/test-release-cache.mjs','dist/multi/index.html','dist/multi/app.mjs',
    'dist/multi/coordinator.mjs','dist/multi/robot-runtime.mjs','dist/multi/planning.mjs','dist/multi/visuals/frame-scheduler.mjs']);
  check(releaseCache.checks===13&&releaseCache.results?.length===13&&releaseCache.results.every(row=>row.pass===true)
    &&releaseCache.graphs?.length===6&&releaseCache.negativeFixtures?.length===4&&releaseCache.negativeFixtures.every(row=>row.rejected===true),
    'release cache: exact aliases, complete graph and negative stale-cache fixtures absent');
  check(equal(releaseCache.releases,{'multi/index.html':'20261016-responsive-1','r1/index.html':'20261016-native-camera-3','c5/index.html':'20261016-native-camera-3'}),
    'release cache: multi responsive cohort or independent native cohorts changed');
  summary.releaseCacheChecks=releaseCache.checks;
  // Also identify the verifier and its honest, bounded acceptance statement.
  for(const path of ['work/verify-multi-acceptance.mjs','work/test-multi-acceptance-verifier.mjs','work/MULTI-ACCEPTANCE.md'])sourceHashes[path]=await currentHash(path);
}

try{await main();}catch(error){errors.push(error.stack||error.message);}
const result={pass:errors.length===0,scope:'Current-source numerical, orchestration, static UI, framing, responsiveness and publication-path acceptance; not rendered-browser or hardware acceptance',
  ...summary,reportHashes,sourceHashes,sourceCount:Object.keys(sourceHashes).length,
  unverified:['Desktop/mobile rendered appearance and camera pixels','Native pointer/touch and accessibility-tree interactions',
    'Cross-station robot/object collisions','Real-hardware calibration, controls or safety'],errors};
const outputPath=process.argv?.find(arg=>arg.startsWith('--output='))?.slice(9)??null;
if(outputPath){const destination=resolve(root,canonical(outputPath));await mkdir(resolve(destination,'..'),{recursive:true});await writeFile(destination,JSON.stringify(result,null,2)+'\n');}
console.log(JSON.stringify(result,null,2));
if(errors.length)process.exitCode=1;
