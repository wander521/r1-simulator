import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve,relative,isAbsolute} from 'node:path';
import {fileURLToPath} from 'node:url';
import {generatePipeLayout} from '../dist/multi/pipe-layout.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {verifyPerformanceV3Contracts} from './performance-v3-contracts.mjs';

const defaultRoot=fileURLToPath(new URL('../',import.meta.url));
const robotIds=['r1','g1','c5'],fingerTolerance={r1:.001,g1:.0002,c5:.0002};
const fixedNames=['single-r1-eight-pipe-classification','single-g1-eight-pipe-classification','single-c5-eight-pipe-classification',
  'stop-three-way-pipe-task-after-genuine-lifts','unreachable-pipe-preparation-aborts-all-attempted-stations',
  'pipe-task-reset-randomizes-and-explicit-seed-reproduces-all-eight-starts'];
const requiredSources=['work/test-multi-pipes.mjs','dist/multi/robot-runtime.mjs','dist/multi/coordinator.mjs','dist/multi/planning.mjs',
  'dist/multi/pipe-layout.mjs','dist/multi/pipe-controller.mjs','dist/multi/pipe-model.mjs','dist/multi/r1-safe-dynamics.mjs',
  ...robotIds.flatMap(id=>[`work/${id}-test-robot.mjs`,`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,
    `dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/r1/mujoco-runtime.mjs',
  'dist/vendor/three.module.min.js',
  'dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const equal=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
const workerSources=['background-planning','planning-worker','planning-worker-core','planning-worker-transport','planning-serialization','planning-robot','planning-check-cache','planning-worker-protocol','planning-ik-context','planning-budget'].map(name=>`dist/multi/${name}.mjs`);

/** Preparation history is command metadata, not evidence of physical motion.
 * Verify the rejected all-participant barrier from complete raw snapshots;
 * this sub-check alone never certifies a FULL72 manipulation release. */
export function verifyRejectedPreparationSafety(row){
  const errors=[],check=(condition,message)=>{if(!condition)errors.push(message);},safety=row?.preparationSafety;
  // Exact dimensions of the source-bound original robot models plus eight
  // free pipe bodies. Two equally truncated arrays are not a complete proof.
  const dimensions={r1:{qpos:82,qvel:74,ctrl:26,armTargets:22},g1:{qpos:89,qvel:81,ctrl:33,armTargets:29},c5:{qpos:64,qvel:56,ctrl:8,armTargets:6}};
  const record=value=>value!==null&&typeof value==='object'&&!Array.isArray(value);
  const sameNumbers=(left,right)=>{
    if(!Array.isArray(left)||!Array.isArray(right)||left.length!==right.length)return false;
    for(let index=0;index<left.length;index++)if(!Number.isFinite(left[index])||!Number.isFinite(right[index])||!Object.is(left[index],right[index]))return false;
    return true;
  };
  const sameTargets=(left,right)=>record(left)&&record(right)&&Object.keys(left).length>0
    &&equal(Object.keys(left).sort(),Object.keys(right).sort())&&Object.keys(left).every(name=>Number.isFinite(left[name])&&Object.is(left[name],right[name]));
  check(safety?.noPhysicalAdvance===true&&safety?.unchangedPhysicsArrays===true,'rejected preparation: unchanged physical snapshot proof is required');
  check(safety?.noBatchCommit===true,'rejected preparation: no-batch-commit proof is required');
  check(safety?.allWorkersCancelled===true,'rejected preparation: all-Worker cancellation proof is required');
  check(safety?.unchangedArmTargets===true,'rejected preparation: unchanged arm-target proof is required');
  const before=safety?.coordinatorBefore,after=safety?.coordinatorAfter;
  check(Number.isSafeInteger(before?.tick)&&before.tick>=0&&after?.tick===before.tick,'rejected preparation: shared physical tick advanced or is absent');
  check(Number.isSafeInteger(before?.batchNumber)&&before.batchNumber===0&&after?.batchNumber===before.batchNumber
    &&before?.startTick===null&&after?.startTick===null&&record(before?.startTimes)&&record(after?.startTimes)
    &&Object.keys(before.startTimes).length===0&&Object.keys(after.startTimes).length===0,'rejected preparation: batch was committed or start markers changed');
  check(after?.pendingStart===null&&after?.planning===false,'rejected preparation: pending batch or Worker planning was not cancelled');
  check(row?.start?.ok===false&&row.start.startTick==null&&(row.start.startTimes===undefined
    ||(record(row.start.startTimes)&&Object.keys(row.start.startTimes).length===0)),'rejected preparation: failed start claimed a committed physical batch');
  const worlds=Array.isArray(safety?.worlds)?safety.worlds:[];
  check(worlds.length===3&&equal(worlds.map(world=>world?.id).sort(),[...robotIds].sort()),'rejected preparation: three independent raw world snapshots are required');
  for(const id of robotIds){
    const world=worlds.find(world=>world?.id===id),old=world?.before,current=world?.after,robot=row?.robots?.find(robot=>robot.id===id),stats=row?.stats?.[id];
    check(Number.isFinite(old?.time)&&old.time>=0&&current?.time===old.time&&robot?.time===current?.time
      &&Number.isFinite(before?.tick)&&Math.abs(old.time-before.tick*.001)<1e-10,`rejected preparation/${id}: physical time advanced or disagrees with the shared tick`);
    for(const name of ['qpos','qvel','ctrl'])check(old?.[name]?.length===dimensions[id][name]
      &&sameNumbers(old?.[name],current?.[name]),`rejected preparation/${id}: unchanged physical ${name} array is absent or changed`);
    check(record(old?.armTargets)&&Object.keys(old.armTargets).length===dimensions[id].armTargets
      &&sameTargets(old?.armTargets,current?.armTargets),`rejected preparation/${id}: non-finger arm targets changed or are absent`);
    check(current?.planning===false&&current?.running===false&&current?.verified===false&&current?.completed===0
      &&current?.objectEditsDuringTask===0&&robot?.running===false&&robot?.state==='stopped'
      &&robot?.classification?.verified===false&&robot?.classification?.completed===0,`rejected preparation/${id}: a Worker, task or classification remained active`);
    check(stats?.sequence?.length===0&&stats?.objectEditsDuringTask===0,'rejected preparation: a physical task advanced after failed preparation');
  }
  return{pass:errors.length===0,errors,scope:'Rejected preparation barrier only; raw physical snapshots and batch/cancellation state, not a full manipulation or browser acceptance.'};
}

/** Read-only report validation. It does not run absent physics, certify browser
 * pixels/hardware, repair saved evidence, or treat quick/development runs as a
 * seventy-two-item full release. */
export async function verifyPipeAcceptance(report,{rootPath=defaultRoot,requireBackground=false,reportDirectory='work/validation/performance-v3',readEvidence=readFile}={}) {
  const errors=[],sourceHashes={},reportHashes={},summary={cases:0,physicalPipeClassifications:0,simultaneousSeeds:[],
    layoutChecks:0,millisecondSamples:0,contactScans:0,minActualLiftM:null,maxReleasedSlotErrorM:0,
    maxTranslationResidualM:0,maxRotationResidualRad:0,maxIllegalPenetrationM:0,
    maxFingerLimitViolationM:{r1:0,g1:0,c5:0},carryObservations:{samples:0,minDualFraction:null,
      maxContinuousGripLossSeconds:0,minTransferDualFraction:null,maxTransferGripLossSeconds:0,
      maxTcpDistanceM:0,maxTransferTcpDistanceM:0,maxRelativePositionDriftM:0,maxRelativeAngleDriftRad:0}};
  const check=(condition,message)=>{if(!condition)errors.push(message);};
  const finite=(value,message)=>check(typeof value==='number'&&Number.isFinite(value),`${message}: non-finite or missing value`);
  const velocityCache=new Map();
  async function velocitiesFor(id) {
    if(!velocityCache.has(id)) {
      const path={r1:'dist/r1/assets/r1.urdf',g1:'dist/g1/assets/g1_29dof_with_dex1_1.urdf',c5:'dist/c5/assets/c5-dex1.urdf'}[id];
      const urdf=await readFile(resolve(rootPath,path),'utf8'),velocities=new Map();
      for(const [,attributes,body]of urdf.matchAll(/<joint\b([^>]*)>([\s\S]*?)<\/joint>/g)) {
        const name=attributes.match(/\bname="([^"]+)"/)?.[1],limit=Number(body.match(/<limit\b[^>]*\bvelocity="([^"]+)"/)?.[1]);
        if(name&&Number.isFinite(limit)&&limit>0)velocities.set(name,limit);
      }
      velocityCache.set(id,velocities);
    }
    return velocityCache.get(id);
  }
  try {
    const contracts=await verifyPerformanceV3Contracts({rootPath,reportDirectory,readEvidence});
    errors.push(...contracts.errors);Object.assign(sourceHashes,contracts.sourceHashes);Object.assign(reportHashes,contracts.reportHashes);
    summary.v3ContractChecks=contracts.counts;
    check(report?.pass===true,'pipe report did not pass');check(report?.mode==='full','quick/static-only/filtered evidence cannot certify a full release');
    check(report?.sourceFrozen===true,'source was not frozen during physical acceptance');
    if(requireBackground){
      check(report?.planningExecution?.worker===true&&report.planningExecution.cooperative===false&&report.planningExecution.productionBudgetOverride===false,'real background worker execution without a planning budget override is required');
      const progress=report?.planningExecution?.independentProgress;
      check(report?.planningExecution?.independentPlanning===true&&progress?.activeWorldTicksDuringPeerPlanning>0
        &&progress?.pausedWorldTicks>0&&progress?.pausedStateChecks===progress?.pausedWorldTicks,
        'independent planning with active peers and exact frozen-world checks is required');
    }
    check(report?.clockStepSeconds===.001,'shared physical timestep is not 1 ms');
    check(equal(report?.fingerToleranceM,fingerTolerance),'original R1/Dex1 finger guards changed');
    check(report?.cases===8&&report?.expectedCases===8,'expected eight complete pipe-task cases');
    check(report?.physicallySortedPipes===72,'expected seventy-two genuine pipe classifications');
    const rows=report?.results??[],names=rows.map(row=>row.name),simultaneous=rows.filter(row=>/^three-way-eight-pipe-classification-seed-\d+$/.test(row.name));
    check(rows.length===8&&new Set(names).size===8&&rows.every(row=>row.pass===true),'missing, duplicate or failed pipe-task cases');
    check(simultaneous.length===2&&new Set(simultaneous.map(row=>row.seed)).size===2,'two distinct simultaneous physical seeds are required');
    check(fixedNames.every(name=>names.includes(name)),'required single/stop/rollback/reset case is missing');
    check(names.every(name=>fixedNames.includes(name)||/^three-way-eight-pipe-classification-seed-\d+$/.test(name)),'unexpected physical case cannot replace a required case');
    summary.cases=rows.length;summary.simultaneousSeeds=simultaneous.map(row=>row.seed);
    for(const row of simultaneous)check(row.name.endsWith(String(row.seed)),'simultaneous seed/name mismatch');
    const layout=report?.layout;
    check(layout?.pass===true&&layout?.seedsPerRobot===100&&layout?.totalLayouts===300,'three hundred production-layout checks are absent');
    check(robotIds.every(id=>layout?.distinctLayouts?.[id]===100),'random layouts are not reproducible/distinct for every robot');
    check(Array.isArray(layout?.samples)&&layout.samples.length===6&&robotIds.every(id=>layout.samples.filter(sample=>sample.id===id).length===2),'representative seeded layout evidence is missing');
    summary.layoutChecks=layout?.totalLayouts??0;
    for(const sample of layout?.samples??[]) {
      const targets=sample.targets??[];check(targets.length===8&&new Set(targets.map(target=>target.id)).size===8,'layout: eight unique pipe IDs absent');
      for(const color of ['red','yellow'])check(targets.filter(target=>target.color===color).length===4,'layout: four red/four yellow cylinders absent');
      for(const target of targets) {
        check(target.frame?.colorKey===target.color,'layout: wrong-color destination frame');
        check(Array.isArray(target.start)&&target.start.length===3&&target.start.every(Number.isFinite)
          &&Array.isArray(target.drop)&&target.drop.length===3&&target.drop.every(Number.isFinite),'layout: invalid source/destination coordinates');
        check(target.start?.[0]<target.drop?.[0],'layout: pipes are not on the near side of the frames');
      }
    }
    const lifts=[];
    for(const row of rows) {
      const label=`pipes/${row.name}`,stats=row.stats??{},robots=row.robots??[];
      check(equal(Object.keys(stats).sort(),[...robotIds].sort())&&equal(robots.map(robot=>robot.id).sort(),[...robotIds].sort()),`${label}: three independently monitored worlds absent`);
      const times=robots.map(robot=>robot.time);times.forEach(time=>finite(time,`${label}/time`));
      const independent=report.planningExecution?.independentPlanning===true;
      if(!independent)check(Math.max(...times)-Math.min(...times)<1e-10,`${label}: physical clocks diverged`);
      const counts=robotIds.map(id=>stats[id]?.millisecondSamples);
      check(counts.every(count=>Number.isInteger(count)&&count>0)&&(independent||new Set(counts).size===1),`${label}: all-world 1 ms sampling absent`);
      // The explicit reset scenario retains pre-reset sample counters while
      // resetting each physical clock to zero. Its reset state is checked below.
      if(independent&&row.name!=='pipe-task-reset-randomizes-and-explicit-seed-reproduces-all-eight-starts')for(const robot of robots)
        check(Math.abs(robot.time-stats[robot.id].millisecondSamples*.001)<1e-7,`${label}: physical clocks diverged from exact per-world 1 ms samples`);
      const success=row.name.startsWith('three-way-')||row.name.startsWith('single-');
      const selected=success?(row.name.startsWith('single-')?[row.name.split('-')[1]]:robotIds):robotIds;
      check(equal(row.selectedIds,selected),`${label}: selected-robot participants changed`);
      check(row.start?.ok===!row.name.startsWith('unreachable-'),`${label}: start outcome missing or incorrect`);
      if(!row.name.startsWith('unreachable-'))check(equal(row.start?.runIds,selected)
        &&new Set(Object.values(row.start?.startTimes??{})).size===1,`${label}: tasks did not start at one shared physical time`);
      check(row.physicallySortedPipes===(success?selected.length*8:0),`${label}: claimed physical item count incorrect`);
      if(row.name.startsWith('unreachable-'))errors.push(...verifyRejectedPreparationSafety(row).errors.map(error=>`${label}: ${error}`));
      for(const id of robotIds) {
        const stationErrorStart=errors.length;
        const s=stats[id],robot=robots.find(item=>item.id===id);if(!s||!robot)continue;
        for(const key of ['millisecondSamples','contactSamples','objectEditsDuringTask','maxTranslationResidualM','maxRotationResidualRad',
          'maxTorqueRatio','maxFingerLimitViolationM','fingerToleranceM','maxRevoluteLimitViolationRad',
          'maxActualVelocityRatio','maxIllegalPenetrationM','maxPipePipePenetrationM','maxOffTargetFingerForceN',
          'maxG1LegAngleRad','maxG1FootDriftM','maxUnratedWheelVelocityRadps','extraStableSamples'])finite(s[key],`${label}/${id}/${key}`);
        check(s.contactSamples===Math.floor(s.millisecondSamples/10)&&s.contactSamples>0,`${label}/${id}: actual 10 ms contact scans absent`);
        check(s.objectEditsDuringTask===0&&s.maxTranslationResidualM<1e-5&&s.maxRotationResidualRad<1e-5,`${label}/${id}: task objects were teleported or rotated without physical velocity`);
        check(s.maxTorqueRatio<=1+1e-6&&s.maxRevoluteLimitViolationRad<=.001&&s.maxActualVelocityRatio<=1.02,`${label}/${id}: original motor effort/range/velocity bound exceeded`);
        const velocityRows=Object.entries(s.criticalVelocityExtrema??{}),sourceVelocities=await velocitiesFor(id);
        if(s.maxActualVelocityRatio>.9)check(velocityRows.length>0,`${label}/${id}: near-limit velocity has no actual per-joint trace`);
        for(const [name,event]of velocityRows) {
          const limit=sourceVelocities.get(name);
          check(Number.isFinite(limit)&&event.sourceVelocity===limit&&Number.isFinite(event.absoluteVelocity)
            &&event.absoluteVelocity===Math.abs(event.velocity)&&event.absoluteVelocity<=limit*1.01+.0001,
            `${label}/${id}/${name}: actual per-joint source velocity guard exceeded or changed`);
        }
        if(velocityRows.length)check(Math.abs(Math.max(...velocityRows.map(([,event])=>event.absoluteVelocity/event.sourceVelocity))-s.maxActualVelocityRatio)<1e-12,
          `${label}/${id}: aggregate velocity ratio disagrees with actual per-joint extrema`);
        check(s.fingerToleranceM===fingerTolerance[id]&&s.maxFingerLimitViolationM<=fingerTolerance[id],`${label}/${id}: original finger guard exceeded`);
        check(s.maxIllegalPenetrationM<=.001&&s.maxPipePipePenetrationM<=.001,`${label}/${id}: illegal actual physical penetration`);
        check(s.maxOffTargetFingerForceN<=.015,`${label}/${id}: gripper pushed a non-target body`);
        if(id==='g1')check(s.maxG1LegAngleRad<.001&&s.maxG1FootDriftM<.0005,`${label}/${id}: fixed legs/feet moved or squatted`);
        check(id==='r1'?equal([...(s.unratedWheelNames??[])].sort(),['wheel_motor_joint1','wheel_motor_joint2','wheel_motor_joint3'])
          &&s.maxUnratedWheelVelocityRadps<.001:equal(s.unratedWheelNames,[]),`${label}/${id}: unrated continuous-wheel scope/stationarity changed`);
        summary.millisecondSamples+=s.millisecondSamples;summary.contactScans+=s.contactSamples;
        summary.maxTranslationResidualM=Math.max(summary.maxTranslationResidualM,s.maxTranslationResidualM);
        summary.maxRotationResidualRad=Math.max(summary.maxRotationResidualRad,s.maxRotationResidualRad);
        summary.maxIllegalPenetrationM=Math.max(summary.maxIllegalPenetrationM,s.maxIllegalPenetrationM,s.maxPipePipePenetrationM);
        summary.maxFingerLimitViolationM[id]=Math.max(summary.maxFingerLimitViolationM[id],s.maxFingerLimitViolationM);
        const observed=Object.values(s.objects??{}),classification=robot.classification;
        check(observed.length===8&&new Set(observed.map(object=>object.id)).size===8,`${label}/${id}: eight independently observed pipe bodies absent`);
        check(classification?.total===8,`${label}/${id}: eight-item production task absent`);
        const expectedLayout=generatePipeLayout(ROBOT_CONFIGS[id],{seed:row.seed});
        for(const object of observed) {
          const target=expectedLayout.targets.find(target=>target.id===object.id),bin=expectedLayout.bins.find(bin=>bin.colorKey===target?.colorKey);
          check(target&&bin&&object.color===target.colorKey&&equal(object.start,target.start.toArray())&&equal(object.drop,target.drop.toArray()),
            `${label}/${id}/${object.id}: observed props disagree with the current seeded production layout`);
          if(bin)check(equal(object.frame,{colorKey:bin.colorKey,minX:bin.center.x-bin.halfX,maxX:bin.center.x+bin.halfX,
            minZ:bin.center.z-bin.halfZ,maxZ:bin.center.z+bin.halfZ}),`${label}/${id}/${object.id}: destination bounds disagree with the production color frame`);
        }
        if(success&&selected.includes(id)) {
          check(robot.state==='complete'&&robot.running===false&&classification?.completed===8&&classification?.verified===true
            &&classification.started===true&&classification.finalStableSeconds>=3,`${label}/${id}: eight-item task did not actually complete and stabilize`);
          check(row.restartAfterComplete?.ok===false,`${label}/${id}: completed batch accepted a restart without reset`);
          check(classification?.check?.stable===true&&classification.check.states?.length===8
            &&equal(classification.check.states.map(state=>state.id).sort(),expectedLayout.targets.map(target=>target.id).sort())
            &&classification.check.states.every(state=>state.stable&&state.supported&&state.released===true&&state.inside&&state.speed<.025&&state.distance<.02),
            `${label}/${id}: production final all-eight support check absent`);
          check(s.extraStableSamples===300&&row.extraStableSeconds===3,`${label}/${id}: extra three seconds of independent support absent`);
          check(s.visualFramesChecked===8,`${label}/${id}: eight actual/visual local-frame mappings were not checked`);
          const expectedArms=id==='c5'?Array(8).fill('left'):Array.from({length:8},(_,index)=>index%2?'left':'right');
          check(s.sequence?.length===8&&new Set(s.sequence.map(item=>item.id)).size===8,`${label}/${id}: repeated/omitted pipe manipulation`);
          check(equal(s.sequence?.map(item=>item.arm),expectedArms)&&equal(classification.armSequence,expectedArms),`${label}/${id}: left/right arms did not alternate automatically`);
          check(classification.items?.length===8&&equal(classification.items.map(item=>item.id),s.sequence?.map(item=>item.id)),`${label}/${id}: production item evidence disagrees with physical sequence`);
          for(const [itemIndex,item]of(classification.items??[]).entries()) {
            check(item.dual===true&&Number.isFinite(item.lift)&&item.lift>.035&&item.tableSupport===true,`${label}/${id}/${item.id}: source item has no actual lift/contact/table support`);
            check(item.arm===expectedArms[itemIndex]&&item.colorKey===s.objects?.[item.id]?.color,`${label}/${id}/${item.id}: source item arm/color disagrees with physical sequence`);
            check(Number.isFinite(item.placementError)&&item.placementError<.02&&Number.isFinite(item.speed)&&item.speed<.025,`${label}/${id}/${item.id}: source item was not stably released`);
          }
          for(const color of ['red','yellow'])check(observed.filter(object=>object.color===color).length===4,`${label}/${id}: incorrect color count`);
          for(const object of observed) {
            const itemLabel=`${label}/${id}/${object.id}`,final=object.final;
            check(object.actualDual===true&&object.maxFingerForcesN?.length===2&&object.maxFingerForcesN.every(force=>Number.isFinite(force)&&force>.015),`${itemLabel}: no actual dual-finger force`);
            check(Number.isFinite(object.maxLiftM)&&object.maxLiftM>.035,`${itemLabel}: no actual 35 mm physical lift`);
            check(Number.isFinite(object.maxSpeedMps)&&object.maxSpeedMps<1,`${itemLabel}: pipe flung during task`);
            check(object.arms?.length===1&&object.arms[0]===s.sequence?.find(item=>item.id===object.id)?.arm,`${itemLabel}: hand changed during manipulation`);
            check(['approach','descend','closing','lift','transfer','place','release','retreat','settling'].every(state=>object.states?.includes(state)),`${itemLabel}: actual manipulation phases absent`);
            const carry=object.carry;
            check(carry?.samplePeriodSeconds===.01&&Number.isInteger(carry.samples)&&carry.samples>0
              &&Number.isInteger(carry.dualSamples)&&carry.dualSamples>=0&&carry.dualSamples<=carry.samples
              &&Math.abs(carry.dualFraction-carry.dualSamples/carry.samples)<1e-12,
              `${itemLabel}: actual 100 Hz transport contact observations absent/inconsistent`);
            for(const key of ['longestGripLossSeconds','maxTcpDistanceM','maxRelativePositionDriftM','maxRelativeAngleDriftRad'])
              finite(carry?.[key],`${itemLabel}/carry/${key}`);
            check(['lift','transfer'].every(phase=>Number.isInteger(carry?.phases?.[phase]?.samples)&&carry.phases[phase].samples>0)
              &&Array.isArray(carry?.trace)&&carry.trace.length>=2,`${itemLabel}: actual lift/transfer relative-pose trace absent`);
            for(const event of carry?.trace??[])check(event.forces?.length===2&&event.forces.every(Number.isFinite)
              &&event.dual===event.forces.every(force=>force>.015)&&event.relativePosition?.length===3
              &&event.relativePosition.every(Number.isFinite)&&event.relativeQuaternion?.length===4
              &&event.relativeQuaternion.every(Number.isFinite),`${itemLabel}: invalid actual carry keyframe`);
            if(carry) {
              const c=summary.carryObservations;c.samples+=Number.isInteger(carry.samples)?carry.samples:0;
              if(Number.isFinite(carry.dualFraction))c.minDualFraction=Math.min(c.minDualFraction??Infinity,carry.dualFraction);
              if(Number.isFinite(carry.phases?.transfer?.dualFraction))c.minTransferDualFraction=Math.min(c.minTransferDualFraction??Infinity,carry.phases.transfer.dualFraction);
              for(const [key,value]of Object.entries({maxContinuousGripLossSeconds:carry.longestGripLossSeconds,
                maxTransferGripLossSeconds:carry.phases?.transfer?.longestGripLossSeconds,maxTcpDistanceM:carry.maxTcpDistanceM,
                maxTransferTcpDistanceM:carry.phases?.transfer?.maxTcpDistanceM,maxRelativePositionDriftM:carry.maxRelativePositionDriftM,
                maxRelativeAngleDriftRad:carry.maxRelativeAngleDriftRad}))if(Number.isFinite(value))c[key]=Math.max(c[key],value);
            }
            check(Number.isFinite(object.supportNormalN)&&object.supportNormalN>.005,`${itemLabel}: no actual table support after completion`);
            check(Number.isFinite(final?.speedMps)&&final.speedMps<.025&&Number.isFinite(final?.slotErrorM)&&final.slotErrorM<.02
              &&Number.isFinite(final?.allFingerNormalN)&&final.allFingerNormalN<.015,`${itemLabel}: pipe not stably released at the reserved slot`);
            const frame=object.frame,p=final?.position;
            check(frame?.colorKey===object.color&&Array.isArray(p)&&p.length===3&&p.every(Number.isFinite),`${itemLabel}: actual final color/position absent`);
            check(p?.[0]-.02>=frame?.minX-.003&&p?.[0]+.02<=frame?.maxX+.003
              &&p?.[2]-.02>=frame?.minZ-.003&&p?.[2]+.02<=frame?.maxZ+.003,`${itemLabel}: pipe is outside its correct color frame`);
            if(object.actualDual===true&&Number.isFinite(object.maxLiftM)&&object.maxLiftM>.035)lifts.push(object.maxLiftM);
            if(Number.isFinite(final?.slotErrorM))summary.maxReleasedSlotErrorM=Math.max(summary.maxReleasedSlotErrorM,final.slotErrorM);
          }
          if(errors.length===stationErrorStart&&row.pass===true)summary.physicalPipeClassifications+=8;
        } else if(success) {
          check(robot.state==='idle'&&robot.running===false&&robot.history?.length===0&&classification?.completed===0
            &&classification?.verified===false&&classification.started===false&&s.sequence?.length===0&&observed.every(object=>object.actualDual===false),`${label}/${id}: unselected station received a task`);
        } else if(row.name.startsWith('stop-')) {
          check(robot.state==='stopped'&&robot.running===false&&classification?.verified===false&&classification.started===true&&classification.completed<8,`${label}/${id}: stopped task falsely reported completion`);
          check(row.restartAfterStop?.ok===false,`${label}/${id}: stopped batch accepted a restart without reset`);
          check(observed.some(object=>object.actualDual===true&&object.maxLiftM>.035)&&row.postStopSeconds===3,`${label}/${id}: stop was not exercised after a true physical grasp/lift`);
        } else if(row.name.startsWith('unreachable-')) {
          check(robot.running===false&&classification?.completed===0&&classification?.verified===false,`${label}/${id}: rejected start left a running/complete classification`);
          check(row.restartAfterFailure?.ok===false,`${label}/${id}: failed batch accepted a restart without reset`);
        } else {
          check(robot.state==='idle'&&robot.running===false&&robot.time===0&&classification?.completed===0&&classification?.verified===false
            &&classification.started===false&&classification?.items?.length===0&&classification?.armSequence?.length===0,`${label}/${id}: reset left a task/evidence active`);
          check(row.restartAfterReset?.ok===true,`${label}/${id}: reset did not allow a fresh random-batch start`);
        }
      }
    }
    check(summary.physicalPipeClassifications===72,'seventy-two physical successes were not independently verified');
    summary.minActualLiftM=lifts.length?Math.min(...lifts):null;
    const hashes=report?.sourceHashes;
    check(hashes&&typeof hashes==='object'&&!Array.isArray(hashes),'source hash manifest absent');
    for(const path of [...requiredSources,...(requireBackground?[...workerSources,'work/background-planning-node-worker.mjs']:[])])check(Object.hasOwn(hashes??{},path),`source binding missing: ${path}`);
    for(const [path,expected]of Object.entries(hashes??{})) {
      if(typeof path!=='string'||!path||isAbsolute(path)||path.includes('\0')){errors.push(`invalid source path: ${path}`);continue;}
      const absolute=resolve(rootPath,path),local=relative(rootPath,absolute);
      if(!local||local==='..'||local.startsWith('../')){errors.push(`source outside checkout: ${path}`);continue;}
      check(typeof expected==='string'&&/^[a-f0-9]{64}$/.test(expected),`invalid source SHA: ${path}`);
      const actual=sha(await readFile(absolute));sourceHashes[path]=actual;check(actual===expected,`stale source: ${path}`);
    }
  } catch(error) {errors.push(error.stack??error.message);}
  for(const path of ['work/verify-pipe-acceptance.mjs','work/test-pipe-acceptance-verifier.mjs']){
    try{sourceHashes[path]=sha(await readFile(resolve(rootPath,path)));}catch(error){errors.push(`verifier source unavailable: ${path}: ${error.message}`);}
  }
  return {pass:errors.length===0,scope:'Current-source numerical pipe-classification acceptance; not rendered-browser or real-hardware certification',
    ...summary,sourceHashes,reportHashes,sourceCount:Object.keys(sourceHashes).length,
    unverified:['Rendered desktop/mobile pixels and native pointer/touch interactions','Cross-station collisions between independent worlds','Hardware calibration, controls and safety'],errors};
}

if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)) {
  let result;
  try {
    const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)
      ??'work/validation/performance-v3/pipes-physical-compiled-model-final.json';
    const reportDirectory=process.argv.find(arg=>arg.startsWith('--report-dir='))?.slice(13)??relative(defaultRoot,resolve(defaultRoot,reportPath,'..'));
    const bytes=await readFile(resolve(defaultRoot,reportPath));result=await verifyPipeAcceptance(JSON.parse(bytes),{requireBackground:process.argv.includes('--require-background'),reportDirectory});
    result.reportSha256=sha(bytes);result.report=reportPath;result.verifierSha256=sha(await readFile(fileURLToPath(import.meta.url)));
  } catch(error) {result={pass:false,errors:[error.stack??error.message]};}
  const outputPath=process.argv.find(arg=>arg.startsWith('--output='))?.slice(9)??null;
  if(outputPath){const destination=resolve(defaultRoot,outputPath);await mkdir(resolve(destination,'..'),{recursive:true});await writeFile(destination,JSON.stringify(result,null,2)+'\n');}
  console.log(JSON.stringify(result,null,2));if(!result.pass)process.exitCode=1;
}
