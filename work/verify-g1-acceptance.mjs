// Verify only final, source-bound reports. This does not run a browser or robot.
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve,relative} from 'node:path';
import {fileURLToPath} from 'node:url';

const root=fileURLToPath(new URL('../',import.meta.url));
const physicsOnly=process.argv.includes('--physics-only'),historicalUI=[];
const retiredUISources=new Set(['dist/g1/index.html','dist/g1/g1-app.mjs']);
const expectedRelease={
  'dist/g1/grasp-controller.mjs':'68c147b966a9c138e98ae0e1fdad0cabb816684d2422ee2fa459feeea2bb198d',
  'dist/g1/models/g1-dynamic-scene.xml':'a1cee708eb8b0e99d81fbb1ec1572420d6b2e2f31b77da08a3af4925b9284f09',
};
const errors=[],check=(condition,message)=>{if(!condition)errors.push(message);};
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const hashCache=new Map(),sourceHashes={},reportHashes={};
async function currentHash(path){
  const absolute=resolve(root,path),local=relative(root,absolute);
  if(!path||local.startsWith('..')||resolve(path)===path)throw new Error(`Invalid source path: ${path}`);
  if(!hashCache.has(path))hashCache.set(path,readFile(absolute).then(sha));
  return hashCache.get(path);
}
const dynamicSources=['work/test-g1-dynamic-grasp.mjs','work/build-g1-dynamics.mjs','dist/g1/grasp-controller.mjs','dist/g1/grasp-kinematics.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/models/g1-dynamic-scene.xml','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/assets/dex1_1/meshes/dex1_col_1.stl','dist/g1/assets/dex1_1/meshes/dex1_col_2.stl'];
async function bind(record,label,required){
  const hashes=record.sourceHashes;
  check(hashes&&typeof hashes==='object'&&!Array.isArray(hashes),`${label}: sourceHashes missing`);
  if(!hashes||typeof hashes!=='object')return;
  for(const path of required)check(typeof hashes[path]==='string',`${label}: missing source hash ${path}`);
  for(const [path,hash]of Object.entries(hashes)){
    check(typeof hash==='string'&&/^[0-9a-f]{64}$/.test(hash),`${label}: invalid SHA256 for ${path}`);
    try{const actual=await currentHash(path);
      if(physicsOnly&&retiredUISources.has(path)){historicalUI.push({path,expected:hash,current:actual});continue;}
      sourceHashes[path]=actual;check(actual===hash,`${label}: stale source ${path}`);}catch(error){errors.push(`${label}: ${error.message}`);}
  }
}
async function report(name,jsonl=false){
  const path=`work/validation/g1/${name}`,bytes=await readFile(resolve(root,path));reportHashes[path]=sha(bytes);
  return jsonl?bytes.toString().trim().split(/\r?\n/).filter(Boolean).map(line=>JSON.parse(line)):JSON.parse(bytes);
}
const stable=value=>JSON.stringify(value&&typeof value==='object'&&!Array.isArray(value)?Object.fromEntries(Object.keys(value).sort().map(key=>[key,value[key]])):value);
const scenarios=['right','left'].flatMap(arm=>['red','blue','yellow'].map(id=>({arm,id})));
scenarios.push({arm:'right',id:'blue',offset:[-.015,.012]},{arm:'left',id:'yellow',offset:[.012,-.012]},{arm:'right',id:'blue',yaw:.4},{arm:'left',id:'yellow',roll:Math.PI/2},{arm:'right',stack:true},{arm:'left',stack:true});
const recoveries=['unreachable_stack','failed_reset_then_sort','stop_stack_mid_transfer','stop_happy_mid_gesture'];
const expectedScenarios=repeats=>[...scenarios.flatMap(s=>Array.from({length:repeats},(_,i)=>stable({...s,repeat:i+1}))),...recoveries.map(recovery=>stable({recovery}))].sort();
const finite=(value,label)=>check(typeof value==='number'&&Number.isFinite(value),`${label}: non-finite number`);
function stableStack(stack,label){
  check(stack?.stable===true,`${label}: released stack is not stable`);
  check(stack?.states?.length===3,`${label}: three stack supports not recorded`);
  for(const [i,state]of (stack?.states??[]).entries()){
    check(state.id===['blue','red','yellow'][i]&&state.supported===true,`${label}: wrong stack/support order`);
    finite(state.speed,`${label}: stack speed`);finite(state.error,`${label}: stack error`);
    check(state.speed<.0175&&state.error<.012,`${label}: stack stability threshold exceeded`);
  }
}
function lifted(evidence,label){
  check(evidence?.dual===true,`${label}: dual-finger contact not recorded`);
  finite(evidence?.lift,`${label}: lift`);check(evidence?.lift>.035,`${label}: insufficient physical lift`);
}
function healthy(row,label){
  check(row.pass===true,`${label}: test did not pass`);
  for(const key of ['samples','fingerLimitSamples','maxFingerLimitViolation','maxTorque','maxFingerForce','maxJointError','maxFootDrift','maxLegAngle','maxWaistTilt','maxWaistYaw'])finite(row[key],`${label}: ${key}`);
  check(row.samples>0&&row.fingerLimitSamples>=row.samples,`${label}: physical/finger sampling absent`);
  check(row.maxFingerLimitViolation<=.0002,`${label}: real finger stroke exceeds 0.2mm tolerance`);
  check(row.maxFingerForce<=20+1e-7,`${label}: finger force exceeds source 20N`);
  check(row.maxJointError<.07&&row.maxFootDrift<.0005&&row.maxLegAngle<.001&&row.maxWaistTilt<.003&&row.maxWaistYaw<1.001,`${label}: tracking/fixed-stance threshold exceeded`);
  check(Object.keys(row.jointExtrema??{}).length===33,`${label}: 33 actual joint ranges missing`);
  check(Object.keys(row.jointExtrema??{}).filter(n=>n.includes('_dex1_finger_joint_')).length===4,`${label}: four real fingers not covered`);
  for(const [name,j]of Object.entries(row.jointExtrema??{})){
    for(const key of ['min','max','targetMin','targetMax','minimumLimitMargin','lower','upper'])finite(j[key],`${label}: ${name}/${key}`);
    const tolerance=name.includes('_dex1_finger_joint_')?.0002:.001;
    check(j.min>=j.lower-tolerance&&j.max<=j.upper+tolerance&&j.minimumLimitMargin>=-tolerance,`${label}: actual joint range exceeded: ${name}`);
    check(j.targetMin>=j.lower-1e-12&&j.targetMax<=j.upper+1e-12,`${label}: commanded joint range exceeded: ${name}`);
  }
  const collision=row.collision;
  check(collision?.samples>0&&collision.samples===row.samples,`${label}: actual contact sampling absent`);
  finite(collision?.maxIllegalPenetration,`${label}: illegal penetration`);
  check(collision?.maxIllegalPenetration<=.001&&collision?.illegalContacts?.length===0&&Object.keys(collision?.kinds??{}).length===0,`${label}: illegal actual collision (>1mm)`);
  if(!row.scenario?.recovery){
    check(row.actualDualForce===true,`${label}: no real dual-finger force`);lifted(row.evidence,label);
    const phases=row.transitions?.map(t=>t.key.split('/').at(-1))??[];
    for(const phase of ['descend','closing','lift','transfer','place','release','retreat'])check(phases.includes(phase),`${label}: phase ${phase} absent`);
    if(row.scenario.stack){
      check(row.stackStableSeconds>=3,`${label}: final stack stability under 3s`);
      check(row.stackEvidence?.length===3,`${label}: three physical stack lifts not recorded`);
      for(const [i,e]of (row.stackEvidence??[]).entries()){check(e.id===['blue','red','yellow'][i],`${label}: incorrect stack lift order`);lifted(e,label);}
      stableStack(row.stackcheck,label);
      for(const phase of ['stack_settle','happy','complete'])check(phases.includes(phase),`${label}: phase ${phase} absent`);
    }else{
      const m=row.measurement;check(m?.table===true&&m.support===true&&m.distance<.02&&m.speed<.025,`${label}: released object not settled in frame`);
      check(row.evidence.maxSpeed<1,`${label}: excessive object speed`);
    }
  }else{
    const recovery=row.scenario.recovery;
    if(recovery==='unreachable_stack')check(row.history?.includes('shy')&&!row.history.includes('happy')&&typeof row.reason==='string',`${label}: unreachable-stack failure reaction missing`);
    if(recovery==='failed_reset_then_sort'){lifted(row.evidence,label);check(row.measurement?.table===true&&row.measurement.distance<.02&&row.actualDualForce===true,`${label}: reset-to-sort recovery missing`);}
    if(recovery.startsWith('stop_'))check(row.state==='stopped'&&row.collision.phases.includes('stopped'),`${label}: stop state not preserved`);
    // The bound test asserts state===transfer and lift>35mm immediately before
    // stop(); its 100ms contact monitor need not sample that instantaneous state.
    if(recovery==='stop_stack_mid_transfer')check(row.actualDualForce===true&&row.collision.phases.includes('lift'),`${label}: did not actually stop after a real grasp`);
    if(recovery==='stop_happy_mid_gesture')stableStack(row.stackcheck,label);
  }
}
async function main(){
  for(const [path,hash]of Object.entries(expectedRelease))check(await currentHash(path)===hash,`Current release changed: ${path}`);
  const [single,repeat,opening,physics,visual]=await Promise.all([report('final-single.jsonl',true),report('final-repeat.jsonl',true),report('opening-safety-final.json'),report('finger-limit-physics-kv20.json'),report('photo-camera-material-check.json')]);
  for(const [name,rows,times]of [['single',single,1],['repeat',repeat,3]]){
    check(rows.length===12*times+4,`${name}: incomplete count ${rows.length}/${12*times+4}`);
    check(JSON.stringify(rows.map(r=>stable(r.scenario)).sort())===JSON.stringify(expectedScenarios(times)),`${name}: missing/duplicate/unexpected scenarios`);
    for(const [i,row]of rows.entries()){await bind(row,`${name}[${i}]`,dynamicSources);healthy(row,`${name}[${i}]`);}
  }
  await bind(opening,'opening',['work/test-g1-opening-safety.mjs',...dynamicSources.filter(p=>['dist/g1/grasp-controller.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/models/g1-dynamic-scene.xml'].includes(p))]);
  const openingCases=['source-mesh-opening-counterexample','manual-opening-rejected-without-state-or-command-write','arm-switch-keeps-both-openings-through-a-physics-step','failed-automatic-preparation-preserves-safe-gaps','stop-keeps-safe-gap-when-release-would-hit-table','failure-keeps-safe-gap-when-release-would-hit-table','reaction-opening-is-checked-before-phase-start','idle-open-close-is-accepted-and-physically-driven','selected-object-fingertip-contact-remains-legal'];
  check(opening.pass===true&&opening.cases===9&&opening.results?.length===9, 'opening: nine-case result incomplete');
  check(JSON.stringify(opening.results?.map(r=>r.name).sort())===JSON.stringify(openingCases.sort())&&opening.results.every(r=>r.pass===true),'opening: required boundary cases missing');
  const counter=opening.results?.find(r=>r.name==='source-mesh-opening-counterexample');check(counter?.closedSafe===true&&counter.openSafe===false&&counter.penetration>.001,'opening: source-mesh unsafe-opening counterexample absent');
  await bind(physics,'physics',['work/build-g1-dynamics.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/models/g1-dynamic-scene.xml','dist/g1/assets/g1_29dof_with_dex1_1.urdf']);
  check(physics.nu===33&&physics.nq===54&&physics.nv===51&&physics.links===43,'physics: fixed-base 33-axis model identity changed');
  check(physics.massError<1e-12&&physics.comError<1e-12&&physics.inertiaError<2e-8&&physics.fkPositionError<1e-10&&physics.fkAngleError<1e-6,'physics: source inertias/FK not preserved');
  check(physics.maxStaticError<.001&&physics.maxFootDrift<.0002&&physics.maxTrackingError<.003,'physics: standing/tracking test failed');
  for(const flag of ['neutralSafe','selectedFingerIgnoreSafe','selectedPalmStillBlocked','actualStateUnchanged'])check(physics.collisionChecks?.[flag]===true,`physics: ${flag} failed`);
  check(physics.collisionChecks?.blockedContacts?.some(c=>c.kind==='robot-self'&&c.dist<0),'physics: invalid arm-through-torso pose not detected');
  check(physics.objectPoseChecks?.predictionPositionError<1e-12&&physics.objectPoseChecks.predictionRotationError<1e-7&&physics.objectPoseChecks.invalidCasesRejected===7&&physics.objectPoseChecks.selectedObjectTorsoBlocked===true&&physics.objectPoseChecks.actualStateUnchanged===true,'physics: shadow prediction mutated state or bypassed body collision');
  check(physics.actualLegs?.length===12&&physics.actualLegs.every(j=>j.target===0&&Math.abs(j.actual)<.001),'physics: fixed zero-angle legs not retained');
  await bind(visual,'visual-static',['work/check-g1-camera-materials.mjs','dist/g1/g1-materials.mjs','dist/g1/g1-workcell.mjs','dist/g1/g1-profile.mjs','dist/g1/assets/g1_29dof_with_dex1_1.urdf']);
  check(visual.pass===true&&visual.materialReport?.unmappedLinks?.length===0&&visual.visualMeshes===39,'visual-static: material coverage incomplete');
  check(visual.geometryUnchanged?.length===39&&visual.geometryUnchanged.every(v=>v.sourceBytes&&v.positions&&v.normals),'visual-static: source visual geometry modified');
  check(visual.mainFrustums?.length===2&&visual.mainFrustums.every(v=>v.clipped===0),'visual-static: desktop/phone projection clipped');
  check(visual.wristChecks?.length===6&&visual.wristChecks.every(v=>v.groups.every(g=>g.clipped===0)&&v.lensAndCameraSeparation<1e-9),'visual-static: camera/lens projection failed');
  check(JSON.stringify(visual.cameraConfiguration?.lensPosition)===JSON.stringify([.06,0,.06])&&JSON.stringify(visual.cameraConfiguration?.targetPosition)===JSON.stringify([.12,0,0])&&visual.cameraConfiguration?.fov===76&&visual.mountToFingerSweepZClearance>.011,'visual-static: photo-guided front/upper camera changed');
  // Detect source edits during verification; never bless a mixed snapshot.
  for(const [path,hash]of Object.entries(sourceHashes))check(sha(await readFile(resolve(root,path)))===hash,`Source changed during verification: ${path}`);
  const rows=[...single,...repeat],normal=rows.filter(r=>!r.scenario.recovery),stacks=normal.filter(r=>r.scenario.stack),lifts=normal.flatMap(r=>r.scenario.stack?r.stackEvidence:[r.evidence]);
  const max=key=>Math.max(...rows.map(r=>r[key]));
  const result={pass:errors.length===0,physicsOnly,historicalUI,scope:physicsOnly?'Retained original physical/model evidence only; historical app/index UI hashes excluded. Current Worker execution and browser UI require separate performance-v4 evidence.':'G1 browser-simulator preview; recorded headless physics plus static camera/material checks only',reports:{single:single.length,repeat:repeat.length,total:rows.length,sort:normal.length-stacks.length,stack:stacks.length,recovery:rows.length-normal.length,opening:opening.cases},observed:{fingerLimitSamples:rows.reduce((n,r)=>n+r.fingerLimitSamples,0),contactSamples:rows.reduce((n,r)=>n+r.collision.samples,0),maxFingerLimitViolationMetres:max('maxFingerLimitViolation'),maxIllegalPenetrationMetres:Math.max(...rows.map(r=>r.collision.maxIllegalPenetration)),maxJointErrorRad:max('maxJointError'),maxFootDriftMetres:max('maxFootDrift'),maxLegAngleRad:max('maxLegAngle'),maxWaistTiltRad:max('maxWaistTilt'),maxWaistYawRad:max('maxWaistYaw'),maxFingerForceN:max('maxFingerForce'),maxMotorTorqueNm:max('maxTorque'),maxWristRollRad:Math.max(...rows.flatMap(r=>Object.values(r.maxWristRoll))),wristBelowPoint5IsSoftPreference:rows.every(r=>r.naturalWristRollPreference===true),minimumPhysicalLiftMetres:Math.min(...lifts.map(e=>e.lift)),maxFinalSortErrorMetres:Math.max(...normal.filter(r=>!r.scenario.stack).map(r=>r.measurement.distance))},limits:{realFingerStrokeToleranceMetres:.0002,illegalContactPenetrationThresholdMetres:.001,contactSamplingSeconds:.1,fingerSamplingSeconds:.001},notVerified:['rendered browser appearance/occlusion/reflections','physical touchscreen interaction','real-robot balance, calibration or operational safety'],expectedRelease,sourceHashes,reportHashes,errors};
  console.log(JSON.stringify(result,null,2));if(!result.pass)process.exitCode=1;
}
main().catch(error=>{console.error(JSON.stringify({pass:false,error:error.message}));process.exitCode=1;});
