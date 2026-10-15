import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';

// Read-only acceptance audit. It reports what saved evidence actually proves;
// it does not run the application, alter reports or infer hardware touch tests.
const root=new URL('../',import.meta.url),checks=[],details={};
const check=(name,pass,evidence)=>checks.push({name,pass:Boolean(pass),evidence});
const json=async path=>JSON.parse(await readFile(new URL(path,root),'utf8'));
const jsonl=async path=>(await readFile(new URL(path,root),'utf8')).trim().split('\n').filter(Boolean).map(JSON.parse);
const finite=value=>typeof value==='number'&&Number.isFinite(value);
const distance=(a,b)=>Math.hypot(...a.map((v,i)=>v-b[i]));
const stageKeys=['closing','lift','place','release'];
function jpegSize(buffer){
  if(buffer[0]!==255||buffer[1]!==216)throw new Error('Not a JPEG');
  for(let i=2;i<buffer.length;){
    if(buffer[i++]!==255)continue;while(buffer[i]===255)i++;const marker=buffer[i++];
    if(marker===217||marker===218)break;if(marker>=208&&marker<=215)continue;
    const size=buffer.readUInt16BE(i);
    if([192,193,194,195,197,198,199,201,202,203,205,206,207].includes(marker))return {width:buffer.readUInt16BE(i+5),height:buffer.readUInt16BE(i+3)};
    i+=size;
  }
  throw new Error('JPEG dimensions unavailable');
}
function mp4Info(buffer){
  const atoms=[];let duration=null;
  function scan(start,end){
    for(let offset=start;offset+8<=end;){
      let size=buffer.readUInt32BE(offset),header=8;const type=buffer.toString('ascii',offset+4,offset+8);
      if(size===1){size=Number(buffer.readBigUInt64BE(offset+8));header=16;}if(size===0)size=end-offset;
      if(size<header||offset+size>end)throw new Error('Invalid MP4 atom');atoms.push(type);
      if(type==='moov')scan(offset+header,offset+size);
      if(type==='mvhd'){
        const data=offset+header,version=buffer[data],timescale=buffer.readUInt32BE(data+(version===1?20:12));
        const ticks=version===1?Number(buffer.readBigUInt64BE(data+24)):buffer.readUInt32BE(data+16);duration=ticks/timescale;
      }
      offset+=size;
    }
  }
  scan(0,buffer.length);return {bytes:buffer.length,atoms,duration};
}

try{
  const baseline=await jsonl('work/validation/r1-dynamic-grasp-3x.jsonl');
  check('baseline_36_passes',baseline.length===36&&baseline.every(r=>r.pass===true),{cases:baseline.length,passed:baseline.filter(r=>r.pass).length});
  const baseSort=baseline.filter(r=>!r.scenario.stack&&!r.scenario.offset&&!r.scenario.yaw&&!r.scenario.roll);
  const coverage=[];
  for(const arm of ['right','left'])for(const id of ['red','blue','yellow']){
    const rows=baseSort.filter(r=>r.scenario.arm===arm&&r.scenario.id===id),repeats=new Set(rows.map(r=>r.scenario.repeat));
    const valid=rows.every(r=>r.evidence.dual&&r.evidence.lift>.035&&r.evidence.maxSpeed<1&&r.measurement.table&&r.measurement.distance<.02);
    coverage.push({arm,id,cases:rows.length,repeats:[...repeats],valid});
  }
  check('six_sort_combinations_three_passes_each',coverage.every(c=>c.cases===3&&c.repeats.every(n=>[1,2,3].includes(n))&&c.repeats.length===3&&c.valid),coverage);
  const stacks=baseline.filter(r=>r.scenario.stack);
  check('both_arms_stack_three_passes_each',stacks.length===6&&['right','left'].every(arm=>stacks.filter(r=>r.scenario.arm===arm).length===3)&&stacks.every(r=>r.stackStableSeconds>=3&&r.stackEvidence.length===3&&r.stackEvidence.every(e=>e.dual&&e.lift>.035)&&r.stackcheck.stable&&r.stackcheck.states.every(s=>s.supported&&s.speed<.0175&&s.error<.012)),{cases:stacks.length,stableSeconds:stacks.map(r=>r.stackStableSeconds)});
  const edited=baseline.filter(r=>r.scenario.offset||r.scenario.yaw||r.scenario.roll);
  check('edited_position_and_rotation_cases',edited.length===12&&edited.every(r=>r.pass&&r.evidence.dual&&r.evidence.lift>.035),{cases:edited.length});

  const final=await jsonl('work/validation/r1-dynamic-final-stack-recovery.jsonl');
  check('final_six_cases_pass',final.length===6&&final.every(r=>r.pass===true),final.map(r=>r.scenario));
  const hashResults=[];
  for(const [path,expected] of Object.entries(final[0]?.sourceHashes||{})){
    const actual=createHash('sha256').update(await readFile(new URL(path,root))).digest('hex');
    hashResults.push({path,expected,actual,match:expected===actual&&final.every(r=>r.sourceHashes?.[path]===expected)});
  }
  check('final_physics_source_hashes_match_current',hashResults.length===5&&hashResults.every(r=>r.match),hashResults);
  const finalStacks=final.filter(r=>r.scenario.stack),recoveries=final.filter(r=>r.scenario.recovery);
  check('final_both_arms_full_physical_stack',finalStacks.length===2&&new Set(finalStacks.map(r=>r.scenario.arm)).size===2&&finalStacks.every(r=>r.stackStableSeconds>=3&&r.stackcheck.stable&&r.stackEvidence.length===3&&r.stackEvidence.every(e=>e.dual&&e.lift>.035)),finalStacks.map(r=>({arm:r.scenario.arm,stableSeconds:r.stackStableSeconds,maxWristError:r.maxWristError})));
  check('failure_stop_reset_recoveries',recoveries.length===4&&['unreachable_stack','failure_reset_then_sort','stop_stack_mid_transfer','stop_happy_mid_gesture'].every(name=>recoveries.some(r=>r.scenario.recovery===name&&r.pass)),recoveries.map(r=>({scenario:r.scenario,reason:r.reason,state:r.state})));

  const trace=await json('work/validation/stack-screen-trace.json');
  check('screen_trace_continuous_frames',trace.length>=100&&trace.every((f,i)=>f.frame===i&&finite(f.simTime)&&(!i||(f.at>=trace[i-1].at&&f.simTime>=trace[i-1].simTime))),{frames:trace.length,simulationStart:trace[0]?.simTime,simulationEnd:trace.at(-1)?.simTime});
  const itemEvidence=[];
  for(const [itemIndex,id] of ['blue','red','yellow'].entries()){
    const rows=trace.filter(f=>f.itemIndex===itemIndex&&f.selected===id),positions=stageKeys.map(state=>rows.findIndex(f=>f.state===state));
    const releasedSupported=rows.some(f=>['release','retreat','settling'].includes(f.state)&&f.stackCheck?.states.find(s=>s.id===id)?.supported);
    const lift=Math.max(...rows.map(f=>f.evidence?.lift||0)),dual=rows.some(f=>f.evidence?.dual);
    itemEvidence.push({itemIndex,id,stages:Object.fromEntries(stageKeys.map((state,i)=>[state,positions[i]])),ordered:positions.every((p,i)=>p>=0&&(!i||p>positions[i-1])),lift,dual,releasedSupported});
  }
  check('screen_all_three_real_grasp_stages',itemEvidence.every(i=>i.ordered&&i.dual&&i.lift>.035&&i.releasedSupported),itemEvidence);
  const firstReaction=trace.find(f=>f.reaction==='happy'||f.state==='happy'),firstHappy=trace.find(f=>f.state==='happy'),complete=trace.at(-1);
  let stableStart=null;for(const f of trace){if(f.stackCheck?.stable){if(stableStart===null)stableStart=f.simTime;}else stableStart=null;if(f===firstReaction)break;}
  const observedStable=firstReaction&&stableStart!==null?firstReaction.simTime-stableStart:0;
  check('screen_success_only_after_three_seconds_supported',firstReaction?.stackStableSeconds>=3&&observedStable>=3&&firstHappy?.stackCheck?.stable&&complete?.state==='complete'&&complete.running===false&&complete.stackCheck?.stable&&complete.stackStableSeconds>=3,{firstHappyTime:firstHappy?.simTime,firstReactionTime:firstReaction?.simTime,observedStableSeconds:observedStable,stableCounter:firstReaction?.stackStableSeconds,finalState:complete?.state,finalSupports:complete?.stackCheck?.states});
  check('screen_physics_rms_finite',trace.every(f=>finite(f.rms)&&f.rms>=0),{maximumRms:Math.max(...trace.map(f=>f.rms))});

  const progress=await json('work/validation/loading-progress.json');
  check('loading_progress_monotonic_to_100',progress.length>3&&progress.every((p,i)=>finite(p.percent)&&p.percent>=0&&p.percent<=100&&(!i||p.percent>=progress[i-1].percent))&&progress.at(-1).percent===100&&new Set(progress.map(p=>p.percent)).size>10,{samples:progress.length,start:progress[0]?.percent,end:progress.at(-1)?.percent,distinct:new Set(progress.map(p=>p.percent)).size});
  const screenshots=[];
  for(const mode of ['desktop','narrow']){
    const reference=jpegSize(await readFile(new URL(`work/validation/screenshots/reference-${mode}.jpg`,root))),r1=jpegSize(await readFile(new URL(`work/validation/screenshots/r1-${mode}.jpg`,root)));
    screenshots.push({mode,reference,r1,sameViewport:reference.width===r1.width&&reference.height===r1.height});
  }
  check('paired_screenshots_same_viewport',screenshots.every(s=>s.sameViewport),screenshots);
  const video=mp4Info(await readFile(new URL('work/validation/media/r1-stack-complete.mp4',root)));
  check('stack_operation_video_valid_container',video.bytes>10000&&['ftyp','moov','mdat'].every(atom=>video.atoms.includes(atom))&&video.duration>=45,{bytes:video.bytes,duration:video.duration,atoms:video.atoms});

  const browser=await json('work/validation/browser-regressions.json');
  check('browser_manual_joint_and_guides',finite(browser.jointKeyboard?.torque)&&Math.abs(browser.jointKeyboard?.target)>0&&Math.abs(browser.jointKeyboard?.error)<.001&&browser.guides?.lines>=6&&browser.guides?.pressed==='true',{joint:browser.jointKeyboard,guides:browser.guides});
  check('browser_tcp_drag_finishes_and_restores_orbit',browser.tcp?.tcp?.visible&&browser.tcp?.drag===null&&browser.tcp?.orbit===true&&browser.tcp?.joints?.length===6&&browser.tcp.joints.every(j=>finite(j.target)&&finite(j.actual)&&Math.abs(j.target-j.actual)<.003),browser.tcp);
  check('browser_gripper_closed',browser.closed?.length===2&&browser.closed.every(j=>j.target===0&&Math.abs(j.actual)<.002&&j.effortUnit==='N'),browser.closed);
  check('browser_camera_controls',browser.cameraSwitch?.column===true&&browser.cameraSwitch?.feedsHidden===true&&browser.cameraSwitch?.modelButton==='false',browser.cameraSwitch);
  const blue=browser.objectW?.objects?.find(o=>o.id==='blue'),dragBlue=browser.objectDrag?.objects?.find(o=>o.id==='blue');
  check('browser_object_move_separate_from_orbit',blue?.position[0]>.78&&distance(browser.objectW?.camera||[],browser.blankOrbit?.camera||[])>.1&&browser.blankOrbit?.orbitEnabled===true&&browser.blankOrbit?.drag===null&&browser.blankOrbit.objects.every(o=>distance(o.position,browser.objectW.objects.find(p=>p.id===o.id).position)<.0001),{keyboardBlue:blue,orbitCamera:browser.blankOrbit?.camera});
  check('browser_object_drag_and_rotation',dragBlue&&distance(dragBlue.position,[.78,.711,0])>.01&&browser.objectDrag?.orbit===true&&browser.objectDrag?.drag===null&&browser.objectRotate?.drag==='rotate'&&browser.objectRotate?.orbit===false,{drag:browser.objectDrag,rotate:browser.objectRotate});
  check('browser_narrow_layout_and_control_dismissal',browser.narrow?.scrollWidth===browser.narrow?.width&&browser.narrow?.panel?.width===390&&browser.narrowChip?.open===true&&browser.narrowChip?.slider?.right<=browser.narrowChip?.width&&browser.outsideClose===true,{narrow:browser.narrow,narrowChip:browser.narrowChip,outsideClose:browser.outsideClose});
  check('browser_gripper_close_then_reopen',browser.finalClosed?.length===2&&browser.finalOpened?.length===2&&browser.finalClosed.every(j=>j.target===0&&Math.abs(j.actual)<.002)&&browser.finalOpened.every(j=>j.target===.05&&Math.abs(j.actual-.05)<.002),{closed:browser.finalClosed,opened:browser.finalOpened});
  check('browser_left_arm_and_wrist_camera',browser.leftArm?.names?.length===6&&browser.leftArm.names.every((name,i)=>name===`left_arm_joint${i+1}`)&&browser.leftArm?.wrist?.position?.length===3&&browser.leftArm.wrist.position.every(finite)&&browser.leftArm.wrist.direction.every(finite)&&finite(browser.leftArm.wrist.fov),browser.leftArm);
  check('browser_stop_and_reset_states',browser.stopped?.state==='stopped'&&browser.stopped.running===false&&browser.stopped.reaction===null&&browser.finalReset?.state==='idle'&&browser.finalReset.running===false&&browser.finalReset.history?.length===0,{stopped:browser.stopped,reset:browser.finalReset});
  // Report newly supplied browser recovery observations without substituting
  // simulated physics checks for actual browser input evidence.
  details.additionalBrowserEvidence=Object.fromEntries(Object.entries(browser).filter(([key])=>!['jointKeyboard','guides','tcp','closed','cameraSwitch','objectW','blankOrbit','objectDrag','objectRotate','narrow','narrowChip','outsideClose','finalClosed','finalOpened','leftArm','stopped','finalReset'].includes(key)));
  details.artifactRoot=fileURLToPath(new URL('work/validation/',root));
  details.evidenceScope={baseline:'36 saved simulations before reaction-centering adjustment',final:'Current-source-hashed two-arm full stacks and four recovery scenarios',screen:'Saved browser trace, screenshots and R1 operation video',touchHardwareTested:false};
  details.limitations=['Physical touchscreen hardware input was not tested.','Screenshot checks verify file dimensions and pairing; visual fidelity still requires human/rendered-image review.','The R1 operation recording is verified as an MP4 container with duration; this script does not inspect every decoded video frame.'];
}catch(error){check('evidence_read_or_parse',false,{error:error.message});}
const passed=checks.filter(c=>c.pass).length,failed=checks.filter(c=>!c.pass);
console.log(JSON.stringify({passed:failed.length===0,checksPassed:passed,checksTotal:checks.length,checks,details},null,2));
if(failed.length)process.exitCode=1;
