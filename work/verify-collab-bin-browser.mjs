import {readFile,writeFile} from 'node:fs/promises';
const input=process.argv[2];if(!input)throw new Error('Usage: node work/verify-collab-bin-browser.mjs report.json [summary.json]');
const r=JSON.parse(await readFile(input)),samples=r.samples??[],f=r.final?.snapshot,first=samples.find(s=>s.state==='running'),end=samples.find(s=>s.state==='complete');
const checks=[],check=(name,pass,detail)=>checks.push({name,pass:Boolean(pass),detail});
check('Normal observed run and complete state',first&&end&&f?.state==='complete');
check('At least 300 real seconds before completion',end&&first&&(end.performanceAt-first.performanceAt)>=300000);
check('Admission cutoff at 300 s',f?.elapsedSeconds>=300&&f.elapsedSeconds<301,f?.elapsedSeconds);
check('No browser/runtime/station errors',!r.errors?.length&&!samples.some(s=>s.error||s.commandError||s.robots?.some(a=>a.error)),r.errors);
check('Page remained foreground',samples.length>300&&samples.every(s=>s.visible==='visible'));
check('Camera never auto-moved',samples.every(s=>JSON.stringify(s.camera)===JSON.stringify(first?.camera)));
check('All original bins and parts accounted for',f?.bins?.length===4&&f?.parts?.length===8&&new Set(f.parts.map(p=>p.id)).size===8&&f.completedParts===8&&f.returnedBins===4&&f.bins.every(b=>b.location.startsWith('return-')&&b.packed===2));
const identity=sample=>JSON.stringify(['bins','parts'].map(kind=>(sample?.[kind]??[]).map(o=>[o.id,o.body]).sort((a,b)=>a[0].localeCompare(b[0]))));
check('Physical object identities unchanged in every sample',first&&samples.every(s=>identity(s)===identity(first))&&identity(f)===identity(first));
const placements=(f?.parts??[]).map(part=>{
  const bin=f.bins.find(b=>b.id===part.binId),slot=f.metadata.slots.find(s=>s.id===part.location),index=part.outputIndex??bin?.parts.indexOf(part.id),p=slot?.partSlots[index];
  return {id:part.id,error:p?Math.hypot(part.position[0]-p[0],part.position[1]-p[2],part.position[2]+p[1]):Infinity};
});
check('Finished parts remain at measured output positions',placements.length===8&&placements.every(p=>p.error<.025),placements);
if(f?.parts?.some(p=>p.category)){
  const slots=f.metadata.slots.filter(s=>s.kind==='output');
  check('Four separate categories with two distinct physical positions each',slots.length===4&&new Set(slots.map(s=>s.category)).size===4&&slots.every(s=>{
    const members=f.parts.filter(p=>p.location===s.id);
    return members.length===2&&members.every(p=>p.outputSlot===s.id&&p.category===s.category&&p.color===s.color&&p.shape===s.shape)&&new Set(members.map(p=>p.outputIndex)).size===2;
  }));
}
const evidence=f?.evidence??[],moves=evidence.filter(e=>['fetch','deliver','pack','return-empty'].includes(e.type));
check('Every physical manipulation has contact/lift/support/inventory evidence',moves.length===20&&moves.every(e=>e.ready&&e.dualContact&&e.lifted&&e.supported&&e.inventory?.valid),{count:moves.length});
check('Independent planning and motion overlap',f?.stats?.planningMotionOverlapSteps>0&&f.stats.multiRobotMotionSteps>0&&f.stats.maxMovingRobots===3,f?.stats);
check('All arms returned home',f?.robots?.length===3&&f.robots.every(a=>a.state==='home'&&!a.active));
const origin=f?.metadata?.robots?.r1?.origin,base=f?.robots?.find(a=>a.id==='r1')?.basePose,expected=origin&&[origin[0],origin[2],-origin[1]],baseError=expected&&base?Math.hypot(...expected.map((v,i)=>v-base.position[i])):Infinity;
check('R1 mobile base also returned home',baseError<.004,{errorMetres:baseError});
const frames=[...(r.frameDeltas??[])].sort((a,b)=>a-b),p95=frames[Math.floor(frames.length*.95)],maxLong=Math.max(0,...(r.longTasks??[]).map(t=>t.duration));
check('Responsive foreground render',frames.length>1000&&p95<50&&maxLong<250,{p95FrameMs:p95,maxFrameMs:frames.at(-1),longTasks:r.longTasks?.length,maxLongTaskMs:maxLong});
const report={scope:'Read-only real-browser 300 s admission plus drain; desktop session only, not hardware safety or every-device performance',pass:checks.every(c=>c.pass),durationUntilCompleteSeconds:end&&first?(end.performanceAt-first.performanceAt)/1000:null,checks};
if(process.argv[3])await writeFile(process.argv[3],JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));if(!report.pass)process.exitCode=1;
