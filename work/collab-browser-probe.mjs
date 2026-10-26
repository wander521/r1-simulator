/** Temporary browser acceptance observer. Install through the development
 * browser interface, then use the normal start button. It never starts the
 * engine, writes physics, changes clocks, or accelerates the simulation. */
export function installCollaborationBrowserProbe(){
  if(window.__collabAcceptance)throw new Error('An acceptance observer is already installed');
  if(!window.__collabDiagnostics?.().ready)throw new Error('Workcell is not ready');
  const probe={installedAt:Date.now(),installedPerformance:performance.now(),firstRunningAt:null,samples:[],frameDeltas:[],longTasks:[],errors:[],live:true};
  let previousFrame=null,animationFrame=0;
  function capture(){
    const d=window.__collabDiagnostics(),s=d.snapshot;
    if(s?.state==='running'&&probe.firstRunningAt===null)probe.firstRunningAt=Date.now();
    probe.samples.push({wallAt:Date.now(),performanceAt:performance.now(),visible:document.visibilityState,
      ready:d.ready,error:d.error??s?.error,commandError:s?.commandError,state:s?.state,elapsedSeconds:s?.elapsedSeconds,simTime:s?.simTime,
      completedOrders:s?.completedOrders,completedParts:s?.completedParts,currentOrder:s?.currentOrder,
      activeRobot:s?.activeRobot,activePart:s?.activePart,phase:s?.phase,phaseLabel:s?.phaseLabel,
      robots:s?.robots?.map(({id,enabled,state,force,torque,basePose,job,phase,error,active})=>({id,enabled,state,force,torque,basePose,job,phase,error,active})),
      parts:s?.parts?.map(({id,body,location,position,quaternion})=>({id,body,location,position,quaternion})),
      trays:s?.trays?.map(({orderId,body,position})=>({orderId,body,position})),
      bins:s?.bins?.map(({id,body,location,position,quaternion,packed})=>({id,body,location,position,quaternion,packed})),
      stats:s?.stats,camera:d.camera,evidenceCount:s?.evidence?.length??0});
  }
  function frame(time){
    if(!probe.live)return;
    if(previousFrame!==null)probe.frameDeltas.push(time-previousFrame);
    previousFrame=time;animationFrame=requestAnimationFrame(frame);
  }
  const onError=event=>probe.errors.push({wallAt:Date.now(),type:'error',message:event.message});
  const onRejection=event=>probe.errors.push({wallAt:Date.now(),type:'unhandledrejection',message:String(event.reason?.message??event.reason)});
  window.addEventListener('error',onError);window.addEventListener('unhandledrejection',onRejection);
  let observer=null;
  if(PerformanceObserver.supportedEntryTypes.includes('longtask')){
    observer=new PerformanceObserver(list=>{for(const e of list.getEntries())probe.longTasks.push({startTime:e.startTime,duration:e.duration});});
    observer.observe({type:'longtask',buffered:false});
  }
  const timer=setInterval(capture,1000);capture();animationFrame=requestAnimationFrame(frame);
  window.__collabAcceptance=probe;
  window.__stopCollabAcceptance=()=>{
    capture();probe.live=false;clearInterval(timer);cancelAnimationFrame(animationFrame);observer?.disconnect();
    window.removeEventListener('error',onError);window.removeEventListener('unhandledrejection',onRejection);
    const result={...probe,finishedAt:Date.now(),finishedPerformance:performance.now(),final:window.__collabDiagnostics()};
    delete window.__collabAcceptance;delete window.__stopCollabAcceptance;
    return result;
  };
  return {installedAt:probe.installedAt,initialState:probe.samples[0].state};
}
// Deliberately development-only, installed by Playwright addScriptTag.
if(typeof window!=='undefined')window.installCollaborationBrowserProbe=installCollaborationBrowserProbe;
