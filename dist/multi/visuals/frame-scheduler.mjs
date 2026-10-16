// Wall time may be discarded under load; physical time is never coarsened.
// Every callback advances ALL independent worlds by exactly one 1 ms tick.
export function createFrameScheduler({now=()=>performance.now(),budgetMs=8,maxSteps=24,maxBacklog=.05}={}){
  let previous=null,accumulator=0,droppedWallSeconds=0;
  return{
    reset(timestamp=null){previous=timestamp;accumulator=0;},
    advance(timestamp,{paused=false,step,planning=()=>false}={}){
      if(!Number.isFinite(timestamp))throw new TypeError('Frame time must be finite');
      const elapsed=previous===null?0:Math.max(0,(timestamp-previous)/1000);previous=timestamp;
      if(paused||planning()){accumulator=0;return{steps:0,backlogSeconds:0,droppedWallSeconds};}
      const pending=accumulator+elapsed;
      accumulator=Math.min(pending,maxBacklog);droppedWallSeconds+=Math.max(0,pending-maxBacklog);
      const start=now();let steps=0;
      while(accumulator+1e-12>=.001&&steps<maxSteps&&now()-start<budgetMs){
        step(.001);steps++;accumulator=Math.max(0,accumulator-.001);
        if(planning()){accumulator=0;break;}
      }
      return{steps,backlogSeconds:accumulator,droppedWallSeconds};
    },
    get backlogSeconds(){return accumulator;},
    get droppedWallSeconds(){return droppedWallSeconds;},
  };
}

export function cameraFeedVisible(rect,{width,height}){
  return Boolean(rect&&rect.width>0&&rect.height>0&&rect.bottom>0&&rect.right>0&&rect.top<height&&rect.left<width);
}

// The task does not own the user's camera. Resize changes projection only.
export function shouldRefitCamera(reason){
  return ['initial','participants','panorama','focus'].includes(reason);
}
