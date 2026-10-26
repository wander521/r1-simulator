import {IDS,LABELS,bodyPose} from './state.mjs';

/** Rendering consumes measured transforms only. No body teleport or separate
 * per-robot simulation; the ledger is joined by immutable object IDs. */
export function binPresentationSnapshot(core){
  const raw=core.snapshot,{ledger}=raw,byBin=new Map(ledger.bins.map(b=>[b.id,b])),byPart=new Map(ledger.parts.map(p=>[p.id,p]));
  const errors=Object.values(ledger.robots).filter(r=>r.error);
  return {...raw,state:ledger.state,error:errors.map(r=>`${LABELS[r.id]}：${r.error}`).join('；')||null,
    elapsedSeconds:ledger.elapsedSeconds,durationSeconds:ledger.durationSeconds,
    completedOrders:ledger.bins.filter(b=>b.packed===b.parts.length).length,completedParts:ledger.parts.filter(p=>p.packed).length,
    returnedBins:ledger.bins.filter(b=>b.location.startsWith('return-')).length,
    robots:IDS.map(id=>{
      const a=core.agents.get(id),c=a.controller,action=core.actions.get(id),robot=ledger.robots[id];
      return {id,label:LABELS[id],enabled:robot.enabled,state:action?.failed?'failed':action?.pending?'planning':c.state,error:robot.error,
        active:Boolean(action),job:action?.job??null,phase:action?.phase??null,heldCheck:action?.resume!==undefined,
        basePose:bodyPose(core.data,core.id('BODY',core.metadata.robots[id].mountName)),
        pose:Object.fromEntries([...a.dynamics.map].map(([name,j])=>[name,core.data.qpos[j.q]])),force:a.measure().force};
    }),
    bins:core.metadata.bins.map(b=>({...b,...byBin.get(b.id),...bodyPose(core.data,core.objectBodies.get(b.id))})),
    parts:core.metadata.parts.map(p=>({...p,...byPart.get(p.id),...bodyPose(core.data,core.objectBodies.get(p.id))})),
    events:ledger.events.slice(-18),metadata:core.metadata};
}
