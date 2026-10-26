const durations={descend:1.7,lift:1.8,transfer:2,place:1.8,retreat:1.6};
const common={approach:'descend',descend:'closing',closing:'lift',lift:'transfer',transfer:'place',place:'release',release:'retreat',retreat:'settling'};
const smooth=t=>{const u=Math.max(0,Math.min(1,t));return u*u*(3-2*u);};

/** Describe only the unexecuted tail. This never restarts a pick, rewinds to a
 * path's first sample, changes elapsed time, or writes physical state. */
export function remainingMotionSegments({robotId,state,elapsed=0,plan,pose,placePose,phaseTarget,pickOnly=false}){
  if(!plan)throw new Error('恢复动作缺少已验证路径');
  const next={...common,...(robotId==='g1'?{
    spread:'raise',raise:'ready',ready:'align',recenter:'align',align:'approach',settling:'return_clear',return_clear:'return_align',return_align:'operation_ready',
  }:{spread:'crouch',crouch:'align',align_clear:'align',align:'approach',settling:'operation_return',operation_return:'operation_ready'})};
  if(pickOnly)next.lift=null;
  const known=new Set([...Object.keys(next),'operation_ready']);
  if(!known.has(state))throw new Error(`无法安全恢复未知阶段：${state}`);
  const segments=[];let phase=state;
  for(let guard=0;phase&&guard<24;guard++,phase=next[phase]){
    const held=['lift','transfer','place'].includes(phase),released=['retreat','settling','return_clear','return_align','operation_return','operation_ready'].includes(phase);
    const add=to=>segments.push({phase,pose:to,objectPhase:held?phase:released?'retreat':null});
    const path=plan[phase];
    if(durations[phase]){
      if(!Array.isArray(path)||path.length<2)throw new Error(`恢复路径不完整：${phase}`);
      if(phase===state&&phase==='place'&&placePose)add(placePose);
      else{
        const first=phase===state?Math.min(path.length-1,Math.floor(smooth(elapsed/durations[phase])*(path.length-1))+1):0;
        for(let i=first;i<path.length;i++)add(path[i]);
      }
    }else if(!['closing','release','settling','operation_ready'].includes(phase)){
      const target=phaseTarget(phase,segments.at(-1)?.pose??pose);
      if(!target)throw new Error(`恢复动作缺少关节目标：${phase}`);
      add(target);
    }
  }
  return segments;
}

export function requiresHeldContact(action,state){
  return Boolean(action.navigation?.carry||['need-outbound','clear-source','await-destination','await-destination-zone','need-destination-dock','need-place'].includes(action.phase)||['lift','transfer','place','transport_hold'].includes(state));
}
