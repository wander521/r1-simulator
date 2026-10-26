/** Read-only physical evidence, independent of the bookkeeping ledger.
 * Contacts require positive solver force; proximity is never called support. */
export function supportPairs(model,data){
  const pairs=new Set(),list=data.contact;
  try{for(let i=0;i<Math.min(data.ncon,list.size());i++){
    const c=list.get(i);try{
      if(c.efc_address<0||data.efc_force[c.efc_address]<=.005)continue;
      const a=model.geom_bodyid[c.geom1],b=model.geom_bodyid[c.geom2];pairs.add(`${a}:${b}`);pairs.add(`${b}:${a}`);
    }finally{c.delete();}
  }}finally{list.delete();}return pairs;
}

export function verifyBinInventory({metadata,pipeline,model,data,bodyId,job}){
  const pairs=supportPairs(model,data),checks=[];
  for(const part of metadata.parts){
    const record=pipeline.parts.get(part.id),newlyPacked=job.type==='pack'&&part.id===job.partId;
    const packed=record.packed||newlyPacked;
    // Other cells may be moving a different unprocessed bin concurrently.
    if(!packed&&part.binId!==job.binId)continue;
    const b=bodyId(part.body),bin=pipeline.bins.get(part.binId);
    const expected=packed?`fixture_${newlyPacked?job.destination:record.location}`:metadata.bins.find(x=>x.id===part.binId).body;
    let withinDestination=true;
    if(packed){
      const destination=newlyPacked?job.destination:record.location;
      const slot=metadata.slots.find(s=>s.id===destination),index=part.outputIndex??bin.parts.indexOf(part.id),p=slot?.partSlots[index];
      const categoryMatches=part.outputSlot===undefined||(destination===part.outputSlot&&slot?.category===part.category&&(!newlyPacked||job.partIndex===index));
      withinDestination=Boolean(categoryMatches&&p&&Math.hypot(...p.map((v,k)=>data.xpos[b*3+k]-v))<.025);
    }
    const supported=pairs.has(`${b}:${bodyId(expected)}`);
    checks.push({partId:part.id,supported,withinDestination,supportBody:expected});
  }
  return {valid:checks.every(c=>c.supported&&c.withinDestination),checks};
}

/** Hand-off access is a spatial lease. Check actual collision shapes against
 * the destination bench's contact volume before releasing its reservation.
 * The conservative bounding sphere also covers rotated geom orientations. */
export function verifyHandoffClearance({api,model,data,metadata,robotId,zone,objectBodies=[]}){
  const supportName={'g1-out':'g1_out_bench','c5-in':'c5_input_bench','empty-return':'empty_return_bench'}[zone];
  const support=metadata.supports.find(s=>s.body===supportName);
  if(!support)throw new Error(`Unknown handoff zone: ${zone}`);
  const lo=[support.center[0]-support.halfExtents[0]-.025,support.center[1]-support.halfExtents[1]-.025,support.tableHeight-.02];
  const hi=[support.center[0]+support.halfExtents[0]+.025,support.center[1]+support.halfExtents[1]+.025,support.tableHeight+.29];
  const blocked=[];
  for(let i=0;i<model.ngeom;i++){
    if(!model.geom_contype[i]&&!model.geom_conaffinity[i])continue;
    const name=api.mj_id2name(model,api.mjtObj.mjOBJ_GEOM.value,i);
    const body=objectBodies.length?api.mj_id2name(model,api.mjtObj.mjOBJ_BODY.value,model.geom_bodyid[i]):null;
    if(!name?.startsWith(`${robotId}__`)&&!objectBodies.includes(body))continue;
    const p=data.geom_xpos.subarray(i*3,i*3+3),r=model.geom_rbound[i];
    const separation=Math.hypot(...Array.from(p,(v,k)=>Math.max(lo[k]-v,0,v-hi[k])));
    if(separation<=r)blocked.push(name);
  }
  return {clear:blocked.length===0,blocked,zone,robotId};
}
