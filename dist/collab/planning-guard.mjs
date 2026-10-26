// Read-only checks for a trajectory computed from an older private world.
// Quaternion signs are equivalent; rotation in place is not equivalent.
export function sameFreeBodyPose(current, snapshot, offset, {positionTolerance=.004, angleTolerance=.02, axialSymmetry=false}={}){
  const a=Array.from(current.slice(offset,offset+7)),b=Array.from(snapshot.slice(offset,offset+7));
  if(a.length!==7||b.length!==7||![...a,...b].every(Number.isFinite))return false;
  if(Math.hypot(...a.slice(0,3).map((v,i)=>v-b[i]))>positionTolerance)return false;
  const qa=a.slice(3),qb=b.slice(3),na=Math.hypot(...qa),nb=Math.hypot(...qb);
  if(na<1e-9||nb<1e-9)return false;
  if(axialSymmetry){
    // A cylinder's spin around its own Z axis does not change collision
    // geometry. Tilt still invalidates the trajectory; this is not a wider
    // angular tolerance for boxes or the carried handle.
    const axis=(q,n)=>{const [w,x,y,z]=q.map(v=>v/n);return [2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)];};
    const za=axis(qa,na),zb=axis(qb,nb);
    return Math.acos(Math.min(1,Math.abs(za.reduce((sum,v,i)=>sum+v*zb[i],0))))<=angleTolerance;
  }
  const cosine=Math.min(1,Math.abs(qa.reduce((sum,v,i)=>sum+v*qb[i],0)/(na*nb)));
  return 2*Math.acos(cosine)<=angleTolerance;
}

export function guardedBodies(target){
  return target?[...new Set([target.body,...(target.payloadBodies??[])].filter(Boolean))]:[];
}
