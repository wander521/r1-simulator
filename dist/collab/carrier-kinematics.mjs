import * as THREE from '../vendor/three.module.min.js';
import {armJointNames} from '../g1/g1-profile.mjs';
import {tcpPose,naturalArmLimits} from '../g1/grasp-kinematics.mjs';

function rotationError(target,actual){
  const q=target.clone().multiply(actual.clone().invert()).normalize();
  if(q.w<0)q.set(-q.x,-q.y,-q.z,-q.w);
  const s=Math.hypot(q.x,q.y,q.z),factor=s<1e-9?2:2*Math.atan2(s,q.w)/s;
  return [q.x*factor,q.y*factor,q.z*factor];
}
function linearSolve(a,b){
  const m=a.map((row,i)=>[...row,b[i]]);
  for(let k=0;k<b.length;k++){
    let pivot=k;for(let i=k+1;i<b.length;i++)if(Math.abs(m[i][k])>Math.abs(m[pivot][k]))pivot=i;
    [m[k],m[pivot]]=[m[pivot],m[k]];if(Math.abs(m[k][k])<1e-12)return null;
    const divisor=m[k][k];for(let j=k;j<=b.length;j++)m[k][j]/=divisor;
    for(let i=0;i<b.length;i++)if(i!==k){const scale=m[i][k];for(let j=k;j<=b.length;j++)m[i][j]-=scale*m[k][j];}
  }
  return m.map(row=>row[b.length]);
}

/** Bin extraction constrains orientation for loose contents. Solve waist yaw
 * together with the arm instead of prescribing yaw from a point-only heuristic.
 * Feet, waist roll and pitch stay fixed. This solver only writes the planning
 * URDF tree; it does not command physics or bypass swept collision validation. */
export function solveCarrierPose(robot,arm,target,orientation,seed,iterations=240){
  const names=['waist_yaw_joint',...armJointNames(arm)],limits=[[-1,1],...naturalArmLimits(arm)],pose={...seed};
  const bounds=names.map((name,i)=>({lower:Math.max(robot.joints[name].limit.lower,limits[i][0])+.001,upper:Math.min(robot.joints[name].limit.upper,limits[i][1])-.001}));
  names.forEach((name,i)=>pose[name]=THREE.MathUtils.clamp(pose[name]??0,bounds[i].lower,bounds[i].upper));
  const evaluate=()=>{robot.setJointValues(pose);robot.updateMatrixWorld(true);return tcpPose(robot,arm);};
  let best={...pose},score=Infinity;
  for(let iteration=0;iteration<iterations;iteration++){
    const current=evaluate(),p=target.clone().sub(current.position),r=rotationError(orientation,current.quaternion),weight=.18;
    const value=p.length()+weight*Math.hypot(...r);
    if(value<score){score=value;best={...pose};}
    if(p.length()<.00035&&Math.hypot(...r)<.0035)break;
    const error=[...p.toArray(),...r.map(v=>v*weight)];
    const columns=names.map(name=>{
      const joint=robot.joints[name],axis=joint.axis.clone().applyQuaternion(joint.getWorldQuaternion(new THREE.Quaternion())).normalize();
      const velocity=new THREE.Vector3().crossVectors(axis,current.position.clone().sub(joint.getWorldPosition(new THREE.Vector3())));
      return [...velocity.toArray(),...axis.multiplyScalar(weight).toArray()];
    });
    // An unconstrained DLS update repeatedly asks a saturated waist to rotate
    // farther, then clipping throws away that contribution. Re-solve using
    // the remaining joints instead of accepting a persistent millimetre error.
    // Limits are unchanged; a joint can re-enter when the update points inward.
    const active=new Set(names.map((_,i)=>i));let steps=null;
    for(let pass=0;pass<=names.length;pass++){
      const matrix=Array.from({length:6},(_,i)=>Array.from({length:6},(_,j)=>[...active].reduce((sum,k)=>sum+columns[k][i]*columns[k][j],0)+(i===j?.00006:0)));
      const delta=linearSolve(matrix,error);if(!delta)break;
      steps=columns.map((column,i)=>active.has(i)?column.reduce((sum,v,k)=>sum+v*delta[k],0):0);
      const blocked=[...active].filter(i=>(pose[names[i]]<=bounds[i].lower+1e-7&&steps[i]<0)||(pose[names[i]]>=bounds[i].upper-1e-7&&steps[i]>0));
      if(!blocked.length)break;for(const i of blocked)active.delete(i);
    }
    if(!steps)break;
    names.forEach((name,i)=>{
      const step=THREE.MathUtils.clamp(steps[i],-.10,.10);
      pose[name]=THREE.MathUtils.clamp(pose[name]+step,bounds[i].lower,bounds[i].upper);
    });
  }
  Object.assign(pose,best);const final=evaluate();
  return {pose,positionError:final.position.distanceTo(target),angleError:Math.hypot(...rotationError(orientation,final.quaternion))};
}
