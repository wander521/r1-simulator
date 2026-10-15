import * as THREE from '../vendor/three.module.min.js';
import {armJointNames,gripperLinkName,GRIPPER_TCP_X} from './g1-profile.mjs';

// Dex1's contact center is measured from the supplied collision meshes, not
// from the wrist origin or the end of the visible shell.
export const TCP = new THREE.Vector3(GRIPPER_TCP_X,0,0);
export function tcpPose(robot,arm){
  const link=robot.getObjectByName(gripperLinkName(arm));
  return {position:link.localToWorld(TCP.clone()),quaternion:link.getWorldQuaternion(new THREE.Quaternion())};
}
export function topDownQuaternion(yaw=0){
  const x=new THREE.Vector3(0,-1,0),y=new THREE.Vector3(Math.cos(yaw),0,Math.sin(yaw));
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x,y,new THREE.Vector3().crossVectors(x,y)));
}
function rotationError(target,actual){
  const q=target.clone().multiply(actual.clone().invert()).normalize();
  if(q.w<0)q.set(-q.x,-q.y,-q.z,-q.w);
  const s=Math.hypot(q.x,q.y,q.z),a=s<1e-9?2:2*Math.atan2(s,q.w)/s;
  return [q.x*a,q.y*a,q.z*a];
}
function linearSolve(a,b){
  const m=a.map((row,i)=>[...row,b[i]]);
  for(let k=0;k<b.length;k++){
    let pivot=k;for(let i=k+1;i<b.length;i++)if(Math.abs(m[i][k])>Math.abs(m[pivot][k]))pivot=i;
    [m[k],m[pivot]]=[m[pivot],m[k]];
    if(Math.abs(m[k][k])<1e-12)return null;
    const d=m[k][k];for(let j=k;j<=b.length;j++)m[k][j]/=d;
    for(let i=0;i<b.length;i++)if(i!==k){const v=m[i][k];for(let j=k;j<=b.length;j++)m[i][j]-=v*m[k][j];}
  }
  return m.map(row=>row[b.length]);
}

// Seven revolute joints, six Cartesian constraints. The analytic geometric
// Jacobian avoids seven complete FK evaluations per iteration and the damped
// task-space inverse remains well-defined at redundant/singular postures.
export function naturalArmLimits(arm){
  return [[-1.6,.55],arm==='right'?[-1.25,.10]:[-.10,1.25],[-1.3,1.3],[-1.0,1.7],[-1.55,1.55],[-1.3,1.3],[-1.3,1.3]];
}
export function naturalArmPose(arm){const s=arm==='left'?1:-1;return [0,.30*s,.15*s,-.35,0,.10,0];}
export function solvePose(robot,arm,target,quaternion,seed,iterations=120,options={}){
  const names=armJointNames(arm),pose={...seed},weight=.18;
  const bounds=names.map((name,i)=>{const limit=robot.joints[name].limit,wanted=options.limits?.[i];return wanted?{lower:Math.max(limit.lower,wanted[0]),upper:Math.min(limit.upper,wanted[1])}:limit;});
  names.forEach((name,i)=>pose[name]=THREE.MathUtils.clamp(pose[name]??0,bounds[i].lower+.001,bounds[i].upper-.001));
  const evaluate=()=>{robot.setJointValues(pose);robot.updateMatrixWorld(true);return tcpPose(robot,arm);};
  let best={...pose},score=Infinity;
  for(let n=0;n<iterations;n++){
    const current=evaluate(),p=target.clone().sub(current.position),r=rotationError(quaternion,current.quaternion);
    const value=p.length()+weight*Math.hypot(...r);
    if(value<score){score=value;best={...pose};}
    if(p.length()<.00035&&Math.hypot(...r)<.0035)break;
    const error=[...p.toArray(),...r.map(v=>v*weight)];
    const columns=names.map(name=>{
      const joint=robot.joints[name],axis=joint.axis.clone().applyQuaternion(joint.getWorldQuaternion(new THREE.Quaternion())).normalize();
      const velocity=new THREE.Vector3().crossVectors(axis,current.position.clone().sub(joint.getWorldPosition(new THREE.Vector3())));
      return [...velocity.toArray(),...axis.multiplyScalar(weight).toArray()];
    });
    const a=Array.from({length:6},(_,i)=>Array.from({length:6},(_,j)=>columns.reduce((s,c)=>s+c[i]*c[j],0)+(i===j?.00006:0)));
    const deltaTask=linearSolve(a,error);if(!deltaTask)break;
    const posture=options.preferred?.map((v,i)=>(v-pose[names[i]])*.045);
    const projection=posture&&linearSolve(a,Array.from({length:6},(_,k)=>columns.reduce((s,c,i)=>s+c[k]*posture[i],0)));
    names.forEach((name,i)=>{
      const limit=bounds[i],secondary=projection?posture[i]-columns[i].reduce((s,v,k)=>s+v*projection[k],0):0,delta=columns[i].reduce((s,v,k)=>s+v*deltaTask[k],0)+secondary;
      pose[name]=THREE.MathUtils.clamp((pose[name]??0)+THREE.MathUtils.clamp(delta,-.14,.14),limit.lower+.001,limit.upper-.001);
    });
  }
  Object.assign(pose,best);const final=evaluate();
  return {pose,positionError:final.position.distanceTo(target),angleError:Math.hypot(...rotationError(quaternion,final.quaternion))};
}
