import * as THREE from '../vendor/three.module.min.js';
import {armJointNames,gripperLinkName,GRIPPER_TCP_X} from './c5-profile.mjs';

export const TCP=new THREE.Vector3(GRIPPER_TCP_X,0,0);
export function tcpPose(robot,arm='left'){
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
  }return m.map(row=>row[b.length]);
}

// C5 has six actual revolute axes. Solve translation AND orientation in the
// supplied joint-local frames, without borrowing a humanoid shoulder chain.
export function solvePose(robot,arm,target,quaternion,seed={},iterations=180,options={}){
  const names=armJointNames(arm),pose={...seed},weight=.18;
  const bounds=names.map((name,i)=>{const limit=robot.joints[name].limit,wanted=options.limits?.[i];return wanted?{lower:Math.max(limit.lower,wanted[0]),upper:Math.min(limit.upper,wanted[1])}:limit;});
  names.forEach((name,i)=>pose[name]=THREE.MathUtils.clamp(pose[name]??0,bounds[i].lower+.001,bounds[i].upper-.001));
  const evaluate=()=>{robot.setJointValues(pose);robot.updateMatrixWorld(true);return tcpPose(robot,arm);};
  let best={...pose},score=Infinity;
  for(let n=0;n<iterations;n++){
    const current=evaluate(),p=target.clone().sub(current.position),r=rotationError(quaternion,current.quaternion);
    const value=p.length()+weight*Math.hypot(...r);
    if(value<score){score=value;best={...pose};}
    if(p.length()<.00025&&Math.hypot(...r)<.0025)break;
    const error=[...p.toArray(),...r.map(v=>v*weight)];
    const columns=names.map(name=>{
      const joint=robot.joints[name],axis=joint.axis.clone().applyQuaternion(joint.getWorldQuaternion(new THREE.Quaternion())).normalize();
      const velocity=new THREE.Vector3().crossVectors(axis,current.position.clone().sub(joint.getWorldPosition(new THREE.Vector3())));
      return [...velocity.toArray(),...axis.multiplyScalar(weight).toArray()];
    });
    const a=Array.from({length:6},(_,i)=>Array.from({length:6},(_,j)=>columns.reduce((s,c)=>s+c[i]*c[j],0)+(i===j?.000025:0)));
    const delta=linearSolve(a,error);if(!delta)break;
    names.forEach((name,i)=>{const limit=bounds[i],step=columns[i].reduce((sum,v,k)=>sum+v*delta[k],0);pose[name]=THREE.MathUtils.clamp(pose[name]+THREE.MathUtils.clamp(step,-.12,.12),limit.lower+.001,limit.upper-.001);});
  }
  Object.assign(pose,best);const final=evaluate();
  return {pose,positionError:final.position.distanceTo(target),angleError:Math.hypot(...rotationError(quaternion,final.quaternion))};
}
