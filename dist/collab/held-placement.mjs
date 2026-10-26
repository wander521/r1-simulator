import * as THREE from '../vendor/three.module.min.js';
import {solvePose,tcpPose,naturalArmLimits} from '../r1/grasp-kinematics.mjs';

/** Re-plan only the loaded placement leg from the measured tool/object rigid
 * transform. This is a collision-prediction hypothesis, never an attachment:
 * neither this function nor its caller writes physical body poses. */
export function buildHeldPlacementPlan({controller:c,actualPose,objectPose,drop,clearance=.13}){
  if(!Number.isFinite(clearance)||clearance<.06||clearance>.2)throw new Error('Invalid placement clearance');
  const robot=c.robot,arm=c.arm,names=Array.from({length:6},(_,i)=>`${arm}_arm_joint${i+1}`);
  robot.setJointValues(actualPose);robot.updateMatrixWorld(true);
  const actualTcp=tcpPose(robot,arm),inverse=actualTcp.quaternion.clone().invert();
  const graspTransform={position:objectPose.position.clone().sub(actualTcp.position).applyQuaternion(inverse),quaternion:inverse.clone().multiply(objectPose.quaternion)};
  if(graspTransform.position.length()>.04)throw new Error('Held carrier is no longer centred in the gripper');
  // Preserve measured heading, but level the bin before lowering it. A small
  // grip slip is corrected by moving the wrist, not by rotating the payload.
  const heading=new THREE.Vector3(1,0,0).applyQuaternion(objectPose.quaternion);heading.y=0;
  if(heading.length()<.5)throw new Error('Carrier is too tilted for a safe placement');
  const releaseObjectQuaternion=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),Math.atan2(-heading.z,heading.x));
  const orientation=releaseObjectQuaternion.clone().multiply(graspTransform.quaternion.clone().invert());
  const above=drop.clone().add(new THREE.Vector3(0,clearance,0));
  const limits=naturalArmLimits(arm,{wristPitch:c.planData.naturalSelection.wristPitchLimit});
  let seed={...actualPose};const quality={maxPositionError:0,maxAngleError:0,maxAdjacentJointJump:0};
  const solve=(objectPosition,objectQuaternion)=>{
    const q=objectQuaternion.clone().multiply(graspTransform.quaternion.clone().invert());
    const p=objectPosition.clone().sub(graspTransform.position.clone().applyQuaternion(q));
    robot.setJointValues(seed);robot.updateMatrixWorld(true);
    const solved=solvePose(robot,arm,p,q,seed,300,{limits}),jump=Math.max(...names.map(n=>Math.abs(solved.pose[n]-seed[n])));
    if(solved.positionError>.003||solved.angleError>.035||jump>.3)throw Object.assign(new Error('No continuous loaded placement solution'),{detail:{position:objectPosition.toArray(),positionError:solved.positionError,angleError:solved.angleError,jump}});
    quality.maxPositionError=Math.max(quality.maxPositionError,solved.positionError);quality.maxAngleError=Math.max(quality.maxAngleError,solved.angleError);quality.maxAdjacentJointJump=Math.max(quality.maxAdjacentJointJump,jump);
    seed={...seed,...solved.pose};return {...seed};
  };
  const line=(a,b,qa,qb)=>Array.from({length:41},(_,i)=>solve(a.clone().lerp(b,i/40),qa.clone().slerp(qb,i/40)));
  const transfer=line(objectPose.position,above,objectPose.quaternion,releaseObjectQuaternion);
  const place=line(above,drop,releaseObjectQuaternion,releaseObjectQuaternion);
  // After release, the same Cartesian wrist path retreats without the bin.
  const retreat=line(drop,above,releaseObjectQuaternion,releaseObjectQuaternion);
  return {...c.planData,drop:drop.clone(),orientation,graspTransform,releaseObjectQuaternion,transfer,place,retreat,loadedPlacementQuality:quality};
}
