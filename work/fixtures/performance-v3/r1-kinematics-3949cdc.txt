import * as THREE from '../vendor/three.module.min.js';

export const TCP = new THREE.Vector3(.068, 0, 0);
export function tcpPose(robot, arm) {
  const link = robot.getObjectByName(`${arm}_gripper_link`);
  return { position: link.localToWorld(TCP.clone()), quaternion: link.getWorldQuaternion(new THREE.Quaternion()) };
}
export function topDownQuaternion(yaw = 0) {
  const x = new THREE.Vector3(0, -1, 0), y = new THREE.Vector3(Math.cos(yaw), 0, Math.sin(yaw));
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, new THREE.Vector3().crossVectors(x, y)));
}
// Display/task comfort bounds, strictly inside the original source ranges.
// The native roll axes are mirrored; keeping one wrist branch prevents an
// otherwise valid six-DOF solution from unwinding the forearm by a half-turn.
// These do not modify the URDF/MJCF joint, velocity, effort or contact limits.
export function naturalArmLimits(arm,{wristPitch=1.35}={}) {
  const right=arm==='right',ranges=[right?[-1.35,2.2]:[-2.2,1.35],[.02,3.20],[-3.25,-.35],
    right?[-2.85,-.15]:[.15,2.85],right?[-wristPitch,-.04]:[.04,wristPitch],[-2.35,2.35]];
  return Object.fromEntries(ranges.map(([lower,upper],index)=>[`${arm}_arm_joint${index+1}`,{lower,upper}]));
}
function rotationError(target, actual) {
  const q = target.clone().multiply(actual.clone().invert()).normalize();
  if (q.w < 0) q.set(-q.x, -q.y, -q.z, -q.w);
  const s = Math.hypot(q.x, q.y, q.z), a = s < 1e-9 ? 2 : 2 * Math.atan2(s, q.w) / s;
  return [q.x * a, q.y * a, q.z * a];
}
function linearSolve(a, b) {
  const m = a.map((row, i) => [...row, b[i]]);
  for (let k = 0; k < b.length; k++) {
    let pivot = k;
    for (let i = k + 1; i < b.length; i++) if (Math.abs(m[i][k]) > Math.abs(m[pivot][k])) pivot = i;
    [m[k], m[pivot]] = [m[pivot], m[k]];
    if (Math.abs(m[k][k]) < 1e-12) return null;
    const d = m[k][k]; for (let j = k; j <= b.length; j++) m[k][j] /= d;
    for (let i = 0; i < b.length; i++) if (i !== k) { const v = m[i][k]; for (let j = k; j <= b.length; j++) m[i][j] -= v * m[k][j]; }
  }
  return m.map(row => row[b.length]);
}

// Solve both position and orientation. A position-only solution can put a
// finger below the table or aim the gripper upward even when the TCP is close.
export function solvePose(robot, arm, target, quaternion, seed, iterations = 180, {limits=null}={}) {
  const names = Array.from({length: 6}, (_, i) => `${arm}_arm_joint${i + 1}`);
  // A full controller pose is a valid reference seed, but the arm solver
  // must not silently restore its old torso/other-arm values while solving.
  const pose = Object.fromEntries(names.map(name=>[name,seed[name]??0]));
  const bounds=Object.fromEntries(names.map(name=>{
    const source=robot.joints[name].limit,preferred=limits?.[name];
    const lower=Math.max(source.lower,preferred?.lower??-Infinity),upper=Math.min(source.upper,preferred?.upper??Infinity);
    if(!(upper-lower>.004))throw new RangeError(`Invalid IK comfort range: ${name}`);
    return[name,{lower,upper}];
  }));
  for(const name of names)pose[name]=THREE.MathUtils.clamp(pose[name]??0,bounds[name].lower+.002,bounds[name].upper-.002);
  const evaluate = () => { robot.setJointValues(pose); robot.updateMatrixWorld(true); return tcpPose(robot, arm); };
  let positionError = Infinity, angleError = Infinity;
  for (let n = 0; n < iterations; n++) {
    const current = evaluate(), p = target.clone().sub(current.position), r = rotationError(quaternion, current.quaternion);
    positionError = p.length(); angleError = Math.hypot(...r);
    if (positionError < .0004 && angleError < .004) break;
    const error = [...p.toArray(), ...r.map(v => v * .18)];
    const columns = names.map(name => {
      const initial = pose[name] || 0; pose[name] = initial + .0001;
      const perturbed = evaluate(); pose[name] = initial;
      return [...perturbed.position.sub(current.position).multiplyScalar(1e4).toArray(), ...rotationError(perturbed.quaternion, current.quaternion).map(v => v * 1800)];
    });
    const a = columns.map((ci, i) => columns.map((cj, j) => ci.reduce((s, v, k) => s + v * cj[k], 0) + (i === j ? .00008 : 0)));
    const b = columns.map(c => c.reduce((s, v, k) => s + v * error[k], 0)), delta = linearSolve(a, b);
    if (!delta) break;
    names.forEach((name, i) => { const limit = bounds[name]; pose[name] = THREE.MathUtils.clamp((pose[name] || 0) + THREE.MathUtils.clamp(delta[i], -.16, .16), limit.lower + .002, limit.upper - .002); });
  }
  const final = evaluate();
  return { pose, positionError: final.position.distanceTo(target), angleError: Math.hypot(...rotationError(quaternion, final.quaternion)) };
}
