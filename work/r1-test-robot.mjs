import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
// URDF joint tree only: the production solver is shared, no hand-entered poses.
export async function loadTestRobot() {
  const xml = await readFile(new URL('../dist/r1/assets/r1.urdf', import.meta.url), 'utf8');
  const robot = new THREE.Group(); robot.rotation.x = -Math.PI / 2; robot.joints = {};
  const links = new Map([...xml.matchAll(/<link\s+name="([^"]+)"/g)].map(m => [m[1], Object.assign(new THREE.Group(), {name:m[1]})]));
  const attr = (s, tag, key, fallback = '') => s.match(new RegExp(`<${tag}\\b[^>]*\\b${key}="([^"]+)"`))?.[1] ?? fallback;
  for (const match of xml.matchAll(/<joint\s+name="([^"]+)"\s+type="([^"]+)"[^>]*>([\s\S]*?)<\/joint>/g)) {
    const [, name, type, body] = match, joint = new THREE.Group();
    const xyz = attr(body,'origin','xyz','0 0 0').split(/\s+/).map(Number), rpy = attr(body,'origin','rpy','0 0 0').split(/\s+/).map(Number);
    joint.position.fromArray(xyz); joint.quaternion.setFromEuler(new THREE.Euler(...rpy,'ZYX'));
    const original = joint.quaternion.clone(), origin = joint.position.clone(), axis = new THREE.Vector3().fromArray(attr(body,'axis','xyz','0 0 1').split(/\s+/).map(Number));
    joint.limit = {lower:Number(attr(body,'limit','lower', '-6.28')), upper:Number(attr(body,'limit','upper','6.28'))};
    joint.setValue = value => { if (type === 'revolute' || type === 'continuous') joint.quaternion.copy(original).multiply(new THREE.Quaternion().setFromAxisAngle(axis,value)); else if (type === 'prismatic') joint.position.copy(origin).add(axis.clone().applyQuaternion(original).multiplyScalar(value)); };
    links.get(attr(body,'parent','link')).add(joint); joint.add(links.get(attr(body,'child','link'))); robot.joints[name] = joint;
  }
  robot.add(links.get('base_link')); robot.setJointValues = values => Object.entries(values).forEach(([n,v]) => robot.joints[n]?.setValue(v));
  return robot;
}
