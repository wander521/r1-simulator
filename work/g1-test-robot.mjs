import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
import {ROOT_HEIGHT} from '../dist/g1/g1-profile.mjs';

// Source URDF tree, without visual meshes, for the same production IK and
// dynamics adapter. Joint axes remain in the actual joint local frame.
export async function loadG1TestRobot({rootHeight=ROOT_HEIGHT}={}){
  const xml=await readFile(new URL('../dist/g1/assets/g1_29dof_with_dex1_1.urdf',import.meta.url),'utf8'),robot=new THREE.Group();robot.rotation.x=-Math.PI/2;robot.position.y=rootHeight;robot.joints={};robot.links={};
  const links=new Map([...xml.matchAll(/<link\s+name="([^"]+)"/g)].map(([,name])=>{const link=Object.assign(new THREE.Group(),{name,isURDFLink:true});robot.links[name]=link;return [name,link];}));
  const attr=(source,tag,key,fallback='')=>source.match(new RegExp(`<${tag}\\b[^>]*\\b${key}="([^"]+)"`))?.[1]??fallback;
  for(const [,name,type,body]of xml.matchAll(/<joint\s+name="([^"]+)"\s+type="([^"]+)"[^>]*>([\s\S]*?)<\/joint>/g)){
    const joint=new THREE.Group(),xyz=attr(body,'origin','xyz','0 0 0').split(/\s+/).map(Number),rpy=attr(body,'origin','rpy','0 0 0').split(/\s+/).map(Number);joint.position.fromArray(xyz);joint.quaternion.setFromEuler(new THREE.Euler(...rpy,'ZYX'));
    const original=joint.quaternion.clone(),origin=joint.position.clone(),axis=new THREE.Vector3().fromArray(attr(body,'axis','xyz','0 0 1').split(/\s+/).map(Number));Object.assign(joint,{name,axis,jointType:type,isURDFJoint:true,jointValue:[0]});
    joint.limit={lower:Number(attr(body,'limit','lower','-6.28')),upper:Number(attr(body,'limit','upper','6.28')),effort:Number(attr(body,'limit','effort','0')),velocity:Number(attr(body,'limit','velocity','0'))};
    joint.setValue=value=>{joint.jointValue[0]=value;if(type==='revolute'||type==='continuous')joint.quaternion.copy(original).multiply(new THREE.Quaternion().setFromAxisAngle(axis,value));else if(type==='prismatic')joint.position.copy(origin).add(axis.clone().applyQuaternion(original).multiplyScalar(value));};
    links.get(attr(body,'parent','link')).add(joint);joint.add(links.get(attr(body,'child','link')));robot.joints[name]=joint;
  }
  robot.add(links.get('pelvis'));robot.setJointValues=values=>{for(const [name,value]of Object.entries(values))robot.joints[name]?.setValue(value);};robot.updateMatrixWorld(true);return robot;
}
export const loadTestRobot=loadG1TestRobot;
