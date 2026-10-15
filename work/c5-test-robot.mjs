import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
import {ROOT_HEIGHT} from '../dist/c5/c5-profile.mjs';

// Mesh-free source URDF tree used by the production kinematics/dynamics tests.
// It retains the joint-local source axes and does not clamp requested poses.
export async function loadC5TestRobot({rootHeight=ROOT_HEIGHT}={}){
  const xml=await readFile(new URL('../dist/c5/assets/c5-dex1.urdf',import.meta.url),'utf8'),robot=new THREE.Group();
  robot.rotation.x=-Math.PI/2;robot.position.y=rootHeight;robot.joints={};robot.links={};
  const attribute=(s,key,fallback='')=>s.match(new RegExp(`\\b${key}\\s*=\\s*"([^"]*)"`))?.[1]??fallback;
  const tag=(s,name,key,fallback='')=>attribute(s.match(new RegExp(`<${name}\\b[^>]*>`))?.[0]||'',key,fallback);
  const links=new Map([...xml.matchAll(/<link\b([^>]*)>/g)].map(([,attributes])=>{
    const name=attribute(attributes,'name'),link=Object.assign(new THREE.Group(),{name,isURDFLink:true});robot.links[name]=link;return [name,link];
  })),children=new Set();
  for(const [,attributes,body]of xml.matchAll(/<joint\b([^>]*)>([\s\S]*?)<\/joint>/g)){
    const name=attribute(attributes,'name'),type=attribute(attributes,'type'),joint=new THREE.Group(),xyz=tag(body,'origin','xyz','0 0 0').trim().split(/\s+/).map(Number),rpy=tag(body,'origin','rpy','0 0 0').trim().split(/\s+/).map(Number);
    joint.position.fromArray(xyz);joint.quaternion.setFromEuler(new THREE.Euler(...rpy,'ZYX'));
    const original=joint.quaternion.clone(),origin=joint.position.clone(),axis=new THREE.Vector3().fromArray(tag(body,'axis','xyz','0 0 1').trim().split(/\s+/).map(Number));
    Object.assign(joint,{name,axis,jointType:type,isURDFJoint:true,jointValue:[0]});
    joint.limit={lower:Number(tag(body,'limit','lower','-6.283185307179586')),upper:Number(tag(body,'limit','upper','6.283185307179586')),effort:Number(tag(body,'limit','effort','0')),velocity:Number(tag(body,'limit','velocity','0'))};
    joint.setValue=value=>{joint.jointValue[0]=value;if(type==='revolute'||type==='continuous')joint.quaternion.copy(original).multiply(new THREE.Quaternion().setFromAxisAngle(axis,value));else if(type==='prismatic')joint.position.copy(origin).add(axis.clone().applyQuaternion(original).multiplyScalar(value));};
    const parent=tag(body,'parent','link'),child=tag(body,'child','link');if(!links.has(parent)||!links.has(child))throw new Error(`Unknown URDF parent/child in ${name}`);
    links.get(parent).add(joint);joint.add(links.get(child));children.add(child);robot.joints[name]=joint;
  }
  const roots=[...links.keys()].filter(name=>!children.has(name));if(roots.length!==1)throw new Error(`Expected a single C5 root, found ${roots}`);
  robot.rootLinkName=roots[0];robot.add(links.get(roots[0]));robot.setJointValues=values=>{for(const [name,value]of Object.entries(values))robot.joints[name]?.setValue(value);};robot.updateMatrixWorld(true);return robot;
}
export const loadTestRobot=loadC5TestRobot;
