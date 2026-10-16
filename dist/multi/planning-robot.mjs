import * as THREE from '../vendor/three.module.min.js';

/** Read the actual source URDF's kinematic tree without DOM, meshes or a
 * renderer. No link lengths, joint axes, offsets or limits are hand-authored. */
export function parsePlanningRobot(source,{rootHeight=0}={}){
  const xml=String(source).replace(/<!--[\s\S]*?-->/g,''),robot=new THREE.Group();
  robot.rotation.x=-Math.PI/2;robot.position.y=rootHeight;robot.joints={};robot.links={};
  const attribute=(s,key,fallback='')=>s.match(new RegExp(`\\b${key}\\s*=\\s*["']([^"']*)["']`))?.[1]??fallback;
  const tag=(s,name,key,fallback='')=>attribute(s.match(new RegExp(`<${name}\\b[^>]*>`))?.[0]||'',key,fallback);
  const vector=value=>{const values=value.trim().split(/\s+/).map(Number);if(values.length!==3||!values.every(Number.isFinite))throw new Error('URDF 运动链向量无效');return values;};
  const links=new Map([...xml.matchAll(/<link\b([^>]*)>/g)].map(([,attributes])=>{
    const name=attribute(attributes,'name');if(!name||robot.links[name])throw new Error('URDF 连杆名称无效或重复');
    const link=Object.assign(new THREE.Group(),{name,isURDFLink:true});robot.links[name]=link;return[name,link];
  })),children=new Set();
  for(const [,attributes,body]of xml.matchAll(/<joint\b([^>]*)>([\s\S]*?)<\/joint>/g)){
    const name=attribute(attributes,'name'),type=attribute(attributes,'type');
    if(!name||robot.joints[name]||!['fixed','revolute','continuous','prismatic'].includes(type)||/<mimic\b/.test(body))throw new Error(`URDF 关节类型或名称不受支持：${name}`);
    const joint=new THREE.Group(),xyz=vector(tag(body,'origin','xyz','0 0 0')),rpy=vector(tag(body,'origin','rpy','0 0 0'));
    joint.position.fromArray(xyz);joint.quaternion.setFromEuler(new THREE.Euler(...rpy,'ZYX'));
    const original=joint.quaternion.clone(),origin=joint.position.clone(),axis=new THREE.Vector3().fromArray(vector(tag(body,'axis','xyz','0 0 1'))).normalize();
    Object.assign(joint,{name,axis,jointType:type,isURDFJoint:true,jointValue:[0],origPosition:origin,origQuaternion:original});
    joint.limit={lower:Number(tag(body,'limit','lower','0')),upper:Number(tag(body,'limit','upper','0')),effort:Number(tag(body,'limit','effort','0')),velocity:Number(tag(body,'limit','velocity','0'))};
    joint.setValue=value=>{
      if(type==='fixed')return;
      if(!Number.isFinite(value))throw new Error(`URDF 关节值无效：${name}`);
      if(type!=='continuous')value=Math.max(joint.limit.lower,Math.min(joint.limit.upper,value));
      joint.jointValue[0]=value;
      if(type==='revolute'||type==='continuous')joint.quaternion.copy(original).multiply(new THREE.Quaternion().setFromAxisAngle(axis,value));
      else joint.position.copy(origin).add(axis.clone().applyQuaternion(original).multiplyScalar(value));
    };
    joint.setJointValue=(...values)=>joint.setValue(values[0]);
    const parent=tag(body,'parent','link'),child=tag(body,'child','link');
    if(!links.has(parent)||!links.has(child)||children.has(child))throw new Error(`URDF 关节连接无效：${name}`);
    links.get(parent).add(joint);joint.add(links.get(child));children.add(child);robot.joints[name]=joint;
  }
  const roots=[...links.keys()].filter(name=>!children.has(name));
  if(roots.length!==1)throw new Error('URDF 必须包含唯一运动链根节点');
  robot.rootLinkName=roots[0];robot.add(links.get(roots[0]));
  robot.setJointValues=values=>{for(const [name,value]of Object.entries(values))robot.joints[name]?.setValue(Array.isArray(value)?value[0]:value);};
  robot.updateMatrixWorld(true);return robot;
}
