import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import * as THREE from '../dist/vendor/three.module.min.js';
import {ROOT_HEIGHT,TABLE_HEIGHT,TABLE_CENTER_X,TABLE_HALF_X,TABLE_HALF_Z,STAND_POSE,OBJECT_START_X,OBJECT_LATERAL_OFFSET} from '../dist/g1/g1-profile.mjs';

const attr=(source,tag,key,fallback='')=>source.match(new RegExp(`<${tag}\\b[^>]*\\b${key}="([^"]+)"`))?.[1]??fallback;
const values=s=>s.trim().split(/\s+/).map(Number),num=n=>Number(n.toPrecision(14)).toString(),vec=v=>v.map(num).join(' ');
function quaternion(rpy){const q=new THREE.Quaternion().setFromEuler(new THREE.Euler(...values(rpy),'ZYX'));return vec([q.w,q.x,q.y,q.z]);}
function parseURDF(xml){
  const bodies=new Map([...xml.matchAll(/<link\s+name="([^"]+)"(?![^>]*\/>)[^>]*>([\s\S]*?)<\/link>/g)].map(([,name,body])=>[name,body]));
  const links=new Map([...xml.matchAll(/<link\s+name="([^"]+)"/g)].map(([,name])=>[name,{name,body:bodies.get(name)||'',children:[]}]));
  const joints=[...xml.matchAll(/<joint\s+name="([^"]+)"\s+type="([^"]+)"[^>]*>([\s\S]*?)<\/joint>/g)].map(([,name,type,body])=>({name,type,body,parent:attr(body,'parent','link'),child:attr(body,'child','link')}));
  for(const joint of joints)links.get(joint.parent).children.push(joint);return {links,joints};
}
export function sourceFootSole(xml,pose=STAND_POSE){
  const {links}=parseURDF(xml),points=[];
  const walk=(name,parent)=>{
    const link=links.get(name);
    if(name.endsWith('_ankle_roll_link'))for(const [,body]of link.body.matchAll(/<collision>([\s\S]*?)<\/collision>/g)){
      const radius=Number(attr(body,'sphere','radius','NaN'));if(!Number.isFinite(radius))continue;
      const p=new THREE.Vector3().fromArray(values(attr(body,'origin','xyz','0 0 0'))).applyMatrix4(parent);points.push({link:name,position:p.toArray(),radius,bottom:p.z-radius});
    }
    for(const joint of link.children){
      const origin=new THREE.Matrix4().compose(new THREE.Vector3().fromArray(values(attr(joint.body,'origin','xyz','0 0 0'))),new THREE.Quaternion().setFromEuler(new THREE.Euler(...values(attr(joint.body,'origin','rpy','0 0 0')),'ZYX')),new THREE.Vector3(1,1,1));
      const rotation=new THREE.Matrix4().makeRotationAxis(new THREE.Vector3().fromArray(values(attr(joint.body,'axis','xyz','0 0 1'))),pose[joint.name]??0);
      walk(joint.child,parent.clone().multiply(origin).multiply(rotation));
    }
  };
  walk('pelvis',new THREE.Matrix4());if(points.length!==8)throw new Error(`Expected eight source foot collision spheres, got ${points.length}`);
  return {rootHeight:-Math.min(...points.map(p=>p.bottom)),points};
}
function embeddedSTL(buffer,name){
  const count=buffer.readUInt32LE(80);if(buffer.length!==84+count*50)throw new Error(`Invalid binary STL: ${name}`);
  const vertices=[],faces=[],indices=new Map();
  for(let face=0;face<count;face++)for(let corner=0;corner<3;corner++){
    const offset=84+face*50+12+corner*12,point=[buffer.readFloatLE(offset),buffer.readFloatLE(offset+4),buffer.readFloatLE(offset+8)],key=vec(point);
    if(!indices.has(key)){indices.set(key,vertices.length/3);vertices.push(...point);}faces.push(indices.get(key));
  }
  return `<mesh name="${name}" vertex="${vec(vertices)}" face="${faces.join(' ')}"/>`;
}
const trunkCollisionNames=new Set(['pelvis','pelvis_contour_link','torso_link','head_link']);
const hasArmCollision=name=>/_(?:shoulder_(?:pitch|roll|yaw)|elbow|wrist_(?:roll|pitch|yaw))_link$/.test(name)||name.endsWith('_dex1_base_link');
function collisionBlocks(name,body){
  if(!trunkCollisionNames.has(name)&&!hasArmCollision(name))return [];
  const blocks=[...body.matchAll(/<collision>([\s\S]*?)<\/collision>/g)].map(m=>m[1]);
  // Pelvis has a visual housing but no source collision; use that same shell
  // as the physical hip guard rather than inventing a differently sized box.
  if(!blocks.length&&name==='pelvis')blocks.push(body.match(/<visual>([\s\S]*?)<\/visual>/)?.[1]||'');return blocks;
}
export async function readG1CollisionMeshes(xml){
  const files=new Set();for(const {name,body}of parseURDF(xml).links.values())for(const block of collisionBlocks(name,body)){const file=attr(block,'mesh','filename');if(file)files.add(file);}
  return new Map(await Promise.all([...files].map(async file=>[file,await readFile(new URL('../dist/g1/assets/'+file,import.meta.url))])));
}
export function convertG1URDF(xml,{fingerSTLs,collisionMeshes,tableHeight=TABLE_HEIGHT,tableCenterX=TABLE_CENTER_X,tableHalfX=TABLE_HALF_X,tableHalfZ=TABLE_HALF_Z,rootHeight=ROOT_HEIGHT}={}){
  if(!fingerSTLs||fingerSTLs.length!==2)throw new Error('Two original Dex1 collision STLs are required');
  if(!collisionMeshes)throw new Error('Original torso, head, pelvis and full-arm collision meshes are required');
  const {links}=parseURDF(xml),sole=sourceFootSole(xml);if(Math.abs(sole.rootHeight-rootHeight)>1e-10)throw new Error(`Profile root height ${rootHeight} differs from URDF sole ${sole.rootHeight}`);
  const motors=[],meshNames=new Map([...collisionMeshes.keys()].map((file,i)=>[file,`g1_shell_${i}`]));
  const emit=(name,joint,depth)=>{
    const link=links.get(name),pad='  '.repeat(depth),lines=[];
    const pose=joint?` pos="${attr(joint.body,'origin','xyz','0 0 0')}" quat="${quaternion(attr(joint.body,'origin','rpy','0 0 0'))}"`:` pos="0 0 ${num(rootHeight)}"`;
    lines.push(`${pad}<body name="${name}"${pose}>`);
    if(joint&&joint.type!=='fixed'){
      const finger=joint.type==='prismatic',effort=Number(attr(joint.body,'limit','effort')),lower=attr(joint.body,'limit','lower'),upper=attr(joint.body,'limit','upper');
      // Dex1's 44.5mm stroke uses a stiff physical end-stop, not MuJoCo's
      // default 20ms soft joint limit. Keep the source range/mass untouched;
      // no running qpos clamp or enlarged stroke is used to hide overshoot.
      const physicalLimit=finger?' limited="true" solreflimit=".002 1" solimplimit=".99 .999 .0001"':'';
      lines.push(`${pad}  <joint name="${joint.name}" type="${finger?'slide':'hinge'}" axis="${attr(joint.body,'axis','xyz','0 0 1')}" range="${lower} ${upper}" damping="${finger?1:.02}" armature=".005"${physicalLimit}/>`);
      if(finger)motors.push(`<position name="${joint.name.split('_')[0]}_motor${joint.name.at(-1)}" joint="${joint.name}" kp="1000" kv="20" ctrlrange="${lower} ${upper}" forcelimited="true" forcerange="-${effort} ${effort}"/>`);
      else {
        const legs=joint.name.includes('_hip_')||joint.name.includes('_knee_')||joint.name.includes('_ankle_'),waist=joint.name.startsWith('waist_'),wrist=joint.name.includes('_wrist_'),smallWrist=joint.name.includes('_wrist_pitch_')||joint.name.includes('_wrist_yaw_'),elbow=joint.name.includes('_elbow_');
        const kp=legs||waist?12000:smallWrist?400:wrist?700:elbow?1400:2000,kv=legs?200:waist?180:smallWrist?9:wrist?14:elbow?30:40;
        motors.push(`<general name="${joint.name}_motor" joint="${joint.name}" gainprm="1" biastype="affine" biasprm="0 -${kp} -${kv}" forcelimited="true" forcerange="-${effort} ${effort}"/>`);
      }
    }
    const inertial=link.body.match(/<inertial>([\s\S]*?)<\/inertial>/)?.[1];
    if(inertial){
      if(values(attr(inertial,'origin','rpy','0 0 0')).some(v=>Math.abs(v)>1e-12))throw new Error(`Nonzero inertial RPY must rotate tensor: ${name}`);
      const tensor=['ixx','iyy','izz','ixy','ixz','iyz'].map(key=>attr(inertial,'inertia',key));
      lines.push(`${pad}  <inertial pos="${attr(inertial,'origin','xyz','0 0 0')}" mass="${attr(inertial,'mass','value')}" fullinertia="${tensor.join(' ')}"/>`);
    }
    if(name.includes('_dex1_finger_link_')){
      const side=name.split('_')[0],index=name.at(-1);lines.push(`${pad}  <geom name="${side}_tip${index}" type="mesh" mesh="dex1_collision_${index}" contype="18" conaffinity="25"/>`);
    }
    for(const [i,block]of collisionBlocks(name,link.body).entries()){
      const trunk=trunkCollisionNames.has(name),file=attr(block,'mesh','filename'),position=attr(block,'origin','xyz','0 0 0'),rotation=quaternion(attr(block,'origin','rpy','0 0 0'));
      const type=file?'mesh':'cylinder',shape=file?`mesh="${meshNames.get(file)}"`:`size="${attr(block,'cylinder','radius')} ${num(Number(attr(block,'cylinder','length'))/2)}"`;
      lines.push(`${pad}  <geom name="${name}_collision${i+1}" type="${type}" ${shape} pos="${position}" quat="${rotation}" contype="${trunk?8:16}" conaffinity="${trunk?17:25}"/>`);
    }
    if(name.endsWith('_ankle_roll_link')){
      let index=0;for(const [,body]of link.body.matchAll(/<collision>([\s\S]*?)<\/collision>/g)){
        const radius=attr(body,'sphere','radius');if(radius)lines.push(`${pad}  <geom name="${name}_sole${++index}" type="sphere" pos="${attr(body,'origin','xyz','0 0 0')}" size="${radius}" contype="4" conaffinity="1"/>`);
      }
    }
    for(const child of link.children)lines.push(emit(child.child,child,depth+1));lines.push(`${pad}</body>`);return lines.join('\n');
  };
  const tree=emit('pelvis',null,2);
  const excludes=[];
  for(const side of ['left','right']){
    const chain=['torso_link',`${side}_shoulder_pitch_link`,`${side}_shoulder_roll_link`,`${side}_shoulder_yaw_link`,`${side}_elbow_link`,`${side}_wrist_roll_link`,`${side}_wrist_pitch_link`,`${side}_wrist_yaw_link`,`${side}_dex1_base_link`];
    for(let i=1;i<chain.length;i++)excludes.push(`<exclude body1="${chain[i-1]}" body2="${chain[i]}"/>`);
    for(const index of [1,2])excludes.push(`<exclude body1="${side}_dex1_base_link" body2="${side}_dex1_finger_link_${index}"/>`);
    excludes.push(`<exclude body1="${side}_dex1_finger_link_1" body2="${side}_dex1_finger_link_2"/>`);
  }
  return `<mujoco model="g1_33_joint_fixed_stand">
  <!-- Generated solely from g1_29dof_with_dex1_1.urdf and its Dex1 collision
       STLs. Original transforms, axes, COM, full tensors and effort limits.
       Pelvis is fixed; no walking, squat, free root or floating foot servo.
       Foot spheres, shoulder cylinders and shell frames are source URDF.
       Torso/head/pelvis, all seven arm links, palms and fingers physically
       collide; original mesh vertices are embedded and MuJoCo uses convex
       hulls, not exact nonconvex surfaces. Only direct mechanical neighbours
       and same-gripper fingers are excluded. No opposite-arm exemptions.
       No object weld, carrier or suction. Robot collision margin stays zero. -->
  <compiler angle="radian" autolimits="true" fusestatic="false" inertiafromgeom="false"/>
  <option timestep=".001" integrator="implicitfast" solver="Newton" iterations="60" cone="elliptic" gravity="0 0 -9.81"/>
  <default><geom contype="0" conaffinity="0" friction="3.5 .08 .01" solref=".006 1" solimp=".95 .99 .001" condim="6"/></default>
  <asset>
    ${embeddedSTL(fingerSTLs[0],'dex1_collision_1')}
    ${embeddedSTL(fingerSTLs[1],'dex1_collision_2')}
${[...collisionMeshes].map(([file,buffer])=>'    '+embeddedSTL(buffer,meshNames.get(file))).join('\n')}
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="3 3 .1" contype="1" conaffinity="7"/>
    <body name="task_table" pos="${num(tableCenterX)} 0 ${num(tableHeight-.0325)}"><geom name="table_top" type="box" size="${num(tableHalfX)} ${num(tableHalfZ)} .0325" contype="1" conaffinity="3"/></body>
    <body name="red_cube" pos="${num(OBJECT_START_X)} ${num(-OBJECT_LATERAL_OFFSET)} ${num(tableHeight+.0225)}"><freejoint/><inertial pos="0 0 0" mass=".045" diaginertia=".0000151875 .0000151875 .0000151875"/><geom type="box" size=".0225 .0225 .0225" contype="1" conaffinity="3"/></body>
    <body name="blue_block" pos="${num(OBJECT_START_X)} 0 ${num(tableHeight+.019)}"><freejoint/><inertial pos="0 0 0" mass=".050" diaginertia=".0000102833333333333 .0000172833333333333 .0000155333333333333"/><geom type="box" size=".026 .016 .019" contype="1" conaffinity="3"/></body>
    <body name="yellow_cylinder" pos="${num(OBJECT_START_X)} ${num(OBJECT_LATERAL_OFFSET)} ${num(tableHeight+.026)}"><freejoint/><inertial pos="0 0 0" mass=".042" diaginertia=".000013664 .000013664 .0000084"/><geom type="cylinder" size=".020 .026" contype="1" conaffinity="3"/></body>
${tree}
  </worldbody>
  <contact>
${excludes.map(exclude=>'    '+exclude).join('\n')}
  </contact>
  <actuator>
${motors.map(m=>'    '+m).join('\n')}
  </actuator>
</mujoco>\n`;
}
if(process.argv[1]===fileURLToPath(import.meta.url)){
  const source=await readFile(new URL('../dist/g1/assets/g1_29dof_with_dex1_1.urdf',import.meta.url),'utf8');
  const fingerSTLs=await Promise.all([1,2].map(i=>readFile(new URL(`../dist/g1/assets/dex1_1/meshes/dex1_col_${i}.stl`,import.meta.url))));
  const collisionMeshes=await readG1CollisionMeshes(source),output=convertG1URDF(source,{fingerSTLs,collisionMeshes}),target=new URL('../dist/g1/models/g1-dynamic-scene.xml',import.meta.url);await mkdir(new URL('../dist/g1/models/',import.meta.url),{recursive:true});await writeFile(target,output);console.log(JSON.stringify({file:fileURLToPath(target),bytes:output.length,collisionMeshes:collisionMeshes.size,sole:sourceFootSole(source)}));
}
