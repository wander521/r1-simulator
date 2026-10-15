import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import * as THREE from '../dist/vendor/three.module.min.js';
import {ROOT_HEIGHT,TABLE_HEIGHT,TABLE_CENTER_X,TABLE_HALF_X,TABLE_HALF_Z,PLINTH_RADIUS,PLINTH_HEIGHT,OBJECT_START_X,OBJECT_LATERAL_OFFSET,TOOL_ADAPTER_XYZ,TOOL_ADAPTER_RPY} from '../dist/c5/c5-profile.mjs';

const attribute=(s,key,fallback='')=>s.match(new RegExp(`\\b${key}\\s*=\\s*"([^"]*)"`))?.[1]??fallback;
export const attr=(s,tag,key,fallback='')=>attribute(s.match(new RegExp(`<${tag}\\b[^>]*>`))?.[0]||'',key,fallback);
const values=s=>s.trim().split(/\s+/).map(Number),num=n=>Number(n.toPrecision(15)).toString(),vec=v=>v.map(num).join(' ');
const rotation=rpy=>new THREE.Quaternion().setFromEuler(new THREE.Euler(...values(rpy),'ZYX'));
const quaternion=rpy=>{const q=rotation(rpy);return vec([q.w,q.x,q.y,q.z]);};
export function parseURDF(xml){
  const bodies=new Map([...xml.matchAll(/<link\b([^>]*?)(?<!\/)>([\s\S]*?)<\/link>/g)].map(([,a,body])=>[attribute(a,'name'),body]));
  const links=new Map([...xml.matchAll(/<link\b([^>]*)>/g)].map(([,a])=>{const name=attribute(a,'name');return [name,{name,body:bodies.get(name)||'',children:[]}];}));
  const joints=[...xml.matchAll(/<joint\b([^>]*)>([\s\S]*?)<\/joint>/g)].map(([,a,body])=>({name:attribute(a,'name'),type:attribute(a,'type'),body,parent:attr(body,'parent','link'),child:attr(body,'child','link')}));
  for(const joint of joints){if(!links.has(joint.parent)||!links.has(joint.child))throw new Error(`Missing parent/child: ${joint.name}`);links.get(joint.parent).children.push(joint);}
  const children=new Set(joints.map(j=>j.child)),roots=[...links.keys()].filter(name=>!children.has(name));if(roots.length!==1)throw new Error(`Expected single root: ${roots}`);return {links,joints,root:roots[0]};
}
export function deriveC5Dex1(c5Source,g1Source){
  let source=c5Source.replace(/\r\n/g,'\n').replace(/<robot\b[^>]*>/,'<robot name="aubo_C5_with_unitree_dex1_sim_adapter">');
  source=source.replace(/\s*<link\s+name="world"\s*\/>/,'').replace(/\s*<joint\s+name="world_joint"[\s\S]*?<\/joint>/,'');
  source=source.replace(/package:\/\/aubo_description\//g,'');
  source=source.replace(/<visual>([\s\S]*?)<\/visual>/g,block=>block.replace(/<mesh\s+filename="([^"]+)\.DAE"\s*\/>/g,'<mesh filename="$1.3ds" scale="1 1 1" />'));
  const dexNames=['left_dex1_base_link','left_dex1_finger_link_1','left_dex1_finger_link_2'],dexLinks=dexNames.map(name=>{
    const match=g1Source.match(new RegExp(`<link\\s+name="${name}"[^>]*>[\\s\\S]*?<\\/link>`));if(!match)throw new Error(`Missing source ${name}`);return match[0].replace(/filename="dex1_1\//g,'filename="../../g1/assets/dex1_1/');
  }),dexJoints=[1,2].map(i=>{const match=g1Source.match(new RegExp(`<joint\\s+name="left_dex1_finger_joint_${i}"[^>]*>[\\s\\S]*?<\\/joint>`));if(!match)throw new Error('Missing Dex1 finger joint');return match[0];});
  const note=`\n  <!-- DERIVED SIMULATION ASSEMBLY, NOT AN OFFICIAL AUBO GRIPPER PACKAGE.
       C5 source: AuboRobot/aubo_description commit 47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1.
       Original six movable transforms, link inertias and limits are unchanged.
       Only the zero external world anchor is omitted; base_link is fixed .72m up.
       The original visual DAE references are unavailable upstream: official 3DS
       meshes are used with the C5 3DS loader converting millimetres to metres;
       URDF scale is 1 to avoid a second conversion; collision STL is metres.
       Unitree G1 source left Dex1 links/joints are reused without inertia/mesh edits.
       A massless fixed simulation adapter maps Dex1 +X to the C5 flange local +Z. -->\n`;
  const adapter=`  <joint name="c5_dex1_adapter_joint" type="fixed"><parent link="wrist3_Link"/><child link="left_dex1_base_link"/><origin xyz="${vec(TOOL_ADAPTER_XYZ)}" rpy="${vec(TOOL_ADAPTER_RPY)}"/></joint>`;
  return source.replace('</robot>',note+dexLinks.join('\n')+'\n'+adapter+'\n'+dexJoints.join('\n')+'\n</robot>\n');
}
function embeddedSTL(buffer,name){
  const count=buffer.readUInt32LE(80);if(buffer.length!==84+count*50)throw new Error(`Invalid binary STL: ${name}`);
  const vertices=[],faces=[],indices=new Map();for(let face=0;face<count;face++)for(let corner=0;corner<3;corner++){
    const offset=84+face*50+12+corner*12,p=[buffer.readFloatLE(offset),buffer.readFloatLE(offset+4),buffer.readFloatLE(offset+8)],key=vec(p);if(!indices.has(key)){indices.set(key,vertices.length/3);vertices.push(...p);}faces.push(indices.get(key));
  }return `<mesh name="${name}" vertex="${vec(vertices)}" face="${faces.join(' ')}"/>`;
}
export async function readC5CollisionMeshes(xml){
  const files=new Set();for(const link of parseURDF(xml).links.values())for(const [,block]of link.body.matchAll(/<collision>([\s\S]*?)<\/collision>/g)){const file=attr(block,'mesh','filename');if(file)files.add(file);}
  return new Map(await Promise.all([...files].map(async file=>[file,await readFile(new URL('../dist/c5/assets/'+file,import.meta.url))])));
}
function sourceTensor(inertial){
  const [xx,yy,zz,xy,xz,yz]=['ixx','iyy','izz','ixy','ixz','iyz'].map(key=>Number(attr(inertial,'inertia',key))),r=new THREE.Matrix4().makeRotationFromQuaternion(rotation(attr(inertial,'origin','rpy','0 0 0'))),tensor=new THREE.Matrix4().set(xx,xy,xz,0,xy,yy,yz,0,xz,yz,zz,0,0,0,0,1),out=r.clone().multiply(tensor).multiply(r.clone().transpose()).elements;
  return [out[0],out[5],out[10],out[4],out[8],out[9]];
}
export function convertC5URDF(xml,{collisionMeshes}={}){
  if(!collisionMeshes)throw new Error('Original C5 and Dex1 collision meshes are required');
  const {links,joints,root}=parseURDF(xml);if(root!=='base_link')throw new Error(`Unexpected C5 root ${root}`);
  const meshNames=new Map([...collisionMeshes.keys()].map((file,i)=>[file,`c5_mesh_${i}`])),motors=[];
  const emit=(name,joint,depth)=>{
    const link=links.get(name),pad='  '.repeat(depth),pose=joint?`pos="${attr(joint.body,'origin','xyz','0 0 0')}" quat="${quaternion(attr(joint.body,'origin','rpy','0 0 0'))}"`:`pos="0 0 ${ROOT_HEIGHT}"`,lines=[`${pad}<body name="${name}" ${pose}>`];
    if(joint&&joint.type!=='fixed'){
      const finger=joint.type==='prismatic',lower=attr(joint.body,'limit','lower'),upper=attr(joint.body,'limit','upper'),effort=Number(attr(joint.body,'limit','effort'));
      const limit=finger?' solreflimit=".002 1" solimplimit=".99 .999 .0001" armature=".005"':'';
      const damping=finger?1:Number(attr(joint.body,'dynamics','damping',attr(joint.body,'property','damping','0')));
      lines.push(`${pad}  <joint name="${joint.name}" type="${finger?'slide':'hinge'}" axis="${attr(joint.body,'axis','xyz','0 0 1')}" limited="true" range="${lower} ${upper}" damping="${damping}"${limit}/>`);
      if(finger)motors.push(`<position name="left_motor${joint.name.at(-1)}" joint="${joint.name}" kp="1000" kv="20" ctrlrange="${lower} ${upper}" forcelimited="true" forcerange="-${effort} ${effort}"/>`);
      else {const index=['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint'].indexOf(joint.name),kp=[12000,18000,12000,2500,2500,1500][index],kv=[180,250,180,35,35,25][index];if(index<0)throw new Error(`Unknown C5 actuator ${joint.name}`);motors.push(`<general name="${joint.name}_motor" joint="${joint.name}" gainprm="1" biastype="affine" biasprm="0 -${kp} -${kv}" forcelimited="true" forcerange="-${effort} ${effort}"/>`);}
    }
    const inertial=link.body.match(/<inertial>([\s\S]*?)<\/inertial>/)?.[1];if(inertial)lines.push(`${pad}  <inertial pos="${attr(inertial,'origin','xyz','0 0 0')}" mass="${attr(inertial,'mass','value')}" fullinertia="${vec(sourceTensor(inertial))}"/>`);
    for(const [index,match]of [...link.body.matchAll(/<collision>([\s\S]*?)<\/collision>/g)].entries()){
      const block=match[1],file=attr(block,'mesh','filename');if(!file)throw new Error(`Expected original mesh collider: ${name}`);const finger=name.includes('_dex1_finger_link_');
      lines.push(`${pad}  <geom name="${finger?'left_tip'+name.at(-1):name+'_collision'+(index+1)}" type="mesh" mesh="${meshNames.get(file)}" pos="${attr(block,'origin','xyz','0 0 0')}" quat="${quaternion(attr(block,'origin','rpy','0 0 0'))}" contype="${finger?18:16}" conaffinity="17"/>`);
    }
    for(const child of link.children)lines.push(emit(child.child,child,depth+1));lines.push(`${pad}</body>`);return lines.join('\n');
  };
  const tree=emit(root,null,2),excludes=joints.map(j=>`<exclude body1="${j.parent}" body2="${j.child}"/>`);excludes.push('<exclude body1="left_dex1_finger_link_1" body2="left_dex1_finger_link_2"/>');
  return `<mujoco model="aubo_c5_dex1_fixed_base">
  <!-- Official C5 six-axis URDF + Unitree left Dex1; the flange adapter and
       implicit PD gains are simulation choices, not AUBO factory hardware.
       Source frames, axes, mass, COM, full inertias, ranges and effort limits
       are retained. C5 custom motor property metadata remains in derived URDF;
       undocumented motor inertia fields are not guessed as link/rotor inertia.
       Original collision vertices are embedded; MuJoCo uses convex mesh hulls,
       not exact nonconvex contact. Only immediate mechanical neighbours and
       same-gripper paired fingers are excluded. No object weld or suction.
       Dex1 uses the validated 2ms physical end-stop and kp1000/kv20 control. -->
  <compiler angle="radian" autolimits="true" fusestatic="false" inertiafromgeom="false"/>
  <option timestep=".001" integrator="implicitfast" solver="Newton" iterations="60" cone="elliptic" gravity="0 0 -9.81"/>
  <default><geom contype="0" conaffinity="0" friction="3.5 .08 .01" solref=".006 1" solimp=".95 .99 .001" condim="6"/></default>
  <asset>
${[...collisionMeshes].map(([file,buffer])=>'    '+embeddedSTL(buffer,meshNames.get(file))).join('\n')}
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="3 3 .1" contype="1" conaffinity="3"/>
    <body name="robot_plinth" pos="0 0 ${PLINTH_HEIGHT/2}"><geom name="plinth_cylinder" type="cylinder" size="${PLINTH_RADIUS} ${PLINTH_HEIGHT/2}" contype="1" conaffinity="3"/></body>
    <body name="task_table" pos="${TABLE_CENTER_X} 0 ${TABLE_HEIGHT-.0325}"><geom name="table_top" type="box" size="${TABLE_HALF_X} ${TABLE_HALF_Z} .0325" contype="1" conaffinity="3"/></body>
    <body name="red_cube" pos="${OBJECT_START_X} ${-OBJECT_LATERAL_OFFSET} ${TABLE_HEIGHT+.0225}"><freejoint/><inertial pos="0 0 0" mass=".045" diaginertia=".0000151875 .0000151875 .0000151875"/><geom type="box" size=".0225 .0225 .0225" contype="1" conaffinity="3"/></body>
    <body name="blue_block" pos="${OBJECT_START_X} 0 ${TABLE_HEIGHT+.019}"><freejoint/><inertial pos="0 0 0" mass=".050" diaginertia=".0000102833333333333 .0000172833333333333 .0000155333333333333"/><geom type="box" size=".026 .016 .019" contype="1" conaffinity="3"/></body>
    <body name="yellow_cylinder" pos="${OBJECT_START_X} ${OBJECT_LATERAL_OFFSET} ${TABLE_HEIGHT+.026}"><freejoint/><inertial pos="0 0 0" mass=".042" diaginertia=".000013664 .000013664 .0000084"/><geom type="cylinder" size=".020 .026" contype="1" conaffinity="3"/></body>
${tree}
  </worldbody>
  <contact>
${excludes.map(e=>'    '+e).join('\n')}
  </contact>
  <actuator>
${motors.map(m=>'    '+m).join('\n')}
  </actuator>
</mujoco>\n`;
}
if(process.argv[1]===fileURLToPath(import.meta.url)){
  const [c5,g1]=await Promise.all([readFile(new URL('../dist/c5/assets/aubo_C5.urdf',import.meta.url),'utf8'),readFile(new URL('../dist/g1/assets/g1_29dof_with_dex1_1.urdf',import.meta.url),'utf8')]),derived=deriveC5Dex1(c5,g1),urdfURL=new URL('../dist/c5/assets/c5-dex1.urdf',import.meta.url);await writeFile(urdfURL,derived);
  const collisionMeshes=await readC5CollisionMeshes(derived),xml=convertC5URDF(derived,{collisionMeshes}),target=new URL('../dist/c5/models/c5-dynamic-scene.xml',import.meta.url);await mkdir(new URL('../dist/c5/models/',import.meta.url),{recursive:true});await writeFile(target,xml);console.log(JSON.stringify({derived:fileURLToPath(urdfURL),model:fileURLToPath(target),bytes:xml.length,collisionMeshes:collisionMeshes.size}));
}
