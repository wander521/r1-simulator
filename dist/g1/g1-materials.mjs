import * as THREE from '../vendor/three.module.min.js';

// Neutral metallic finishes from the user's G1 image. No emissive paint:
// the existing workcell lights/environment supply all visible highlights.
export const G1_FINISHES=Object.freeze({
  shell:Object.freeze({color:0xcbd0d4,metalness:.38,roughness:.41,envMapIntensity:.72}),
  silver:Object.freeze({color:0xa4adb5,metalness:.64,roughness:.36,envMapIntensity:.78}),
  headShell:Object.freeze({color:0x64717d,metalness:.52,roughness:.39,envMapIntensity:.66}),
  graphite:Object.freeze({color:0x29323a,metalness:.32,roughness:.45,envMapIntensity:.58}),
  face:Object.freeze({color:0x151d25,metalness:.18,roughness:.42,envMapIntensity:.52}),
  lens:Object.freeze({color:0x080f16,metalness:.18,roughness:.20,envMapIntensity:.48}),
  foot:Object.freeze({color:0x141b21,metalness:.08,roughness:.72,envMapIntensity:.42}),
  gripper:Object.freeze({color:0x1b2228,metalness:.24,roughness:.49,envMapIntensity:.55}),
  badge:Object.freeze({color:0x697780,metalness:.73,roughness:.28,envMapIntensity:.72}),
});

// Measured in head_link's actual STL/URDF frame, not the display Y-up frame.
// The one connected source shell includes the neck, oval face and crown.
// At z=.39..51 its forward surface is x≈.037..074 and half-width .03..055.
// The mask follows that oval; an x threshold only rejects the rear cap.
export const G1_HEAD_MASK_PROFILE=Object.freeze({
  sourceBounds:Object.freeze({min:Object.freeze([-.0663562194,-.0777746066,.3247958720]),max:Object.freeze([.0739840269,.0778166428,.5305725932])}),
  neckTop:.3895,
  centerZ:.451,
  halfHeight:.069,
  halfWidth:.064,
  rearRejectX:.025,
});

export function classifyG1Finish(linkName,point){
  if(linkName==='head_link'){
    const p=G1_HEAD_MASK_PROFILE;
    if(point.z<p.neckTop)return 'graphite';
    const oval=(point.y/p.halfWidth)**2+((point.z-p.centerZ)/p.halfHeight)**2;
    if(point.x>p.rearRejectX&&oval<=1){
      const upperSensor=point.z>.470&&point.z<.503&&Math.abs(point.y)<.044;
      const lowerWindow=point.z>.407&&point.z<.450&&Math.abs(point.y)<.036&&point.x<.0415;
      return upperSensor||lowerWindow?'lens':'face';
    }
    return 'headShell';
  }
  if(linkName==='torso_link')return 'shell';
  if(linkName==='logo_link')return 'badge';
  if(linkName==='pelvis'||linkName==='pelvis_contour_link'||/^waist_(yaw|roll)_link$/.test(linkName))return 'graphite';
  if(/_dex1_(base|finger)_link/.test(linkName))return 'gripper';
  if(/_ankle_roll_link$/.test(linkName))return 'foot';
  if(/_hip_pitch_link$/.test(linkName))return 'graphite';
  if(/_(hip_roll|shoulder_pitch|elbow)_link$/.test(linkName))return 'shell';
  if(/_(hip_yaw|knee|ankle_pitch|shoulder_roll|shoulder_yaw|wrist_roll|wrist_pitch|wrist_yaw)_link$/.test(linkName))return 'silver';
  return null;
}

export function applyG1Materials(robot){
  const names=Object.keys(G1_FINISHES),materials=names.map(name=>new THREE.MeshStandardMaterial({name:`G1 ${name}`,...G1_FINISHES[name],emissive:0x000000})),ids=Object.fromEntries(names.map((name,index)=>[name,index]));
  const report={meshCount:0,links:{},unmappedLinks:[],head:{triangles:0,area:0,finishes:{}}};
  const a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3(),centroid=new THREE.Vector3(),ab=new THREE.Vector3(),ac=new THREE.Vector3();
  robot.updateMatrixWorld(true);
  robot.traverse(mesh=>{
    if(!mesh.isMesh||!mesh.geometry?.attributes.position||mesh.userData.g1PhotoMaterials)return;
    let link=mesh;while(link&&!link.isURDFLink)link=link.parent;if(!link)return;
    const name=link.name,source=mesh.geometry,position=source.attributes.position,index=source.index,count=index?index.count:position.count,triangles=count/3,toLink=new THREE.Matrix4().copy(link.matrixWorld).invert().multiply(mesh.matrixWorld);
    const uniform=classifyG1Finish(name,new THREE.Vector3());report.meshCount++;
    report.links[name]={triangles,finishes:{}};
    if(name!=='head_link'){
      const finish=uniform||'shell';mesh.material=materials[ids[finish]];report.links[name].finishes[finish]=triangles;
      if(!uniform)report.unmappedLinks.push(name);
    }else{
      const buckets=names.map(()=>[]);
      for(let i=0;i<count;i+=3){
        const vertices=[0,1,2].map(j=>index?index.getX(i+j):i+j);
        a.fromBufferAttribute(position,vertices[0]).applyMatrix4(toLink);b.fromBufferAttribute(position,vertices[1]).applyMatrix4(toLink);c.fromBufferAttribute(position,vertices[2]).applyMatrix4(toLink);
        centroid.copy(a).add(b).add(c).multiplyScalar(1/3);const finish=classifyG1Finish(name,centroid),area=ab.subVectors(b,a).cross(ac.subVectors(c,a)).length()/2;
        buckets[ids[finish]].push(...vertices);report.head.triangles++;report.head.area+=area;
        const stat=report.head.finishes[finish]||(report.head.finishes[finish]={triangles:0,area:0});stat.triangles++;stat.area+=area;
        report.links[name].finishes[finish]=(report.links[name].finishes[finish]||0)+1;
      }
      // Only render indices/material groups change. Original positions,
      // normals, STL bytes, silhouette, URDF and collision hulls stay intact.
      const geometry=source.clone(),reordered=[];geometry.clearGroups();
      buckets.forEach((bucket,i)=>{if(!bucket.length)return;const start=reordered.length;for(const vertex of bucket)reordered.push(vertex);geometry.addGroup(start,bucket.length,i);});geometry.setIndex(reordered);mesh.geometry=geometry;mesh.material=materials;
    }
    mesh.userData.g1PhotoMaterials=true;
  });
  for(const stat of Object.values(report.head.finishes))stat.areaFraction=stat.area/report.head.area;
  robot.userData.g1MaterialReport=report;
  return report;
}
