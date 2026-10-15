import * as THREE from '../vendor/three.module.min.js';

// Photo-derived finishes. These affect visual meshes only, never URDF joints
// or MuJoCo collision geometry. The supplied STL files have no texture UVs.
export const R1_FINISHES = {
  shell: {color:0xe8e5dc,metalness:.12,roughness:.32},
  silver: {color:0xb8bfc2,metalness:.58,roughness:.31},
  graphite: {color:0x252a30,metalness:.22,roughness:.48},
  mechanism: {color:0x151a20,metalness:.48,roughness:.36},
  rubber: {color:0x171b20,metalness:.02,roughness:.83},
  visor: {color:0x080e16,metalness:.24,roughness:.17},
};

// Subdivide faces at paint seams, interpolating existing normals. Classifying
// only face centroids would make a jagged seam on the low-poly chest shell.
function splitAtSeam(source,transform,axis,value){
  const positions=source.attributes.position,normals=source.attributes.normal,index=source.index;
  const outP=[],outN=[],count=index?index.count:positions.count;
  const vertex=i=>{const p=new THREE.Vector3().fromBufferAttribute(positions,i),n=new THREE.Vector3().fromBufferAttribute(normals,i);return {p,n,d:p.clone().applyMatrix4(transform)[axis]-value};};
  const clip=(vertices,sign)=>{
    const result=[];
    for(let i=0;i<vertices.length;i++){
      const a=vertices[i],b=vertices[(i+1)%vertices.length],inside=a.d*sign>=0,next=b.d*sign>=0;
      if(inside)result.push(a);
      if(inside!==next){const t=a.d/(a.d-b.d);result.push({p:a.p.clone().lerp(b.p,t),n:a.n.clone().lerp(b.n,t).normalize(),d:0});}
    }
    return result;
  };
  for(let i=0;i<count;i+=3){
    const vertices=[0,1,2].map(j=>vertex(index?index.getX(i+j):i+j));
    const polygons=vertices.some(v=>v.d>0)&&vertices.some(v=>v.d<0)?[clip(vertices,1),clip(vertices,-1)]:[vertices];
    for(const polygon of polygons)for(let j=1;j<polygon.length-1;j++)for(const v of [polygon[0],polygon[j],polygon[j+1]]){outP.push(v.p.x,v.p.y,v.p.z);outN.push(v.n.x,v.n.y,v.n.z);}
  }
  const result=new THREE.BufferGeometry();result.setAttribute('position',new THREE.Float32BufferAttribute(outP,3));result.setAttribute('normal',new THREE.Float32BufferAttribute(outN,3));return result;
}

export function applyRobotMaterials(robot){
  const names=Object.keys(R1_FINISHES);
  const finishes=names.map(name=>new THREE.MeshStandardMaterial({name:`R1 ${name}`,...R1_FINISHES[name]}));
  const ids=Object.fromEntries(names.map((name,i)=>[name,i]));
  const point=new THREE.Vector3(),a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3();
  robot.updateMatrixWorld(true);
  robot.traverse(mesh=>{
    if(!mesh.isMesh||!mesh.geometry?.attributes.position)return;
    let link=mesh;
    while(link&&!link.isURDFLink)link=link.parent;
    if(!link)return;
    const name=link.name;let geometry=mesh.geometry.clone(),positions=geometry.attributes.position;
    const toLink=new THREE.Matrix4().copy(link.matrixWorld).invert().multiply(mesh.matrixWorld);
    let headTransform,headBounds;
    if(name==='zed_link'){
      // Head has a rotated URDF frame: measure the face in torso coordinates,
      // where +X is forward, instead of assuming an STL axis is the face.
      const torso=robot.getObjectByName('torso_link4');
      headTransform=new THREE.Matrix4().copy(torso.matrixWorld).invert().multiply(mesh.matrixWorld);
      headBounds=new THREE.Box3();
      for(let i=0;i<positions.count;i++)headBounds.expandByPoint(point.fromBufferAttribute(positions,i).applyMatrix4(headTransform));
    }
    const seam=name==='torso_link4'?['z',.31]:name==='base_link'?['z',.145]:name==='zed_link'?['x',headBounds.max.x-.038]:null;
    if(seam){const split=splitAtSeam(geometry,headTransform||toLink,...seam);geometry.dispose();geometry=split;positions=geometry.attributes.position;}
    const classify=p=>{
      if(name==='base_link')return p.z>.145?'shell':'graphite';
      if(name.startsWith('wheel_motor'))return Math.hypot(p.x,p.z)<.047?'silver':'rubber';
      if(name.startsWith('steer_motor'))return 'mechanism';
      if(/^torso_link[12]$/.test(name))return Math.abs(p.y)>.067?'silver':'mechanism';
      if(name==='torso_link3')return 'mechanism';
      if(name==='torso_link4'){
        return p.z>.31?'graphite':'shell';
      }
      if(name==='zed_link'){
        return p.x>headBounds.max.x-.038?'visor':'shell';
      }
      if(name.includes('gripper')||name.includes('realsense')||name.includes('arm_base'))return 'mechanism';
      if(name.endsWith('arm_link1'))return 'shell';
      if(name.endsWith('arm_link2'))return p.x<-.29?'graphite':'silver';
      if(name.endsWith('arm_link3'))return p.x<.045?'graphite':'silver';
      if(name.endsWith('arm_link4')||name.endsWith('arm_link6'))return 'mechanism';
      if(name.endsWith('arm_link5'))return 'silver';
      return 'shell';
    };
    const buckets=names.map(()=>[]),index=geometry.index,count=index?index.count:positions.count;
    for(let i=0;i<count;i+=3){
      const ia=index?index.getX(i):i,ib=index?index.getX(i+1):i+1,ic=index?index.getX(i+2):i+2;
      a.fromBufferAttribute(positions,ia);b.fromBufferAttribute(positions,ib);c.fromBufferAttribute(positions,ic);
      point.copy(a).add(b).add(c).multiplyScalar(1/3).applyMatrix4(headTransform||toLink);
      buckets[ids[classify(point)]].push(ia,ib,ic);
    }
    // Group indices by finish (at most six draw calls), not one group per face.
    // The vertex positions, normals and model silhouette remain untouched.
    geometry.clearGroups();const reordered=[];
    buckets.forEach((bucket,i)=>{if(bucket.length){const start=reordered.length;for(const v of bucket)reordered.push(v);geometry.addGroup(start,bucket.length,i);}});
    geometry.setIndex(reordered);mesh.geometry=geometry;mesh.material=finishes;
    mesh.userData.photoFinishes=true;
  });
}
