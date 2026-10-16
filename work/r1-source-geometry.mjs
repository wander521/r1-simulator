// Independent source-visual checks. These are sampled geometric witnesses,
// not MuJoCo shell/self-collision geoms or a robot safety certification.
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
const root=new URL('../',import.meta.url);
const attr=(text,key,fallback='')=>text.match(new RegExp(`\\b${key}="([^"]*)"`))?.[1]??fallback;
const numbers=(text,key,fallback)=>attr(text,key,fallback).trim().split(/\s+/).map(Number);
export async function loadR1SourceGeometry(robot){
  const urdfPath='dist/r1/assets/r1.urdf',scenePath='dist/r1/models/r1-dynamic-scene.xml';
  const urdf=await readFile(new URL(urdfPath,root),'utf8'),xml=await readFile(new URL(scenePath,root),'utf8');
  const files=new Set([urdfPath,scenePath,'work/r1-source-geometry.mjs']),records=[];
  const bodyNames=new Set(['base_link','torso_link1','torso_link2','torso_link3','torso_link4','zed_link']);
  for(const [,attrs,body]of urdf.matchAll(/<link\b([^>]*)>([\s\S]*?)<\/link>/g)){
    const name=attr(attrs,'name');if(!bodyNames.has(name)&&!/^(right|left)_(arm_link[2-6]|gripper|realsense)/.test(name))continue;
    for(const [,visual]of body.matchAll(/<visual\b[^>]*>([\s\S]*?)<\/visual>/g)){
      const mesh=visual.match(/<mesh\b[^>]*>/)?.[0]??'',filename=attr(mesh,'filename');if(!filename)continue;
      assert(filename.startsWith('package://r1_v2_1_0/'));
      const path='dist/r1/assets/'+filename.slice('package://r1_v2_1_0/'.length);files.add(path);
      const bytes=await readFile(new URL(path,root)),view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),count=view.getUint32(80,true);
      assert.equal(84+count*50,bytes.length,'Expected unchanged binary source STL');
      const positions=new Float32Array(count*9),box=new THREE.Box3(),point=new THREE.Vector3();
      for(let i=0;i<count;i++)for(let j=0;j<3;j++){
        const offset=84+i*50+12+j*12,x=view.getFloat32(offset,true),y=view.getFloat32(offset+4,true),z=view.getFloat32(offset+8,true);
        positions.set([x,y,z],i*9+j*3);box.expandByPoint(point.set(x,y,z));
      }
      const origin=visual.match(/<origin\b[^>]*>/)?.[0]??'';
      const matrix=new THREE.Matrix4().compose(new THREE.Vector3(...numbers(origin,'xyz','0 0 0')),
        new THREE.Quaternion().setFromEuler(new THREE.Euler(...numbers(origin,'rpy','0 0 0'),'ZYX')),new THREE.Vector3(...numbers(mesh,'scale','1 1 1')));
      records.push({name,path,link:robot.getObjectByName(name),positions,box,origin:matrix});
    }
  }
  const match=xml.match(/<body name="task_table" pos="([^"]+)"[^>]*>\s*<geom[^>]*size="([^"]+)"/);assert(match);
  const p=match[1].split(/\s+/).map(Number),h=match[2].split(/\s+/).map(Number);
  const table=new THREE.Box3(new THREE.Vector3(p[0]-h[0],p[2]-h[2],-p[1]-h[1]),new THREE.Vector3(p[0]+h[0],p[2]+h[2],-p[1]+h[1]));
  const arms=records.filter(r=>!bodyNames.has(r.name)),bodies=records.filter(r=>bodyNames.has(r.name));
  const transform=r=>new THREE.Matrix4().copy(r.link.matrixWorld).multiply(r.origin);
  function tableIntersections(){
    robot.updateMatrixWorld(true);const hits=[],triangle=new THREE.Triangle();
    for(const r of arms){
      const matrix=transform(r);if(!r.box.clone().applyMatrix4(matrix).intersectsBox(table))continue;
      let triangles=0,verticesInside=0,maxVertexDepthM=0;
      for(let i=0;i<r.positions.length;i+=9){
        triangle.a.fromArray(r.positions,i).applyMatrix4(matrix);triangle.b.fromArray(r.positions,i+3).applyMatrix4(matrix);triangle.c.fromArray(r.positions,i+6).applyMatrix4(matrix);
        if(table.intersectsTriangle(triangle))triangles++;
        for(const point of [triangle.a,triangle.b,triangle.c])if(table.containsPoint(point)){verticesInside++;maxVertexDepthM=Math.max(maxVertexDepthM,table.max.y-point.y);}
      }
      if(triangles||verticesInside)hits.push({name:r.name,triangles,verticesInside,maxVertexDepthM});
    }
    return hits;
  }
  function conservativeBodyGap(){
    robot.updateMatrixWorld(true);let minimum=Infinity;
    for(const b of bodies){const ref=b.link.matrixWorld.clone().invert(),bb=b.box.clone().applyMatrix4(ref.clone().multiply(transform(b)));
      for(const a of arms){const ab=a.box.clone().applyMatrix4(ref.clone().multiply(transform(a))),gap=Math.hypot(...['x','y','z'].map(k=>Math.max(0,ab.min[k]-bb.max[k],bb.min[k]-ab.max[k])));minimum=Math.min(minimum,gap);}}
    return minimum;
  }
  return {files:[...files],table,tableIntersections,conservativeBodyGap,records};
}
