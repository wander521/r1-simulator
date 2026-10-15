import * as THREE from '../vendor/three.module.min.js';

// C5 render assets retain the official 3DS face groups and source colors.
// Only the mounted colorless Dex1 STL receives the established black finish.
export const C5_FINISHES=Object.freeze({
  gripper:Object.freeze({color:0x1b2228,metalness:.24,roughness:.49,envMapIntensity:.55}),
});

export function applyC5Materials(robot){
  const materials=Object.fromEntries(Object.entries(C5_FINISHES).map(([key,value])=>[key,new THREE.MeshStandardMaterial({name:`C5 ${key}`,...value,emissive:0})]));
  const report={meshCount:0,sourceMaterials:0,finishes:{},links:{}};
  robot.traverse(mesh=>{
    if(!mesh.isMesh||!mesh.geometry?.attributes.position)return;
    let link=mesh;while(link&&!link.isURDFLink)link=link.parent;
    if(!link)return;
    const name=link.name;report.meshCount++;
    if(!/_dex1_/.test(name)){report.sourceMaterials++;report.links[name]='source';return;}
    const finish='gripper';
    mesh.material=materials[finish];mesh.userData.c5Finish=finish;
    report.finishes[finish]=(report.finishes[finish]||0)+1;report.links[name]=finish;
  });
  robot.userData.c5MaterialReport=report;
  return report;
}
