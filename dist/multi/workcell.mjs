import * as THREE from '../vendor/three.module.min.js';
import {createWorkcell, configureRenderer} from '../g1/g1-workcell.mjs';

// Only display transforms differ between stations. Their original metres,
// tabletop heights and local physical coordinates are retained exactly.
export const STATION_OFFSETS = Object.freeze({r1:[0,0,1.6], g1:[0,0,0], c5:[0,0,-1.6]});
export {configureRenderer};

// Display envelopes measured from source visual meshes in named preparation
// poses (C5 also includes the original/pipe plans), plus table/plinth and 20 mm.
// These are framing guards, not a collision model or all-joint safety limits.
export const STATION_VIEW_BOUNDS=Object.freeze({
  r1:Object.freeze({min:Object.freeze([-.371,-.062,-.872]),max:Object.freeze([1.160,1.709,.872])}),
  g1:Object.freeze({min:Object.freeze([-.100,-.021,-.696]),max:Object.freeze([.800,1.343,.695])}),
  c5:Object.freeze({min:Object.freeze([-.539,-.020,-.780]),max:Object.freeze([.800,1.814,.698])}),
});

/** Fit every corner in the actual oblique camera basis, not world-axis size.
 * Only presentation coordinates change; robot scale and local physics do not.
 */
export function fitWorkstationView(bounds,{fov=42,aspect=1,viewportHeight=720,near=.02}={}) {
  if(bounds.isEmpty()||![fov,aspect,viewportHeight,near].every(Number.isFinite)
    ||fov<=0||fov>=179||aspect<=0||viewportHeight<=0||near<=0)throw new RangeError('Invalid workstation view bounds or viewport');
  const target=bounds.getCenter(new THREE.Vector3());
  const direction=new THREE.Vector3(1,.33,.06).normalize();
  const right=new THREE.Vector3().crossVectors(new THREE.Vector3(0,1,0),direction).normalize();
  const up=new THREE.Vector3().crossVectors(direction,right).normalize();
  const tan=Math.tan(THREE.MathUtils.degToRad(fov/2)),fillX=.90;
  const fillY=Math.max(.50,1-2*76/viewportHeight);
  let distance=near+.05;
  for(const x of [bounds.min.x,bounds.max.x])for(const y of [bounds.min.y,bounds.max.y])for(const z of [bounds.min.z,bounds.max.z]) {
    const corner=new THREE.Vector3(x,y,z).sub(target),forward=corner.dot(direction);
    distance=Math.max(distance,forward+Math.abs(corner.dot(right))/(tan*aspect*fillX),
      forward+Math.abs(corner.dot(up))/(tan*fillY),forward+near+.05);
  }
  return{target,position:target.clone().addScaledVector(direction,distance),distance,fillX,fillY};
}

export function createSharedWorkcell(scene) {
  const base = createWorkcell(scene, {THREE, TABLE_HEIGHT:.9});
  base.table.removeFromParent();
  base.table.traverse(node => {node.geometry?.dispose(); if (node.material) node.material.dispose();});
  base.group.name = 'shared-three-robot-workcell';
  const key = base.lights.find(light => light.isDirectionalLight && light.castShadow);
  if (key) {
    key.position.set(3,4.8,1);
    Object.assign(key.shadow.camera, {left:-4.2, right:4.2, top:4.2, bottom:-4.2, far:12});
    key.shadow.camera.updateProjectionMatrix();
  }
  for (const light of base.lights) if (light.isRectAreaLight) light.width *= 3;
  return base;
}

export function addStationTable(station) {
  const c = station.config;
  const group = new THREE.Group(); group.name = `${station.id}-grounded-worktable`;
  group.position.x = c.tableCenter;
  const tableColor = new THREE.Color().setRGB(.34*.58,.37*.58,.36*.58,THREE.LinearSRGBColorSpace);
  const top = new THREE.Mesh(new THREE.BoxGeometry(c.tableHalfX*2,.065,c.tableHalfZ*2),
    new THREE.MeshStandardMaterial({color:tableColor,metalness:.12,roughness:.55}));
  top.position.y = c.tableHeight-.0325; top.name = `${station.id}-tabletop`;
  group.add(top);
  const legs = new THREE.MeshStandardMaterial({color:0x26343f,metalness:.25,roughness:.56});
  for (const x of [-c.tableHalfX+.055,c.tableHalfX-.055]) for (const z of [-c.tableHalfZ+.055,c.tableHalfZ-.055]) {
    const leg = new THREE.Mesh(new THREE.BoxGeometry(.045,c.tableHeight-.065,.045),legs);
    leg.position.set(x,(c.tableHeight-.065)/2,z); group.add(leg);
  }
  group.traverse(node => {if (node.isMesh) {node.castShadow=true; node.receiveShadow=true;}});
  station.group.add(group);
  if (c.plinthHeight) {
    const plinth = new THREE.Mesh(new THREE.CylinderGeometry(c.plinthRadius,c.plinthRadius,c.plinthHeight,40),
      new THREE.MeshStandardMaterial({color:0x38495b,metalness:.32,roughness:.44}));
    plinth.position.y = c.plinthHeight/2; plinth.name='c5-grounded-mount';
    plinth.castShadow=plinth.receiveShadow=true; station.group.add(plinth);
  }
  return group;
}
