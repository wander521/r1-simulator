import * as THREE from '../vendor/three.module.min.js';
import {createWorkcell, configureRenderer} from '../g1/g1-workcell.mjs';

// Only display transforms differ between stations. Their original metres,
// tabletop heights and local physical coordinates are retained exactly.
export const STATION_OFFSETS = Object.freeze({r1:[0,0,2.2], g1:[0,0,0], c5:[0,0,-2.2]});
export {configureRenderer};

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
