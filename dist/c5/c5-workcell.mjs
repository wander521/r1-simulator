// Reuse the reference workcell rather than reimplementing its reflections,
// lights, feeds and Dex1 optical frame for a different robot mechanism.
import {createWorkcell as createReferenceWorkcell,createWristVision as createDex1Vision} from '../g1/g1-workcell.mjs';
import {PLINTH_RADIUS,PLINTH_HEIGHT} from './c5-profile.mjs';
export {configureRenderer,configureOverheadCamera,WORKCELL_REFERENCE} from '../g1/g1-workcell.mjs';
export const C5_PLINTH=Object.freeze({radius:PLINTH_RADIUS,height:PLINTH_HEIGHT});

export function createWorkcell(scene,options){
  const {THREE}=options,workcell=createReferenceWorkcell(scene,options);
  workcell.group.name='c5-reference-workcell';workcell.table.name='c5-task-table';
  const plinth=new THREE.Mesh(
    new THREE.CylinderGeometry(C5_PLINTH.radius,C5_PLINTH.radius,C5_PLINTH.height,48),
    new THREE.MeshStandardMaterial({color:0x556779,metalness:.46,roughness:.44}),
  );
  plinth.name='c5-grounded-base-plinth';plinth.position.set(0,C5_PLINTH.height/2,0);plinth.castShadow=true;plinth.receiveShadow=true;
  workcell.group.add(plinth);
  return {...workcell,plinth};
}

export function createWristVision(options){
  const vision=createDex1Vision(options);
  // This is the same physical Dex1, with its original local +X TCP frame.
  // Only the route/robot annotation changes; no guessed hardware calibration.
  vision.model.traverse(node=>{node.name=node.name.replace(/^g1-/,'c5-');});
  return vision;
}
