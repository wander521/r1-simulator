import {applyRobotMaterials} from '../r1/r1-materials.mjs';
import {applyG1Materials} from '../g1/g1-materials.mjs';
import {applyC5Materials} from '../c5/c5-materials.mjs';
import {ROOT_HEIGHT as G1_HEIGHT} from '../g1/g1-profile.mjs';
import {ROOT_HEIGHT as C5_HEIGHT} from '../c5/c5-profile.mjs';

// Presentation imports no controllers, physics runtime or planning services.
const url=path=>new URL(path,import.meta.url).href;
export const VISUAL_CONFIGS=Object.freeze({
  r1:{id:'r1',label:'星海图 R1',rootHeight:0,urdf:url('../r1/assets/r1.urdf'),applyMaterials:applyRobotMaterials},
  g1:{id:'g1',label:'宇树 G1',rootHeight:G1_HEIGHT,urdf:url('../g1/assets/g1_29dof_with_dex1_1.urdf'),applyMaterials:applyG1Materials},
  c5:{id:'c5',label:'遨博 C5',rootHeight:C5_HEIGHT,urdf:url('../c5/assets/c5-dex1.urdf'),applyMaterials:applyC5Materials},
});
