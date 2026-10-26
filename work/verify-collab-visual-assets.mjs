import {readFile} from 'node:fs/promises';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {VISUAL_CONFIGS} from '../dist/collab/visual-config.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';

const root=new URL('../dist/collab/',import.meta.url),manifest=JSON.parse(await readFile(new URL('visual-assets.json',root)));
let sourceBytes=0,compressedBytes=0;
for(const [path,spec]of Object.entries(manifest)){
  const [source,packed]=await Promise.all([readFile(new URL(path,root)),readFile(new URL(spec.file,root))]);
  assert.deepEqual(gunzipSync(packed),source,`${path}: decoded STL must be byte-identical`);
  assert.equal(createHash('sha256').update(source).digest('hex'),spec.sha256);
  sourceBytes+=source.length;compressedBytes+=packed.length;
}
for(const id of ['r1','g1','c5'])for(const field of ['id','label','rootHeight','urdf','applyMaterials'])assert.equal(VISUAL_CONFIGS[id][field],ROBOT_CONFIGS[id][field],`${id}.${field}: visual configuration drift`);
console.log(JSON.stringify({pass:true,files:Object.keys(manifest).length,sourceBytes,compressedBytes,reduction:1-compressedBytes/sourceBytes}));
