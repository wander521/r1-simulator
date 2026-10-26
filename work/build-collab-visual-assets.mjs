import {readdir,readFile,mkdir,writeFile} from 'node:fs/promises';
import {gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';

// Lossless transport copies only. Original URDFs, STL files and collision
// models remain authoritative. Hash filenames invalidate changed geometry.
const root=new URL('../dist/',import.meta.url),out=new URL('collab/visual-assets/',root);
await mkdir(out,{recursive:true});const assets={};let originalBytes=0,compressedBytes=0;
for(const path of ['r1/assets/meshes/','g1/assets/meshes/','g1/assets/dex1_1/meshes/']){
  for(const file of (await readdir(new URL(path,root))).sort()){
    if(!/\.stl$/i.test(file))continue;
    const original=await readFile(new URL(path+file,root));
    const compressed=gzipSync(original,{level:9}),hash=createHash('sha256').update(original).digest('hex');
    const target=`${hash}.stl.gz`;await writeFile(new URL(target,out),compressed);
    assets[`../${path}${file}`]={file:`./visual-assets/${target}`,sourceBytes:original.length,bytes:compressed.length,sha256:hash};
    originalBytes+=original.length;compressedBytes+=compressed.length;
  }
}
await writeFile(new URL('collab/visual-assets.json',root),JSON.stringify(assets));
console.log(JSON.stringify({files:Object.keys(assets).length,originalBytes,compressedBytes,reduction:1-compressedBytes/originalBytes}));
