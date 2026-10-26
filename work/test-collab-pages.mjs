import assert from 'node:assert/strict';
import {readFile,stat} from 'node:fs/promises';

// Deployment checks use both the localhost root and GitHub project subpath.
// Browser/GPU/physics acceptance is exercised separately.
const root=new URL('../dist/',import.meta.url),seen=new Set();let references=0;
const html=await readFile(new URL('collab/index.html',root),'utf8');
const imports=JSON.parse(html.match(/<script type="importmap">([\s\S]*?)<\/script>/)[1]).imports;
async function resource(from,path){
  // Emscripten's dual-runtime module imports Node builtins only under Node.
  if(from==='r1/mujoco-runtime.mjs'&&path.startsWith('node:'))return;
  let spec=path;
  if(!spec.startsWith('.')){
    if(imports[spec])spec=new URL(imports[spec],new URL('collab/index.html',root)).href;
    else if(!spec.startsWith('file:'))throw new Error(`Unresolved import: ${from} → ${path}`);
  }
  const url=new URL(spec,new URL(from,root));url.search='';url.hash='';
  assert.ok(url.href.startsWith(root.href),`Resource escapes site: ${path}`);
  const info=await stat(url);references++;
  if(info.isDirectory())return;
  assert.ok(info.size>0,`Empty resource: ${path}`);
  if(!/\.(?:mjs|js)$/.test(url.pathname)||seen.has(url.href))return;
  seen.add(url.href);const source=await readFile(url,'utf8'),relative=url.href.slice(root.href.length);
  for(const match of source.matchAll(/(?:^|\n)\s*(?:import|export)\s+(?:[^;'"\n]*(?:\n[^;'"\n]*)*?\s+from\s*)?['"]([^'"]+)['"]/g))await resource(relative,match[1]);
  for(const match of source.matchAll(/\b(?:new URL|import)\(\s*['"]([^'"]+)['"]/g))await resource(relative,match[1]);
}
for(const match of html.matchAll(/<(?:link|script)\b[^>]*(?:href|src)=['"]([^'"]+)['"]/g))await resource('collab/index.html',match[1]);
const manifest=JSON.parse(await readFile(new URL('collab/visual-assets.json',root)));
for(const [source,asset]of Object.entries(manifest)){
  await resource('collab/visual-assets.json',source);await resource('collab/visual-assets.json',asset.file);
  for(const base of ['http://127.0.0.1:4173/','https://wander521.github.io/r1-simulator/']){
    const page=new URL('collab/visual-assets.json',base),resolved=new URL(asset.file,page);
    assert.ok(resolved.href.startsWith(base),`Asset escapes deployment prefix: ${asset.file}`);
  }
}
assert.ok(seen.has(new URL('collab/c5-controller.mjs',root).href),'Collaboration C5 controller is disconnected');
console.log(JSON.stringify({pass:true,modules:seen.size,references,compressedAssets:Object.keys(manifest).length}));
