import {readdir,readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
const root=new URL('../',import.meta.url),files=(await readdir(new URL('dist/collab/',root))).filter(f=>/\.(mjs|css|html)$/.test(f)).map(f=>`dist/collab/${f}`);
files.push('dist/c5/grasp-controller.mjs','work/collab-browser-probe.mjs');
for(const id of ['r1','g1','c5'])files.push(`dist/${id}/models/${id}-dynamic-scene.xml`);
files.push('dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf');
const hashes=Object.fromEntries(await Promise.all(files.map(async file=>[file,createHash('sha256').update(await readFile(new URL(file,root))).digest('hex')])));
const path=new URL(process.env.MANIFEST??'work/validation/collab/browser/bin-run-source.json',root);
if(process.env.VERIFY==='1'){
  const previous=JSON.parse(await readFile(path));
  const changed=files.filter(f=>previous.hashes[f]!==hashes[f]);
  if(changed.length)throw new Error(`Browser source changed: ${changed.join(', ')}`);
  console.log(JSON.stringify({sourceUnchanged:true,count:files.length,capturedAt:previous.capturedAt}));
}else{
  const report={capturedAt:new Date().toISOString(),scope:'Browser verification source manifest; timestamp determines whether recorded before/after start',hashes};
  await writeFile(path,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({capturedAt:report.capturedAt,count:files.length}));
}
