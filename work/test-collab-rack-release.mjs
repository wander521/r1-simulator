import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
import test from 'node:test';

test('rack release versions both render and simulation dependency chains',async()=>{
  const html=await readFile(new URL('../dist/collab/index.html',import.meta.url),'utf8');
  const version=html.match(/app\.mjs\?v=([^"']+)/)[1];
  for(const [file,target] of [
    ['index.html','app.mjs'],['app.mjs','view.mjs'],['app.mjs','engine.mjs'],
    ['engine.mjs','simulation-worker.mjs'],['simulation-worker.mjs','bin-model.mjs'],
    ['bin-model.mjs','cell-fixtures.mjs'],
  ]){
    const source=await readFile(new URL(`../dist/collab/${file}`,import.meta.url),'utf8');
    assert.ok(source.includes(`./${target}?v=${version}`),`${file} must invalidate ${target}`);
  }
});
