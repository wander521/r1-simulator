import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {createServer} from 'node:http';

// URL/import-map resolution and a deliberately stale HTTP-body cache model.
// This is not a browser engine, native cache inspection or rendered-pixel QA.
const root=new URL('../dist/',import.meta.url),release='20261016-native-camera-3';
const protectedPaths=['multi/app.mjs','multi/robot-runtime.mjs','multi/presentation.mjs','multi/workcell.mjs',
  'r1/r1-app.mjs','r1/grasp-controller.mjs','r1/grasp-kinematics.mjs','r1/r1-workcell.mjs',
  'c5/c5-app.mjs','c5/grasp-controller.mjs','c5/c5-profile.mjs'];
const protectedSet=new Set(protectedPaths),pages=['multi/index.html','r1/index.html','c5/index.html'];
const bytesByPath=new Map(),sourceHashes={},checks=[],graphs=[],negativeFixtures=[],requests=[];
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
async function bytes(path){
  assert(path&&!path.startsWith('/')&&!path.split('/').includes('..'));
  if(!bytesByPath.has(path)){const value=await readFile(new URL(path,root));bytesByPath.set(path,value);sourceHashes['dist/'+path]=sha(value);}
  return bytesByPath.get(path);
}
function check(name,fn){const detail=fn();checks.push({name,pass:true,...(detail===undefined?{}:{detail})});}
function mapFor(html,pageURL){
  const matches=[...html.matchAll(/<script\b[^>]*type=['"]importmap['"][^>]*>([\s\S]*?)<\/script>/gi)];assert.equal(matches.length,1,'Retain one merged import map');
  const imports=JSON.parse(matches[0][1]).imports,result=new Map();
  for(const[key,value]of Object.entries(imports)){
    const normalized=/^(?:\.?\.?\/|[a-z][a-z\d+.-]*:)/i.test(key)?new URL(key,pageURL).href:key;
    result.set(normalized,new URL(value,pageURL).href);
  }return result;
}
function resolveImport(specifier,parentURL,map){
  if(specifier.startsWith('node:'))return null; // Emscripten's conditional Node branch.
  const normalized=/^(?:\.?\.?\/|[a-z][a-z\d+.-]*:)/i.test(specifier)?new URL(specifier,parentURL).href:specifier;
  const target=map.get(normalized)??normalized;assert(/^https?:/.test(target),`Unmapped bare import: ${specifier}`);return target;
}
// The same token boundaries as the publication checker: quoted prose,
// comments, regexes and shader templates are not import declarations.
function references(source){
  const out=[];let i=0,previous='';
  const push=(type,value)=>{out.push({type,value});previous=value;};
  while(i<source.length){const c=source[i],n=source[i+1];
    if(/\s/.test(c)){i++;continue;}
    if(c==='/'&&n==='/'){i=source.indexOf('\n',i+2);if(i<0)break;continue;}
    if(c==='/'&&n==='*'){const end=source.indexOf('*/',i+2);assert(end>=0);i=end+2;continue;}
    if(c==='"'||c==="'"){const quote=c;let value='';i++;while(i<source.length&&source[i]!==quote){if(source[i]==='\\'){i++;const escape=source[i++];if(escape==='\n')continue;if(escape==='\r'){if(source[i]==='\n')i++;continue;}if(escape==='x'||escape==='u'){const length=escape==='x'?2:4;if(escape==='u'&&source[i]==='{'){const end=source.indexOf('}',++i);assert(end>=0);value+=String.fromCodePoint(parseInt(source.slice(i,end),16));i=end+1;}else{value+=String.fromCodePoint(parseInt(source.slice(i,i+length),16));i+=length;}}else value+=({n:'\n',r:'\r',t:'\t',b:'\b',f:'\f',v:'\v',0:'\0'})[escape]??escape;}else value+=source[i++];}assert.equal(source[i++],quote);push('string',value);continue;}
    if(c==='`'){i++;while(i<source.length){if(source[i]==='\\')i+=2;else if(source[i++]==='`')break;}push('template','<template>');continue;}
    if(c==='/'&&(!previous||/^(?:=|\(|\[|\{|,|:|;|!|\?|\||&|return|throw|case|=>)$/.test(previous))){let characterClass=false;i++;while(i<source.length){if(source[i]==='\\'){i+=2;continue;}if(source[i]==='[')characterClass=true;else if(source[i]===']')characterClass=false;else if(source[i]==='/'&&!characterClass){i++;break;}i++;}while(/[a-z]/i.test(source[i]||'')&&i<source.length)i++;push('regex','<regex>');continue;}
    if(/[a-zA-Z_$]/.test(c)){const start=i++;while(/[a-zA-Z\d_$]/.test(source[i]||'')&&i<source.length)i++;push('word',source.slice(start,i));continue;}
    push('punctuation',c);i++;
  }
  const found=[],value=index=>out[index]?.value,literal=index=>out[index]?.type==='string';
  for(let index=0;index<out.length;index++)if(['import','export'].includes(value(index))&&value(index-1)!=='.'){
    if(value(index)==='import'&&value(index+1)==='('&&literal(index+2)&&value(index+3)===')'){found.push(value(index+2));continue;}
    if(value(index)==='import'&&literal(index+1))found.push(value(index+1));
    else if(!['.','('].includes(value(index+1)))for(let next=index+1;next<out.length&&value(next)!==';';next++){if(value(next)==='from'&&literal(next+1)){found.push(value(next+1));break;}if(['const','let','class','function','default'].includes(value(next)))break;}
  }return found;
}
check('Reference scanner excludes prose/comments and covers literal static/dynamic imports',()=>{
  assert.deepEqual(references(`// import './wrong';\nconst s="import './wrong2'";import './a.mjs';import{x}from'./b.mjs';export{x}from'./c.mjs';import('./d.mjs');`),['./a.mjs','./b.mjs','./c.mjs','./d.mjs']);
});
const htmlByPage=new Map();for(const page of pages)htmlByPage.set(page,(await bytes(page)).toString('utf8'));
for(const base of ['http://127.0.0.1:4173/','https://wander521.github.io/r1-simulator/'])for(const page of pages)check(`${base}${page}: exact aliases share one release identity`,()=>{
  const pageURL=new URL(page,base),html=htmlByPage.get(page),map=mapFor(html,pageURL),entries=[...html.matchAll(/<script\b[^>]*type=['"]module['"][^>]*src=['"]([^'"]+)['"][^>]*>/gi)];assert.equal(entries.length,1);assert.equal(new URL(entries[0][1],pageURL).searchParams.get('v'),release);
  for(const path of protectedPaths){const canonical=new URL(path,base).href,target=new URL(`${path}?v=${release}`,base).href;assert.equal(map.get(canonical),target);assert.equal(map.get(target),target);}
  assert.equal(map.get('three'),new URL('vendor/three.module.min.js',base).href);
  assert.equal(map.get('three/examples/jsm/loaders/STLLoader.js'),new URL('vendor/STLLoader.js',base).href);
  for(const[key,target]of map){assert(!target.includes('/g1/')&&!/[.]wasm(?:[?]|$)|[.]urdf(?:[?]|$)|[.]STL(?:[?]|$)/.test(target),'Do not bust unchanged G1/source assets');if(key.startsWith('http')){assert.equal(new URL(target).searchParams.get('v'),release);assert.equal(new URL(target).pathname,new URL(key).pathname);}}
});
const server=createServer(async(request,response)=>{try{
  const url=new URL(request.url,'http://test.invalid'),path=decodeURIComponent(url.pathname.replace(/^\/r1-simulator\//,'').replace(/^\//,''));
  const stale=protectedSet.has(path)&&url.searchParams.get('v')!==release;
  const body=stale?Buffer.from(`/* STALE_HTTP_BODY:${path} */ export const __staleRelease = true;\n`):await bytes(path);
  response.writeHead(200,{'Content-Type':path.endsWith('.html')?'text/html; charset=utf-8':'text/javascript; charset=utf-8','Cache-Control':'public, max-age=31536000, immutable','X-Release-Cache-Fixture':stale?'old-body':'current-body'});response.end(body);requests.push({url:request.url,path,stale});
}catch(error){response.writeHead(404);response.end(error.message);}});
await new Promise((resolve,reject)=>{server.once('error',reject);server.listen(0,'127.0.0.1',resolve);});
const origin=`http://127.0.0.1:${server.address().port}`,bodyCache=new Map(),parsed=new Map(),observedPaths=new Set();
let hits=0,misses=0;
async function cachedHTTP(url){
  if(bodyCache.has(url)){hits++;return bodyCache.get(url);}
  misses++;const response=await fetch(url);assert(response.ok,`${response.status} ${url}`);const record={body:await response.text(),stale:response.headers.get('X-Release-Cache-Fixture')==='old-body',cacheControl:response.headers.get('Cache-Control')};bodyCache.set(url,record);return record;
}
async function walk(page,base,mapOverride){
  const pageURL=new URL(page,base),html=htmlByPage.get(page),map=mapOverride??mapFor(html,pageURL),entry=html.match(/<script\b[^>]*type=['"]module['"][^>]*src=['"]([^'"]+)['"]/i)[1];
  const queue=[new URL(entry,pageURL).href],visited=new Set(),protectedIdentities=new Map();let edges=0;
  while(queue.length){const url=queue.shift();if(visited.has(url))continue;visited.add(url);const record=await cachedHTTP(url);assert(!record.stale,`Stale protected dependency reached: ${url}`);
    const path=new URL(url).pathname.slice(new URL(base).pathname.length);observedPaths.add(path);
    if(protectedSet.has(path)){assert.equal(new URL(url).searchParams.get('v'),release);const identities=protectedIdentities.get(path)??new Set();identities.add(url);protectedIdentities.set(path,identities);}
    const key=sha(record.body);if(!parsed.has(key))parsed.set(key,references(record.body));
    for(const specifier of parsed.get(key)){const target=resolveImport(specifier,url,map);if(target){assert(new URL(target).pathname.startsWith(new URL(base).pathname),'Import escaped project deployment prefix');queue.push(target);edges++;}}
  }
  for(const[path,identities]of protectedIdentities)assert.equal(identities.size,1,`${path}: duplicated release identity`);
  if(page==='c5/index.html'||page==='multi/index.html')assert(protectedIdentities.has('c5/c5-profile.mjs'));
  return{page,base,modules:visited.size,edges,protectedIdentities:Object.fromEntries([...protectedIdentities].map(([path,ids])=>[path,[...ids]]))};
}
try{
  for(const prefix of ['/','/r1-simulator/']){
    const base=origin+prefix;
    // Prewarm every canonical/old-query alias with an explicitly old body.
    // The server keeps returning old bodies for these URLs throughout the run;
    // only a new, uniformly mapped release URL can reach current source bytes.
    const allMap=mapFor(htmlByPage.get('multi/index.html'),new URL('multi/index.html',base));
    for(const[key]of allMap)if(key.startsWith(origin)&&new URL(key).searchParams.get('v')!==release){const old=await cachedHTTP(key);assert(old.stale);assert.match(old.cacheControl,/max-age=31536000/);}
    const oldURL=new URL('r1/r1-workcell.mjs',base).href;assert((await cachedHTTP(oldURL)).stale);
    for(const page of pages){const detail=await walk(page,base);graphs.push(detail);check(`${prefix}${page}: fresh graph bypasses still-stale HTTP dependencies`,()=>detail);}
    const htmlOnly=new Map([...allMap].filter(([key])=>!key.startsWith('http')));
    await assert.rejects(walk('multi/index.html',base,htmlOnly),/Stale protected dependency/);negativeFixtures.push({prefix,name:'entry-only-version-does-not-refresh-grandchildren',rejected:true});
    const c5Map=mapFor(htmlByPage.get('c5/index.html'),new URL('c5/index.html',base));c5Map.delete(new URL('c5/c5-profile.mjs?v=20261015-c5-3',base).href);
    await assert.rejects(walk('c5/index.html',base,c5Map),/Stale protected dependency/);negativeFixtures.push({prefix,name:'missing-old-profile-query-alias-is-rejected',rejected:true});
    assert((await cachedHTTP(oldURL)).stale,'The old canonical body must remain stale after successful fresh entry loads');
  }
  for(const path of protectedPaths)assert(observedPaths.has(path),`Protected module not exercised: ${path}`);
  for(const[path,value]of bytesByPath)assert.equal(sha(await readFile(new URL(path,root))),sha(value),`Source changed during release-cache verification: ${path}`);
  sourceHashes['work/test-release-cache.mjs']=sha(await readFile(new URL(import.meta.url)));
  const report={pass:true,release,checks:checks.length,results:checks,negativeFixtures,graphs,sourceHashes,sourceFrozen:true,
    httpCacheSimulation:{actualLoopbackHTTP:true,bodyCacheEmulated:true,oldCanonicalAndQueryAliasesRemainStale:true,cacheHits:hits,cacheMisses:misses,httpRequests:requests.length,staleHTTPRequests:requests.filter(row=>row.stale).length},
    scope:'Standard exact-URL/import-map normalization at local and GitHub subpath bases, literal source module-graph traversal, plus actual loopback HTTP responses and an explicitly emulated immutable body cache. Not a browser module loader, browser HTTP-cache inspection, executed app, rendered-pixel or physical test.'};
  const directory=new URL('../work/validation/release-cache/',root);await mkdir(directory,{recursive:true});await writeFile(new URL('final.json',directory),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({pass:true,release,checks:checks.length,graphs:graphs.length,negativeFixtures:negativeFixtures.length,sourceFrozen:true,cacheHits:hits}));
}finally{await new Promise(resolve=>server.close(resolve));}
