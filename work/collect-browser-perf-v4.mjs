import {createServer} from 'node:http';
import {mkdir,readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';

// Local-only evidence sink for native-browser measurements and JPEG captures.
// It does not expose a simulator control API or alter any production resource.
const root=resolve(new URL('../',import.meta.url).pathname);
const output=resolve(root,'work/validation/performance-v4/browser');
const sources=['work/collect-browser-perf-v4.mjs','dist/multi/app.mjs','dist/multi/index.html',
  'dist/multi/coordinator.mjs','dist/multi/robot-runtime.mjs','dist/multi/planning.mjs',
  'dist/multi/pipe-controller.mjs','dist/multi/background-planning.mjs',
  'dist/multi/planning-worker.mjs','dist/multi/planning-worker-core.mjs',
  'dist/multi/planning-worker-transport.mjs','dist/multi/planning-worker-protocol.mjs',
  'dist/multi/planning-serialization.mjs','dist/multi/planning-robot.mjs',
  'dist/multi/planning-ik-context.mjs','dist/multi/planning-budget.mjs',
  'dist/multi/planning-check-cache.mjs','dist/multi/standalone-planning.mjs','dist/multi/mesh-cache.mjs','dist/multi/shadow-state.mjs',
  ...['r1','g1','c5'].flatMap(id=>[`dist/${id}/${id}-app.mjs`,`dist/${id}/index.html`]),'dist/multi/visuals/frame-scheduler.mjs',
  'dist/multi/visuals/camera-feeds.mjs','dist/multi/visuals/camera-feed-queue.mjs'];
await mkdir(output,{recursive:true});
const server=createServer(async(req,res)=>{
  const origin=req.headers.origin;
  if(origin!=='http://127.0.0.1:4173'){res.writeHead(403).end();return;}
  res.setHeader('Access-Control-Allow-Origin',origin);
  res.setHeader('Access-Control-Allow-Methods','POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers','content-type');
  if(req.method==='OPTIONS'){res.writeHead(204).end();return;}
  const match=/^\/performance-v4\/((?:desktop|mobile)-(?:ready|planning|run|stopped|restored)(?:-(?:night|hangar|r1|g1|c5))?)\.(json|jpg)$/.exec(req.url??'');
  if(req.method!=='POST'||!match){res.writeHead(404).end();return;}
  try{
    const chunks=[];let size=0;
    for await(const chunk of req){size+=chunk.length;if(size>8e6)throw new Error('Payload too large');chunks.push(chunk);}
    const body=Buffer.concat(chunks),file=resolve(output,`${match[1]}.${match[2]}`);
    if(match[2]==='json'){
      const measurement=JSON.parse(body.toString('utf8'));
      const sourceHashes=Object.fromEntries(await Promise.all(sources.map(async path=>[path,
        createHash('sha256').update(await readFile(resolve(root,path))).digest('hex')])));
      await writeFile(file,JSON.stringify({capturedAt:new Date().toISOString(),sourceHashes,measurement},null,2)+'\n');
    }else{
      if(body[0]!==255||body[1]!==216)throw new Error('JPEG required');
      await writeFile(file,body);
    }
    res.writeHead(200,{'Content-Type':'text/plain'}).end('saved');
  }catch(error){res.writeHead(400,{'Content-Type':'text/plain'}).end(error.message);}
});
server.listen(53379,'127.0.0.1',()=>process.stdout.write(`Performance-v4 collector listening at 127.0.0.1:53379\n`));
process.on('SIGINT',()=>server.close(()=>process.exit(0)));
