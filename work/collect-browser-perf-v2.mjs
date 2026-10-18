import {createServer} from 'node:http';
import {mkdir,readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';

const root=resolve(new URL('../',import.meta.url).pathname);
const output=resolve(root,'work/validation/performance-v2/browser');
const sources=['dist/multi/app.mjs','dist/multi/index.html','dist/multi/robot-runtime.mjs',
  'dist/multi/planning-worker-core.mjs','dist/multi/planning-robot.mjs',
  'dist/multi/visuals/frame-scheduler.mjs','dist/multi/visuals/scene-environment.mjs',
  'dist/multi/visuals/camera-feeds.mjs','dist/multi/visuals/camera-feed-queue.mjs',
  'dist/multi/planning-check-cache.mjs'];
const hash=bytes=>createHash('sha256').update(bytes).digest('hex');
await mkdir(output,{recursive:true});
const server=createServer(async(req,res)=>{
  const origin=req.headers.origin;
  if(origin!=='http://127.0.0.1:4173'){res.writeHead(403).end();return;}
  res.setHeader('Access-Control-Allow-Origin',origin);
  res.setHeader('Access-Control-Allow-Methods','POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers','content-type');
  if(req.method==='OPTIONS'){res.writeHead(204).end();return;}
  const match=/^\/performance-v2\/((?:baseline|optimized)-(?:desktop|mobile)-(?:idle|run|planning|cameras|ready)(?:-[123])?)\.(json|jpg|png)$/.exec(req.url??'');
  if(req.method!=='POST'||!match){res.writeHead(404).end();return;}
  try{
    const chunks=[];let size=0;
    for await(const chunk of req){size+=chunk.length;if(size>8e6)throw new Error('Payload too large');chunks.push(chunk);}
    const body=Buffer.concat(chunks),file=resolve(output,`${match[1]}.${match[2]}`);
    if(match[2]==='json'){
      const measurement=JSON.parse(body.toString('utf8'));
      const sourceHashes=Object.fromEntries(await Promise.all(sources.map(async path=>[path,hash(await readFile(resolve(root,path)))])));
      await writeFile(file,JSON.stringify({capturedAt:new Date().toISOString(),sourceHashes,measurement},null,2)+'\n');
    }else{
      if(match[2]==='jpg'&&(body[0]!==255||body[1]!==216))throw new Error('JPEG required');
      if(match[2]==='png'&&body.subarray(0,8).toString('hex')!=='89504e470d0a1a0a')throw new Error('PNG required');
      await writeFile(file,body);
    }
    res.writeHead(200,{'Content-Type':'text/plain'}).end('saved');
  }catch(error){res.writeHead(400,{'Content-Type':'text/plain'}).end(error.message);}
});
server.listen(53379,'127.0.0.1',()=>process.stdout.write(`Performance collector listening at 127.0.0.1:53379; output ${output}\n`));
process.on('SIGINT',()=>server.close(()=>process.exit(0)));
