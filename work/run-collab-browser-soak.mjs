// Sequential real-browser soak via the installed Playwright CLI, not a fake
// clock or a physics-state driver. Production sources are never edited here.
import {execFile} from 'node:child_process';
import {promisify} from 'node:util';
import {readFile,writeFile,readdir,mkdir,rename} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import path from 'node:path';
import {setTimeout as delay} from 'node:timers/promises';

const root=fileURLToPath(new URL('../',import.meta.url));
const runId=new Date().toISOString().replaceAll(':','-').replaceAll('.','-');
const out=path.join(root,'output/playwright',`collab-soak-${runId}`);
const cli='/Users/wander/.codex/skills/playwright/scripts/playwright_cli.sh';
const session='collab-soak20',total=20,exec=promisify(execFile);
const env={...process.env,PATH:`/Users/wander/.nvm/versions/node/v22.23.2/bin:${process.env.PATH}`};
const status={runId,pid:process.pid,session,total,completed:0,passed:0,failed:0,state:'starting',round:0,startedAt:new Date().toISOString(),updatedAt:null,output:out,results:[]};
await mkdir(out,{recursive:true});
const checkpoint=async()=>{
  status.updatedAt=new Date().toISOString();
  await writeFile(path.join(out,'status.next.json'),JSON.stringify(status,null,2));
  await rename(path.join(out,'status.next.json'),path.join(out,'status.json'));
};
const log=async(message,data={})=>{
  const line=JSON.stringify({at:new Date().toISOString(),message,...data});
  console.log(line);await writeFile(path.join(out,'runner.jsonl'),line+'\n',{flag:'a'});
};
const files=(await readdir(path.join(root,'dist/collab'))).filter(f=>/\.(mjs|css|html)$/.test(f)).map(f=>`dist/collab/${f}`);
files.push('dist/c5/grasp-controller.mjs','work/collab-browser-probe.mjs','work/verify-collab-bin-browser.mjs','work/run-collab-browser-soak.mjs');
for(const id of ['r1','g1','c5'])files.push(`dist/${id}/models/${id}-dynamic-scene.xml`);
const hashes=async()=>Object.fromEntries(await Promise.all(files.map(async f=>[f,createHash('sha256').update(await readFile(path.join(root,f))).digest('hex')])));
const source=await hashes();await writeFile(path.join(out,'source.json'),JSON.stringify(source,null,2));
const assertSource=async()=>{const now=await hashes(),changed=files.filter(f=>now[f]!==source[f]);if(changed.length)throw new Error(`SOURCE_CHANGED: ${changed.join(', ')}`);};
async function pw(...args){
  const {stdout,stderr}=await exec(cli,['--session',session,...args],{cwd:root,env,timeout:90000,maxBuffer:8*1024*1024});
  if(/^### Error/m.test(stdout))throw new Error(stdout);
  const block=stdout.match(/### Result\s*\n([\s\S]*?)(?=\n### |$)/);
  if(args[0]==='run-code'&&!block)throw new Error(`Missing CLI result: ${stdout}\n${stderr}`);
  return block?JSON.parse(block[1].trim()):stdout;
}
const js=code=>pw('run-code',`async page => { ${code} }`);
await checkpoint();await log('suite-created',{out,total});
try{
  await pw('open','http://127.0.0.1:4173/collab/','--headed');
  await js('await page.setViewportSize({width:1512,height:982}); await page.waitForFunction(()=>window.__collabDiagnostics?.().ready, null, {timeout:60000}); return {ready:true};');
  for(let round=1;round<=total;round++){
    await assertSource();status.round=round;status.state='preparing';status.current=null;await checkpoint();
    const prefix=path.join(out,`round-${String(round).padStart(2,'0')}`);
    // Reload creates new physics and planning Workers, preventing stale plans,
    // state carryover or an observer from a previous failure leaking forward.
    if(round>1)await js('await page.reload(); await page.waitForFunction(()=>window.__collabDiagnostics?.().ready, null, {timeout:60000}); return {ready:true};');
    await pw('snapshot');
    await js('await page.getByRole("button",{name:"复位工作站",exact:true}).click(); await page.waitForFunction(()=>window.__collabDiagnostics().snapshot.state==="idle"); await page.addScriptTag({path:"work/collab-browser-probe.mjs",type:"module"}); await page.waitForFunction(()=>typeof window.installCollaborationBrowserProbe==="function"); await page.evaluate(()=>window.installCollaborationBrowserProbe()); await page.getByRole("button",{name:"开始 5 分钟作业",exact:true}).click(); return {started:true};');
    const started=Date.now();status.state='running';await checkpoint();await log('round-start',{round});
    let observation,outcome;
    while(true){
      await delay(15000);
      observation=await js('return await page.evaluate(()=>{const d=window.__collabDiagnostics(),s=d.snapshot;return {state:s.state,error:d.error??s.error,commandError:s.commandError,elapsed:s.elapsedSeconds,simTime:s.simTime,parts:s.completedParts,returned:s.returnedBins,robots:s.robots?.map(r=>({id:r.id,state:r.state,error:r.error,phase:r.phase})),visibility:document.visibilityState};});');
      status.current=observation;await checkpoint();await log('round-progress',{round,...observation});
      if(observation.error||observation.commandError||observation.robots?.some(r=>r.error)){outcome='application-failure';break;}
      if(observation.state==='complete'){outcome='complete';break;}
      if(observation.state==='failed'){outcome='application-failure';break;}
      if(Date.now()-started>900000){outcome='timeout';break;}
    }
    const reportFile=`${prefix}.json`;
    await js(`const pending=page.waitForEvent("download"); await page.evaluate(()=>{const report=window.__stopCollabAcceptance();const a=document.createElement("a");a.href=URL.createObjectURL(new Blob([JSON.stringify(report)],{type:"application/json"}));a.download="round-report.json";a.click();}); await (await pending).saveAs(${JSON.stringify(reportFile)}); await page.screenshot({path:${JSON.stringify(prefix+'.png')}}); return {saved:true};`);
    // Preserve specific fault details and screenshots BEFORE resetting next round.
    const raw=JSON.parse(await readFile(reportFile));
    let verifierExit=0;
    try{await exec(process.execPath,['work/verify-collab-bin-browser.mjs',reportFile,`${prefix}-checks.json`],{cwd:root,env,timeout:30000,maxBuffer:8*1024*1024});}
    catch(error){verifierExit=error.code??1;await writeFile(`${prefix}-verifier.log`,String(error.stdout??error.stack));}
    let checks;try{checks=JSON.parse(await readFile(`${prefix}-checks.json`));}catch{checks={pass:false,checks:[],error:'Verifier did not produce a report'};}
    await assertSource();
    const result={round,outcome,pass:outcome==='complete'&&verifierExit===0&&checks.pass,wallSeconds:(Date.now()-started)/1000,
      error:raw.final?.snapshot?.error??raw.final?.error??null,actions:raw.final?.snapshot?.actions,
      failedChecks:checks.checks.filter(c=>!c.pass),report:reportFile,screenshot:`${prefix}.png`};
    status.results.push(result);status.completed++;if(result.pass)status.passed++;else status.failed++;
    await checkpoint();await log('round-finished',result);
  }
  status.state='complete';status.finishedAt=new Date().toISOString();await checkpoint();
  await log('suite-complete',{passed:status.passed,failed:status.failed,total});
}catch(error){
  // Infrastructure failure is NOT counted as a completed robot test or silently
  // retried. An explicit continuation can preserve and resume this evidence.
  status.state='blocked';status.infrastructureError=error.stack;await checkpoint();await log('suite-blocked',{error:error.stack});process.exitCode=1;
}
