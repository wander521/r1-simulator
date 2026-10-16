import assert from 'node:assert/strict';
import {readFile, mkdir, writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import vm from 'node:vm';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';
import {STAGE_NAMES, taskProgress} from '../dist/multi/presentation.mjs';
import {pipeControllerClass} from '../dist/multi/pipe-controller.mjs';

// These checks intentionally have no browser, GPU, native-pointer or MuJoCo
// surrogate. Small source functions execute in explicit orchestration fixtures;
// they prove wiring and pure contracts, not rendered pixels or physical grasps.
const root=new URL('../',import.meta.url),ids=['r1','g1','c5'];
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)??'work/validation/multi-pipes/ui-final.json';
const sourcePaths=['work/test-pipe-ui.mjs','dist/multi/index.html','dist/multi/app.mjs',
  'dist/multi/styles.css','dist/multi/presentation.mjs','dist/multi/coordinator.mjs',
  'dist/multi/robot-runtime.mjs','dist/multi/planning.mjs','dist/multi/pipe-layout.mjs','dist/multi/pipe-controller.mjs','dist/multi/visuals/frame-scheduler.mjs'];
async function hashes(){return Object.fromEntries(await Promise.all(sourcePaths.map(async path=>
  [path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));}
const sourceHashes=await hashes(),source=Object.fromEntries(await Promise.all(sourcePaths.slice(1).map(async path=>
  [path,await readFile(new URL(path,root),'utf8')])));
const html=source['dist/multi/index.html'],app=source['dist/multi/app.mjs'],runtimeSource=source['dist/multi/robot-runtime.mjs'];
const checks=[],errors=[],progressTraces={},contractTraces={};
async function check(name,fn){try{const detail=await fn();checks.push({name,pass:true,...(detail===undefined?{}:{detail})});}
  catch(error){checks.push({name,pass:false,error:error.message});errors.push({name,message:error.message});}}

function namedFunction(text,name){
  const match=new RegExp(`(?:async\\s+)?function\\s+${name}\\s*\\(`).exec(text);
  assert(match,`${name}: source function missing`);const start=match.index,body=text.indexOf('{',text.indexOf(')',start));
  let depth=0,quote=null,escaped=false,lineComment=false,blockComment=false;
  for(let index=body;index<text.length;index++){
    const char=text[index],next=text[index+1];
    if(lineComment){if(char==='\n')lineComment=false;continue;}
    if(blockComment){if(char==='*'&&next==='/'){blockComment=false;index++;}continue;}
    if(quote){if(escaped)escaped=false;else if(char==='\\')escaped=true;else if(char===quote)quote=null;continue;}
    if(char==='/'&&next==='/'){lineComment=true;index++;continue;}
    if(char==='/'&&next==='*'){blockComment=true;index++;continue;}
    if(['\'','"','`'].includes(char)){quote=char;continue;}
    if(char==='{')depth++;else if(char==='}'&&!--depth)return text.slice(start,index+1);
  }
  assert.fail(`${name}: unbalanced source function`);
}
function functionFixture(name,globals){
  const context=vm.createContext(globals);vm.runInContext(`${namedFunction(app,name)}\nthis.invoke=${name};`,context);
  return context;
}
function station(id,{ready=true,accept=true,configurationFailure=null}={}){
  const calls={configure:[],start:[],stop:0,reset:0,step:[]},controller={running:false,state:'idle',message:'',time:0,classification:null};
  const runtime={id,ready,config:{label:id.toUpperCase(),defaultArm:id==='c5'?'left':'right'},group:{visible:true},calls,controller,
    mode:'sort',seed:0,objectPosition:[.4,.9,.1],
    configureTask(task,{seed}={}){
      calls.configure.push({task,seed});
      if(configurationFailure?.(task,calls.configure.length))return {ok:false,reason:'测试工作台准备失败'};
      this.mode=task;if(seed!==undefined)this.seed=seed;controller.time=0;controller.running=false;controller.state='idle';
      controller.classification=task==='pipes'?{total:8,completed:0,seed:this.seed,started:false,armSequence:[],items:[]}:null;
      return {ok:true,task,seed:this.seed};
    },
    start(options){calls.start.push({...options,time:controller.time});controller.running=true;
      controller.state=accept?'approach':'shy';if(!accept)controller.message='测试圆管路径不可达';
      if(accept&&options.task==='pipes')controller.classification.started=true;return accept;},
    stop(){calls.stop++;controller.running=false;controller.state='stopped';controller.message='已停止';},
    reset(){calls.reset++;controller.running=false;controller.state='idle';controller.time=0;if(this.mode==='pipes'){
      this.seed++;controller.classification={total:8,completed:0,seed:this.seed,started:false,armSequence:[],items:[]};}},
    step(dt){calls.step.push(dt);controller.time+=dt;},
    get snapshot(){return {id,time:controller.time,state:controller.state,running:controller.running,message:controller.message,
      task:this.mode,classification:controller.classification?{...controller.classification}:null};},
  };return runtime;
}
function coordinatorFixture(options={}){
  const stations=new Map(ids.map(id=>[id,station(id,options[id])]));
  return {stations,c:new MultiRobotCoordinator({stations,selectedIds:options.selectedIds??ids})};
}
const startOptions={task:'pipes',objectId:'pipe-red-4'};

await check('The native unified task select includes the Chinese pipe-classification option',()=>{
  assert(/<label[^>]*for="task-mode"/.test(html));
  assert(/<select[^>]*id="task-mode"[^>]*>[^]*?<option\s+value="pipes">圆管分类<\/option>/.test(html));
  assert(html.includes('统一抓取任务'));assert(!/<(?:div|span)[^>]*role="(?:listbox|combobox)"/.test(html));
});
await check('Task instructions specify eight cylinders, exact colors/counts and near/far table regions',()=>{
  const info=html.match(/<div\s+id="pipe-task-info"[^>]*>([^]*?)<\/div>/)?.[1];assert(info);
  for(const text of ['近侧随机摆放','黄色圆柱 4 个','红色圆柱 4 个','远侧同色方框','每台完成 8 个'])assert(info.includes(text),text);
  assert(info.includes('R1 / G1 自动交替左右臂'));assert(info.includes('C5 使用单臂'));
});
await check('Pipe mode hides manual arm and legacy single-object choices but preserves them for other tasks',()=>{
  assert(app.includes("$('.object-options').hidden=pipes"));
  assert(app.includes("$('#pipe-task-info').hidden=!pipes"));
  assert(app.includes("$('#arm-field').hidden=pipes||focus?.id==='c5'"));
  assert(app.includes("$('#arm-select').disabled=busy||pipes"));
  for(const object of ['red','blue','yellow'])assert(html.includes(`data-object="${object}"`));
  assert(app.includes("pipes?'开始执行'"));assert(app.includes("${selected.length} 台"));
});
await check('Randomize/task/participant controls lock while preparing or running, not observation focus',()=>{
  const expression=app.match(/\bconst busy=([^,;]+)/)?.[1];assert(expression);
  for(const state of [{running:true,planning:false},{running:false,planning:true},{running:false,planning:false}]){
    for(const preparing of [false,true])assert.equal(new Function('coordinator','preparing',`return (${expression});`)(state,preparing),state.running||state.planning||preparing);
  }
  assert(app.includes("$('#randomize-pipes').disabled=busy||!stations.size"));
  assert(app.includes("$('#task-mode').disabled=busy"));
  assert(app.includes('input.disabled=busy||!stations.has(input.dataset.robot)'));
  assert(app.includes("$('#focus-robot').disabled=!focus"));
  assert(app.includes("$('#randomize-pipes').addEventListener('click',()=>configureTask('pipes',{randomize:true}))"));
});
await check('Rendered pipe progress text includes per-station item counts and automatic active-arm detail',()=>{
  assert(app.includes('已归类 ${classification.completed} / ${classification.total} 个'));
  assert(app.includes("id==='c5'?'单臂':snapshot.arm==='left'?'左臂':'右臂'"));
  assert(app.includes("STAGE_NAMES[snapshot.state]??'执行中'"));
  assert(app.includes('progress=taskProgress({...snapshot,itemIndex:station.controller.itemIndex})'));
});
await check('Start forwards the selected unified task without requiring a legacy color object ID',()=>{
  assert(app.includes("coordinator.start({objectId:selectedObject,task:$('#task-mode').value,armByRobot,reducedMotion})"));
  assert(runtimeSource.includes("if(task==='pipes')"));assert(runtimeSource.includes('controller.startClassification()'));
  assert(runtimeSource.includes("setArm(arm){if(disposed||controller.running||taskMode==='pipes'"));
});

function configureFixture({running=false,busy=false,ok=true}={}){
  const calls={configure:[],endDrag:0,updates:0,clear:0,fit:0,random:0,clockReset:[]},taskSelect={value:'pipes'};
  const globals={coordinator:{running,configureTask(task,options){calls.configure.push({task,options:{...options}});return ok?{ok:true}:{ok:false,reason:'测试准备失败'};}},
    preparing:busy,currentTask:'sort',selectedObject:'blue',notice:'',lastFrame:0,accumulator:5,lastFeed:3,
    focused:()=>({selected:'pipe-yellow-2'}),endDrag:()=>calls.endDrag++,updateUI:()=>calls.updates++,clearHistories:()=>calls.clear++,fitView:()=>calls.fit++,
    renderer:{shadowMap:{needsUpdate:false}},frameScheduler:{reset:timestamp=>calls.clockReset.push(timestamp)},
    requestAnimationFrame:callback=>callback(123),performance:{now:()=>456},Uint32Array,
    crypto:{getRandomValues(array){calls.random++;array[0]=0xdecafbad;return array;}},$:selector=>{assert.equal(selector,'#task-mode');return taskSelect;}};
  return {context:functionFixture('configureTask',globals),calls,taskSelect};
}
await check('Actual source task configuration forwards cryptographic random seed and resets UI clocks after preparation',async()=>{
  const {context,calls}=configureFixture();await context.invoke('pipes',{randomize:true});
  assert.equal(calls.configure.length,1);assert.deepEqual(calls.configure[0],{task:'pipes',options:{seed:0xdecafbad}});
  assert.equal(calls.random,1);assert.equal(calls.endDrag,1);assert.equal(calls.clear,1);assert.equal(calls.fit,0,'Task configuration must preserve the user camera');
  assert.deepEqual(calls.clockReset,[456]);assert.equal(context.renderer.shadowMap.needsUpdate,true);
  assert.equal(context.currentTask,'pipes');assert.equal(context.selectedObject,'pipe-yellow-2');
  assert.equal(context.preparing,false);assert.equal(context.accumulator,0);assert.equal(context.lastFrame,456);assert.equal(context.lastFeed,0);
});
await check('Standard non-pipe task changes do not request a random pipe seed',async()=>{
  const {context,calls}=configureFixture();await context.invoke('sort');
  assert.equal(calls.random,0);assert.deepEqual(calls.configure,[{task:'sort',options:{}}]);
});
await check('Entering pipe mode randomizes the layout even without pressing the separate randomize action',async()=>{
  const {context,calls}=configureFixture();await context.invoke('pipes');
  assert.equal(calls.random,1);assert.deepEqual(calls.configure,[{task:'pipes',options:{seed:0xdecafbad}}]);
});
await check('Preparing/running task configuration cannot rebuild or randomize a station',async()=>{
  for(const state of [{running:true},{busy:true}]){
    const {context,calls}=configureFixture(state);await context.invoke('pipes',{randomize:true});
    assert.equal(calls.configure.length,0);assert.equal(calls.random,0);assert.equal(calls.endDrag,0);
  }
});
await check('Failed preparation restores the task picker and leaves the original selected object/history intact',async()=>{
  const {context,calls,taskSelect}=configureFixture({ok:false});await context.invoke('pipes',{randomize:true});
  assert.equal(taskSelect.value,'sort');assert.equal(context.currentTask,'sort');assert.equal(context.selectedObject,'blue');
  assert.equal(context.notice,'测试准备失败');assert.equal(context.preparing,false);assert.equal(calls.clear,0);assert.equal(calls.fit,0);
});

function selectionFixture({pipes=true,running=false,busy=false,started=false}={}){
  const calls=Object.fromEntries(ids.map(id=>[id,[]]));
  const stations=new Map(ids.map(id=>[id,{targets:[{id:'pipe-red-3',label:'红色圆管 3'}],controller:{classification:{started,completed:started?3:0}},
    select:value=>calls[id].push(value)}]));
  const context=functionFixture('selectObject',{coordinator:{running,selectedIds:ids},preparing:busy,selectedObject:'blue',pipeTask:()=>pipes,
    focused:()=>stations.get('g1'),stations,notice:'',objectNames:{blue:'蓝色长方块'},updateUI:()=>{}});
  return {context,calls,stations};
}
await check('Actual pipe selection affects only the focused station and still classifies all eight objects',()=>{
  const {context,calls}=selectionFixture();context.invoke('pipe-red-3');
  assert.deepEqual(calls,{r1:[],g1:['pipe-red-3'],c5:[]});assert.equal(context.selectedObject,'pipe-red-3');
  assert(context.notice.includes('全部 8 个圆柱'));
});
await check('Legacy color selection remains shared; preparing and active pipe runs reject editing',()=>{
  const legacy=selectionFixture({pipes:false});legacy.context.invoke('blue');for(const value of Object.values(legacy.calls))assert.deepEqual(value,['blue']);
  for(const state of [{running:true},{busy:true}]){const {context,calls}=selectionFixture(state);context.invoke('pipe-red-3');assert(Object.values(calls).every(values=>values.length===0));}
});
await check('Stopped/finished pipe batches reject object selection without changing recorded item progress',()=>{
  const {context,calls,stations}=selectionFixture({started:true}),before=[...stations.values()].map(s=>({...s.controller.classification}));
  context.invoke('pipe-red-3');assert(Object.values(calls).every(values=>values.length===0));
  assert.deepEqual([...stations.values()].map(s=>({...s.controller.classification})),before);
});
await check('The actual pointer-down source refuses dragging a started pipe batch before focus, selection or capture changes',()=>{
  const calls={ray:0,focus:0,select:0,capture:0},target={id:'pipe-red-3'},station={id:'g1',targets:[target],solidMeshes:[],controller:{classification:{started:true,completed:3}}};
  const context=functionFixture('down',{coordinator:{running:false,selectedIds:['g1']},preparing:false,drag:null,
    ray:()=>calls.ray++,raycaster:{intersectObjects:()=>[{object:{userData:{robotId:'g1',taskId:target.id}}}]},
    stations:new Map([['g1',station]]),pipeTask:()=>true,setFocus:()=>calls.focus++,selectObject:()=>calls.select++,
    canvas:{setPointerCapture:()=>calls.capture++}});
  context.invoke({button:0,isPrimary:true,pointerId:7});assert.deepEqual(calls,{ray:1,focus:0,select:0,capture:0});
  assert.equal(context.drag,null);assert.equal(station.controller.classification.completed,3);
  const keyHandler=app.slice(app.indexOf("window.addEventListener('keydown'"),app.indexOf('\nfunction drawChart'));
  assert(keyHandler.includes('if(pipeTask()&&station.controller.classification?.started)return;'));
  assert(keyHandler.indexOf('classification?.started')<keyHandler.indexOf('station.moveObject'));
});
await check('Start-button source disables consumed selected batches with explicit new-randomization guidance',()=>{
  const needsNewBatch=app.match(/const needsNewBatch=([^;]+);/)?.[1];assert(needsNewBatch);
  const disabled=app.match(/\$\('#start'\)\.disabled=([^;]+);/)?.[1];assert(disabled);
  const caption=app.match(/\$\('#start'\)\.textContent=([^;]+);/)?.[1];assert(caption);
  for(const consumed of [false,true]){
    const stations=new Map(ids.map(id=>[id,{controller:{classification:{started:id==='g1'&&consumed}}}]));
    const context=vm.createContext({pipes:true,selected:ids,stations,busy:false,preparing:false,coordinator:{planning:false}});
    vm.runInContext(`this.needsNewBatch=${needsNewBatch};this.disabled=${disabled};this.caption=${caption};`,context);
    assert.equal(context.needsNewBatch,consumed);assert.equal(context.disabled,consumed);
    assert.equal(context.caption,consumed?'请先重新随机摆放':'开始执行 · 3 台');
    context.selected=['r1','c5'];vm.runInContext(`this.needsNewBatch=${needsNewBatch};this.disabled=${disabled};`,context);
    assert.equal(context.needsNewBatch,false);assert.equal(context.disabled,false);
    context.coordinator.planning=true;context.busy=true;vm.runInContext(`this.disabled=${disabled};this.caption=${caption};`,context);
    assert.equal(context.disabled,true);assert.equal(context.caption,'后台检查路径…');
  }
});
await check('Pointer and keyboard pipe editing remain station-local; empty-space orbit stays separate',()=>{
  assert(app.includes('setFocus(station.id);selectObject(target.id)'));
  assert(app.includes('const {station,target}=drag'));
  assert(app.includes('station.moveObject(target.id,station.worldToLocal(point.add(drag.offset)))'));
  assert(app.includes("pipeTask()?station.selected:selectedObject"));
  assert(app.includes('if(!hit)return'));assert(app.includes('orbit.enabled=false'));assert(app.includes('orbit.enabled=true'));
  assert(app.includes('hit.object.userData.robotId'));assert(app.includes('hit.object.userData.taskId'));
});
await check('Reset preserves pipe mode, delegates to every station and exposes rerandomization in Chinese',()=>{
  assert(app.includes("endDrag();coordinator?.reset();selectedObject=pipeTask()?focused().selected:'blue'"));
  assert(app.includes("if(!pipeTask())for(const station of stations.values())station.select('blue')"));
  assert(app.includes("pipes?'复位并重新随机摆放':'复位全部工作台'"));
  assert(runtimeSource.includes('seed??normalizePipeSeed(pipeLayout.seed+1)'));
});

await check('Configuring pipes updates every ready station, including hidden ones, with the exact requested seed',()=>{
  const {c,stations}=coordinatorFixture();assert(c.select(['g1']).ok);for(let index=0;index<11;index++)c.step();
  assert(c.configureTask('pipes',{seed:521}).ok);assert.equal(c.task,'pipes');assert.equal(c.tick,0);
  assert.deepEqual(c.selectedIds,['g1']);assert.deepEqual(c.runIds,[]);assert.equal(c.startTick,null);assert.deepEqual(c.startTimes,{});
  for(const [id,s]of stations){assert.deepEqual(s.calls.configure,[{task:'pipes',seed:521}]);assert.equal(s.group.visible,id==='g1');}
  assert(c.message.includes('4 黄 + 4 红'));assert(c.message.includes('远侧同色框'));
});
await check('Explicit seed zero remains reproducible rather than falling through to an implicit default',()=>{
  const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes',{seed:0}).ok);
  for(const s of stations.values())assert.deepEqual(s.calls.configure,[{task:'pipes',seed:0}]);
});
await check('Unknown task and active-run reconfiguration are rejected before any new station mutation',()=>{
  const {c,stations}=coordinatorFixture();assert.equal(c.configureTask('unknown').ok,false);
  assert([...stations.values()].every(s=>!s.calls.configure.length));
  assert(c.configureTask('pipes').ok);assert(c.start(startOptions).ok);const before=[...stations.values()].map(s=>s.calls.configure.length);
  assert.equal(c.configureTask('sort').ok,false);assert.deepEqual([...stations.values()].map(s=>s.calls.configure.length),before);
  assert.equal(c.task,'pipes');assert.equal(c.running,true);
});
await check('A partial scene-preparation failure restores every ready station to the previous task atomically',()=>{
  const {c,stations}=coordinatorFixture({g1:{configurationFailure:(task,count)=>task==='pipes'&&count===1}});
  const result=c.configureTask('pipes',{seed:12});assert.equal(result.ok,false);assert(result.reason.includes('测试工作台准备失败'));
  for(const s of stations.values()){assert.equal(s.mode,'sort');assert.equal(s.calls.stop,1);assert.equal(s.calls.reset,1);assert.equal(s.controller.running,false);}
  assert.deepEqual(stations.get('r1').calls.configure.map(item=>item.task),['pipes','sort']);
  assert.deepEqual(stations.get('g1').calls.configure.map(item=>item.task),['pipes','sort']);
  assert.deepEqual(stations.get('c5').calls.configure.map(item=>item.task),['sort']);
  assert.equal(c.task,'sort');assert.equal(c.state,'failed');assert.equal(c.tick,0);assert.deepEqual(c.runIds,[]);
});
await check('Unsupported and unavailable stations never produce a partially configured pipe task',()=>{
  const missing=coordinatorFixture();delete missing.stations.get('g1').configureTask;
  assert.equal(missing.c.configureTask('pipes').ok,false);assert.equal(missing.c.task,'sort');
  assert([...missing.stations.values()].every(s=>s.calls.stop===1&&s.calls.reset===1));
  const partial=coordinatorFixture({g1:{ready:false},selectedIds:['r1']});assert(partial.c.configureTask('pipes',{seed:44}).ok);
  assert.equal(partial.stations.get('g1').calls.configure.length,0);assert.equal(partial.stations.get('r1').mode,'pipes');assert.equal(partial.stations.get('c5').mode,'pipes');
});

for(const selectedIds of [['r1'],['g1'],['c5'],['r1','g1'],['g1','c5'],ids])await check(`Pipe task starts only selected ${selectedIds.join('+')} at one shared tick`,()=>{
  const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes',{seed:52}).ok);assert(c.select(selectedIds).ok);
  for(let index=0;index<17;index++)c.step();const result=c.start(startOptions);
  assert.equal(result.ok,true);assert.deepEqual(result.runIds,selectedIds);assert.equal(result.startTick,17);
  for(const [id,s]of stations){assert.equal(s.calls.start.length,selectedIds.includes(id)?1:0);
    if(selectedIds.includes(id)){assert.equal(s.calls.start[0].task,'pipes');assert.equal(s.calls.start[0].objectId,'pipe-red-4');assert(Math.abs(s.calls.start[0].time-.017)<1e-12);}}
  const times=Object.values(result.startTimes);assert(Math.max(...times)-Math.min(...times)<1e-12);
  contractTraces[selectedIds.join('+')]={startTick:result.startTick,startTimes:result.startTimes};
});
await check('Clock mismatch and invalid/empty participation prevent every pipe start',()=>{
  const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes').ok);stations.get('g1').controller.time=.002;
  assert.equal(c.start(startOptions).ok,false);assert([...stations.values()].every(s=>!s.calls.start.length));
  stations.get('g1').controller.time=0;assert.equal(c.select([]).ok,false);assert.equal(c.select(['unknown']).ok,false);
  assert.deepEqual(c.selectedIds,ids);assert.equal(c.start({task:'unknown'}).ok,false);assert([...stations.values()].every(s=>!s.calls.start.length));
});
await check('Rejected pipe planning cancels all attempted stations without resetting or teleporting objects',()=>{
  const {c,stations}=coordinatorFixture({g1:{accept:false}});assert(c.configureTask('pipes').ok);
  const positions=[...stations.values()].map(s=>[...s.objectPosition]),result=c.start(startOptions);
  assert.equal(result.ok,false);assert(result.reason.includes('测试圆管路径不可达'));assert.equal(c.startTick,null);assert.deepEqual(c.startTimes,{});
  assert.equal(stations.get('r1').calls.stop,1);assert.equal(stations.get('g1').calls.stop,1);assert.equal(stations.get('c5').calls.start.length,0);
  assert([...stations.values()].every(s=>s.calls.reset===0&&!s.controller.running));assert.deepEqual([...stations.values()].map(s=>s.objectPosition),positions);
});
await check('Every ready world keeps the identical 1 ms clock while the selected pipe queue runs',()=>{
  const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes').ok);assert(c.select(['c5']).ok);assert(c.start(startOptions).ok);
  for(let index=0;index<125;index++)c.step();
  for(const s of stations.values()){assert.equal(s.calls.step.length,125);assert(s.calls.step.every(dt=>dt===.001));assert(Math.abs(s.controller.time-.125)<1e-12);}
  for(const invalid of [0,.002,NaN,Infinity])assert.throws(()=>c.step(invalid));assert.equal(c.tick,125);
});
await check('Stop cancels selected tasks without a reset and reset restores all pipe stations without changing participants',()=>{
  const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes',{seed:19}).ok);assert(c.select(['r1','c5']).ok);assert(c.start(startOptions).ok);c.step();c.stop();
  assert.equal(c.running,false);assert.equal(c.state,'stopped');for(const [id,s]of stations){assert.equal(s.calls.stop,id==='g1'?0:1);assert.equal(s.calls.reset,0);}
  c.reset();assert.deepEqual(c.selectedIds,['r1','c5']);assert.equal(c.task,'pipes');assert.equal(c.state,'idle');assert.equal(c.tick,0);
  assert(c.message.includes('重新随机摆放'));for(const s of stations.values()){assert.equal(s.calls.reset,1);assert.equal(s.seed,20);assert.equal(s.controller.time,0);}
});
await check('A consumed pipe batch cannot restart after stop or erase completed counters until reset/new configuration',()=>{
  const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes',{seed:29}).ok);assert(c.start(startOptions).ok);c.step();
  for(const [index,s]of [...stations.values()].entries())s.controller.classification.completed=index+2;
  c.stop();assert(c.message.includes('保留已归类进度'));assert(c.message.includes('新批次'));
  const before=[...stations.values()].map(s=>({...s.controller.classification})),batchNumber=c.batchNumber,tick=c.tick;
  const rejected=c.start(startOptions);assert.equal(rejected.ok,false);assert(rejected.reason.includes('复位或重新随机摆放'));
  assert.equal(c.state,'stopped');assert.equal(c.batchNumber,batchNumber);assert.equal(c.tick,tick);
  assert.deepEqual([...stations.values()].map(s=>({...s.controller.classification})),before);
  assert([...stations.values()].every(s=>s.calls.start.length===1&&s.calls.reset===0));
  c.reset();for(const s of stations.values()){assert.equal(s.controller.classification.started,false);assert.equal(s.controller.classification.completed,0);}
  assert(c.start(startOptions).ok);c.stop();assert(c.configureTask('pipes',{seed:99}).ok);
  for(const s of stations.values()){assert.equal(s.controller.classification.started,false);assert.equal(s.controller.classification.completed,0);assert.equal(s.seed,99);}
  assert(c.start(startOptions).ok);
});
await check('Pipe batch-restart preflight stops before any station dispatch when only a later selected station is consumed',()=>{
  const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes').ok);
  const classification=stations.get('g1').controller.classification;classification.started=true;classification.completed=5;
  const result=c.start(startOptions);assert.equal(result.ok,false);assert([...stations.values()].every(s=>!s.calls.start.length));
  assert.equal(classification.started,true);assert.equal(classification.completed,5);assert.equal(c.state,'idle');
});
await check('A failed or stopped station cannot be aggregated as all-robot pipe-classification success',()=>{
  for(const failure of ['failed','stopped']){
    const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes').ok);assert(c.start(startOptions).ok);
    for(const s of stations.values()){s.controller.running=false;s.controller.state='complete';}stations.get('g1').controller.state=failure;c.step();
    assert.equal(c.state,'failed');assert.equal(c.running,false);assert(!c.message.includes('均已完成'));
  }
  const {c,stations}=coordinatorFixture();assert(c.configureTask('pipes').ok);assert(c.select(['c5']).ok);assert(c.start(startOptions).ok);
  stations.get('c5').controller.running=false;stations.get('c5').controller.state='complete';c.step();assert.equal(c.state,'complete');assert(c.message.includes('1 台机器人均已完成圆管分类'));
});

// The legacy controller below only records queue dispatch. It neither supplies
// contacts nor calls step; the separate real-physics suite owns grasp outcomes.
class QueueOnlyLegacy {
  constructor(){this.targets=Array.from({length:8},(_,index)=>({id:`fixture-pipe-${index}`}));this.history=[];this.calls=[];this.running=false;}
  reset(){this.running=false;this.state='idle';this.history=[];}
  start(arm,id){this.calls.push({arm,id});this.arm=arm;this.selected=id;this.running=true;this.history=['approach'];return true;}
}
for(const robotId of ids)await check(`${robotId}: the production queue dispatches eight automatic arms without a manual arm choice`,()=>{
  const Controller=pipeControllerClass(QueueOnlyLegacy,{robotId}),controller=new Controller();
  const queue=controller.targets.map(target=>target.id);controller.setLayout({seed:521,queue,bins:[]});
  assert.equal(controller.startClassification(),true);
  for(let index=1;index<8;index++){controller.running=false;controller.classification.completed=index;assert.equal(controller.startPipeItem(),true);}
  const expected=robotId==='c5'?Array(8).fill('left'):Array.from({length:8},(_,index)=>index%2?'left':'right');
  assert.deepEqual(controller.calls.map(call=>call.id),queue);assert.deepEqual(controller.calls.map(call=>call.arm),expected);
  assert.deepEqual(controller.classification.armSequence,expected);assert.equal(controller.classification.total,8);
  return {dispatchOnly:true,armSequence:expected};
});
for(const robotId of ids)await check(`${robotId}: production batch-start guard preserves stopped queue evidence until an explicit reset`,()=>{
  const Controller=pipeControllerClass(QueueOnlyLegacy,{robotId}),controller=new Controller(),queue=controller.targets.map(target=>target.id);
  controller.setLayout({seed:521,queue,bins:[]});assert(controller.startClassification());controller.running=false;controller.state='stopped';
  controller.classification.completed=3;controller.classification.items.push({id:queue[0],evidence:'fixture-counter-only'});
  const record=controller.classification,serialized=JSON.stringify(record),dispatches=controller.calls.length;
  assert.equal(controller.startClassification(),false);assert.equal(controller.classification,record);
  assert.equal(JSON.stringify(controller.classification),serialized);assert.equal(controller.calls.length,dispatches);
  controller.reset();assert.equal(controller.classification.started,false);assert.equal(controller.classification.completed,0);
  assert.equal(controller.classification.items.length,0);assert.equal(controller.startClassification(),true);
  assert.equal(controller.calls.length,dispatches+1);assert.equal(controller.classification.started,true);
});

const prep={r1:['spread','crouch'],g1:['spread','raise','ready','recenter','align'],c5:[]};
const handling=['approach','descend','closing','lift','transfer','place','release','retreat','settling'];
const returns={r1:['return'],g1:['return_clear','return_align','return_fold','return_wide','return','return_center'],c5:['return']};
for(const id of ids)await check(`${id}: all eight pipe milestones progress monotonically through final stability verification`,()=>{
  const entries=[];
  for(let completed=0;completed<8;completed++)for(const state of [...prep[id],...handling,...returns[id]])entries.push({id,state,task:'pipes',classification:{total:8,completed}});
  entries.push({id,state:'pipe_settle',task:'pipes',classification:{total:8,completed:8}},{id,state:'complete',task:'pipes',classification:{total:8,completed:8}});
  const trace=entries.map(entry=>({state:entry.state,completed:entry.classification.completed,value:taskProgress(entry)}));let previous=0;
  for(const point of trace){assert(Number.isFinite(point.value)&&point.value>=0&&point.value<=1);assert(point.value>=previous-1e-12,`${id}/${point.completed}/${point.state}: ${point.value}<${previous}`);previous=point.value;}
  assert.equal(trace.at(-2).value,.98);assert.equal(trace.at(-1).value,1);progressTraces[id]=trace;return {milestones:trace.length};
});
await check('The final pipe-settle milestone has a Chinese label and never reports premature 100 percent',()=>{
  assert.equal(STAGE_NAMES.pipe_settle,'全部圆管稳定验证');
  for(const id of ids)for(let completed=0;completed<=8;completed++)for(const state of ['idle','approach','failed','stopped','pipe_settle']){
    const value=taskProgress({id,state,task:'pipes',classification:{total:8,completed}});assert(value>=0&&value<1,`${id}/${completed}/${state}`);
  }
  for(const id of ids)for(let completed=0;completed<8;completed++)assert.equal(taskProgress({id,state:'idle',task:'pipes',classification:{total:8,completed}}),completed/8);
});

const finalHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(finalHashes);
if(!sourceFrozen)errors.push({name:'source freeze',message:'A tested source changed during the UI/contract suite'});
const report={pass:errors.length===0,scope:'Static HTML/application wiring, extracted source-function fixtures, pure task presentation and orchestration contracts. No browser rendering, real pointer/touch, camera-pixel or physical-contact claims.',
  limitations:['UI source checks and narrow source-function fixtures do not replace a real DOM/accessibility/native-input or rendered desktop/mobile inspection.',
    'Coordinator station fixtures verify calls, clocks, cancellation and aggregate state only; their counters do not prove physical sorting or arm trajectories.',
    'Cylinder layout/collision/reachability, alternating real arm execution, stable bin support and non-teleporting transport require the separate production MuJoCo acceptance suite.'],
  total:checks.length,passed:checks.filter(item=>item.pass).length,checks,contractTraces,progressTraces,sourceFrozen,sourceHashes,errors};
const destination=new URL(reportPath,root);await mkdir(new URL('./',destination),{recursive:true});
await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,total:report.total,passed:report.passed,sourceFrozen,report:reportPath,errors},null,2));
if(!report.pass)process.exitCode=1;
