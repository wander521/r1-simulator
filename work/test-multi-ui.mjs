import assert from 'node:assert/strict';
import {readFile, mkdir, writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {STATION_OFFSETS, createSharedWorkcell, addStationTable} from '../dist/multi/workcell.mjs';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';
import {STAGE_NAMES, taskProgress} from '../dist/multi/presentation.mjs';

// This suite deliberately does not emulate a browser, DOM, GPU or contacts.
// It verifies production geometry/helpers and the source-level UI contracts;
// native rendering, pointer gestures and physical grasping have separate QA.
const root=new URL('../',import.meta.url),ids=['r1','g1','c5'];
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)||'work/validation/multi/ui-static-final.json';
const sourcePaths=[
  'work/test-multi-ui.mjs','dist/multi/index.html','dist/multi/styles.css','dist/multi/app.mjs',
  'dist/multi/presentation.mjs','dist/multi/workcell.mjs','dist/multi/robot-runtime.mjs','dist/multi/coordinator.mjs','dist/multi/planning.mjs','dist/multi/pipe-controller.mjs',
  ...['background-planning','planning-worker','planning-worker-core','planning-worker-transport','planning-serialization','planning-robot','planning-check-cache','planning-worker-protocol','planning-ik-context','planning-budget'].map(name=>`dist/multi/${name}.mjs`),
  'dist/multi/visuals/theme-controller.mjs','dist/multi/visuals/scene-environment.mjs','dist/multi/visuals/camera-feeds.mjs','dist/multi/visuals/camera-feed-queue.mjs','dist/multi/visuals/frame-scheduler.mjs',
  'dist/r1/grasp-controller.mjs','dist/g1/grasp-controller.mjs','dist/c5/grasp-controller.mjs',
  'dist/r1/r1-workcell.mjs','dist/g1/g1-workcell.mjs','dist/r1/r1-area-light-textures.mjs',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/vendor/three.module.min.js',
];
async function hashes(){return Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,
  createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));}
const sourceHashes=await hashes();
const text=async path=>readFile(new URL(path,root),'utf8');
const [html,app,css,runtimeSource,coordinatorSource]=await Promise.all([
  text('dist/multi/index.html'),text('dist/multi/app.mjs'),text('dist/multi/styles.css'),
  text('dist/multi/robot-runtime.mjs'),text('dist/multi/coordinator.mjs'),
]);
const feedSource=await text('dist/multi/visuals/camera-feeds.mjs');
const controllers=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await text(`dist/${id}/grasp-controller.mjs`)])));
const checks=[],errors=[],stateCoverage={},progressTraces={},geometry={};
function check(name,fn){try{const detail=fn();checks.push({name,pass:true,...(detail===undefined?{}:{detail})});}
  catch(error){checks.push({name,pass:false,error:error.message});errors.push({name,message:error.message});}}
function near(actual,expected,message,tolerance=1e-6){assert(Math.abs(actual-expected)<=tolerance,`${message}: ${actual} != ${expected}`);}

// Read the argument of literal transition calls without accidentally consuming
// the rest of a minified line, including later comparisons or error messages.
function callArguments(source,callee){
  const result=[];let cursor=0;
  while((cursor=source.indexOf(`${callee}(`,cursor))>=0){let depth=1,quote=null,escaped=false,end=cursor+callee.length+1;
    const start=end;
    for(;end<source.length&&depth;end++){
      const c=source[end];
      if(quote){if(escaped)escaped=false;else if(c==='\\')escaped=true;else if(c===quote)quote=null;continue;}
      if(c==='\''||c==='"'||c==='`'){quote=c;continue;}
      if(c==='(')depth++;else if(c===')')depth--;
    }
    assert.equal(depth,0,`Unbalanced ${callee} argument`);result.push(source.slice(start,end-1));cursor=end;
  }
  return result;
}
function actualStates(source){
  const states=new Set(['idle','happy','shy','complete','failed','stopped']);
  for(const match of source.matchAll(/\bthis\.state\s*(?:={1,3}|!==?)\s*['"]([a-z][a-z0-9_]*)['"]/g))states.add(match[1]);
  for(const match of source.matchAll(/\bstate\s*:\s*['"]([a-z][a-z0-9_]*)['"]/g))states.add(match[1]);
  for(const expression of callArguments(source,'this.transition'))for(const match of expression.matchAll(/['"]([a-z][a-z0-9_]*)['"]/g))states.add(match[1]);
  return [...states].sort();
}

for(const id of ids)check(`${id}: every actual controller state has a Chinese UI label`,()=>{
  const states=actualStates(controllers[id]);assert(states.length>=14);
  for(const state of states){assert(Object.hasOwn(STAGE_NAMES,state),`${id}: missing label for ${state}`);
    assert(/\p{Script=Han}/u.test(STAGE_NAMES[state]),`${id}: non-Chinese stage ${state}`);}
  stateCoverage[id]=states;return {states:states.length};
});
check('Unknown controller states use a Chinese fallback instead of exposing raw English',()=>{
  assert(/STAGE_NAMES\[snapshot\.state\]\s*\?\?\s*['"]执行中['"]/.test(app));
  assert(!/STAGE_NAMES\[snapshot\.state\]\s*\?\?\s*snapshot\.state/.test(app));
  assert.equal(taskProgress({id:'g1',state:'future_internal_state',task:'putaway'}),0);
});
check('UI consumes the presentation helper rather than legacy generic runtime progress',()=>{
  assert(/import\s*\{STAGE_NAMES,\s*taskProgress\}\s*from\s*['"]\.\/presentation\.mjs['"]/.test(app));
  assert(app.includes('progress=taskProgress({...snapshot,itemIndex:station.controller.itemIndex})'));
  assert(!app.includes('snapshot.progress'));
});

const handling=['approach','descend','closing','lift','transfer','place','release','retreat','settling'];
const prep={r1:['spread','crouch'],g1:['spread','raise','ready','align'],c5:[]};
const returning={r1:['return'],g1:['return_clear','return_align','return_fold','return_wide','return','return_center'],c5:['return']};
function assertTrace(id,task,entries,key){
  const trace=entries.map(([state,itemIndex=0])=>({state,itemIndex,value:taskProgress({id,state,task,itemIndex})}));
  let previous=-1;
  for(const item of trace){assert(Number.isFinite(item.value)&&item.value>=0&&item.value<=1,`${key}: invalid ${item.state}`);
    assert(item.value+1e-12>=previous,`${key}: ${item.state}/${item.itemIndex} regressed ${previous} -> ${item.value}`);previous=item.value;}
  assert.equal(trace.at(-1).value,1);progressTraces[key]=trace;return {milestones:trace.length};
}
for(const id of ids)check(`${id}: successful sort progress never regresses through actual mechanism phases`,()=>
  assertTrace(id,'putaway',[...prep[id],...handling,...returning[id],'complete'].map(state=>[state]),`${id}-sort`));
for(const id of ids)check(`${id}: stack item and reaction milestones remain ordered`,()=>{
  const entries=[];
  for(let item=0;item<3;item++){
    const begin=item===0?prep[id]:id==='g1'?['recenter','align']:[];
    entries.push(...[...begin,...handling].map(state=>[state,item]));
  }
  const reactions=id==='g1'?['stack_settle','reaction_clear','reaction_align','reaction_fold','reaction_wide','reaction_home','face_observer','happy','complete']
    :id==='r1'?['stack_settle','face_observer','happy','complete']:['stack_settle','reaction_home','happy','complete'];
  entries.push(...reactions.map(state=>[state,2]));return assertTrace(id,'stack',entries,`${id}-stack`);
});
check('Idle, aborted and failed tasks never falsely display completion',()=>{
  for(const id of ids)for(const task of ['putaway','stack'])for(const state of ['idle','stopped','failed','shy'])
    assert.equal(taskProgress({id,state,task,itemIndex:2}),0);
});

check('Three station origins lie in the requested compact row with 1.6 m gaps',()=>{
  assert.deepEqual(Object.keys(STATION_OFFSETS),ids);
  assert.deepEqual(STATION_OFFSETS,{r1:[0,0,1.6],g1:[0,0,0],c5:[0,0,-1.6]});
  for(let index=0;index<ids.length-1;index++)near(new THREE.Vector3(...STATION_OFFSETS[ids[index]]).distanceTo(new THREE.Vector3(...STATION_OFFSETS[ids[index+1]])),1.6,'Adjacent station spacing');
});

const scene=new THREE.Scene(),shared=createSharedWorkcell(scene),stationGroups=[];
const lights=()=>{const result=[];scene.traverse(node=>{if(node.isLight)result.push(node);});return result;};
const initialLights=lights();
check('One shared environment owns one floor and one shadow-casting key light',()=>{
  assert.equal(scene.children.filter(child=>child===shared.group).length,1);
  assert(shared.floor?.isMesh,'The environment must expose its actual shared floor, independent of visual theme/name');
  const floors=[];scene.traverse(node=>{if(node===shared.floor)floors.push(node);});
  assert.equal(floors.length,1);assert.equal(shared.table.parent,null);assert.equal(initialLights.length,shared.lights.length);
  assert.equal(initialLights.filter(light=>light.castShadow).length,1);assert(initialLights.length>=5);
  assert.equal((app.match(/createSharedWorkcell\(scene(?:,|\))/g)||[]).length,1);
});
for(const id of ids){
  const station={id,config:ROBOT_CONFIGS[id],group:new THREE.Group()};station.group.position.fromArray(STATION_OFFSETS[id]);scene.add(station.group);stationGroups.push(station.group);
  const table=addStationTable(station);scene.updateMatrixWorld(true);
  check(`${id}: all four table legs meet the floor and the top matches its physical height`,()=>{
    const c=station.config,bounds=new THREE.Box3().setFromObject(table),top=table.getObjectByName(`${id}-tabletop`),topBounds=new THREE.Box3().setFromObject(top);
    near(bounds.min.y,0,`${id} table bottom`);near(bounds.max.y,c.tableHeight,`${id} table top`);
    near(bounds.min.x,c.tableCenter-c.tableHalfX,`${id} table rear edge`);near(bounds.max.x,c.tableCenter+c.tableHalfX,`${id} table front edge`);
    near(topBounds.max.y,c.tableHeight,`${id} tabletop physical height`);
    const legs=table.children.filter(child=>child!==top);assert.equal(legs.length,4);
    for(const leg of legs){near(new THREE.Box3().setFromObject(leg).min.y,0,`${id} grounded leg`);assert(leg.castShadow&&leg.receiveShadow);}
    geometry[id]={table:{min:bounds.min.toArray(),max:bounds.max.toArray()},tableHeight:c.tableHeight,offset:station.group.position.toArray()};
  });
  if(id==='c5')check('C5 plinth is grounded, supports the source root height and stays clear of the table',()=>{
    const c=station.config,plinth=station.group.getObjectByName('c5-grounded-mount'),box=new THREE.Box3().setFromObject(plinth),tableBox=new THREE.Box3().setFromObject(table);
    assert(plinth);near(box.min.y,0,'C5 plinth bottom');near(box.max.y,c.rootHeight,'C5 mount height');
    near(c.plinthHeight,c.rootHeight,'C5 configured support height');near(plinth.geometry.parameters.radiusTop,c.plinthRadius,'C5 plinth radius');
    assert(tableBox.min.x-box.max.x>.09,'C5 table must not overlap its grounded mount');geometry.c5.plinth={min:box.min.toArray(),max:box.max.toArray(),clearance:tableBox.min.x-box.max.x};
  });
}
check('Station tables introduce no duplicate global lights or floors',()=>{
  assert.deepEqual(lights().map(light=>light.uuid),initialLights.map(light=>light.uuid));
  for(const group of stationGroups)group.traverse(node=>assert(!node.isLight));
  const floors=[];scene.traverse(node=>{if(node===shared.floor)floors.push(node);});assert.equal(floors.length,1);
});
check('Adjacent grounded tables remain geometrically separate',()=>{
  for(let index=0;index<ids.length-1;index++){
    const a=geometry[ids[index]].table,b=geometry[ids[index+1]].table;
    const gap=a.min[2]-b.max[2],expected=1.6-ROBOT_CONFIGS[ids[index]].tableHalfZ-ROBOT_CONFIGS[ids[index+1]].tableHalfZ;
    near(gap,expected,'Compact station table clearance');
    assert(gap>.8,`${ids[index]} and ${ids[index+1]} tables overlap or lose the requested clearance`);
  }
});

check('Native checkboxes, labelled focus/arm selects and status/progress semantics exist',()=>{
  assert(/<label><input\s+type="checkbox"\s+data-robot=/.test(app));
  for(const id of ['focus-robot','arm-select','task-mode']){assert(new RegExp(`<label[^>]*for="${id}"`).test(html));assert(new RegExp(`<select[^>]*id="${id}"`).test(html));}
  assert(html.includes('<fieldset class="robot-picker">'));assert(html.includes('<legend>'));
  assert(html.includes('aria-live="polite"'));assert(app.includes('<progress max="1" value="0" aria-label='));
});
check('Busy/loading states disable selection/start/editing, while focus and stop remain independent',()=>{
  assert(app.includes('input.disabled=busy||!stations.has(input.dataset.robot)'));
  assert(app.includes("$('#start').disabled=busy||!selected.length"));
  const busyExpression=app.match(/\bconst busy=([^,;]+)/)?.[1];assert(busyExpression,'Missing busy-state expression');
  const assignment=id=>{const value=app.match(new RegExp(`\\$\\('#${id}'\\)\\.disabled=([^;]+)`))?.[1];assert(value,`Missing ${id} disabled expression`);return value;};
  const enabled=(expression,{running=false,planning=false,preparing=false,size=3}={})=>
    !new Function('coordinator','preparing','stations',`return (${expression});`)({running,planning},preparing,{size});
  for(const state of [{running:true},{planning:true},{preparing:true}])assert.equal(enabled(busyExpression,state),false,'Preparing/running must lock task edits');
  assert.equal(enabled(assignment('stop')),false);assert.equal(enabled(assignment('stop'),{running:true}),true);
  assert.equal(enabled(assignment('stop'),{planning:true}),true,'Stop must remain available during candidate planning');
  assert.equal(enabled(assignment('reset'),{planning:true}),true,'Reset must be able to cancel candidate planning');
  assert.equal(enabled(assignment('reset'),{size:0}),false);
  assert(app.includes("$('#arm-select').disabled=busy"));
  assert([...app.matchAll(/if\(!coordinator\|\|([^\n;]+)\)return;/g)].some(match=>match[1].includes('coordinator.running')&&match[1].includes('preparing')));
  assert(app.includes("$('#focus-robot').disabled=!focus"));
});
check('Actual selection/focus coordinator contract rejects empty selection and locks active runs',()=>{
  const stations=ids.map(id=>({id,ready:true,config:ROBOT_CONFIGS[id],group:{visible:true},controller:{running:false},
    get snapshot(){return {id,time:0,state:this.controller.running?'approach':'idle'};},
    start(){this.controller.running=true;return true;},stop(){this.controller.running=false;},reset(){this.controller.running=false;},step(){}}));
  const c=new MultiRobotCoordinator({stations});
  assert(c.select(['g1','c5']).ok);const previous=[...c.selectedIds];assert.equal(c.select([]).ok,false);assert.deepEqual(c.selectedIds,previous);
  assert(c.setFocus('c5').ok);assert.deepEqual(c.selectedIds,previous);assert.equal(c.setFocus('r1').ok,false);
  assert(c.start().ok);assert.equal(c.select(['r1']).ok,false);assert.deepEqual(c.runIds,previous);
  c.stop();assert(!c.running);assert(c.select(['r1']).ok);c.reset();assert.equal(c.tick,0);assert.deepEqual(c.runIds,[]);
  assert.deepEqual(c.selectedIds,['r1']);
  assert(app.includes('coordinator.select(ids)'));assert(app.includes('coordinator?.setFocus(id).ok'));
});
check('Solid-object editing uses both robot and object IDs; blank gestures remain with orbit',()=>{
  assert(app.includes('intersectObjects(coordinator.selectedIds.flatMap(id=>stations.get(id).solidMeshes),false)'));
  assert(app.includes('if(!hit)return'));assert(app.includes('hit.object.userData.robotId'));assert(app.includes('hit.object.userData.taskId'));
  assert(app.includes('station.worldToLocal(point.add(drag.offset))'));
  assert(runtimeSource.includes('Object.assign(visualMesh.userData,{taskId:spec.id,robotId:config.id})'));
  assert(app.includes('orbit.enabled=false'));assert(app.includes('orbit.enabled=true'));
});
check('Pointer cleanup and keyboard controls have source-level interruption safeguards',()=>{
  for(const event of ['pointercancel','lostpointercapture','blur','visibilitychange'])assert(app.includes(`'${event}'`));
  for(const key of ['Escape','KeyW','KeyA','KeyS','KeyD'])assert(app.includes(key));
  assert(app.includes('canvas.setPointerCapture(event.pointerId)'));assert(app.includes('canvas.releasePointerCapture(previous.pointerId)'));
  assert(css.includes('touch-action:none'));assert(css.includes(':focus-visible'));assert(css.includes('prefers-reduced-motion:reduce'));
});
check('Two camera feeds and actual measured force/torque charts remain connected',()=>{
  for(const id of ['overhead-feed','wrist-feed','force-chart','torque-chart'])assert(html.includes(`id="${id}"`));
  assert(app.includes('cameraFeedPass=createCameraFeedPass(renderer)'));
  assert(app.includes('cameraFeedQueue=createCameraFeedQueue('));
  assert(app.includes('cameraFeedPass.renderAsync(scene,camera,feedPixels)'));
  assert(app.includes('cameraFeedQueue.request({station:focus,controller:focus.controller,arm:focus.arm,theme:sceneThemes.activeId,index,camera:[vision.overhead,vision.camera][index]})'));
  assert(app.includes('!showCameras||cameraFeedQueue.pending'));
  assert(app.includes('focused()===station&&station.controller===controller&&station.arm===arm&&sceneThemes.activeId===theme'));
  assert(app.includes('cameraFeedQueue.invalidate()'));
  assert(app.includes('cameraFeedVisible(feed.element.getBoundingClientRect()')&&app.includes('nextFeed=(index+1)%2'));
  assert(feedSource.includes('renderer.readRenderTargetPixels(displayTarget, 0, 0, width, height, pixels)'));
  assert(feedSource.includes('renderer.readRenderTargetPixelsAsync(displayTarget, 0, 0, width, height, pixels)'));
  assert(app.includes('feed.image.data.set(pixels.subarray((179-row)*240*4,(180-row)*240*4),row*240*4)'));
  assert(app.includes('feed.ctx.putImageData(feed.image,0,0)'));
  assert(app.includes('画面暂时不可用'));
  assert(app.includes('history.force.push(m.forces)'));assert(app.includes('telemetry.joints.map(joint=>joint.torque)'));
  assert(/\b(?:vision|v)\.wrist\.update\(station\.cameraLink\)/.test(app));assert(/\b(?:vision|v)\.wrist\.setVisible\(station\.group\.visible\)/.test(app));
  assert(app.includes('({scene,THREE,camera:wristCamera})'));assert(runtimeSource.includes('get cameraLink(){return visualRobot.getObjectByName('));
});
check('One RAF advances all worlds through the coordinator and pauses hidden/preparing backlog',()=>{
  assert.equal((app.match(/function frame\(/g)||[]).length,1);
  assert(app.includes('createFrameScheduler'));
  const start=app.indexOf('function frame('),end=app.indexOf('\nasync function boot(',start),frame=app.slice(start,end);
  assert(frame.includes('document.hidden')&&frame.includes('preparing')&&frame.includes('planning'));
  assert(/\.advance\((?:wasPlanning\?performance\.now\(\):)?now,\s*\{/.test(frame)&&/coordinator\.step\(dt\)/.test(frame));
  assert(frame.includes('coordinator.advancePlanning()'),'Pending candidates must advance through the same animation loop');
  assert(coordinatorSource.includes('for (const station of this.stations.values()) if (ready(station)) station.step(dt)'));
  const visibility=app.match(/document\.addEventListener\('visibilitychange',([^\n]+)/)?.[1];assert(visibility?.includes('endDrag()')&&visibility.includes('.reset('));
});
check('Both browser runtime creation paths use isolated background planning with concise pending UI',()=>{
  const creations=callArguments(app,'createRobotRuntime');assert.equal(creations.length,2);
  for(const argumentsText of creations)assert(/backgroundPlanning\s*:\s*true/.test(argumentsText),'Initial load and Retry must both enable the worker');
  assert(runtimeSource.includes('backgroundPlanningControllerClass'));
  assert(app.includes("station.planning?'后台规划'")&&app.includes('后台检查路径'));
  assert(app.includes('正在后台规划，镜头、停止和复位操作仍可用。'));
  assert(!app.includes('planningDescription')&&!app.includes('个规划片段')&&!app.includes('已等待'));
  const update=app.slice(app.indexOf('function updateUI()'),app.indexOf("$('#task-mode').addEventListener"));
  assert(!/第\s*\$\{|planningProgress|elapsedMs|segmentElapsedMs/.test(update),'The visible planning UI must not expose item index, candidate counts, or wait time');
  assert(app.includes('已归类 ${classification.completed} / ${classification.total}'),'Keep the existing actual completion count');
});
check('Public diagnostics expose snapshots only, not test actuators or physics mutation commands',()=>{
  const start=app.indexOf('window.multiDiagnostics=()=>('),end=app.indexOf('\n  }catch(error)',start);
  assert(start>=0&&end>start);const diagnostic=app.slice(start,end);
  assert(diagnostic.includes('...coordinator.snapshot'));assert(diagnostic.includes('canonicalParent:'));assert(diagnostic.includes('backlogSeconds:'));
  assert(!/\.(?:start|startStack|stop|reset|step|setObject|moveObject|setJointValues|setTargets|dispose)\s*\(/.test(diagnostic));
  assert(!/\b(?:qpos|qvel|ctrl|mocap_pos|mocap_quat)\b/.test(diagnostic));
  assert.deepEqual([...app.matchAll(/window\.(\w+)\s*=/g)].map(match=>match[1]),['multiDiagnostics']);
});

const finalHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(finalHashes);
if(!sourceFrozen)errors.push({name:'source freeze',message:'A tested source changed during the static suite'});
const report={pass:errors.length===0,scope:'Node-only static UI wiring, real Three geometry and pure presentation/coordinator contracts. No browser rendering, native input, synthesized touch, accessibility-tree or physical grasp claims.',
  limitations:['Rendered desktop/mobile appearance and camera pixels are not verified by this suite.','Pointer/touch gestures require separate browser or physical-device verification.','Real contacts and grasp outcomes are verified by the separate physical suite, not these coordinator contract fixtures.'],
  total:checks.length,passed:checks.filter(item=>item.pass).length,checks,stateCoverage,progressTraces,geometry,
  sharedLights:{count:initialLights.length,shadowCasting:initialLights.filter(light=>light.castShadow).length},sourceFrozen,sourceHashes,errors};
const destination=new URL(reportPath,root);await mkdir(new URL('./',destination),{recursive:true});
await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,total:report.total,passed:report.passed,sourceFrozen,report:reportPath,errors},null,2));
shared.dispose();
if(!report.pass)process.exitCode=1;
