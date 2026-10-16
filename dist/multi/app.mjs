import * as THREE from '../vendor/three.module.min.js';
import {OrbitControls} from '../vendor/OrbitControls.js';
import {createWristVision as r1WristVision} from '../r1/r1-workcell.mjs';
import {createWristVision as dexWristVision} from '../g1/g1-workcell.mjs';
import {ROBOT_CONFIGS, loadPhysicsAPI, createRobotRuntime} from './robot-runtime.mjs';
import {MultiRobotCoordinator} from './coordinator.mjs';
import {STATION_OFFSETS, STATION_VIEW_BOUNDS, fitWorkstationView, createSharedWorkcell, addStationTable, configureRenderer} from './workcell.mjs';
import {STAGE_NAMES, taskProgress} from './presentation.mjs';
import {createThemeController} from './visuals/theme-controller.mjs';
import {createCameraFeedPass} from './visuals/camera-feeds.mjs';
import {createFrameScheduler,cameraFeedVisible,shouldRefitCamera} from './visuals/frame-scheduler.mjs';

const $ = selector => document.querySelector(selector);
const canvas = $('#main-view'), viewport = $('#viewport');
const scene = new THREE.Scene(), camera = new THREE.PerspectiveCamera(42,1,.02,40);
const renderer = new THREE.WebGLRenderer({canvas,antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));
renderer.shadowMap.enabled=true; renderer.shadowMap.type=THREE.PCFSoftShadowMap;
renderer.shadowMap.autoUpdate=false;renderer.shadowMap.needsUpdate=true;
configureRenderer(renderer,{THREE}); camera.layers.enable(1);
const workcell = createSharedWorkcell(scene,{renderer,initialScene:'night'});
const sceneThemes = createThemeController({workcell,root:document.documentElement,buttons:document.querySelectorAll('[data-scene-switch]')});
const orbit = new OrbitControls(camera,canvas);
orbit.enableDamping=true; orbit.dampingFactor=.12;
orbit.minDistance=.6; orbit.maxDistance=22; orbit.maxPolarAngle=Math.PI*.485;
camera.position.set(7,3,1); orbit.target.set(.4,.8,0); orbit.update();

const stations=new Map(), visions=new Map(), histories=new Map(), loadStates=new Map();
const objectNames={red:'红色方块',blue:'蓝色长方块',yellow:'黄色圆柱'};
const reducedMotion=matchMedia('(prefers-reduced-motion: reduce)').matches;
let coordinator, selectedObject='blue', preparing=false, drag=null, notice='', showCameras=true, currentTask='sort';
let lastFrame=0, accumulator=0, lastUI=0, lastFeed=0, lastStatus='', physicsSteps=0;
const frameTimes=[], renderTimes=[];
const frameScheduler=createFrameScheduler();
let nextFeed=0,feedRenderCount=0,planningSlices=0;
let lastViewFit=null;
const raycaster=new THREE.Raycaster(), pointer=new THREE.Vector2();
const cameraFeedPass=createCameraFeedPass(renderer), feedPixels=new Uint8Array(240*180*4);
const feeds=[$('#overhead-feed'),$('#wrist-feed')].map(element=>({element,ctx:element.getContext('2d'),image:new ImageData(240,180)}));
const contactGroup=new THREE.Group();scene.add(contactGroup);
const contactGeometry=new THREE.SphereGeometry(.004,8,6), contactMaterial=new THREE.MeshBasicMaterial({color:0x5ee8c9});
const contactMarkers=Array.from({length:24},()=>{const marker=new THREE.Mesh(contactGeometry,contactMaterial);marker.visible=false;contactGroup.add(marker);return marker;});

for (const config of Object.values(ROBOT_CONFIGS)) {
  loadStates.set(config.id,{progress:0,message:'准备加载'});
  const row=document.createElement('div'); row.className='robot-option'; row.dataset.robot=config.id;
  row.innerHTML=`<label><input type="checkbox" data-robot="${config.id}" checked disabled><span class="robot-code" aria-hidden="true">${config.id.toUpperCase()}</span><span><strong>${config.label}</strong><small>${config.id==='c5'?'六轴机械臂 · DEX1':config.id==='g1'?'固定站位 · DEX1':'双臂移动操作机器人'}</small></span></label><span class="load-state" id="load-${config.id}">准备加载</span>`;
  $('#robot-options').append(row);
  const result=document.createElement('div'); result.className='result-row'; result.dataset.robot=config.id;
  result.innerHTML=`<div class="result-head"><button type="button" data-focus="${config.id}" disabled>${config.label}</button><output>加载中</output></div><div class="result-track"><progress max="1" value="0" aria-label="${config.label}抓取进度"></progress><span>0%</span></div><p class="result-note"></p>`;
  $('#results').append(result);
  const label=document.createElement('button');label.type='button';label.className='station-tag';label.dataset.focus=config.id;label.textContent=config.label;label.hidden=true;label.setAttribute('aria-pressed','false');$('#station-labels').append(label);
}

function status(message){notice=message; updateUI();}
function focused(){return stations.get(coordinator?.focusId);}
function pipeTask(){return $('#task-mode').value==='pipes';}
function clearHistories(){for(const history of histories.values())history.force.length=history.torque.length=0;}
async function configureTask(task,{randomize=false}={}){
  if(!coordinator||coordinator.running||preparing)return;
  endDrag();preparing=true;notice=task==='pipes'?'正在准备随机圆管与分类工作区…':'正在恢复原工作台…';updateUI();
  await new Promise(requestAnimationFrame);
  const result=coordinator.configureTask(task,task==='pipes'?{seed:crypto.getRandomValues(new Uint32Array(1))[0]}:{});
  if(result.ok){currentTask=task;selectedObject=task==='pipes'?focused().selected:'blue';clearHistories();notice='';renderer.shadowMap.needsUpdate=true;}
  else{$('#task-mode').value=currentTask;notice=result.reason;}
  preparing=false;lastFrame=performance.now();accumulator=0;frameScheduler.reset(lastFrame);lastFeed=0;updateUI();
}
function fitView(ids=coordinator?.selectedIds??[...stations.keys()],{motion=preparing||Boolean(coordinator?.running)||coordinator?.state==='preparing',reason='panorama'}={}) {
  if(!shouldRefitCamera(reason))return;
  const bounds=new THREE.Box3();
  for(const id of ids) {
    const station=stations.get(id);if(!station)continue;
    station.group.updateMatrixWorld(true);
    bounds.union(new THREE.Box3().setFromObject(station.group));
    if(motion){
      const envelope=STATION_VIEW_BOUNDS[id];
      bounds.union(new THREE.Box3(new THREE.Vector3(...envelope.min),new THREE.Vector3(...envelope.max)).applyMatrix4(station.group.matrixWorld));
    }
    bounds.expandByPoint(station.group.localToWorld(new THREE.Vector3(0,station.labelHeight,0)));
    const wrist=visions.get(id)?.wrist;
    if(wrist){wrist.update(station.cameraLink);wrist.model.updateMatrixWorld(true);bounds.union(new THREE.Box3().setFromObject(wrist.model));}
  }
  if(bounds.isEmpty())return;
  // At rest, fit the visible robots and grounded worktables closely. Before
  // movement, reserve the full preparation envelope instead of clipping arms
  // when they unfold. Neither view scales the models or changes local physics.
  const view=fitWorkstationView(bounds,{fov:camera.fov,aspect:camera.aspect,viewportHeight:viewport.clientHeight,near:camera.near});
  orbit.target.copy(view.target);camera.position.copy(view.position);orbit.update();
  lastViewFit={mode:motion?'motion':'idle',ids:[...ids],distance:view.distance,bounds:{min:bounds.min.toArray(),max:bounds.max.toArray()}};
}
function setFocus(id) {
  if(!coordinator?.setFocus(id).ok)return;
  $('#focus-robot').value=id; updateUI(); lastFeed=0;
}
function selectObject(id) {
  if(!coordinator||coordinator.running||preparing)return;
  selectedObject=id;
  if(pipeTask()){
    const station=focused(),target=station?.targets.find(item=>item.id===id);
    if(!target||station.controller.classification?.started)return;
    station.select(id);notice=`已选择${target.label}；开始执行时仍会自动分类全部 8 个圆柱。`;
    updateUI();return;
  }
  for(const robotId of coordinator.selectedIds)stations.get(robotId).select(id);
  notice=`已选择${objectNames[id]}；拖动物体仅调整当前工作台，开始时各机器人抓取各自的${objectNames[id]}`;
  updateUI();
}

$('#robot-options').addEventListener('change',event=>{
  if(!event.target.matches('input[data-robot]')||!coordinator)return;
  endDrag();
  const ids=[...document.querySelectorAll('input[data-robot]:checked')].map(input=>input.dataset.robot);
  const result=coordinator.select(ids); notice=result.ok?'':result.reason;
  updateUI(); if(result.ok)fitView(undefined,{reason:'participants'});
});
$('#select-all').addEventListener('click',()=>{const result=coordinator.select([...stations.keys()]);notice=result.ok?'':result.reason;updateUI();if(result.ok)fitView(undefined,{reason:'participants'});});
document.querySelectorAll('[data-object]').forEach(button=>button.addEventListener('click',()=>selectObject(button.dataset.object)));
document.querySelectorAll('[data-focus]').forEach(button=>button.addEventListener('click',()=>setFocus(button.dataset.focus)));
$('#focus-robot').addEventListener('change',()=>setFocus($('#focus-robot').value));
$('#arm-select').addEventListener('change',()=>{const runtime=focused();if(runtime&&!coordinator.running){runtime.setArm($('#arm-select').value);status(`已选择${runtime.config.label}的${runtime.arm==='right'?'右臂':'左臂'}`);}});
$('#fit-view').addEventListener('click',()=>fitView());
$('#focus-view').addEventListener('click',()=>fitView([coordinator?.focusId],{reason:'focus'}));
$('#toggle-cameras').addEventListener('click',()=>{showCameras=!showCameras;$('#camera-feeds').hidden=!showCameras;$('#toggle-cameras').textContent=showCameras?'隐藏画面':'显示画面';$('#toggle-cameras').setAttribute('aria-pressed',String(showCameras));});
$('#randomize-pipes').addEventListener('click',()=>configureTask('pipes',{randomize:true}));
$('#start').addEventListener('click',()=>{
  if(!coordinator||coordinator.running||preparing)return;
  endDrag();preparing=true;notice='正在共同检查抓取路径；检查通过后同时开始…';updateUI();
  const armByRobot=Object.fromEntries([...stations].map(([id,station])=>[id,station.arm]));
  const result=coordinator.start({objectId:selectedObject,task:$('#task-mode').value,armByRobot,reducedMotion});
  notice=result.ok?'':result.reason;preparing=false;lastFrame=performance.now();accumulator=0;frameScheduler.reset(lastFrame);updateUI();
});
$('#stop').addEventListener('click',()=>{endDrag();coordinator?.stop();notice='';updateUI();});
$('#reset').addEventListener('click',()=>{
  endDrag();coordinator?.reset();selectedObject=pipeTask()?focused().selected:'blue';
  if(!pipeTask())for(const station of stations.values())station.select('blue');
  clearHistories();
  accumulator=0;lastFrame=performance.now();frameScheduler.reset(lastFrame);notice='';renderer.shadowMap.needsUpdate=true;updateUI();
});

function ray(event) {const rect=canvas.getBoundingClientRect();pointer.set((event.clientX-rect.left)/rect.width*2-1,1-(event.clientY-rect.top)/rect.height*2);raycaster.setFromCamera(pointer,camera);}
function down(event) {
  if(!coordinator||coordinator.running||preparing)return;
  if(event.isPrimary===false){endDrag();return;}
  if(drag||![0,2].includes(event.button))return;
  ray(event);
  const hit=raycaster.intersectObjects(coordinator.selectedIds.flatMap(id=>stations.get(id).solidMeshes),false)[0];
  if(!hit)return; // Empty-space gestures belong exclusively to OrbitControls.
  const station=stations.get(hit.object.userData.robotId), target=station.targets.find(item=>item.id===hit.object.userData.taskId);
  if(pipeTask()&&station.controller.classification?.started)return;
  setFocus(station.id);selectObject(target.id);
  const world=station.localToWorld(target.mesh.position),plane=new THREE.Plane(new THREE.Vector3(0,1,0),-world.y);
  const point=raycaster.ray.intersectPlane(plane,new THREE.Vector3());if(!point)return;
  drag={pointerId:event.pointerId,station,target,plane,offset:world.sub(point),x:event.clientX,y:event.clientY,rotation:target.mesh.quaternion.clone(),mode:event.button===2||event.shiftKey?'rotate':'move'};
  orbit.enabled=false;canvas.classList.add('dragging');canvas.setPointerCapture(event.pointerId);event.preventDefault();event.stopImmediatePropagation();
}
function move(event) {
  if(!drag||event.pointerId!==drag.pointerId)return;
  const {station,target}=drag;let moved=false;
  if(drag.mode==='rotate') {
    const yaw=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),(event.clientX-drag.x)*.012);
    const axis=new THREE.Vector3(1,0,0).applyQuaternion(camera.quaternion),pitch=new THREE.Quaternion().setFromAxisAngle(axis,(event.clientY-drag.y)*.012);
    moved=station.moveObject(target.id,target.mesh.position,yaw.multiply(pitch).multiply(drag.rotation));
  } else {ray(event);const point=raycaster.ray.intersectPlane(drag.plane,new THREE.Vector3());if(point)moved=station.moveObject(target.id,station.worldToLocal(point.add(drag.offset)));}
  if(!moved)notice=station.controller.message;event.preventDefault();
}
function endDrag(event) {
  if(!drag||(event&&event.pointerId!==drag.pointerId))return;
  const previous=drag;drag=null;if(canvas.hasPointerCapture(previous.pointerId))canvas.releasePointerCapture(previous.pointerId);
  canvas.classList.remove('dragging');orbit.enabled=true;
}
canvas.addEventListener('pointerdown',down,{capture:true});canvas.addEventListener('lostpointercapture',endDrag);
window.addEventListener('pointermove',move);window.addEventListener('pointerup',endDrag);window.addEventListener('pointercancel',endDrag);window.addEventListener('blur',()=>endDrag());
document.addEventListener('visibilitychange',()=>{endDrag();lastFrame=performance.now();accumulator=0;frameScheduler.reset(lastFrame);});
canvas.addEventListener('contextmenu',event=>event.preventDefault());
window.addEventListener('keydown',event=>{
  if(event.code==='Escape'){endDrag();return;}
  if(!coordinator||coordinator.running||preparing||event.target.closest('input,select,textarea,button,a')||event.ctrlKey||event.metaKey||event.altKey)return;
  const change={KeyW:[.012,0],KeyS:[-.012,0],KeyA:[0,-.012],KeyD:[0,.012]}[event.code];if(!change)return;
  const station=focused(), target=station?.targets.find(item=>item.id===(pipeTask()?station.selected:selectedObject));if(!target)return;
  if(pipeTask()&&station.controller.classification?.started)return;
  event.preventDefault();station.moveObject(target.id,target.mesh.position.clone().add(new THREE.Vector3(change[0],0,change[1])));
});

function drawChart(element,values,colors,{signed=false,minMax=1}={}) {
  const ctx=element.getContext('2d'),w=element.width,h=element.height;ctx.clearRect(0,0,w,h);
  ctx.strokeStyle=workcell.palette.chartGrid;ctx.lineWidth=1;for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke();}
  const scale=Math.max(minMax,...values.flatMap(row=>row.map(Math.abs)))*1.15;
  colors.forEach((color,axis)=>{ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();values.forEach((row,index)=>{const x=index*w/179,y=signed?h/2-row[axis]/scale*(h/2-5):h-5-row[axis]/scale*(h-10);index?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();});
}
function updateUI() {
  if(!coordinator)return;
  const busy=coordinator.running||coordinator.planning||preparing, selected=coordinator.selectedIds, focus=focused(),pipes=pipeTask();
  const needsNewBatch=pipes&&selected.some(id=>stations.get(id).controller.classification?.started);
  $('#selected-count').textContent=`${selected.length} 台`;
  for(const input of document.querySelectorAll('input[data-robot]')){input.checked=selected.includes(input.dataset.robot);input.disabled=busy||!stations.has(input.dataset.robot);}
  $('#select-all').disabled=busy||!stations.size;
  $('#start').disabled=busy||!selected.length||needsNewBatch;$('#start').textContent=coordinator.planning?'路径检查中…':preparing?'准备任务中…':needsNewBatch&&!busy?'请先重新随机摆放':`${pipes?'开始执行':$('#task-mode').value==='stack'?'开始叠放':'开始抓取'} · ${selected.length} 台`;
  $('#stop').disabled=!(coordinator.running||coordinator.planning);$('#stop').textContent=coordinator.planning?'停止规划':'全部停止';$('#reset').disabled=preparing||!stations.size;
  $('#task-mode').disabled=busy;for(const button of document.querySelectorAll('[data-object]')){button.disabled=busy||$('#task-mode').value==='stack';button.setAttribute('aria-pressed',String(button.dataset.object===selectedObject));}
  $('.object-options').hidden=pipes;$('#pipe-task-info').hidden=!pipes;$('#randomize-pipes').hidden=!pipes;$('#randomize-pipes').disabled=busy||!stations.size;
  $('#reset').textContent=pipes?'复位并重新随机摆放':'复位全部工作台';
  const selectionKey=selected.join(',');if($('#focus-robot').dataset.selection!==selectionKey){$('#focus-robot').replaceChildren(...selected.map(id=>{const option=document.createElement('option');option.value=id;option.textContent=ROBOT_CONFIGS[id].label;return option;}));$('#focus-robot').dataset.selection=selectionKey;}
  $('#focus-robot').value=coordinator.focusId;$('#focus-robot').disabled=!focus;
  $('#arm-field').hidden=pipes||focus?.id==='c5';$('#arm-select').disabled=busy||pipes;$('#arm-select').value=focus?.arm??'right';
  $('#focus-hint').textContent=focus?`当前：${focus.config.label}。${pipes?(focus.id==='c5'?'单臂自动分类。':'左右臂自动交替。'):''}${pipes&&focus.controller.classification?.started?'本批次已锁定；复位或重新随机后可开启新批次。':'拖动只修改该工作台；切换观察不改变参与选择。'}`:'请先选择已就绪的机器人。';
  $('#scene-summary').textContent=`${selected.map(id=>ROBOT_CONFIGS[id].shortLabel).join(' / ')} · ${selected.length>1?'同步抓取工作站':'单机器人工作站'}`;
  for(const [id,station] of stations){const snapshot=station.snapshot,row=$(`.result-row[data-robot="${id}"]`),included=selected.includes(id),progress=taskProgress({...snapshot,itemIndex:station.controller.itemIndex});row.dataset.state=snapshot.state;
    row.querySelector('[data-focus]').disabled=!included;row.querySelector('output').textContent=included?(station.planning?'路径规划中':STAGE_NAMES[snapshot.state]??'执行中'):'未参与';
    row.querySelector('progress').value=included?progress:0;row.querySelector('.result-track span').textContent=`${Math.round((included?progress:0)*100)}%`;
    const classification=snapshot.classification;
    row.querySelector('.result-note').textContent=['failed','stopped'].includes(snapshot.state)?snapshot.message:classification?`已归类 ${classification.completed} / ${classification.total} 个${snapshot.running?` · ${id==='c5'?'单臂':snapshot.arm==='left'?'左臂':'右臂'}执行中`:''}`:'';
    $(`.station-tag[data-focus="${id}"]`).setAttribute('aria-pressed',String(id===coordinator.focusId));
  }
  const message=notice||coordinator.message;if(message!==lastStatus){$('#status').textContent=message;lastStatus=message;}
  if(!focus||coordinator.planning)return;
  const m=focus.controller.measure(),telemetry=focus.dynamics.telemetry(focus.arm),history=histories.get(focus.id);
  history.force.push(m.forces);history.torque.push(telemetry.joints.map(joint=>joint.torque));
  if(history.force.length>180){history.force.shift();history.torque.shift();}
  $('#contact-count').textContent=m.count;$('#contact-force').textContent=`${m.force.toFixed(2)} N`;$('#tcp-error').textContent=`${(focus.controller.wristError()*1000).toFixed(2)} mm`;
  drawChart($('#force-chart'),history.force,workcell.palette.chartForce,{minMax:1});
  drawChart($('#torque-chart'),history.torque,workcell.palette.chartTorque.slice(0,telemetry.joints.length),{signed:true,minMax:2});
  $('#joint-readout').textContent=telemetry.joints.map((joint,index)=>`J${index+1} ${(joint.actual*180/Math.PI).toFixed(1)}°`).join(' · ');
  $('#single-link').href=`../${focus.id}/`;$('#single-link').textContent=`打开 ${focus.config.shortLabel} 独立控制页`;
  contactMarkers.forEach((marker,index)=>{const contact=m.contacts[index];marker.visible=Boolean(contact&&contact.force>.015);if(contact){marker.position.copy(focus.localToWorld(contact.position));marker.scale.setScalar(1+Math.min(contact.force,5)*.15);}});
}
$('#task-mode').addEventListener('change',()=>configureTask($('#task-mode').value));

function project(point){const p=point.clone().project(camera);return {x:(p.x+1)*viewport.clientWidth/2,y:(1-p.y)*viewport.clientHeight/2,visible:p.z>-1&&p.z<1&&Math.abs(p.x)<1&&Math.abs(p.y)<1};}
function updateLabels(){
  for(const [id,station]of stations){const tag=$(`.station-tag[data-focus="${id}"]`),p=project(station.localToWorld(new THREE.Vector3(0,station.labelHeight,0)));tag.hidden=!station.group.visible||!p.visible;if(!tag.hidden){tag.style.left=`${p.x}px`;tag.style.top=`${p.y-tag.offsetHeight}px`;}}
}
function updateCameras() {
  const focus=focused(), vision=visions.get(focus?.id);if(!focus||!vision||!showCameras)return;
  const visible=feeds.map((feed,index)=>cameraFeedVisible(feed.element.getBoundingClientRect(),{width:innerWidth,height:innerHeight})?index:-1).filter(index=>index>=0);
  if(!visible.length)return;
  const index=visible.includes(nextFeed)?nextFeed:visible[0];nextFeed=(index+1)%2;
  const top=focus.localToWorld(new THREE.Vector3(focus.config.tableCenter,focus.config.tableHeight+.72,0));
  vision.overhead.position.copy(top);vision.overhead.up.set(1,0,0);vision.overhead.lookAt(focus.localToWorld(new THREE.Vector3(focus.config.tableCenter,focus.config.tableHeight,0)));vision.overhead.updateMatrixWorld(true);
  const shadowUpdate=renderer.shadowMap.autoUpdate;renderer.shadowMap.autoUpdate=false;
  try{
    cameraFeedPass.render(scene,[vision.overhead,vision.camera][index],feedPixels);feedRenderCount++;
    const image=feeds[index].image;for(let row=0;row<180;row++)image.data.set(feedPixels.subarray((179-row)*240*4,(180-row)*240*4),row*240*4);
    feeds[index].ctx.putImageData(image,0,0);
  }finally{renderer.setRenderTarget(null);renderer.shadowMap.autoUpdate=shadowUpdate;}
}
function resize(){const rect=viewport.getBoundingClientRect();renderer.setSize(rect.width,rect.height,false);camera.aspect=rect.width/rect.height;camera.updateProjectionMatrix();}
new ResizeObserver(resize).observe(viewport);resize();
function frame(now) {
  const frameStart=performance.now();if(lastFrame)frameTimes.push(now-lastFrame);if(frameTimes.length>120)frameTimes.shift();
  const wasPlanning=Boolean(coordinator?.planning);
  if(wasPlanning&&!document.hidden&&!preparing){const result=coordinator.advancePlanning();planningSlices++;frameScheduler.reset(performance.now());if(result.done){notice=result.ok?'':result.reason;updateUI();}}
  // A route candidate may finish well after the rAF timestamp. Start the
  // paused clock at its completion so planning time never becomes catch-up.
  const clock=frameScheduler.advance(wasPlanning?performance.now():now,{paused:!coordinator||document.hidden||preparing||wasPlanning,
    step:dt=>{coordinator.step(dt);physicsSteps++;},planning:()=>Boolean(coordinator?.planning)});accumulator=clock.backlogSeconds;
  if(coordinator&&!document.hidden&&!preparing){
    if(clock.steps){for(const station of stations.values())station.syncVisuals();renderer.shadowMap.needsUpdate=true;}
    for(const station of stations.values())if(station.group.visible)station.animate(now,{reducedMotion});
    if(now-lastUI>100){updateUI();lastUI=now;}
  }
  lastFrame=now;orbit.update();updateLabels();
  // Update all rigid wrist mounts before the main pass. Feed rendering is
  // bounded separately and reuses this renderer instead of extra GL contexts.
  for(const [id,station]of stations){const v=visions.get(id);if(station.group.visible)v.wrist.update(station.cameraLink);v.wrist.setVisible(station.group.visible);}
  renderer.render(scene,camera);
  // Prefer a spare frame, but keep a visible feed from starving on slower
  // devices. updateCameras still skips offscreen feeds and draws only one.
  if(coordinator&&!coordinator.planning&&!wasPlanning&&!document.hidden&&now-lastFeed>110&&(performance.now()-frameStart<12||now-lastFeed>1000)){updateCameras();lastFeed=now;}
  renderTimes.push(performance.now()-frameStart);if(renderTimes.length>120)renderTimes.shift();requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

function loadingProgress(event){
  if(event.id){const current=loadStates.get(event.id);if(current.error)return;current.progress=event.part==='ready'?1:Math.max(current.progress,event.value*.95);current.message=event.label;$(`#load-${event.id}`).textContent=event.part==='ready'?'已就绪':`${Math.round(current.progress*100)}%`;}
  const average=[...loadStates.values()].reduce((sum,item)=>sum+item.progress,0)/3;
  $('#loading-progress').value=event.part==='physics'?event.value*.2:.2+average*.8;$('#loading-message').textContent=event.label;
}
async function boot(){
  try{
    const api=await loadPhysicsAPI({onProgress:loadingProgress});
    // Keep the large original STL batches bounded. Some local static servers
    // reset connections when all three loaders open their mesh batches at once.
    for(const id of Object.keys(ROBOT_CONFIGS)){
      try{
        let station;
        try{station=await createRobotRuntime({id,api,offset:STATION_OFFSETS[id],onProgress:loadingProgress,cooperativePlanning:true});}
        catch(error){
          if(!/零件加载失败|场景加载失败/.test(error.message))throw error;
          loadingProgress({id,part:'model',value:0,label:`${ROBOT_CONFIGS[id].label} 连接中断，重试加载一次…`});
          station=await createRobotRuntime({id,api,offset:STATION_OFFSETS[id],onProgress:loadingProgress,cooperativePlanning:true});
        }
        const tableGroup=addStationTable(station);workcell.registerStationTable?.(station,tableGroup);scene.add(station.group);station.group.updateMatrixWorld(true);
        const bounds=new THREE.Box3().setFromObject(station.visualRobot);station.labelHeight=bounds.max.y+.13;
        const wristCamera=new THREE.PerspectiveCamera(65,4/3,.008,12),overhead=new THREE.PerspectiveCamera(70,4/3,.02,12);overhead.layers.enable(1);
        const wrist=(id==='r1'?r1WristVision:dexWristVision)({scene,THREE,camera:wristCamera});
        stations.set(id,station);visions.set(id,{wrist,camera:wristCamera,overhead});histories.set(id,{force:[],torque:[]});
      }catch(error){console.error(error);loadStates.set(id,{progress:0,message:error.message,error:true});$(`#load-${id}`).textContent='加载失败';$(`#load-${id}`).classList.add('error');const row=$(`.result-row[data-robot="${id}"]`);row.querySelector('output').textContent='加载失败';row.querySelector('.result-note').textContent=error.message;}
    }
    // Preserve the requested row/order even if parallel downloads finish in a
    // different order. All physics worlds start together after loading ends.
    const ordered=new Map(Object.keys(ROBOT_CONFIGS).filter(id=>stations.has(id)).map(id=>[id,stations.get(id)]));
    coordinator=new MultiRobotCoordinator({stations:ordered,selectedIds:[...ordered.keys()]});
    if(!stations.size)throw new Error('三个机器人均未加载成功，请检查网络后刷新重试');
    $('#loading').hidden=true;$('#loading-progress').value=1;lastFrame=performance.now();accumulator=0;
    notice=stations.size===3?'三台机器人已就绪；可单选或多选，点击开始后同时抓取。':'部分机器人加载失败；可使用已就绪的机器人，或刷新重试。';
    updateUI();fitView(undefined,{reason:'initial'});
    window.multiDiagnostics=()=>({ ...coordinator.snapshot, selectedObject,preparing,drag:drag?{robotId:drag.station.id,objectId:drag.target.id,mode:drag.mode}:null,
      camera:camera.position.toArray(),viewFit:lastViewFit,orbitEnabled:orbit.enabled,sceneTheme:sceneThemes.activeId,cameraFeedMode:cameraFeedPass.mode,scenePresentation:{environmentMode:workcell.environmentMode,environmentError:workcell.environmentError,resources:workcell.resourceStats},frames:{averageMs:frameTimes.reduce((a,b)=>a+b,0)/(frameTimes.length||1),averageWorkMs:renderTimes.reduce((a,b)=>a+b,0)/(renderTimes.length||1),maxWorkMs:Math.max(0,...renderTimes),physicsSteps,backlogSeconds:accumulator,droppedWallSeconds:frameScheduler.droppedWallSeconds,feedRenderCount,planningSlices},
      view:coordinator.selectedIds.map(id=>({id,origin:stations.get(id).group.position.toArray(),canonicalParent:stations.get(id).robot.parent?.name??null,objects:stations.get(id).snapshot.objects.map(object=>({...object,screen:project(new THREE.Vector3(...object.worldPosition))}))})),
      loading:Object.fromEntries(loadStates)});
  }catch(error){console.error(error);$('#loading-message').textContent=`加载失败：${error.message}`;$('#loading').querySelector('h2').textContent='场景暂时无法加载';$('#status').textContent='加载失败，请刷新重试';}
}
boot();
