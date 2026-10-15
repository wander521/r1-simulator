import * as THREE from '../vendor/three.module.min.js';
import {OrbitControls} from '../vendor/OrbitControls.js';
import URDFLoader from '../vendor/URDFLoader.js';
import {loadC5Mesh} from './c5-3ds.mjs?v=20261015-c5-3';
import createMuJoCo from '../r1/mujoco-runtime.mjs';
import {GraspController,TABLE_HEIGHT} from './grasp-controller.mjs?v=20261015-c5-3';
import {tcpPose,solvePose} from './grasp-kinematics.mjs?v=20261015-c5-3';
import {C5Dynamics} from './c5-dynamics.mjs?v=20261015-c5-3';
import {applyC5Materials} from './c5-materials.mjs?v=20261015-c5-3';
import {createJointUI,createJointTelemetry} from './c5-joint-ui.mjs?v=20261015-c5-3';
import {createWorkcell,createWristVision,configureRenderer,configureOverheadCamera} from './c5-workcell.mjs?v=20261015-c5-3';
import {ROOT_HEIGHT,TABLE_CENTER_X,TABLE_HALF_X,TABLE_HALF_Z,OBJECT_START_X,OBJECT_DROP_X,OBJECT_LATERAL_OFFSET,gripperLinkName,qToOpening,GRIPPER_Q_MIN,GRIPPER_Q_MAX,GRIPPER_GAP_AT_ZERO} from './c5-profile.mjs?v=20261015-c5-3';

const OPENING_MIN=Math.min(qToOpening(GRIPPER_Q_MIN),qToOpening(GRIPPER_Q_MAX)),OPENING_MAX=Math.max(qToOpening(GRIPPER_Q_MIN),qToOpening(GRIPPER_Q_MAX));

const $=s=>document.querySelector(s),canvas=$('#main-view'),stage=$('#viewport');
$('#start-grasp').disabled=true;$('#reset').disabled=true;
const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(39,1,.02,15);
scene.background=new THREE.Color(0x071323);scene.fog=new THREE.FogExp2(0x071323,.08);
camera.position.set(2.2,1.6,2.6);
const renderer=new THREE.WebGLRenderer({canvas,antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
configureRenderer(renderer,{THREE});camera.layers.enable(1);
const orbit=new OrbitControls(camera,canvas);orbit.target.set(.25,.80,0);orbit.minDistance=.7;orbit.maxDistance=5;orbit.maxPolarAngle=Math.PI*.49;orbit.enableDamping=true;orbit.dampingFactor=.12;orbit.update();
const workcell=createWorkcell(scene,{THREE,TABLE_HEIGHT,tableCenter:TABLE_CENTER_X,tableHalfX:TABLE_HALF_X,tableHalfZ:TABLE_HALF_Z});
const material=(color,metalness=.3,roughness=.45)=>new THREE.MeshStandardMaterial({color,metalness,roughness});

const specs=[
  {id:'red',body:'red_cube',label:'红色方块',color:0xeb140f,zoneColor:0xef5260,h:.0225,z:OBJECT_LATERAL_OFFSET,halfX:.0225,halfZ:.0225},
  {id:'blue',body:'blue_block',label:'蓝色长方块',color:0x076bf0,zoneColor:0x45aef2,h:.019,z:0,halfX:.026,halfZ:.016},
  {id:'yellow',body:'yellow_cylinder',label:'黄色圆柱',color:0xffb80a,zoneColor:0xe6d957,h:.026,z:-OBJECT_LATERAL_OFFSET,halfX:.02,halfZ:.02},
];
const targets=specs.map(s=>{
  const geometry=s.id==='yellow'?new THREE.CylinderGeometry(.02,.02,.052,32):new THREE.BoxGeometry(s.halfX*2,s.h*2,s.halfZ*2);
  const mat=material(s.color,.15,.35),mesh=new THREE.Mesh(geometry,mat);mesh.userData.taskId=s.id;mesh.castShadow=true;mesh.receiveShadow=true;scene.add(mesh);
  const start=new THREE.Vector3(OBJECT_START_X,TABLE_HEIGHT+s.h,s.z),drop=new THREE.Vector3(OBJECT_DROP_X,TABLE_HEIGHT+s.h,s.z);mesh.position.copy(start);
  const zone=new THREE.Group();zone.position.set(drop.x,TABLE_HEIGHT+.0018,drop.z);scene.add(zone);
  const zoneMat=new THREE.MeshStandardMaterial({color:s.zoneColor,emissive:s.zoneColor,emissiveIntensity:.3,transparent:true,opacity:.86});
  for(const [x,z,w,d] of [[0,-.04,.08,.006],[0,.04,.08,.006],[-.04,0,.006,.08],[.04,0,.006,.08]]){const bar=new THREE.Mesh(new THREE.BoxGeometry(w,.002,d),zoneMat);bar.position.set(x,0,z);zone.add(bar);}
  return {...s,start,drop,rotation:new THREE.Quaternion(),mesh,zone};
});
const byId=id=>targets.find(t=>t.id===id);
let robot,controller,jointUI,jointTelemetry,selected='blue',arm='left',drag=null,tcpEnabled=false,showGuides=false,showContacts=true,showCameras=true,lastTime=0,accumulator=0,lastUI=0,lastVision=0;
const reducedMotion=matchMedia('(prefers-reduced-motion: reduce)').matches;
const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();
const contactGroup=new THREE.Group();scene.add(contactGroup);
const markerGeometry=new THREE.SphereGeometry(.0035,8,6),markerMaterial=new THREE.MeshBasicMaterial({color:0x50f4c8});
const contactMarkers=Array.from({length:24},()=>{const m=new THREE.Mesh(markerGeometry,markerMaterial);contactGroup.add(m);return m;});
const forceHistory=[],errorHistory=[];
const cameras=[new THREE.PerspectiveCamera(48,4/3,.02,10),new THREE.PerspectiveCamera(65,4/3,.01,10)];cameras[0].position.set(.8,2.0,.001);cameras[0].up.set(1,0,0);cameras[0].lookAt(.8,TABLE_HEIGHT,0);
const feeds=['#overhead-camera-canvas','#wrist-camera-canvas'].map(selector=>{const r=new THREE.WebGLRenderer({canvas:$(selector),antialias:true});r.setSize(320,240,false);return r;});
feeds.forEach(r=>configureRenderer(r,{THREE}));configureOverheadCamera(cameras[0],{THREE,TABLE_HEIGHT,tableCenter:TABLE_CENTER_X});
const wristVision=createWristVision({scene,THREE,camera:cameras[1]});
const stages={idle:'六轴机械臂待命',ready:'移动至安全预备位',recenter:'回到安全预备位',align:'对准任务目标',spread:'移动至安全预备位',raise:'提升至安全高度',approach:'接近目标',descend:'下降对准',closing:'闭合并验证双指接触',lift:'抬升验证',transfer:'搬运至放置位置',place:'支撑接触验证',release:'松开夹爪',retreat:'夹爪向上撤离',settling:'等待物体稳定',return_clear:'安全撤离',return:'回到待命位',stack_settle:'三层支撑稳定验证',complete:'任务完成',failed:'抓取失败',stopped:'已停止'};
const sequence=['ready','recenter','align','approach','descend','closing','lift','transfer','place','release','retreat','settling','return_clear','return'];
Object.assign(stages,{reaction_home:'结果反馈准备 · 机械臂回位',happy:'叠放成功 · 庆祝',shy:'叠放失败 · 提示'});
const reactionProgress={reaction_home:.93,happy:.99,shy:.99};
const stageLabel=state=>stages[state]||'调整至安全姿态';
const status=message=>{$('#status').textContent=message;};

function select(id){if(controller?.running)return;selected=id;if(controller)controller.selected=id;document.querySelectorAll('.vision-target').forEach(b=>{b.classList.toggle('active',b.dataset.target===id);b.setAttribute('aria-pressed',String(b.dataset.target===id));});$('#object-edit-selection').textContent=`已选择：${byId(id).label}`;}
$('.vision-targets').innerHTML=targets.map(t=>`<button class="vision-target" type="button" data-target="${t.id}" aria-pressed="false"><i class="target-${t.id}"></i><span>${t.label}</span></button>`).join('');
document.querySelectorAll('.vision-target').forEach(b=>b.addEventListener('click',()=>select(b.dataset.target)));
select(selected);
$('.object-edit-hud-heading span').textContent='WASD';
$('.object-edit-hud p:last-child').textContent='拖动物体移动；右键或 Shift＋拖动物体旋转。拖动空白处旋转视角，滚轮缩放；WASD 移动物体。';
$('.control-hint').textContent='选择物体放入同色框，或依次叠放蓝色、红色、黄色。六轴 C5 安装于落地基座，Dex1 双指实际接触并稳定夹持后才开始搬运；物体可点选、拖移和旋转。';
$('.telemetry-chart span').textContent='双指接触力 · N';document.querySelectorAll('.telemetry-chart span')[1].textContent='腕部跟踪误差 · mm';document.querySelectorAll('.telemetry-metrics span')[2].textContent='腕部位置误差';
document.querySelectorAll('.camera-feed-heading span')[2].textContent='C5 夹爪前置相机';
$('#toggle-drag').setAttribute('aria-pressed','false');
$('#toggle-guides').textContent='显示全部引导';$('#start-stack').hidden=false;$('#start-stack').disabled=true;

function objectExtents(target,q){
  const m=new THREE.Matrix4().makeRotationFromQuaternion(q).elements;
  if(target.id==='yellow')return [0,1,2].map(k=>.02*Math.sqrt(Math.max(0,1-m[4+k]*m[4+k]))+target.h*Math.abs(m[4+k]));
  return [0,1,2].map(k=>Math.abs(m[k])*target.halfX+Math.abs(m[4+k])*target.h+Math.abs(m[8+k])*target.halfZ);
}
function moveObject(target,position,quaternion=target.mesh.quaternion){
  const e=objectExtents(target,quaternion),p=position.clone();p.x=THREE.MathUtils.clamp(p.x,TABLE_CENTER_X-TABLE_HALF_X+e[0]+.008,TABLE_CENTER_X+TABLE_HALF_X-e[0]-.008);p.z=THREE.MathUtils.clamp(p.z,-TABLE_HALF_Z+e[2]+.008,TABLE_HALF_Z-e[2]-.008);p.y=TABLE_HEIGHT+e[1]+.0001;
  const overlap=targets.some(t=>{if(t===target)return false;const h=objectExtents(t,t.mesh.quaternion);return Math.abs(p.x-t.mesh.position.x)<e[0]+h[0]+.006&&Math.abs(p.z-t.mesh.position.z)<e[2]+h[2]+.006;});
  if(overlap){status('该位置与其他物体重叠，请移开一些');return false;}
  controller.setObject(target.id,p,quaternion);target.mesh.position.copy(p);target.mesh.quaternion.copy(quaternion);return true;
}
function ray(event){const r=canvas.getBoundingClientRect();pointer.set((event.clientX-r.left)/r.width*2-1,1-(event.clientY-r.top)/r.height*2);raycaster.setFromCamera(pointer,camera);}
function down(event){
  if(!controller||controller.running||drag||event.isPrimary===false||![0,2].includes(event.button))return;
  ray(event);
  // Only the solid object meshes are pickable. THREE.Line's default 1 m
  // threshold previously made almost the entire scene an object hit region.
  const hit=raycaster.intersectObjects(targets.map(t=>t.mesh),false)[0];if(!hit)return;
  const target=byId(hit.object.userData.taskId);select(target.id);
  const plane=new THREE.Plane(new THREE.Vector3(0,1,0),-target.mesh.position.y),point=raycaster.ray.intersectPlane(plane,new THREE.Vector3());if(!point)return;
  drag={id:event.pointerId,target,plane,offset:target.mesh.position.clone().sub(point),x:event.clientX,y:event.clientY,rotation:target.mesh.quaternion.clone(),mode:event.button===2||event.shiftKey?'rotate':'move'};
  orbit.enabled=false;canvas.setPointerCapture(event.pointerId);event.preventDefault();event.stopImmediatePropagation();stage.classList.add('object-dragging');
}
function move(event){
  if(!drag||event.pointerId!==drag.id)return;
  if(drag.mode==='tcp'){
    ray(event);const p=raycaster.ray.intersectPlane(drag.plane,new THREE.Vector3());if(!p)return;
    const result=solvePose(robot,arm,p,drag.rotation,controller.pose,14);
    if(result.positionError<.008&&result.angleError<.08){drag.rejectMessage=controller.applyManualPose({...controller.pose,...result.pose})?null:controller.message;}else drag.rejectMessage='末端目标暂不可达，请移近机器人或调整关节';
    if(drag.rejectMessage)status(drag.rejectMessage);
    controller.dynamics?.syncRobot();return;
  }
  if(drag.mode==='rotate'){
    const yaw=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),(event.clientX-drag.x)*.012),axis=new THREE.Vector3(1,0,0).applyQuaternion(camera.quaternion),pitch=new THREE.Quaternion().setFromAxisAngle(axis,(event.clientY-drag.y)*.012);
    moveObject(drag.target,drag.target.mesh.position,yaw.multiply(pitch).multiply(drag.rotation));
  }else{ray(event);const p=raycaster.ray.intersectPlane(drag.plane,new THREE.Vector3());if(p)moveObject(drag.target,p.add(drag.offset));}
  event.preventDefault();
}
function end(event){if(!drag||(event&&event.pointerId!==drag.id))return;const previous=drag;drag=null;if(canvas.hasPointerCapture(previous.id))canvas.releasePointerCapture(previous.id);orbit.enabled=true;stage.classList.remove('object-dragging','tcp-dragging');status(previous.mode==='tcp'?(previous.rejectMessage||'末端位置已调整'):`${previous.target.label}位置已更新`);}
canvas.addEventListener('pointerdown',down,{capture:true});window.addEventListener('pointermove',move);window.addEventListener('pointerup',end);window.addEventListener('pointercancel',end);canvas.addEventListener('lostpointercapture',end);window.addEventListener('blur',()=>end());canvas.addEventListener('contextmenu',e=>e.preventDefault());
window.addEventListener('keydown',event=>{
  if(event.code==='Escape'){end();return;}
  if(!controller||controller.running||/INPUT|SELECT|TEXTAREA/.test(event.target.tagName)||event.metaKey||event.ctrlKey||event.altKey)return;
  const delta={KeyW:[.012,0],KeyS:[-.012,0],KeyA:[0,-.012],KeyD:[0,.012]}[event.code];if(!delta)return;
  event.preventDefault();const target=byId(selected);moveObject(target,target.mesh.position.clone().add(new THREE.Vector3(delta[0],0,delta[1])));
});

function drawChart(canvas,values,colors,max){const ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height;ctx.clearRect(0,0,w,h);ctx.strokeStyle='#244554';ctx.lineWidth=1;for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke();}colors.forEach((color,j)=>{ctx.strokeStyle=color;ctx.beginPath();values.forEach((v,i)=>{const x=i*w/179,y=h-4-Math.min(1,v[j]/max)*(h-8);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();});}
function updateUI(){
  const m=controller.measurement||controller.measure(),running=controller.running;
  $('#start-grasp').disabled=running;$('#start-stack').disabled=running;$('#cancel-grasp').disabled=!running;$('#reset').disabled=false;
  document.querySelectorAll('.vision-target').forEach(b=>b.disabled=running);
  $('#toggle-drag').disabled=running;$('#gripper-open').disabled=running;$('#gripper-close').disabled=running;
  $('#grasp-stage').textContent=stageLabel(controller.state);
  if(selected!==controller.selected){selected=controller.selected;document.querySelectorAll('.vision-target').forEach(b=>{b.classList.toggle('active',b.dataset.target===selected);b.setAttribute('aria-pressed',String(b.dataset.target===selected));});$('#object-edit-selection').textContent=`已选择：${byId(selected).label}`;}
  const index=sequence.indexOf(controller.state),itemProgress=index<0?0:(index+.5)/sequence.length;
  const progress=controller.state==='complete'?1:controller.task==='stack'?(reactionProgress[controller.state]??(controller.state==='stack_settle'?.88+Math.min(.03,controller.stackStableSeconds*.01):Math.min(.87,(controller.itemIndex+itemProgress)/3*.88))):itemProgress;
  $('#grasp-progress').value=progress;$('#grasp-progress-value').textContent=`${Math.round(progress*100)}%`;
  if(running)status(`C5 · ${controller.task==='stack'?`叠放 ${controller.itemIndex+1}/3 · `:''}${byId(selected).label} · ${stageLabel(controller.state)}${controller.reaction==='shy'?` · ${controller.message}`:''}`);
  else if(['failed','complete','stopped'].includes(controller.state))status(controller.message);
  $('#contact-count').textContent=m.count;$('#grasp-force').textContent=`${m.force.toFixed(2)} N`;
  const error=controller.wristError()*1000;
  $('#rms-error').textContent=`${error.toFixed(2)} mm`;
  forceHistory.push(m.forces);errorHistory.push([error]);if(forceHistory.length>180){forceHistory.shift();errorHistory.shift();}
  drawChart($('#torque-chart'),forceHistory,['#5ee0ff','#f5d76e'],12);drawChart($('#error-chart'),errorHistory,['#8fd7c1'],5);
  const telemetry=controller.dynamics.telemetry(arm);jointTelemetry.update({joints:telemetry.joints.map(j=>({...j,torqueLimit:j.effortLimit})),time:telemetry.time});
  contactMarkers.forEach((marker,i)=>{const c=m.contacts[i];marker.visible=showContacts&&!!c&&c.force>.015;if(c){marker.position.copy(c.position);marker.scale.setScalar(1+Math.min(c.force,5)*.15);}});
}

function disableTCP(){tcpEnabled=false;$('#toggle-drag').textContent='启用末端拖拽';$('#toggle-drag').setAttribute('aria-pressed','false');}
$('#start-grasp').addEventListener('click',()=>{end();disableTCP();if(controller&&!controller.start(arm,selected))status(controller.message);updateUI();});
$('#start-stack').addEventListener('click',()=>{end();disableTCP();controller?.startStack(arm,{observerYaw:Math.atan2(-camera.position.z,camera.position.x),reducedMotion});updateUI();});
$('#cancel-grasp').addEventListener('click',()=>{controller?.stop();updateUI();});
$('#reset').addEventListener('click',()=>{end();controller?.reset();jointTelemetry?.reset();forceHistory.length=errorHistory.length=0;status('已复位，选择物体后开始抓取');});
$('#gripper-open').addEventListener('click',()=>{if(controller&&!controller.running&&!controller.setManualOpening(OPENING_MAX))status(controller.message);});
$('#gripper-close').addEventListener('click',()=>{if(controller&&!controller.running&&!controller.setManualOpening(OPENING_MIN))status(controller.message);});
$('#toggle-drag').addEventListener('click',()=>{tcpEnabled=!tcpEnabled;$('#toggle-drag').textContent=tcpEnabled?'退出末端拖拽':'启用末端拖拽';$('#toggle-drag').setAttribute('aria-pressed',String(tcpEnabled));});
$('#drag-marker').addEventListener('pointerdown',event=>{if(!tcpEnabled||controller.running||drag)return;const p=tcpPose(robot,arm);drag={id:event.pointerId,mode:'tcp',plane:new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()),p.position),rotation:p.quaternion};orbit.enabled=false;stage.classList.add('tcp-dragging');canvas.setPointerCapture(event.pointerId);event.preventDefault();event.stopPropagation();});
$('#toggle-guides').addEventListener('click',()=>{showGuides=!showGuides;jointUI?.setAllGuides(showGuides);$('#toggle-guides').setAttribute('aria-pressed',String(showGuides));$('#toggle-guides').textContent=showGuides?'隐藏全部引导':'显示全部引导';});
$('#toggle-camera-views').addEventListener('click',()=>{showCameras=!showCameras;$('#camera-feed-grid').hidden=!showCameras;$('#toggle-camera-views').textContent=showCameras?'隐藏画面':'显示画面';$('#toggle-camera-views').setAttribute('aria-checked',String(showCameras));});
for(const layout of ['row','column'])$(`#camera-layout-${layout}`).addEventListener('click',()=>{$('#camera-feed-grid').classList.toggle('is-column',layout==='column');for(const l of ['row','column']){$(`#camera-layout-${l}`).setAttribute('aria-pressed',String(l===layout));$(`#camera-layout-${l}`).classList.toggle('active',l===layout);}});
$('#toggle-camera-model').addEventListener('click',()=>{const visible=wristVision.setVisible(!wristVision.visible);$('#toggle-camera-model').textContent=visible?'隐藏相机模型':'显示相机模型';$('#toggle-camera-model').setAttribute('aria-checked',String(visible));});
$('#toggle-contact-visuals').addEventListener('click',()=>{showContacts=!showContacts;$('#toggle-contact-visuals').textContent=showContacts?'隐藏接触点':'显示接触点';$('#toggle-contact-visuals').setAttribute('aria-checked',String(showContacts));});
function project(point){const p=point.clone().project(camera),r=canvas.getBoundingClientRect();return{x:(p.x+1)*r.width/2,y:(1-p.y)*r.height/2,visible:p.z<1&&Math.abs(p.x)<1&&Math.abs(p.y)<1};}
function resize(){const r=stage.getBoundingClientRect();renderer.setSize(r.width,r.height,false);camera.aspect=r.width/r.height;camera.fov=camera.aspect<1?54:39;camera.updateProjectionMatrix();}
new ResizeObserver(resize).observe(stage);resize();
function frame(now){
  requestAnimationFrame(frame);const dt=lastTime?Math.min(.05,(now-lastTime)/1000):0;lastTime=now;
  if(controller){
    accumulator+=dt;while(accumulator>=.001){controller.step(.001);accumulator-=.001;}
    controller.syncFingers();
    for(const t of targets){const p=controller.objectPose(t.id);t.mesh.position.copy(p.position);t.mesh.quaternion.copy(p.quaternion);if(drag?.target===t)controller.setObject(t.id,t.mesh.position,t.mesh.quaternion);
      // Same pulse timing and emissive range as the reference view's Q0().
      const pulse=reducedMotion?.5:.5+.5*Math.sin(now*.006);t.mesh.material.emissive.set(t.color).multiplyScalar(.1);t.mesh.material.emissiveIntensity=.18;
      if(t.id===selected&&!controller.running){t.mesh.material.emissive.lerp(new THREE.Color(0x5ee0ff),.38+pulse*.28);t.mesh.material.emissiveIntensity=.42+pulse*.72;}
    }
    const link=robot.getObjectByName(gripperLinkName(arm)),tcp=tcpPose(robot,arm).position;wristVision.update(link);
    const marker=project(tcp);$('#drag-cluster').classList.toggle('active',tcpEnabled&&marker.visible&&!controller.running);$('#drag-cluster').style.left=`${marker.x}px`;$('#drag-cluster').style.top=`${marker.y}px`;
    if(now-lastUI>100){updateUI();lastUI=now;}
  }
  orbit.update();
  if(controller){const samples=controller.dynamics.telemetry(arm),f=controller.fingers[arm];const opening=GRIPPER_GAP_AT_ZERO+f.reduce((sum,j)=>sum+controller.data.qpos[j.q],0);jointUI.update({joints:samples.joints,opening,openingTarget:controller.opening,openingLower:OPENING_MIN,openingUpper:OPENING_MAX,running:controller.running});}
  renderer.render(scene,camera);
  if(showCameras&&now-lastVision>80){feeds.forEach((r,i)=>r.render(scene,cameras[i]));lastVision=now;}
}
requestAnimationFrame(frame);
document.addEventListener('visibilitychange',()=>{lastTime=0;accumulator=0;});
const loadingParts={model:0,wasm:0,scene:0,physics:0},loadingTrace=[];
function setLoading(part,value,label){loadingParts[part]=value;const percent=Math.round(loadingParts.model*55+loadingParts.wasm*28+loadingParts.scene*5+loadingParts.physics*12);$('#loading-text').textContent=label;$('#loading-progress-fill').style.width=`${percent}%`;$('#loading-progress-value').textContent=`${percent}%`;$('.loading-progress').setAttribute('aria-valuenow',String(percent));loadingTrace.push({part,value,percent,time:performance.now()});}
async function wasmWithProgress(){
  const url=new URL('../Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url),response=await fetch(url);if(!response.ok)throw new Error(`物理运行库加载失败 ${response.status}`);
  const total=Number(response.headers.get('content-length')),chunks=[];let loaded=0;const reader=response.body?.getReader();
  if(reader){while(true){const {done,value}=await reader.read();if(done)break;chunks.push(value);loaded+=value.length;setLoading('wasm',.9*(total?loaded/total:loaded/(loaded+1e6)),`物理运行库 ${Math.round(loaded/1024)} KB${total?` / ${Math.round(total/1024)} KB`:''}`);}}
  else {const bytes=new Uint8Array(await response.arrayBuffer());chunks.push(bytes);loaded=bytes.length;}
  const bytes=new Uint8Array(loaded);let offset=0;for(const chunk of chunks){bytes.set(chunk,offset);offset+=chunk.length;}
  const api=await createMuJoCo({wasmBinary:bytes,locateFile:()=>url.href});setLoading('wasm',1,'物理运行库已就绪');return api;
}
async function boot(){try{
  $('#loading-text').textContent='加载机器人及物理夹爪…';
  const manager=new THREE.LoadingManager();
  manager.onProgress=(url,loaded,total)=>setLoading('model',loaded/total,`机器人零件 ${loaded} / ${total}`);
  const meshesReady=new Promise((resolve,reject)=>{manager.onLoad=resolve;manager.onError=url=>reject(new Error(`模型零件加载失败：${url}`));});
  const loader=new URDFLoader(manager);
  const defaultMeshLoader=loader.defaultMeshLoader.bind(loader);
  loader.loadMeshCb=(path,manager,done)=>/\.3ds(?:[?#]|$)/i.test(path)?loadC5Mesh(path,manager,done):defaultMeshLoader(path,manager,done);
  // The official C5 package supplies 3DS visuals. Preserve their face groups,
  // source colors and millimetre-to-metre conversion, without replacing them
  // with the coarser collision meshes. Dex1 uses its original STL loader.
  const completeRobot=Promise.all([loader.loadAsync('./assets/c5-dex1.urdf'),meshesReady]).then(([r])=>r);
  const [loaded,api,xml]=await Promise.all([completeRobot,wasmWithProgress(),fetch('./models/c5-dynamic-scene.xml?v=20261015-c5-3').then(async r=>{if(!r.ok)throw new Error(`场景加载失败 ${r.status}`);const xml=await r.text();setLoading('scene',1,'关节物理场景已加载');return xml;})]);
  robot=loaded;robot.rotation.x=-Math.PI/2;robot.position.y=ROOT_HEIGHT;robot.traverse(n=>{if(n.isMesh){n.castShadow=true;n.receiveShadow=true;}});scene.add(robot);applyC5Materials(robot);
  setLoading('physics',.2,'建立关节、电机和碰撞状态…');
  const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),dynamics=new C5Dynamics(api,model,data,robot);controller=new GraspController(api,model,data,robot,targets,dynamics);vfs.delete();
  jointUI=createJointUI({root:$('#callouts'),robot,getArm:()=>arm,project,onJoint:(name,value)=>{if(!controller.running&&!controller.applyManualPose({...controller.pose,[name]:value}))status(controller.message);},onOpening:value=>{if(!controller.running&&!controller.setManualOpening(value))status(controller.message);}});
  jointTelemetry=createJointTelemetry($('.panel'));
  setLoading('physics',1,'场景已就绪');await new Promise(requestAnimationFrame);
  $('#loading-overlay').hidden=true;status('遨博 C5 六轴待命；拖动空白旋转视角，点选物体开始抓取');
  // Read-only diagnostics used by browser verification; no test-only motions.
  window.c5Diagnostics=()=>({state:controller.state,task:controller.task,itemIndex:controller.itemIndex,running:controller.running,message:controller.message,history:[...controller.history],evidence:{...controller.evidence},stackEvidence:controller.stackEvidence,stackStableSeconds:controller.stackStableSeconds,reaction:controller.reaction,reactionFinished:controller.reactionFinished,stackCheck:controller.task==='stack'?controller.stackCheck():null,camera:camera.position.toArray(),orbitEnabled:orbit.enabled,drag:drag?.mode||null,selected,objects:targets.map(t=>({id:t.id,position:t.mesh.position.toArray(),quaternion:t.mesh.quaternion.toArray(),screen:project(t.mesh.position)})),measurement:controller.measure(),planError:controller.planError,telemetry:controller.dynamics.telemetry(arm),loading:loadingTrace,wristCamera:{position:cameras[1].position.toArray(),direction:cameras[1].getWorldDirection(new THREE.Vector3()).toArray(),fov:cameras[1].fov,modelVisible:wristVision.visible},tcp:project(tcpPose(robot,arm).position),simTime:controller.time});
}catch(error){console.error(error);$('#loading-text').textContent=`加载失败：${error.message}。刷新可重试。`;}}
boot();

