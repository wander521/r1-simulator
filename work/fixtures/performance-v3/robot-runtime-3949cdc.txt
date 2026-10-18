import * as THREE from '../vendor/three.module.min.js';
import createMuJoCo from '../r1/mujoco-runtime.mjs';
import {GraspController as R1Controller,TABLE_HEIGHT as R1_TABLE_HEIGHT} from '../r1/grasp-controller.mjs';
import {GraspController as G1Controller} from '../g1/grasp-controller.mjs';
import {GraspController as C5Controller} from '../c5/grasp-controller.mjs';
import {R1SafeDynamics} from './r1-safe-dynamics.mjs';
import {G1Dynamics} from '../g1/g1-dynamics.mjs';
import {C5Dynamics} from '../c5/c5-dynamics.mjs';
import {applyRobotMaterials} from '../r1/r1-materials.mjs';
import {applyG1Materials} from '../g1/g1-materials.mjs';
import {applyC5Materials} from '../c5/c5-materials.mjs';
import {loadC5Mesh} from '../c5/c5-3ds.mjs';
import * as G1 from '../g1/g1-profile.mjs';
import * as C5 from '../c5/c5-profile.mjs';
import {tcpPose as r1TCP} from '../r1/grasp-kinematics.mjs';
import {tcpPose as g1TCP} from '../g1/grasp-kinematics.mjs';
import {tcpPose as c5TCP} from '../c5/grasp-kinematics.mjs';
import {generatePipeLayout,normalizePipeSeed} from './pipe-layout.mjs';
import {buildPipeScene} from './pipe-model.mjs';
import {pipeControllerClass,pipeDynamicsClass} from './pipe-controller.mjs';
import {planningControllerClass} from './planning.mjs?v=20261018-perf-2';
import {backgroundPlanningControllerClass} from './background-planning.mjs?v=20261018-perf-2';
import {createPlanningWorkerTransport} from './planning-worker-transport.mjs?v=20261018-perf-2';
import {capturePlanningJob,planningJobStillCurrent} from './planning-serialization.mjs?v=20261018-perf-2';

const url=path=>new URL(path,import.meta.url).href;
export const ROBOT_CONFIGS=Object.freeze({
  r1:Object.freeze({id:'r1',label:'星海图 R1',shortLabel:'R1',color:0x5eddeb,rootHeight:0,
    tableHeight:R1_TABLE_HEIGHT,tableCenter:.78,tableHalfX:.36,tableHalfZ:.38,startX:.78,dropX:.90,lateral:.14,
    urdf:url('../r1/assets/r1.urdf'),xml:url('../r1/models/r1-dynamic-scene.xml'),defaultArm:'right',
    Controller:R1Controller,Dynamics:R1SafeDynamics,applyMaterials:applyRobotMaterials,tcp:r1TCP,
    gripperLink:arm=>`${arm}_gripper_link`,cameraFrame:arm=>`${arm}_realsense_link`}),
  g1:Object.freeze({id:'g1',label:'宇树 G1',shortLabel:'G1',color:0x9bd2f6,rootHeight:G1.ROOT_HEIGHT,
    tableHeight:G1.TABLE_HEIGHT,tableCenter:G1.TABLE_CENTER_X,tableHalfX:G1.TABLE_HALF_X,tableHalfZ:G1.TABLE_HALF_Z,
    startX:G1.OBJECT_START_X,dropX:G1.OBJECT_DROP_X,lateral:G1.OBJECT_LATERAL_OFFSET,
    urdf:url('../g1/assets/g1_29dof_with_dex1_1.urdf'),xml:url('../g1/models/g1-dynamic-scene.xml'),defaultArm:'right',
    Controller:G1Controller,Dynamics:G1Dynamics,applyMaterials:applyG1Materials,tcp:g1TCP,gripperLink:G1.gripperLinkName}),
  c5:Object.freeze({id:'c5',label:'遨博 C5',shortLabel:'C5',color:0x608fe8,rootHeight:C5.ROOT_HEIGHT,
    tableHeight:C5.TABLE_HEIGHT,tableCenter:C5.TABLE_CENTER_X,tableHalfX:C5.TABLE_HALF_X,tableHalfZ:C5.TABLE_HALF_Z,
    startX:C5.OBJECT_START_X,dropX:C5.OBJECT_DROP_X,lateral:C5.OBJECT_LATERAL_OFFSET,
    urdf:url('../c5/assets/c5-dex1.urdf'),xml:url('../c5/models/c5-dynamic-scene.xml'),defaultArm:'left',
    plinthRadius:C5.PLINTH_RADIUS,plinthHeight:C5.PLINTH_HEIGHT,
    Controller:C5Controller,Dynamics:C5Dynamics,applyMaterials:applyC5Materials,tcp:c5TCP,gripperLink:C5.gripperLinkName}),
});

let physicsPromise;
const physicsListeners=new Set();
function physicsProgress(value,label){for(const listener of physicsListeners)listener({part:'physics',value,label});}

/** One Wasm module, independent MjModel/MjData per station. No model is shared. */
export function loadPhysicsAPI({onProgress}={}){
  if(onProgress)physicsListeners.add(onProgress);
  if(!physicsPromise)physicsPromise=(async()=>{
    const response=await fetch(url('../Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'));
    if(!response.ok)throw new Error(`物理运行库加载失败（${response.status}），请重试`);
    const total=Number(response.headers.get('content-length')),chunks=[];let size=0;
    const reader=response.body?.getReader();
    if(reader){while(true){const {done,value}=await reader.read();if(done)break;chunks.push(value);size+=value.length;physicsProgress(total?size/total*.9:size/(size+1e6)*.9,'加载共享物理运行库');}}
    else {const value=new Uint8Array(await response.arrayBuffer());chunks.push(value);size=value.length;}
    const wasmBinary=new Uint8Array(size);let offset=0;for(const chunk of chunks){wasmBinary.set(chunk,offset);offset+=chunk.length;}
    const api=await createMuJoCo({wasmBinary,locateFile:()=>url('../Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm')});
    physicsProgress(1,'物理运行库已就绪');return api;
  })().catch(error=>{physicsPromise=undefined;throw error;});
  return physicsPromise.finally(()=>{if(onProgress)physicsListeners.delete(onProgress);});
}

function configFor(id){const config=ROBOT_CONFIGS[id];if(!config)throw new Error(`未知机器人：${id}`);return config;}

async function loadRobot(config,onProgress){
  // Keep the browser-only loader deferred: headless tests use the same adapter
  // with original-source joint-tree fixtures, without a DOM or fake mesh loader.
  const {default:URDFLoader}=await import('../vendor/URDFLoader.js');
  const manager=new THREE.LoadingManager();
  manager.onProgress=(path,loaded,total)=>onProgress?.({id:config.id,part:'model',value:loaded/total,label:`${config.label} 零件 ${loaded}/${total}`});
  const meshesReady=new Promise((resolve,reject)=>{manager.onLoad=resolve;manager.onError=path=>reject(new Error(`${config.label} 零件加载失败：${path}`));});
  const loader=new URDFLoader(manager);
  if(config.id==='r1')loader.packages={r1_v2_1_0:url('../r1/assets')};
  if(config.id==='c5'){
    const original=loader.defaultMeshLoader.bind(loader);
    loader.loadMeshCb=(path,loadingManager,done)=>/\.3ds(?:[?#]|$)/i.test(path)?loadC5Mesh(path,loadingManager,done):original(path,loadingManager,done);
  }
  const [robot]=await Promise.all([loader.loadAsync(config.urdf),meshesReady]);
  robot.rotation.x=-Math.PI/2;robot.position.set(0,config.rootHeight,0);
  robot.traverse(node=>{if(node.isMesh){node.castShadow=true;node.receiveShadow=true;}});
  config.applyMaterials(robot);robot.updateMatrixWorld(true);return robot;
}

function taskTargets(config,group){
  const specs=[
    {id:'red',body:'red_cube',label:'红色方块',color:0xeb140f,zoneColor:0xef5260,h:.0225,z:config.lateral,halfX:.0225,halfZ:.0225},
    {id:'blue',body:'blue_block',label:'蓝色长方块',color:0x076bf0,zoneColor:0x45aef2,h:.019,z:0,halfX:.026,halfZ:.016},
    {id:'yellow',body:'yellow_cylinder',label:'黄色圆柱',color:0xffb80a,zoneColor:0xe6d957,h:.026,z:-config.lateral,halfX:.02,halfZ:.02},
  ];
  return specs.map(spec=>{
    const geometry=spec.id==='yellow'?new THREE.CylinderGeometry(.02,.02,.052,32):new THREE.BoxGeometry(spec.halfX*2,spec.h*2,spec.halfZ*2);
    const material=new THREE.MeshStandardMaterial({color:spec.color,metalness:.15,roughness:.35});
    const mesh=new THREE.Mesh(geometry,material),visualMesh=mesh.clone();
    const start=new THREE.Vector3(config.startX,config.tableHeight+spec.h,spec.z),drop=new THREE.Vector3(config.dropX,config.tableHeight+spec.h,spec.z);
    mesh.position.copy(start);visualMesh.position.copy(start);visualMesh.castShadow=true;visualMesh.receiveShadow=true;
    Object.assign(visualMesh.userData,{taskId:spec.id,robotId:config.id});visualMesh.name=`${config.id}-${spec.id}-task-object`;group.add(visualMesh);
    const zone=new THREE.Group();zone.name=`${config.id}-${spec.id}-placement-frame`;zone.position.set(drop.x,config.tableHeight+.0018,drop.z);group.add(zone);
    const zoneMaterial=new THREE.MeshStandardMaterial({color:spec.zoneColor,emissive:spec.zoneColor,emissiveIntensity:.3,transparent:true,opacity:.86});
    for(const [x,z,width,depth]of [[0,-.04,.08,.006],[0,.04,.08,.006],[-.04,0,.006,.08],[.04,0,.006,.08]]){
      const bar=new THREE.Mesh(new THREE.BoxGeometry(width,.002,depth),zoneMaterial);bar.position.set(x,0,z);zone.add(bar);
    }
    return {...spec,start,drop,rotation:new THREE.Quaternion(),mesh,visualMesh,zone};
  });
}

function pipeTargets(layout,config,group){
  for(const target of layout.targets){
    const geometry=new THREE.CylinderGeometry(target.halfX,target.halfX,target.h*2,32),material=new THREE.MeshStandardMaterial({color:target.color,metalness:.15,roughness:.35});
    const mesh=new THREE.Mesh(geometry,material),visualMesh=mesh.clone();mesh.position.copy(target.start);visualMesh.position.copy(target.start);visualMesh.castShadow=true;visualMesh.receiveShadow=true;
    Object.assign(visualMesh.userData,{taskId:target.id,robotId:config.id});visualMesh.name=`${config.id}-${target.id}-task-object`;group.add(visualMesh);Object.assign(target,{mesh,visualMesh});
  }
  for(const bin of layout.bins){
    const zone=new THREE.Group();zone.name=`${config.id}-${bin.id}-pipe-placement-frame`;zone.position.copy(bin.center);group.add(zone);
    const material=new THREE.MeshStandardMaterial({color:bin.color,emissive:bin.color,emissiveIntensity:.3,transparent:true,opacity:.86});
    for(const [x,z,width,depth]of [[0,-bin.halfZ,bin.halfX*2,.005],[0,bin.halfZ,bin.halfX*2,.005],[-bin.halfX,0,.005,bin.halfZ*2],[bin.halfX,0,.005,bin.halfZ*2]]){
      const bar=new THREE.Mesh(new THREE.BoxGeometry(width,.002,depth),material);bar.position.set(x,0,z);zone.add(bar);
    }
    bin.zone=zone;for(const target of layout.targets.filter(t=>t.colorKey===bin.id))target.zone=zone;
  }
  return layout.targets;
}

function disposeTaskTargets(targets){
  const geometries=new Set(),materials=new Set(),nodes=new Set();
  for(const target of targets)for(const node of [target.mesh,target.visualMesh,target.zone])if(node)nodes.add(node);
  for(const root of nodes){root.removeFromParent();root.traverse(node=>{if(node.geometry)geometries.add(node.geometry);for(const material of(Array.isArray(node.material)?node.material:[node.material]))if(material)materials.add(material);});}
  geometries.forEach(geometry=>geometry.dispose());materials.forEach(material=>material.dispose());
}

export function objectExtents(target,quaternion){
  const m=new THREE.Matrix4().makeRotationFromQuaternion(quaternion).elements;
  if(target.id==='yellow'||target.shape==='cylinder')return [0,1,2].map(k=>target.halfX*Math.sqrt(Math.max(0,1-m[4+k]*m[4+k]))+target.h*Math.abs(m[4+k]));
  return [0,1,2].map(k=>Math.abs(m[k])*target.halfX+Math.abs(m[4+k])*target.h+Math.abs(m[8+k])*target.halfZ);
}

const phases=['spread','crouch','align_clear','align','approach','descend','closing','lift','transfer','place','release','retreat','settling','return'];
function progressOf(controller){
  if(controller.state==='complete')return 1;
  if(['idle','failed','stopped'].includes(controller.state))return 0;
  const index=phases.indexOf(controller.state),part=index<0?0:(index+.5)/phases.length;
  if(controller.task==='stack')return (controller.reaction||controller.state==='reaction_home') ? .95 : Math.min(.94,(controller.itemIndex+part)/3);
  return part;
}

/**
 * The canonical robot and target meshes NEVER receive station transforms or a
 * scene parent. Controllers solve in their original metre-scale local frame.
 * Only an independent URDF clone is translated for display. Physics and actual
 * contacts remain identical to the validated single-robot applications.
 */
export function createRobotRuntimeFromParts({id,api,model,data,robot,offset=[0,0,0],visualRobot=null,sourceXml=null,modelXml=sourceXml,cooperativePlanning=false,backgroundPlanning=false,planningTransportFactory=createPlanningWorkerTransport}){
  const config=configFor(id);
  if(robot.parent)throw new Error('物理机器人必须保留独立局部坐标，不能挂到可视场景');
  const group=new THREE.Group();group.name=`${id}-station`;group.position.fromArray(offset);
  let planningTransport=null;
  let activeModelXml=modelXml,modelPipeSeed=null;
  const transport=()=>planningTransport??=(planningTransportFactory());
  const deferRobotSync=instance=>{
    // MuJoCo qpos/qvel and every 1 ms motor/contact update remain authoritative.
    // This detached Three tree is only needed by FK/IK and display reads; the
    // single-robot dynamics classes keep their original afterStep behavior.
    instance.afterStep=()=>{};return instance;
  };
  const createController=(Controller,nextModel,nextData,nextTargets,nextDynamics)=>{
    const CooperativeController=planningControllerClass(Controller,{robotId:id});
    const PlanningController=backgroundPlanningControllerClass(CooperativeController,{robotId:id,transport,
      captureJob:request=>capturePlanningJob(runtime,request),isCurrent:job=>planningJobStillCurrent(runtime,job)});
    const instance=new PlanningController(api,nextModel,nextData,robot,nextTargets,nextDynamics);
    instance.cooperativePlanning=cooperativePlanning;
    instance.backgroundPlanning=backgroundPlanning;
    // wristError temporarily evaluates desired FK, then restores actual FK.
    // Synchronize its initial read too now that rendering owns routine sync.
    const wristError=instance.wristError.bind(instance);
    instance.wristError=(...args)=>{nextDynamics.syncRobot();return wristError(...args);};
    return instance;
  };
  let targets=taskTargets(config,group),dynamics=deferRobotSync(new config.Dynamics(api,model,data,robot));
  let controller=createController(config.Controller,model,data,targets,dynamics),taskMode='legacy',pipeLayout=null;
  visualRobot??=robot.clone(true);group.add(visualRobot);visualRobot.name=`${id}-visible-robot`;
  const sourceNodes=[],displayNodes=[];
  if(!visualRobot.setJointValues){robot.traverse(node=>sourceNodes.push(node));visualRobot.traverse(node=>displayNodes.push(node));if(sourceNodes.length!==displayNodes.length)throw new Error('可视镜像结构与物理机器人不匹配');}
  let disposed=false;
  const runtime={
    id,config,api,model,data,robot,visualRobot,group,targets,controller,dynamics,ready:true,
    get sourceXml(){return sourceXml;},get modelXml(){return activeModelXml;},get modelPipeSeed(){return modelPipeSeed;},get backgroundPlanning(){return backgroundPlanning;},
    get planningProgress(){return controller.planningProgress??null;},
    get taskMode(){return taskMode;},
    get planning(){return Boolean(controller.planning);},
    get solidMeshes(){return targets.map(target=>target.visualMesh);},
    get selected(){return controller.selected;},get running(){return controller.running;},get arm(){return controller.arm;},
    get cameraLink(){return visualRobot.getObjectByName((config.cameraFrame??config.gripperLink)(controller.arm));},
    get status(){
      return {id,state:controller.state,running:controller.running,planning:Boolean(controller.planning),time:controller.time,physicsTime:data.time,
        message:controller.message,task:taskMode==='pipes'?'pipes':controller.task,planningProgress:runtime.planningProgress,
        classification:taskMode==='pipes'?{started:Boolean(controller.classification.started)}:null};
    },
    get snapshot(){
      const measurement=controller.measure();
      return {id,label:config.label,state:controller.state,running:controller.running,planning:Boolean(controller.planning),task:taskMode==='pipes'?'pipes':controller.task,
        selected:controller.selected,arm:controller.arm,message:controller.message,progress:progressOf(controller),planningProgress:runtime.planningProgress,
        time:controller.time,physicsTime:data.time,evidence:{...controller.evidence},history:[...controller.history],
        force:measurement.force,forces:[...measurement.forces],contactCount:measurement.count,
        distance:measurement.distance,wristError:controller.wristError(),planError:controller.planError||null,
        objects:targets.map(target=>{const pose=controller.objectPose(target.id);return {id:target.id,position:pose.position.toArray(),quaternion:pose.quaternion.toArray(),
          worldPosition:runtime.localToWorld(pose.position).toArray()};}),
        telemetry:dynamics.telemetry(controller.arm),offset:group.position.toArray(),
        classification:taskMode==='pipes'?{...controller.classification,items:controller.classification.items.map(item=>({...item})),armSequence:[...controller.classification.armSequence],queue:[...controller.classification.queue],
          bins:controller.classification.bins.map(bin=>({id:bin.id,colorKey:bin.colorKey,label:bin.label,color:bin.color,center:bin.center.toArray(),halfX:bin.halfX,halfZ:bin.halfZ,slots:bin.slots.map(slot=>slot.toArray())}))}:null};
    },
    configureTask(task,{seed}={}){
      if(!disposed)controller.cancelPlanning?.('工作台已切换，路径检查已取消');
      if(disposed||controller.running)return{ok:false,reason:'任务执行中不能切换工作台'};
      const mode=task==='pipes'?'pipes':'legacy';
      if(mode===taskMode){runtime.reset({seed:mode==='pipes'?seed??pipeLayout.seed:undefined});return{ok:true,task:mode,seed:pipeLayout?.seed};}
      if(!sourceXml)return{ok:false,reason:'缺少原始物理场景，不能切换任务'};
      let nextModel,nextData,nextDynamics,nextTargets;
      const vfs=new api.MjVFS();
      try{
        const layout=mode==='pipes'?generatePipeLayout(config,{seed:seed??1}):null;
        const nextModelXml=layout?buildPipeScene(sourceXml,layout):sourceXml;
        nextModel=api.MjModel.from_xml_string(nextModelXml,vfs);nextData=new api.MjData(nextModel);
        nextTargets=layout?pipeTargets(layout,config,group):taskTargets(config,group);
        const Dynamics=layout?pipeDynamicsClass(config.Dynamics,{robotId:id}):config.Dynamics;
        nextDynamics=deferRobotSync(new Dynamics(api,nextModel,nextData,robot));
        const Controller=layout?pipeControllerClass(config.Controller,{robotId:id}):config.Controller;
        const nextController=createController(Controller,nextModel,nextData,nextTargets,nextDynamics);
        if(layout)nextController.setLayout(layout);nextController.arm=config.defaultArm;
        disposeTaskTargets(targets);dynamics.dispose?.();data.delete();model.delete();
        model=nextModel;data=nextData;dynamics=nextDynamics;controller=nextController;targets=nextTargets;taskMode=mode;pipeLayout=layout;activeModelXml=nextModelXml;modelPipeSeed=layout?.seed??null;
        Object.assign(runtime,{model,data,dynamics,controller,targets});runtime.syncVisuals();
        return{ok:true,task:mode,seed:layout?.seed};
      }catch(error){if(nextTargets)disposeTaskTargets(nextTargets);nextDynamics?.dispose?.();nextData?.delete();nextModel?.delete();dynamics.syncRobot();return{ok:false,reason:error.message};}
      finally{vfs.delete();}
    },
    start({objectId=controller.selected||'blue',task='sort',arm=config.defaultArm,observerYaw=-.8,reducedMotion=false}={}){
      if(disposed||controller.running)return false;
      if(task==='pipes'){
        if(taskMode!=='pipes'){controller.message='请先切换到圆管分类工作台';return false;}
        const accepted=controller.startClassification();runtime.syncVisuals();return accepted;
      }
      if(taskMode!=='legacy'){controller.message='请先切换到对应的标准抓取工作台';return false;}
      if(!targets.some(target=>target.id===objectId)){controller.message='请选择有效的桌面物体';return false;}
      const side=id==='c5'?'left':arm;if(!['left','right'].includes(side))return false;
      const accepted=task==='stack'?controller.startStack(side,{observerYaw,reducedMotion}):controller.start(side,objectId);
      runtime.syncVisuals();return accepted;
    },
    advancePlanning(){
      if(disposed)return{pending:false,done:true,ok:false,reason:'机器人已关闭'};
      const result=controller.advancePlanning?.()??{pending:false,done:true,ok:true};
      runtime.syncVisuals();return result;
    },
    stop(){
      if(disposed)return;
      controller.cancelPlanning?.('已停止路径检查');
      if(id==='r1'){
        // The legacy single-R1 stop command requests a full opening step.
        // In a synchronized stop keep its present, already-ramped command;
        // neither finger nor arm targets may jump during a batch abort.
        const opening=controller.opening;
        controller.stop('已停止并保持当前夹爪开口');controller.opening=opening;
      }else controller.stop();
      runtime.syncVisuals();
    },
    reset({seed}={}){
      if(disposed)return;
      controller.cancelPlanning?.('已复位，路径检查已取消');
      if(taskMode==='pipes'){
        const layout=generatePipeLayout(config,{seed:seed??normalizePipeSeed(pipeLayout.seed+1)});
        for(const target of targets){const source=layout.targets.find(t=>t.id===target.id);target.start.copy(source.start);target.drop.copy(source.drop);target.rotation.copy(source.rotation);}
        layout.targets=targets;pipeLayout=layout;controller.setLayout(layout);
      }else controller.reset();runtime.syncVisuals();
    },
    select(objectId){if(disposed||controller.running||!targets.some(target=>target.id===objectId))return false;controller.selected=objectId;return true;},
    setArm(arm){if(disposed||controller.running||taskMode==='pipes'||!['left','right'].includes(arm))return false;if(id==='c5')return arm==='left';if(controller.selectArm&&!controller.selectArm(arm))return false;controller.arm=arm;return true;},
    step(dt=.001){if(disposed)return;if(Math.abs(dt-.001)>1e-10)throw new Error('机器人需要使用统一的 1 ms 物理时钟');controller.step(dt);},
    syncVisuals(){
      if(disposed)return;
      const actual=dynamics.syncRobot();
      if(visualRobot.setJointValues)visualRobot.setJointValues(actual);
      else sourceNodes.forEach((node,index)=>{displayNodes[index].position.copy(node.position);displayNodes[index].quaternion.copy(node.quaternion);displayNodes[index].scale.copy(node.scale);});
      for(const target of targets){const pose=controller.objectPose(target.id);target.mesh.position.copy(pose.position);target.mesh.quaternion.copy(pose.quaternion);target.visualMesh.position.copy(pose.position);target.visualMesh.quaternion.copy(pose.quaternion);}
      // The renderer refreshes its visible scene once per frame. World-space
      // queries (camera links, bounds and picking) refresh their own ancestors.
    },
    animate(now,{reducedMotion=false}={}){
      const pulse=reducedMotion ? .5 : .5+.5*Math.sin(now*.006);
      for(const target of targets){const material=target.visualMesh.material;material.emissive.set(target.color).multiplyScalar(.1);material.emissiveIntensity=.18;
        if(target.id===controller.selected&&!controller.running){material.emissive.lerp(new THREE.Color(0x5ee0ff),.38+pulse*.28);material.emissiveIntensity=.42+pulse*.72;}}
    },
    localToWorld(position){group.updateWorldMatrix(true,false);return group.localToWorld(position.clone());},
    worldToLocal(position){group.updateWorldMatrix(true,false);return group.worldToLocal(position.clone());},
    tcpWorld(){dynamics.syncRobot();return runtime.localToWorld(config.tcp(robot,controller.arm).position);},
    moveObject(objectId,position,quaternion){
      if(disposed||controller.running)return false;
      if(taskMode==='pipes'&&controller.classification.started){controller.message='请先复位或重新随机摆放，再编辑圆管';return false;}
      const target=targets.find(item=>item.id===objectId);if(!target)return false;
      const q=quaternion?.clone()||target.mesh.quaternion.clone();
      if(![position.x,position.y,position.z,q.x,q.y,q.z,q.w].every(Number.isFinite)||Math.abs(q.lengthSq()-1)>.0001)return false;
      const e=objectExtents(target,q),p=position.clone();
      p.x=THREE.MathUtils.clamp(p.x,config.tableCenter-config.tableHalfX+e[0]+.008,config.tableCenter+config.tableHalfX-e[0]-.008);
      p.z=THREE.MathUtils.clamp(p.z,-config.tableHalfZ+e[2]+.008,config.tableHalfZ-e[2]-.008);p.y=config.tableHeight+e[1]+.0001;
      if(targets.some(other=>{if(other===target)return false;const h=objectExtents(other,other.mesh.quaternion);return Math.abs(p.x-other.mesh.position.x)<e[0]+h[0]+.006&&Math.abs(p.z-other.mesh.position.z)<e[2]+h[2]+.006;})){controller.message='该位置与其他物体重叠，请移开一些';return false;}
      if(controller.setObject(objectId,p,q)===false)return false;runtime.syncVisuals();return true;
    },
    dispose(){
      if(disposed)return;controller.cancelPlanning?.('机器人已关闭，路径检查已取消');planningTransport?.dispose();disposed=true;runtime.ready=false;group.removeFromParent();dynamics.dispose?.();data.delete();model.delete();
      const geometries=new Set(),materials=new Set();group.traverse(node=>{if(node.geometry)geometries.add(node.geometry);for(const material of(Array.isArray(node.material)?node.material:[node.material]))if(material)materials.add(material);});
      geometries.forEach(geometry=>geometry.dispose());materials.forEach(material=>material.dispose());
    },
  };
  controller.arm=config.defaultArm;runtime.syncVisuals();return runtime;
}

export async function createRobotRuntime({id,api,offset=[0,0,0],onProgress,cooperativePlanning=false,backgroundPlanning=false}={}){
  const config=configFor(id);
  const [robot,physics,xml]=await Promise.all([loadRobot(config,onProgress),api||loadPhysicsAPI({onProgress}),
    fetch(config.xml).then(async response=>{if(!response.ok)throw new Error(`${config.label} 物理场景加载失败（${response.status}）`);return response.text();})]);
  const vfs=new physics.MjVFS();let model,data;
  try{model=physics.MjModel.from_xml_string(xml,vfs);data=new physics.MjData(model);
    const runtime=createRobotRuntimeFromParts({id,api:physics,model,data,robot,offset,sourceXml:xml,cooperativePlanning,backgroundPlanning});
    onProgress?.({id,part:'ready',value:1,label:`${config.label} 已就绪`});return runtime;
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
