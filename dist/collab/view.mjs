import * as THREE from '../vendor/three.module.min.js';
import {OrbitControls} from '../vendor/OrbitControls.js';
import URDFLoader from '../vendor/URDFLoader.js';
import {VISUAL_CONFIGS as ROBOT_CONFIGS} from './visual-config.mjs';
import {loadC5Mesh} from '../c5/c5-3ds.mjs';
import {createVisualMeshLoader} from './visual-mesh-loader.mjs?v=20261026-load-2';
import {createTrainingHall} from './training-hall.mjs';

const zup=([x,y,z])=>[x,z,-y];
const partColors={valve:0x4d86ae,pin:0xbd4f49,sleeve:0xd4a831};

export function applyRobotBasePose(group,pose){
  const valid=pose?.position?.length===3&&pose?.quaternion?.length===4
    &&[...pose.position,...pose.quaternion].every(Number.isFinite)
    &&Math.abs(pose.quaternion.reduce((sum,value)=>sum+value*value,0)-1)<1e-4;
  group.visible=Boolean(valid);if(!valid)return false;
  group.position.fromArray(pose.position);group.quaternion.fromArray(pose.quaternion);return true;
}

/** Presentation only: every moving pose comes from the one shared simulation. */
export function createCollaborationView({canvas,container}){
  const scene=new THREE.Scene();scene.background=new THREE.Color(0x344956);scene.fog=new THREE.Fog(0x344956,8,18);
  const renderer=new THREE.WebGLRenderer({canvas,antialias:true,powerPreference:'high-performance'});
  renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio??1,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.02;
  renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;renderer.shadowMap.autoUpdate=false;renderer.shadowMap.needsUpdate=true;
  const camera=new THREE.PerspectiveCamera(40,1,.025,40),orbit=new OrbitControls(camera,canvas);
  orbit.enableDamping=true;orbit.dampingFactor=.10;orbit.minDistance=.4;orbit.maxDistance=12;orbit.maxPolarAngle=Math.PI*.49;
  camera.position.set(3.8,2.55,3.15);orbit.target.set(.7,.8,0);orbit.update();
  const geometries=new Set(),materials=new Set(),textures=new Set(),robots=new Map(),parts=new Map(),trays=new Map(),bins=new Map(),basePoses=new Map();
  const meshLoader=createVisualMeshLoader({onGeometry:g=>geometries.add(g)});
  let metadata=null,disposed=false,firstFit=false,lastShadowPose='',robotLoadPromise=null,signalMaterial=null;
  const geometry=g=>(geometries.add(g),g);
  const material=(color,metalness=.3,roughness=.5)=>{const m=new THREE.MeshStandardMaterial({color,metalness,roughness});materials.add(m);return m;};
  const box=(parent,name,size,position,mat,{shadow=true}={})=>{
    const mesh=new THREE.Mesh(geometry(new THREE.BoxGeometry(...size)),mat);mesh.name=name;mesh.position.fromArray(position);
    mesh.castShadow=shadow;mesh.receiveShadow=true;parent.add(mesh);return mesh;
  };
  const metal=material(0x879ba7,.6,.43),dark=material(0x263d4b,.35,.57),table=material(0x627b89,.38,.48),amber=material(0xd5aa5e,.25,.52),mint=material(0x6cbbae,.25,.5);
  const floorMaterial=material(0x455a67,.18,.74);
  const floor=new THREE.Mesh(geometry(new THREE.PlaneGeometry(16,16)),floorMaterial);floor.rotation.x=-Math.PI/2;floor.position.y=-.012;floor.receiveShadow=true;scene.add(floor);
  const architecture=new THREE.Group();architecture.name='industrial-workcell-architecture';scene.add(architecture);
  const trainingHall=createTrainingHall({parent:architecture,box,geometry,material,materials,textures});
  // Keep the active cell open; surrounding architecture is presentation only.
  const diffuser=new THREE.MeshBasicMaterial({color:0xf4f7f8});materials.add(diffuser);
  // A single suspended task-light spine replaces the scattered fixtures.
  box(architecture,'task-light-housing',[3.6,.055,.20],[1.45,2.72,.35],dark);
  box(architecture,'task-light-diffuser',[3.48,.012,.15],[1.45,2.686,.35],diffuser,{shadow:false});
  for(const x of[0,2.9])box(architecture,'task-light-suspension',[.018,.45,.018],[x,2.97,.35],metal);
  // Keep the room a stop quieter than the robots. The key and all fixture
  // targets point vertically down; the moving base remains in a broad pool.
  scene.add(new THREE.HemisphereLight(0xeaf4ff,0x58616b,.95),new THREE.AmbientLight(0xffffff,.18));
  const key=new THREE.DirectionalLight(0xfffbf4,1.65);key.position.set(1.45,5,.1);key.target.position.set(1.45,.7,.1);key.castShadow=true;
  key.shadow.mapSize.set(2048,2048);key.shadow.camera.left=-2.5;key.shadow.camera.right=2.5;key.shadow.camera.top=2.7;key.shadow.camera.bottom=-2.7;key.shadow.camera.near=.1;key.shadow.camera.far=10;key.shadow.bias=-.00015;key.shadow.normalBias=.012;scene.add(key,key.target);
  const fill=new THREE.DirectionalLight(0xbad9f1,.55);fill.position.set(2,2,-3);scene.add(fill);
  const rim=new THREE.DirectionalLight(0xffdba7,.45);rim.position.set(-1,2,1);scene.add(rim);
  for(const[id,x,z,angle,power]of[['g1',0,.65,.68,15],['r1-aisle',1.45,-.85,.98,18],['c5',2.35,.65,.68,15]]){
    box(architecture,`${id}-light-feed`,[.045,.045,Math.abs(z-.35)+.18],[x,2.72,(z+.35)/2],dark,{shadow:false});
    box(architecture,`${id}-downlight-housing`,[.34,.08,.25],[x,2.69,z],dark,{shadow:false});
    box(architecture,`${id}-downlight-diffuser`,[.28,.012,.19],[x,2.643,z],diffuser,{shadow:false});
    const light=new THREE.SpotLight(0xfffcf6,power,6,angle,.65,2);light.name=`${id}-vertical-task-light`;
    light.position.set(x,2.62,z);light.target.position.set(x,.3,z);scene.add(light,light.target);
    // Reuse the single overhead shadow map rather than render three more maps
    // per physics frame. These real spotlights supply the local light falloff.
  }
  // Code-native neutral softboxes give original robot metal readable reflections.
  const envScene=new THREE.Scene();envScene.background=new THREE.Color(.20,.25,.30);
  const envGeometries=[],envMaterials=[];
  for(const[p,size,intensity]of[[[0,4,0],[5,.1,4],2.4],[[3,1,0],[.1,4,4],1.1],[[-3,1,0],[.1,3,4],.65]]){
    const g=new THREE.BoxGeometry(...size),m=new THREE.MeshBasicMaterial({color:new THREE.Color(intensity,intensity,intensity)}),mesh=new THREE.Mesh(g,m);mesh.position.fromArray(p);envScene.add(mesh);envGeometries.push(g);envMaterials.push(m);
  }
  const pmrem=new THREE.PMREMGenerator(renderer),environment=pmrem.fromScene(envScene,.025);scene.environment=environment.texture;scene.environmentIntensity=.8;
  pmrem.dispose();envGeometries.forEach(g=>g.dispose());envMaterials.forEach(m=>m.dispose());
  const physical=new THREE.Group();physical.name='shared-physical-workcell';scene.add(physical);

  function faceLabel(parent,text,width,position,back=false){
    const canvas=document.createElement('canvas');canvas.width=768;canvas.height=128;
    const context=canvas.getContext('2d');context.fillStyle='#253d4b';context.fillRect(0,0,768,128);
    context.fillStyle='#e6eff2';context.font='500 96px "PingFang SC", "Microsoft YaHei", sans-serif';context.textAlign='center';context.textBaseline='middle';context.fillText(text,384,66);
    const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;textures.add(texture);
    const mat=new THREE.MeshBasicMaterial({map:texture});materials.add(mat);
    const label=new THREE.Mesh(geometry(new THREE.PlaneGeometry(width,width/6)),mat);label.position.fromArray(position);if(back)label.rotation.y=Math.PI;parent.add(label);
    return label;
  }

  function tableAt(name,center,half,height){
    const group=new THREE.Group();group.name=name;physical.add(group);
    box(group,`${name}-top`,[half[0]*2,half[2]*2,half[1]*2],zup(center),table);
    if(!metadata.fixtures)for(const x of[-1,1])for(const z of[-1,1]){
      const heightToTop=height-.065;
      box(group,`${name}-leg`,[.05,heightToTop,.05],[center[0]+x*(half[0]-.045),heightToTop/2,-center[1]+z*(half[1]-.045)],metal);
    }
    return group;
  }
  function setMetadata(value){
    if(!value||metadata)return;
    if(value.bins&&!['wall_rack_back','wall_rack_foot'].every(name=>value.rackStructure?.some(g=>g.name===name)))throw new Error('料架支撑数据未完整加载，请刷新页面更新场景。');
    metadata=value;
    const stationSpecs=value.supports??value.stations??{shared:value.workbench};
    for(const[id,station]of Object.entries(stationSpecs))if(station?.center&&station.halfExtents)tableAt(station.body??`${id}-workbench`,station.center,station.halfExtents,station.tableHeight??value.tableHeight);
    for(const output of[value.conveyor,value.queueBed,value.outputBed])if(output?.center&&output.halfExtents)tableAt(output.body,output.center,output.halfExtents,output.tableHeight??value.tableHeight);
    const platform=value.mobility?.platform;
    if(platform?.center&&platform.halfExtents)box(physical,platform.body??'r1-connected-mobile-platform',[platform.halfExtents[0]*2,platform.halfExtents[2]*2,platform.halfExtents[1]*2],zup(platform.center),floorMaterial);
    drawMobility(value.mobility?.aisleY!==undefined?{...value.mobility,route:[[.38,value.mobility.aisleY,.208],[2.93,value.mobility.aisleY,.208]]}:value.mobility);drawDocks(value);
    if(value.bins){
      for(const fixture of value.fixtures??[])box(physical,fixture.name,[fixture.half[0]*2,fixture.half[2]*2,fixture.half[1]*2],zup(fixture.position),dark);
      for(const base of value.plinths??[])box(physical,base.name,[base.half[0]*2,base.half[2]*2,base.half[1]*2],zup(base.position),dark);
      const captions={g1_out_bench:'取料交接',empty_return_bench:'空盒回收',c5_input_bench:'分拣交接',c5_output_bench:'分类出料'};
      for(const s of value.supports){
        if(!captions[s.body])continue;
        // Flush cabinet signs with twice the previous text size.
        const front=-s.center[1]+s.halfExtents[1]-.034;
        faceLabel(physical,captions[s.body]??s.body,Math.min(.45,s.halfExtents[0]*1.5),[s.center[0],.58,front]);
        faceLabel(physical,captions[s.body],Math.min(.45,s.halfExtents[0]*1.5),[s.center[0],.58,-s.center[1]-s.halfExtents[1]+.034],true);
      }
      // Readable aluminum shelves and backing, with the original grounded foot.
      for(const shelf of value.rack??[])box(physical,shelf.geom,[shelf.halfExtents[0]*2,shelf.halfExtents[2]*2,shelf.halfExtents[1]*2],zup(shelf.center),metal);
      for(const g of value.rackStructure??[])box(physical,g.name,[g.half[0]*2,g.half[2]*2,g.half[1]*2],zup(g.position),g.name==='wall_rack_back'?table:dark);
      canvas.dataset.rackSupports=physical.children.filter(m=>['wall_rack_back','wall_rack_foot'].includes(m.name)&&m.visible).map(m=>m.name).join(' ');
      canvas.dataset.modelVersion=value.version;
      const carrierMaterials=new Map();
      for(const spec of value.bins){
        const group=new THREE.Group();group.name=spec.body;group.position.fromArray(zup(spec.start));physical.add(group);
        for(const geom of spec.geometry){
          if(!carrierMaterials.has(geom.color)){const [r,g,b]=geom.color.split(' ').map(Number);carrierMaterials.set(geom.color,material(new THREE.Color(r,g,b),.35,.5));}
          box(group,geom.name,[geom.half[0]*2,geom.half[2]*2,geom.half[1]*2],zup(geom.position),carrierMaterials.get(geom.color));
        }
        bins.set(spec.id,group);
      }
      for(const slot of value.slots){
        const [x,y,z]=slot.position,output=slot.kind==='output',a=output?.09:.087,b=output?.065:.084,h=value.tableHeight;
        if(output)box(physical,`fixture_${slot.id}_floor`,[.18,.006,.13],[x,h+.003,-y],metal);
        const ink=output?material(slot.color,.05,.6):amber;
        for(const dx of[-a,a])box(physical,`${slot.id}-outline`,[.004,.001,b*2],[x+dx,h+(output?.0065:.0005),-y],ink,{shadow:false});
        if(output){
          for(const dy of[-b,b])box(physical,`${slot.id}-outline`,[a*2,.001,.004],[x,h+.0065,-y+dy],ink,{shadow:false});
          const label=faceLabel(physical,slot.label,.16,[x,h+.0072,-y+b-.017]);label.rotation.x=-Math.PI/2;
        }
      }
    }
    const c5=value.robots?.c5?.origin;if(c5){const h=.72+c5[2],pedestal=new THREE.Mesh(geometry(new THREE.CylinderGeometry(.12,.12,h,40)),metal);pedestal.position.set(c5[0],h/2,-c5[1]);pedestal.castShadow=pedestal.receiveShadow=true;physical.add(pedestal);}
    for(const spec of value.parts??[]){
      const [a,b,h]=spec.halfExtents,g=spec.shape==='cylinder'?new THREE.CylinderGeometry(a,a,h*2,28):new THREE.BoxGeometry(a*2,h*2,b*2);
      const mesh=new THREE.Mesh(geometry(g),material(spec.color??partColors[spec.kind]??0x8fa9b8,.28,.43));mesh.name=spec.id;mesh.position.fromArray(zup(spec.start));mesh.castShadow=mesh.receiveShadow=true;physical.add(mesh);parts.set(spec.id,mesh);
    }
    for(const spec of value.trays??[]){
      const group=new THREE.Group();group.name=spec.body;group.position.fromArray(zup(spec.initialPosition??spec.center));physical.add(group);
      const [a,b,h]=spec.halfExtents;box(group,'tray-floor',[a*2,h*2,b*2],[0,0,0],dark);
      for(const[x,z,w,d]of[[0,-b,a*2,.008],[0,b,a*2,.008],[-a,0,.008,b*2],[a,0,.008,b*2]])box(group,'tray-rim',[w,.012,d],[x,.006,z],metal);
      for(const slot of spec.slots??[]){const[x,y,z]=slot.offset;const radius=slot.kind==='valve'?.025:.020;for(const[dx,dz,w,d]of[[0,-radius,radius*2,.002],[0,radius,radius*2,.002],[-radius,0,.002,radius*2],[radius,0,.002,radius*2]])box(group,'tray-slot-outline',[w,.001,d],[x+dx,.0045,-y+dz],amber,{shadow:false});}
      const staging=spec.staging??value.staging;
      if(staging?.offset){
        // A painted landing marker only. The free workpiece is held by real
        // tray contact; no socket, weld, capture constraint or fake attachment.
        const ring=new THREE.Mesh(geometry(new THREE.RingGeometry(.0165,.020,32)),mint);ring.name=`${spec.body}-staging-outline`;
        ring.rotation.x=-Math.PI/2;ring.position.set(staging.offset[0],h+.0005,-staging.offset[1]);ring.receiveShadow=true;group.add(ring);
      }
      trays.set(spec.orderId,group);trays.set(spec.body,group);
    }
    const signal=new THREE.Group();signal.position.set(3.04,.54,.327);physical.add(signal);
    signalMaterial=material(0x6b7e88,.15,.4);
    // Flush status lamp on the cabinet, outside both the aisle and arm path.
    const lamp=new THREE.Mesh(geometry(new THREE.CircleGeometry(.013,20)),signalMaterial);signal.add(lamp);
    renderer.shadowMap.needsUpdate=true;
    if(!firstFit){fitView();firstFit=true;}
  }
  function drawMobility(mobility){
    const route=mobility?.route??mobility?.waypoints??[];
    const points=route.map(point=>Array.isArray(point)?point:point.mountPosition??point.position??point.origin).filter(point=>point?.length===3);
    const top=mobility?.platform?.topHeight??Math.max(0,...points.map(point=>point[2]));
    for(let i=1;i<points.length;i++){
      const a=new THREE.Vector3(...zup(points[i-1])),b=new THREE.Vector3(...zup(points[i])),delta=b.clone().sub(a),length=Math.hypot(delta.x,delta.z);
      if(length<1e-6)continue;
      const angle=-Math.atan2(delta.z,delta.x),normal=new THREE.Vector3(-delta.z,0,delta.x).normalize();
      for(const side of[-1,1]){const mark=box(physical,'r1-fixed-route',[length,.002,.012],[(a.x+b.x)/2+normal.x*.36*side,top+.002,(a.z+b.z)/2+normal.z*.36*side],amber,{shadow:false});mark.rotation.y=angle;}
    }
    // No intermediate transverse stripes: the aisle reads as one clear lane.
  }
  function drawDocks(value){
    const docks=value.trays?.[0]?.docks??value.docks??value.conveyor?.docks??{};
    for(const dock of['r1','g1','c5'].map(id=>docks[id]).filter(Boolean)){
      const position=Array.isArray(dock)?dock:dock.position??dock.center;if(position?.length!==3)continue;
      const[x,y,z]=zup(position),a=.14,b=.14;
      for(const[dx,dz,w,d]of[[0,-b,a*2,.005],[0,b,a*2,.005],[-a,0,.005,b*2],[a,0,.005,b*2]])box(physical,'tray-dock-outline',[w,.002,d],[x+dx,y-.003,z+dz],amber,{shadow:false});
    }
  }
  async function loadRobot(config,onProgress){
    const manager=new THREE.LoadingManager();manager.onProgress=(_,loaded,total)=>onProgress?.({id:config.id,value:total?loaded/total:0,label:`加载${config.label} · ${loaded}/${total}`});
    const ready=new Promise((resolve,reject)=>{manager.onLoad=resolve;manager.onError=path=>reject(new Error(`${config.label}模型加载失败：${path.split('/').at(-1)}`));});
    const loader=new URDFLoader(manager);
    if(config.id==='r1')loader.packages={r1_v2_1_0:new URL('../r1/assets',import.meta.url).href};
    if(config.id==='c5'){const original=loader.defaultMeshLoader.bind(loader);loader.loadMeshCb=(path,lm,done)=>/\.3ds(?:[?#]|$)/i.test(path)?loadC5Mesh(path,lm,done):original(path,lm,done);}
    const original=loader.loadMeshCb.bind(loader);loader.loadMeshCb=(path,lm,done)=>{if(!meshLoader.load(path,lm,done))original(path,lm,done);};
    const [robot]=await Promise.all([loader.loadAsync(config.urdf),ready]);
    if(disposed){disposeTree(robot);return;}
    robot.rotation.x=-Math.PI/2;robot.position.y=config.rootHeight;config.applyMaterials(robot);
    robot.traverse(node=>{if(node.isMesh){node.castShadow=node.receiveShadow=true;}});
    const group=new THREE.Group();group.name=`${config.id}-shared-robot`;group.add(robot);scene.add(group);robots.set(config.id,{robot,group});applyRobotBasePose(group,basePoses.get(config.id));renderer.shadowMap.needsUpdate=true;
    return robot;
  }
  function loadRobots(onProgress){return robotLoadPromise??=(Promise.all(Object.values(ROBOT_CONFIGS).map(config=>loadRobot(config,onProgress))).catch(error=>{meshLoader.dispose();throw error;}).finally(()=>meshLoader.releaseWorkers()));}
  function sync(snapshot){
    if(!snapshot||disposed||snapshot===sync.lastSnapshot)return;setMetadata(snapshot.metadata);
    for(const row of snapshot.robots??[]){
      if(row.basePose)basePoses.set(row.id,row.basePose);
      const visual=robots.get(row.id);if(!visual)continue;
      applyRobotBasePose(visual.group,basePoses.get(row.id));
      if(row.pose)visual.robot.setJointValues(row.pose);
    }
    for(const part of snapshot.parts??[]){const mesh=parts.get(part.id);if(mesh&&part.position){mesh.position.fromArray(part.position);if(part.quaternion)mesh.quaternion.fromArray(part.quaternion);}}
    for(const bin of snapshot.bins??[]){const group=bins.get(bin.id);if(group&&bin.position){group.position.fromArray(bin.position);group.quaternion.fromArray(bin.quaternion);}}
    for(const tray of snapshot.trays??[]){const group=trays.get(tray.id??tray.orderId??tray.body);if(group&&tray.position){group.position.fromArray(tray.position);if(tray.quaternion)group.quaternion.fromArray(tray.quaternion);}}
    if(signalMaterial){const running=['running','draining','planning'].includes(snapshot.state);signalMaterial.color.set(running?0x70c6a4:snapshot.state==='failed'?0xdb8b87:0xd3b373);signalMaterial.emissive.copy(signalMaterial.color);signalMaterial.emissiveIntensity=running?.25:.08;}
    const physicalStamp=snapshot.simTime??snapshot.simulationTime??snapshot.physicsTime??snapshot.tick;
    // Shadow invalidation is tied to received physical updates, not orbit gestures.
    if(physicalStamp!==undefined&&physicalStamp!==lastShadowPose){lastShadowPose=physicalStamp;renderer.shadowMap.needsUpdate=true;}
    else if(physicalStamp===undefined&&snapshot!==sync.lastSnapshot){renderer.shadowMap.needsUpdate=true;}
    sync.lastSnapshot=snapshot;
  }
  function fitView(){
    clearViewMomentum();
    const bounds=fitBounds();
    const center=bounds.getCenter(new THREE.Vector3()),direction=new THREE.Vector3(-1.15,.8,-1.35).normalize();
    center.y+=.16;
    const right=new THREE.Vector3().crossVectors(new THREE.Vector3(0,1,0),direction).normalize(),up=new THREE.Vector3().crossVectors(direction,right).normalize(),tan=Math.tan(THREE.MathUtils.degToRad(camera.fov/2));
    const framing=[];
    for(const x of[bounds.min.x,bounds.max.x])for(const y of[bounds.min.y,bounds.max.y])for(const z of[bounds.min.z,bounds.max.z])framing.push(new THREE.Vector3(x,y,z));
    // Only the title joins the explicit panorama framing, not the entire hall.
    // Resizing or task progress never calls fitView or takes the user's camera.
    const banner=trainingHall.getObjectByName('training-hall-banner');
    for(const x of[-2.7,2.7])for(const y of[-.37,.37])framing.push(banner.localToWorld(new THREE.Vector3(x,y,0)));
    let distance=1;
    for(const p of framing){const point=p.sub(center),depth=point.dot(direction);distance=Math.max(distance,depth+Math.abs(point.dot(right))/(tan*camera.aspect*.94),depth+Math.abs(point.dot(up))/(tan*.84));}
    orbit.target.copy(center);camera.position.copy(center).addScaledVector(direction,distance);orbit.update();
  }
  function clearViewMomentum(){
    const damping=orbit.enableDamping;orbit.enableDamping=false;orbit.update();orbit.enableDamping=damping;
  }
  function handleCameraKey(key){
    if(disposed||!orbit.enabled)return false;
    if(key==='Home'){fitView();return true;}
    if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','=','-','_'].includes(key))return false;
    clearViewMomentum();
    const spherical=new THREE.Spherical().setFromVector3(camera.position.clone().sub(orbit.target)),angle=.075;
    if(key==='ArrowLeft')spherical.theta-=angle;
    if(key==='ArrowRight')spherical.theta+=angle;
    if(key==='ArrowUp')spherical.phi-=angle;
    if(key==='ArrowDown')spherical.phi+=angle;
    if(key==='+'||key==='=')spherical.radius/=1.12;
    if(key==='-'||key==='_')spherical.radius*=1.12;
    spherical.radius=THREE.MathUtils.clamp(spherical.radius,orbit.minDistance,orbit.maxDistance);
    spherical.phi=THREE.MathUtils.clamp(spherical.phi,Math.max(.0001,orbit.minPolarAngle),Math.min(Math.PI-.0001,orbit.maxPolarAngle));
    spherical.theta=THREE.MathUtils.clamp(spherical.theta,orbit.minAzimuthAngle,orbit.maxAzimuthAngle);
    camera.position.copy(orbit.target).add(new THREE.Vector3().setFromSpherical(spherical));orbit.update();return true;
  }
  function fitBounds(){
    const bounds=new THREE.Box3();
    const includeBox=(center,half)=>{
      if(!center||!half)return;const[x,y,z]=zup(center),[a,b,h]=half;
      bounds.expandByPoint(new THREE.Vector3(x-a,y-h,z-b));bounds.expandByPoint(new THREE.Vector3(x+a,y+h,z+b));
    };
    for(const station of Object.values(metadata?.supports??metadata?.stations??{}))includeBox(station.center,station.halfExtents);
    for(const shelf of metadata?.rack??[])includeBox(shelf.center,shelf.halfExtents);
    for(const support of[metadata?.conveyor,metadata?.queueBed,metadata?.outputBed,metadata?.mobility?.platform])includeBox(support?.center,support?.halfExtents);
    for(const[id,frame]of Object.entries(metadata?.robots??{})){
      const[x,y,z]=zup(frame.origin);const halfWidth=id==='r1'?.42:.39;
      bounds.expandByPoint(new THREE.Vector3(x-.33,0,z-halfWidth));bounds.expandByPoint(new THREE.Vector3(x+.55,y+(id==='g1'?1.46:1.83),z+halfWidth));
    }
    const route=metadata?.mobility?.route??metadata?.mobility?.waypoints??[];
    for(const point of route){const p=Array.isArray(point)?point:point.mountPosition??point.position??point.origin;if(!p)continue;const[x,y,z]=zup(p);bounds.expandByPoint(new THREE.Vector3(x-.42,0,z-.42));bounds.expandByPoint(new THREE.Vector3(x+.42,y+1.83,z+.42));}
    if(bounds.isEmpty())return new THREE.Box3(new THREE.Vector3(-.45,-.02,-2.05),new THREE.Vector3(1.6,2.1,1.35));
    bounds.min.y=-.02;return bounds;
  }
  function resize(){const w=Math.max(1,container.clientWidth),h=Math.max(1,container.clientHeight);renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
  function render(){if(disposed)return;orbit.update();renderer.render(scene,camera);}
  function disposeTree(root){root.traverse(node=>{if(node.geometry)geometries.add(node.geometry);for(const m of(Array.isArray(node.material)?node.material:[node.material]))if(m)materials.add(m);});root.removeFromParent();}
  function dispose(){if(disposed)return;disposed=true;meshLoader.dispose();for(const{group}of robots.values())disposeTree(group);orbit.dispose();environment.dispose();geometries.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());textures.forEach(t=>t.dispose());renderer.dispose();}
  resize();
  return{loadRobots,sync,resize,render,fitView,handleCameraKey,dispose,
    get rackGeometry(){return physical.children.filter(m=>m.name.startsWith('wall_')).map(m=>({name:m.name,visible:m.visible,position:m.position.toArray(),size:m.geometry?.parameters}));},
    get cameraPose(){return{position:camera.position.toArray(),target:orbit.target.toArray()};},
    get robotBasePoses(){return Object.fromEntries([...robots].map(([id,{group}])=>[id,{position:group.position.toArray(),quaternion:group.quaternion.toArray(),visible:group.visible}]));},
    get loadedRobots(){return[...robots.keys()];}};
}
