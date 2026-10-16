import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {relative,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import * as THREE from '../dist/vendor/three.module.min.js';
import {parseC5_3DS} from '../dist/c5/c5-3ds.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import * as workcell from '../dist/multi/workcell.mjs';
import {loadTestRobot as loadR1} from './r1-test-robot.mjs';
import {loadG1TestRobot as loadG1} from './g1-test-robot.mjs';
import {loadC5TestRobot as loadC5} from './c5-test-robot.mjs';
import * as R1 from '../dist/r1/grasp-controller.mjs';
import * as G1 from '../dist/g1/grasp-controller.mjs';
import {STAND_POSE as G1_STAND} from '../dist/g1/g1-profile.mjs';
import {HOME_POSE as C5_HOME} from '../dist/c5/c5-profile.mjs';
import {createWristVision as r1WristVision} from '../dist/r1/r1-workcell.mjs';
import {createWristVision as dexWristVision} from '../dist/g1/g1-workcell.mjs';

// Geometry and perspective mathematics only. Original URDF visual meshes are
// loaded from disk; no browser, GPU, contacts or physical simulation is mocked.
// Named source poses are not a claim about every possible joint configuration.
const root=new URL('../',import.meta.url),rootPath=fileURLToPath(root),ids=['r1','g1','c5'];
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)??'work/validation/multi-framing/final.json';
const paths=new Set(['work/test-multi-framing.mjs','dist/multi/workcell.mjs','dist/multi/app.mjs',
  'dist/multi/robot-runtime.mjs','dist/multi/styles.css',
  'dist/multi/visuals/theme-controller.mjs','dist/multi/visuals/scene-environment.mjs','dist/multi/visuals/camera-feeds.mjs','dist/vendor/three.module.min.js',
  'dist/vendor/three.core.min.js','dist/vendor/STLLoader.js','dist/c5/c5-3ds.mjs',
  'dist/r1/r1-workcell.mjs','dist/g1/g1-workcell.mjs',
  ...ids.flatMap(id=>[`work/${id}-test-robot.mjs`,`dist/${id}/grasp-controller.mjs`]),
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs']);
const sourceHashes={},checks=[],errors=[],geometry={},projections=[],historicalMagnifications=[],currentPoseMagnifications=[],idleFramingGains=[];
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
async function boundBytes(path){
  const bytes=await readFile(new URL(path,root));
  if(!Object.hasOwn(sourceHashes,path))sourceHashes[path]=sha(bytes);
  paths.add(path);return bytes;
}
async function check(name,fn){try{const detail=await fn();checks.push({name,pass:true,...(detail===undefined?{}:{detail})});}
  catch(error){checks.push({name,pass:false,error:error.message});errors.push({name,message:error.message});}}
const near=(a,b,message,epsilon=1e-7)=>assert(Math.abs(a-b)<=epsilon,`${message}: ${a} != ${b}`);
const attr=(source,key,fallback='')=>source.match(new RegExp(`\\b${key}="([^"]*)"`))?.[1]??fallback;
const numbers=text=>text.trim().split(/\s+/).map(Number);
const serial=box=>({min:box.min.toArray(),max:box.max.toArray()});
// The browser STLLoader uses the page's import map for its bare `three`
// dependency. Read the original STL positions directly in this Node-only
// geometry suite instead of changing production imports or requiring a DOM.
function originalSTLGeometry(bytes){
  const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  const triangles=bytes.byteLength>=84?view.getUint32(80,true):0;let positions=[];
  if(84+triangles*50===bytes.byteLength){
    positions=new Float32Array(triangles*9);
    for(let face=0;face<triangles;face++)for(let component=0;component<9;component++)
      positions[face*9+component]=view.getFloat32(84+face*50+12+component*4,true);
  }else for(const match of bytes.toString('utf8').matchAll(/vertex\s+(\S+)\s+(\S+)\s+(\S+)/g))positions.push(...match.slice(1).map(Number));
  assert(positions.length>=9&&positions.length%9===0&&positions.every(Number.isFinite),'Invalid original STL vertex data');
  return new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
}
function corners(box){return [box.min.x,box.max.x].flatMap(x=>[box.min.y,box.max.y]
  .flatMap(y=>[box.min.z,box.max.z].map(z=>new THREE.Vector3(x,y,z))));}
function preciseBounds(robot,meshes){
  robot.updateMatrixWorld(true);const box=new THREE.Box3(),point=new THREE.Vector3();
  for(const mesh of meshes){const positions=mesh.geometry.attributes.position;
    for(let index=0;index<positions.count;index++)box.expandByPoint(point.fromBufferAttribute(positions,index).applyMatrix4(mesh.matrixWorld));}
  return box;
}

async function visualFixture(id){
  const urdf={r1:'r1.urdf',g1:'g1_29dof_with_dex1_1.urdf',c5:'c5-dex1.urdf'}[id];
  const file=`dist/${id}/assets/${urdf}`,xml=(await boundBytes(file)).toString('utf8');
  const robot=await ({r1:loadR1,g1:loadG1,c5:loadC5}[id])(),meshes=[];
  let visualCount=0;
  for(const [,attributes,body]of xml.matchAll(/<link\b([^>]*)>([\s\S]*?)<\/link>/g)){
    const linkName=attr(attributes,'name'),link=robot.getObjectByName(linkName);
    for(const [,visual]of body.matchAll(/<visual\b[^>]*>([\s\S]*?)<\/visual>/g)){
      assert(link,`${id}: original visual link ${linkName} is absent`);
      const tag=visual.match(/<mesh\b[^>]*>/)?.[0],filename=attr(tag??'','filename');
      assert(filename,`${id}/${linkName}: unsupported non-mesh visual must not be omitted`);
      const url=filename.startsWith('package://r1_v2_1_0/')
        ?new URL(`dist/r1/assets/${filename.slice('package://r1_v2_1_0/'.length)}`,root)
        :new URL(filename,new URL(file,root));
      const path=relative(rootPath,fileURLToPath(url));assert(!path.startsWith('../'),'Mesh escaped checkout');
      const bytes=await boundBytes(path);
      const object=/\.3ds$/i.test(filename)?parseC5_3DS(bytes,THREE)
        :new THREE.Mesh(originalSTLGeometry(bytes),new THREE.MeshBasicMaterial());
      const origin=visual.match(/<origin\b[^>]*>/)?.[0]??'';
      object.position.fromArray(numbers(attr(origin,'xyz','0 0 0')));
      object.quaternion.setFromEuler(new THREE.Euler(...numbers(attr(origin,'rpy','0 0 0')),'ZYX'));
      object.scale.fromArray(numbers(attr(tag,'scale','1 1 1')));link.add(object);visualCount++;
      object.traverse(node=>{if(node.isMesh){assert(node.geometry.attributes.position?.count>0);meshes.push(node);}});
    }
  }
  assert.equal(visualCount,[...xml.matchAll(/<visual\b/g)].length,`${id}: every original visual must be loaded`);
  assert(meshes.length>0);return {id,robot,meshes,visualCount};
}
const r1Standby={...R1.REST_TORSO,...R1.REST.right,...R1.REST.left},r1Spread={...R1.REST_TORSO,...R1.SPREAD.right,...R1.SPREAD.left};
const poseSets={
  r1:{idle:r1Standby,spread:r1Spread,
    crouch:{...R1.CROUCH,...R1.SPREAD.right,...R1.SPREAD.left},
    ...Object.fromEntries(Array.from({length:41},(_,index)=>[
      `spread-entry-${index}`,Object.fromEntries(Object.keys(r1Spread).map(name=>[name,r1Standby[name]+(r1Spread[name]-r1Standby[name])*index/40])),
    ])),
    ...Object.fromEntries(Array.from({length:21},(_,index)=>{
      const yaw=Number((-1+index*.1).toFixed(1));
      return [`crouch-waist-${yaw.toFixed(1)}`,{...R1.CROUCH,...R1.SPREAD.right,...R1.SPREAD.left,torso_joint4:yaw}];
    }))},
  g1:{idle:{...G1_STAND,...G1.REST.right,...G1.REST.left},
    spread:{...G1_STAND,...G1.SPREAD.right,...G1.SPREAD.left},
    raised:{...G1_STAND,...G1.RAISED.right,...G1.RAISED.left},
    ready:{...G1_STAND,...G1.READY.right,...G1.READY.left}},
  c5:{idle:{...C5_HOME}},
};
// Preserve the compact-layout regression's original pose, not the new shorter
// C5 silhouette. Both sides of that historical camera comparison must use the
// same explicitly frozen joint targets. These hashes document the commit from
// which the constants were read; they are historical provenance, not assertions
// that today's changed controllers still have the old bytes.
const historicalStandbyFixture={
  commit:'b271a9aeb5bbdf8d2bad5853519da34bd59bf8ea',
  sourceFileSha256:{
    'dist/r1/grasp-controller.mjs':'c042b05bb52c2a3e8e9940e90a0742fd5048681a078ca620902f65594ba77f0f',
    'dist/g1/grasp-controller.mjs':'68c147b966a9c138e98ae0e1fdad0cabb816684d2422ee2fa459feeea2bb198d',
    'dist/g1/g1-profile.mjs':'6a7b4023c8c3e6e9df69d6e463b6f72bd73523f1f3a967f90b60a4c63a17de08',
    'dist/c5/c5-profile.mjs':'6a2419169fdf4cda2b4d34b56b3f1b6f97cc2f74b38497e08e67a12b97c135dc',
  },
  motionViewBounds:{
    r1:{min:[-.371,-.062,-.872],max:[1.160,1.709,.872]},
    g1:{min:[-.100,-.021,-.696],max:[.800,1.343,.695]},
    c5:{min:[-.539,-.020,-.780],max:[.800,1.814,.698]},
  },
  poses:{
    r1:{torso_joint1:0,torso_joint2:.06,torso_joint3:0,torso_joint4:0,
      ...Object.fromEntries(['right','left'].flatMap(side=>[-1.57*(side==='right'?1:-1),2.20,-1.40,0,0,0]
        .map((value,index)=>[`${side}_arm_joint${index+1}`,value])))},
    g1:{...Object.fromEntries(['left','right'].flatMap(side=>['hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll']
      .map(suffix=>[`${side}_${suffix}_joint`,0]))),waist_yaw_joint:0,waist_roll_joint:0,waist_pitch_joint:0,
      ...Object.fromEntries(['right','left'].flatMap(side=>['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']
        .map((suffix,index)=>[`${side}_${suffix}_joint`,[0,.025*(side==='left'?1:-1),0,1.50,0,0,0][index]])))},
    c5:Object.fromEntries(['shoulder','upperArm','foreArm','wrist1','wrist2','wrist3'].map(name=>[`${name}_joint`,0])),
  },
};
function applyPose(fixture,pose){
  for(const joint of Object.values(fixture.robot.joints))joint.setValue(0);
  const fingers=Object.fromEntries(Object.keys(fixture.robot.joints).filter(name=>name.includes('finger'))
    .map(name=>[name,fixture.id==='r1'?.05:.0245]));
  fixture.robot.setJointValues({...pose,...fingers});fixture.robot.updateMatrixWorld(true);
}
function envelopeFor(id){
  const item=workcell.STATION_VIEW_BOUNDS?.[id];assert(item,`${id}: production STATION_VIEW_BOUNDS is required`);
  assert(item.min?.length===3&&item.max?.length===3,`${id}: malformed production view envelope`);
  return new THREE.Box3(new THREE.Vector3(...item.min),new THREE.Vector3(...item.max));
}
function shifted(box,offset){return box.clone().translate(new THREE.Vector3(...offset));}
function sceneBounds(selected,{old=false,historical=false,motion=true}={}){
  const box=new THREE.Box3();for(const id of selected){const offset=old?[0,0,(1-ids.indexOf(id))*2.2]:workcell.STATION_OFFSETS[id];
    const measured=historical?geometry[id].historical:geometry[id];
    box.union(shifted(old?measured.idleStationBox:motion?measured.framingBox:measured.idleFrameBox,offset));}return box;
}
function cameraFor(fit,view){
  const camera=new THREE.PerspectiveCamera(42,view.width/view.height,.02,40);
  camera.position.copy(fit.position);camera.lookAt(fit.target);camera.updateMatrixWorld(true);return camera;
}
function oldFit(box,view){
  const target=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3()),tan=Math.tan(THREE.MathUtils.degToRad(21));
  const distance=Math.max(size.z/(2*tan*view.width/view.height),size.y/(2*tan),size.x/(2*tan))*1.35+size.x*.45;
  return {target,position:target.clone().addScaledVector(new THREE.Vector3(1,.33,.10).normalize(),distance),distance};
}
function projectedBox(box,camera,view){
  const points=corners(box).map(p=>p.project(camera));
  return {points,width:(Math.max(...points.map(p=>p.x))-Math.min(...points.map(p=>p.x)))*view.width/2,
    height:(Math.max(...points.map(p=>p.y))-Math.min(...points.map(p=>p.y)))*view.height/2};
}
function compareFraming(view,{historical=false}={}){
  const oldCamera=cameraFor(oldFit(sceneBounds(ids,{old:true,historical}),view),view);
  const fit=workcell.fitWorkstationView(sceneBounds(ids,{historical}),
    {fov:42,aspect:view.width/view.height,viewportHeight:view.height,near:.02}),newCamera=cameraFor(fit,view),robots=[];
  for(const id of ids){const measured=historical?geometry[id].historical:geometry[id];
    const oldRect=projectedBox(shifted(measured.idleBox,[0,0,(1-ids.indexOf(id))*2.2]),oldCamera,view);
    const newRect=projectedBox(shifted(measured.idleBox,workcell.STATION_OFFSETS[id]),newCamera,view);
    robots.push({id,oldHeightPixels:oldRect.height,newHeightPixels:newRect.height,heightRatio:newRect.height/oldRect.height,
      oldWidthPixels:oldRect.width,newWidthPixels:newRect.width,widthRatio:newRect.width/oldRect.width});}
  return {viewport:view.name,averageHeightRatio:robots.reduce((sum,item)=>sum+item.heightRatio,0)/3,robots};
}
function compareIdleToMotion(view){
  const fit=bounds=>workcell.fitWorkstationView(bounds,
    {fov:42,aspect:view.width/view.height,viewportHeight:view.height,near:.02});
  const priorBounds=new THREE.Box3();
  for(const id of ids){const item=historicalStandbyFixture.motionViewBounds[id],priorEnvelope=new THREE.Box3(new THREE.Vector3(...item.min),new THREE.Vector3(...item.max));
    priorBounds.union(shifted(geometry[id].idleFrameBox.clone().union(priorEnvelope),workcell.STATION_OFFSETS[id]));}
  const priorPolicy=fit(priorBounds),idle=fit(sceneBounds(ids,{motion:false})),priorCamera=cameraFor(priorPolicy,view),idleCamera=cameraFor(idle,view),robots=[];
  for(const id of ids){const box=shifted(geometry[id].idleBox,workcell.STATION_OFFSETS[id]);
    const priorRect=projectedBox(box,priorCamera,view),idleRect=projectedBox(box,idleCamera,view);
    robots.push({id,priorMotionPolicyHeightPixels:priorRect.height,idleHeightPixels:idleRect.height,heightRatio:idleRect.height/priorRect.height,
      priorMotionPolicyWidthPixels:priorRect.width,idleWidthPixels:idleRect.width,widthRatio:idleRect.width/priorRect.width});}
  return {viewport:view.name,priorMotionPolicyDistance:priorPolicy.distance,idleDistance:idle.distance,
    averageHeightRatio:robots.reduce((sum,item)=>sum+item.heightRatio,0)/3,robots};
}
// These are canvas dimensions, not full-window dimensions. The 340 px desktop
// panel and 58svh narrow-screen canvas follow the incumbent CSS unchanged.
const views=[
  {name:'desktop-1280',width:940,height:720,desktop:true},
  {name:'desktop-1440',width:1100,height:900,desktop:true},
  {name:'desktop-wide',width:2220,height:1440,desktop:true},
  {name:'desktop-breakpoint-901',width:561,height:700,desktop:true},
  {name:'narrow-breakpoint-900',width:900,height:406,desktop:false},
  {name:'phone-390',width:390,height:844*.58,desktop:false},
  {name:'tablet-834',width:834,height:1112*.58,desktop:false},
];

for(const path of paths)await boundBytes(path);
await check('The station row uses exact 1.6 metre display offsets and the application fits the complete production presentation bounds',async()=>{
  assert.deepEqual(workcell.STATION_OFFSETS,{r1:[0,0,1.6],g1:[0,0,0],c5:[0,0,-1.6]});
  const runtimeBytes=sourceHashes['dist/multi/robot-runtime.mjs'];assert(runtimeBytes);
  const app=(await boundBytes('dist/multi/app.mjs')).toString('utf8');
  const start=app.indexOf('function fitView('),end=app.indexOf('\nfunction setFocus(',start);assert(start>=0&&end>start);
  const fitSource=app.slice(start,end);
  for(const fragment of ['new THREE.Box3().setFromObject(station.group)','STATION_VIEW_BOUNDS[id]',
    '.applyMatrix4(station.group.matrixWorld)','station.labelHeight','new THREE.Box3().setFromObject(wrist.model)',
    'fitWorkstationView(bounds,{fov:camera.fov,aspect:camera.aspect,viewportHeight:viewport.clientHeight,near:camera.near})',
    'orbit.target.copy(view.target);camera.position.copy(view.position)'])assert(fitSource.includes(fragment),`Application fit omits ${fragment}`);
  assert(app.includes('p.y-tag.offsetHeight'),'Station labels must end at their world anchor instead of extending over the robot');
  assert(fitSource.includes("{motion=preparing||Boolean(coordinator?.running)||coordinator?.state==='preparing'}"),'Fit policy must follow preparation/running state');
  assert(/if\(motion\)\{\s*const envelope=STATION_VIEW_BOUNDS\[id\];/.test(fitSource),'Only motion mode may add the conservative preparation envelope');
  const startHandler=app.slice(app.indexOf("$('#start').addEventListener"),app.indexOf("$('#stop').addEventListener"));
  assert(startHandler.includes('updateUI();fitView(undefined,{motion:true});'),'Preparing must expand the view before yielding a frame');
  assert(startHandler.includes('fitView(undefined,{motion:result.ok})'),'Accepted starts must retain motion view and rejected starts must refit actual geometry');
  const resetHandler=app.slice(app.indexOf("$('#reset').addEventListener"),app.indexOf('\nfunction ray('));
  assert(resetHandler.includes('coordinator?.reset()')&&resetHandler.includes('fitView(undefined,{motion:false})'),'Reset must return to the actual standby fit');
  const configure=app.slice(app.indexOf('async function configureTask('),start);
  assert(configure.includes('fitView(undefined,{motion:false})'),'Task/randomization preparation must not leave an unnecessary motion envelope at rest');
  assert(app.includes("$('#fit-view').addEventListener('click',()=>fitView())")&&app.includes("$('#focus-view').addEventListener('click',()=>fitView([coordinator?.focusId]))"),'Panorama/focus must use the state-aware policy');
  assert(app.includes('if(coordinator)fitView();'),'Resize must refit with the state-aware policy');
  assert.equal(typeof workcell.fitWorkstationView,'function');
  for(const bad of [{aspect:0},{viewportHeight:0},{fov:180},{near:0}])
    assert.throws(()=>workcell.fitWorkstationView(new THREE.Box3(new THREE.Vector3(),new THREE.Vector3(1,1,1)),bad),RangeError);
  assert.throws(()=>workcell.fitWorkstationView(new THREE.Box3()),RangeError);
  return {offsets:workcell.STATION_OFFSETS,scope:'Display-group origins only; local-frame isolation is separately source-bound below.'};
});
await check('The exact production fit function selects current geometry at rest and reserves motion bounds only when needed',async()=>{
  const app=(await boundBytes('dist/multi/app.mjs')).toString('utf8'),start=app.indexOf('function fitView('),end=app.indexOf('\nfunction setFocus(',start),source=app.slice(start,end);
  // An explicit small-box contract fixture exercises production branching, not
  // robot appearance. The source-mesh suites below provide actual geometry.
  const group=new THREE.Group(),mesh=new THREE.Mesh(new THREE.BoxGeometry(.2,.4,.2),new THREE.MeshBasicMaterial());
  mesh.position.set(.3,.5,0);group.position.fromArray(workcell.STATION_OFFSETS.r1);group.add(mesh);group.updateMatrixWorld(true);
  const station={group,labelHeight:.9},fixtureStations=new Map([['r1',station]]),fixtureVisions=new Map(),coordinator={selectedIds:['r1'],running:false,state:'idle'};
  const viewport={clientHeight:720},camera=new THREE.PerspectiveCamera(42,940/720,.02,40),orbit={target:new THREE.Vector3(),update(){}};
  function fixtureFit(preparing=false){return new Function('THREE','stations','visions','coordinator','preparing','STATION_VIEW_BOUNDS','fitWorkstationView','viewport','camera','orbit',
    `let lastViewFit=null;${source};return (ids,options)=>{fitView(ids,options);return lastViewFit;};`)
    (THREE,fixtureStations,fixtureVisions,coordinator,preparing,workcell.STATION_VIEW_BOUNDS,workcell.fitWorkstationView,viewport,camera,orbit);}
  const run=fixtureFit(),idle=run();assert.equal(idle.mode,'idle');assert.deepEqual(idle.ids,['r1']);
  const actual=new THREE.Box3().setFromObject(group);actual.expandByPoint(group.localToWorld(new THREE.Vector3(0,station.labelHeight,0)));
  assert.deepEqual(idle.bounds,serial(actual),'Standby fit must use actual object bounds, without the future envelope');
  coordinator.running=true;const running=run();assert.equal(running.mode,'motion');assert(running.distance>idle.distance);
  coordinator.running=false;assert.equal(fixtureFit(true)().mode,'motion');coordinator.state='preparing';assert.equal(run().mode,'motion');
  coordinator.state='idle';assert.equal(run(undefined,{motion:false}).mode,'idle');assert.equal(run(undefined,{motion:true}).mode,'motion');
  mesh.geometry.dispose();mesh.material.dispose();
  return {idleDistance:idle.distance,motionDistance:running.distance,preparingAndRunningBranchesChecked:true,
    scope:'Exact production function with a declared box-only state-contract fixture; no robot rendering or physical execution.'};
});
for(const id of ids)await check(`${id}: all original visual meshes and named source poses fit the production view envelope`,async()=>{
  const fixture=await visualFixture(id),envelope=envelopeFor(id),samples={},idleWristBounds=new THREE.Box3();let idleBox;
  const vision=(id==='r1'?r1WristVision:dexWristVision)({scene:new THREE.Scene(),THREE,camera:new THREE.PerspectiveCamera()});
  for(const [name,pose]of Object.entries(poseSets[id])){
    applyPose(fixture,pose);const actual=preciseBounds(fixture.robot,fixture.meshes),productionAABB=new THREE.Box3().setFromObject(fixture.robot);
    assert(envelope.clone().expandByScalar(1e-6).containsBox(actual),`${id}/${name}: actual visual vertices exceed framing envelope`);
    assert(envelope.clone().expandByScalar(1e-6).containsBox(productionAABB),`${id}/${name}: production per-mesh AABB exceeds framing envelope`);
    const wrists=[];for(const side of id==='c5'?['left']:['right','left']){
      const link=fixture.robot.getObjectByName((ROBOT_CONFIGS[id].cameraFrame??ROBOT_CONFIGS[id].gripperLink)(side));assert(link,`${id}/${side}: original camera link absent`);
      assert(vision.update(link));const mountBox=new THREE.Box3().setFromObject(vision.model);
      assert(envelope.clone().expandByScalar(1e-6).containsBox(mountBox),`${id}/${name}/${side}: real camera attachment exceeds production view envelope`);
      wrists.push({side,bounds:serial(mountBox),containedByProductionEnvelope:true});if(name==='idle')idleWristBounds.union(mountBox);
    }
    samples[name]={preciseVertices:serial(actual),productionObjectAABB:serial(productionAABB),wristCameraAttachments:wrists};
    if(name==='idle')idleBox=productionAABB.clone();
  }
  applyPose(fixture,historicalStandbyFixture.poses[id]);
  const historicalIdleBox=new THREE.Box3().setFromObject(fixture.robot),historicalIdleVertices=preciseBounds(fixture.robot,fixture.meshes);
  const historicalWristBounds=new THREE.Box3();
  for(const side of id==='c5'?['left']:['right','left']){
    const link=fixture.robot.getObjectByName((ROBOT_CONFIGS[id].cameraFrame??ROBOT_CONFIGS[id].gripperLink)(side));assert(vision.update(link));
    historicalWristBounds.union(new THREE.Box3().setFromObject(vision.model));
  }
  vision.dispose();
  const station={id,config:ROBOT_CONFIGS[id],group:new THREE.Group()};
  const table=workcell.addStationTable(station);station.group.updateMatrixWorld(true);
  const tableBox=new THREE.Box3().setFromObject(table),supportBox=new THREE.Box3().setFromObject(station.group);
  const config=ROBOT_CONFIGS[id],taskSpace=new THREE.Box3(
    new THREE.Vector3(config.tableCenter-config.tableHalfX,config.tableHeight,-config.tableHalfZ),
    new THREE.Vector3(config.tableCenter+config.tableHalfX,config.tableHeight+.065,config.tableHalfZ));
  // This task-space box is explicitly a conservative table-local proxy. Robot
  // visual bounds above are original mesh measurements, not this proxy.
  const idleStationBox=idleBox.clone().union(supportBox).union(taskSpace),idleFrameBox=idleStationBox.clone().union(idleWristBounds);
  const labelHeight=idleBox.max.y+.13;idleFrameBox.expandByPoint(new THREE.Vector3(0,labelHeight,0));
  const framingBox=idleFrameBox.clone().union(envelope);
  const historicalIdleStationBox=historicalIdleBox.clone().union(supportBox).union(taskSpace);
  const historicalGuard=historicalStandbyFixture.motionViewBounds[id];
  const historicalFramingBox=historicalIdleStationBox.clone().union(new THREE.Box3(new THREE.Vector3(...historicalGuard.min),new THREE.Vector3(...historicalGuard.max))).union(historicalWristBounds);
  const historicalLabelHeight=historicalIdleBox.max.y+.13;historicalFramingBox.expandByPoint(new THREE.Vector3(0,historicalLabelHeight,0));
  geometry[id]={visualCount:fixture.visualCount,meshNodeCount:fixture.meshes.length,
    vertexCount:fixture.meshes.reduce((n,mesh)=>n+mesh.geometry.attributes.position.count,0),
    namedPoses:samples,envelope:serial(envelope),table:serial(tableBox),taskSpaceProxy:serial(taskSpace),
    idleRobot:serial(idleBox),idleStation:serial(idleStationBox),idleFraming:serial(idleFrameBox),motionFraming:serial(framingBox),
    labelHeight,idleWristAttachments:serial(idleWristBounds),idleBox,idleStationBox,idleFrameBox,framingBox,tableBox,
    historical:{idleRobot:serial(historicalIdleBox),idleVisualVertices:serial(historicalIdleVertices),idleStation:serial(historicalIdleStationBox),
      framing:serial(historicalFramingBox),labelHeight:historicalLabelHeight,wristAttachments:serial(historicalWristBounds),
      idleBox:historicalIdleBox,idleStationBox:historicalIdleStationBox,framingBox:historicalFramingBox}};
  assert.equal(fixture.robot.parent,null,'Source kinematic robot must remain outside display station');
  return {visualCount:fixture.visualCount,meshNodes:fixture.meshes.length,poses:Object.keys(samples)};
});
await check('Only the independent display parent receives the station translation',async()=>{
  const runtime=(await boundBytes('dist/multi/robot-runtime.mjs')).toString('utf8');
  assert(runtime.includes("if(robot.parent)throw new Error("));assert(runtime.includes('group.position.fromArray(offset)'));
  assert(runtime.includes('visualRobot??=robot.clone(true);group.add(visualRobot)'));
  assert(!/robot\.position\.(?:add|set|copy|fromArray)\([^\n;]*offset/.test(runtime));
  const local=new THREE.Vector3(.31,.926,.12);
  for(const id of ids){const group=new THREE.Group();group.position.fromArray(workcell.STATION_OFFSETS[id]);group.updateMatrixWorld(true);
    const untouched=local.clone(),world=group.localToWorld(local.clone());
    assert(world.distanceTo(local.clone().add(new THREE.Vector3(...workcell.STATION_OFFSETS[id])))<1e-12);
    assert(group.worldToLocal(world).distanceTo(untouched)<1e-12);assert(local.equals(untouched));}
  const preserved=[];
  for(const path of ['work/validation/multi/physical-final.json','work/validation/multi/stack-final.json','work/validation/multi-pipes/physical-final.json']){
    const report=JSON.parse(await readFile(new URL(path,root),'utf8'));assert.equal(report.pass,true);
    for(const [source,hash]of Object.entries(report.sourceHashes)){assert.equal(sha(await readFile(new URL(source,root))),hash,`${path}: physical source changed: ${source}`);}
    preserved.push({path,sourceCount:Object.keys(report.sourceHashes).length});
  }
  return {preserved,scope:'Current immutable physical-source bindings plus display transform round-trips; no new physical execution.'};
});
await check('Grounded tables retain a positive independently measured inter-station gap',()=>{
  const gaps=[];for(let i=0;i<ids.length-1;i++){
    const a=shifted(geometry[ids[i]].tableBox,workcell.STATION_OFFSETS[ids[i]]),b=shifted(geometry[ids[i+1]].tableBox,workcell.STATION_OFFSETS[ids[i+1]]);
    const actual=a.min.z-b.max.z,expected=1.6-ROBOT_CONFIGS[ids[i]].tableHalfZ-ROBOT_CONFIGS[ids[i+1]].tableHalfZ;
    near(actual,expected,'Real table gap');assert(actual>.8,'Tables must retain more than .8m edge clearance');
    gaps.push({pair:[ids[i],ids[i+1]],metres:actual});}return gaps;
});
if(ids.every(id=>geometry[id]))for(let mask=1;mask<8;mask++){
  const selected=ids.filter((id,index)=>mask&(1<<index));
  for(const motion of [false,true])for(const view of views)await check(`${selected.join('+')}/${view.name}/${motion?'motion':'idle'}: full robots, feet, worktables and required view bounds stay inside the camera`,()=>{
    assert.equal(typeof workcell.fitWorkstationView,'function');const bounds=sceneBounds(selected,{motion});
    const fit=workcell.fitWorkstationView(bounds,{fov:42,aspect:view.width/view.height,viewportHeight:view.height,near:.02}),camera=cameraFor(fit,view);
    assert(Number.isFinite(fit.distance)&&fit.distance>=.6&&fit.distance<=22,'Fit must remain inside actual OrbitControls distance limits');near(fit.fillX,.9,'Horizontal fill');
    near(fit.fillY,Math.max(.50,1-152/view.height),'Vertical HUD fill');
    let maxX=0,maxY=0,minNear=Infinity;
    for(const point of corners(bounds)){
      const depth=-point.clone().applyMatrix4(camera.matrixWorldInverse).z,p=point.clone().project(camera);
      assert(Math.abs(p.x)<=fit.fillX+1e-7&&Math.abs(p.y)<=fit.fillY+1e-7,'Envelope clipped or entered reserved HUD margin');
      assert(p.z>-1&&p.z<1,'Envelope lies outside near/far clip planes');assert(depth>=.07-1e-7,'Near-plane clearance missing');
      maxX=Math.max(maxX,Math.abs(p.x));maxY=Math.max(maxY,Math.abs(p.y));minNear=Math.min(minNear,depth);
    }
    const labels=[];for(const id of selected){
      const anchor=new THREE.Vector3(0,geometry[id].labelHeight,0).add(new THREE.Vector3(...workcell.STATION_OFFSETS[id])).project(camera);
      const x=(anchor.x+1)*view.width/2,y=(1-anchor.y)*view.height/2,width=view.desktop?110:94,height=view.desktop?32:30;
      assert(x-width/2>=0&&x+width/2<=view.width&&y-height>=0&&y<=view.height,`${id}: conservative label rectangle cropped`);
      labels.push({id,x,anchorBottomY:y,topY:y-height,assumedWidth:width,assumedHeight:height});
    }
    projections.push({selected,mode:motion?'motion':'idle',viewport:view.name,width:view.width,height:view.height,distance:fit.distance,maxAbsNdcX:maxX,maxAbsNdcY:maxY,minDepthM:minNear,labels});
    return {cornerCount:8,distance:fit.distance,topBottomReservePixels:(1-fit.fillY)*view.height/2};
  });
}
if(ids.every(id=>geometry[id]))for(const view of views.filter(view=>view.desktop))await check(`${view.name}: identical frozen b271a9a standby poses retain the historical 15 percent compact-framing gain`,()=>{
  const historical=compareFraming(view,{historical:true}),current=compareFraming(view);
  historicalMagnifications.push(historical);currentPoseMagnifications.push(current);
  assert(historical.averageHeightRatio>=1.15,`Historical fixture average visual height only ${historical.averageHeightRatio.toFixed(3)} times old panorama`);
  assert(Number.isFinite(current.averageHeightRatio)&&current.averageHeightRatio>0,'Current-pose observation must be finite and positive');
  return {historicalAverageHeightRatio:historical.averageHeightRatio,historicalMinimumRatio:1.15,
    currentPoseObservedAverageHeightRatio:current.averageHeightRatio,currentPoseHasNoMinimumMagnificationGate:true};
});
if(ids.every(id=>geometry[id]))for(const view of views.filter(view=>view.desktop))await check(`${view.name}: current standby is closer than the previous always-motion policy without changing any robot pose or scale`,()=>{
  const observation=compareIdleToMotion(view);idleFramingGains.push(observation);
  assert(observation.idleDistance<observation.priorMotionPolicyDistance,'Current standby must not retain the farther motion fit');
  assert(Number.isFinite(observation.averageHeightRatio)&&observation.averageHeightRatio>1,'Standby robots must be visibly enlarged by the camera change alone');
  return {averageHeightRatio:observation.averageHeightRatio,priorMotionPolicyDistance:observation.priorMotionPolicyDistance,idleDistance:observation.idleDistance};
});
const sourceFrozen=(await Promise.all([...paths].map(async path=>sha(await readFile(new URL(path,root)))===sourceHashes[path]))).every(Boolean);
if(!sourceFrozen)errors.push({name:'source freeze',message:'A measured source or visual asset changed during the geometry suite'});
const outputGeometry=Object.fromEntries(Object.entries(geometry).map(([id,data])=>{
  const {historical,...current}=data,omitRuntimeBoxes=record=>Object.fromEntries(Object.entries(record)
    .filter(([key])=>!['idleBox','idleStationBox','idleFrameBox','framingBox','tableBox'].includes(key)));
  return [id,{...omitRuntimeBoxes(current),historicalStandbyFixture:omitRuntimeBoxes(historical)}];
}));
const report={schemaVersion:3,pass:errors.length===0,scope:'Current-source visual vertices and named poses, separate idle/motion perspective projections, exact production fit-state contracts, plus an explicitly frozen historical-pose camera regression. No browser rendering or physical simulation.',
  limitations:['Named poses do not cover arbitrary manually requested joint configurations or full continuous physical trajectories.',
    'Task-space boxes conservatively cover the existing table-local task objects; they are not robot visual measurements.',
    'Label rectangles use explicit conservative pixel estimates, not browser font metrics; final native desktop/mobile inspection remains necessary.',
    'The 15 percent gate applies only to identical frozen historical standby poses under the two camera/layout implementations. Current standby magnifications are separate observations, not a claim that every viewport exceeds 15 percent.',
    'Both camera comparisons use source-mesh geometry measured with the same joint pose on each side; neither compares the new shorter C5 pose with the historical straight C5 pose or with user-photo pixels.',
    'Independent physical worlds do not calculate cross-station collision; geometric separation is not hardware safety certification.'],
  total:checks.length,passed:checks.filter(item=>item.pass).length,sourceFrozen,sourceHashes,geometry:outputGeometry,
  motionPoseSampling:{r1CrouchWaistYaw:{minimumRad:-1,maximumRad:1,stepRad:.1,samples:21,
    scope:'Source visual meshes at explicitly sampled CROUCH+SPREAD waist poses only. Not a continuous whole-joint envelope, collision test or safety proof.'},
    r1StandbyToSpread:{samples:41,interpolation:'Linear joint interpolation at fixed 1/40 path fractions, covering the same geometric path as smoothstep timing.',
      scope:'Explicit preparation-entry samples only; this does not prove all planned returns, arbitrary joint paths or collision safety.'}},
  projectionCases:projections.length,idleProjectionCases:projections.filter(row=>row.mode==='idle').length,
  motionProjectionCases:projections.filter(row=>row.mode==='motion').length,projections,
  comparisons:{
    historical:{fixture:historicalStandbyFixture,minimumAverageHeightRatio:1.15,magnifications:historicalMagnifications,
      scope:'Frozen b271a9a standby geometry and motion guards on both sides: 2.2m/previous fit versus 1.6m/corner-fit with that historical motion envelope. Historical regression, not the new default idle view or current wider waist-yaw guard.'},
    current:{poses:Object.fromEntries(ids.map(id=>[id,poseSets[id].idle])),minimumAverageHeightRatio:null,magnifications:currentPoseMagnifications,
      scope:'Same current standby geometry on both sides: 2.2m/previous fit versus 1.6m/motion-envelope fit; factual observations only. Both 49-case current idle/motion projection sets remain gated above.'},
    idleVersusPriorMotionPolicy:{policyBaselineCommit:'b271a9aeb5bbdf8d2bad5853519da34bd59bf8ea',poses:Object.fromEntries(ids.map(id=>[id,poseSets[id].idle])),
      stationOffsets:workcell.STATION_OFFSETS,magnifications:idleFramingGains,
      scope:'Camera-only counterfactual: identical current standby source geometry, source-camera attachments, 1.6m offsets, lens settings and HUD margins on both sides; frozen b271a9a always-motion guards versus new actual-idle fit. Newly widened motion guards are not used to inflate this gain. This does not compare different robot poses or reproduce historical browser pixels.'},
  },checks,errors};
await mkdir(new URL('work/validation/multi-framing/',root),{recursive:true});
await writeFile(new URL(reportPath,root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,total:report.total,passed:report.passed,projectionCases:projections.length,
  historicalMagnifications:historicalMagnifications.map(({viewport,averageHeightRatio})=>({viewport,averageHeightRatio})),
  currentPoseObservations:currentPoseMagnifications.map(({viewport,averageHeightRatio})=>({viewport,averageHeightRatio})),
  idleVersusPriorMotionPolicy:idleFramingGains.map(({viewport,averageHeightRatio})=>({viewport,averageHeightRatio})),sourceFrozen,report:reportPath,errors},null,2));
if(!report.pass)process.exitCode=1;
