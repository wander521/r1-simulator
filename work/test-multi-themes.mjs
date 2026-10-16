import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import {createSharedWorkcell,STATION_OFFSETS,STATION_VIEW_BOUNDS} from '../dist/multi/workcell.mjs';
import {createThemeController} from '../dist/multi/visuals/theme-controller.mjs';

// Execute the production theme controller with the production Three scene.
// DOM controls/storage and task-state sentinels below are declared fixtures,
// not a browser, renderer, real robot, contact or successful-grasp simulation.
const root=new URL('../',import.meta.url),checks=[],errors=[],observations=[];
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)??'work/validation/multi-themes/contracts-final.json';
const sources=['work/test-multi-themes.mjs','dist/multi/workcell.mjs','dist/multi/app.mjs','dist/multi/index.html','dist/multi/styles.css',
  'dist/multi/visuals/theme-controller.mjs','dist/multi/visuals/scene-environment.mjs','dist/multi/visuals/camera-feeds.mjs','dist/multi/visuals/frame-scheduler.mjs',
  'dist/vendor/three.module.min.js','dist/vendor/three.core.min.js'];
async function hashes(){return Object.fromEntries(await Promise.all(sources.map(async p=>[p,createHash('sha256').update(await readFile(new URL(p,root))).digest('hex')])));}
const sourceHashes=await hashes(),themeSource=await readFile(new URL('dist/multi/visuals/theme-controller.mjs',root),'utf8'),app=await readFile(new URL('dist/multi/app.mjs',root),'utf8');
const html=await readFile(new URL('dist/multi/index.html',root),'utf8'),css=await readFile(new URL('dist/multi/styles.css',root),'utf8');
const check=(name,fn)=>{try{const detail=fn();checks.push({name,pass:true,...(detail?{detail}:{})});}catch(error){checks.push({name,pass:false,error:error.message});errors.push({name,message:error.message});}};
function button(id){const listeners=new Map(),attributes=new Map();return {dataset:{sceneSwitch:id},disabled:false,
  setAttribute:(name,value)=>attributes.set(name,String(value)),getAttribute:name=>attributes.get(name)??null,
  addEventListener:(event,fn)=>{const set=listeners.get(event)??new Set();set.add(fn);listeners.set(event,set);},
  removeEventListener:(event,fn)=>listeners.get(event)?.delete(fn),
  click(){for(const fn of listeners.get('click')??[])fn({target:this,currentTarget:this,preventDefault(){}});},
  listenerCount:()=>[...listeners.values()].reduce((sum,set)=>sum+set.size,0)};}
function stored(value=null){const calls={get:[],set:[]};return {calls,getItem(key){calls.get.push(key);return value;},setItem(key,next){calls.set.push([key,next]);value=String(next);}};}
function fixture(storage=stored(),options={}){
  const scene=new THREE.Scene(),workcell=createSharedWorkcell(scene),domRoot={dataset:{}},buttons=[button('night'),button('hangar')];
  const controller=createThemeController({workcell,root:domRoot,buttons,storageProvider:()=>storage,...options});
  return {scene,workcell,root:domRoot,buttons,controller,storage,dispose(){controller.dispose();workcell.dispose();}};
}
function aligned(f,id){assert.equal(f.controller.activeId,id);assert.equal(f.workcell.activeId,id);assert.equal(f.root.dataset.scene,id);
  for(const b of f.buttons)assert.equal(b.getAttribute('aria-pressed'),String(b.dataset.sceneSwitch===id));}
function resources(scene){
  const nodes=[],geometry=new Set(),materials=new Set(),textures=new Set(),lights=[];
  scene.traverse(node=>{nodes.push(node.uuid);if(node.geometry)geometry.add(node.geometry.uuid);if(node.isLight)lights.push(node.uuid);
    for(const material of Array.isArray(node.material)?node.material:node.material?[node.material]:[]){materials.add(material.uuid);for(const value of Object.values(material))if(value?.isTexture)textures.add(value.uuid);}});
  return {nodes:nodes.sort(),geometry:[...geometry].sort(),materials:[...materials].sort(),textures:[...textures].sort(),lights:lights.sort()};
}
function themeTokens(id){
  const base=css.match(/:root\s*\{([^}]+)\}/)?.[1]??'';
  const override=css.match(new RegExp(`:root\\[data-scene="${id}"\\]\\s*\\{([^}]+)\\}`))?.[1]??'';
  return Object.fromEntries([...`${base}\n${override}`.matchAll(/(--[\w-]+)\s*:\s*([^;]+);/g)].map(([,key,value])=>[key,value.trim()]));
}
function luminance(hex){
  assert(/^#[0-9a-f]{6}$/i.test(hex),`Expected opaque sRGB theme color: ${hex}`);
  const linear=[1,3,5].map(index=>parseInt(hex.slice(index,index+2),16)/255).map(value=>value<=.04045?value/12.92:((value+.055)/1.055)**2.4);
  return linear[0]*.2126+linear[1]*.7152+linear[2]*.0722;
}
const contrast=(a,b)=>{const la=luminance(a),lb=luminance(b);return(Math.max(la,lb)+.05)/(Math.min(la,lb)+.05);};
const fixtures=[];const make=(...args)=>{const f=fixture(...args);fixtures.push(f);return f;};
try{
  check('Theme controller has no task, robot, physics, camera or timing dependencies',()=>{
    assert(!/\b(?:import|export)[^;\n]*from\s*['"][^'"]*(?:robot-runtime|coordinator|grasp-|dynamics|mujoco|camera)/.test(themeSource));
    assert(!/\.(?:start|startStack|stop|reset|step|setObject|moveObject|setJointValues|setTargets|configureTask|setArm)\s*\(/.test(themeSource));
    assert(!/\b(?:requestAnimationFrame|setTimeout|setInterval|performance\.now|Date\.now)\s*\(/.test(themeSource));
    assert(!/\b(?:qpos|qvel|mocap_pos|mocap_quat)\b/.test(themeSource));
  });
  check('Default night aligns real workcell, document state and native pressed-button state',()=>{const f=make();aligned(f,'night');assert.equal(f.buttons.filter(b=>b.getAttribute('aria-pressed')==='true').length,1);});
  check('Valid persisted hangar restores before interaction without requiring available storage writes',()=>{const f=make(stored('hangar'));aligned(f,'hangar');assert(f.storage.calls.get.includes('robot-lab-scene'));});
  check('Retired day and invalid persisted values fall back to night instead of entering a broken scene',()=>{for(const value of ['day','','space','__proto__','NIGHT']){const f=make(stored(value));aligned(f,'night');}});
  check('Both actual native button handlers switch the real scene and persist only the supported scene ID',()=>{
    const f=make();f.buttons[1].click();aligned(f,'hangar');assert.deepEqual(f.storage.calls.set.at(-1),['robot-lab-scene','hangar']);
    f.buttons[0].click();aligned(f,'night');assert.deepEqual(f.storage.calls.set.at(-1),['robot-lab-scene','night']);
  });
  check('Explicit non-persisted transitions preserve storage and unknown IDs preserve the existing scene',()=>{
    const f=make(),before=f.storage.calls.set.length;f.controller.setScene('hangar',{persist:false});aligned(f,'hangar');assert.equal(f.storage.calls.set.length,before);
    for(const id of ['day','bad','',null,undefined,'__proto__']){f.controller.setScene(id);aligned(f,'hangar');assert.equal(f.storage.calls.set.length,before);}
  });
  check('A denied localStorage provider/getter falls back to live in-memory switching',()=>{
    const f=make(null,{storageProvider(){throw new DOMException('denied','SecurityError');}});aligned(f,'night');f.buttons[1].click();aligned(f,'hangar');f.buttons[0].click();aligned(f,'night');
  });
  check('Throwing storage reads do not prevent scene initialization or switching',()=>{
    const f=make({getItem(){throw new DOMException('denied','SecurityError');},setItem(){throw new Error('also denied');}});aligned(f,'night');f.controller.setScene('hangar');aligned(f,'hangar');
  });
  check('Throwing storage writes never undo the scene already selected by the user',()=>{
    const f=make({getItem(){return 'night';},setItem(){throw new DOMException('quota','QuotaExceededError');}});f.controller.setScene('hangar');aligned(f,'hangar');f.controller.setScene('night');aligned(f,'night');
  });
  check('Both palettes alter real Three scene appearance while preserving station transforms and table geometry',()=>{
    const f=make(),station=new THREE.Group();station.name='declared-protected-station-fixture';station.position.fromArray(STATION_OFFSETS.r1);
    const mesh=new THREE.Mesh(new THREE.BoxGeometry(.3,.7,.3),new THREE.MeshStandardMaterial({color:0x8296ad}));station.add(mesh);f.scene.add(station);f.scene.updateMatrixWorld(true);
    const identities={scene:f.scene,group:f.workcell.group,floor:f.workcell.floor,geometry:mesh.geometry,material:mesh.material},transform=station.matrixWorld.toArray();
    const appearance=()=>JSON.stringify({background:f.scene.background?.getHex?.(),fog:f.scene.fog?.color?.getHex?.(),floor:f.workcell.floor.material.color.getHex(),palette:f.workcell.palette,
      lights:f.workcell.lights.map(l=>({color:l.color.getHex(),intensity:l.intensity,visible:l.visible}))});
    const themeColors={};
    for(const id of ['night','hangar']){
      f.controller.setScene(id,{persist:false});const tokens=themeTokens(id),palette=f.workcell.palette;
      assert.equal(palette.chartForce.length,2);assert.equal(new Set(palette.chartForce).size,2);assert(palette.chartTorque.length>=7);
      for(const color of [palette.chartGrid,...palette.chartForce,...palette.chartTorque])luminance(color);
      assert(Number.isFinite(palette.exposure)&&palette.exposure>0);
      for(const color of [f.scene.background,f.scene.fog.color,f.workcell.floor.material.color,...f.workcell.lights.map(light=>light.color)])
        assert(color.isColor&&[color.r,color.g,color.b].every(value=>Number.isFinite(value)&&value>=0));
      const textPairs=[['--ink','--panel'],['--muted','--panel'],['--selected-ink','--selected-field'],['--action-ink','--action']];
      const ratios=Object.fromEntries(textPairs.map(([fg,bg])=>{const ratio=contrast(tokens[fg],tokens[bg]);assert(ratio>=4.5,`${id}: ${fg}/${bg} normal text contrast ${ratio}`);return[`${fg}/${bg}`,ratio];}));
      assert(luminance(tokens['--page'])<.15&&luminance(tokens['--panel'])<.15,`${id}: both user-approved scenes must stay in the dark visual family`);
      const chartRatios=[...palette.chartForce,...palette.chartTorque].map(color=>contrast(color,tokens['--field']));assert(chartRatios.every(ratio=>ratio>=3),`${id}: measured chart series must remain legible on their actual CSS field token`);
      themeColors[id]={textContrast:ratios,minimumChartContrast:Math.min(...chartRatios),palette};
    }
    f.controller.setScene('night',{persist:false});const night=appearance();f.controller.setScene('hangar',{persist:false});const hangar=appearance();assert.notEqual(night,hangar);
    f.scene.updateMatrixWorld(true);assert.deepEqual(station.matrixWorld.toArray(),transform);assert.strictEqual(f.scene,identities.scene);assert.strictEqual(f.workcell.group,identities.group);assert.strictEqual(f.workcell.floor,identities.floor);assert.strictEqual(mesh.geometry,identities.geometry);assert.strictEqual(mesh.material,identities.material);
    observations.push({name:'real-scene-theme-appearance',night:JSON.parse(night),hangar:JSON.parse(hangar),themeColors,
      contrastScope:'Opaque CSS sRGB token pairs and actual palette values; not rendered-pixel colorimetry.'});
    mesh.geometry.dispose();mesh.material.dispose();
  });
  check('One hundred real scene switches preserve all scene, light, geometry and material identities',()=>{
    const f=make(),floor=f.workcell.floor,allowedTextures=new Set(),environmentTextures=new Set();
    // The two prebuilt floor maps/IBLs are legitimately exchanged. Their
    // finite owned pool must stay stable; a newly allocated map per switch
    // must not be mistaken for that intended two-theme resource selection.
    for(const id of ['hangar','night']){f.controller.setScene(id,{persist:false});for(const uuid of resources(f.scene).textures)allowedTextures.add(uuid);if(f.scene.environment)environmentTextures.add(f.scene.environment.uuid);}
    const initial=resources(f.scene),owned={...f.workcell.resourceStats};
    for(let i=0;i<100;i++){const id=i%2?'night':'hangar';f.controller.setScene(id,{persist:false});aligned(f,id);assert.strictEqual(f.workcell.floor,floor);
      const current=resources(f.scene);for(const uuid of current.textures)assert(allowedTextures.has(uuid));if(f.scene.environment)assert(environmentTextures.has(f.scene.environment.uuid));
      assert.deepEqual({...current,textures:[]},{...initial,textures:[]});assert.deepEqual(f.workcell.resourceStats,owned);}
    assert.equal(f.workcell.lights.filter(l=>l.castShadow).length,1);observations.push({name:'100-switch-resources',switches:100,nodeCount:initial.nodes.length,geometryCount:initial.geometry.length,materialCount:initial.materials.length,lightCount:initial.lights.length});
  });
  check('Switching in every task phase leaves declared run, queue, physics identity and clock state untouched',()=>{
    const f=make();for(const state of ['idle','preparing','planning','approach','closing','transfer','settling','stopped','failed']){
      const forbidden=()=>assert.fail('Theme switch must not invoke task or physics APIs');
      const sentinel={state,running:['planning','approach','closing','transfer','settling'].includes(state),planning:state==='planning',planningGeneration:7,planningProgress:{steps:69,elapsedMs:7391},runIds:['r1','g1','c5'],seed:521,completed:3,queue:['red4','yellow2'],history:['spread','align',state],
        tick:4371,time:4.371,accumulator:.003,lastFrame:12345,camera:[7,3,1],orbit:[.4,.8,0],qpos:[.5,1.2,-.7],ctrl:[.3,.1],model:{id:1},data:{id:2},
        start:forbidden,stop:forbidden,reset:forbidden,configureTask:forbidden,setObject:forbidden};
      f.scene.userData.declaredTaskState=sentinel;const before=JSON.stringify(sentinel),model=sentinel.model,data=sentinel.data;
      for(const id of ['hangar','night']){f.controller.setScene(id);aligned(f,id);assert.equal(JSON.stringify(sentinel),before);assert.strictEqual(sentinel.model,model);assert.strictEqual(sentinel.data,data);}
    }
    return {scope:'Unchanged declared task-state sentinels and production controller dependency isolation, not new physics execution.'};
  });
  check('Theme controls remain enabled during task execution and dispose removes their listeners',()=>{
    const f=make();assert(f.buttons.every(b=>b.listenerCount()>0));assert(f.buttons.every(b=>!b.disabled));f.controller.setScene('hangar');f.controller.dispose();
    assert(f.buttons.every(b=>b.listenerCount()===0));f.buttons[0].click();aligned(f,'hangar');
  });
  check('Reduced-motion styling preserves useful state and production fixed-step/measurement updates',()=>{
    assert(css.includes('prefers-reduced-motion:reduce')||css.includes('prefers-reduced-motion: reduce'));
    assert(!/animation-duration\s*:\s*0?\.0*1ms|transition-duration\s*:\s*0?\.0*1ms/.test(css),'Do not replace every useful state change with a global tiny-duration kill');
    assert(app.includes('station.animate(now,{reducedMotion})'));assert(app.includes('createFrameScheduler')&&/coordinator\.step\(dt\)/.test(app));
    assert(app.includes('history.force.push(m.forces)'));assert(app.includes('telemetry.joints.map(joint=>joint.torque)'));
  });
  check('Both named native scene controls expose independent accessible pressed states',()=>{
    for(const id of ['night','hangar'])assert(new RegExp(`<button[^>]*data-scene-switch="${id}"[^>]*aria-pressed="(?:true|false)"`).test(html),`${id}: missing real native labelled scene button`);
    assert(html.includes('深空指挥舱')&&html.includes('轨道机库'));assert(!html.includes('极昼实验室'));assert(!html.includes('data-scene-switch="day"'));assert(app.includes('createThemeController'));
    assert.deepEqual(STATION_OFFSETS,{r1:[0,0,1.6],g1:[0,0,0],c5:[0,0,-1.6]});assert(Object.isFrozen(STATION_VIEW_BOUNDS));
  });
}finally{for(const f of fixtures)f.dispose();}
const sourceFrozen=JSON.stringify(await hashes())===JSON.stringify(sourceHashes);if(!sourceFrozen)errors.push({name:'source freeze',message:'Theme sources changed during this suite'});
const report={pass:errors.length===0,total:checks.length,passed:checks.filter(c=>c.pass).length,checks,observations,sourceFrozen,sourceHashes,errors,
  scope:'Node-only actual production theme-controller and actual Three workcell contracts using declared DOM/storage/task sentinels. No browser engine, WebGL renderer, real physics, rendered pixels or successful grasp is asserted.',
  limitations:['Actual desktop/mobile main-scene and camera-feed pixels must be inspected for both themes in a separate native browser pass.','Reduced-motion OS/media-query behaviour and keyboard/touch interaction require actual browser verification.','The preserved full72 physical report is separate evidence, never relabelled as new theme or physics execution.']};
const destination=new URL(reportPath,root);await mkdir(new URL('./',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,total:report.total,passed:report.passed,sourceFrozen,errors},null,2));if(!report.pass)process.exitCode=1;
