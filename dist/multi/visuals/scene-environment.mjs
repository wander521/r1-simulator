import * as THREE from '../../vendor/three.module.min.js';

const palette=(value)=>Object.freeze({...value,chartForce:Object.freeze(value.chartForce),chartTorque:Object.freeze(value.chartTorque)});
export const SCENE_PALETTES=Object.freeze({
  night:palette({chartGrid:'#465e6d',chartForce:['#84dfd0','#edc584'],chartTorque:['#84dfd0','#9bbcff','#edc584','#efa28c','#d0b1ec','#aad3a0','#c2d7e3'],exposure:1.08}),
  hangar:palette({chartGrid:'#536473',chartForce:['#9bd7d8','#f0bc75'],chartTorque:['#9bd7d8','#a8c5ef','#f0bc75','#efa28c','#d0b1ec','#bad3a0','#c2d7e3'],exposure:1.08}),
});
const PRESETS=Object.freeze({
  night:{background:0x283d4b,floor:0x637785,floorMetalness:.43,floorRoughness:.40,environmentIntensity:1.06,
    housing:0x344c5b,inlay:0xc6a271,topColor:0xfffcf5,topIntensity:34,ambient:.20,hemi:1.12,key:1.60,fill:.65,rim:.40,rimColor:0xffd3a1,
    table:{top:0x7c929f,leg:0x4e6778,plinth:0x698493}},
  hangar:{background:0x293b49,floor:0x596d7c,floorMetalness:.48,floorRoughness:.44,environmentIntensity:1.06,
    housing:0x3d5363,inlay:0xbb9765,topColor:0xfffcf5,topIntensity:34,ambient:.20,hemi:1.12,key:1.60,fill:.65,rim:.48,rimColor:0xffc584,
    table:{top:0x788d9c,leg:0x4a6274,plinth:0x657e91}},
});

function floorMap(id){
  const width=128,data=new Uint8Array(width*width*4);
  for(let y=0;y<width;y++)for(let x=0;x<width;x++){
    const seam=id==='hangar'?(x<2||(y<1&&x<70)):(y<2||(x<1&&y<48));
    const value=seam?(id==='hangar'?142:158):238,offset=(y*width+x)*4;
    data.set([value,value,value,255],offset);
  }
  const texture=new THREE.DataTexture(data,width,width,THREE.RGBAFormat);
  texture.name=`${id}-architectural-floor-map`;texture.colorSpace=THREE.SRGBColorSpace;
  texture.wrapS=texture.wrapT=THREE.RepeatWrapping;texture.repeat.set(id==='hangar'?5:4,6);
  texture.magFilter=texture.minFilter=THREE.LinearFilter;texture.needsUpdate=true;return texture;
}

function reflectionMap(id){
  // Scene-light cards rather than a downloaded panorama. Values are linear HDR
  // radiance; the source images and their PMREM targets remain owned here.
  const width=256,height=128,data=new Float32Array(width*height*4),hangar=id==='hangar';
  for(let y=0;y<height;y++)for(let x=0;x<width;x++){
    const u=x/width,v=y/height,sky=Math.max(0,1-v),base=.09+.20*sky;
    let card=0;
    for(const centre of [.10,.40,.72]){
      const du=Math.min(Math.abs(u-centre),1-Math.abs(u-centre));
      const horizontal=Math.max(0,1-du/.095),vertical=Math.max(0,1-Math.abs(v-.29)/.19);
      card+=horizontal*vertical*3.6;
    }
    const lower=v>.69?.025:0,offset=(y*width+x)*4;
    // The work lamps add a restrained warm reflection below neutral softboxes.
    const workLamp=hangar?Math.max(0,1-Math.abs(v-.59)/.08)*Math.max(0,1-Math.abs(u-.52)/.14)*.32:0;
    data.set([base*.94+card+lower+workLamp,base*1.04+card+lower+workLamp*.62,base*1.12+card+lower+workLamp*.30,1],offset);
  }
  const half=Uint16Array.from(data,value=>THREE.DataUtils.toHalfFloat(value));
  const texture=new THREE.DataTexture(half,width,height,THREE.RGBAFormat,THREE.HalfFloatType);
  texture.name=`${id}-linear-softbox-environment`;texture.mapping=THREE.EquirectangularReflectionMapping;
  texture.colorSpace=THREE.LinearSRGBColorSpace;texture.magFilter=texture.minFilter=THREE.LinearFilter;
  texture.needsUpdate=true;return texture;
}

function resourceBook(){
  const geometries=new Set(),materials=new Set(),textures=new Set(),targets=new Set();
  return{geometries,materials,textures,targets,
    geometry:value=>(geometries.add(value),value),material:value=>(materials.add(value),value),texture:value=>(textures.add(value),value),
    dispose(){targets.forEach(value=>value.dispose());textures.forEach(value=>value.dispose());materials.forEach(value=>value.dispose());geometries.forEach(value=>value.dispose());},
  };
}
function geometryBudget(group){
  let drawCalls=0,triangles=0,points=0;
  group.traverse(node=>{if(!node.isMesh&&!node.isPoints)return;drawCalls++;
    if(node.isPoints){points+=node.geometry.attributes.position.count;return;}
    triangles+=(node.geometry.index?.count??node.geometry.attributes.position.count)/3*(node.isInstancedMesh?node.count:1);
  });return{drawCalls,triangles,points};
}

export function createSceneEnvironment(scene,{renderer,initialScene='night'}={}){
  if(!Object.hasOwn(PRESETS,initialScene))throw new RangeError('Unknown laboratory scene');
  const book=resourceBook(),group=new THREE.Group();group.name='multi-laboratory-environment';
  const common=new THREE.Group();common.name='shared-ground-and-overhead-rig';group.add(common);
  const stageSets={night:new THREE.Group(),hangar:new THREE.Group()};
  for(const[id,set]of Object.entries(stageSets)){set.name=`${id}-architectural-set`;group.add(set);}
  const previous={background:scene.background,environment:scene.environment,fog:scene.fog,
    environmentIntensity:scene.environmentIntensity,exposure:renderer?.toneMappingExposure};
  const standard=(color,metalness=.25,roughness=.5)=>book.material(new THREE.MeshStandardMaterial({color,metalness,roughness}));
  const basic=color=>book.material(new THREE.MeshBasicMaterial({color}));
  const box=(parent,name,size,position,material)=>{
    const mesh=new THREE.Mesh(book.geometry(new THREE.BoxGeometry(...size)),material);mesh.name=name;mesh.position.fromArray(position);parent.add(mesh);return mesh;
  };
  // Large architectural surfaces have an interior face only. Unlike a solid
  // box they cannot mask the robot work area when unrestricted Orbit is behind
  // the set; the small load-bearing frames remain genuinely three-dimensional.
  const face=(parent,name,width,height,position,material,rotationY=Math.PI/2)=>{
    const mesh=new THREE.Mesh(book.geometry(new THREE.PlaneGeometry(width,height)),material);mesh.name=name;mesh.position.fromArray(position);mesh.rotation.y=rotationY;parent.add(mesh);return mesh;
  };
  const doorLeaf=(parent,name,points,position,material)=>{
    const outline=new THREE.Shape();points.forEach(([z,y],index)=>index?outline.lineTo(-z,y):outline.moveTo(-z,y));outline.closePath();
    const mesh=new THREE.Mesh(book.geometry(new THREE.ShapeGeometry(outline)),material);mesh.name=name;mesh.rotation.y=Math.PI/2;mesh.position.fromArray(position);parent.add(mesh);return mesh;
  };
  const instances=(parent,name,size,positions,material)=>{
    const mesh=new THREE.InstancedMesh(book.geometry(new THREE.BoxGeometry(...size)),material,positions.length),matrix=new THREE.Matrix4();mesh.name=name;
    positions.forEach((position,index)=>mesh.setMatrixAt(index,matrix.makeTranslation(...position)));mesh.instanceMatrix.needsUpdate=true;parent.add(mesh);return mesh;
  };
  const maps={night:book.texture(floorMap('night')),hangar:book.texture(floorMap('hangar'))};
  const floorMaterial=standard(PRESETS.night.floor,.43,.40);floorMaterial.map=maps.night;floorMaterial.envMapIntensity=.7;
  const floor=new THREE.Mesh(book.geometry(new THREE.PlaneGeometry(12,12)),floorMaterial);floor.name='laboratory-ground';
  floor.rotation.x=-Math.PI/2;floor.position.y=-.012;floor.receiveShadow=true;common.add(floor);
  const rigMaterial=standard(0x697f8c,.72,.32),diffuserMaterial=basic(0xf5faff),inlayMaterial=standard(0x849aa5,.5,.5);
  const inlayPositions=[];
  for(const z of[1.6,0,-1.6])for(const[x,dz]of[[-.46,0],[.66,0]])inlayPositions.push([x,-.009,z+dz]);
  instances(common,'ground-workcell-long-inlays',[.012,.002,1.20],inlayPositions,inlayMaterial);
  instances(common,'ground-workcell-cross-inlays',[1.12,.002,.012],[1.6,0,-1.6].flatMap(z=>[[.10,-.009,z-.60],[.10,-.009,z+.60]]),inlayMaterial);
  const lampGeometry=book.geometry(new THREE.BoxGeometry(.96,.065,.86)),diffuserGeometry=book.geometry(new THREE.PlaneGeometry(.86,.76));
  for(const[index,z]of[1.6,0,-1.6].entries()){
    const lamp=new THREE.Mesh(lampGeometry,rigMaterial);lamp.name=`overhead-light-${index+1}-housing`;lamp.position.set(.05,2.72,z);common.add(lamp);
    const diffuser=new THREE.Mesh(diffuserGeometry,diffuserMaterial);diffuser.name=`overhead-light-${index+1}-diffuser`;diffuser.rotation.x=Math.PI/2;diffuser.position.set(.05,2.686,z);common.add(diffuser);
  }
  instances(common,'overhead-rig-suspension',[.035,.60,.035],[1.6,0,-1.6].flatMap(z=>[[-.33,3.04,z],[.43,3.04,z]]),rigMaterial);
  instances(common,'overhead-rig-rails',[.07,.06,5.12],[[-.33,3.34,0],[.43,3.34,0]],rigMaterial);

  const night=stageSets.night,nightWall=standard(0x344b5b,.42,.47),nightFrame=standard(0x6b8392,.7,.30),nightWindow=basic(0x142b3a),amber=basic(0xb28c58);
  face(night,'night-rear-bulkhead',7.8,3.30,[-1.89,1.64,0],nightWall);
  face(night,'night-observatory-aperture',6.96,1.76,[-1.8495,1.80,0],nightWindow);
  instances(night,'night-aperture-side-frames',[.13,1.94,.10],[[-1.77,1.80,-3.53],[-1.77,1.80,3.53]],nightFrame);
  instances(night,'night-aperture-cross-frames',[.13,.09,7.14],[[-1.77,.86,0],[-1.77,2.74,0]],nightFrame);
  instances(night,'night-structural-bay-ribs',[.14,2.72,.055],[-2.42,0,2.42].map(z=>[-1.73,1.47,z]),nightFrame);
  instances(night,'night-amber-lightwell-edges',[.018,1.62,.012],[-3.40,3.40].map(z=>[-1.68,1.80,z]),amber);
  box(night,'night-low-service-spine',[.30,.31,7.40],[-1.70,.145,0],nightFrame);
  for(const z of[-3.7,3.7])face(night,'night-splayed-open-side-fin',1.15,3.04,[-1.43-Math.sin(.22)*.065,1.52,z+(z<0?1:-1)*Math.cos(.22)*.065],nightWall,z<0?-.22:Math.PI+.22);
  const stars=new Float32Array(180*3);let randomState=521;
  for(let i=0;i<180;i++){randomState=(Math.imul(randomState,1664525)+1013904223)>>>0;const z=(randomState/0x100000000-.5)*6.65;randomState=(Math.imul(randomState,1664525)+1013904223)>>>0;stars.set([-1.838,.99+randomState/0x100000000*1.60,z],i*3);}
  const starGeometry=book.geometry(new THREE.BufferGeometry().setAttribute('position',new THREE.BufferAttribute(stars,3)));
  const starField=new THREE.Points(starGeometry,book.material(new THREE.PointsMaterial({color:0x98afbe,size:.012,sizeAttenuation:true})));starField.name='night-static-observatory-stars';night.add(starField);

  const hangar=stageSets.hangar,hangarWall=standard(0x3b5062,.48,.46),hangarDoor=standard(0x536a7e,.62,.38),hangarFrame=standard(0x8294a0,.72,.32),hangarAmber=basic(0xd5a366);
  face(hangar,'hangar-blue-steel-bulkhead',7.8,3.30,[-1.89,1.64,0],hangarWall);
  doorLeaf(hangar,'hangar-port-door-leaf',[[-3.33,.39],[.35,.39],[-.35,1.66],[.35,2.93],[-3.33,2.93]],[-1.85,0,0],hangarDoor);
  doorLeaf(hangar,'hangar-starboard-door-leaf',[[3.33,.39],[.47,.39],[-.23,1.66],[.47,2.93],[3.33,2.93]],[-1.85,0,0],hangarDoor);
  instances(hangar,'hangar-door-side-portal',[.15,2.78,.12],[-3.46,3.46].map(z=>[-1.75,1.66,z]),hangarFrame);
  instances(hangar,'hangar-door-cross-portal',[.15,.12,7.04],[[-1.75,.30,0],[-1.75,3.02,0]],hangarFrame);
  const seam=new THREE.InstancedMesh(book.geometry(new THREE.BoxGeometry(.025,.025,Math.hypot(1.20,.66))),hangarAmber,2),matrix=new THREE.Matrix4(),rotation=new THREE.Quaternion(),translation=new THREE.Vector3();
  seam.name='hangar-diagonal-door-worklights';
  for(const[index,[y,z,angle]]of[[1.03,.07,Math.atan2(1.20,.66)],[2.29,.07,-Math.atan2(1.20,.66)]].entries()){
    rotation.setFromAxisAngle(new THREE.Vector3(1,0,0),angle);translation.set(-1.82,y,z);matrix.compose(translation,rotation,new THREE.Vector3(1,1,1));seam.setMatrixAt(index,matrix);
  }seam.instanceMatrix.needsUpdate=true;hangar.add(seam);
  instances(hangar,'hangar-overdoor-amber-worklights',[.025,.045,1.18],[-2.25,0,2.25].map(z=>[-1.66,2.84,z]),hangarAmber);
  const brace=new THREE.InstancedMesh(book.geometry(new THREE.BoxGeometry(.12,.085,1.72)),hangarFrame,4);brace.name='hangar-diagonal-load-braces';
  for(const[index,[y,z,angle]]of[[2.39,-2.55,-.43],[.88,-2.55,.43],[2.39,2.55,.43],[.88,2.55,-.43]].entries()){
    rotation.setFromAxisAngle(new THREE.Vector3(1,0,0),angle);translation.set(-1.73,y,z);matrix.compose(translation,rotation,new THREE.Vector3(1,1,1));brace.setMatrixAt(index,matrix);
  }brace.instanceMatrix.needsUpdate=true;hangar.add(brace);
  box(hangar,'hangar-grounded-door-service-rail',[.30,.25,7.40],[-1.70,.115,0],hangarFrame);
  for(const z of[-3.7,3.7])face(hangar,'hangar-open-portal-return',1.15,3.04,[-1.43,1.52,z],hangarWall,z<0?-.22:Math.PI+.22);

  const lights=[],addLight=light=>(lights.push(light),common.add(light),light);
  const ambient=addLight(new THREE.AmbientLight(0xffffff,.26)),hemi=addLight(new THREE.HemisphereLight(0xe8f4ff,0x586b79,1.65));
  const key=addLight(new THREE.DirectionalLight(0xfff8ee,2.05));key.name='laboratory-shadow-key';key.position.set(3.5,5.8,2.4);key.target.position.set(.25,.8,0);common.add(key.target);key.castShadow=true;
  key.shadow.mapSize.set(2048,2048);Object.assign(key.shadow.camera,{left:-4.25,right:4.25,top:3.50,bottom:-3.50,near:.1,far:15});key.shadow.camera.updateProjectionMatrix();key.shadow.bias=-.00008;key.shadow.normalBias=.002;key.shadow.radius=3;
  const fill=addLight(new THREE.DirectionalLight(0xd9eeff,.85));fill.position.set(2.3,2.7,-3.7);fill.target.position.set(.20,.8,0);common.add(fill.target);
  const rim=addLight(new THREE.DirectionalLight(0xe8f4ff,.50));rim.position.set(-2.4,2.9,2.3);rim.target.position.set(.15,1,0);common.add(rim.target);
  const topLights=[];
  for(const[index,z]of[1.6,0,-1.6].entries()){
    const top=addLight(new THREE.SpotLight(0xf5faff,22,7,.66,.78,2));top.name=`laboratory-overhead-spot-${index+1}`;top.position.set(.05,2.66,z);top.target.position.set(.15,.25,z);common.add(top.target);topLights.push(top);
  }
  const rawEnvironments={night:book.texture(reflectionMap('night')),hangar:book.texture(reflectionMap('hangar'))},environments={...rawEnvironments};
  let environmentMode='procedural-linear-equirectangular',environmentError=null;
  if(renderer?.isWebGLRenderer){
    let generator;
    const renderTarget=renderer.getRenderTarget(),cubeFace=renderer.getActiveCubeFace(),mipmapLevel=renderer.getActiveMipmapLevel();
    try{generator=new THREE.PMREMGenerator(renderer);generator.compileEquirectangularShader();for(const id of['night','hangar']){const target=generator.fromEquirectangular(rawEnvironments[id]);book.targets.add(target);environments[id]=target.texture;}environmentMode='prebaked-pmrem';}
    catch(error){
      book.targets.forEach(target=>target.dispose());book.targets.clear();Object.assign(environments,rawEnvironments);
      environmentMode='procedural-linear-equirectangular-fallback';environmentError=error.message;
    }finally{generator?.dispose();renderer.setRenderTarget(renderTarget,cubeFace,mipmapLevel);}
  }
  const backgrounds=Object.fromEntries(Object.entries(PRESETS).map(([id,preset])=>[id,new THREE.Color(preset.background)])),fogs=Object.fromEntries(Object.entries(PRESETS).map(([id,preset])=>[id,new THREE.Fog(preset.background,12,32)]));
  const tableMaterials=new Map();let activeId=null,disposed=false;
  function styleTable(material,role,id){
    const preset=PRESETS[id];material.color.set(preset.table[role]);material.metalness=role==='top'?.38:.62;material.roughness=role==='top'?.36:.34;material.envMapIntensity=.82;
  }
  const api={group,floor,grid:floor,lights,table:new THREE.Group(),
    get activeId(){return activeId;},get palette(){return SCENE_PALETTES[activeId];},get environmentMode(){return environmentMode;},get environmentError(){return environmentError;},
    get resourceStats(){return{geometries:book.geometries.size,materials:book.materials.size,textures:book.textures.size,environmentTargets:book.targets.size,
      common:geometryBudget(common),night:geometryBudget(night),hangar:geometryBudget(hangar),lights:lights.length,topLights:topLights.length,shadowLights:lights.filter(light=>light.castShadow).length};},
    registerStationTable(station,table=station.group.getObjectByName(`${station.id}-grounded-worktable`)){
      if(disposed)return false;if(!table?.isObject3D||table.parent!==station.group)throw new TypeError('A station-owned worktable is required');
      const register=(material,role)=>{if(!tableMaterials.has(material))tableMaterials.set(material,{role,color:material.color.clone(),metalness:material.metalness,roughness:material.roughness,envMapIntensity:material.envMapIntensity});styleTable(material,role,activeId);};
      table.traverse(node=>{if(node.isMesh)for(const material of(Array.isArray(node.material)?node.material:[node.material]))register(material,node.name===`${station.id}-tabletop`?'top':'leg');});
      const plinth=station.group.getObjectByName('c5-grounded-mount');if(plinth?.material)register(plinth.material,'plinth');return true;
    },
    setScene(id){
      if(disposed)return false;if(!Object.hasOwn(PRESETS,id))throw new RangeError('Unknown laboratory scene');
      const preset=PRESETS[id];activeId=id;night.visible=id==='night';hangar.visible=id==='hangar';
      floorMaterial.color.set(preset.floor);floorMaterial.metalness=preset.floorMetalness;floorMaterial.roughness=preset.floorRoughness;floorMaterial.map=maps[id];
      rigMaterial.color.set(preset.housing);inlayMaterial.color.set(preset.inlay);diffuserMaterial.color.set(preset.topColor);
      ambient.intensity=preset.ambient;hemi.intensity=preset.hemi;key.intensity=preset.key;fill.intensity=preset.fill;rim.intensity=preset.rim;rim.color.set(preset.rimColor);
      for(const top of topLights){top.color.set(preset.topColor);top.intensity=preset.topIntensity;}
      scene.background=backgrounds[id];scene.environment=environments[id];scene.environmentIntensity=preset.environmentIntensity;scene.fog=fogs[id];
      if(renderer){renderer.toneMappingExposure=SCENE_PALETTES[id].exposure;renderer.shadowMap&&(renderer.shadowMap.needsUpdate=true);}
      for(const[material,{role}]of tableMaterials)styleTable(material,role,id);return true;
    },
    dispose(){
      if(disposed)return;disposed=true;group.removeFromParent();
      for(const[material,original]of tableMaterials){material.color.copy(original.color);for(const key of['metalness','roughness','envMapIntensity'])material[key]=original[key];}tableMaterials.clear();
      scene.background=previous.background;scene.environment=previous.environment;scene.environmentIntensity=previous.environmentIntensity;scene.fog=previous.fog;
      if(renderer)renderer.toneMappingExposure=previous.exposure;lights.forEach(light=>light.dispose?.());book.dispose();
    },
  };
  api.table.name='unused-shared-worktable';scene.add(group);api.setScene(initialScene);return api;
}
