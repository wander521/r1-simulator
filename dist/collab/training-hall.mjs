import * as THREE from '../vendor/three.module.min.js';

export const TRAINING_HALL_TITLE='一脑多型具身训练场';

/** Display-only surroundings, outside the workcell. Never part of physics,
 * planning, task inventory or the camera's automatic workcell bounds. */
export function createTrainingHall({parent,box,geometry,material,materials,textures}){
  const hall=new THREE.Group();hall.name='training-hall-background';
  hall.position.set(1.45,0,.25);hall.rotation.y=Math.atan2(-1.15,-1.35);parent.add(hall);
  const steel=material(0x304958,.4,.6),trim=material(0x597482,.55,.42),inset=material(0x1b303d,.25,.65);
  const light=new THREE.MeshBasicMaterial({color:0xbbe6e4}),warm=new THREE.MeshBasicMaterial({color:0xdfc397});materials.add(light);materials.add(warm);
  const block=(name,size,pos,mat)=>box(hall,name,size,pos,mat,{shadow:false});
  // Single-sided architectural surfaces disappear from the reverse side,
  // leaving orbit inspection open rather than trapping the camera in a room.
  function plane(name,w,h,pos,mat){const m=new THREE.Mesh(geometry(new THREE.PlaneGeometry(w,h)),mat);m.name=name;m.position.fromArray(pos);hall.add(m);return m;}
  plane('hall-rear-wall',8,3.65,[0,1.825,-2.8],steel);
  // Modular sheet-metal cladding and a cable tray identify a working factory,
  // while the central opening and transport aisle remain completely clear.
  for(let x=-3.8;x<4;x+=.4){
    block('wall-panel-seam',[.009,.52,.016],[x,.38,-2.77],inset);
    block('upper-panel-seam',[.009,.27,.016],[x,3.48,-2.77],inset);
  }
  block('overhead-cable-tray',[8,.10,.19],[0,3.68,-2.66],inset);
  for(let x=-3.7;x<4;x+=.5)block('cable-tray-bracket',[.025,.17,.23],[x,3.62,-2.65],trim);
  block('hall-skirt',[8,.12,.04],[0,.065,-2.77],inset);
  for(const x of[-3.9,-2,2,3.9]){
    block('hall-structural-pier',[.08,3.65,.07],[x,1.825,-2.73],trim);
    block('pier-service-conduit',[.018,1.7,.018],[x+.06,1.65,-2.68],trim);
  }
  // Observation bays: solid tinted glass, not noisy transparency or fake feeds.
  for(const x of[-2.95,0,2.95]){
    const width=x===0?3.65:1.65;
    plane('observation-glazing',width,1.3,[x,1.43,-2.71],inset);
    for(const y of[.76,2.10])block('window-cross-member',[width+.06,.035,.04],[x,y,-2.66],trim);
    for(const dx of[-width/2,width/2])block('window-mullion',[.035,1.36,.04],[x+dx,1.43,-2.66],trim);
    block('observation-sill',[width+.12,.06,.24],[x,.73,-2.59],steel);
  }
  // The user-supplied title is drawn at native texture resolution.
  const canvas=document.createElement('canvas');canvas.width=2048;canvas.height=256;
  const ctx=canvas.getContext('2d');ctx.fillStyle='#132e3a';ctx.fillRect(0,0,2048,256);
  ctx.fillStyle='#edf7f5';ctx.font='600 156px "PingFang SC", "Microsoft YaHei", sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(TRAINING_HALL_TITLE,1024,132);
  const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;textures.add(texture);
  const sign=new THREE.MeshBasicMaterial({map:texture,toneMapped:false});materials.add(sign);
  plane('training-hall-banner',5.2,.65,[0,2.60,-2.60],sign);
  block('banner-top-rail',[5.32,.045,.06],[0,2.955,-2.62],trim);
  block('banner-bottom-light',[5.2,.018,.025],[0,2.245,-2.58],light);
  for(const x of[-2.70,2.70])block('banner-end-cap',[.045,.74,.06],[x,2.60,-2.62],warm);

  // Sparse perimeter equipment anchors the scale without adding task objects.
  for(const x of[-3.25,3.25]){
    block('perimeter-equipment-base',[.74,.12,.65],[x,.06,-1.72],inset);
    block('perimeter-service-cabinet',[.68,1.25,.58],[x,.745,-1.72],steel);
    block('service-door',[.57,1.06,.015],[x,.76,-1.421],inset);
    block('service-handle',[.018,.18,.025],[x+.20,.82,-1.40],trim);
    for(let i=0;i<5;i++)block('cabinet-vent',[.35,.012,.012],[x,.36+i*.035,-1.40],trim);
    block('cabinet-status-lamp',[.08,.02,.012],[x-.17,1.16,-1.40],light);
  }
  // Ground the set with a perimeter walkway and restrained slab joints.
  const floor=material(0x3d5361,.15,.75),joint=material(0x283e4a,.1,.8);
  const pad=plane('hall-floor-inlay',8,6.8,[0,-.009,-.2],floor);pad.rotation.x=-Math.PI/2;
  for(const x of[-3.9,-2.6,2.6,3.9])block('perimeter-slab-joint',[.012,.001,6.6],[x,-.007,-.2],joint);
  for(const z of[-2.65,2.85]){
    block('perimeter-walkway-line',[7.8,.002,.022],[0,-.006,z],trim);
    for(const x of[-3.2,3.2])block('floor-guide-light',[.58,.003,.028],[x,-.004,z+.045],warm);
  }
  return hall;
}
