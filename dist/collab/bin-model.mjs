import {ROBOT_IDS,sourceRobot,boxGeom,fixedTable,partXml} from './model.mjs';
import {cabinetFixtures,cabinetPlinths,CELL_PART_TYPES,RACK_STRUCTURE} from './cell-fixtures.mjs?v=20261026-load-2';

export const BIN_MODEL_VERSION='20261026-rack-support-1';
const vector=v=>v.join(' ');
// All dimensions are metres in MuJoCo Z-up. The origin is the pinch handle,
// NOT the bin floor: native grasp IK targets this real, colliding feature.
export const CARRIER=Object.freeze({floorOffset:[.025,0,-.100],floorHalf:[.055,.075,.004],bottomOffset:-.104,handleHalf:[.016,.016,.022],mass:.09});
function carrierGeometry(id){
  return [
    {name:`${id}_floor`,position:CARRIER.floorOffset,half:CARRIER.floorHalf,color:'.22 .32 .39 1'},
    {name:`${id}_handle`,position:[0,0,0],half:CARRIER.handleHalf,color:'.92 .65 .16 1'},
    {name:`${id}_post`,position:[0,0,-.054],half:[.012,.012,.042],color:'.28 .34 .4 1'},
    ...[[.025,-.075,.055,.004],[.025,.075,.055,.004],[-.03,0,.004,.075],[.08,0,.004,.075]].map(([x,y,a,b],i)=>({name:`${id}_rim_${i}`,position:[x,y,-.08],half:[a,b,.020],color:'.34 .45 .52 1'})),
  ];
}
function staticSupport(id,position,half,height){return {id,body:id,geom:`${id}_top`,center:[...position,height-.0325],halfExtents:[...half,.0325],tableHeight:height};}

/** New physical topology, kept separate until the bin grasp vertical slice is
 * verified. The live /collab entry point is not silently switched to an
 * unvalidated task. No carrier motor, weld, mocap or scripted payload motion. */
export function buildBinCollaborationModel(sources,{binCount=4}={}){
  // Two reachable shelf levels for fixed-foot G1. Do not silently populate a
  // third tier beyond its validated natural arm workspace.
  if(!Number.isInteger(binCount)||binCount<1||binCount>4)throw new Error('binCount must be 1–4 (two reachable wall levels)');
  const height=.79;
  const layouts={
    g1:{origin:[0,-.65,0],yaw:0,root:'pelvis'},
    r1:{origin:[.38,.40,.208],yaw:-Math.PI/2,root:'base_link'},
    c5:{origin:[2.35,-.65,.18],yaw:0,root:'base_link'},
  };
  const robots=ROBOT_IDS.map(id=>sourceRobot(sources[id],id,layouts[id],{interRobotContacts:true,mobileXMin:-.3,mobileXMax:2.9,mobileYMin:-.11,mobileYMax:.48}));
  for(const robot of robots)robot.metadata.binFlow=true;
  robots.find(r=>r.metadata.id==='g1').metadata.entryShoulderRoll=1.35;
  robots.find(r=>r.metadata.id==='g1').metadata.operationReturn='ready';
  robots.find(r=>r.metadata.id==='r1').metadata.operationReturn='ready';
  robots.find(r=>r.metadata.id==='c5').metadata.operationReturn='ready';
  robots.find(r=>r.metadata.id==='g1').metadata.readyShoulderPitch=1;
  for(const robot of robots)if(robot.defaultXml!==robots[0].defaultXml||JSON.stringify(robot.option)!==JSON.stringify(robots[0].option))throw new Error('Source physics contracts differ');
  const supports=[staticSupport('g1_out_bench',[.55,-.40],[.27,.12],height),staticSupport('c5_input_bench',[2.80,-.23],[.31,.13],height),staticSupport('c5_output_bench',[2.87,-.82],[.24,.25],height),staticSupport('empty_return_bench',[1.35,-.33],[.49,.24],height),staticSupport('c5_link_bench',[2.87,-.465],[.24,.105],height)];
  const slots=[];
  for(const [kind,centres]of Object.entries({full:[[.32,-.40],[.44,-.43]],input:[[2.61,-.23],[2.83,-.23]],returned:Array.from({length:binCount},(_,i)=>[1.02+(i%3)*.24,-.23-Math.floor(i/3)*.20]),output:Array.from({length:4},(_,i)=>[2.73+(i%2)*.21,-.66-Math.floor(i/2)*.16])})){
    for(const [i,p]of centres.entries())slots.push({id:`${kind==='returned'?'return':kind}-${i+1}`,kind,position:[...p,height-CARRIER.bottomOffset],supportBody:kind==='full'?'g1_out_bench':kind==='input'?'c5_input_bench':kind==='returned'?'empty_return_bench':'c5_output_bench',partSlots:[[-.035,0,.026],[.035,0,.026]]});
  }
  const outputs=slots.filter(s=>s.kind==='output');
  outputs.forEach((s,i)=>Object.assign(s,CELL_PART_TYPES[i]));
  const bins=[],parts=[],rack=[];
  for(let i=0;i<binCount;i++){
    const id=`bin-${i+1}`,level=Math.floor(i/2),y=-.95+(i%2)*.22,shelfHeight=.79+level*.21,start=[.44,y,shelfHeight-CARRIER.bottomOffset];
    // Keep the G1-facing edge at x=.4, but extend the rear to x=.656 so
    // the tray support joins the backing (front x=.652) instead of floating.
    const shelf={body:`wall_shelf_${i+1}`,geom:`wall_shelf_${i+1}_top`,center:[.528,y,shelfHeight-.012],halfExtents:[.128,.105,.012],tableHeight:shelfHeight};rack.push(shelf);
    const geometries=carrierGeometry(`carrier_${i+1}`),bin={id,body:`carrier_${i+1}`,joint:`carrier_${i+1}_free`,source:`wall-${i+1}`,sourceSupport:shelf.body,start,geometry:geometries,carrier:true,shape:'box',halfX:CARRIER.handleHalf[0],halfZ:CARRIER.handleHalf[1],h:CARRIER.handleHalf[2],mass:CARRIER.mass,parts:[],payloadBodies:[]};
    for(let j=0;j<2;j++){
      const type=CELL_PART_TYPES[(i%2)*2+j],radius=type.shape==='box'?.014:.017;
      const kind=j?'sleeve':'pin',pid=`${id}-${kind}`,part={id:pid,binId:id,orderId:id,body:`part_${i+1}_${j+1}`,joint:`part_${i+1}_${j+1}_free`,geom:`part_${i+1}_${j+1}_geom`,kind,index:0,...type,halfExtents:[radius,radius,.023],halfX:radius,halfZ:radius,h:.023,mass:.025,start:[start[0]+.04,start[1]+(j?.04:-.04),shelfHeight+.008+.023],sourceSupport:bin.body};
      bin.parts.push(pid);bin.payloadBodies.push(part.body);parts.push(part);
    }bins.push(bin);
  }
  // Immutable per-category destinations, independent of the incoming bin.
  // Each category has two separate contact-supported positions, never a stack.
  for(const slot of outputs){
    const members=parts.filter(p=>p.category===slot.category);
    if(members.length>slot.partSlots.length)throw new Error('分类区域容量不足');
    members.forEach((p,i)=>{p.outputSlot=slot.id;p.outputIndex=i;});
  }
  // The carriage is explicitly an ideal force-driven planar base, not a
  // claim of wheel-ground dynamics. This preserves the established R1 scope.
  const platform={body:'r1_mobile_platform',geom:'r1_mobile_platform_top',center:[1.55,.595,.08325],halfExtents:[1.65,.675,.08325],topHeight:.1665};
  const mobility={robotId:'r1',mountName:'r1__mount',model:'ideal-planar-actuated-base-not-wheel-traction',
    joints:{x:'r1__base_slide_x',y:'r1__base_slide_y',yaw:'r1__base_yaw'},actuators:{x:'r1__base_slide_x_motor',y:'r1__base_slide_y_motor',yaw:'r1__base_yaw_motor'},
    baseXRange:[-.3,2.9],baseYRange:[-.11,.48],aisleY:.85,maxSpeed:.12,dockTolerance:{position:.004,yaw:.008,speed:.01},platform,
    workingReach:.73,workingLateral:.10,
    docks:Object.fromEntries(slots.filter(s=>s.kind!=='output').map(s=>[s.id,{position:[s.position[0]+.10,s.position[1]+.73,.208],yaw:-Math.PI/2}])),
    actuationAssumption:'Ideal force-limited planar servos; original wheel joints retained, wheel traction not simulated.'};
  const geomXml=g=>boxGeom(g.name,g.position,g.half,g.color);
  const freeBins=bins.map(bin=>`<body name="${bin.body}" pos="${vector(bin.start)}"><freejoint name="${bin.joint}"/><inertial pos=".025 0 -.071" mass="${bin.mass}" diaginertia=".00032 .00028 .00029"/>${bin.geometry.map(geomXml).join('')}</body>`).join('');
  // Fixed finished boxes represent fixtures with finite capacity. Their parts
  // remain loose free bodies; no constraint attaches a packed part to a box.
  const outputFixtures=slots.filter(s=>s.kind==='output').map(s=>{
    const p=[s.position[0],s.position[1],height+.003];s.partSlots=s.partSlots.map(v=>[p[0]+v[0],p[1]+v[1],p[2]+.003+.023]);
    return `<body name="fixture_${s.id}" pos="${vector(p)}">${boxGeom(`fixture_${s.id}_floor`,[0,0,0],[.09,.065,.003],'.35 .42 .48 1')}</body>`;
  }).join('');
  const fixtures=cabinetFixtures(supports),plinths=cabinetPlinths(supports);
  const cell=[
    '<geom name="cell_floor" type="plane" size="6 4 .1" rgba=".15 .18 .22 1" contype="1" conaffinity="511"/>',
    ...supports.map(s=>`<body name="${s.body}">${boxGeom(s.geom,s.center,s.halfExtents,'.24 .29 .33 1')}</body>`),
    ...[...fixtures,...plinths].map(g=>`<body name="${g.name}">${geomXml(g)}</body>`),
    ...rack.map(s=>`<body name="${s.body}">${boxGeom(s.geom,s.center,s.halfExtents,'.25 .32 .38 1')}</body>`),
    `<body name="wall_rack">${RACK_STRUCTURE.map(geomXml).join('')}</body>`,
    `<body name="${platform.body}">${boxGeom(platform.geom,platform.center,platform.halfExtents,'.19 .22 .26 1')}</body>`,
    '<body name="c5_pedestal_body"><geom name="c5_grounded_pedestal" type="cylinder" pos="2.35 -.65 .45" size=".12 .45" rgba=".26 .29 .33 1" contype="1" conaffinity="511"/></body>',
    freeBins,outputFixtures,...parts.map(partXml),...robots.map(r=>r.body),
  ].join('\n');
  // Elliptic soft friction at the default impedance ratio slowly creeps
  // during long holds despite sufficient force. The same-grasp 63 s A/B test
  // selected 10: no change to friction coefficients, mass or force limits.
  // https://mujoco.readthedocs.io/en/latest/modeling.html#preventing-slip
  const xml=`<mujoco model="bin_supply_transport_kitting"><compiler angle="radian" autolimits="true" fusestatic="false" inertiafromgeom="false"/><option timestep=".001" integrator="implicitfast" solver="Newton" iterations="60" cone="elliptic" impratio="10" gravity="0 0 -9.81"/><default>${robots[0].defaultXml}</default><asset>${robots.map(r=>r.asset).join('')}</asset><worldbody>${cell}</worldbody><contact>${robots.map(r=>r.contact).join('')}</contact><actuator>${robots.map(r=>r.actuator).join('')}</actuator></mujoco>`;
  return {xml,metadata:{version:BIN_MODEL_VERSION,coordinateSystem:'MuJoCo Z-up metres',singleSharedWorld:true,contactSolver:{cone:'elliptic',impratio:10,sourceFrictionAndForceLimitsUnchanged:true},tableHeight:height,robots:Object.fromEntries(robots.map(r=>[r.metadata.id,r.metadata])),workbench:supports[1],supports,fixtures,plinths,rack,rackStructure:RACK_STRUCTURE,slots,bins,parts,mobility,carrier:CARRIER,safety:{partAttachmentConstraints:false,carrierActuation:false,r1FullBodyCollision:false}}};
}
