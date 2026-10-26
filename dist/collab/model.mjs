/**
 * One physical workcell, not three visual copies of independent simulations.
 * All positions in this file use MuJoCo's metre-scale Z-up world frame.
 * The original robot transforms, inertias, joints, actuators and contact
 * settings are retained. New high collision bits enable inter-robot contact
 * without changing any original within-robot collision pair.
 */
export const COLLAB_MODEL_VERSION='20261023-shared-1';
export const ROBOT_IDS=Object.freeze(['r1','g1','c5']);
export const DEFAULT_LAYOUT=Object.freeze({
  tableHeight:.90,
  robots:Object.freeze({
    r1:Object.freeze({origin:Object.freeze([0,1.55,.208]),yaw:-Math.PI/2,root:'base_link',collisionBit:64}),
    g1:Object.freeze({origin:Object.freeze([0,-.85,0]),yaw:0,root:'pelvis',collisionBit:128}),
    c5:Object.freeze({origin:Object.freeze([0,.55,.18]),yaw:0,root:'base_link',collisionBit:256}),
  }),
  handoff:Object.freeze([.53,.82,.90]),
  trayCenter:Object.freeze([.38,-.67,.90]),
  c5TrayCenter:Object.freeze([.53,.31,.90]),
});

const ATTRIBUTE=/([\w:.-]+)\s*=\s*("[^"]*"|'[^']*')/g;
const REFERENCE_TYPES=Object.freeze({
  body:'body',body1:'body',body2:'body',target:'body',joint:'joint',joint1:'joint',joint2:'joint',
  geom:'geom',geom1:'geom',geom2:'geom',site:'site',site1:'site',site2:'site',
  mesh:'mesh',material:'material',texture:'texture',hfield:'hfield',tendon:'tendon',
  actuator:'actuator',sensor:'sensor',camera:'camera',class:'class',childclass:'class',
});
const ROOT_SECTIONS=new Set(['compiler','option','default','asset','worldbody','contact','actuator']);
const NAME_TYPES=Object.freeze({body:'body',joint:'joint',freejoint:'joint',geom:'geom',mesh:'mesh',material:'material',texture:'texture',hfield:'hfield',site:'site',camera:'camera',light:'light'});
const escapeAttribute=value=>String(value).replaceAll('&','&amp;').replaceAll('"','&quot;').replaceAll('<','&lt;');
const number=value=>Number(value.toPrecision(15)).toString();
const vector=value=>value.map(number).join(' ');
const finiteVector=(value,label)=>{
  if(!Array.isArray(value)||value.length!==3||value.some(n=>!Number.isFinite(n)))throw new Error(`${label} must be a finite Z-up position`);
  return [...value];
};

/** A small non-evaluating MJCF reader; no DOM and no external entities. */
function parse(xml){
  if(typeof xml!=='string'||!xml.trim())throw new Error('Missing original robot MJCF');
  if(/<!DOCTYPE|<!ENTITY|<include\b/i.test(xml))throw new Error('External MJCF includes/entities are not supported');
  const document={tag:'#document',children:[]},stack=[document];
  const token=/<\!--[\s\S]*?-->|<\?[\s\S]*?\?>|<[^>]*>/g;
  let match;
  while((match=token.exec(xml))){
    const raw=match[0];if(raw.startsWith('<!--')||raw.startsWith('<?'))continue;
    if(raw.startsWith('</')){
      const tag=raw.slice(2,-1).trim(),node=stack.pop();
      if(!node||node.tag!==tag)throw new Error(`Unbalanced MJCF closing tag ${tag}`);
      node.end=token.lastIndex;node.closeStart=match.index;continue;
    }
    const tag=raw.match(/^<([\w:.-]+)/)?.[1];if(!tag)throw new Error('Unsupported MJCF markup');
    const attrs=Object.fromEntries([...raw.matchAll(ATTRIBUTE)].map(m=>[m[1],m[2].slice(1,-1)]));
    const node={tag,attrs,start:match.index,openEnd:token.lastIndex,end:token.lastIndex,children:[]};
    stack.at(-1).children.push(node);if(!raw.endsWith('/>'))stack.push(node);
  }
  if(stack.length!==1||document.children.length!==1||document.children[0].tag!=='mujoco')throw new Error('Invalid MJCF document');
  return document.children[0];
}

function visit(node,callback){callback(node);for(const child of node.children)visit(child,callback);}
function section(root,tag){const items=root.children.filter(node=>node.tag===tag);if(items.length>1)throw new Error(`Duplicate MJCF section ${tag}`);return items[0];}
function inner(xml,node){return node?xml.slice(node.openEnd,node.closeStart):'';}
export const collabName=(robotId,name)=>`${robotId}__${name}`;

export function localToShared(position,frame){
  const [x,y,z]=finiteVector(position,'position'),c=Math.cos(frame.yaw??0),s=Math.sin(frame.yaw??0);
  return [frame.origin[0]+c*x-s*y,frame.origin[1]+s*x+c*y,frame.origin[2]+z];
}
export function sharedToLocal(position,frame){
  const [x,y,z]=finiteVector(position,'position').map((v,i)=>v-frame.origin[i]),c=Math.cos(frame.yaw??0),s=Math.sin(frame.yaw??0);
  return [c*x+s*y,-s*x+c*y,z];
}

function rewrite(xml,nodes,id,names,{interRobotContacts=true}={}){
  const ownBit=DEFAULT_LAYOUT.robots[id].collisionBit,otherBits=ROBOT_IDS.filter(x=>x!==id).reduce((bits,x)=>bits|DEFAULT_LAYOUT.robots[x].collisionBit,0);
  const rewriteNode=node=>{
    let raw=xml.slice(node.start,node.openEnd);
    raw=raw.replace(ATTRIBUTE,(whole,key,quoted)=>{
      const value=quoted.slice(1,-1);let type;
      if(key==='name')type=node.nameType;
      else if(key==='class'&&node.tag==='default')type='class';
      else if(key==='objname')type=node.attrs.objtype;
      else if(key==='refname')type=node.attrs.reftype;
      else type=REFERENCE_TYPES[key];
      if(!type)return whole;
      if(!names[type]?.[value])throw new Error(`${id} has unresolved ${type} reference ${key}=${value}`);
      return `${key}="${escapeAttribute(names[type][value])}"`;
    });
    if(node.tag==='geom'&&interRobotContacts){
      // Source defaults have zero contact bits. Only extant colliders acquire
      // cross-robot bits; purely visual/no-contact geoms remain non-colliding.
      const contype=Number(node.attrs.contype??0),affinity=Number(node.attrs.conaffinity??0);
      if(contype||affinity){
        raw=raw.replace(/\scontype\s*=\s*(?:"[^"]*"|'[^']*')/,'').replace(/\sconaffinity\s*=\s*(?:"[^"]*"|'[^']*')/,'');
        raw=raw.replace(/\s*\/>$/,` contype="${contype|ownBit}" conaffinity="${affinity|otherBits}"/>`);
      }
    }
    if(!node.children.length&&node.closeStart===undefined)return raw;
    let cursor=node.openEnd;
    for(const child of node.children){raw+=xml.slice(cursor,child.start)+rewriteNode(child);cursor=child.end;}
    return raw+xml.slice(cursor,node.end);
  };
  return nodes.map(rewriteNode).join('\n');
}

export function sourceRobot(xml,id,layout,options){
  const root=parse(xml);
  for(const node of root.children)if(!ROOT_SECTIONS.has(node.tag))throw new Error(`${id}: unsupported top-level MJCF section ${node.tag}`);
  const world=section(root,'worldbody'),robot=world?.children.find(node=>node.tag==='body'&&node.attrs.name===layout.root);
  if(!robot)throw new Error(`${id}: original robot root ${layout.root} is missing`);
  const asset=section(root,'asset'),contact=section(root,'contact'),actuator=section(root,'actuator');
  const names={},count={};
  const register=(node,type)=>{
    node.nameType=type;const name=node.attrs.name;
    if(!name)return;names[type]??={};if(names[type][name])throw new Error(`${id}: duplicate ${type} ${name}`);
    names[type][name]=collabName(id,name);count[type]=(count[type]??0)+1;
  };
  visit(robot,node=>register(node,NAME_TYPES[node.tag]));
  if(asset)for(const node of asset.children)visit(node,item=>register(item,NAME_TYPES[item.tag]));
  if(actuator)for(const node of actuator.children)register(node,'actuator');
  // These generated source files share an unnamed default geom. Refuse future
  // changes until their own default inheritance is merged deliberately.
  const defaults=section(root,'default');
  if(defaults?.attrs.class||defaults?.children.some(node=>node.tag!=='geom'||node.attrs.class))throw new Error(`${id}: unsupported named/nested default`);
  const compiler=section(root,'compiler'),option=section(root,'option');
  let body= rewrite(xml,[robot],id,names,options);
  const yaw=layout.yaw??0,q=[Math.cos(yaw/2),0,0,Math.sin(yaw/2)];
  let mountJoints='',mobileActuators='';
  if(id==='r1'){
    // This is an explicitly ideal planar, motor-driven carriage. The source
    // steer/wheel mechanism remains intact, but wheel traction is NOT modeled.
    // A source-envelope chassis proxy adds external collision coverage without
    // replacing any original R1 fingertip geom or changing its contact bits.
    body=body.replace(/^<body\b[^>]*>/,match=>`${match}<geom name="r1__chassis_proxy" type="box" pos="-.0325 0 .13175" size=".3137 .3375 .17325" contype="64" conaffinity="${options.interRobotContacts===false?1:385}" rgba=".1 .13 .16 0"/>`);
    const maxX=number(options.mobileXMax??.65),minY=number(options.mobileYMin??-.06),maxY=number(options.mobileYMax??.06);
    mountJoints=`<joint name="r1__base_slide_x" type="slide" axis="0 1 0" range="${number(options.mobileXMin)} ${maxX}" damping="200"/><joint name="r1__base_slide_y" type="slide" axis="-1 0 0" range="${minY} ${maxY}" damping="200"/><joint name="r1__base_yaw" type="hinge" axis="0 0 1" range="-.10 .10" damping="200"/><inertial pos="0 0 0" mass=".01" diaginertia=".0001 .0001 .0001"/>`;
    // Joint axes are mount-local: at the initial -PI/2 yaw, local +Y is
    // world +X, local -X is world +Y. All route commands are relative metres.
    mobileActuators=`<position name="r1__base_slide_x_motor" joint="r1__base_slide_x" kp="4000" kv="1000" ctrlrange="${number(options.mobileXMin)} ${maxX}" forcelimited="true" forcerange="-600 600"/><position name="r1__base_slide_y_motor" joint="r1__base_slide_y" kp="4000" kv="1000" ctrlrange="${minY} ${maxY}" forcelimited="true" forcerange="-600 600"/><position name="r1__base_yaw_motor" joint="r1__base_yaw" kp="6000" kv="1200" ctrlrange="-.10 .10" forcelimited="true" forcerange="-1000 1000"/>`;
  }
  return {
    body:`<body name="${id}__mount" pos="${vector(layout.origin)}" quat="${vector(q)}">${mountJoints}${body}</body>`,
    asset:asset?rewrite(xml,asset.children,id,names,options):'',
    contact:contact?rewrite(xml,contact.children,id,names,options):'',
    actuator:(actuator?rewrite(xml,actuator.children,id,names,options):'')+mobileActuators,
    defaultXml:inner(xml,defaults).trim(),option:option?.attrs,compiler:compiler?.attrs,
    metadata:{id,prefix:`${id}__`,origin:[...layout.origin],yaw,rootName:names.body[layout.root],mountName:`${id}__mount`,names,count,
      collisionScope:id==='r1'?'Original fingertip boxes only; no R1 shell/body geometry is present in the source model':'Original convex-hull collision meshes, original mechanical-neighbour exclusions',
      crossRobotCollisionBit:DEFAULT_LAYOUT.robots[id].collisionBit,mobile:id==='r1',
      addedCollisionGeoms:id==='r1'?['r1__chassis_proxy']:[]},
  };
}

export function boxGeom(name,pos,size,rgba,{contype=1,conaffinity=511}={}){
  return `<geom name="${name}" type="box" pos="${vector(pos)}" size="${vector(size)}" rgba="${rgba}" contype="${contype}" conaffinity="${conaffinity}"/>`;
}
export function fixedTable(name,center,half,height){
  const top=.0325,parts=[boxGeom(`${name}_top`,[center[0],center[1],height-top],[half[0],half[1],top],'.24 .29 .33 1')];
  for(const x of[-1,1])for(const y of[-1,1])parts.push(boxGeom(`${name}_leg_${x<0?'l':'r'}${y<0?'f':'b'}`,[center[0]+x*(half[0]-.045),center[1]+y*(half[1]-.045),(height-top*2)/2],[.025,.025,(height-top*2)/2],'.11 .14 .17 1'));
  return parts.join('\n');
}

function createParts(orderCount,height,robotLayouts){
  const parts=[];
  for(let order=0;order<orderCount;order++){
    const bank=Math.floor(order/3),column=order%3,feedOrigin=[-.32*order,robotLayouts.r1.origin[1],robotLayouts.r1.origin[2]],forward=.73;
    const cylinderStart=lateral=>localToShared([forward,lateral,height-robotLayouts.r1.origin[2]+.026],{origin:feedOrigin,yaw:robotLayouts.r1.yaw});
    // Leave real jaw-sweep clearance between source cells, not merely a
    // positive body/body gap. The original .065 m X pitch allowed a wide-open
    // Dex1 jaw to touch the next order's valve during the closing transition.
    const valveStart=localToShared([.35+bank*.15,[-.16,-.30,-.02][column],height+.019],robotLayouts.g1);
    const specs=[
      ['valve',0,'box',[.026,.016,.019],.050,valveStart,'.21 .45 .68 1'],
      ['pin',0,'cylinder',[.020,.020,.026],.042,cylinderStart(-.09),'.74 .18 .17 1'],
      ['pin',1,'cylinder',[.020,.020,.026],.042,cylinderStart(-.03),'.74 .18 .17 1'],
      ['sleeve',0,'cylinder',[.020,.020,.026],.042,cylinderStart(.03),'.85 .66 .16 1'],
      ['sleeve',1,'cylinder',[.020,.020,.026],.042,cylinderStart(.09),'.85 .66 .16 1'],
    ];
    for(const[kind,index,shape,halfExtents,mass,start,rgba]of specs){
      const id=`order_${order+1}_${kind}_${index+1}`;
      parts.push({id,orderId:`order_${order+1}`,kind,index,shape,body:`part__${id}`,joint:`part__${id}_free`,geom:`part__${id}_geom`,start,halfExtents,
        halfX:halfExtents[0],halfZ:halfExtents[1],h:halfExtents[2],mass,rgba,
        sourceSupport:kind==='valve'?'g1_workbench':'feed_workbench',feedBank:kind==='valve'?null:order,feedDock:kind==='valve'?null:feedOrigin});
    }
  }
  return parts;
}

export function partXml(part){
  const [a,b,h]=part.halfExtents,m=part.mass;
  const inertia=part.shape==='box'?[m*(h*h+b*b)/3,m*(a*a+h*h)/3,m*(a*a+b*b)/3]:[m*(3*a*a+4*h*h)/12,m*(3*a*a+4*h*h)/12,m*a*a/2];
  const size=part.shape==='box'?[a,b,h]:[a,h];
  return `<body name="${part.body}" pos="${vector(part.start)}"><freejoint name="${part.joint}"/><inertial pos="0 0 0" mass="${m}" diaginertia="${vector(inertia)}"/><geom name="${part.geom}" type="${part.shape}" size="${size.map(number).join(' ')}" rgba="${part.rgba}" contype="1" conaffinity="511"/></body>`;
}

/**
 * sources must contain the unmodified dynamic-scene XML for r1, g1 and c5.
 * Returned metadata contains only structured-clone-safe values for Workers.
 * Shared parts are single free bodies: no mocap, weld, duplication or capture
 * constraints are introduced. A grasp must be produced by actual contacts.
 */
export function buildCollaborationModel(sources,{orderCount=3,layout=DEFAULT_LAYOUT,interRobotContacts=true}={}){
  if(!Number.isInteger(orderCount)||orderCount<1||orderCount>6)throw new Error('orderCount must be between 1 and 6');
  const height=layout.tableHeight??DEFAULT_LAYOUT.tableHeight;
  if(!Number.isFinite(height)||height<.75||height>1.1)throw new Error('Unsupported common workbench height');
  const robotLayouts=Object.fromEntries(ROBOT_IDS.map(id=>{
    const frame={...DEFAULT_LAYOUT.robots[id],...layout.robots?.[id]};frame.origin=finiteVector(frame.origin,`${id} origin`);
    if(!Number.isFinite(frame.yaw))throw new Error(`${id} yaw must be finite`);return[id,frame];
  }));
  // Separate order bays leave real open-jaw sweep clearance. Each bay uses the
  // already validated canonical four-piece layout; R1 must physically dock at
  // the order's bay instead of reaching through the next order's inventory.
  const feedSpan=.32*(orderCount-1),mobileXMin=Math.min(-.36,-feedSpan-.04);
  const robots=ROBOT_IDS.map(id=>sourceRobot(sources?.[id],id,robotLayouts[id],{interRobotContacts,mobileXMin}));
  for(const robot of robots){
    if(JSON.stringify(robot.option)!==JSON.stringify(robots[0].option))throw new Error('Source robot physics options must agree exactly');
    if(robot.defaultXml!==robots[0].defaultXml)throw new Error('Source robot default contact settings must agree exactly');
  }
  const handoff=finiteVector(layout.handoff??DEFAULT_LAYOUT.handoff,'handoff'),trayCenter=finiteVector(layout.trayCenter??DEFAULT_LAYOUT.trayCenter,'tray center');
  handoff[2]=height;trayCenter[2]=height;
  const c5TrayCenter=finiteVector(layout.c5TrayCenter??DEFAULT_LAYOUT.c5TrayCenter,'C5 tray center');c5TrayCenter[2]=height;
  const parts=createParts(orderCount,height,robotLayouts),trays=[];
  // One active common tray; additional physical empty trays wait downstream.
  // The low rim bounds loose parts. Metadata slots are marked landing areas,
  // not invented insertion constraints or attachments.
  let trayBodies='',trayActuators='';
  for(let order=0;order<orderCount;order++){
    const body=`tray__order_${order+1}`,queue=order===0?trayCenter:[1.08+(order-1)%3*.28,-1.07+Math.floor((order-1)/3)*.28,height];
    const center=[queue[0],queue[1],height+.004],jointX=`${body}_slide_x`,jointY=`${body}_slide_y`,actuatorX=`${body}_motor_x`,actuatorY=`${body}_motor_y`;
    const slots=[['valve',0,0,0],['pin',0,-.07,.065],['pin',1,.07,.065],['sleeve',0,-.07,-.065],['sleeve',1,.07,-.065]].map(([kind,index,x,y])=>({kind,index,offset:[x,y,.004+(kind==='valve'?.019:.026)],position:[center[0]+x,center[1]+y,height+.008+(kind==='valve'?.019:.026)]}));
    const half=[.12,.12,.004];
    trayBodies+=`<body name="${body}" pos="${vector(center)}"><joint name="${jointX}" type="slide" axis="1 0 0" range="-1.6 1.6" damping="8"/><joint name="${jointY}" type="slide" axis="0 1 0" range="-2 2" damping="8"/><inertial pos="0 0 0" mass=".6" diaginertia=".0028832 .0028832 .00576"/>${boxGeom(`${body}_floor`,[0,0,0],half,'.22 .26 .29 1')}`;
    for(const[x,y,sx,sy]of[[0,-.12,.12,.004],[0,.12,.12,.004],[-.12,0,.004,.12],[.12,0,.004,.12]])trayBodies+=boxGeom(`${body}_rim_${x}_${y}`,[x,y,.006],[sx,sy,.006],'.51 .57 .61 1');
    trayBodies+='</body>';
    trayActuators+=`<position name="${actuatorX}" joint="${jointX}" kp="10000" kv="150" ctrlrange="-1.6 1.6" forcelimited="true" forcerange="-60 60"/><position name="${actuatorY}" joint="${jointY}" kp="10000" kv="150" ctrlrange="-2 2" forcelimited="true" forcerange="-60 60"/>`;
    trays.push({orderId:`order_${order+1}`,body,center,initialPosition:center,baseHeight:height+.004,surfaceHeight:height+.008,
      jointX,jointY,actuatorX,actuatorY,halfExtents:half,slots,transport:'actuated-planar-slide',
      destination:[trayCenter[0],trayCenter[1],height+.004],exit:[1.65+order*.01,c5TrayCenter[1],height+.004],
      staging:{offset:[0,.095,.030],kind:'small-part-staging'},
      docks:{g1:[trayCenter[0],trayCenter[1],height+.004],c5:[c5TrayCenter[0],c5TrayCenter[1],height+.004],r1:[.53,.725,height+.004],queue:center,exit:[1.65+order*.01,c5TrayCenter[1],height+.004]}});
  }
  const stations={
    g1:{body:'g1_workbench',geom:'g1_workbench_top',center:[.545,-.85,height-.0325],halfExtents:[.275,.35,.0325],tableHeight:height},
    c5:{body:'shared_workbench',geom:'shared_workbench_top',center:[.60,.55,height-.0325],halfExtents:[.35,.45,.0325],tableHeight:height},
    feed:{body:'feed_workbench',geom:'feed_workbench_top',center:[-feedSpan/2,.84,height-.0325],halfExtents:[feedSpan/2+.25,.19,.0325],tableHeight:height},
  };
  const conveyor={body:'transfer_roller_bed',geom:'transfer_roller_bed_top',center:[.54,-.20,height-.0325],halfExtents:[.17,.35,.0325],tableHeight:height};
  const queueBed={body:'queue_roller_bed',geom:'queue_roller_bed_top',center:[1.30,-.95,height-.0325],halfExtents:[.54,.38,.0325],tableHeight:height};
  const outputBed={body:'output_roller_bed',geom:'output_roller_bed_top',center:[1.10,c5TrayCenter[1],height-.0325],halfExtents:[.70,.16,.0325],tableHeight:height};
  const platformMinX=mobileXMin-.40,platformMaxX=1.05;
  const platform={body:'r1_mobile_platform',geom:'r1_mobile_platform_top',center:[(platformMinX+platformMaxX)/2,1.55,.08325],halfExtents:[(platformMaxX-platformMinX)/2,.42,.08325],topHeight:.1665};
  const mobility={robotId:'r1',mountName:'r1__mount',model:'ideal-planar-actuated-base-not-wheel-traction',
    joints:{x:'r1__base_slide_x',y:'r1__base_slide_y',yaw:'r1__base_yaw'},actuators:{x:'r1__base_slide_x_motor',y:'r1__base_slide_y_motor',yaw:'r1__base_yaw_motor'},
    docks:{feed:{position:[0,1.55,.208],yaw:-Math.PI/2},handoff:{position:[.53,1.55,.208],yaw:-Math.PI/2}},
    feedByOrder:Array.from({length:orderCount},(_,i)=>({orderId:`order_${i+1}`,position:[-.32*i,1.55,.208],yaw:-Math.PI/2})),
    route:[...Array.from({length:orderCount},(_,i)=>({id:i===0?'feed':`feed-bank-${i+1}`,label:`第${i+1}单来料停靠`,mountPosition:[-.32*i,1.55,.208],yaw:-Math.PI/2})).reverse(),{id:'handoff',label:'交接停靠',mountPosition:[.53,1.55,.208],yaw:-Math.PI/2}],
    feedBayPitch:.32,baseXRange:[mobileXMin,.65],
    dockTolerance:{position:.004,yaw:.008,speed:.01},maxSpeed:.08,maxYawSpeed:.04,platform,chassisProxy:{body:'r1__base_link',geom:'r1__chassis_proxy',localCenter:[-.0325,0,.13175],halfExtents:[.3137,.3375,.17325]},
    axisConvention:{coordinates:'relative joint positions, not absolute world coordinates',worldAxes:{x:[1,0,0],y:[0,1,0],yaw:[0,0,1]},mountLocalAxes:{x:[0,1,0],y:[-1,0,0],yaw:[0,0,1]}},
    addedMass:.01,actuationAssumption:'Three ideal planar servo axes at fixed base height; original steer/wheel axes are preserved but do not generate wheel-ground traction.'};
  const cell=[
    '<geom name="cell_floor" type="plane" size="5 5 .1" rgba=".15 .18 .22 1" contype="1" conaffinity="511"/>',
    ...Object.entries(stations).map(([id,station])=>`<body name="${station.body}">${fixedTable(id==='c5'?'shared_workbench':`${id}_workbench`,station.center.slice(0,2),station.halfExtents.slice(0,2),height)}</body>`),
    ...[conveyor,queueBed,outputBed].map(support=>`<body name="${support.body}">${fixedTable(support.body,support.center.slice(0,2),support.halfExtents.slice(0,2),height)}</body>`),
    `<body name="${platform.body}">${boxGeom(platform.geom,platform.center,platform.halfExtents,'.19 .22 .26 1')}</body>`,
    `<body name="c5_pedestal_body"><geom name="c5_grounded_pedestal" type="cylinder" pos="${vector([robotLayouts.c5.origin[0],robotLayouts.c5.origin[1],(.72+robotLayouts.c5.origin[2])/2])}" size=".12 ${number((.72+robotLayouts.c5.origin[2])/2)}" rgba=".26 .29 .33 1" contype="1" conaffinity="511"/></body>`,
    `<site name="cell_handoff" pos="${vector([handoff[0],handoff[1],height+.001])}" type="box" size=".065 .06 .001" rgba=".11 .7 .63 .3"/>`,
    trayBodies,...parts.map(partXml),...robots.map(robot=>robot.body),
  ].join('\n');
  // The tray's fixed-height planar rail and bed normal contact must not also
  // impose a dry-friction tangent constraint. Otherwise the redundant Z
  // constraint can grow an arbitrary normal force to resist the drive motor.
  // These explicit pairs model an ideal roller bed (normal support only).
  // Cargo/tray friction, tray rims and every source robot contact stay intact;
  // the finite 8 N s/m planar rail damping supplies transport resistance.
  const rollerSupports=[...Object.values(stations),conveyor,queueBed,outputBed];
  const rollerPairs=trays.flatMap(tray=>rollerSupports.map(support=>`<pair name="${tray.body}_${support.body}_roller_contact" geom1="${tray.body}_floor" geom2="${support.geom}" condim="1" solref=".006 1" solimp=".95 .99 .001"/>`)).join('');
  const xml=`<mujoco model="shared_industrial_kitting"><compiler angle="radian" autolimits="true" fusestatic="false" inertiafromgeom="false"/><option timestep=".001" integrator="implicitfast" solver="Newton" iterations="60" cone="elliptic" gravity="0 0 -9.81"/><default>${robots[0].defaultXml}</default><asset>${robots.map(robot=>robot.asset).join('\n')}</asset><worldbody>${cell}</worldbody><contact>${robots.map(robot=>robot.contact).join('\n')}${rollerPairs}</contact><actuator>${robots.map(robot=>robot.actuator).join('\n')}${trayActuators}</actuator></mujoco>`;
  return {xml,metadata:{version:COLLAB_MODEL_VERSION,coordinateSystem:'MuJoCo Z-up metres',singleSharedWorld:true,tableHeight:height,
    robots:Object.fromEntries(robots.map(robot=>[robot.metadata.id,robot.metadata])),
    workbench:stations.c5,stations,conveyor,queueBed,outputBed,mobility,
    traySupport:{model:'ideal-roller-normal-support',normalContactDimension:1,railDamping:8,pairCount:trays.length*rollerSupports.length,
      cargoTrayFriction:[3.5,.08,.01],sourceRobotContactsUnchanged:true,notes:'Only tray floors against fixed support tops use normal-only ideal-roller contact; no cargo friction or rim collision is removed.'},
    handoff:{position:handoff,halfExtents:[.065,.06],site:'cell_handoff'},parts,trays,
    supports:{floor:{body:'world',geom:'cell_floor'},c5Pedestal:{body:'c5_pedestal_body',geom:'c5_grounded_pedestal'},r1Platform:{body:platform.body,geom:platform.geom}},
    safety:{interRobotContacts,originalContactParametersRetained:true,originalActuatorLimitsRetained:true,partAttachmentConstraints:false,
      r1FullBodyCollision:false,notes:['R1 source has fingertip collision boxes only. Planner must not claim full-body R1 collision validation.','MuJoCo source collision meshes are convex hulls, not exact non-convex surfaces.']}}};
}
