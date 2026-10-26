import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {buildCollaborationModel,DEFAULT_LAYOUT,COLLAB_MODEL_VERSION,ROBOT_IDS,localToShared,sharedToLocal} from '../dist/collab/model.mjs';
import {REST as R1_REST,REST_TORSO as R1_TORSO} from '../dist/r1/grasp-controller.mjs';
import {REST as G1_REST,REST_TORSO as G1_TORSO} from '../dist/g1/grasp-controller.mjs';
import {HOME_POSE as C5_HOME} from '../dist/c5/c5-profile.mjs';

const root=new URL('../',import.meta.url),checks=[],started=performance.now();
const paths=['work/test-collab-model.mjs','dist/collab/model.mjs','dist/r1/grasp-controller.mjs','dist/g1/grasp-controller.mjs','dist/c5/c5-profile.mjs',...ROBOT_IDS.map(id=>`dist/${id}/models/${id}-dynamic-scene.xml`)];
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
const sourceHashes=await hashes(),sources=Object.fromEntries(await Promise.all(ROBOT_IDS.map(async id=>[id,await readFile(new URL(`dist/${id}/models/${id}-dynamic-scene.xml`,root),'utf8')])));
async function check(name,run){try{const detail=await run();checks.push({name,pass:true,detail});console.log('PASS',name);}catch(error){checks.push({name,pass:false,error:error.message,stack:error.stack});console.error('FAIL',name,error.message);}}
const near=(a,b,tol=1e-10)=>assert(Math.abs(a-b)<=tol,`${a} != ${b}`),vectorNear=(a,b,tol=1e-10)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>near(v,b[i],tol));};
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const vfs=new api.MjVFS(),{xml,metadata}=buildCollaborationModel(sources),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model),original={};
const nameId=(model,type,name)=>api.mj_name2id(model,api.mjtObj[`mjOBJ_${type}`].value,name);
try{
  for(const id of ROBOT_IDS){const m=api.MjModel.from_xml_string(sources[id],vfs),d=new api.MjData(m);api.mj_forward(m,d);original[id]={model:m,data:d};}
  api.mj_forward(model,data);
  await check('One MjModel/MjData contains three articulated robots, fifteen unique free parts, three planar trays and no attachment equality',()=>{
    assert.equal(model.nq,181);assert.equal(model.nv,166);assert.equal(model.nu,76);assert.equal(model.nmesh,27);assert.equal(model.nexclude,34);assert.equal(model.neq,0);
    assert.equal(metadata.parts.length,15);assert.equal(metadata.trays.length,3);assert.equal(metadata.singleSharedWorld,true);
    assert(!/<mocap\b|mocap="true"|<weld\b|<equality\b/.test(xml));
    const bodies=metadata.parts.map(part=>nameId(model,'BODY',part.body));assert.equal(new Set(bodies).size,15);assert(bodies.every(id=>id>0));
    for(const part of metadata.parts){const joint=nameId(model,'JOINT',part.joint);assert.equal(model.jnt_type[joint],api.mjtJoint.mjJNT_FREE.value);assert.equal(model.body_jntadr[nameId(model,'BODY',part.body)],joint);}
    for(const legacy of ['red_cube','blue_block','yellow_cylinder','task_table'])assert.equal(nameId(model,'BODY',legacy),-1);
    return {nq:model.nq,nv:model.nv,nu:model.nu,nbody:model.nbody,ngeom:model.ngeom,nmesh:model.nmesh,nexclude:model.nexclude,neq:model.neq,parts:15,trays:3};
  });
  for(const id of ROBOT_IDS)await check(`${id}: exact source body inertias, joint axes/ranges and actuator gains/force limits survive namespace merge`,()=>{
    const source=original[id].model,names=metadata.robots[id].names;
    for(const [canonical,mapped]of Object.entries(names.body)){
      const a=nameId(source,'BODY',canonical),b=nameId(model,'BODY',mapped);assert(a>=0&&b>=0);near(source.body_mass[a],model.body_mass[b]);
      for(const[field,stride]of[['body_inertia',3],['body_ipos',3],['body_iquat',4]])vectorNear([...source[field].slice(a*stride,(a+1)*stride)],[...model[field].slice(b*stride,(b+1)*stride)]);
    }
    for(const[canonical,mapped]of Object.entries(names.joint)){
      const a=nameId(source,'JOINT',canonical),b=nameId(model,'JOINT',mapped);assert.equal(source.jnt_type[a],model.jnt_type[b]);
      for(const[field,stride]of[['jnt_pos',3],['jnt_axis',3],['jnt_range',2],['jnt_solref',2],['jnt_solimp',5]])vectorNear([...source[field].slice(a*stride,(a+1)*stride)],[...model[field].slice(b*stride,(b+1)*stride)]);
      const da=source.jnt_dofadr[a],db=model.jnt_dofadr[b];for(const field of['dof_damping','dof_armature'])near(source[field][da],model[field][db]);
    }
    for(const[canonical,mapped]of Object.entries(names.actuator)){
      const a=nameId(source,'ACTUATOR',canonical),b=nameId(model,'ACTUATOR',mapped);
      for(const[field,stride]of[['actuator_gainprm',10],['actuator_biasprm',10],['actuator_forcerange',2],['actuator_ctrlrange',2]])vectorNear([...source[field].slice(a*stride,(a+1)*stride)],[...model[field].slice(b*stride,(b+1)*stride)]);
      const originalJoint=source.actuator_trnid[a*2],mappedJoint=model.actuator_trnid[b*2];assert.equal(names.joint[api.mj_id2name(source,api.mjtObj.mjOBJ_JOINT.value,originalJoint)],api.mj_id2name(model,api.mjtObj.mjOBJ_JOINT.value,mappedJoint));
    }
    return metadata.robots[id].count;
  });
  for(const id of ROBOT_IDS)await check(`${id}: all original contact parameters and masks retained; cross-robot bits do not change self-pair filtering`,()=>{
    const source=original[id].model,names=metadata.robots[id].names,own=DEFAULT_LAYOUT.robots[id].collisionBit,other=ROBOT_IDS.filter(x=>x!==id).reduce((bits,x)=>bits|DEFAULT_LAYOUT.robots[x].collisionBit,0);
    for(const[canonical,mapped]of Object.entries(names.geom)){
      const a=nameId(source,'GEOM',canonical),b=nameId(model,'GEOM',mapped);
      const type=source.geom_contype[a],affinity=source.geom_conaffinity[a],collider=Boolean(type||affinity);
      assert.equal(model.geom_contype[b],collider?type|own:type);assert.equal(model.geom_conaffinity[b],collider?affinity|other:affinity);
      for(const[field,stride]of[['geom_friction',3],['geom_solref',2],['geom_solimp',5],['geom_size',3],['geom_pos',3],['geom_quat',4]])vectorNear([...source[field].slice(a*stride,(a+1)*stride)],[...model[field].slice(b*stride,(b+1)*stride)]);
    }
    const geoms=Object.keys(names.geom);for(const a of geoms)for(const b of geoms){
      const sa=nameId(source,'GEOM',a),sb=nameId(source,'GEOM',b),ma=nameId(model,'GEOM',names.geom[a]),mb=nameId(model,'GEOM',names.geom[b]);
      const before=Boolean(source.geom_contype[sa]&source.geom_conaffinity[sb]||source.geom_contype[sb]&source.geom_conaffinity[sa]);
      const after=Boolean(model.geom_contype[ma]&model.geom_conaffinity[mb]||model.geom_contype[mb]&model.geom_conaffinity[ma]);assert.equal(after,before);
    }
    return {geoms:geoms.length,pairFilterChecks:geoms.length**2,originalSourceScope:metadata.robots[id].collisionScope};
  });
  await check('Every robot body FK is the original canonical FK transformed once into the shared frame',()=>{
    for(const id of ROBOT_IDS){const {model:source,data:sd}=original[id],frame=metadata.robots[id];for(const[canonical,mapped]of Object.entries(frame.names.body)){
      const a=nameId(source,'BODY',canonical),b=nameId(model,'BODY',mapped);vectorNear([...data.xpos.slice(b*3,b*3+3)],localToShared([...sd.xpos.slice(a*3,a*3+3)],frame));
      const[w,x,y,z]=sd.xquat.slice(a*4,a*4+4),c=Math.cos(frame.yaw/2),s=Math.sin(frame.yaw/2);
      vectorNear([...data.xquat.slice(b*4,b*4+4)],[c*w-s*z,c*x-s*y,c*y+s*x,c*z+s*w]);
    }}
    const frame={origin:[.2,-.4,.3],yaw:.7},point=[.5,.1,.9];vectorNear(sharedToLocal(localToShared(point,frame),frame),point);
  });
  await check('Shared support, handoff, loose part BOM and tray slot metadata agree with physical geometry',()=>{
    assert(nameId(model,'BODY',metadata.workbench.body)>0);const support=nameId(model,'GEOM',metadata.workbench.geom);near(data.geom_xpos[support*3+2]+model.geom_size[support*3+2],metadata.tableHeight);
    for(const orderId of new Set(metadata.parts.map(part=>part.orderId))){const parts=metadata.parts.filter(p=>p.orderId===orderId);assert.equal(parts.filter(p=>p.kind==='valve').length,1);assert.equal(parts.filter(p=>p.kind==='pin').length,2);assert.equal(parts.filter(p=>p.kind==='sleeve').length,2);}
    for(const part of metadata.parts){const id=nameId(model,'BODY',part.body);vectorNear([...data.xpos.slice(id*3,id*3+3)],part.start);near(part.start[2]-part.h,metadata.tableHeight);near(model.body_mass[id],part.mass);}
    for(const tray of metadata.trays){assert.equal(tray.slots.length,5);for(const [joint,actuator,axis]of[[tray.jointX,tray.actuatorX,[1,0,0]],[tray.jointY,tray.actuatorY,[0,1,0]]]){
      const j=nameId(model,'JOINT',joint),a=nameId(model,'ACTUATOR',actuator);assert(j>=0&&a>=0);assert.equal(model.jnt_type[j],api.mjtJoint.mjJNT_SLIDE.value);vectorNear([...model.jnt_axis.slice(j*3,j*3+3)],axis);assert.equal(model.actuator_trnid[a*2],j);}
      near(tray.surfaceHeight,tray.baseHeight+.004);assert(tray.exit[0]>=1.65);for(const slot of tray.slots)vectorNear(slot.position,tray.center.map((x,i)=>x+slot.offset[i]));
      assert.equal(tray.staging.kind,'small-part-staging');vectorNear(tray.staging.offset,[0,.095,.030]);vectorNear(tray.docks.r1,[.53,.725,metadata.tableHeight+.004]);
      near(tray.docks.r1[0]+tray.staging.offset[0],metadata.handoff.position[0]);near(tray.docks.r1[1]+tray.staging.offset[1],metadata.handoff.position[1]);
      near(tray.docks.r1[2]+tray.staging.offset[2]-.026,tray.surfaceHeight);assert(.116-(Math.abs(tray.staging.offset[1])+.020)>=.001-1e-10);
    }
    assert.equal(metadata.traySupport.model,'ideal-roller-normal-support');assert.equal(model.npair,metadata.traySupport.pairCount);
    const floorIds=new Set(metadata.trays.map(tray=>nameId(model,'GEOM',`${tray.body}_floor`))),supportIds=new Set([...Object.values(metadata.stations),metadata.conveyor,metadata.queueBed,metadata.outputBed].map(support=>nameId(model,'GEOM',support.geom)));
    for(let i=0;i<model.npair;i++){
      assert.equal(model.pair_dim[i],1);const a=model.pair_geom1[i],b=model.pair_geom2[i];
      // MuJoCo canonicalizes pair geom ordering during compilation. The pair
      // is physically symmetric: require exactly one tray floor and one fixed
      // support top, irrespective of their compiled numeric index order.
      assert(floorIds.has(a)&&supportIds.has(b)||floorIds.has(b)&&supportIds.has(a),'Normal-only roller pair must not affect robot or cargo collision');
    }
    for(const part of metadata.parts){const g=nameId(model,'GEOM',part.geom);vectorNear([...model.geom_friction.slice(g*3,g*3+3)],[3.5,.08,.01]);}
    for(const tray of metadata.trays)for(const name of[tray.actuatorX,tray.actuatorY]){const a=nameId(model,'ACTUATOR',name);near(-model.actuator_biasprm[a*10+1],10000);near(-model.actuator_biasprm[a*10+2],150);vectorNear([...model.actuator_forcerange.slice(a*2,a*2+2)],[-60,60]);}
    assert.equal(metadata.safety.r1FullBodyCollision,false);assert.equal(metadata.safety.partAttachmentConstraints,false);
  });
  await check('R1 ideal planar carriage uses genuinely world-aligned slide axes and reports the actual moving mount frame',()=>{
    const scratch=new api.MjData(model),mobility=metadata.mobility,mount=nameId(model,'BODY',mobility.mountName);
    try{
      assert.equal(mobility.model,'ideal-planar-actuated-base-not-wheel-traction');assert.equal(metadata.robots.r1.mobile,true);
      for(const x of[0,.20,.53,-.32])for(const y of[-.02,0,.02])for(const yaw of[-.05,0,.05]){
        api.mj_resetData(model,scratch);
        for(const[axis,value]of Object.entries({x,y,yaw})){const j=nameId(model,'JOINT',mobility.joints[axis]);assert(j>=0);scratch.qpos[model.jnt_qposadr[j]]=value;}
        api.mj_forward(model,scratch);vectorNear([...scratch.xpos.slice(mount*3,mount*3+3)],[x,1.55+y,.208]);
        const angle=-Math.PI/2+yaw;vectorNear([...scratch.xquat.slice(mount*4,mount*4+4)],[Math.cos(angle/2),0,0,Math.sin(angle/2)]);
      }
      const platform=nameId(model,'GEOM',mobility.platform.geom),proxy=nameId(model,'GEOM',mobility.chassisProxy.geom);assert(platform>=0&&proxy>=0);assert.equal(model.geom_bodyid[proxy],nameId(model,'BODY','r1__base_link'));
      near(mobility.platform.center[2]+mobility.platform.halfExtents[2],.1665);near(metadata.robots.r1.origin[2]+mobility.chassisProxy.localCenter[2]-mobility.chassisProxy.halfExtents[2],.1665);
      assert(mobility.docks.feed.position[0]===0&&mobility.docks.handoff.position[0]===.53);assert.equal(mobility.feedByOrder.length,3);
      return{poseCombinations:108,originalWheelTraction:false,extraServoAxes:3,actualMount:mobility.mountName};
    }finally{scratch.delete();}
  });
  await check('Six orders use six separated feed bays, actual per-order docking and individually grounded station supports',()=>{
    const full=buildCollaborationModel(sources,{orderCount:6}).metadata;assert.equal(full.parts.length,30);
    for(const part of full.parts){const support=Object.values(full.stations).find(s=>s.body===part.sourceSupport);assert(support);for(const axis of[0,1])assert(Math.abs(part.start[axis]-support.center[axis])+part.halfExtents[axis]<=support.halfExtents[axis]+1e-10);
      if(part.kind!=='valve'){const position=sharedToLocal(part.start,{origin:part.feedDock,yaw:full.robots.r1.yaw});near(position[0],.73);assert(Math.abs(position[1])<=.09+1e-10);}}
    for(let i=0;i<full.parts.length;i++)for(let j=i+1;j<full.parts.length;j++){
      const a=full.parts[i],b=full.parts[j],separated=[0,1].some(axis=>Math.abs(a.start[axis]-b.start[axis])>=a.halfExtents[axis]+b.halfExtents[axis]+.005);assert(separated,`Overlapping/too-close source bodies ${a.id} and ${b.id}`);
    }
    near(full.mobility.feedBayPitch,.32);full.mobility.feedByOrder.forEach((dock,i)=>{
      near(dock.position[0],-.32*i);assert(dock.position[0]>=full.mobility.baseXRange[0]&&dock.position[0]<=full.mobility.baseXRange[1]);
      for(const part of full.parts.filter(part=>part.orderId===dock.orderId&&part.kind!=='valve'))vectorNear(part.feedDock,dock.position);
      const proxyHalfX=full.mobility.chassisProxy.halfExtents[1],platform=full.mobility.platform;assert(Math.abs(dock.position[0]-platform.center[0])+proxyHalfX<=platform.halfExtents[0]);
      if(i)near(full.mobility.feedByOrder[i-1].position[0]-dock.position[0],.32);
    });
    const valves=full.parts.filter(part=>part.kind==='valve');valves.forEach((part,i)=>vectorNear(sharedToLocal(part.start,full.robots.g1),[.35+Math.floor(i/3)*.15,[-.16,-.30,-.02][i%3],full.tableHeight+.019]));
    for(let i=0;i<valves.length;i++)for(let j=i+1;j<valves.length;j++)assert(Math.hypot(valves[i].start[0]-valves[j].start[0],valves[i].start[1]-valves[j].start[1])>=.14-1e-10,'Valve source cells need explicit gripper-sweep clearance');
    assert(full.stations.g1.center[1]+full.stations.g1.halfExtents[1]<full.stations.c5.center[1]-full.stations.c5.halfExtents[1]);
    near(full.stations.g1.center[0]-full.stations.g1.halfExtents[0],.27);
    for(const tray of full.trays)for(const slot of tray.slots){const extent=slot.kind==='valve'?.026:.020;assert(tray.docks.g1[0]+slot.offset[0]-extent>=.29-1e-10,'Part landing slot overhangs G1 table');}
    return{parts:30,feedBanks:6,feedBayPitch:.32,lastFeedDock:full.mobility.feedByOrder.at(-1).position,scope:'Geometry, FK support and axis-range contracts only; not six-order physical task execution',sourcePairsChecked:435,robotLateralSeparation:Math.abs(full.robots.g1.origin[1]-full.robots.c5.origin[1])};
  });
  await check('R1 reaches both docks through genuine rate-limited servo forces without writing navigation or part qpos during stepping',()=>{
    const moving=new api.MjData(model),mobility=metadata.mobility,mount=nameId(model,'BODY',mobility.mountName),baseActuator=nameId(model,'ACTUATOR',mobility.actuators.x),baseJoint=nameId(model,'JOINT',mobility.joints.x),baseDof=model.jnt_dofadr[baseJoint];
    try{
      const poses={r1:{...R1_TORSO,...R1_REST.left,...R1_REST.right},g1:{...G1_TORSO,...G1_REST.left,...G1_REST.right},c5:C5_HOME};
      for(const id of ROBOT_IDS)for(const[name,value]of Object.entries(poses[id])){const joint=nameId(model,'JOINT',metadata.robots[id].names.joint[name]);if(joint>=0)moving.qpos[model.jnt_qposadr[joint]]=value;}
      api.mj_forward(model,moving);
      const controls=Array.from({length:model.nu},(_,a)=>{const joint=model.actuator_trnid[a*2];return{a,q:model.jnt_qposadr[joint],d:model.jnt_dofadr[joint],target:moving.qpos[model.jnt_qposadr[joint]],kp:-model.actuator_biasprm[a*10+1]};});
      const partInitial=metadata.parts.map(part=>{const b=nameId(model,'BODY',part.body);return[...moving.xpos.slice(b*3,b*3+3)];});
      let command=0,maxSpeed=0,maxForce=0,steps=0;const arrivals=[];
      for(const destination of[.53,0]){
        const rampSteps=Math.ceil(Math.abs(destination-command)/(.001*mobility.maxSpeed));
        for(let tick=0;tick<rampSteps+2500;tick++){
          command+=Math.max(-mobility.maxSpeed*.001,Math.min(mobility.maxSpeed*.001,destination-command));
          for(const motor of controls)moving.ctrl[motor.a]=model.actuator_gainprm[motor.a*10]===1?motor.kp*motor.target+moving.qfrc_bias[motor.d]:motor.target;
          moving.ctrl[baseActuator]=command;api.mj_step(model,moving);steps++;
          assert([...moving.qpos,...moving.qvel].every(Number.isFinite));maxSpeed=Math.max(maxSpeed,Math.abs(moving.qvel[baseDof]));maxForce=Math.max(maxForce,Math.abs(moving.qfrc_actuator[baseDof]));
          assert(Math.abs(moving.qfrc_actuator[baseDof])<=600+1e-6);
        }
        api.mj_forward(model,moving);const position=[...moving.xpos.slice(mount*3,mount*3+3)];vectorNear(position,[destination,1.55,.208],.001);assert(Math.abs(moving.qvel[baseDof])<mobility.dockTolerance.speed);arrivals.push({command:destination,position,speed:Math.abs(moving.qvel[baseDof])});
      }
      assert(maxSpeed<=mobility.maxSpeed+.003);assert(maxForce>.05);
      for(let i=0;i<metadata.parts.length;i++){const b=nameId(model,'BODY',metadata.parts[i].body),p=[...moving.xpos.slice(b*3,b*3+3)];assert(Math.hypot(p[0]-partInitial[i][0],p[1]-partInitial[i][1])<.001);}
      return{steps,simulationSeconds:moving.time,worldTravelMetres:1.06,arrivals,maxSpeed,maxForce,qposWritesDuringStepping:0,scope:'Unloaded ideal-planar servo translation only; not wheel traction or loaded navigation validation.'};
    }finally{moving.delete();}
  });
  await check('Unresolved references, conflicting source options/defaults, missing robot roots and external includes fail closed',()=>{
    assert.throws(()=>buildCollaborationModel({...sources,c5:sources.c5.replace('joint="shoulder_joint"','joint="missing_joint"')}),/unresolved joint/);
    assert.throws(()=>buildCollaborationModel({...sources,g1:sources.g1.replace('iterations="60"','iterations="59"')}),/options must agree/);
    assert.throws(()=>buildCollaborationModel({...sources,r1:sources.r1.replace('friction="3.5 .08 .01"','friction="3.4 .08 .01"')}),/contact settings must agree/);
    assert.throws(()=>buildCollaborationModel({...sources,c5:sources.c5.replace('<body name="base_link"','<body name="wrong_root"')}),/root base_link is missing/);
    assert.throws(()=>buildCollaborationModel({...sources,r1:sources.r1.replace('<worldbody>','<include file="other.xml"/><worldbody>')}),/External MJCF includes/);
    assert.throws(()=>buildCollaborationModel(sources,{orderCount:0}),/orderCount/);
  });
  await check('Original actuator control and gravity stepping leave shared loose parts supported without qpos rewriting',()=>{
    const poses={r1:{...R1_TORSO,...R1_REST.left,...R1_REST.right},g1:{...G1_TORSO,...G1_REST.left,...G1_REST.right},c5:C5_HOME};
    for(const id of ROBOT_IDS)for(const[name,value]of Object.entries(poses[id])){const joint=nameId(model,'JOINT',metadata.robots[id].names.joint[name]);if(joint>=0)data.qpos[model.jnt_qposadr[joint]]=value;}
    api.mj_forward(model,data);const hold=Array.from({length:model.nu},(_,a)=>{const joint=model.actuator_trnid[a*2];return{a,q:model.jnt_qposadr[joint],d:model.jnt_dofadr[joint],target:data.qpos[model.jnt_qposadr[joint]],kp:-model.actuator_biasprm[a*10+1]};});
    const partStart=metadata.parts.map(part=>{const b=nameId(model,'BODY',part.body);return[...data.xpos.slice(b*3,b*3+3)];});
    for(let tick=0;tick<1000;tick++){
      // Position actuators have gainprm=kp and bias=-kp*q, general motors have
      // gain 1. This is control input only; no part or robot state is rewritten.
      for(const motor of hold)data.ctrl[motor.a]=model.actuator_gainprm[motor.a*10]===1?motor.kp*motor.target+data.qfrc_bias[motor.d]:motor.target;
      api.mj_step(model,data);
    }
    assert([...data.qpos,...data.qvel].every(Number.isFinite));near(data.time,1,1e-10);
    const result=metadata.parts.map((part,i)=>{const b=nameId(model,'BODY',part.body),p=[...data.xpos.slice(b*3,b*3+3)],drift=Math.hypot(p[0]-partStart[i][0],p[1]-partStart[i][1]);assert(drift<.001,`${part.id} planar drift ${drift}`);assert(Math.abs(p[2]-part.h-metadata.tableHeight)<.001);return{id:part.id,planarDrift:drift,surfaceError:p[2]-part.h-metadata.tableHeight};});
    return {seconds:data.time,steps:1000,partStateWritesDuringStepping:0,maxPlanarDrift:Math.max(...result.map(r=>r.planarDrift)),maxSurfaceError:Math.max(...result.map(r=>Math.abs(r.surfaceError))),finalContacts:data.ncon};
  });
}finally{
  for(const item of Object.values(original)){item.data.delete();item.model.delete();}
  data.delete();model.delete();vfs.delete();
}
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes),pass=sourceFrozen&&checks.every(c=>c.pass);
const report={pass,version:COLLAB_MODEL_VERSION,total:checks.length,passed:checks.filter(c=>c.pass).length,wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,endSourceHashes,checks};
const destination=new URL(process.argv.find(x=>x.startsWith('--report='))?.slice(9)??'work/validation/collab/model-contracts.json',root);
await mkdir(new URL('./',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass,total:report.total,passed:report.passed,wallSeconds:report.wallSeconds,report:destination.pathname}));if(!pass)process.exitCode=1;
