import * as THREE from '../vendor/three.module.min.js';

const tag='__robotPlanningType';
/** Structured clone does not retain Three prototypes. These explicit tags do,
 * without turning arbitrary payload properties into executable constructors. */
export function encodePlanningValue(value){
  if(value===null||typeof value!=='object')return value;
  if(value.isVector3)return{[tag]:'Vector3',value:value.toArray()};
  if(value.isQuaternion)return{[tag]:'Quaternion',value:value.toArray()};
  if(ArrayBuffer.isView(value))return Array.from(value);
  if(Array.isArray(value))return value.map(encodePlanningValue);
  const result={};for(const [key,item]of Object.entries(value)){
    if(['__proto__','constructor','prototype'].includes(key))throw new Error('路径数据包含无效字段');
    if(typeof item==='function')throw new Error(`路径数据不能包含函数：${key}`);
    result[key]=encodePlanningValue(item);
  }return result;
}
export function decodePlanningValue(value){
  if(value===null||typeof value!=='object')return value;
  if(value[tag]){
    const count=value[tag]==='Vector3'?3:value[tag]==='Quaternion'?4:0;
    if(!count||!Array.isArray(value.value)||value.value.length!==count||!value.value.every(Number.isFinite))throw new Error('路径向量或姿态数据无效');
    return count===3?new THREE.Vector3().fromArray(value.value):new THREE.Quaternion().fromArray(value.value);
  }
  if(Array.isArray(value))return value.map(decodePlanningValue);
  const result={};for(const [key,item]of Object.entries(value)){
    if(['__proto__','constructor','prototype'].includes(key))throw new Error('路径数据包含无效字段');
    result[key]=decodePlanningValue(item);
  }return result;
}

const controllerFields=['state','running','time','elapsed','tick','arm','selected','openings','pose','evidence','history','task','itemIndex',
  'stackEvidence','stackStableSeconds','stackResultVerified','reaction','reactionFinished','observerYaw','reducedMotion','crouch','from',
  'phaseGoal','continuing','manualMotion','placeLowering','placePose','planningBudgetMs'];
const dynamicsFields=['targets','previous','targetVelocity','fingerDesiredTargets','fingerRampedTargets','pipeFingerDesired','pipeFingerRamped'];
const stateArrays=['qpos','qvel','ctrl','act','qacc_warmstart','mocap_pos','mocap_quat','qfrc_applied','xfrc_applied','userdata'];
const derivedArrays=['xpos','xquat'];
const targetFields=['id','body','label','shape','colorKey','color','zoneColor','h','halfX','halfZ','start','drop','rotation','slotIndex','binId'];
const fields=(object,names)=>Object.fromEntries(names.filter(name=>object[name]!==undefined).map(name=>[name,object[name]]));

export function capturePlanningJob(runtime,{arm,id,continuing}={}){
  if(typeof runtime.sourceXml!=='string'||!runtime.sourceXml)throw new Error('缺少原始物理模型，不能开始后台路径检查');
  if(typeof runtime.modelXml!=='string'||!runtime.modelXml)throw new Error('缺少当前编译模型，不能开始后台路径检查');
  const controller=runtime.controller,physics={time:runtime.data.time};
  for(const name of stateArrays){const array=runtime.data[name];if(array&&typeof array.length==='number')physics[name]=Array.from(array);}
  const context=fields(controller,controllerFields);context.opening=controller.opening;
  const derived=Object.fromEntries(derivedArrays.map(name=>[name,Array.from(runtime.data[name])]));
  return encodePlanningValue({version:2,robotId:runtime.id,sourceXml:runtime.sourceXml,modelXml:runtime.modelXml,modelPipeSeed:runtime.modelPipeSeed,taskMode:runtime.taskMode,
    pipeSeed:controller.pipeLayout?.seed??null,targets:runtime.targets.map(target=>fields(target,targetFields)),
    request:{arm,id,continuing:Boolean(continuing)},physics,derived,controller:context,dynamics:fields(runtime.dynamics,dynamicsFields)});
}

/** Main-thread acceptance guard. The shared physical clock normally remains
 * paused at the barrier, but never install a plan if a caller changed that
 * snapshot through another API while the worker was busy. */
export function planningJobStillCurrent(runtime,job){
  if(runtime.id!==job.robotId||runtime.sourceXml!==job.sourceXml||runtime.modelXml!==job.modelXml||runtime.modelPipeSeed!==job.modelPipeSeed||runtime.taskMode!==job.taskMode||runtime.data.time!==job.physics.time)return false;
  if(runtime.controller.arm!==job.request.arm||runtime.controller.selected!==job.request.id)return false;
  if((runtime.controller.pipeLayout?.seed??null)!==job.pipeSeed)return false;
  const same=(actual,expected)=>JSON.stringify(encodePlanningValue(actual))===JSON.stringify(expected);
  if(!same(runtime.targets.map(target=>fields(target,targetFields)),job.targets))return false;
  if(!same(fields(runtime.dynamics,dynamicsFields),job.dynamics))return false;
  const plannedTask=job.controller.task,currentTask=runtime.controller.task;
  if(currentTask!==plannedTask&&!(job.taskMode==='pipes'&&plannedTask==='putaway'&&currentTask==='pipes'))return false;
  // startPipeItem updates only classification history/message after prepareItem
  // returns. All numerical inputs to the pending route must stay unchanged.
  for(const name of ['opening','openings','pose','crouch','itemIndex','observerYaw','reducedMotion','from','phaseGoal','continuing','manualMotion','placeLowering','placePose','planningBudgetMs'])
    if(!same(runtime.controller[name],job.controller[name]))return false;
  for(const name of stateArrays)if(job.physics[name]){
    const actual=runtime.data[name],expected=job.physics[name];
    if(!actual||actual.length!==expected.length)return false;
    for(let i=0;i<actual.length;i++)if(actual[i]!==expected[i])return false;
  }
  for(const name of derivedArrays){
    const actual=runtime.data[name],expected=job.derived[name];
    if(!actual||actual.length!==expected.length)return false;
    for(let i=0;i<actual.length;i++)if(actual[i]!==expected[i])return false;
  }
  return true;
}

/** Restore only an isolated worker world. Never use this on a running scene. */
export function restorePlanningJob(runtime,encoded){
  const job=decodePlanningValue(encoded);
  if(job.version!==2||runtime.id!==job.robotId||runtime.modelXml!==job.modelXml||runtime.modelPipeSeed!==job.modelPipeSeed)throw new Error('后台路径检查编译模型不匹配');
  for(const value of job.targets){const target=runtime.targets.find(item=>item.id===value.id);if(!target||target.body!==value.body)throw new Error('后台路径检查物体不匹配');Object.assign(target,value);}
  Object.assign(runtime.controller,job.controller);runtime.controller.opening=job.controller.opening;
  for(const name of dynamicsFields)if(job.dynamics[name]!==undefined)runtime.dynamics[name]=job.dynamics[name];
  for(const name of stateArrays)if(job.physics[name]!==undefined){
    const destination=runtime.data[name],source=job.physics[name];
    if(!destination||destination.length!==source.length||!source.every(Number.isFinite))throw new Error(`后台物理状态不匹配：${name}`);
    destination.set(source);
  }
  if(!Number.isFinite(job.physics.time))throw new Error('后台物理时钟无效');
  runtime.data.time=job.physics.time;runtime.api.mj_forward(runtime.model,runtime.data);
  // mj_step integrates qpos after computing xpos/xquat. Preserve the source
  // controller's exact sampled objectPose, rather than giving this mirror a
  // one-step-newer derived pose. These writes stay in the isolated worker;
  // collision scratch worlds still derive their checks from exact qpos.
  for(const name of derivedArrays){
    const destination=runtime.data[name],source=job.derived?.[name];
    if(!destination||!source||destination.length!==source.length||!source.every(Number.isFinite))throw new Error(`后台运动学快照不匹配：${name}`);
    destination.set(source);
  }
  runtime.dynamics.syncRobot();return job;
}

const resultFields=['planData','crouch','opening','openings','continuing','running','state','elapsed','from','phaseGoal','evidence','placeLowering','placePose','planError'];
export function capturePlanningResult(controller,initialHistoryLength=0){
  return encodePlanningValue({...fields(controller,resultFields),historyAppend:controller.history.slice(initialHistoryLength)});
}
export function applyPlanningResult(controller,encoded){
  const result=decodePlanningValue(encoded);
  if(!result.planData||!result.running||!Array.isArray(result.historyAppend)||!result.state||result.state==='planning')throw new Error('后台未返回经过验证的完整路径');
  // The worker has already run the original opening and entry-segment safety
  // gates. Install their resulting command state, not any worker physics data.
  for(const name of resultFields)if(result[name]!==undefined)controller[name]=result[name];
  controller.history.push(...result.historyAppend);return result;
}
