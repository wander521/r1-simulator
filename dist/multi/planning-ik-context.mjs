import * as THREE from '../vendor/three.module.min.js';

// Only the mesh-free, immutable Worker tree opts in. Original R1/G1/C5 solver
// source and standalone route graphs remain byte-for-byte unchanged.
const contextKey=Symbol.for('r1-simulator.worker-ik-context.v1');
const budgetKey=Symbol.for('r1-simulator.planning-budget.v1');
export function enablePlanningWorkerIK(robot){
  if(robot[contextKey])return robot;
  const find=robot.getObjectByName,set=robot.setJointValues,update=robot.updateMatrixWorld,names=new Map();
  const position=new THREE.Vector3(),scale=new THREE.Vector3();let fresh=false,assignments=0;
  const context={enabled:true};Object.defineProperty(robot,contextKey,{configurable:true,value:context});
  const override=(object,name,value)=>Object.defineProperty(object,name,{configurable:true,writable:true,enumerable:Object.getOwnPropertyDescriptor(object,name)?.enumerable??false,value});
  override(robot,'getObjectByName',function(name){if(!names.has(name))names.set(name,find.call(this,name));return names.get(name);});
  override(robot,'setJointValues',function(...args){
    // Every original solver iteration assigns its pose before FK. Check within
    // a candidate without editing solver arithmetic or importing multi code in
    // standalone routes. R1's perturbed FK calls make this even more frequent.
    if((assignments++&7)===0)robot[budgetKey]?.();fresh=false;return set.apply(this,args);
  });
  override(robot,'updateMatrixWorld',function(...args){fresh=false;const result=update.apply(this,args);fresh=true;return result;});
  for(const joint of Object.values(robot.joints)){
    const setValue=joint.setValue;
    override(joint,'setValue',function(...args){fresh=false;return setValue.apply(this,args);});
  }
  robot.traverse(node=>{
    const worldPosition=node.getWorldPosition,worldQuaternion=node.getWorldQuaternion,localToWorld=node.localToWorld;
    override(node,'getWorldPosition',function(target){return fresh?target.setFromMatrixPosition(this.matrixWorld):worldPosition.call(this,target);});
    override(node,'getWorldQuaternion',function(target){if(!fresh)return worldQuaternion.call(this,target);this.matrixWorld.decompose(position,target,scale);return target;});
    override(node,'localToWorld',function(vector){return fresh?vector.applyMatrix4(this.matrixWorld):localToWorld.call(this,vector);});
  });
  return robot;
}
