import test from 'node:test';
import assert from 'node:assert/strict';
import {sameFreeBodyPose,guardedBodies} from '../dist/collab/planning-guard.mjs';

const pose=[.2,.3,.4,1,0,0,0];
test('in-place rotation invalidates a plan, equivalent quaternion signs do not',()=>{
  assert.ok(sameFreeBodyPose(pose,pose,0));
  assert.ok(sameFreeBodyPose([...pose.slice(0,3),-1,0,0,0],pose,0));
  assert.equal(sameFreeBodyPose([...pose.slice(0,3),Math.cos(.03),0,0,Math.sin(.03)],pose,0),false);
  assert.ok(sameFreeBodyPose([...pose.slice(0,3),Math.cos(.005),0,0,Math.sin(.005)],pose,0));
});
test('translation, invalid numeric data and missing bodies reject stale evidence',()=>{
  assert.equal(sameFreeBodyPose([.205,...pose.slice(1)],pose,0),false);
  assert.equal(sameFreeBodyPose([NaN,...pose.slice(1)],pose,0),false);
  assert.equal(sameFreeBodyPose([...pose.slice(0,3),0,0,0,0],pose,0),false);
  assert.equal(sameFreeBodyPose([],pose,0),false);
  assert.ok(sameFreeBodyPose([99,...pose],[98,...pose],1));
});
test('a loaded carrier guards each original loose part, not only its handle',()=>{
  assert.deepEqual(guardedBodies({body:'bin',payloadBodies:['part1','part2','part1']}),['bin','part1','part2']);
  assert.deepEqual(guardedBodies({body:'empty',payloadBodies:[]}),['empty']);
  assert.deepEqual(guardedBodies(null),[]);
});
test('cylindrical axial spin is equivalent, tilt and translated cylinders are not',()=>{
  const spinning=[...pose.slice(0,3),Math.cos(.7),0,0,Math.sin(.7)];
  assert.equal(sameFreeBodyPose(spinning,pose,0),false);
  assert.ok(sameFreeBodyPose(spinning,pose,0,{axialSymmetry:true}));
  assert.equal(sameFreeBodyPose([...pose.slice(0,3),Math.cos(.07),Math.sin(.07),0,0],pose,0,{axialSymmetry:true}),false);
  assert.equal(sameFreeBodyPose([.21,...spinning.slice(1)],pose,0,{axialSymmetry:true}),false);
});
