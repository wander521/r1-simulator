import test from 'node:test';
import assert from 'node:assert/strict';
import {remainingMotionSegments,requiresHeldContact} from '../dist/collab/resume-route.mjs';

const path=offset=>Array.from({length:5},(_,i)=>({joint:offset+i}));
const plan={descend:path(0),lift:path(10),transfer:path(20),place:path(30),retreat:path(40)};
const options={robotId:'r1',plan,pose:{joint:11},phaseTarget:()=>({joint:50})};
test('resume checks only the unexecuted path tail, not a rewind to the pick',()=>{
  const segments=remainingMotionSegments({...options,state:'lift',elapsed:.9,pickOnly:true});
  assert.deepEqual(segments.map(s=>s.pose.joint),[13,14]);
  assert.ok(segments.every(s=>s.objectPhase==='lift'));
});
test('released-object return checks keep object on its support',()=>{
  const segments=remainingMotionSegments({...options,state:'retreat',elapsed:.8});
  assert.deepEqual(segments.map(s=>s.pose.joint),[43,44,50]);
  assert.ok(segments.every(s=>s.objectPhase==='retreat'));
});
test('G1 resumes ready return without an intermediate home posture',()=>{
  const phases=[];
  remainingMotionSegments({...options,robotId:'g1',state:'settling',phaseTarget:phase=>{phases.push(phase);return{joint:1};}});
  assert.deepEqual(phases,['return_clear','return_align']);
});
test('dynamic placement lowering remains the current goal; unknown phases fail closed',()=>{
  const segments=remainingMotionSegments({...options,state:'place',elapsed:2,placePose:{joint:99}});
  assert.equal(segments[0].pose.joint,99);
  assert.throws(()=>remainingMotionSegments({...options,state:'unexpected'}),/未知阶段/);
  assert.throws(()=>remainingMotionSegments({...options,state:'lift',plan:{}}),/不完整/);
});
test('waiting and planning with a carrier still require dual contact, release does not',()=>{
  for(const phase of ['need-outbound','clear-source','await-destination','await-destination-zone','need-destination-dock','need-place'])assert.equal(requiresHeldContact({phase},'planning'),true);
  assert.equal(requiresHeldContact({phase:'placing'},'release'),false);
  assert.equal(requiresHeldContact({navigation:{carry:true}},'stopped'),true);
});
