import test from 'node:test';
import assert from 'node:assert/strict';
import {advancePlanarCommand} from '../dist/collab/mobile-profile.mjs';
test('loaded base accelerates and brakes without target jumps, including diagonal routes',()=>{
  for(const goal of [[2,0],[0,-.4],[1,1],[.00001,0]]){
    let position=[0,0],velocity=[0,0],maxSpeed=0,maxAcceleration=0;
    for(let i=0;i<30000;i++){
      const next=advancePlanarCommand(position,velocity,goal,.001);
      maxSpeed=Math.max(maxSpeed,Math.hypot(...next.velocity));
      maxAcceleration=Math.max(maxAcceleration,Math.hypot(...next.velocity.map((v,k)=>(v-velocity[k])/.001)));
      assert.ok(Math.hypot(...next.position.map((v,k)=>v-position[k]))<=.000120001);
      position=next.position;velocity=next.velocity;
      if(Math.hypot(...position.map((v,k)=>v-goal[k]))<1e-9&&Math.hypot(...velocity)<1e-9)break;
    }
    assert.ok(maxSpeed<=.12000001);assert.ok(maxAcceleration<=.18001,`${goal}: acceleration ${maxAcceleration}`);
    assert.ok(Math.hypot(...position.map((v,k)=>v-goal[k]))<1e-8);
  }
});
test('invalid navigation inputs fail without mutation',()=>{
  const position=[0,0],velocity=[0,0];assert.throws(()=>advancePlanarCommand(position,velocity,[NaN,1],.001),/Invalid/);
  assert.deepEqual(position,[0,0]);assert.deepEqual(velocity,[0,0]);
});
