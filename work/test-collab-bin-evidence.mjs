import test from 'node:test';
import assert from 'node:assert/strict';
import {supportPairs,verifyBinInventory,verifyHandoffClearance} from '../dist/collab/bin-evidence.mjs';

function fixture(){
  let deleted=0;
  const contacts=[{geom1:0,geom2:1,efc_address:0},{geom1:2,geom2:3,efc_address:1},{geom1:4,geom2:5,efc_address:-1}];
  const data={ncon:3,efc_force:[.05,0],xpos:new Float64Array(18),get contact(){return {size:()=>contacts.length,get:i=>({...contacts[i],delete:()=>deleted++}),delete:()=>deleted++};}};
  const model={geom_bodyid:[0,1,2,3,4,5]};return {data,model,deleted:()=>deleted};
}
test('support is actual positive normal force, not proximity or inactive contact',()=>{
  const f=fixture(),pairs=supportPairs(f.model,f.data);
  assert.deepEqual([...pairs],['0:1','1:0']);assert.equal(f.deleted(),4);
});
test('inventory evidence rejects counting an unplaced part as packed and checks earlier output parts',()=>{
  const {data,model}=fixture(),metadata={parts:[{id:'p',body:'part',binId:'bin'}],bins:[{id:'bin',body:'carrier'}],slots:[{id:'out',partSlots:[[1,0,0]]}]};
  const pipeline={parts:new Map([['p',{packed:false}]]),bins:new Map([['bin',{parts:['p']}]])};
  const args={data,model,metadata,pipeline,bodyId:name=>({part:0,carrier:1,fixture_out:3}[name])};
  assert.equal(verifyBinInventory({...args,job:{type:'fetch',binId:'bin'}}).valid,true);
  assert.equal(verifyBinInventory({...args,job:{type:'pack',partId:'p',destination:'out',binId:'bin'}}).valid,false);
  pipeline.parts.set('p',{packed:true,location:'out'});
  assert.equal(verifyBinInventory({...args,job:{type:'fetch',binId:'other'}}).valid,false,'Packed output must be checked even while another bin moves');
});
test('handoff clearance uses physical geometry and sphere-to-volume distance without mutation',()=>{
  const names=['r1__finger','g1__finger','r1__visual'],api={mjtObj:{mjOBJ_GEOM:{value:5}},mj_id2name:(_m,_t,i)=>names[i]};
  const metadata={supports:[{body:'g1_out_bench',center:[0,0,.75],halfExtents:[.2,.2,.03],tableHeight:.78}]};
  const model={ngeom:3,geom_contype:[1,1,0],geom_conaffinity:[1,1,0],geom_rbound:[.05,.05,10]},data={geom_xpos:new Float64Array([0,0,.85,0,0,.85,0,0,.85])};
  const args={api,metadata,model,data,robotId:'r1',zone:'g1-out'};
  assert.deepEqual(verifyHandoffClearance(args).blocked,['r1__finger']);
  data.geom_xpos[0]=.5;const before=data.geom_xpos.slice();
  assert.equal(verifyHandoffClearance(args).clear,true);assert.deepEqual(data.geom_xpos,before);
  assert.throws(()=>verifyHandoffClearance({...args,zone:'unknown'}),/Unknown/);
});
test('category evidence uses category position, rejects wrong region/index and keeps checking previous parts',()=>{
  const {data,model}=fixture();data.xpos[0]=1;
  const part={id:'p',body:'part',binId:'bin',category:'blue-box',outputSlot:'blue',outputIndex:0};
  const metadata={parts:[part],bins:[{id:'bin',body:'carrier'}],slots:[{id:'blue',category:'blue-box',partSlots:[[1,0,0],[2,0,0]]},{id:'red',category:'red-cylinder',partSlots:[[1,0,0],[2,0,0]]}]};
  const pipeline={parts:new Map([['p',{packed:false}]]),bins:new Map([['bin',{parts:['other','p']}]])};
  const args={data,model,metadata,pipeline,bodyId:()=>1};args.bodyId=name=>name==='part'?0:1;
  const job={type:'pack',partId:'p',destination:'blue',partIndex:0,binId:'bin'};
  assert.equal(verifyBinInventory({...args,job}).valid,true,'Second item in incoming bin can use first spot in its category');
  assert.equal(verifyBinInventory({...args,job:{...job,destination:'red'}}).valid,false);
  assert.equal(verifyBinInventory({...args,job:{...job,partIndex:1}}).valid,false);
  pipeline.parts.set('p',{packed:true,location:'blue'});
  assert.equal(verifyBinInventory({...args,job:{type:'fetch',binId:'other'}}).valid,true);
  data.xpos[0]=2;
  assert.equal(verifyBinInventory({...args,job:{type:'fetch',binId:'other'}}).valid,false);
});
test('a clear wrist cannot release a handoff if its carried bin or loose contents still occupy it',()=>{
  const geoms=['r1__finger','carrier_floor','loose_pin'],bodies=['r1__hand','carrier','pin'];
  const api={mjtObj:{mjOBJ_GEOM:{value:5},mjOBJ_BODY:{value:1}},mj_id2name:(_m,type,i)=>(type===5?geoms:bodies)[i]};
  const metadata={supports:[{body:'g1_out_bench',center:[0,0,.75],halfExtents:[.2,.2,.03],tableHeight:.78}]};
  const model={ngeom:3,geom_bodyid:[0,1,2],geom_contype:[1,1,1],geom_conaffinity:[1,1,1],geom_rbound:[.05,.1,.03]};
  const data={geom_xpos:new Float64Array([.6,0,.85,0,0,.85,.1,0,.85])};
  const args={api,metadata,model,data,robotId:'r1',zone:'g1-out',objectBodies:['carrier','pin']};
  assert.deepEqual(verifyHandoffClearance(args).blocked,['carrier_floor','loose_pin']);
  data.geom_xpos[3]=.6;assert.deepEqual(verifyHandoffClearance(args).blocked,['loose_pin']);
  data.geom_xpos[6]=.6;assert.equal(verifyHandoffClearance(args).clear,true);
});
