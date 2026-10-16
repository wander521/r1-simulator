import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';

// This is a source-bound layout-quality test, not IK, physical-grasp or pixel
// evidence. Its thresholds are independent of the currently generated layout.
const root=new URL('../',import.meta.url),startedAt=new Date().toISOString();
const robotIds=['r1','g1','c5'],EPS=1e-10,RADIUS=.020,HALF_HEIGHT=.026;
const thresholds=Object.freeze({
  seedsPerRobot:100,minXSpanM:.040,minZSpanM:.160,
  medianBestTwoRowsRmsM:.006,minBestTwoRowsRmsM:.004,minSeedsBeyondTwoRows:90,
  minSortedAxisP90RangeM:.012,minSourceCenterDistanceM:.050,minDropCenterDistanceM:.050,
  minNearFrameGapM:.005,minFrameGapM:.012,minFrameAreaRatio:1.4,
  minFrameWidthRatio:1.1,minFrameDepthRatio:1.1,minSlotEdgeMarginM:.006,
  minMeanSlotEdgeMarginM:.012,
});
// Constants from the pre-scatter, physically validated layout. Reading these
// from the new generator would make a "larger frame" test compare it to itself.
const baseline={layoutSha256:'78a9eb02bd8893db2f1c573ede63d527ee426647d93ab25868bf09859af6291c',
  r1:{widthM:.122,depthM:.142,areaM2:.017324},
  g1:{widthM:.112,depthM:.142,areaM2:.015904},
  c5:{widthM:.122,depthM:.142,areaM2:.017324}};
const sourcePaths=['work/test-pipe-scatter.mjs','dist/multi/pipe-layout.mjs',
  'dist/multi/robot-runtime.mjs','dist/r1/grasp-controller.mjs','dist/r1/grasp-kinematics.mjs',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/vendor/three.module.min.js'];
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
async function hashes(){return Object.fromEntries(await Promise.all(sourcePaths.map(async path=>
  [path,sha(await readFile(new URL(path,root)))])));}
const sourceHashes=await hashes();
const seeds=[0,1,521,558,592,620,638,660];
for(let n=1;seeds.length<thresholds.seedsPerRobot;n++){
  const seed=Math.imul(n,0x9e3779b9)>>>0;if(!seeds.includes(seed))seeds.push(seed);
}
const point=v=>[v.x,v.y,v.z];
const quantile=(values,q)=>{
  const sorted=[...values].sort((a,b)=>a-b),i=(sorted.length-1)*q,lo=Math.floor(i),hi=Math.ceil(i);
  return sorted[lo]+(sorted[hi]-sorted[lo])*(i-lo);
};
const near=(a,b)=>Math.abs(a-b)<=EPS;
const squaredError=values=>{
  const mean=values.reduce((sum,value)=>sum+value,0)/values.length;
  return values.reduce((sum,value)=>sum+(value-mean)**2,0);
};
// The exact 1D two-means optimum: an optimum's two clusters are contiguous in
// sorted x. This gives the old layout every opportunity to fit any two rows,
// rather than comparing it only to old hard-coded row coordinates.
function bestTwoRowsRms(points){
  const x=points.map(p=>p[0]).sort((a,b)=>a-b);
  return Math.sqrt(Math.min(...x.slice(1).map((_,i)=>
    squaredError(x.slice(0,i+1))+squaredError(x.slice(i+1))))/x.length);
}
function cloudFingerprint(points){
  return JSON.stringify(points.map(p=>[p[0],p[2]].map(v=>Math.round(v*1e6)/1e6))
    .sort((a,b)=>a[0]-b[0]||a[1]-b[1]));
}
function serializeLayout(layout){
  return JSON.stringify({seed:layout.seed,queue:layout.queue,
    targets:layout.targets.map(t=>({id:t.id,body:t.body,colorKey:t.colorKey,shape:t.shape,
      h:t.h,halfX:t.halfX,halfZ:t.halfZ,start:point(t.start),drop:point(t.drop),slotIndex:t.slotIndex,binId:t.binId})),
    bins:layout.bins.map(b=>({id:b.id,colorKey:b.colorKey,center:point(b.center),halfX:b.halfX,
      halfZ:b.halfZ,slots:b.slots.map(point)}))});
}
function scatterStatistics(rows){
  const good=rows.filter(row=>row.points?.length===8&&row.points.every(p=>p.length===3&&p.every(Number.isFinite)));
  const errors=[];
  if(good.length!==thresholds.seedsPerRobot)errors.push(`Expected ${thresholds.seedsPerRobot} valid eight-point layouts, got ${good.length}`);
  if(!good.length)return{pass:false,errors,validLayouts:0};
  const rowMetrics=good.map(row=>({seed:row.seed,bestTwoRowsRmsM:bestTwoRowsRms(row.points),
    xSpanM:Math.max(...row.points.map(p=>p[0]))-Math.min(...row.points.map(p=>p[0])),
    zSpanM:Math.max(...row.points.map(p=>p[2]))-Math.min(...row.points.map(p=>p[2]))}));
  const distinctPointClouds=new Set(good.map(row=>cloudFingerprint(row.points))).size;
  if(distinctPointClouds!==thresholds.seedsPerRobot)errors.push(`Position-only point clouds are not all distinct (${distinctPointClouds})`);
  const medianRms=quantile(rowMetrics.map(row=>row.bestTwoRowsRmsM),.5);
  const beyond=rowMetrics.filter(row=>row.bestTwoRowsRmsM+EPS>=thresholds.minBestTwoRowsRmsM).length;
  if(medianRms+EPS<thresholds.medianBestTwoRowsRmsM)errors.push(`Median best-two-row RMS ${medianRms} m is below ${thresholds.medianBestTwoRowsRmsM} m`);
  if(beyond<thresholds.minSeedsBeyondTwoRows)errors.push(`Only ${beyond} seeds escape a ${thresholds.minBestTwoRowsRmsM} m two-row fit; need ${thresholds.minSeedsBeyondTwoRows}`);
  for(const row of rowMetrics){
    if(row.xSpanM+EPS<thresholds.minXSpanM)errors.push(`Seed ${row.seed}: x span ${row.xSpanM} m is below ${thresholds.minXSpanM} m`);
    if(row.zSpanM+EPS<thresholds.minZSpanM)errors.push(`Seed ${row.seed}: z span ${row.zSpanM} m is below ${thresholds.minZSpanM} m`);
  }
  // Per-axis order statistics ignore target IDs, colors and queue order. Each
  // of the eight spatial ranks must move by >=12 mm over its central 90% range
  // across seeds; color shuffling and +/-3 mm noise cannot meet this condition.
  const axisP90Ranges={};
  for(const [name,axis]of [['x',0],['z',2]]){
    const sorted=good.map(row=>row.points.map(p=>p[axis]).sort((a,b)=>a-b));
    axisP90Ranges[name]=Array.from({length:8},(_,rank)=>{
      const values=sorted.map(values=>values[rank]);return quantile(values,.95)-quantile(values,.05);
    });
    axisP90Ranges[name].forEach((range,rank)=>{
      if(range+EPS<thresholds.minSortedAxisP90RangeM)errors.push(`${name} spatial rank ${rank}: central-90% motion ${range} m is below ${thresholds.minSortedAxisP90RangeM} m`);
    });
  }
  return{pass:errors.length===0,errors,validLayouts:good.length,distinctPointClouds,
    medianBestTwoRowsRmsM:medianRms,seedsBeyondTwoRows:beyond,
    minBestTwoRowsRmsM:Math.min(...rowMetrics.map(row=>row.bestTwoRowsRmsM)),
    maxBestTwoRowsRmsM:Math.max(...rowMetrics.map(row=>row.bestTwoRowsRmsM)),
    minXSpanM:Math.min(...rowMetrics.map(row=>row.xSpanM)),minZSpanM:Math.min(...rowMetrics.map(row=>row.zSpanM)),
    sortedAxisP90RangesM:axisP90Ranges,rowMetrics};
}
function minimumDistance(points){
  let result=Infinity;
  for(let i=0;i<points.length;i++)for(let j=i+1;j<points.length;j++)
    result=Math.min(result,Math.hypot(points[i][0]-points[j][0],points[i][2]-points[j][2]));
  return result;
}
function rectangle(bin){return{minX:bin.center.x-bin.halfX,maxX:bin.center.x+bin.halfX,
  minZ:bin.center.z-bin.halfZ,maxZ:bin.center.z+bin.halfZ};}
function rectangleDistance(a,b){
  return Math.hypot(Math.max(0,a.minX-b.maxX,b.minX-a.maxX),Math.max(0,a.minZ-b.maxZ,b.minZ-a.maxZ));
}
function layoutGeometry(id,seed,layout,config){
  const errors=[],check=(condition,message)=>{if(!condition)errors.push(message);};
  assert(Array.isArray(layout.targets)&&layout.targets.length===8,'Expected eight target pipes');
  assert(Array.isArray(layout.bins)&&layout.bins.length===2,'Expected two color frames');
  const starts=layout.targets.map(target=>point(target.start)),drops=layout.targets.map(target=>point(target.drop));
  assert([...starts,...drops].every(p=>p.every(Number.isFinite)),'Non-finite source/drop point');
  check(new Set(layout.targets.map(target=>target.id)).size===8,'Duplicate target IDs');
  for(const color of ['red','yellow']){
    check(layout.targets.filter(target=>target.colorKey===color).length===4,`Expected four ${color} pipes`);
    check(layout.bins.filter(bin=>bin.colorKey===color).length===1,`Expected one ${color} frame`);
  }
  check(Array.isArray(layout.queue)&&layout.queue.length===8&&new Set(layout.queue).size===8
    &&layout.queue.every(key=>layout.targets.some(target=>target.id===key)),'Queue is not an exact eight-pipe permutation');
  const sourceDistance=minimumDistance(starts),dropDistance=minimumDistance(drops);
  check(sourceDistance+EPS>=thresholds.minSourceCenterDistanceM,`Source centers too close: ${sourceDistance} m`);
  check(dropDistance+EPS>=thresholds.minDropCenterDistanceM,`Destination centers too close: ${dropDistance} m`);
  for(const target of layout.targets){
    check(target.shape==='cylinder'&&near(target.h,HALF_HEIGHT)&&near(target.halfX,RADIUS)&&near(target.halfZ,RADIUS),`${target.id}: original cylinder dimensions changed`);
    for(const [kind,p]of [['source',target.start],['drop',target.drop]]){
      check(p.x-RADIUS+EPS>=config.tableCenter-config.tableHalfX&&p.x+RADIUS<=config.tableCenter+config.tableHalfX+EPS
        &&Math.abs(p.z)+RADIUS<=config.tableHalfZ+EPS,`${target.id}: ${kind} cylinder extends beyond tabletop`);
      check(near(p.y,config.tableHeight+HALF_HEIGHT),`${target.id}: ${kind} height is not on the tabletop`);
    }
    const bin=layout.bins.find(bin=>bin.colorKey===target.colorKey);
    assert(bin,`${target.id}: matching color frame absent`);
    check(Number.isInteger(target.slotIndex)&&target.slotIndex>=0&&target.slotIndex<4,`${target.id}: invalid reserved slot index`);
    const slot=bin.slots[target.slotIndex];
    check(slot&&Math.hypot(...point(slot).map((value,index)=>value-point(target.drop)[index]))<=EPS,`${target.id}: drop differs from its matching reserved slot`);
  }
  const bins=layout.bins.map(bin=>{
    assert(Number.isFinite(bin.halfX)&&bin.halfX>0&&Number.isFinite(bin.halfZ)&&bin.halfZ>0,'Invalid frame size');
    assert(Array.isArray(bin.slots)&&bin.slots.length===4,'Expected four reserved slots in each frame');
    const box=rectangle(bin),width=bin.halfX*2,depth=bin.halfZ*2,area=width*depth,old=baseline[id];
    check(box.minX+EPS>=config.tableCenter-config.tableHalfX&&box.maxX<=config.tableCenter+config.tableHalfX+EPS
      &&box.minZ+EPS>=-config.tableHalfZ&&box.maxZ<=config.tableHalfZ+EPS,`${bin.colorKey}: frame extends beyond tabletop`);
    check(width/old.widthM+EPS>=thresholds.minFrameWidthRatio,`${bin.colorKey}: frame width did not visibly increase`);
    check(depth/old.depthM+EPS>=thresholds.minFrameDepthRatio,`${bin.colorKey}: frame depth did not visibly increase`);
    check(area/old.areaM2+EPS>=thresholds.minFrameAreaRatio,`${bin.colorKey}: frame area ratio ${area/old.areaM2} is below ${thresholds.minFrameAreaRatio}`);
    const margins=bin.slots.map(slot=>Math.min(slot.x-RADIUS-box.minX,box.maxX-slot.x-RADIUS,
      slot.z-RADIUS-box.minZ,box.maxZ-slot.z-RADIUS));
    const minMargin=Math.min(...margins),meanMargin=margins.reduce((sum,v)=>sum+v,0)/margins.length;
    check(minMargin+EPS>=thresholds.minSlotEdgeMarginM,`${bin.colorKey}: a full-radius reserved slot loses the original 6 mm edge clearance`);
    check(meanMargin+EPS>=thresholds.minMeanSlotEdgeMarginM,`${bin.colorKey}: mean usable slot-edge margin ${meanMargin} m is below ${thresholds.minMeanSlotEdgeMarginM} m`);
    const members=layout.targets.filter(target=>target.colorKey===bin.colorKey);
    check(new Set(members.map(target=>target.slotIndex)).size===4,`${bin.colorKey}: destination slots repeated`);
    return{colorKey:bin.colorKey,...box,widthM:width,depthM:depth,areaM2:area,
      widthRatio:width/old.widthM,depthRatio:depth/old.depthM,areaRatio:area/old.areaM2,
      slotEdgeMarginsM:margins,minSlotEdgeMarginM:minMargin,meanSlotEdgeMarginM:meanMargin};
  });
  const nearGap=Math.min(...bins.map(bin=>bin.minX))-Math.max(...starts.map(p=>p[0]+RADIUS));
  const frameGap=rectangleDistance(bins[0],bins[1]);
  check(nearGap+EPS>=thresholds.minNearFrameGapM,`Near-side full cylinders overlap/approach the far frames: ${nearGap} m gap`);
  check(frameGap+EPS>=thresholds.minFrameGapM,`Color-frame AABBs overlap/approach: ${frameGap} m gap`);
  return{seed,pass:errors.length===0,errors,points:starts,sourceCenterDistanceM:sourceDistance,
    dropCenterDistanceM:dropDistance,nearFrameGapM:nearGap,frameGapM:frameGap,bins};
}

function fixtureRandom(seed){let n=seed>>>0;return()=>{n=(Math.imul(n,1664525)+1013904223)>>>0;return n/2**32;};}
function rowFixture(id){
  return seeds.map(seed=>{
    const random=fixtureRandom(seed),jitter=()=> (random()-.5)*.006;
    return{seed,points:Array.from({length:8},(_,i)=>{
      const sign=i%2===0?1:-1,x=id==='r1'?(i<4?.70:.78):id==='g1'?(i<4?.295:.385):.34;
      const z=sign*(id==='r1'?(Math.floor(i/2)%2===0?.105:.035):id==='g1'?(Math.floor(i/2)%2===0?.135:.045):(.28-Math.floor(i/2)*.08));
      return[x+jitter(),0,z+jitter()];
    })};
  });
}
function negativeFixtures(){
  const checks=[];
  for(const id of robotIds){
    const result=scatterStatistics(rowFixture(id));
    const pass=!result.pass&&result.errors.some(error=>/two-row|two-row RMS/.test(error));
    checks.push({name:`Reject legacy ${id==='c5'?'one':'two'}-row ${id} fixture with only +/-3 mm jitter`,pass,
      medianBestTwoRowsRmsM:result.medianBestTwoRowsRmsM,rejectionReasons:result.errors});
  }
  const fixed=[[.68,0,.115],[.712,0,-.128],[.735,0,.045],[.768,0,-.035],
    [.695,0,-.038],[.738,0,.125],[.782,0,-.10],[.765,0,.048]];
  const result=scatterStatistics(seeds.map((seed,index)=>({seed,points:[...fixed.slice(index%8),...fixed.slice(0,index%8)]})));
  checks.push({name:'Reject fixed scattered positions with only color/ID/array-order shuffling',
    pass:!result.pass&&result.distinctPointClouds===1&&result.errors.some(error=>/spatial rank/.test(error)),
    distinctPointClouds:result.distinctPointClouds,rejectionReasons:result.errors});
  return{pass:checks.every(check=>check.pass),checks};
}

const errors=[],robots=[],fixtureChecks=negativeFixtures();
let attemptedLayouts=0;
try{
  const [{generatePipeLayout,PIPE_RADIUS,PIPE_HALF_HEIGHT},{ROBOT_CONFIGS}]=await Promise.all([
    import('../dist/multi/pipe-layout.mjs'),import('../dist/multi/robot-runtime.mjs')]);
  assert(near(PIPE_RADIUS,RADIUS)&&near(PIPE_HALF_HEIGHT,HALF_HEIGHT),'Production cylinder dimensions changed');
  for(const id of robotIds){
    const rows=[];
    for(const seed of seeds){
      attemptedLayouts++;
      try{
        const layout=generatePipeLayout(ROBOT_CONFIGS[id],{seed}),again=generatePipeLayout(ROBOT_CONFIGS[id],{seed});
        assert.equal(serializeLayout(layout),serializeLayout(again),`${id}/${seed}: explicit seed is not reproducible`);
        rows.push(layoutGeometry(id,seed,layout,ROBOT_CONFIGS[id]));
      }catch(error){rows.push({seed,pass:false,errors:[error.message],points:null});}
    }
    const statistics=scatterStatistics(rows),geometryFailures=rows.filter(row=>!row.pass);
    robots.push({id,pass:statistics.pass&&geometryFailures.length===0,reproducibleLayouts:rows.filter(row=>row.points).length,
      statistics,geometryFailures:geometryFailures.map(({seed,errors})=>({seed,errors})),layouts:rows});
  }
}catch(error){errors.push(error.stack??error.message);}
if(!fixtureChecks.pass)errors.push('A legacy-row/color-only negative fixture was incorrectly accepted');
const finalHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(finalHashes);
if(!sourceFrozen)errors.push('Layout-quality source changed during the run; rerun after freezing it');
const pass=errors.length===0&&robots.length===3&&robots.every(robot=>robot.pass)&&fixtureChecks.pass&&attemptedLayouts===300;
const report={pass,startedAt,mode:'static-scatter-quality',scope:'Deterministic production-layout statistics and geometric clearance only; not IK, physical grasp, native interaction or rendered-pixel evidence',
  attemptedLayouts,seedsPerRobot:thresholds.seedsPerRobot,seeds,thresholds,baseline,fixtureChecks,robots,
  sourceFrozen,sourceHashes,errors};
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)??'work/validation/multi-pipes/scatter-final.json';
const destination=new URL(reportPath,root);await mkdir(new URL('./',destination),{recursive:true});
await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass,attemptedLayouts,sourceFrozen,fixtureChecks:fixtureChecks.checks.map(({name,pass})=>({name,pass})),
  robots:robots.map(robot=>({id:robot.id,pass:robot.pass,medianBestTwoRowsRmsM:robot.statistics.medianBestTwoRowsRmsM,
    seedsBeyondTwoRows:robot.statistics.seedsBeyondTwoRows,geometryFailingSeeds:robot.geometryFailures.length,
    errors:robot.statistics.errors.slice(0,12)})),errors,report:reportPath},null,2));
if(!pass)process.exitCode=1;
