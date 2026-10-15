import * as THREE from '../vendor/three.module.min.js';

export const PIPE_RADIUS=.020;
export const PIPE_HALF_HEIGHT=.026;
export const PIPE_MASS=.042;
export function normalizePipeSeed(seed=1){
  if(typeof seed==='string'){let h=2166136261;for(const c of seed){h^=c.charCodeAt(0);h=Math.imul(h,16777619);}return h>>>0;}
  return Number.isFinite(Number(seed))?Number(seed)>>>0:1;
}
function rng(seed){let value=seed>>>0;return()=>{value+=0x6D2B79F5;let n=Math.imul(value^(value>>>15),1|value);n^=n+Math.imul(n^(n>>>7),61|n);return((n^(n>>>14))>>>0)/4294967296;};}

function sampleC5Sources(seed,y){
  const random=rng(seed);
  // A continuous reachable arc, not a shuffled grid. The inner radius avoids
  // folded-back wrist branches; the spacing reserves a live Dex1 corridor.
  for(let batch=0;batch<40;batch++){
    const points=[];
    for(let attempt=0;attempt<3000&&points.length<8;attempt++){
      const z=(random()*2-1)*.29,x=.29+random()*.14;
      const point=new THREE.Vector3(x,y,z);
      if(Math.hypot(x,z)>=.37&&points.every(other=>other.distanceTo(point)>=.095))points.push(point);
    }
    if(points.length===8)return points;
  }
  throw new Error('无法生成间距足够的圆管布局，请重新随机摆放');
}

function sampleR1Sources(seed,y,colors){
  const random=rng(seed);
  // Broad continuous working cells leave a return corridor. Visit the near,
  // outer pipes first (below) rather than folding back across a live neighbor.
  for(let batch=0;batch<40;batch++){
    const points=[];
    for(let attempt=0;attempt<3000&&points.length<8;attempt++){
      const color=colors[points.length],ordinal=colors.slice(0,points.length).filter(other=>other===color).length;
      const front=ordinal<2,outer=ordinal%2===0,sign=color==='red'?1:-1;
      const x=(front?.68:.755)+random()*(front?.05:.044);
      const z=sign*((outer?.085:.025)+random()*(outer?.04:.025));
      const point=new THREE.Vector3(x,y,z);
      if(points.every(other=>other.distanceTo(point)>=.060))points.push(point);
    }
    if(points.length===8)return points;
  }
  throw new Error('无法生成间距足够的圆管布局，请重新随机摆放');
}

function sampleG1Sources(seed,y){
  const random=rng(seed),anchors=[
    [.277,.150],[.315,-.130],[.293,.047],[.295,-.045],
    [.357,.185],[.377,-.190],[.377,.065],[.377,-.065],
  ];
  // Irregular source corridors, deliberately not two rows. Shared depth motion
  // preserves the narrow palm corridor; every pipe also moves independently.
  for(let trial=0;trial<4096;trial++){
    const dx=(random()-.5)*.016;
    const points=anchors.map(([x,z])=>new THREE.Vector3(x+dx+(random()-.5)*.006,y,z+(random()-.5)*.018));
    if(points.some(point=>point.x>.389||point.x<.265)||points.slice(2,4).some(point=>point.x<.283))continue;
    if(points.some((point,i)=>points.some((other,j)=>j>i&&Math.hypot(other.x-point.x,other.z-point.z)<.085)))continue;
    return points;
  }
  throw new Error('无法生成间距足够的圆管布局，请重新随机摆放');
}

/** Seeded, physically separated samples in the near half of each source table.
 * Randomness is constrained to safe working bands, not an arbitrary tabletop
 * distribution. The far two frames each reserve four non-overlapping slots.
 */
export function generatePipeLayout(config,{seed=1}={}){
  seed=normalizePipeSeed(seed);const random=rng(seed);
  const profile=config.id==='r1'?{far:[.85,.92],bin:.14}:
    config.id==='c5'?{far:[.52,.59],bin:.225}:
    {far:[.44,.50],bin:.10};
  const y=config.tableHeight+PIPE_HALF_HEIGHT;
  let sourcePoints=config.id==='c5'?sampleC5Sources(seed,y):config.id==='g1'?sampleG1Sources(seed,y):null;
  const bins=['red','yellow'].map((colorKey,index)=>{
    const sign=index===0?1:-1,oldHalfX=(profile.far[1]-profile.far[0])/2+.026;
    const halfX=oldHalfX*1.5,halfZ=Math.min(.071*1.4,profile.bin-.006);
    // Grow toward the far edge without moving the proven drop slots or the
    // frame's near edge into the source objects / gripper approach corridor.
    const center=new THREE.Vector3((profile.far[0]+profile.far[1])/2+halfX-oldHalfX,config.tableHeight+.002,sign*profile.bin);
    const slots=[];
    // Far row first keeps the later placement clear of already placed pipes.
    for(const x of [...profile.far].reverse())for(const dz of config.id==='g1'?(x===profile.far[0]?[sign*.045,-sign*.045]:[-sign*.045,sign*.045]):[-.045,.045])slots.push(new THREE.Vector3(x,y,center.z+dz));
    return{id:colorKey,colorKey,label:colorKey==='red'?'红色放置框':'黄色放置框',color:colorKey==='red'?0xef5260:0xf3ce43,
      center,halfX,halfZ,slots};
  });
  const targets=[],queue=[],colors=['red','red','red','red','yellow','yellow','yellow','yellow'],counts={red:0,yellow:0};
  for(let i=colors.length-1;i>0;i--){const j=Math.floor(random()*(i+1));[colors[i],colors[j]]=[colors[j],colors[i]];}
  // G1's front pairs mix seeded color sides. Its rear corridors keep red on the
  // right and yellow on the left: placing in the nearer bin row then leaves
  // the Dex1 palm over the source cell it has just emptied, not a live pipe.
  if(config.id==='g1'){
    for(let pair=0;pair<4;pair++){const first=random()<.5?'red':'yellow';colors[pair*2]=first;colors[pair*2+1]=first==='red'?'yellow':'red';}
    for(let pair=2;pair<4;pair++){colors[pair*2]='red';colors[pair*2+1]='yellow';}
  }
  // Keep G1's first two color pairs mixed; R1 positions follow color-safe sides.
  if(colors.every((color,index)=>color===(index%2?'yellow':'red'))||colors.every((color,index)=>color===(index%2?'red':'yellow')))[colors[0],colors[1]]=[colors[1],colors[0]];
  if(config.id==='r1')sourcePoints=sampleR1Sources(seed,y,colors);
  // Physical source points are sampled separately from the color RNG so seed
  // reproduction never depends on how many color shuffles were consumed.
  for(let item=0;item<8;item++){
    const colorKey=colors[item],bin=bins.find(b=>b.id===colorKey),index=counts[colorKey]++;
    const id=`pipe-${colorKey}-${index+1}`;
    const target={id,body:`pipe_${colorKey}_${index+1}`,label:`${colorKey==='red'?'红':'黄'}色圆管 ${index+1}`,shape:'cylinder',colorKey,
      color:colorKey==='red'?0xe93531:0xffc529,zoneColor:bin.color,h:PIPE_HALF_HEIGHT,halfX:PIPE_RADIUS,halfZ:PIPE_RADIUS,
      start:sourcePoints[item],
      drop:bin.slots[index].clone(),rotation:new THREE.Quaternion(),slotIndex:index,binId:colorKey};
    targets.push(target);queue.push(id);
  }
  // G1 follows physical source side; R1 uses its validated color-safe R/L
  // sequence. Both alternate working arms and visit each color's far slots
  // first. R1 clears nearby outer pipes before its return path can sweep them.
  const byColor={red:targets.filter(t=>t.colorKey==='red'),yellow:targets.filter(t=>t.colorKey==='yellow')};
  for(const colorKey of ['red','yellow']){
    const sign=colorKey==='red'?1:-1,items=byColor[colorKey],bin=bins.find(item=>item.id===colorKey);
    const lateralWeight=config.id==='r1'?-1:1;
    if(config.id!=='g1')items.sort((a,b)=>((a.start.z*sign<0?10:0)+a.start.x+lateralWeight*Math.abs(a.start.z))-((b.start.z*sign<0?10:0)+b.start.x+lateralWeight*Math.abs(b.start.z)));
    items.forEach((target,index)=>{target.drop.copy(bin.slots[index]);target.slotIndex=index;});
  }
  queue.length=0;
  if(config.id==='g1')queue.push(...targets.map(target=>target.id));
  else for(let i=0;i<4;i++)queue.push(byColor.red[i].id,byColor.yellow[i].id);
  return{seed,targets,bins,queue};
}

export function pipeInsideBin(position,bin,{margin=0,radius=PIPE_RADIUS}={}){
  return Math.abs(position.x-bin.center.x)+radius+margin<=bin.halfX&&Math.abs(position.z-bin.center.z)+radius+margin<=bin.halfZ;
}
