import * as THREE from '../vendor/three.module.min.js';

export const PIPE_RADIUS=.020;
export const PIPE_HALF_HEIGHT=.026;
export const PIPE_MASS=.042;
export function normalizePipeSeed(seed=1){
  if(typeof seed==='string'){let h=2166136261;for(const c of seed){h^=c.charCodeAt(0);h=Math.imul(h,16777619);}return h>>>0;}
  return Number.isFinite(Number(seed))?Number(seed)>>>0:1;
}
function rng(seed){let value=seed>>>0;return()=>{value+=0x6D2B79F5;let n=Math.imul(value^(value>>>15),1|value);n^=n+Math.imul(n^(n>>>7),61|n);return((n^(n>>>14))>>>0)/4294967296;};}

/** Seeded, physically separated samples in the near half of each source table.
 * Randomness is constrained to safe working bands, not an arbitrary tabletop
 * distribution. The far two frames each reserve four non-overlapping slots.
 */
export function generatePipeLayout(config,{seed=1}={}){
  seed=normalizePipeSeed(seed);const random=rng(seed),jitter=()=> (random()-.5)*.006;
  const profile=config.id==='r1'?{near:.72,far:[.85,.92],bin:.14}:
    config.id==='c5'?{near:.34,far:[.52,.59],bin:.225}:
    {near:.315,far:[.44,.50],bin:.10};
  const y=config.tableHeight+PIPE_HALF_HEIGHT;
  const bins=['red','yellow'].map((colorKey,index)=>{
    const sign=index===0?1:-1,center=new THREE.Vector3((profile.far[0]+profile.far[1])/2,config.tableHeight+.002,sign*profile.bin);
    const slots=[];
    // Far row first keeps the later placement clear of already placed pipes.
    for(const x of [...profile.far].reverse())for(const dz of config.id==='g1'?(x===profile.far[0]?[sign*.045,-sign*.045]:[-sign*.045,sign*.045]):[-.045,.045])slots.push(new THREE.Vector3(x,y,center.z+dz));
    return{id:colorKey,colorKey,label:colorKey==='red'?'红色放置框':'黄色放置框',color:colorKey==='red'?0xef5260:0xf3ce43,
      center,halfX:(profile.far[1]-profile.far[0])/2+.026,halfZ:.071,slots};
  });
  const targets=[],queue=[],colors=['red','red','red','red','yellow','yellow','yellow','yellow'],counts={red:0,yellow:0};
  for(let i=colors.length-1;i>0;i--){const j=Math.floor(random()*(i+1));[colors[i],colors[j]]=[colors[j],colors[i]];}
  // G1's front pairs mix seeded color sides. Its rear row keeps red on the
  // right and yellow on the left: placing in the nearer bin row then leaves
  // the Dex1 palm over the source cell it has just emptied, not a live pipe.
  if(config.id==='g1'){
    for(let pair=0;pair<4;pair++){const first=random()<.5?'red':'yellow';colors[pair*2]=first;colors[pair*2+1]=first==='red'?'yellow':'red';}
    for(let pair=2;pair<4;pair++){colors[pair*2]='red';colors[pair*2+1]='yellow';}
  }
  // Never accidentally present an already color-partitioned left/right row.
  if(colors.every((color,index)=>color===(index%2?'yellow':'red'))||colors.every((color,index)=>color===(index%2?'red':'yellow')))[colors[0],colors[1]]=[colors[1],colors[0]];
  // Each dual-arm row is visited outer-to-inner, right-to-left. The source
  // colors remain independent of the working arm and both colors sort by bin.
  for(let item=0;item<8;item++){
    const colorKey=colors[item],bin=bins.find(b=>b.id===colorKey),index=counts[colorKey]++,sign=item%2===0?1:-1;
    const id=`pipe-${colorKey}-${index+1}`;
    const target={id,body:`pipe_${colorKey}_${index+1}`,label:`${colorKey==='red'?'红':'黄'}色圆管 ${index+1}`,shape:'cylinder',colorKey,
      color:colorKey==='red'?0xe93531:0xffc529,zoneColor:bin.color,h:PIPE_HALF_HEIGHT,halfX:PIPE_RADIUS,halfZ:PIPE_RADIUS,
      start:new THREE.Vector3((config.id==='r1'?(item<4?.70:.78):config.id==='g1'?(item<4?.295:.385):profile.near)+jitter(),y,
        sign*(config.id==='r1'?(Math.floor(item/2)%2===0?.105:.035):config.id==='g1'?(Math.floor(item/2)%2===0?.135:.045):(.28-Math.floor(item/2)*.08))+jitter()),
      drop:bin.slots[index].clone(),rotation:new THREE.Quaternion(),slotIndex:index,binId:colorKey};
    targets.push(target);queue.push(id);
  }
  // G1 follows physical source side; R1 uses its validated color-safe R/L
  // sequence. Both alternate working arms and visit each color's far slots
  // first. Initial colors remain mixed independently of the destination bins.
  const byColor={red:targets.filter(t=>t.colorKey==='red'),yellow:targets.filter(t=>t.colorKey==='yellow')};
  for(const colorKey of ['red','yellow']){
    const sign=colorKey==='red'?1:-1,items=byColor[colorKey],bin=bins.find(item=>item.id===colorKey);
    if(config.id!=='g1')items.sort((a,b)=>((a.start.z*sign<0?10:0)+a.start.x+Math.abs(a.start.z))-((b.start.z*sign<0?10:0)+b.start.x+Math.abs(b.start.z)));
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
