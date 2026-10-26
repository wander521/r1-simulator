/** Logistics only. No joint, body, clock or camera mutation is permitted here.
 * A port performs real motion; only validated physical completion advances stock.
 * The same ledger is usable by the browser worker and deterministic tests. */
export const PIPELINE_ROBOTS=Object.freeze(['g1','r1','c5']);
const clone=value=>structuredClone(value);
const requireThat=(condition,message)=>{if(!condition)throw new Error(message);};

export class BinPipeline {
  constructor({bins,fullSlots=['full-1','full-2'],inputSlots=['input-1','input-2'],returnSlots,outputSlots,partDestinations=null,durationSeconds=300}) {
    requireThat(Array.isArray(bins)&&bins.length>0,'必须提供预装料盒库存');
    requireThat(Number.isFinite(durationSeconds)&&durationSeconds>0,'无效作业时长');
    this.durationSeconds=durationSeconds;this.elapsedSeconds=0;this.lastTime=null;this.state='idle';this.sequence=0;this.generation=0;
    this.bins=new Map();this.parts=new Map();this.slots=new Map();this.jobs=new Map();this.leases=new Map();this.events=[];
    this.robots=Object.fromEntries(PIPELINE_ROBOTS.map(id=>[id,{id,state:'home',enabled:true,jobId:null,error:null}]));
    for(const bin of bins){
      requireThat(bin.id&&!this.bins.has(bin.id)&&Array.isArray(bin.parts)&&bin.parts.length>0,'料盒 ID 必须唯一且非空');
      const ids=bin.parts.map(part=>typeof part==='string'?part:part.id);
      for(const id of ids){requireThat(id&&!this.parts.has(id),'零件 ID 必须唯一');this.parts.set(id,{id,binId:bin.id,location:'bin',packed:false});}
      this.bins.set(bin.id,{id:bin.id,parts:ids,location:bin.source??`wall:${bin.id}`,admitted:false,output:null,packed:0});
    }
    const groups={full:fullSlots,input:inputSlots,returned:returnSlots??bins.map((_,i)=>`return-${i+1}`),output:outputSlots??bins.map((_,i)=>`output-${i+1}`)};
    for(const [kind,ids]of Object.entries(groups))for(const id of ids){requireThat(id&&!this.slots.has(id),'缓冲位 ID 必须唯一');this.slots.set(id,{id,kind,binId:null});}
    this.partDestinations=partDestinations===null?null:clone(partDestinations);
    if(this.partDestinations){
      const occupied=new Set();
      for(const id of this.parts.keys()){
        const target=this.partDestinations[id],key=`${target?.destination}:${target?.partIndex}`;
        requireThat(this.slots.get(target?.destination)?.kind==='output'&&Number.isInteger(target.partIndex)&&target.partIndex>=0&&target.partIndex<2,'缺少有效分类落点');
        requireThat(!occupied.has(key),'分类落点重复');occupied.add(key);
      }
    }
    requireThat(fullSlots.length&&inputSlots.length,'满盒和加工缓冲不可为空');
    this.assertInvariants();
  }
  log(message){this.events.push({time:this.elapsedSeconds,message});if(this.events.length>100)this.events.shift();}
  start(){
    requireThat(['idle','paused'].includes(this.state),'当前状态不能启动');
    requireThat(!Object.values(this.robots).some(r=>r.error),'先处理故障再继续');
    this.state=this.elapsedSeconds>=this.durationSeconds?'draining':'running';this.lastTime=null;
    this.log('独立流水线启动：G1 取满盒，R1 运输及回收，C5 分类放置。');
  }
  pause(){if(!['running','draining','homing'].includes(this.state))return;this.resumeState=this.state;this.state='paused';this.lastTime=null;this.log('暂停推进，执行端保持当前夹持及关节目标。');}
  resume(){requireThat(this.state==='paused','流水线未暂停');requireThat(!Object.values(this.robots).some(r=>r.error),'仍有未处理故障');this.state=this.resumeState??'running';this.lastTime=null;}
  /** Called with a monotonic wall clock in seconds, never simulation time. */
  tick(time){
    requireThat(Number.isFinite(time),'无效单调时钟');
    if(this.lastTime!==null)requireThat(time>=this.lastTime,'时钟不可倒退');
    if(!['running','draining','homing'].includes(this.state)){this.lastTime=null;return [];}
    if(this.lastTime!==null&&this.state==='running')this.elapsedSeconds+=time-this.lastTime;
    this.lastTime=time;
    if(this.state==='running'&&this.elapsedSeconds>=this.durationSeconds){this.state='draining';this.log('五分钟作业窗口结束：不再取新料盒，排空在途批次。');}
    const issued=[];
    for(const id of PIPELINE_ROBOTS){
      const robot=this.robots[id];if(!robot.enabled||robot.error||robot.jobId)continue;
      if(robot.state==='home'&&this.state!=='homing'){issued.push(this.issue(id,'ready',{}));continue;}
      if(this.state==='homing'){if(robot.state!=='home')issued.push(this.issue(id,'home',{}));continue;}
      const next=this.next(id);if(next)issued.push(next);
    }
    if(this.state==='draining'&&this.isDrained())this.state='homing';
    if(this.state==='homing'&&Object.values(this.robots).every(r=>r.state==='home'&&!r.jobId)){this.state='complete';this.log('在途物料全部收尾，三机已回初始位。');}
    this.assertInvariants();return issued.map(clone);
  }
  freeSlot(kind){return [...this.slots.values()].find(s=>s.kind===kind&&!s.binId&&!this.leases.has(`slot:${s.id}`));}
  availableBins(predicate){return [...this.bins.values()].filter(b=>!this.leases.has(`bin:${b.id}`)&&predicate(b));}
  next(robot){
    if(robot==='g1'){
      if(this.state!=='running')return null;
      const bin=this.availableBins(b=>!b.admitted)[0],slot=this.freeSlot('full');
      // Reserve final output and empty return capacity at admission. This
      // prevents taking a full bin which cannot ever leave the downstream cell.
      const output=this.freeSlot('output'),returned=this.freeSlot('returned');
      if(!bin||!slot||(!this.partDestinations&&!output)||!returned)return null;
      const job=this.issue(robot,'fetch',{binId:bin.id,source:bin.location,destination:slot.id},[`bin:${bin.id}`,`slot:${slot.id}`]);
      if(!job)return null;
      bin.admitted=true;bin.output=this.partDestinations?null:output.id;bin.returnSlot=returned.id;
      // Category subslots are preallocated uniquely for the finite inventory.
      if(!this.partDestinations)output.binId=bin.id;
      returned.binId=bin.id; // Capacity reservation, not physical bin occupancy.
      return job;
    }
    if(robot==='r1'){
      const empty=this.availableBins(b=>this.slots.get(b.location)?.kind==='input'&&b.packed===b.parts.length)[0];
      if(empty)return this.issue(robot,'return-empty',{binId:empty.id,source:empty.location,destination:empty.returnSlot},[`bin:${empty.id}`,`slot:${empty.location}`]);
      const bin=this.availableBins(b=>this.slots.get(b.location)?.kind==='full')[0],slot=this.freeSlot('input');
      if(!bin||!slot)return null;
      return this.issue(robot,'deliver',{binId:bin.id,source:bin.location,destination:slot.id},[`bin:${bin.id}`,`slot:${bin.location}`,`slot:${slot.id}`]);
    }
    // C5 always finishes its current bin before switching; transport can work
    // at the other input slot concurrently (slot leases, not a cell-wide lock).
    const candidates=this.availableBins(b=>this.slots.get(b.location)?.kind==='input'&&b.packed<b.parts.length);
    const bin=candidates.find(b=>b.id===this.currentC5Bin)??candidates[0];if(!bin)return null;
    const part=bin.parts.find(id=>!this.parts.get(id).packed);this.currentC5Bin=bin.id;
    const target=this.partDestinations?.[part]??{destination:bin.output,partIndex:bin.packed};
    return this.issue(robot,'pack',{binId:bin.id,partId:part,source:bin.location,...target},[`bin:${bin.id}`,`slot:${bin.location}`,`slot:${target.destination}`]);
  }
  issue(robot,type,fields,resources=[]){
    // Failed reservations do not mutate ownership or consume a job ID.
    if(resources.some(resource=>this.leases.has(resource)))return null;
    const id=`${this.generation}:${++this.sequence}`,job={id,generation:this.generation,robot,type,...fields,resources,status:'reserved'};
    this.jobs.set(id,job);for(const resource of resources)this.leases.set(resource,id);
    Object.assign(this.robots[robot],{jobId:id,state:type,error:null});this.log(`${robot}: ${type}${fields.binId?` ${fields.binId}`:''}`);return job;
  }
  planning(jobId){const job=this.job(jobId);requireThat(job.status==='reserved','任务已进入执行');job.status='planning';}
  enterZone(jobId,zone){
    requireThat(['g1-out','c5-in','empty-return'].includes(zone),'未知交接区');
    const job=this.job(jobId),resource=`zone:${zone}`,owner=this.leases.get(resource);
    // The two reachable placement marks share the wrist approach envelope.
    // Do not admit another fetch while a previously delivered bin occupies
    // it. A mobile job releases its source only after physically departing.
    if(zone==='g1-out'&&job.type==='fetch'&&[...this.slots.values()].some(s=>s.kind==='full'&&s.binId&&s.binId!==job.binId))return false;
    if(owner&&owner!==jobId)return false;
    if(!owner){this.leases.set(resource,jobId);job.resources.push(resource);}return true;
  }
  leaveZone(jobId,zone,{clear}={}){
    const job=this.job(jobId),resource=`zone:${zone}`;
    requireThat(clear===true&&this.leases.get(resource)===jobId,'机械臂未退出交接区');
    this.leases.delete(resource);job.resources=job.resources.filter(r=>r!==resource);
  }
  executing(jobId){const job=this.job(jobId);requireThat(['reserved','planning'].includes(job.status),'任务不可重复启动');job.status='executing';}
  sourceDeparted(jobId,evidence){
    const job=this.job(jobId),bin=this.bins.get(job.binId),source=this.slots.get(job.source),resource=`slot:${job.source}`;
    requireThat(job.robot==='r1'&&['deliver','return-empty'].includes(job.type)&&job.status==='executing','只有运输任务可以释放源位');
    requireThat(!job.sourceVacated,'源位不可重复释放');
    requireThat(evidence?.objectId===job.binId&&evidence.clear===true&&evidence.dualContact===true&&evidence.lifted===true,'缺少夹持并退出源位的实物证据');
    requireThat(source?.binId===job.binId&&bin.location===job.source&&this.leases.get(resource)===jobId,'源位物料归属不匹配');
    source.binId=null;bin.location=`transport:${job.robot}`;job.sourceVacated=true;
    this.leases.delete(resource);job.resources=job.resources.filter(r=>r!==resource);
    this.assertInvariants();
  }
  job(id){const job=this.jobs.get(id);requireThat(job&&job.generation===this.generation&&this.robots[job.robot].jobId===id,'过期或未知任务');return job;}
  /** The port must first verify physical identity, dual contact/lift, stable
   * support at the destination, and an arm clear of the shared zone. */
  complete(jobId,evidence){
    const job=this.job(jobId);requireThat(job.status==='executing','只有执行中的任务可以完成');
    requireThat(evidence?.ready===true,'机械臂未退出交接区/到达准备位');
    if(!['ready','home'].includes(job.type)){
      requireThat(evidence.objectId===(job.partId??job.binId),'物料身份不匹配');
      requireThat(evidence.destination===job.destination,'放置位置不匹配');
      requireThat(evidence.dualContact===true&&evidence.lifted===true&&evidence.supported===true,'缺少真实夹持、提升或放稳证据');
      const bin=this.bins.get(job.binId);
      if(job.type==='pack'){
        const part=this.parts.get(job.partId);requireThat(!part.packed,'零件不可重复装盒');part.packed=true;part.location=job.destination;bin.packed++;
      }else{
        const from=this.slots.get(job.source);if(!job.sourceVacated&&from&&from.kind!=='output'&&from.kind!=='returned')from.binId=null;
        const to=this.slots.get(job.destination);requireThat(to&&!to.binId||to?.binId===bin.id,'目标位已被占用');to.binId=bin.id;bin.location=job.destination;
      }
    }
    this.release(job);job.status='complete';job.evidence=clone(evidence);
    Object.assign(this.robots[job.robot],{jobId:null,state:job.type==='home'?'home':'ready',error:null});this.assertInvariants();
  }
  release(job){for(const resource of job.resources){requireThat(this.leases.get(resource)===job.id,'资源所有者不一致');this.leases.delete(resource);}}
  fail(jobId,message){const job=this.job(jobId);job.status='failed';this.robots[job.robot].error=String(message);this.log(`${job.robot} 安全保持：${message}`);}
  /** Retry is explicit and retains all leases: a failed grasp may be holding
   * a load, so its source/destination cannot be reassigned behind its back. */
  retry(jobId){const job=this.job(jobId);requireThat(job.status==='failed','任务没有失败');job.status='reserved';this.robots[job.robot].error=null;return clone(job);}
  isDrained(){return !Object.values(this.robots).some(r=>r.jobId)&&[...this.bins.values()].filter(b=>b.admitted).every(b=>this.slots.get(b.location)?.kind==='returned');}
  snapshot(){return {state:this.state,elapsedSeconds:this.elapsedSeconds,durationSeconds:this.durationSeconds,robots:clone(this.robots),bins:clone([...this.bins.values()]),parts:clone([...this.parts.values()]),slots:clone([...this.slots.values()]),jobs:clone([...this.jobs.values()]),events:clone(this.events)};}
  assertInvariants(){
    for(const [resource,id]of this.leases){const job=this.jobs.get(id);requireThat(job&&job.resources.includes(resource)&&!['complete'].includes(job.status),'悬空资源锁');}
    for(const robot of Object.values(this.robots))if(robot.jobId)requireThat(this.jobs.get(robot.jobId)?.robot===robot.id,'机器人任务归属错误');
    for(const bin of this.bins.values()){
      requireThat(bin.packed===bin.parts.filter(id=>this.parts.get(id).packed).length,'库存计数不守恒');
      if(this.slots.has(bin.location))requireThat(this.slots.get(bin.location).binId===bin.id,'缓冲位物料身份错误');
      if(bin.location.startsWith('transport:')){
        const owner=this.jobs.get(this.leases.get(`bin:${bin.id}`));
        requireThat(owner?.sourceVacated&&owner.binId===bin.id&&owner.robot===bin.location.slice(10),'在途料盒失去运输归属');
      }
      if(this.slots.get(bin.location)?.kind==='returned')requireThat(bin.packed===bin.parts.length,'非空料盒不能回收');
    }
    return true;
  }
}
