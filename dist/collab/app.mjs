import {createCollaborationView} from './view.mjs?v=20261026-load-2';
import {createCollaborationEngine} from './engine.mjs?v=20261026-load-2';

const $=selector=>document.querySelector(selector);
const labels={r1:'星海图 R1',g1:'宇树 G1',c5:'遨博 C5'};
const operations={fetch:'取满盒',deliver:'送满盒','return-empty':'回收空盒',pack:'分类放置',ready:'进入准备位',home:'返回初始位'};
const extraStates={paused:'已暂停',homing:'返回初始位',home:'初始位',ready:'准备就绪',readying:'进入准备位',recenter:'收臂对齐',raise:'抬臂准备',return_clear:'退出作业区',return_align:'回准备位',operation_return:'回准备位',operation_ready:'确认准备位',navigation:'底盘移动',transport_hold:'载盒移动'};
const states={idle:'待命',running:'作业中',waiting:'等待恢复','awaiting-material':'等待实体补料',draining:'安全收尾',stopped:'已停止',failed:'需要处理',complete:'已结束',loading:'加载中',planning:'规划路径',spread:'展开双臂',crouch:'调整躯干',align_clear:'避让对齐',align:'对齐物件',approach:'靠近物件',descend:'下降抓取',closing:'夹爪闭合',lift:'抬升物件',transfer:'搬运中',place:'放置中',release:'松开夹爪',retreat:'退出作业区',settling:'核验放置',return:'返回待命',reaction_home:'返回待命'};
const clock=value=>{const seconds=Math.max(0,Math.floor(Number(value)||0));return`${String(Math.floor(seconds/60)).padStart(2,'0')}:${String(seconds%60).padStart(2,'0')}`;};
const text=(selector,value)=>{const node=$(selector),next=String(value??'—');if(node.textContent!==next)node.textContent=next;};
let view=null,engine=null,ready=false,disposed=false,lastUI=0,lastEventSignature='',pendingCommandUntil=0,initializationError=null;
const loadProgress=new Map();let initializationVersion=0,animationFrame=0;
const loadTimings={};

function showLoading(message,{error=false}={}){
  $('#load-notice').hidden=false;$('#load-notice').dataset.error=String(error);
  text('#load-title',error?'工作站未能就绪':'正在准备共享工作站');text('#load-message',message);
  $('#retry').hidden=!error;$('#load-progress').hidden=error;
  if(error){text('#run-state','加载失败');$('#run-state').dataset.tone='failed';text('#run-message','请重新加载；作业计时尚未开始。');}
}
function progress(event={}){
  if(disposed||initializationError)return;
  const key=event.id??event.part??'physics',value=Number(event.value);
  if(Number.isFinite(value))loadProgress.set(key,Math.max(0,Math.min(1,value)));
  const sum=['r1','g1','c5','physics'].reduce((total,id)=>total+(loadProgress.get(id)??0),0);
  $('#load-progress').value=sum/4;
  if(event.label)text('#load-message',event.label);
}
function notice(message){text('#run-message',message);}
function runCommand(action){
  if(!engine||!ready||disposed)return;
  try{action();pendingCommandUntil=performance.now()+180;updateUI(engine.snapshot);}
  catch(error){notice(error.message);pendingCommandUntil=0;}
}
$('#start').addEventListener('click',()=>runCommand(()=>engine.start({durationSeconds:300})));
$('#stop').addEventListener('click',()=>runCommand(()=>engine.stop()));
$('#reset').addEventListener('click',()=>runCommand(()=>engine.reset()));
$('#retry').addEventListener('click',()=>location.reload());
$('#fit-view').addEventListener('click',()=>view?.fitView());
$('#main-view').addEventListener('keydown',event=>{
  if(event.ctrlKey||event.metaKey||event.altKey||event.defaultPrevented)return;
  if(view?.handleCameraKey(event.key))event.preventDefault();
});
const stockRows=new Map();
const locationLabel=value=>value?.startsWith('wall')?'物料墙':value?.startsWith('full')?'出料交接':value?.startsWith('input')?'C5 加工位':value?.startsWith('return')?'空盒回收':value?.startsWith('transport')?'R1 运输中':'流转中';

function updateUI(snapshot){
  if(!snapshot)return;const state=snapshot.state,busy=['running','draining','homing'].includes(state),pending=performance.now()<pendingCommandUntil;
  text('#run-state',snapshot.error?'工位故障':extraStates[state]??states[state]??'准备中');$('#run-state').dataset.tone=snapshot.error||state==='failed'?'failed':state==='complete'?'complete':busy?'running':'';
  text('#elapsed',clock(snapshot.elapsedSeconds));$('.duration').textContent=`/ ${clock(snapshot.durationSeconds??300)}`;
  $('#shift-progress').max=snapshot.durationSeconds??300;$('#shift-progress').value=Math.min(snapshot.elapsedSeconds??0,snapshot.durationSeconds??300);
  text('#completed-orders',snapshot.completedOrders??0);text('#completed-parts',snapshot.completedParts??0);text('#returned-bins',snapshot.returnedBins??0);
  $('#start').disabled=!ready||busy||pending||Boolean(snapshot.error)||['failed','complete'].includes(state);
  text('#start',state==='paused'?'检查并继续作业':'开始 5 分钟作业');
  $('#stop').disabled=!ready||!busy||pending;$('#reset').disabled=!ready||busy||pending;
  for(const id of['r1','g1','c5']){
    const robot=snapshot.robots?.find(row=>row.id===id),row=$(`[data-robot-row="${id}"]`);
    row.dataset.active=String(Boolean(robot?.active));row.dataset.failed=String(Boolean(robot?.error));
    const waiting=robot?.active&&!robot.heldCheck&&!robot.error&&robot.phase?.startsWith('await-')&&robot.state!=='planning'&&!['transport_hold','navigation'].includes(robot.state);
    const status=!robot?'加载中':robot.error?'故障保持':state==='paused'?'保持当前姿态':robot.heldCheck?'恢复路径复核':waiting?'等待交接':extraStates[robot.state]??states[robot.state]??'准备就绪';
    text(`[data-robot-state="${id}"]`,robot?.active&&robot.job?`${operations[robot.job.type]} · ${status}`:status);
  }
  for(const bin of snapshot.bins??[]){
    if(!stockRows.size)$('#bin-stock').replaceChildren();
    let cells=stockRows.get(bin.id);
    if(!cells){const row=document.createElement('tr'),name=document.createElement('th'),location=document.createElement('td'),count=document.createElement('td');name.scope='row';name.textContent=`料盒 ${bin.id.split('-').at(-1)}`;row.append(name,location,count);$('#bin-stock').append(row);cells={location,count};stockRows.set(bin.id,cells);}
    const location=locationLabel(bin.location),count=`${bin.packed} / ${bin.parts.length}`;
    if(cells.location.textContent!==location)cells.location.textContent=location;if(cells.count.textContent!==count)cells.count.textContent=count;
  }
  const exhausted=snapshot.bins?.length&&snapshot.bins.every(b=>b.location.startsWith('return-'));
  const messages={idle:'四盒预装物料已就绪。G1 取盒、R1 配送，C5 按颜色与形状分入四区。',running:exhausted?'本批物料已处理完，等待窗口结束；不会自动生成新物料。':'独立取盒、运输与分类；单次完成回准备位，交接区域局部互锁。',draining:'五分钟窗口结束，不再取新盒；继续分类在途物料并回收空盒。',homing:'在途批次已排空，三台机器人返回初始位。',paused:'已暂停并保持夹持。继续前会后台复核剩余路径，也可复位重新开始。',complete:'本次分类已结束，在途物料已收尾，三机已回初始位。',failed:'仿真中断，请复位或重新加载工作站。'};
  if(!pending)notice(snapshot.commandError??snapshot.error??messages[state]??'正在准备工作站。');
  const events=snapshot.events??[],signature=JSON.stringify(events);
  if(signature!==lastEventSignature){
    lastEventSignature=signature;const log=$('#event-log');log.replaceChildren();
    for(const event of[...events].reverse()){
      const item=document.createElement('li'),time=document.createElement('time'),message=document.createElement('span');
      time.textContent=clock(event.time);time.dateTime=`PT${Math.max(0,Number(event.time)||0).toFixed(1)}S`;message.textContent=event.message;item.append(time,message);log.append(item);
    }
    if(!events.length){const item=document.createElement('li');item.className='empty-event';item.textContent='工作站准备好后，可开始连续作业。';log.append(item);}
  }
  const working=(snapshot.robots??[]).filter(r=>r.active&&!r.error);
  $('#scene-note').textContent=working.length?`${working.map(r=>`${labels[r.id].split(' ').at(-1)} ${operations[r.job.type]}`).join(' · ')} · 视角可自由拖动`:'拖动 / 方向键旋转 · 滚轮 / +/- 缩放 · Home 全景';
}

async function initialize(){
  const version=++initializationVersion;
  try{
    view=createCollaborationView({canvas:$('#main-view'),container:$('#viewport')});
    loadTimings.sceneMs=performance.now();
    const models=view.loadRobots(progress).then(()=>{loadTimings.modelsMs=performance.now();});
    const physical=createCollaborationEngine({onProgress:progress}).then(value=>{if(disposed||version!==initializationVersion){value.dispose();return null;}engine=value;loadTimings.physicsMs=performance.now();loadProgress.set('physics',1);return value;});
    await Promise.all([models,physical]);if(disposed||version!==initializationVersion||!engine)return;
    ready=true;view.sync(engine.snapshot);$('#load-notice').hidden=true;updateUI(engine.snapshot);
    loadTimings.readyMs=performance.now();
  }catch(error){initializationVersion++;initializationError=error.message;ready=false;engine?.dispose();engine=null;view?.dispose();view=null;showLoading(error.message,{error:true});}
}
function frame(now){
  if(disposed)return;
  try{
    engine?.advance(now);if(engine){view?.sync(engine.snapshot);if(now-lastUI>=100){updateUI(engine.snapshot);lastUI=now;}}
    view?.render();
  }catch(error){initializationError=error.message;ready=false;engine?.dispose();engine=null;showLoading(`渲染或仿真中断：${error.message}`,{error:true});}
  animationFrame=requestAnimationFrame(frame);
}
const resize=()=>view?.resize();
window.addEventListener('resize',resize);const resizeObserver=new ResizeObserver(resize);resizeObserver.observe($('#viewport'));
window.addEventListener('pagehide',()=>{disposed=true;initializationVersion++;cancelAnimationFrame(animationFrame);resizeObserver.disconnect();window.removeEventListener('resize',resize);engine?.dispose();view?.dispose();},{once:true});
// Read-only inspection for the source-bound browser acceptance test.
window.__collabDiagnostics=()=>({ready,error:initializationError,snapshot:engine?.snapshot??null,camera:view?.cameraPose,robotBasePoses:view?.robotBasePoses,loadedRobots:view?.loadedRobots??[],rackGeometry:view?.rackGeometry,loadTimings:{...loadTimings}});
initialize();animationFrame=requestAnimationFrame(frame);
