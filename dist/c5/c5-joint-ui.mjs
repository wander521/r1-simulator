import {Vector3} from '../vendor/three.module.min.js';
import {armJointNames,gripperLinkName,GRIPPER_TCP_X,qToOpening,GRIPPER_Q_MIN,GRIPPER_Q_MAX} from './c5-profile.mjs?v=20261015-c5-3';

// Same labels, palette, column order and guide behaviour as reference gh().
// Values passed to this view are radians and metres; it never moves the model.
export const JOINT_COLORS = ['#5ee0ff','#8fd7c1','#f5d76e','#ff9f5a','#ff7ab6','#7aa2ff','#be98ff','#d4b3ff'];
const SVG_NS = 'http://www.w3.org/2000/svg';
const finite = value => Number.isFinite(value);
const clamp = (value,lower,upper) => Math.min(upper,Math.max(lower,value));
const fixed = (value,digits=1) => {
  if(!finite(value))return '—';
  const rounded=Math.round(value*10**digits)/10**digits;
  return (Math.abs(rounded)<.5/10**digits?0:rounded).toFixed(digits);
};
const format = (value,gripper=false) => `${fixed(value*(gripper?1000:180/Math.PI))}${gripper?' mm':'°'}`;
const node = (tag,className,text) => {const el=document.createElement(tag);if(className)el.className=className;if(text!==undefined)el.textContent=text;return el;};

/**
 * project receives a Three Vector3 in world coordinates and returns pixel
 * coordinates relative to root: {x,y,visible}. Call update once per frame so
 * guides track both orbit-camera movement and articulated robot movement.
 * onJoint(name,targetRadians), onOpening(targetMetres) receive clamped values.
 */
export function createJointUI({root,robot,getArm=()=> 'left',onJoint=()=>{},onOpening=()=>{},project}) {
  if(!root||!robot||typeof project!=='function')throw new TypeError('关节标注缺少容器、机器人或投影函数');
  const svg=document.createElementNS(SVG_NS,'svg');svg.classList.add('callout-lines','c5-joint-lines');svg.setAttribute('aria-hidden','true');root.append(svg);
  const abort=new AbortController(),signal=abort.signal,items=[],world=new Vector3();
  let allGuides=false,disposed=false,arm=getArm(),running=false,layoutInvalid=true,lastWidth=0,lastHeight=0,openAnchor=null;
  const finePointer=()=>matchMedia('(hover: hover) and (pointer: fine)').matches;
  const blocked=()=>root.parentElement?.classList.contains('tcp-dragging');

  function closeChips(){for(const item of items){item.chip.classList.remove('is-open');item.header.setAttribute('aria-expanded','false');}openAnchor=null;layoutInvalid=true;}
  function openChip(item,open){
    if(blocked())return;
    // Opening on hover must not move that header under a pending pointerdown.
    // Capture before changing display/width, including when another chip's
    // expansion has already displaced this one from its baseline row.
    if(open&&item.chip.classList.contains('is-open'))return;
    openAnchor=open?{item,top:item.chip.getBoundingClientRect().top-root.getBoundingClientRect().top}:null;
    for(const other of items){const enabled=other===item&&open;other.chip.classList.toggle('is-open',enabled);other.header.setAttribute('aria-expanded',String(enabled));}
    layoutInvalid=true;
  }
  for(let i=0;i<7;i++) {
    const gripper=i===6,label=gripper?'夹爪':`J${i+1}`,color=JOINT_COLORS[i];
    const chip=node('div','callout-chip c5-joint-chip');chip.dataset.index=String(i);chip.style.setProperty('--callout-color',color);
    const header=node('button','callout-header c5-joint-header');header.type='button';header.setAttribute('aria-expanded','false');
    const bodyId=`c5-joint-slider-${i+1}`;header.setAttribute('aria-controls',bodyId);header.setAttribute('aria-label',`${label}：展开手动控制`);
    const name=node('span','callout-name',label),readouts=node('strong'),actual=node('span','actual','—'),target=node('span','target-readout','—');
    actual.title=gripper?'实际最小开口':'实际值';target.title=gripper?'目标最小开口':'目标值';readouts.append(actual,target);header.append(name,readouts);
    const body=node('div','callout-body');body.id=bodyId;
    const slider=node('input');slider.type='range';slider.step=gripper?'.0005':'.01';slider.disabled=true;slider.style.accentColor=color;slider.setAttribute('aria-label',gripper?'夹爪目标最小开口（毫米显示）':`${label} 目标角度（度显示）`);
    const legend=node('div','c5-joint-slider-legend');legend.append(node('span',null,'实际 / 目标'),node('span','c5-joint-limits',''));
    body.append(slider,legend);chip.append(header,body);root.append(chip);
    const line=document.createElementNS(SVG_NS,'line'),dot=document.createElementNS(SVG_NS,'circle');line.classList.add('callout-line');dot.classList.add('callout-dot');dot.setAttribute('r','3.5');
    for(const shape of [line,dot]){shape.style.setProperty('--callout-color',color);shape.setAttribute('visibility','hidden');}svg.append(line,dot);
    const item={index:i,gripper,chip,header,slider,actual,target,line,dot,limits:legend.lastChild,side:i<3?'left':'right',value:NaN,anchor:null};items.push(item);
    chip.addEventListener('pointerdown',event=>event.stopPropagation(),{signal});
    chip.addEventListener('mouseenter',()=>{if(finePointer())openChip(item,true);},{signal});
    chip.addEventListener('mouseleave',()=>{if(finePointer()&&!chip.contains(document.activeElement))openChip(item,false);},{signal});
    header.addEventListener('click',event=>{event.stopPropagation();if(!finePointer()||event.detail===0)openChip(item,!chip.classList.contains('is-open'));},{signal});
    slider.addEventListener('input',()=>{
      if(running||slider.disabled)return;
      const value=clamp(Number(slider.value),Number(slider.min),Number(slider.max));if(!finite(value))return;
      item.value=value;target.textContent=format(value,gripper);slider.setAttribute('aria-valuetext',format(value,gripper));
      if(gripper)onOpening(value);else onJoint(armJointNames(getArm())[i],value);
    },{signal});
    chip.addEventListener('keydown',event=>{if(event.key==='Escape'){closeChips();header.focus();}},{signal});
  }
  root.classList.add('c5-joint-callouts');
  document.addEventListener('pointerdown',event=>{if(!event.target.closest('.c5-joint-chip'))closeChips();},{signal});
  const resize=new ResizeObserver(()=>{layoutInvalid=true;});resize.observe(root);

  function layout() {
    const width=root.clientWidth,height=root.clientHeight;
    if(!layoutInvalid&&width===lastWidth&&height===lastHeight)return;
    if(width!==lastWidth||height!==lastHeight)openAnchor=null;
    lastWidth=width;lastHeight=height;layoutInvalid=false;
    svg.setAttribute('viewBox',`0 0 ${Math.max(1,width)} ${Math.max(1,height)}`);
    const columns=[[],[]];for(const item of items){const side=width>=280?item.side:'right';item.chip.dataset.side=side;columns[side==='left'?0:1].push(item);}columns[0].reverse();
    for(const column of columns){
      // Centre the collapsed headers, not the expanded bodies. An expanded
      // body's extra height may displace siblings but never its own header.
      const heights=column.map(item=>Math.max(1,item.chip.offsetHeight));
      const collapsed=column.map(item=>{const style=getComputedStyle(item.chip);return Math.max(1,item.header.offsetHeight+parseFloat(style.paddingTop)*2+parseFloat(style.borderTopWidth)+parseFloat(style.borderBottomWidth));});
      const sum=collapsed.reduce((a,b)=>a+b,0),count=Math.max(0,column.length-1);
      const gap=count?Math.max(0,Math.min(8,(height-16-sum)/count)):0,total=sum+gap*count;
      let y=clamp((height-total)/2,8,Math.max(8,height-8-total));
      const tops=collapsed.map(h=>{const top=y;y+=h+gap;return top;});
      const expanded=column.findIndex(item=>item.chip.classList.contains('is-open'));
      if(expanded>=0){
        if(openAnchor?.item===column[expanded])tops[expanded]=openAnchor.top;
        for(let i=expanded-1;i>=0;i--)tops[i]=Math.min(tops[i],tops[i+1]-heights[i]-gap);
        for(let i=expanded+1;i<column.length;i++)tops[i]=Math.max(tops[i],tops[i-1]+heights[i-1]+gap);
      }
      column.forEach((item,i)=>{item.chip.style.left=item.chip.dataset.side==='left'?'8px':'auto';item.chip.style.right=item.chip.dataset.side==='right'?'8px':'auto';item.chip.style.transform=`translateY(${tops[i]}px)`;});
    }
  }
  function anchors() {
    const rect=root.getBoundingClientRect();
    for(const item of items){
      const joint=item.gripper?robot.getObjectByName(gripperLinkName(arm)):robot.joints?.[armJointNames(arm)[item.index]];
      if(!joint){item.line.setAttribute('visibility','hidden');item.dot.setAttribute('visibility','hidden');continue;}
      if(item.gripper)joint.localToWorld(world.set(GRIPPER_TCP_X,0,0));else joint.getWorldPosition(world);
      const point=project(world),visible=point&&point.visible!==false&&finite(point.x)&&finite(point.y);
      const show=visible&&(allGuides||item.chip.classList.contains('is-open')||item.chip.matches(':hover, :focus-within'));
      item.line.setAttribute('visibility',show?'visible':'hidden');item.dot.setAttribute('visibility',show?'visible':'hidden');if(!show)continue;
      const box=item.chip.getBoundingClientRect(),x=item.chip.dataset.side==='right'?box.left-rect.left:box.right-rect.left,y=box.top-rect.top+box.height/2;
      item.line.setAttribute('x1',String(point.x));item.line.setAttribute('y1',String(point.y));item.line.setAttribute('x2',String(x));item.line.setAttribute('y2',String(y));item.dot.setAttribute('cx',String(point.x));item.dot.setAttribute('cy',String(point.y));
    }
  }
  function update({joints=[],opening,openingTarget,running:busy=false,openingLower,openingUpper}={}) {
    if(disposed)return;
    const nextArm=getArm();if(nextArm!==arm){arm=nextArm;closeChips();}running=!!busy;
    root.classList.toggle('c5-joints-running',running);
    for(const item of items){
      const name=armJointNames(arm)[item.index],sample=joints.find(j=>j.name===name)||joints[item.index],definition=robot.joints?.[name];
      const lower=item.gripper?(openingLower??Math.min(qToOpening(GRIPPER_Q_MIN),qToOpening(GRIPPER_Q_MAX))):(sample?.lower??definition?.limit?.lower),upper=item.gripper?(openingUpper??Math.max(qToOpening(GRIPPER_Q_MIN),qToOpening(GRIPPER_Q_MAX))):(sample?.upper??definition?.limit?.upper);
      const actual=item.gripper?opening:(sample?.actual??sample?.position),target=item.gripper?openingTarget:sample?.target;
      item.actual.textContent=format(actual,item.gripper);item.target.textContent=format(target,item.gripper);
      const usable=finite(lower)&&finite(upper)&&upper>lower&&finite(target);
      item.slider.disabled=running||!usable;
      if(usable){item.slider.min=String(lower);item.slider.max=String(upper);item.slider.value=String(clamp(target,lower,upper));item.slider.setAttribute('aria-valuetext',format(target,item.gripper));item.value=target;item.limits.textContent=`${format(lower,item.gripper)} – ${format(upper,item.gripper)}`;}
      item.header.setAttribute('aria-label',`${item.gripper?'夹爪':`J${item.index+1}`}：实际 ${format(actual,item.gripper)}，目标 ${format(target,item.gripper)}${running?'，自动任务执行中':''}`);
    }
    layout();anchors();
  }
  return {
    update,closeChips,
    setAllGuides(visible){allGuides=!!visible;anchors();},
    dispose(){if(disposed)return;disposed=true;abort.abort();resize.disconnect();for(const item of items)item.chip.remove();svg.remove();root.classList.remove('c5-joint-callouts','c5-joints-running');}
  };
}

/** Six revolute-joint measurements only. Torque must be qfrc_actuator-derived;
 * no pose-derived torque estimate, and no prismatic units mixed into RMS. */
export function createJointTelemetry(host,{maxSamples=150}={}) {
  if(!host)throw new TypeError('关节数据缺少容器');
  const section=node('section','control-card c5-joint-telemetry');
  const heading=node('div','control-card-heading');heading.append(node('h2',null,'关节反馈'),node('span','c5-joint-units','N·m / rad'));
  const rmsRow=node('p','c5-joint-rms');rmsRow.append(node('span',null,'关节均方根误差'),node('strong',null,'— mrad'));
  const bars=node('div','torque-bars c5-joint-torque-bars'),rows=[];
  for(let i=0;i<6;i++){
    const row=node('div','torque-row'),track=node('span','torque-track'),fill=node('i','torque-fill'),value=node('span','torque-value','— N·m');track.append(fill);row.style.setProperty('--joint-color',JOINT_COLORS[i]);row.append(node('span',null,`J${i+1}`),track,value);bars.append(row);rows.push({row,fill,value});
  }
  const canvases=['关节力矩曲线 · N·m','目标与实际关节误差 · rad'].map((title,i)=>{
    const wrapper=node('div','telemetry-chart c5-joint-chart'),label=node('span',null,title),canvas=node('canvas');canvas.width=552;canvas.height=176;canvas.setAttribute('role','img');canvas.setAttribute('aria-label',title);wrapper.append(label,canvas);section.append(wrapper);return canvas;
  });
  const legend=node('div','c5-joint-chart-legend');for(let i=0;i<6;i++){const label=node('span',null,`J${i+1}`);label.style.color=JOINT_COLORS[i];legend.append(label);}
  section.prepend(heading,rmsRow,bars);section.append(legend);host.append(section);
  const history=[];let disposed=false,lastTime=NaN,lastNames='';
  function draw(canvas,key,minScale){
    const ctx=canvas.getContext('2d');if(!ctx)return;const w=canvas.width,h=canvas.height,left=54,right=8,top=14,bottom=22,pw=w-left-right,ph=h-top-bottom;
    let scale=minScale;for(const frame of history)for(const j of frame.joints)if(finite(j[key]))scale=Math.max(scale,Math.abs(j[key]));
    scale=scale>0?scale*1.12:minScale;
    ctx.clearRect(0,0,w,h);ctx.fillStyle='#020b18';ctx.fillRect(0,0,w,h);ctx.font='19px sans-serif';ctx.textAlign='right';ctx.fillStyle='#96b9c9';ctx.lineWidth=1;
    for(let i=0;i<5;i++){const y=top+ph*i/4,value=scale*(1-i/2);ctx.strokeStyle=i===2?'#3b677d':'#183b50';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(w-right,y);ctx.stroke();ctx.fillText(Math.abs(value)<1e-10?'0':Math.abs(value)>=10?value.toFixed(0):value.toFixed(key==='error'?3:1),left-6,y+6);}
    for(let i=0;i<6;i++){ctx.strokeStyle=JOINT_COLORS[i];ctx.lineWidth=2;ctx.beginPath();let started=false;history.forEach((frame,index)=>{const value=frame.joints[i]?.[key];if(!finite(value)){started=false;return;}const x=left+index/Math.max(1,maxSamples-1)*pw,y=top+ph/2-clamp(value/scale,-1,1)*ph/2;if(started)ctx.lineTo(x,y);else ctx.moveTo(x,y);started=true;});ctx.stroke();}
    ctx.textAlign='left';ctx.fillStyle='#96b9c9';ctx.fillText('较早',left,h-3);ctx.textAlign='right';ctx.fillText('当前',w-right,h-3);
  }
  function reset(){history.length=0;lastTime=NaN;lastNames='';rmsRow.lastChild.textContent='— mrad';for(const row of rows){row.fill.style.width='0%';row.value.textContent='— N·m';}draw(canvases[0],'torque',1);draw(canvases[1],'error',.004);}
  function update({joints=[],time}={}){
    if(disposed)return null;
    const selected=joints.filter(j=>!j.unit||j.unit==='rad').slice(0,6),names=selected.map(j=>j.name).join('|');
    if((finite(time)&&finite(lastTime)&&time<lastTime)||lastNames&&names!==lastNames)reset();lastNames=names;
    const values=Array.from({length:6},(_,i)=>{const j=selected[i],actual=j?.actual??j?.position,error=finite(j?.target)&&finite(actual)?j.target-actual:NaN;return{torque:j?.torque,error};});
    const valid=values.length===6&&values.every(j=>finite(j.error)),rms=valid?Math.sqrt(values.reduce((sum,j)=>sum+j.error*j.error,0)/6):NaN;rmsRow.lastChild.textContent=`${fixed(rms*1000,1)} mrad`;
    rows.forEach((row,i)=>{const j=selected[i],value=j?.torque,limit=j?.torqueLimit??j?.effort;row.value.textContent=`${fixed(value,2)} N·m`;row.fill.style.width=finite(value)&&finite(limit)&&limit>0?`${Math.min(1,Math.abs(value)/limit)*50}%`:'0%';row.fill.classList.toggle('is-negative',value<0);row.row.title=finite(limit)?`J${i+1}：电机力矩 ${fixed(value,2)} N·m，限制 ±${fixed(limit,1)} N·m`:`J${i+1}：电机力矩 ${fixed(value,2)} N·m`;});
    if(!finite(time)||time!==lastTime){history.push({time,joints:values});if(history.length>maxSamples)history.shift();lastTime=time;}
    draw(canvases[0],'torque',1);draw(canvases[1],'error',.004);
    canvases[0].setAttribute('aria-label',`六关节实际电机力矩，牛顿米：${values.map((j,i)=>`J${i+1} ${fixed(j.torque,2)}`).join('，')}`);
    canvases[1].setAttribute('aria-label',`六关节目标减实际角度，弧度：${values.map((j,i)=>`J${i+1} ${fixed(j.error,4)}`).join('，')}`);
    return {rmsError:rms,samples:history.length};
  }
  reset();return{update,reset,element:section,dispose(){disposed=true;section.remove();history.length=0;}};
}

