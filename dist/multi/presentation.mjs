export const STAGE_NAMES = Object.freeze({idle:'就绪',spread:'双臂展开',raise:'外侧抬臂',ready:'胸前准备',align_clear:'收臂准备转腰',align:'转腰对准',recenter:'重新对准',crouch:'前倾下蹲',approach:'接近目标',descend:'下降对准',closing:'双指接触验证',lift:'抬升验证',transfer:'搬运',place:'放置支撑验证',release:'松开夹爪',retreat:'夹爪撤离',settling:'等待物体稳定',return:'回到待命',return_clear:'抬臂清障返回',return_align:'转腰返回',return_fold:'收回前臂',return_wide:'外侧收臂',return_center:'恢复待命',pipe_settle:'全部圆管稳定验证',reaction_clear:'抬臂清障',reaction_align:'转腰回正',reaction_fold:'收回前臂',reaction_wide:'外侧收臂',reaction_home:'返回安全姿态',stack_settle:'叠放稳定验证',face_observer:'转向观察者',happy:'叠放完成',shy:'任务失败',complete:'任务完成',failed:'抓取失败',stopped:'已停止'});
const handling=['approach','descend','closing','lift','transfer','place','release','retreat','settling'];
const preparation={r1:['spread','crouch','align_clear','align'],g1:['spread','raise','ready','recenter','align'],c5:[]};
const returning={r1:['return'],g1:['return_clear','return_align','return_fold','return_wide','return','return_center'],c5:['return']};
const reaction={stack_settle:.86,reaction_clear:.88,reaction_align:.89,reaction_fold:.90,reaction_wide:.91,reaction_home:.92,face_observer:.94,happy:.97};

// Milestone progress, not an estimate of remaining seconds. Each mechanism
// keeps its real preparation/return phases; G1's extra stages cannot reset it.
export function taskProgress({id,state,task,itemIndex=0,classification}) {
  if(state==='complete')return 1;
  if(task==='pipes'&&classification){
    if(state==='pipe_settle')return .98;
    const item=taskProgress({id,state,task:'sort'});
    return Math.min(.97,(classification.completed+(classification.completed<8?item:0))/8);
  }
  if(['idle','stopped','failed','shy'].includes(state))return 0;
  if(task==='stack'&&state in reaction)return reaction[state];
  const phases=[...(preparation[id]??[]),...handling,...(task==='stack'?[]:returning[id]??[])];
  const index=phases.indexOf(state);
  if(index<0)return 0;
  const item=(index+.5)/phases.length;
  return task==='stack'?Math.min(.85,.85*(itemIndex+item)/3):item;
}
