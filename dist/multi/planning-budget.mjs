const budgetKey=Symbol.for('r1-simulator.planning-budget.v1');
export const PLANNING_BUDGET_MESSAGE='路径检查累计计算片段耗时已达时限，请将物体移近桌面中央后重试';

export class PlanningBudgetError extends Error{
  constructor(detail){super(PLANNING_BUDGET_MESSAGE);this.name='PlanningBudgetError';this.planningBudgetExceeded=true;this.detail={type:'budget',...detail};}
}
export const isPlanningBudgetError=error=>error instanceof PlanningBudgetError||error?.planningBudgetExceeded===true;
export function planningBudgetCheckpoint(robot){robot?.[budgetKey]?.();}

// A scope exists only while one cooperative planning computation is executing.
// Nested calls restore the previous callback; no hook survives return/throw.
export function withPlanningBudget(robot,checkpoint,compute,dynamics){
  const previous=Object.getOwnPropertyDescriptor(robot,budgetKey),poseDescriptor=dynamics?Object.getOwnPropertyDescriptor(dynamics,'checkPose'):undefined,checkPose=dynamics?.checkPose;
  const wrapped=typeof checkPose==='function'?function(...args){checkpoint();return checkPose.apply(this,args);}:null;
  let installed=false,poseAttempted=false;
  try{
    Object.defineProperty(robot,budgetKey,{configurable:true,value:checkpoint});installed=true;
    if(wrapped){poseAttempted=true;dynamics.checkPose=wrapped;}
    checkpoint();const result=compute();checkpoint();return result;
  }
  finally{
    try{if(poseAttempted){if(poseDescriptor)Object.defineProperty(dynamics,'checkPose',poseDescriptor);else delete dynamics.checkPose;}}
    finally{if(installed){if(previous)Object.defineProperty(robot,budgetKey,previous);else delete robot[budgetKey];}}
  }
}
