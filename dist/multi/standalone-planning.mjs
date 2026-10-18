import {planningControllerClass} from './planning.mjs?v=20261018-perf-4';
import {backgroundPlanningControllerClass} from './background-planning.mjs?v=20261018-perf-4';
import {createPlanningWorkerTransport} from './planning-worker-transport.mjs?v=20261018-perf-4';
import {capturePlanningJob,planningJobStillCurrent} from './planning-serialization.mjs?v=20261018-perf-4';

/** Preserve each original page's model, controller, manual tools and rendering.
 * Only automatic preparation moves to the shared isolated Worker boundary. */
export function createStandalonePlanningController(Controller,{robotId,api,model,data,robot,targets,dynamics,sourceXml,
  transportFactory=createPlanningWorkerTransport}={}){
  let channel=null,runtime;
  const transport=()=>channel??=transportFactory();
  const Cooperative=planningControllerClass(Controller,{robotId});
  const Background=backgroundPlanningControllerClass(Cooperative,{robotId,transport,
    captureJob:request=>capturePlanningJob(runtime,request),isCurrent:job=>planningJobStillCurrent(runtime,job)});
  const controller=new Background(api,model,data,robot,targets,dynamics);
  controller.cooperativePlanning=true;controller.backgroundPlanning=true;
  runtime={id:robotId,api,model,data,robot,targets,dynamics,controller,sourceXml,modelXml:sourceXml,modelPipeSeed:null,taskMode:'legacy'};
  controller.disposePlanning=()=>{controller.cancelPlanning('页面已关闭');channel?.dispose();channel=null;};
  return controller;
}
