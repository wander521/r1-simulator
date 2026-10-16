import {applyPlanningResult} from './planning-serialization.mjs?v=20261016-worker-1';

/** Browser-only opt-in, outside the cooperative adapter used by Node/physics
 * tests. No IK candidate, swept segment or collision search runs in poll(). */
export function backgroundPlanningControllerClass(CooperativeController,{robotId,transport,captureJob,isCurrent=()=>true}){
  return class BackgroundPlanningController extends CooperativeController{
    get planning(){return Boolean(this._backgroundJob)||super.planning;}
    get planningProgress(){
      if(!this._backgroundJob)return null;
      const progress=transport().progress;
      return progress?{mode:'worker',phase:progress.stage==='loading'?'loading':'planning',...progress}:null;
    }
    prepareItem(arm,id,continuing){
      if(!this.backgroundPlanning)return super.prepareItem(arm,id,continuing);
      if(this.planning)return false;
      this.arm=robotId==='c5'?'left':arm;this.selected=id;
      this.planData=null;this.planError=null;this.planningResult=null;
      try{
        const job=captureJob({arm:this.arm,id,continuing});
        this._backgroundJob={before:{...this.pose},snapshot:job};
        transport().submit(job);
        this.running=true;this.state='planning';this.message='正在后台检查自然且连续的抓取路径…';return true;
      }catch(error){
        this._backgroundJob=null;this.running=false;this.state='failed';this.message=error.message;
        this.planningResult={ok:false,reason:error.message,error:{type:'worker'}};return false;
      }
    }
    advancePlanning(){
      if(!this._backgroundJob)return super.advancePlanning();
      const result=transport().poll();if(result.pending)return result;
      const current=isCurrent(this._backgroundJob.snapshot);
      this._backgroundJob=null;
      if(!current)return this._backgroundFailure({ok:false,reason:'机器人或物体状态已变化，旧路径已丢弃，请重新开始',error:{type:'stale'}});
      if(result.ok){
        try{applyPlanningResult(this,result.payload);}
        catch(error){return this._backgroundFailure({...result,ok:false,reason:error.message,error:{type:'message'}});}
        this.planningResult={ok:true,steps:result.steps,segmentElapsedMs:result.segmentElapsedMs};
        return{...result,result:this.planningResult};
      }
      return this._backgroundFailure(result);
    }
    _backgroundFailure(result){
      this.planData=null;this.planError=result.error?.detail??null;this.running=false;this.state='failed';this.reaction=null;
      this.message=result.reason||'后台路径检查失败，请复位后重试';this.planningResult={...result,reason:this.message};
      return{...result,pending:false,done:true,ok:false,result:this.planningResult};
    }
    cancelPlanning(reason='已停止路径检查'){
      if(!this._backgroundJob)return super.cancelPlanning(reason);
      this._backgroundJob=null;transport().cancel(reason);
      this.planData=null;this.running=false;this.state='stopped';this.reaction=null;this.message=reason;
      this.planningResult={ok:false,reason,error:{type:'cancelled'}};return true;
    }
  };
}
