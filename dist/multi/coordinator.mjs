const FIXED_STEP = .001;
const details = station => typeof station.snapshot === 'function' ? station.snapshot() : station.snapshot ?? {};
const isRunning = station => Boolean(station.controller?.running ?? station.running ?? details(station).running);
const ready = station => station.ready !== false;

/** A single logical clock for independent, local-coordinate physical stations. */
export class MultiRobotCoordinator {
  constructor({stations, selectedIds = ['r1', 'g1', 'c5']} = {}) {
    this.stations = stations instanceof Map ? new Map(stations)
      : new Map((Array.isArray(stations) ? stations : Object.values(stations ?? {})).map(s => [s.id, s]));
    this.tick = 0;
    this.selectedIds = selectedIds.filter(id => this.stations.has(id) && ready(this.stations.get(id)));
    this.focusId = this.selectedIds[0] ?? null;
    this.runIds = [];
    this.startTick = null;
    this.batchNumber = 0;
    this.state = 'idle';
    this.task = 'sort';
    this.message = this.selectedIds.length ? '请选择任务，所选机器人将同时开始' : '没有已就绪的机器人，请刷新重试';
    this.startTimes = {};
    this.applyVisibility();
  }

  get running() { return this.runIds.some(id => isRunning(this.stations.get(id))); }

  applyVisibility() {
    for (const [id, station] of this.stations) if (station.group) station.group.visible = this.selectedIds.includes(id);
  }

  select(ids) {
    if (this.running) return {ok:false, reason:'任务执行中，请先停止再切换机器人'};
    if (!Array.isArray(ids) || ids.length === 0) return {ok:false, reason:'请至少保留一个机器人'};
    const unique = [...new Set(ids)];
    if (unique.some(id => !this.stations.has(id) || !ready(this.stations.get(id))))
      return {ok:false, reason:'所选机器人尚未就绪，请等待加载或刷新重试'};
    this.selectedIds = unique;
    if (!unique.includes(this.focusId)) this.focusId = unique[0];
    this.applyVisibility();
    this.message = `已选择 ${unique.length} 台机器人；点击开始后同时抓取`;
    return {ok:true};
  }

  setFocus(id) {
    if (!this.selectedIds.includes(id)) return {ok:false, reason:'请先勾选该机器人'};
    this.focusId = id;
    return {ok:true};
  }

  configureTask(task, {seed} = {}) {
    if (this.running) return {ok:false, reason:'任务执行中，请先停止再切换工作台'};
    if (!['sort','stack','pipes'].includes(task)) return {ok:false, reason:'请选择有效任务'};
    const previous=this.task;
    try {
      for (const station of this.stations.values()) if (ready(station)) {
        const result=station.configureTask?.(task,{seed});
        if (result?.ok===false) throw new Error(`${station.config?.label??station.id}：${result.reason}`);
        if (task==='pipes'&&!station.configureTask) throw new Error('该机器人尚不支持圆管分类');
      }
    } catch(error) {
      // Rebuilding a task scene is idle-only. On a partial preparation error,
      // restore every world together rather than leaving mixed task modes.
      for (const station of this.stations.values()) if (ready(station)) {
        station.stop();station.configureTask?.(previous);station.reset();
      }
      this.clearRun();this.state='failed';this.message=`工作台准备失败：${error.message}；请重试`;
      return {ok:false,reason:this.message};
    }
    this.task=task;this.clearRun();this.state='idle';
    this.message=task==='pipes'?'圆管分类已就绪：近侧 4 黄 + 4 红，自动分类到远侧同色框':'工作台已切换，可开始抓取';
    this.applyVisibility();return {ok:true,task};
  }

  clearRun() {
    this.tick=0;this.runIds=[];this.startTick=null;this.startTimes={};
  }

  start({objectId = 'blue', task = 'sort', armByRobot = {}, reducedMotion = false} = {}) {
    if (this.running) return {ok:false, reason:'已有机器人正在执行，请先停止'};
    if (!['sort', 'stack', 'pipes'].includes(task) || (task!=='pipes'&&!['red', 'blue', 'yellow'].includes(objectId)))
      return {ok:false, reason:'请选择有效的任务和物体'};
    const ids = [...this.selectedIds];
    if (!ids.length || ids.some(id => !this.stations.has(id) || !ready(this.stations.get(id))))
      return {ok:false, reason:'请至少选择一台已就绪的机器人'};
    if (ids.some(id => isRunning(this.stations.get(id))))
      return {ok:false, reason:'所选机器人已有独立任务正在执行，请先停止'};
    if (task==='pipes'&&ids.some(id=>details(this.stations.get(id)).classification?.started))
      return {ok:false,reason:'本批圆管分类已开始过；请先复位或重新随机摆放，再开始新批次'};
    // No physics advances while planning. Successful preparations therefore
    // enter the same next tick, regardless of CPU time spent in each planner.
    const times = ids.map(id => Number(details(this.stations.get(id)).time ?? this.stations.get(id).data?.time ?? 0));
    if (times.some(time => !Number.isFinite(time) || Math.abs(time - times[0]) > 1e-8))
      return {ok:false, reason:'机器人时钟不同步，请复位全部后重试'};
    this.state = 'preparing';
    this.message = '正在检查所选机器人的抓取路径…';
    this.runIds = ids;
    const attempted = [];
    for (const id of ids) {
      const station = this.stations.get(id);
      attempted.push(id);
      let accepted = false, error;
      try { accepted = station.start({objectId, task, arm:armByRobot[id] ?? station.config?.defaultArm ?? 'right', reducedMotion}); }
      catch (caught) { error = caught; }
      if (!accepted || accepted.ok === false) {
        const rejection = details(station);
        const reason = error?.message ?? rejection.message ?? station.controller?.message ?? '目标不可达';
        // A rejected controller can itself enter a failure reaction. Stop every
        // attempted station, not only the previously accepted controllers.
        for (const attemptedId of attempted) this.stations.get(attemptedId).stop();
        this.state = 'failed';
        this.message = `${station.config?.label ?? id} 无法开始：${reason}；本次未启动，请调整物体后重试`;
        this.startTick = null;
        this.startTimes = {};
        return {ok:false, reason:this.message, runIds:[...ids]};
      }
    }
    this.startTick = this.tick;
    this.startTimes = Object.fromEntries(ids.map((id, index) => [id, times[index]]));
    this.batchNumber++;
    this.state = 'running';
    this.task=task;
    this.message = `${ids.length} 台机器人已同时开始${task === 'pipes' ? '圆管分类' : task === 'stack' ? '叠放' : '抓取'}`;
    return {ok:true, runIds:[...ids], startTick:this.startTick, startTimes:{...this.startTimes}};
  }

  step(dt = FIXED_STEP) {
    if (!Number.isFinite(dt) || Math.abs(dt - FIXED_STEP) > 1e-10) throw new Error('必须以统一的 1 ms 步长推进所有机器人');
    for (const station of this.stations.values()) if (ready(station)) station.step(dt);
    this.tick++;
    if (this.state === 'running' && !this.running) {
      const allComplete = this.runIds.every(id => details(this.stations.get(id)).state === 'complete');
      this.state = allComplete ? 'complete' : 'failed';
      this.message = allComplete ? `${this.runIds.length} 台机器人均已完成${this.task==='pipes'?'圆管分类':'抓取'}` : '部分机器人未完成；查看各机器人状态，调整物体后可重试';
    }
  }

  stop() {
    for (const id of this.runIds) this.stations.get(id)?.stop();
    this.state = 'stopped';
    this.message = this.task==='pipes'?'分类任务已停止；保留已归类进度，请复位或重新随机后开始新批次':'本次任务已全部停止；物理仍继续运行，可复位或重新开始';
    return {ok:true};
  }

  reset() {
    for (const station of this.stations.values()) if (ready(station)) { station.stop(); station.reset(); }
    this.tick = 0;
    this.runIds = [];
    this.startTick = null;
    this.startTimes = {};
    this.state = 'idle';
    this.message = this.task==='pipes'?'全部工作台已重新随机摆放，可再次分类':'全部工作台已复位，可重新开始';
    this.applyVisibility();
    return {ok:true};
  }

  get snapshot() {
    return {selectedIds:[...this.selectedIds], focusId:this.focusId, runIds:[...this.runIds], tick:this.tick,
      simTime:this.tick * FIXED_STEP, startTick:this.startTick, startTimes:{...this.startTimes}, batchNumber:this.batchNumber,
      running:this.running, state:this.state, task:this.task, message:this.message,
      stations:[...this.stations].map(([id, station]) => ({...details(station), id, ready:ready(station), selected:this.selectedIds.includes(id)}))};
  }
}
