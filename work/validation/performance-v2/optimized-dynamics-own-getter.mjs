// Multi-station adapter only. The retained single-robot motor implementations
// remain the authority for formulas, iteration order and the 1 ms command step.
const installations=new WeakMap(),sources=new WeakMap();

function mayCache(view){
  // Growing ordinary Wasm memory detaches its previous ArrayBuffer. Shared
  // memory and resizable buffers need not detach, so without an exposed memory
  // epoch retain the original getters rather than risk an outdated view.
  return ArrayBuffer.isView(view)&&view.buffer instanceof ArrayBuffer&&!view.buffer.resizable;
}

/**
 * Cache expensive embind array-view getters only during one beforeStep call.
 * No view is retained between steps, worlds, model rebuilds or reset calls.
 * Other MjData property getters use the real native object as their receiver.
 */
export function optimizeDynamics(dynamics){
  if(!dynamics||typeof dynamics.beforeStep!=='function')throw new TypeError('A dynamics instance with beforeStep is required');
  const installed=installations.get(dynamics);
  if(installed?.wrapper===dynamics.beforeStep)return dynamics;
  const original=dynamics.beforeStep;
  const wrapper=function(...args){
    const previous=this.data,source=sources.get(previous)??previous;
    if(!source)return original.apply(this,args);
    let ctrl,bias;
    const fallback=new Proxy(source,{
      get:(target,key)=>Reflect.get(target,key,target),
      set:(target,key,value)=>Reflect.set(target,key,value,target),
    });
    // The hot getters are own accessors, not Proxy traps. Unknown properties
    // fall through the prototype proxy to preserve native getter receivers.
    const facade={
      get ctrl(){
        if(ctrl&&ctrl.buffer.byteLength!==0)return ctrl;
        const current=Reflect.get(source,'ctrl',source);if(mayCache(current))ctrl=current;return current;
      },
      set ctrl(value){ctrl=undefined;if(!Reflect.set(source,'ctrl',value,source))throw new TypeError('Cannot assign read-only MjData.ctrl');},
      get qfrc_bias(){
        if(bias&&bias.buffer.byteLength!==0)return bias;
        const current=Reflect.get(source,'qfrc_bias',source);if(mayCache(current))bias=current;return current;
      },
      set qfrc_bias(value){bias=undefined;if(!Reflect.set(source,'qfrc_bias',value,source))throw new TypeError('Cannot assign read-only MjData.qfrc_bias');},
    };
    Object.setPrototypeOf(facade,fallback);
    sources.set(facade,source);this.data=facade;
    try{return original.apply(this,args);}
    finally{
      // Preserve a deliberate data replacement by the original implementation.
      // Normal and nested invocations restore their original native receiver.
      if(this.data===facade)this.data=previous;
      sources.delete(facade);
    }
  };
  dynamics.beforeStep=wrapper;installations.set(dynamics,{original,wrapper});
  return dynamics;
}

/** Restore only this adapter; never remove a subsequently installed wrapper. */
export function restoreDynamics(dynamics){
  const installed=installations.get(dynamics);
  if(!installed||dynamics.beforeStep!==installed.wrapper)return false;
  dynamics.beforeStep=installed.original;installations.delete(dynamics);return true;
}
