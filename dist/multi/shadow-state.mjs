/** Skip identical shadow passes; compare against the last invalidation so
 * sub-threshold motion accumulates instead of drifting forever unnoticed. */
export function createShadowStateTracker({epsilon=1e-6}={}){
  if(!Number.isFinite(epsilon)||epsilon<0)throw new TypeError('Invalid shadow tolerance');
  const states=new Map();
  return {changed(key,values){
    if(!values||!Number.isInteger(values.length))throw new TypeError('Invalid shadow state');
    const previous=states.get(key);
    let changed=!previous||previous.length!==values.length;
    for(let i=0;i<values.length;i++){
      if(!Number.isFinite(values[i]))throw new TypeError('Non-finite shadow state');
      if(previous&&Math.abs(values[i]-previous[i])>epsilon)changed=true;
    }
    if(changed)states.set(key,Float64Array.from(values));
    return changed;
  },clear(){states.clear();}};
}
