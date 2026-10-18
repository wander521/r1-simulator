/** One in-flight readback/presentation owns its pixel buffer until it settles.
 * Invalidating presentation never reuses it early or cancels the renderer. */
export function createCameraFeedQueue({render,present,isCurrent=()=>true,onError=()=>{}}){
  let generation=0,pending=null;
  const report=(error,context)=>{try{onError(error,context);}catch(reporterError){try{console.error(reporterError);}catch{}}};
  return{
    get pending(){return pending!==null;},
    invalidate(){generation++;},
    request(context){
      if(pending)return false;
      let finish;
      const token={generation,done:new Promise(resolve=>{finish=resolve;})};pending=token;
      const release=()=>{if(pending===token)pending=null;finish();};
      const current=()=>{
        if(token.generation!==generation)return false;
        try{return Boolean(isCurrent(context))&&token.generation===generation;}
        catch(error){if(token.generation===generation)report(error,context);return false;}
      };
      let result;
      try{result=render(context);}catch(error){if(current())report(error,context);release();return false;}
      Promise.resolve(result).then(pixels=>{
        if(current())return present(pixels,context);
      }).catch(error=>{if(current())report(error,context);}).finally(release);
      return true;
    },
    whenIdle(){return pending?.done??Promise.resolve();},
  };
}
