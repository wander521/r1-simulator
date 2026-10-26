// Acceleration-limited planar setpoint. No robot or payload state writes.
export function advancePlanarCommand(position,velocity,goal,dt,{maxSpeed=.12,maxAcceleration=.18}={}){
  if(![...position,...velocity,...goal,dt,maxSpeed,maxAcceleration].every(Number.isFinite)||dt<=0||maxSpeed<=0||maxAcceleration<=0)throw new Error('Invalid mobile motion profile');
  const delta=goal.map((v,i)=>v-position[i]),distance=Math.hypot(...delta);
  if(distance<1e-10)return {position:[...goal],velocity:[0,0]};
  // Reserve one integration step for braking to avoid a final target snap.
  const speed=Math.min(maxSpeed,Math.sqrt((maxAcceleration*dt)**2+2*maxAcceleration*distance)-maxAcceleration*dt);
  const desired=delta.map(v=>v/distance*speed),change=desired.map((v,i)=>v-velocity[i]),norm=Math.hypot(...change),fraction=Math.min(1,maxAcceleration*dt/Math.max(norm,1e-12));
  const nextVelocity=velocity.map((v,i)=>v+fraction*change[i]),step=nextVelocity.map(v=>v*dt);
  if(Math.hypot(...step)>=distance&&step.every((v,i)=>v*delta[i]>=0))return {position:[...goal],velocity:[0,0]};
  return {position:position.map((v,i)=>v+step[i]),velocity:nextVelocity};
}
