import * as THREE from '../vendor/three.module.min.js';
import * as G1 from '../g1/g1-profile.mjs';
import * as C5 from '../c5/c5-profile.mjs';
import {READY} from '../g1/grasp-controller.mjs';
import {solvePose as solveG1,naturalArmLimits,naturalArmPose} from '../g1/grasp-kinematics.mjs';
import {solvePose as solveC5,topDownQuaternion as c5TopDown} from '../c5/grasp-kinematics.mjs';
import {solveCarrierPose} from './carrier-kinematics.mjs';

// Native planners, with genuine task-height endpoints and actual part widths.
// Original IK error, branch-continuity and collision thresholds are retained.
const sidePinch=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2);
const g1Scope={...G1,naturalArmPose,naturalArmLimits,READY,OPEN:G1.qToOpening(G1.GRIPPER_Q_MAX)};
const names=C5.armJointNames(),fromArray=a=>Object.fromEntries(names.map((name,i)=>[name,a[i]]));
const planningLimits=[[-Math.PI,Math.PI],[-2.9,2.9],[-2.9,2.9],[-Math.PI,Math.PI],[-2.9,2.9],[-Math.PI,Math.PI]];
function taskWidth(target,objectRotation,axis){
  const axes=[new THREE.Vector3(1,0,0),new THREE.Vector3(0,1,0),new THREE.Vector3(0,0,1)].map(value=>Math.abs(value.applyQuaternion(objectRotation).dot(axis)));
  const half=[target.halfX??(target.id==='blue'?.026:.0225),target.h??(target.id==='blue'?.019:.0225),target.halfZ??(target.id==='blue'?.016:.0225)];
  return target.shape==='cylinder'||target.id==='yellow'?2*(half[0]*Math.sqrt(Math.max(0,1-axes[1]**2))+half[1]*axes[1]):2*half.reduce((sum,value,index)=>sum+value*axes[index],0);
}

export function planG1Task(target,yawOffset=0,seedIndex=0,lateral=.14){
  const {armJointNames,STAND_POSE,naturalArmPose,naturalArmLimits,READY,OPEN}=g1Scope;
    // A carried container needs the elbow outside the torso while the wrist
    // stays in front of the shelf. Use wider shoulder-relative lanes, not
    // larger joint limits or relaxed IK tolerances.
    if(target.carrier)lateral+=.09;
    const {start,drop}=this.placementEndpoints(target);
    if(this.task==='stack'){
      const base=this.targets.find(t=>t.id==='blue').drop;drop.x=base.x;drop.z=base.z;
      if(this.itemIndex>0){const id=['blue','red'][this.itemIndex-1],support=this.objectPose(id).position,t=this.targets.find(t=>t.id===id);drop.x=support.x;drop.z=support.z;drop.y=support.y+t.h+target.h;}
    }
    const names=armJointNames(this.arm),sign=this.arm==='left'?1:-1;
    if(start.x>.65||Math.hypot(start.x,start.z)>.66||drop.x>.65)throw new Error('目标超出站立抓取范围，请将物体移近桌面中央');
    const align=point=>THREE.MathUtils.clamp(Math.atan2(-point.z,point.x)-Math.asin(THREE.MathUtils.clamp(sign*lateral/Math.hypot(point.x,point.z),-.9,.9)),-1.0,1.0);
    const startYaw=align(start),dropYaw=align(drop),tilt=(target.carrier?(target.graspTilt??-10):20)*Math.PI/180;
    const quaternion=(yaw,down=0)=>new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),yaw+yawOffset).multiply(sidePinch).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),down));
    const orientation=quaternion(startYaw,tilt),dropOrientation=quaternion(dropYaw,tilt),stand={...STAND_POSE,waist_yaw_joint:startYaw};
    let seed={...stand,...READY.right,...READY.left};
    const seeds=[
      naturalArmPose(this.arm),[.15,.5*sign,.2*sign,-.5,0,.15,0],[-.6,.3*sign,0,.2,0,.1,0],
      [-.27,.15*sign,.03*sign,-.85,-.24*sign,1.10,.24*sign],[-.3,.7*sign,-.3*sign,-.4,.4*sign,.3,0],
      [-.7,.45*sign,.4*sign,-.8,-.3*sign,.7,0]
    ];
    Object.assign(seed,Object.fromEntries(names.map((n,i)=>[n,seeds[seedIndex][i]])));
    const options={limits:naturalArmLimits(this.arm)};
    const solve=(p,continuous=false,down=0)=>{
      const yaw=align(p);
      if(!target.carrier)seed.waist_yaw_joint=yaw;
      const solveOptions=target.carrier?{...options,preferred:names.map(name=>seed[name])}:options;
      let result=target.carrier?solveCarrierPose(this.robot,this.arm,p,quaternion(yaw,down),seed,continuous?180:320):solveG1(this.robot,this.arm,p,quaternion(yaw,down),seed,continuous?140:240,solveOptions);
      if(target.carrier&&result.positionError>.003&&result.positionError<.012&&result.angleError<.1){
        // Refine a nearby solution without the null-space posture bias. Keep
        // the same success thresholds and the subsequent branch-jump check.
        const refined=solveCarrierPose(this.robot,this.arm,p,quaternion(yaw,down),result.pose,360);
        if(refined.positionError+.18*refined.angleError<result.positionError+.18*result.angleError)result=refined;
      }
      // A valid isolated IK solution is not necessarily a valid trajectory.
      // Reject branch jumps instead of interpolating a wrist through a table
      // or throwing an object when the redundant elbow flips configuration.
      if(result.positionError>.003||result.angleError>.035){const error=new Error('目标超出当前站姿可稳定抓取的范围，请向桌面中央移动');error.detail={position:p.toArray(),positionError:result.positionError,angleError:result.angleError,...(target.carrier?{yaw,down,joints:Object.fromEntries(names.map((n,i)=>[n,{value:result.pose[n],limits:options.limits[i]}]))}:{})};throw error;}
      const jump=Math.max(...(target.carrier?['waist_yaw_joint',...names]:names).map(n=>Math.abs(result.pose[n]-(seed[n]??0))));
      if(continuous&&jump>.30){const error=new Error('当前位置的连续抓取路径不可达，请将物体稍微移近');error.detail={position:p.toArray(),jointJump:jump};throw error;}
      seed={...stand,...READY.right,...READY.left,...result.pose};return {...seed};
    };
    // The outer buffer needs vertical clearance, not an additional extension
    // away from the shoulder. Keep the original inner-slot approach unchanged.
    const above=start.clone().add(new THREE.Vector3(target.carrier?-.12:.04,target.carrier?.02:.11,0)),aboveDrop=drop.clone().add(new THREE.Vector3(target.carrier&&drop.x>.4?0:.04,target.carrier?.08:.11,0));
    const approach=solve(above,false,target.carrier?tilt:0),line=(from,to,ta=0,tb=ta)=>{
      const samples=[];
      const append=(u,previous,depth=0)=>{
        try{samples.push(solve(from.clone().lerp(to,u),true,THREE.MathUtils.lerp(ta,tb,u)));}
        catch(error){
          if(!target.carrier||!error.detail?.jointJump||u===previous||depth>=5)throw error;
          // Resolve a curved redundant-IK segment at a finer Cartesian step;
          // never interpolate across a rejected joint jump or relax its bound.
          const middle=(previous+u)/2;append(middle,previous,depth+1);append(u,middle,depth+1);
        }
      };
      for(let i=0;i<=40;i++)append(i/40,Math.max(0,(i-1)/40));return samples;
    };
    const closingAxis=new THREE.Vector3(0,1,0).applyQuaternion(orientation),objectQ=this.objectPose(target.id).quaternion;
    const axes=[new THREE.Vector3(1,0,0),new THREE.Vector3(0,1,0),new THREE.Vector3(0,0,1)].map(a=>Math.abs(a.applyQuaternion(objectQ).dot(closingAxis)));
    const width=taskWidth(target,objectQ,closingAxis);
    const descend=line(above,start,target.carrier?tilt:0,tilt);
    let lift,transfer;
    if(target.carrier){
      // A wall shelf is not an open tabletop. Lift only enough to clear its
      // lip, withdraw the complete bin in front of the rack, then gain height
      // and traverse sideways. Keep loose contents upright throughout.
      const lifted=start.clone().add(new THREE.Vector3(0,start.y>1?.06:.045,0)),withdrawn=lifted.clone().add(new THREE.Vector3(-.135,0,0));
      const clear=withdrawn.clone();clear.y=Math.max(withdrawn.y,aboveDrop.y);
      const lateralClear=clear.clone();lateralClear.z=aboveDrop.z;
      lift=[...line(start,lifted,tilt),...line(lifted,withdrawn,tilt).slice(1),...line(withdrawn,clear,tilt).slice(1)];
      transfer=[...line(clear,lateralClear,tilt),...line(lateralClear,aboveDrop,tilt).slice(1)];
    }else{lift=line(start,above,tilt,0);transfer=line(above,aboveDrop);}
    const plan={start,drop,startYaw,dropYaw,stand,objectRotation:objectQ.clone(),orientation:dropOrientation,graspOrientation:orientation,preOpening:target.carrier?OPEN:Math.min(OPEN,width+.018),approach,descend,lift,transfer,place:line(aboveDrop,drop,target.carrier?tilt:0,tilt),retreat:line(drop,aboveDrop,tilt,0)};
    if(target.carrier){plan.startYaw=approach.waist_yaw_joint;plan.dropYaw=plan.place.at(-1).waist_yaw_joint;plan.stand={...stand,waist_yaw_joint:plan.startYaw};plan.searchHint={yawOffset,seedIndex,lateral:lateral-.09};}
    const ignore=[this.bodies.get(this.selected)];
    for(const phase of ['descend','lift','transfer','place','retreat']){const path=plan[phase];for(let i=0;i<path.length;i++){
      const objectPoseFor=phase==='descend'?null:pose=>this.predictedObject(pose,plan,phase);
      const precontact=target.carrier&&phase==='descend';
      const check=this.dynamics.checkPose(path[i],{opening:{[this.arm]:plan.preOpening,[this.arm==='right'?'left':'right']:OPEN},ignoreObjectBodies:precontact?[]:ignore,objectPoses:objectPoseFor?.(path[i])});
      if(!check.safe){const error=new Error('抓取路径会与身体、桌面或相邻物体相撞，请调整物体位置');error.detail={phase,sample:i,contacts:check.contacts.slice(0,3),limits:check.limitViolations};throw error;}
      if(i)this.validateSegment(path[i-1],path[i],{opening:{[this.arm]:plan.preOpening,[this.arm==='right'?'left':'right']:OPEN},ignoreSelected:!precontact,objectPoseFor});
    }}
    this.planData=plan;
  }
export function planC5Task(target,seedIndex=0,yawOffset=0,tilt=20*Math.PI/180){
  const OPEN=C5.qToOpening(C5.GRIPPER_Q_MAX);
    const binPart=Number.isFinite(target.binGraspYaw);
    // The loose cylinders protrude above the carrier rim. Grasp their exposed
    // upper section; the Dex1 fingertips extend below its nominal TCP.
    const tcpOffset=new THREE.Vector3(0,binPart?.040:0,0);
    const {start,drop}=this.placementEndpoints(target);
    if(start.x<.25||start.x>.68||Math.hypot(start.x,start.z)>.72)throw new Error('目标超出当前作业范围，请移回桌面中央');
    if(this.task==='stack'){
      const base=this.targets.find(t=>t.id==='blue').drop;drop.x=base.x;drop.z=base.z;
      if(this.itemIndex>0){const id=['blue','red'][this.itemIndex-1],p=this.objectPose(id).position,t=this.targets.find(t=>t.id===id);drop.set(p.x,p.y+t.h+target.h,p.z);}
    }
    const aim=p=>Math.atan2(-p.z,p.x)+yawOffset;
    const quaternion=(p,down=0)=>binPart?c5TopDown(target.binGraspYaw+yawOffset):new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),aim(p)).multiply(sidePinch).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),down));
    const seeds=[
      [0,-.7,1.4,-.7,1.5,0],[0,.7,-1.4,.7,-1.5,0],
      [Math.PI,-.7,1.4,-.7,-1.5,0],[-Math.PI,.7,-1.4,.7,1.5,0],
      [1.5,-1.2,1.8,-1,1.5,0],[-1.5,1.2,-1.8,1,-1.5,0],
      [0,-1.5,1.5,0,1.5,0],[0,1.5,-1.5,0,-1.5,0],Object.values(this.pose)
    ];
    let seed=fromArray(seeds[seedIndex]);
    const solve=(point,continuous=false,down=0)=>{
      const result=solveC5(this.robot,'left',point.clone().add(tcpOffset),quaternion(point,down),seed,continuous?160:350,{limits:planningLimits});
      if(result.positionError>.0025||result.angleError>.025){const e=new Error('目标姿态不可稳定到达，请移动或转动物体');e.detail={point:point.toArray(),positionError:result.positionError,angleError:result.angleError};throw e;}
      const jump=Math.max(...names.map(n=>Math.abs(result.pose[n]-(seed[n]??0))));
      if(continuous&&jump>.28){const e=new Error('连续抓取路径跨越关节分支，请调整目标位置');e.detail={jointJump:jump};throw e;}
      seed={...result.pose};return {...seed};
    };
    const above=start.clone().add(new THREE.Vector3(binPart?0:.04,.14,0)),aboveDrop=drop.clone().add(new THREE.Vector3(binPart?0:.04,.14,0));
    const approach=solve(above),line=(a,b,ta=0,tb=ta)=>Array.from({length:41},(_,i)=>solve(a.clone().lerp(b,i/40),true,THREE.MathUtils.lerp(ta,tb,i/40)));
    const orientation=quaternion(drop,tilt),graspOrientation=quaternion(start,tilt),objectRotation=this.objectPose(target.id).quaternion;
    const axis=new THREE.Vector3(0,1,0).applyQuaternion(graspOrientation);
    const axes=[new THREE.Vector3(1,0,0),new THREE.Vector3(0,1,0),new THREE.Vector3(0,0,1)].map(v=>Math.abs(v.applyQuaternion(objectRotation).dot(axis)));
    const width=taskWidth(target,objectRotation,axis);
    const plan={start,drop,orientation,graspOrientation,objectRotation:objectRotation.clone(),preOpening:Math.min(OPEN,width+.018),approach,descend:line(above,start,0,tilt),lift:line(start,above,tilt,0),transfer:line(above,aboveDrop),place:line(aboveDrop,drop,0,tilt),retreat:line(drop,aboveDrop,tilt,0)};
    if(binPart){
      plan.tcpOffset=tcpOffset;
      plan.minimumCarryHeight=Math.min(start.y,drop.y)-(target.h??.023)-.03;
      plan.graspTransform={position:tcpOffset.clone().negate().applyQuaternion(graspOrientation.clone().invert()),quaternion:graspOrientation.clone().invert().multiply(objectRotation)};
      plan.releaseObjectQuaternion=objectRotation.clone();
      plan.readyPose=solve(new THREE.Vector3(.38,1.05,0));
    }
    this.validateSegment(this.pose,approach,{opening:OPEN});
    for(const phase of ['descend','lift','transfer','place','retreat']){
      const path=plan[phase],objectPoseFor=phase==='descend'?null:pose=>this.predictedObject(pose,plan,phase);
      for(let i=0;i<path.length;i++)this.validateSegment(i?path[i-1]:path[i],path[i],{opening:plan.preOpening,ignoreSelected:!(binPart&&phase==='descend'),objectPoseFor});
    }
    this.planData=plan;
  }
