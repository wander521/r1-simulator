// Official AUBO C5 six-axis chain with a simulated, non-factory Unitree Dex1
// flange adapter. Source URDF coordinates are Z-up; the viewer is Three Y-up.
export const ROOT_HEIGHT=.72;
export const TABLE_HEIGHT=.72;
export const TABLE_CENTER_X=.50;
export const TABLE_HALF_X=.28;
export const TABLE_HALF_Z=.35;
export const PLINTH_RADIUS=.12;
export const PLINTH_HEIGHT=ROOT_HEIGHT;
export const OBJECT_START_X=.38;
export const OBJECT_DROP_X=.55;
export const OBJECT_LATERAL_OFFSET=.10;
export const ARM_SUFFIXES=Object.freeze(['shoulder','upperArm','foreArm','wrist1','wrist2','wrist3']);
export const ARM_JOINT_NAMES=Object.freeze(ARM_SUFFIXES.map(name=>`${name}_joint`));
export const armJointNames=()=>[...ARM_JOINT_NAMES];
export const jointNames=armJointNames;
export const BASE_LINK='base_link';
export const FLANGE_LINK='wrist3_Link';
export const TOOL_ADAPTER_XYZ=Object.freeze([0,0,0]);
// Ry(-pi/2) maps Dex1's tool-forward +X onto the C5 flange's local +Z.
export const TOOL_ADAPTER_RPY=Object.freeze([0,-Math.PI/2,0]);
export const fingerJointNames=()=>[1,2].map(index=>`left_dex1_finger_joint_${index}`);
export const fingerBodyNames=()=>[1,2].map(index=>`left_dex1_finger_link_${index}`);
export const fingerActuatorNames=()=>['left_motor1','left_motor2'];
export const gripperLinkName=()=>'left_dex1_base_link';
export const GRIPPER_LINK='left_dex1_base_link';
// The browser URDFLoader only retains lower/upper, so keep audited source
// velocity metadata here as well. The physics test checks every entry against
// the official C5 / original Unitree URDF, not against invented defaults.
export const JOINT_VELOCITY_LIMITS=Object.freeze(Object.fromEntries([
  ...ARM_JOINT_NAMES.map((name,index)=>[name,index<3?2.565634:3.106686]),
  ...fingerJointNames().map(name=>[name,.2])
]));
export const GRIPPER_Q_MIN=-.020;
export const GRIPPER_Q_MAX=.0245;
// Minimum geometric opening; source inner faces tilt about 0.5 degrees.
export const GRIPPER_GAP_AT_ZERO=.045876835;
export const GRIPPER_TCP_X=.12;
export const FLANGE_TCP_LOCAL=Object.freeze([0,0,GRIPPER_TCP_X]);
const clampQ=q=>Math.max(GRIPPER_Q_MIN,Math.min(GRIPPER_Q_MAX,q));
export const openingToQ=gap=>clampQ((gap-GRIPPER_GAP_AT_ZERO)/2);
export const qToOpening=q=>GRIPPER_GAP_AT_ZERO+2*clampQ(q);
export const GRIPPER_OPENING_MIN=qToOpening(GRIPPER_Q_MIN);
export const GRIPPER_OPENING_MAX=qToOpening(GRIPPER_Q_MAX);
export const STAND_POSE=Object.freeze({});
export const HOME_POSE=Object.freeze(Object.fromEntries(ARM_JOINT_NAMES.map(name=>[name,0])));
