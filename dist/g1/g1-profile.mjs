// G1 29-DoF + two Dex1 parallel grippers, metres/radians throughout.
// Zero-angle ankle_roll origin is -0.7568637524222113 m in pelvis frame.
// Source URDF foot spheres end another .035 m below that origin.
export const ROOT_HEIGHT=.7918637524222113;
export const G1_ROOT_HEIGHT=ROOT_HEIGHT;
export const TABLE_HEIGHT=.90;
export const TABLE_CENTER_X=.50;
export const TABLE_HALF_X=.28;
export const TABLE_HALF_Z=.35;
export const OBJECT_START_X=.33;
export const OBJECT_DROP_X=.44;
export const OBJECT_LATERAL_OFFSET=.09;
export const ARM_SUFFIXES=Object.freeze(['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']);
export const armJointNames=side=>ARM_SUFFIXES.map(suffix=>`${side}_${suffix}_joint`);
export const fingerJointNames=side=>[1,2].map(index=>`${side}_dex1_finger_joint_${index}`);
export const fingerBodyNames=side=>[1,2].map(index=>`${side}_dex1_finger_link_${index}`);
export const fingerActuatorNames=side=>[1,2].map(index=>`${side}_motor${index}`);
export const gripperLinkName=side=>`${side}_dex1_base_link`;
export const GRIPPER_LINK=Object.freeze({left:gripperLinkName('left'),right:gripperLinkName('right')});
export const GRIPPER_Q_MIN=-.020;
export const GRIPPER_Q_MAX=.0245;
// Minimum mesh opening: the real inner faces tilt about .5 degrees, so the
// gap at TCP x=.12 is about .4 mm larger. Physics uses the source STL hulls.
export const GRIPPER_GAP_AT_ZERO=.045876835;
export const GRIPPER_TCP_X=.12;
export const GRIPPER_WRIST_TCP_X=.1615;
const clampQ=q=>Math.max(GRIPPER_Q_MIN,Math.min(GRIPPER_Q_MAX,q));
export const openingToQ=gap=>clampQ((gap-GRIPPER_GAP_AT_ZERO)/2);
export const qToOpening=q=>GRIPPER_GAP_AT_ZERO+2*clampQ(q);
export const LEG_JOINT_NAMES=Object.freeze(['left','right'].flatMap(side=>['hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll'].map(suffix=>`${side}_${suffix}_joint`)));
export const WAIST_JOINT_NAMES=Object.freeze(['waist_yaw_joint','waist_roll_joint','waist_pitch_joint']);
// Legs and waist remain upright: no navigation or squat trajectory is used.
// Arm rest/working poses are separate, solved for the seven-axis chain.
export const STAND_POSE=Object.freeze(Object.fromEntries([...LEG_JOINT_NAMES,...WAIST_JOINT_NAMES].map(name=>[name,0])));
