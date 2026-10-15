import assert from 'node:assert/strict';
import { readFile, mkdir, writeFile } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import { GraspController } from '../dist/c5/grasp-controller.mjs';
import { C5Dynamics } from '../dist/c5/c5-dynamics.mjs';
import { tcpPose } from '../dist/c5/grasp-kinematics.mjs';
import { TABLE_HEIGHT, OBJECT_START_X, OBJECT_DROP_X, OBJECT_LATERAL_OFFSET,
  fingerJointNames, qToOpening, GRIPPER_Q_MIN, GRIPPER_Q_MAX } from '../dist/c5/c5-profile.mjs';
import { loadC5TestRobot } from './c5-test-robot.mjs';

const root = new URL('../', import.meta.url), OPEN = qToOpening(GRIPPER_Q_MAX), CLOSED = qToOpening(GRIPPER_Q_MIN);
const hashPaths = ['work/test-c5-opening-safety.mjs', 'work/build-c5-dynamics.mjs', 'work/c5-test-robot.mjs',
  'dist/c5/grasp-controller.mjs', 'dist/c5/grasp-kinematics.mjs', 'dist/c5/c5-dynamics.mjs', 'dist/c5/c5-profile.mjs',
  'dist/c5/models/c5-dynamic-scene.xml', 'dist/c5/assets/aubo_C5.urdf', 'dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-dynamics.mjs', 'dist/g1/g1-profile.mjs', 'dist/g1/assets/g1_29dof_with_dex1_1.urdf',
  ...Array.from({ length: 7 }, (_, index) => `dist/c5/assets/meshes/aubo_C5/collision/link${index}.STL`),
  ...[1, 2].map(index => `dist/g1/assets/dex1_1/meshes/dex1_col_${index}.stl`)];
const hashes = async () => Object.fromEntries(await Promise.all(hashPaths.map(async path => [path,
  createHash('sha256').update(await readFile(new URL(path, root))).digest('hex')])));
const sourceHashes = await hashes();
const targets = [
  { id: 'red', body: 'red_cube', h: .0225, z: OBJECT_LATERAL_OFFSET },
  { id: 'blue', body: 'blue_block', h: .019, z: 0 },
  { id: 'yellow', body: 'yellow_cylinder', h: .026, z: -OBJECT_LATERAL_OFFSET },
].map(target => ({ ...target, start: new THREE.Vector3(OBJECT_START_X, TABLE_HEIGHT + target.h, target.z),
  drop: new THREE.Vector3(OBJECT_DROP_X, TABLE_HEIGHT + target.h, target.z), rotation: new THREE.Quaternion() }));
const api = await createMuJoCo({ locateFile: () => new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', root).pathname });
const vfs = new api.MjVFS(), model = api.MjModel.from_xml_string(await readFile(new URL('dist/c5/models/c5-dynamic-scene.xml', root), 'utf8'), vfs);
const data = new api.MjData(model), robot = await loadC5TestRobot(); vfs.delete();
const dynamics = new C5Dynamics(api, model, data, robot), c = new GraspController(api, model, data, robot, targets, dynamics), results = [];
const actual = () => ({ qpos: [...data.qpos], qvel: [...data.qvel], ctrl: [...data.ctrl],
  xpos: [...data.xpos], xquat: [...data.xquat], time: data.time });
const command = () => ({ pose: { ...c.pose }, openings: { ...c.openings }, targets: { ...dynamics.targets },
  targetVelocity: { ...dynamics.targetVelocity }, previous: { ...dynamics.previous },
  fingerDesired: { ...dynamics.fingerDesiredTargets }, fingerRamped: { ...dynamics.fingerRampedTargets } });
const snapshot = () => ({ actual: actual(), command: command() });
function unchanged(before, label) {
  assert.deepEqual(actual(), before.actual, `${label}: do not rewrite actual solver state`);
  assert.deepEqual(command(), before.command, `${label}: rejected operation must preserve all commands`);
}
function record(name, detail = {}) { results.push({ name, pass: true, ...detail }); }
function query(pose, opening, ignore = [c.bodies.get('blue')]) {
  return dynamics.checkPose(pose, { opening, ignoreObjectBodies: ignore });
}
function findRealTableFixture() {
  // A native wrist-3 turn rolls the attached Dex1 about its tool-forward axis.
  // Probe actual source convex meshes near a production descent trajectory;
  // do not mock checkPose or manufacture safe=true values.
  c.reset(); assert(c.start('left', 'blue'), `${c.message}: ${JSON.stringify(c.planError)}`);
  const plan = c.planData;
  for (const index of [40, 36, 32, 28, 24, 20]) for (const roll of [-Math.PI / 2, Math.PI / 2, -1.1, 1.1, -.7, .7]) {
    const pose = { ...plan.descend[index], wrist3_joint: plan.descend[index].wrist3_joint + roll };
    const closed = query(pose, CLOSED), opened = query(pose, OPEN);
    const contacts = opened.contacts.filter(contact => contact.dist < -.003
      && [contact.body1, contact.body2].includes('task_table')
      && [contact.body1, contact.body2].some(name => name.includes('_dex1_finger_link_')));
    if (closed.safe && !opened.safe && contacts.length) return { pose, index, roll,
      closedSafe: true, openSafe: false, penetration: Math.max(...contacts.map(contact => -contact.dist)), contacts };
  }
  throw new Error('No genuine closed-safe/open-table-colliding source-mesh fixture was found');
}
let fixture;
function tableFixture() {
  c.reset(); c.opening = CLOSED; c.pose = { ...fixture.pose };
  // Explicit fixture initialization only. Production test operations never
  // clamp qpos or replace physics with a commanded visual pose.
  dynamics.reset({ ...fixture.pose, ...Object.fromEntries(fingerJointNames().map(name => [name, GRIPPER_Q_MIN])) });
  assert(query(c.pose, c.opening).safe);
}
try {
  fixture = findRealTableFixture();
  // findRealTableFixture begins with an explicit reset; no physics step or
  // object manipulation occurs in any source-mesh check after that reset.
  const searchStable = actual(); query(fixture.pose, OPEN); query(fixture.pose, CLOSED);
  assert.deepEqual(actual(), searchStable, 'Collision-query search must leave real state unchanged');
  record('source-mesh-opening-counterexample', fixture);

  tableFixture(); let before = snapshot();
  assert.equal(c.setManualOpening(OPEN), false); unchanged(before, 'manual opening rejection');
  assert.match(c.message, /拒绝|干涉|碰撞|安全/);
  record('manual-opening-into-table-rejected-without-state-write');

  tableFixture(); before = snapshot();
  assert.equal(c.start('left', 'blue'), false); assert.equal(c.state, 'failed'); assert.equal(c.running, false);
  unchanged(before, 'failed automatic expansion');
  record('automatic-preparation-rejects-colliding-initial-expansion');

  tableFixture(); c.running = true; c.state = 'transfer'; before = snapshot(); c.stop();
  assert.equal(c.running, false); assert.equal(c.state, 'stopped'); unchanged(before, 'stop expansion');
  record('stop-keeps-safe-gap-when-opening-would-hit-table');

  tableFixture(); c.running = true; c.state = 'approach'; c.task = 'putaway'; before = snapshot(); c.fail('opening-safety failure fixture');
  assert.equal(c.running, false); assert.equal(c.state, 'failed'); assert.match(c.message, /opening-safety failure fixture/);
  unchanged(before, 'failure expansion'); record('failure-keeps-safe-gap-when-opening-would-hit-table');

  tableFixture(); c.running = true; c.reaction = 'happy'; c.reactionSteps = [{ opening: OPEN, hold: .4, timeout: 1 }];
  c.reactionIndex = 0; before = snapshot(); c.transition('happy');
  assert.equal(c.running, false); assert.equal(c.state, 'failed'); unchanged(before, 'reaction expansion');
  record('reaction-opening-is-collision-checked-before-motion');

  c.reset(); before = snapshot(); assert.equal(c.setManualOpening(NaN), false); unchanged(before, 'invalid gap');
  record('nonfinite-manual-opening-rejected');

  let maxFingerViolation = 0, maxFingerSpeed = 0;
  const realStep = () => {
    c.step();
    for (const joint of dynamics.telemetry().allJoints) {
      assert(Number.isFinite(joint.actual) && Number.isFinite(joint.velocity) && Number.isFinite(joint.torque));
      assert(Math.abs(joint.torque) <= joint.effortLimit + 1e-7);
      if (joint.unit === 'm') {
        const violation = Math.max(0, joint.lower - joint.actual, joint.actual - joint.upper);
        maxFingerViolation = Math.max(maxFingerViolation, violation); maxFingerSpeed = Math.max(maxFingerSpeed, Math.abs(joint.velocity));
        assert(violation <= .0002, `Physical finger stroke violation ${violation} m`);
        assert(Math.abs(joint.velocity) <= joint.velocityLimit * 1.01 + .0001, 'Physical finger speed exceeds audited limit');
      }
    }
  };
  c.reset(); let beforeActual = actual(); assert(c.setManualOpening(CLOSED));
  assert.deepEqual(actual(), beforeActual, 'Accepted target must not teleport physical fingers');
  for (let index = 0; index < 1500; index++) realStep();
  assert(c.fingers.left.every(finger => Math.abs(data.qpos[finger.q] - GRIPPER_Q_MIN) < .001));
  beforeActual = actual(); assert(c.setManualOpening(OPEN)); assert.deepEqual(actual(), beforeActual);
  for (let index = 0; index < 1500; index++) realStep();
  assert(c.fingers.left.every(finger => Math.abs(data.qpos[finger.q] - GRIPPER_Q_MAX) < .001));
  record('normal-open-close-is-accepted-and-physically-motor-driven', { steps: 3000, maxFingerViolation, maxFingerSpeed });

  c.reset(); const tcp = tcpPose(robot).position; assert(c.setObject('blue', tcp));
  const selectedBlocked = query(c.pose, CLOSED, []), selectedAllowed = query(c.pose, CLOSED);
  assert(!selectedBlocked.safe && selectedBlocked.contacts.some(contact => [contact.body1, contact.body2].some(name => name.includes('_dex1_finger_link_'))));
  assert(selectedAllowed.safe); beforeActual = actual(); assert(c.setManualOpening(CLOSED));
  assert.deepEqual(actual(), beforeActual, 'Allowed selected contact must still only set a target');
  record('selected-fingertip-contact-is-real-and-allowed', { detectedWithoutException: !selectedBlocked.safe, safeWithException: selectedAllowed.safe });

  c.reset(); assert(c.setObject('red', tcpPose(robot).position)); before = snapshot();
  const neighbor = query(c.pose, CLOSED);
  assert(!neighbor.safe && neighbor.contacts.some(contact => [contact.body1, contact.body2].includes('red_cube')));
  assert.equal(c.setManualOpening(CLOSED), false); unchanged(before, 'neighbor collision rejection');
  record('unselected-neighbor-fingertip-contact-is-not-exempt');

  c.reset(); const palmPoint = robot.links.left_dex1_base_link.localToWorld(new THREE.Vector3(.04, 0, 0));
  assert(c.setObject('blue', palmPoint)); before = snapshot(); const palm = query(c.pose, CLOSED);
  assert(!palm.safe && palm.contacts.some(contact => [contact.body1, contact.body2].includes('left_dex1_base_link')));
  assert.equal(c.setManualOpening(CLOSED), false); unchanged(before, 'selected palm collision rejection');
  record('selected-object-palm-contact-is-not-exempt');

  c.reset(); const armPoint = robot.links.upperArm_Link.localToWorld(new THREE.Vector3(.204, 0, 0));
  assert(c.setObject('blue', armPoint)); before = snapshot(); const arm = query(c.pose, CLOSED);
  assert(!arm.safe && arm.contacts.some(contact => [contact.body1, contact.body2].includes('upperArm_Link')));
  assert.equal(c.setManualOpening(CLOSED), false); unchanged(before, 'selected arm collision rejection');
  record('selected-object-arm-contact-is-not-exempt');

  assert.deepEqual(await hashes(), sourceHashes, 'Source changed during opening-safety regression; rerun before accepting evidence');
  const report = { pass: true, cases: results.length, results, sourceHashes, sourceFrozen: true,
    scope: 'Original MuJoCo model, native source collision meshes and production controller. No stubbed safety decisions or attachment constraints.' };
  await mkdir(new URL('work/validation/c5/', root), { recursive: true });
  await writeFile(new URL('work/validation/c5/opening-safety-final.json', root), `${JSON.stringify(report, null, 2)}\n`);
  console.log(JSON.stringify(report));
} finally { dynamics.dispose(); data.delete(); model.delete(); }
