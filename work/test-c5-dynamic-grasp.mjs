import assert from 'node:assert/strict';
import { readFile, mkdir, appendFile } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import { GraspController } from '../dist/c5/grasp-controller.mjs';
import { C5Dynamics } from '../dist/c5/c5-dynamics.mjs';
import { ROOT_HEIGHT, TABLE_HEIGHT, HOME_POSE, OBJECT_START_X, OBJECT_DROP_X, OBJECT_LATERAL_OFFSET } from '../dist/c5/c5-profile.mjs';
import { loadC5TestRobot } from './c5-test-robot.mjs';

const root = new URL('../', import.meta.url), args = process.argv.slice(2);
const filter = args.find(arg => !arg.startsWith('--'));
const repeat = Number(args.find(arg => arg.startsWith('--repeat='))?.split('=')[1] || 1);
assert(Number.isInteger(repeat) && repeat > 0 && repeat <= 10, 'Invalid repetition count');
const reportPath = args.find(arg => arg.startsWith('--report='))?.slice(9) || `work/validation/c5/dynamic-grasp-${Date.now()}.jsonl`;
await mkdir(new URL('work/validation/c5/', root), { recursive: true });
const hashPaths = [
  'work/test-c5-dynamic-grasp.mjs', 'work/build-c5-dynamics.mjs', 'work/c5-test-robot.mjs',
  'dist/c5/grasp-controller.mjs', 'dist/c5/grasp-kinematics.mjs', 'dist/c5/c5-dynamics.mjs', 'dist/c5/c5-profile.mjs',
  'dist/c5/models/c5-dynamic-scene.xml', 'dist/c5/assets/aubo_C5.urdf', 'dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-dynamics.mjs', 'dist/g1/g1-profile.mjs', 'dist/g1/assets/g1_29dof_with_dex1_1.urdf',
  ...Array.from({ length: 7 }, (_, index) => `dist/c5/assets/meshes/aubo_C5/collision/link${index}.STL`),
  ...[1, 2].map(index => `dist/g1/assets/dex1_1/meshes/dex1_col_${index}.stl`),
];
async function hashes() {
  return Object.fromEntries(await Promise.all(hashPaths.map(async path => [path,
    createHash('sha256').update(await readFile(new URL(path, root))).digest('hex')])));
}
const sourceHashes = await hashes();
const report = async record => {
  const line = JSON.stringify({ ...record, sourceHashes });
  console.log(JSON.stringify({ scenario: record.scenario, pass: record.pass, error: record.error,
    state: record.state, lift: record.evidence?.lift, collisionSamples: record.collision?.samples,
    maxIllegalPenetration: record.collision?.maxIllegalPenetration, maxFingerLimitViolation: record.maxFingerLimitViolation }));
  await appendFile(new URL(reportPath, root), `${line}\n`);
};
const api = await createMuJoCo({ locateFile: () => new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', root).pathname });
const xml = await readFile(new URL('dist/c5/models/c5-dynamic-scene.xml', root), 'utf8');
const targetsFor = () => [
  { id: 'red', body: 'red_cube', z: OBJECT_LATERAL_OFFSET, h: .0225 },
  { id: 'blue', body: 'blue_block', z: 0, h: .019 },
  { id: 'yellow', body: 'yellow_cylinder', z: -OBJECT_LATERAL_OFFSET, h: .026 },
].map(target => ({ ...target, start: new THREE.Vector3(OBJECT_START_X, TABLE_HEIGHT + target.h, target.z),
  drop: new THREE.Vector3(OBJECT_DROP_X, TABLE_HEIGHT + target.h, target.z), rotation: new THREE.Quaternion() }));
async function scene(targets) {
  const vfs = new api.MjVFS();
  const model = api.MjModel.from_xml_string(xml, vfs), data = new api.MjData(model), robot = await loadC5TestRobot();
  vfs.delete();
  const dynamics = new C5Dynamics(api, model, data, robot), c = new GraspController(api, model, data, robot, targets, dynamics);
  assert.equal(model.nu, 8); assert.equal(model.nmocap, 0); assert.equal(model.neq, 0);
  return { model, data, robot, dynamics, c };
}
const compact = measurement => ({ count: measurement.count, force: measurement.force, forces: measurement.forces,
  table: measurement.table, support: measurement.support, position: measurement.position.toArray(),
  speed: measurement.speed, distance: measurement.distance });
function createMonitor({ model, data, robot, dynamics, c }) {
  const base = api.mj_name2id(model, api.mjtObj.mjOBJ_BODY.value, 'base_link');
  const plinth = api.mj_name2id(model, api.mjtObj.mjOBJ_BODY.value, 'robot_plinth');
  const basePosition = [...data.xpos.subarray(base * 3, base * 3 + 3)];
  const objects = [...c.bodies.values()].map(body => {
    const joint = model.body_jntadr[body];
    return { body, q: model.jnt_qposadr[joint], d: model.jnt_dofadr[joint] };
  });
  const stats = { samples: 0, millisecondSamples: 0, fingerLimitSamples: 0, maxFingerLimitViolation: 0,
    maxTorque: 0, maxFingerForce: 0, maxJointError: 0, maxBaseDrift: 0, maxActualVelocityRatio: 0,
    maxObjectTranslationResidual: 0, objectEditsDuringTask: 0, actualDualForce: false, jointExtrema: {}, trackingByPhase: {},
    collision: { samples: 0, intervalSeconds: .01, maxIllegalPenetration: 0, illegalContacts: [], kinds: {}, phases: [] } };
  const originalSetObject = c.setObject.bind(c);
  c.setObject = (...parameters) => {
    if (c.running) { stats.objectEditsDuringTask++; assert.fail('Task motion must not edit actual task-object state'); }
    return originalSetObject(...parameters);
  };
  const inspectEveryMillisecond = () => {
    stats.millisecondSamples++; stats.fingerLimitSamples++;
    const telemetry = dynamics.telemetry();
    assert.equal(telemetry.joints.length, 6); assert.equal(telemetry.allJoints.length, 8);
    for (const joint of telemetry.allJoints) {
      assert([joint.actual, joint.torque, joint.error, joint.velocity, joint.targetVelocity].every(Number.isFinite));
      assert(Math.abs(joint.torque) <= joint.effortLimit + 1e-7, `${joint.name}: effort limit at ${data.time}/${c.state}`);
      const violation = Math.max(0, joint.lower - joint.actual, joint.actual - joint.upper);
      assert(violation <= (joint.unit === 'm' ? .0002 : .001), `${joint.name}: actual range violation ${violation} at ${data.time}/${c.state}`);
      assert(joint.velocityLimit > 0, `${joint.name}: missing original source velocity limit`);
      const ratio = Math.abs(joint.velocity) / joint.velocityLimit;
      stats.maxActualVelocityRatio = Math.max(stats.maxActualVelocityRatio, ratio);
      assert(Math.abs(joint.velocity) <= joint.velocityLimit * 1.01 + .0001,
        `${joint.name}: actual velocity ${joint.velocity} exceeds source ${joint.velocityLimit} (+1% numerical allowance) at ${data.time}/${c.state}`);
      const range = stats.jointExtrema[joint.name] ??= { min: joint.actual, max: joint.actual, targetMin: joint.target,
        targetMax: joint.target, maxAbsVelocity: 0, maxAbsTorque: 0, minimumLimitMargin: Infinity,
        lower: joint.lower, upper: joint.upper, velocityLimit: joint.velocityLimit, effortLimit: joint.effortLimit, unit: joint.unit };
      range.min = Math.min(range.min, joint.actual); range.max = Math.max(range.max, joint.actual);
      range.targetMin = Math.min(range.targetMin, joint.target); range.targetMax = Math.max(range.targetMax, joint.target);
      range.maxAbsVelocity = Math.max(range.maxAbsVelocity, Math.abs(joint.velocity));
      range.maxAbsTorque = Math.max(range.maxAbsTorque, Math.abs(joint.torque));
      range.minimumLimitMargin = Math.min(range.minimumLimitMargin, joint.actual - joint.lower, joint.upper - joint.actual);
      if (joint.unit === 'm') {
        assert.equal(joint.effortLimit, 20); stats.maxFingerForce = Math.max(stats.maxFingerForce, Math.abs(joint.torque));
        stats.maxFingerLimitViolation = Math.max(stats.maxFingerLimitViolation, violation);
      } else {
        stats.maxTorque = Math.max(stats.maxTorque, Math.abs(joint.torque));
        stats.maxJointError = Math.max(stats.maxJointError, Math.abs(joint.error));
      }
    }
    const baseActual = new THREE.Vector3(...data.xpos.subarray(base * 3, base * 3 + 3));
    stats.maxBaseDrift = Math.max(stats.maxBaseDrift, baseActual.distanceTo(new THREE.Vector3(...basePosition)));
    assert(Math.abs(baseActual.z - ROOT_HEIGHT) < 1e-12, 'Fixed C5 base height must not move');
    assert(stats.maxBaseDrift < 1e-12, 'C5 fixed-base mounting must not drift');
  };
  const inspectContacts = () => {
    stats.samples++;
    const telemetry = dynamics.telemetry(), measurement = c.measure();
    if (measurement.count === 2 && measurement.forces.every(force => force > .015)) stats.actualDualForce = true;
    const phase = stats.trackingByPhase[c.state] ??= { jointError: 0, wristError: 0 };
    phase.jointError = Math.max(phase.jointError, ...telemetry.joints.map(joint => Math.abs(joint.error)));
    phase.wristError = Math.max(phase.wristError, c.wristError());
    const selected = c.bodies.get(c.selected), fingers = new Set(c.fingers.left.map(finger => finger.body));
    const illegal = [], contacts = data.contact;
    stats.collision.samples++;
    if (!stats.collision.phases.includes(c.state)) stats.collision.phases.push(c.state);
    try {
      for (let index = 0; index < Math.min(data.ncon, contacts.size()); index++) {
        const contact = contacts.get(index);
        try {
          const body1 = model.geom_bodyid[contact.geom1], body2 = model.geom_bodyid[contact.geom2];
          const robot1 = dynamics.robotBodyIds.has(body1), robot2 = dynamics.robotBodyIds.has(body2);
          if (!robot1 && !robot2) continue;
          const mounting = (body1 === base && body2 === plinth) || (body2 === base && body1 === plinth);
          const selectedFinger = (fingers.has(body1) && body2 === selected) || (fingers.has(body2) && body1 === selected);
          if (mounting || selectedFinger) continue;
          const penetration = Math.max(0, -contact.dist), kind = robot1 && robot2 ? 'robot-self' : 'robot-environment';
          stats.collision.maxIllegalPenetration = Math.max(stats.collision.maxIllegalPenetration, penetration);
          if (penetration > .001) {
            const record = { time: data.time, phase: c.state, selected: c.selected, kind, dist: contact.dist,
              body1: dynamics.bodyNames[body1], body2: dynamics.bodyNames[body2],
              geom1: dynamics.geomNames[contact.geom1], geom2: dynamics.geomNames[contact.geom2],
              worldPosition: [contact.pos[0], contact.pos[2], -contact.pos[1]] };
            illegal.push(record); stats.collision.kinds[kind] = (stats.collision.kinds[kind] || 0) + 1;
            if (stats.collision.illegalContacts.length < 12) stats.collision.illegalContacts.push(record);
          }
        } finally { contact.delete(); }
      }
    } finally { contacts.delete(); }
    assert.equal(illegal.length, 0, `Actual contact penetration >1 mm: ${JSON.stringify(illegal.slice(0, 3))}`);
  };
  let steps = 0;
  return { stats,
    inspect() { inspectEveryMillisecond(); inspectContacts(); },
    step() {
      const before = objects.map(object => ({ ...object,
        position: [...data.qpos.subarray(object.q, object.q + 3)], velocity: [...data.qvel.subarray(object.d, object.d + 3)] }));
      c.step();
      for (const object of before) {
        const displacement = new THREE.Vector3(...data.qpos.subarray(object.q, object.q + 3)).sub(new THREE.Vector3(...object.position));
        const oldSpeed = Math.hypot(...object.velocity), newSpeed = Math.hypot(...data.qvel.subarray(object.d, object.d + 3));
        const residual = Math.max(0, displacement.length() - .001 * Math.max(oldSpeed, newSpeed));
        stats.maxObjectTranslationResidual = Math.max(stats.maxObjectTranslationResidual, residual);
        assert(residual < 1e-5, `Task object ${object.body} moved without corresponding physical velocity: ${residual} m`);
      }
      inspectEveryMillisecond();
      if (steps++ % 10 === 0) inspectContacts();
    },
    snapshot() { return { ...stats }; },
  };
}
function assertHealthy(monitor) {
  assert(monitor.stats.maxJointError < .07, 'Actual six-axis motion must track without saturation/blocking');
  assert.equal(monitor.stats.objectEditsDuringTask, 0);
  assert(monitor.stats.maxFingerLimitViolation <= .0002);
  assert(monitor.stats.collision.samples > 0);
}
const scenarios = [
  ...['red', 'blue', 'yellow'].map(id => ({ id, kind: 'default_sort' })),
  { id: 'blue', offset: [-.015, .012] }, { id: 'yellow', offset: [.012, -.012] },
  { id: 'blue', yaw: .4 }, { id: 'yellow', roll: Math.PI / 2 }, { stack: true },
];
const chosen = scenarios.filter(scenario => (!filter || JSON.stringify(scenario).includes(filter))
  && (!args.includes('--smoke') || scenario.kind === 'default_sort' || scenario.stack));
const failures = [];
for (const scenario of chosen.flatMap(scenario => Array.from({ length: repeat }, (_, index) => ({ ...scenario, repeat: index + 1 })))) {
  const targets = targetsFor(), target = targets.find(candidate => candidate.id === scenario.id);
  if (scenario.offset) { target.start.x += scenario.offset[0]; target.start.z += scenario.offset[1]; }
  if (scenario.yaw) target.rotation.setFromAxisAngle(new THREE.Vector3(0, 1, 0), scenario.yaw);
  if (scenario.roll) { target.rotation.setFromAxisAngle(new THREE.Vector3(1, 0, 0), scenario.roll); target.start.y = TABLE_HEIGHT + .02; }
  const current = await scene(targets), { model, data, dynamics, c } = current, monitor = createMonitor(current);
  let previous = ''; const transitions = [];
  try {
    for (let index = 0; index < 300; index++) monitor.step();
    assert(scenario.stack ? c.startStack('left') : c.start('left', scenario.id), `${c.message}: ${JSON.stringify(c.planError)}`);
    for (let index = 0; index < (scenario.stack ? 130000 : 40000) && c.running; index++) {
      monitor.step(); const key = `${c.itemIndex}/${c.selected}/${c.state}`;
      if (key !== previous) { transitions.push({ time: data.time, key, measurement: compact(c.measure()) }); previous = key; }
    }
    monitor.inspect(); assert.equal(c.state, 'complete', c.message);
    assert(monitor.stats.actualDualForce, 'Both physical fingers must exhibit actual solver normal force');
    if (scenario.stack) {
      assert(c.stackResultVerified); assert(c.stackStableSeconds >= 3); assert.equal(c.stackEvidence.length, 3);
      assert.deepEqual(c.stackEvidence.map(item => item.id), ['blue', 'red', 'yellow']);
      assert(c.stackEvidence.every(item => item.dual && item.lift > .035));
      for (let index = 0; index < 3000; index++) monitor.step();
      monitor.inspect(); assert(c.stackCheck().stable, 'Released stack must remain physically supported another 3 seconds');
      assert(c.history.includes('happy') && !c.history.includes('shy'));
    } else {
      assert(c.evidence.dual && c.evidence.lift > .035); assert(c.evidence.maxSpeed < 1);
      assert(c.measure().distance < .02 && c.measure().table);
    }
    for (const [name, value] of Object.entries(HOME_POSE)) assert.equal(c.pose[name], value, 'Return must end at the defined six-axis home');
    assertHealthy(monitor);
    const result = { scenario, pass: true, evidence: c.evidence, measurement: compact(c.measure()),
      stackStableSeconds: c.stackStableSeconds, stackEvidence: c.stackEvidence,
      stackcheck: scenario.stack ? c.stackCheck() : undefined, ...monitor.snapshot(), transitions };
    c.reset(); assert.equal(c.state, 'idle'); assert.equal(c.running, false); assert.equal(c.reaction, null);
    assert(!c.stackResultVerified && !c.reactionFinished); assert.equal(c.history.length, 0);
    await report(result);
  } catch (error) {
    const failure = { scenario, pass: false, error: error.message, state: c.state, message: c.message, selected: c.selected,
      itemIndex: c.itemIndex, planError: c.planError, commandedPose: c.pose, evidence: c.evidence, measurement: compact(c.measure()),
      stackEvidence: c.stackEvidence, stackcheck: scenario.stack ? c.stackCheck() : undefined, ...monitor.snapshot(), transitions };
    failures.push(failure); await report(failure);
  } finally { dynamics.dispose(); data.delete(); model.delete(); }
}
if ((!filter || filter === 'recovery') && !args.includes('--smoke')) {
  const targets = targetsFor(); targets.find(target => target.id === 'blue').start.x = 1.8;
  const current = await scene(targets), { model, data, dynamics, c } = current;
  let monitor = createMonitor(current);
  try {
    for (let index = 0; index < 300; index++) monitor.step();
    assert.equal(c.startStack('left'), false); assert(c.running && c.reaction === 'shy'); const reason = c.message;
    for (let index = 0; index < 15000 && c.running; index++) monitor.step();
    monitor.inspect(); assert.equal(c.state, 'failed'); assert.equal(c.message, reason);
    assert(c.history.includes('shy') && !c.history.includes('happy')); assert(!c.stackResultVerified); assertHealthy(monitor);
    await report({ scenario: { recovery: 'unreachable_stack' }, pass: true, reason, history: c.history, ...monitor.snapshot() });
    targets.find(target => target.id === 'blue').start.x = OBJECT_START_X; c.reset(); monitor = createMonitor(current);
    for (let index = 0; index < 300; index++) monitor.step();
    assert(c.start('left', 'blue'));
    for (let index = 0; index < 40000 && c.running; index++) monitor.step();
    monitor.inspect(); assert.equal(c.state, 'complete'); assert(c.evidence.dual && c.evidence.lift > .035 && c.measure().table); assertHealthy(monitor);
    await report({ scenario: { recovery: 'failed_reset_then_sort' }, pass: true, evidence: c.evidence, measurement: compact(c.measure()), ...monitor.snapshot() });
    c.reset(); monitor = createMonitor(current); for (let index = 0; index < 300; index++) monitor.step();
    assert(c.startStack('left'));
    for (let index = 0; index < 30000 && c.running && c.state !== 'transfer'; index++) monitor.step();
    assert.equal(c.state, 'transfer'); assert(c.evidence.lift > .035); c.stop(); const stoppedPose = { ...c.pose };
    for (let index = 0; index < 4000; index++) monitor.step();
    monitor.inspect(); assert.equal(c.state, 'stopped'); assert(!c.running); assert.equal(c.reaction, null);
    assert.deepEqual(c.pose, stoppedPose); assert(!c.stackResultVerified); assertHealthy(monitor);
    await report({ scenario: { recovery: 'stop_stack_mid_transfer' }, pass: true, state: c.state, measurement: compact(c.measure()), ...monitor.snapshot() });
    c.reset(); monitor = createMonitor(current); for (let index = 0; index < 300; index++) monitor.step();
    assert(c.startStack('left'));
    for (let index = 0; index < 130000 && c.running && c.state !== 'happy'; index++) monitor.step();
    assert.equal(c.state, 'happy'); assert(c.stackResultVerified);
    for (let index = 0; index < 100; index++) monitor.step(); c.stop();
    for (let index = 0; index < 4000; index++) monitor.step();
    monitor.inspect(); assert.equal(c.state, 'stopped'); assert(!c.running); assert.equal(c.reaction, null);
    assert.notEqual(c.reactionFinished, 'happy'); assert(c.stackCheck().stable); assertHealthy(monitor);
    await report({ scenario: { recovery: 'stop_happy_mid_gesture' }, pass: true, state: c.state, stackcheck: c.stackCheck(), ...monitor.snapshot() });
    c.reset(); assert.equal(c.state, 'idle'); assert(!c.stackResultVerified && !c.reactionFinished); assert.equal(c.history.length, 0);
  } catch (error) {
    const failure = { scenario: { recovery: true }, pass: false, error: error.message, state: c.state, message: c.message,
      history: c.history, measurement: compact(c.measure()), planError: c.planError, ...monitor.snapshot() };
    failures.push(failure); await report(failure);
  } finally { dynamics.dispose(); data.delete(); model.delete(); }
}
const frozen = JSON.stringify(sourceHashes) === JSON.stringify(await hashes());
console.log(JSON.stringify({ report: reportPath, failures: failures.length, sourceFrozen: frozen, scenarios: chosen.length * repeat }));
assert(frozen, 'Production/test source changed during this regression; rerun before accepting evidence');
assert.equal(failures.length, 0, `${failures.length} C5 physical grasp acceptance cases failed`);
