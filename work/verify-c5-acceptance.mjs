// Release evidence verifier. It reads explicit reports and hashes; it neither
// simulates a missing test nor promotes development/old-source reports to pass.
import { readFile } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { resolve, relative, isAbsolute } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../', import.meta.url));
const errors = [], sourceHashes = {}, reportHashes = {}, hashCache = new Map();
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const check = (condition, message) => { if (!condition) errors.push(message); };
const finite = (number, label) => check(typeof number === 'number' && Number.isFinite(number), `${label}: non-finite number`);
const isFiniteNumber = number => typeof number === 'number' && Number.isFinite(number);
const sourceURDF = 'dist/c5/assets/aubo_C5.urdf';
const sourceCommit = '47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1';
const officialUrdfSha = 'c87a1b853791f91a7908d77e05bbf5f87496b02016ea1040916e7cad5c1c70df';
const commonSources = ['work/build-c5-dynamics.mjs', 'work/c5-test-robot.mjs',
  'dist/c5/grasp-controller.mjs', 'dist/c5/grasp-kinematics.mjs', 'dist/c5/c5-dynamics.mjs', 'dist/c5/c5-profile.mjs',
  'dist/c5/models/c5-dynamic-scene.xml', sourceURDF, 'dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-dynamics.mjs', 'dist/g1/g1-profile.mjs', 'dist/g1/assets/g1_29dof_with_dex1_1.urdf',
  ...Array.from({ length: 7 }, (_, index) => `dist/c5/assets/meshes/aubo_C5/collision/link${index}.STL`),
  ...[1, 2].map(index => `dist/g1/assets/dex1_1/meshes/dex1_col_${index}.stl`)];

function canonical(path) {
  if (typeof path !== 'string' || !path || isAbsolute(path)) throw new Error(`Invalid source path: ${path}`);
  const normalized = relative(root, resolve(root, path));
  if (normalized === '..' || normalized.startsWith('../') || !normalized) throw new Error(`Source outside repository: ${path}`);
  return normalized;
}
async function currentHash(path) {
  const normalized = canonical(path);
  if (!hashCache.has(normalized)) hashCache.set(normalized, readFile(resolve(root, normalized)).then(sha));
  return hashCache.get(normalized);
}
async function bindHash(path, hash, label) {
  check(typeof hash === 'string' && /^[0-9a-f]{64}$/.test(hash), `${label}: malformed SHA-256 for ${path}`);
  try {
    const normalized = canonical(path), actual = await currentHash(path);
    sourceHashes[normalized] = actual;
    check(actual === hash, `${label}: stale source ${path}`);
  } catch (error) { errors.push(`${label}: ${error.message}`); }
}
async function bind(record, label, required = []) {
  const hashes = record?.sourceHashes;
  check(hashes && typeof hashes === 'object' && !Array.isArray(hashes), `${label}: sourceHashes absent`);
  if (!hashes || typeof hashes !== 'object') return;
  const normalized = new Set();
  for (const path of Object.keys(hashes)) { try { normalized.add(canonical(path)); } catch (error) { errors.push(`${label}: ${error.message}`); } }
  for (const path of required) check(normalized.has(canonical(path)), `${label}: missing source binding ${path}`);
  for (const [path, hash] of Object.entries(hashes)) await bindHash(path, hash, label);
}
async function report(name, jsonl = false) {
  const path = `work/validation/c5/${name}`, bytes = await readFile(resolve(root, path));
  reportHashes[path] = sha(bytes);
  return jsonl ? bytes.toString().trim().split(/\r?\n/).filter(Boolean).map(line => JSON.parse(line)) : JSON.parse(bytes);
}
const stable = object => JSON.stringify(Object.fromEntries(Object.keys(object).sort().map(key => [key, object[key]])));
const scenarios = [...['red', 'blue', 'yellow'].map(id => ({ id, kind: 'default_sort' })),
  { id: 'blue', offset: [-.015, .012] }, { id: 'yellow', offset: [.012, -.012] },
  { id: 'blue', yaw: .4 }, { id: 'yellow', roll: Math.PI / 2 }, { stack: true }];
const recoveries = ['unreachable_stack', 'failed_reset_then_sort', 'stop_stack_mid_transfer', 'stop_happy_mid_gesture'];
const expectedScenarios = repeats => [...scenarios.flatMap(scenario => Array.from({ length: repeats }, (_, index) => stable({ ...scenario, repeat: index + 1 }))),
  ...recoveries.map(recovery => stable({ recovery }))].sort();
const expectedJointNames = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
  'left_dex1_finger_joint_1', 'left_dex1_finger_joint_2'];
let limits;
function limitsFromSource(xml) {
  const result = {};
  for (const [, name, body] of xml.matchAll(/<joint\s+name="([^"]+)"[^>]*>([\s\S]*?)<\/joint>/g)) {
    const limit = body.match(/<limit\b[^>]+>/)?.[0];
    if (!limit) continue;
    const field = key => Number(limit.match(new RegExp(`\\b${key}="([^"]+)"`))?.[1]);
    result[name] = { lower: field('lower'), upper: field('upper'), effort: field('effort'), velocity: field('velocity') };
  }
  return result;
}
function lifted(evidence, label) {
  check(evidence?.dual === true, `${label}: physical dual-finger grasp absent`);
  finite(evidence?.lift, `${label}: actual object lift`);
  check(evidence?.lift > .035, `${label}: actual object did not lift 35 mm`);
}
function stackStable(stack, label) {
  check(stack?.stable === true && stack.states?.length === 3, `${label}: stable three-level stack absent`);
  for (const [index, state] of (stack?.states ?? []).entries()) {
    check(state.id === ['blue', 'red', 'yellow'][index] && state.supported === true, `${label}: incorrect physical support order`);
    finite(state.speed, `${label}: stack speed`); finite(state.error, `${label}: stack error`);
    check(state.speed < .0175 && state.error < .012, `${label}: released stack exceeds stability thresholds`);
  }
}
function jointRange(record, name, label, manual = false) {
  const source = limits[name], tolerance = name.includes('_finger_') ? .0002 : .001;
  if (!source) { check(false, `${label}: unexpected joint ${name}`); return; }
  const keys = manual ? ['min', 'max', 'maxActualVelocity', 'maxTargetVelocity', 'velocityLimit', 'effortLimit', 'maxTorque', 'maxRangeViolation']
    : ['min', 'max', 'targetMin', 'targetMax', 'maxAbsVelocity', 'maxAbsTorque', 'minimumLimitMargin', 'lower', 'upper', 'velocityLimit', 'effortLimit'];
  for (const key of keys) finite(record[key], `${label}/${name}/${key}`);
  check(record.velocityLimit === source.velocity && record.effortLimit === source.effort, `${label}: source velocity/effort limits changed for ${name}`);
  check(record.min >= source.lower - tolerance && record.max <= source.upper + tolerance, `${label}: actual source joint range exceeded for ${name}`);
  if (manual) {
    check(record.lower === source.lower && record.upper === source.upper, `${label}: manual source range metadata changed for ${name}`);
    check(record.maxRangeViolation <= tolerance, `${label}: actual range violation for ${name}`);
    check(record.maxActualVelocity <= source.velocity * 1.01 + .0001 && record.maxTargetVelocity <= source.velocity * 1.01 + .0001,
      `${label}: actual/commanded manual velocity exceeds source for ${name}`);
    check(record.maxTorque <= source.effort + 1e-7, `${label}: real manual effort exceeds source for ${name}`);
  } else {
    check(record.lower === source.lower && record.upper === source.upper, `${label}: source range metadata changed for ${name}`);
    check(record.minimumLimitMargin >= -tolerance && record.targetMin >= source.lower - 1e-12 && record.targetMax <= source.upper + 1e-12,
      `${label}: commanded or actual range exceeded for ${name}`);
    check(record.maxAbsVelocity <= source.velocity * 1.01 + .0001, `${label}: actual source velocity exceeded for ${name}`);
    check(record.maxAbsTorque <= source.effort + 1e-7, `${label}: actual source effort exceeded for ${name}`);
  }
}
function healthy(row, label) {
  check(row.pass === true, `${label}: reported failure`);
  for (const key of ['samples', 'millisecondSamples', 'fingerLimitSamples', 'maxFingerLimitViolation', 'maxTorque', 'maxFingerForce',
    'maxJointError', 'maxBaseDrift', 'maxActualVelocityRatio', 'maxObjectTranslationResidual', 'objectEditsDuringTask']) finite(row[key], `${label}/${key}`);
  check(row.samples > 0 && row.millisecondSamples >= row.samples * 9 && row.fingerLimitSamples === row.millisecondSamples,
    `${label}: 1 ms physical/finger/velocity sampling absent`);
  check(row.maxFingerLimitViolation <= .0002 && row.maxFingerForce <= 20 + 1e-7, `${label}: real finger stroke/force exceeded`);
  check(row.maxJointError < .07 && row.maxBaseDrift < 1e-12, `${label}: actual tracking or fixed mounting failed`);
  check(row.maxActualVelocityRatio <= 1.0105, `${label}: actual source velocity ratio exceeds the stated numerical allowance`);
  check(row.objectEditsDuringTask === 0 && row.maxObjectTranslationResidual < 1e-5, `${label}: object snap/edit not excluded`);
  check(JSON.stringify(Object.keys(row.jointExtrema ?? {}).sort()) === JSON.stringify([...expectedJointNames].sort()), `${label}: all eight real source axes not covered`);
  for (const [name, range] of Object.entries(row.jointExtrema ?? {})) jointRange(range, name, label);
  const collision = row.collision;
  check(collision?.samples === row.samples && collision.samples > 0 && collision.intervalSeconds > 0 && collision.intervalSeconds <= .01,
    `${label}: 10 ms actual solver-contact sampling absent`);
  finite(collision?.maxIllegalPenetration, `${label}: illegal penetration`);
  check(collision?.maxIllegalPenetration <= .001 && collision.illegalContacts?.length === 0 && Object.keys(collision.kinds ?? {}).length === 0,
    `${label}: actual robot self/table/neighbor penetration >1 mm`);
  if (!row.scenario?.recovery) {
    check(row.actualDualForce === true, `${label}: actual dual normal forces absent`); lifted(row.evidence, label);
    const phases = row.transitions?.map(transition => transition.key.split('/').at(-1)) ?? [];
    for (const phase of ['descend', 'closing', 'lift', 'transfer', 'place', 'release', 'retreat']) check(phases.includes(phase), `${label}: missing task phase ${phase}`);
    const itemIds = row.scenario.stack ? ['blue', 'red', 'yellow'] : [row.scenario.id];
    for (const id of itemIds) {
      const retreat = row.transitions?.find(transition => transition.key.split('/')[1] === id && transition.key.endsWith('/retreat'));
      check(retreat?.measurement?.count === 0, `${label}/${id}: finger release not physically observed`);
    }
    if (row.scenario.stack) {
      check(row.stackStableSeconds >= 3 && row.stackEvidence?.length === 3, `${label}: three-second real stack support missing`);
      for (const [index, evidence] of (row.stackEvidence ?? []).entries()) {
        check(evidence.id === ['blue', 'red', 'yellow'][index], `${label}: stack order changed`); lifted(evidence, label);
      }
      stackStable(row.stackcheck, label);
      for (const phase of ['stack_settle', 'happy', 'complete']) check(phases.includes(phase), `${label}: final phase ${phase} missing`);
      const completion = row.transitions?.find(transition => transition.key.endsWith('/complete'))?.time;
      finite(completion, `${label}: physical stack completion time`);
      check((row.millisecondSamples - 2) * .001 - completion >= 2.999, `${label}: additional three-second released-stack run absent`);
    } else {
      const measurement = row.measurement;
      check(measurement?.count === 0 && measurement.table === true && measurement.support === true && measurement.distance < .02 && measurement.speed < .025,
        `${label}: released object not physically settled in its frame`);
      check(row.evidence.maxSpeed < 1, `${label}: object flung during manipulation`);
    }
  } else {
    const recovery = row.scenario.recovery;
    if (recovery === 'unreachable_stack') check(row.history?.includes('shy') && !row.history.includes('happy') && typeof row.reason === 'string', `${label}: unreachable task not failed honestly`);
    if (recovery === 'failed_reset_then_sort') {
      lifted(row.evidence, label); check(row.actualDualForce === true && row.measurement?.table === true && row.measurement.distance < .02, `${label}: reset recovery did not grasp and release`);
    }
    if (recovery?.startsWith('stop_')) check(row.state === 'stopped' && row.collision.phases.includes('stopped'), `${label}: stop falsely completed/continued`);
    if (recovery === 'stop_stack_mid_transfer') check(row.actualDualForce === true && row.collision.phases.includes('lift'), `${label}: stop was not exercised after actual grasp/lift`);
    if (recovery === 'stop_happy_mid_gesture') stackStable(row.stackcheck, label);
  }
}

async function main() {
  const [single, repeated, opening, physics, visuals, ui, browser, manual] = await Promise.all([
    report('grasp-release-1x.jsonl', true), report('grasp-release-3x.jsonl', true), report('opening-release.json'),
    report('physics-current.json'), report('visuals-final.json'), report('ui-static-final.json'), report('browser-final.json'), report('manual-motion-final.json')]);
  check(await currentHash(sourceURDF) === officialUrdfSha, 'Original C5 URDF byte identity changed or was replaced with i5');
  const official = limitsFromSource(await readFile(resolve(root, sourceURDF), 'utf8'));
  const dex1 = limitsFromSource(await readFile(resolve(root, 'dist/g1/assets/g1_29dof_with_dex1_1.urdf'), 'utf8'));
  limits = Object.fromEntries(expectedJointNames.map(name => [name, official[name] ?? dex1[name]]));
  for (const name of expectedJointNames) check(limits[name] && Object.values(limits[name]).every(isFiniteNumber), `Source limit missing: ${name}`);
  for (const [name, rows, repeats] of [['single', single, 1], ['repeat', repeated, 3]]) {
    check(rows.length === 8 * repeats + 4, `${name}: incorrect case count ${rows.length}`);
    check(JSON.stringify(rows.map(row => stable(row.scenario)).sort()) === JSON.stringify(expectedScenarios(repeats)), `${name}: missing/duplicate/unexpected scenarios`);
    for (const [index, row] of rows.entries()) {
      await bind(row, `${name}[${index}]`, ['work/test-c5-dynamic-grasp.mjs', ...commonSources]); healthy(row, `${name}[${index}]`);
    }
  }
  const openingNames = ['source-mesh-opening-counterexample', 'manual-opening-into-table-rejected-without-state-write',
    'automatic-preparation-rejects-colliding-initial-expansion', 'stop-keeps-safe-gap-when-opening-would-hit-table',
    'failure-keeps-safe-gap-when-opening-would-hit-table', 'reaction-opening-is-collision-checked-before-motion',
    'nonfinite-manual-opening-rejected', 'normal-open-close-is-accepted-and-physically-motor-driven',
    'selected-fingertip-contact-is-real-and-allowed', 'unselected-neighbor-fingertip-contact-is-not-exempt',
    'selected-object-palm-contact-is-not-exempt', 'selected-object-arm-contact-is-not-exempt'];
  await bind(opening, 'opening', ['work/test-c5-opening-safety.mjs', ...commonSources]);
  check(opening.pass === true && opening.sourceFrozen === true && opening.cases === 12 && opening.results?.length === 12, 'opening: twelve current-source cases missing');
  check(JSON.stringify(opening.results?.map(result => result.name).sort()) === JSON.stringify(openingNames.sort()) && opening.results.every(result => result.pass === true), 'opening: incomplete safety boundary coverage');
  const fixture = opening.results?.find(result => result.name === 'source-mesh-opening-counterexample');
  check(fixture?.closedSafe === true && fixture.openSafe === false && fixture.penetration > .003
    && fixture.contacts?.some(contact => [contact.body1, contact.body2].includes('task_table') && contact.dist < -.003), 'opening: real source-mesh opening/table counterexample missing');
  const driven = opening.results?.find(result => result.name === 'normal-open-close-is-accepted-and-physically-motor-driven');
  check(driven?.steps >= 3000 && driven.maxFingerViolation <= .0002 && driven.maxFingerSpeed <= .2021, 'opening: real motor-driven physical range/speed checks missing');

  await bind(physics, 'physics', ['work/test-c5-dynamics.mjs', ...commonSources.filter(path => !path.includes('grasp-'))]);
  check(physics.nu === 8 && physics.nq === 29 && physics.nv === 26 && physics.links === 10 && physics.rootHeight === .72, 'physics: six-axis C5 + two-finger fixed assembly changed');
  check(physics.massError < 1e-12 && physics.comError < 1e-12 && physics.inertiaError < 2e-8 && physics.fkPositionError < 1e-10 && physics.fkAngleError < 1e-6, 'physics: original source inertias/transforms not retained');
  check(physics.maxStaticError < .001 && physics.maxTrackingError < .003 && physics.actualMotorWork === true && physics.maxFingerLimitViolation <= .0002 && physics.maxFingerSpeedStress <= .2021, 'physics: true motor tracking/range/speed stress failed');
  for (const flag of ['neutralSafe', 'fingerAllowed', 'palmBlocked', 'carriedObjectArmBlocked', 'actualStateUnchanged']) check(physics.collisionChecks?.[flag] === true, `physics: ${flag} failed`);
  check(physics.collisionChecks?.blockedSelf?.contacts?.some(contact => contact.kind === 'robot-self' && contact.dist < -.001), 'physics: nonadjacent C5 self-collision not detected');
  check(physics.collisionChecks?.blockedEnvironment?.contacts?.some(contact => contact.kind === 'robot-environment' && contact.dist < -.001), 'physics: table/plinth/floor collision not detected');
  check(physics.objectPoseChecks?.predictionPositionError < 1e-12 && physics.objectPoseChecks.predictionRotationError < 1e-7
    && physics.objectPoseChecks.invalidCasesRejected === 5 && physics.objectPoseChecks.actualStateUnchanged === true, 'physics: shadow prediction mutates actual state or accepts invalid objects');

  const manifestPath = 'dist/c5/assets/SOURCE_MANIFEST.json', manifest = JSON.parse(await readFile(resolve(root, manifestPath), 'utf8'));
  check(manifest.repository === 'AuboRobot/aubo_description' && manifest.commit === sourceCommit && manifest.model === 'aubo_C5' && manifest.files?.length === 18, 'visual: official C5 source provenance changed');
  check(manifest.license?.declaration === 'BSD' && manifest.license.fullLicenseTextPresentInSourceRepository === false, 'source: BSD declaration/full-text limitation must remain truthful');
  check(visuals.ok === true && visuals.sourceCommit === sourceCommit && visuals.sourceFilesUnchanged === 18 && visuals.sourceFiles?.length === 18
    && visuals.totalVertices === 14862 && visuals.totalVisualTriangles === 15469 && visuals.totalCollisionTriangles === 3140 && visuals.links?.length === 7, 'visual: original source geometry coverage incomplete');
  await bindHash('dist/c5/c5-3ds.mjs', visuals.parserSha256, 'visual parser');
  sourceHashes[manifestPath] = await currentHash(manifestPath);
  for (const file of manifest.files ?? []) {
    const record = visuals.sourceFiles?.find(item => item.path === file.path), path = `dist/c5/assets/${file.path}`;
    check(record?.unchanged === true && record.bytes === file.bytes && record.sha256 === file.sha256, `visual: missing source identity ${file.path}`);
    await bindHash(path, file.sha256, 'official asset');
    const bytes = await readFile(resolve(root, path));
    check(bytes.length === file.bytes && createHash('sha1').update(`blob ${bytes.length}\0`).update(bytes).digest('hex') === file.gitBlobSha, `official asset: Git blob identity mismatch ${file.path}`);
  }
  for (const [index, link] of (visuals.links ?? []).entries()) check(link.link === index && link.allFacesPreserved === true && link.maxMeterVertexError < 6e-8
    && Object.values(link.faceMaterialCounts ?? {}).reduce((sum, value) => sum + value, 0) === link.visualTriangles, `visual: source face/material/mm conversion mismatch link ${index}`);
  check(visuals.negativeInputs?.length === 10 && visuals.negativeInputs.every(test => test.rejected === true), 'visual: invalid 3DS boundaries incomplete');
  for (const rgb of ['127,127,127', '217,216,233', '24,76,198', '185,185,185']) check(visuals.colors?.includes(rgb), `visual: original C5 paint ${rgb} absent`);

  const uiSources = ['work/check-c5-ui.mjs', 'dist/c5/index.html', 'dist/c5/c5-app.mjs', 'dist/c5/c5-joint-ui.mjs',
    'dist/c5/c5-workcell.mjs', 'dist/c5/styles.css', 'dist/c5/c5-materials.mjs', 'dist/c5/c5-profile.mjs'];
  await bind(ui, 'static UI', uiSources);
  check(ui.pass === true && ui.checks?.length === 10 && new Set(ui.checks).size === 10, 'static UI: ten UI/geometry/camera assertions missing');
  check(Math.abs(ui.plinth?.min?.[1]) < 1e-7 && Math.abs(ui.plinth?.max?.[1] - .72) < 1e-7
    && Math.abs(ui.table?.min?.[1]) < 1e-7 && Math.abs(ui.table?.max?.[1] - .72) < 1e-7
    && ui.table.min[0] - ui.plinth.max[0] > .09, 'static UI: grounded, separated C5 pedestal/table missing');

  await bind(browser, 'native browser', [...uiSources.filter(path => path !== 'work/check-c5-ui.mjs'),
    'dist/c5/grasp-controller.mjs', 'dist/c5/c5-dynamics.mjs', 'dist/c5/c5-3ds.mjs', 'dist/c5/models/c5-dynamic-scene.xml']);
  const release = browser.releaseConfirmation;
  check(release?.pass === true && release.manualMotionRevisionVerified === true, 'browser: final manual-motion revision not inspected');
  check(JSON.stringify(release?.desktop) === '[1280,720]' && JSON.stringify(release?.mobile) === '[390,844]', 'browser: final desktop/phone inspection absent');
  check(release?.controllerSha256 === await currentHash('dist/c5/grasp-controller.mjs'), 'browser: confirmation refers to an old controller');
  const html = await readFile(resolve(root, 'dist/c5/index.html'), 'utf8'), cache = [...html.matchAll(/\?v=([\w-]+)/g)].map(match => match[1]);
  check(cache.length >= 2 && cache.every(version => version === release?.cacheVersion), 'browser: rendered release cache differs from current entry point');
  check(browser.rendered?.desktop?.robotAndGroundedTableFullyVisible === true && browser.rendered.desktop.originalC5SilverGrayBlueMaterialsVisible === true
    && browser.rendered.desktop.dex1Black === true && browser.rendered.desktop.twoCameraFeedsContainRenderedScene === true, 'browser: actual C5/grounded table/material/camera render not observed');
  check(browser.rendered?.mobile?.horizontalOverflow === false && browser.rendered.mobile.allSevenChipsInsideScene === true
    && browser.rendered.mobile.sceneAbovePanel === true && browser.rendered.mobile.fullPageInspected === true, 'browser: phone scene/panel inspection incomplete');
  check(browser.rendered?.screenshots?.captureMethod?.includes('Native browser') && Array.isArray(browser.rendered.screenshots.localPaths), 'browser: native screenshot inspection provenance absent');
  check(browser.console?.entries?.length === 0, 'browser: observed console errors/warnings remain');
  for (const name of ['blankSpaceOrbit', 'solidObjectDrag', 'objectRightDragRotation', 'keyboardObjectMove', 'jointSlider', 'gripperSlider', 'cameraFeeds', 'reset']) {
    check(browser.interactions?.[name]?.status?.startsWith('passed'), `browser: native interaction ${name} not observed`);
  }

  const manualNames = [...expectedJointNames.filter(name => !name.includes('_finger_')).map(name => `large-step-${name}`),
    'rapid-reversals', 'continuous-tcp-edits', 'pending-stop-clears-motion', 'pending-reset-clears-motion',
    'pending-start-clears-motion', 'pending-failure-clears-motion', 'actual-departure-A-B-rejected-without-state-write', 'near-table-arm-and-gripper-interaction'];
  await bind(manual, 'manual motion', ['work/test-c5-manual-motion.mjs', ...commonSources]);
  check(manual.pass === true && manual.sourceFrozen === true && manual.cases === 14 && manual.results?.length === 14, 'manual motion: fourteen current-source cases incomplete');
  check(JSON.stringify(manual.results?.map(result => result.name).sort()) === JSON.stringify(manualNames.sort()), 'manual motion: missing/duplicate/unexpected cases');
  for (const result of manual.results ?? []) {
    const label = `manual/${result.name}`; check(result.pass === true, `${label}: failed`);
    for (const key of ['samples', 'collisionSamples', 'maxVelocityRatio', 'maxFingerLimitViolation', 'maxIllegalPenetration', 'maxPositionIntegrationResidual', 'noActualRewriteChecks']) finite(result[key], `${label}/${key}`);
    check(result.samples > 0 && result.collisionSamples > 0 && result.maxIllegalPenetration <= .001 && result.maxFingerLimitViolation <= .0002
      && result.maxVelocityRatio <= 1.0105 && result.maxPositionIntegrationResidual < 1e-5 && result.noActualRewriteChecks > 0,
    `${label}: actual motion/collision/state-invariance evidence incomplete`);
    check(JSON.stringify(Object.keys(result.perJoint ?? {}).sort()) === JSON.stringify([...expectedJointNames].sort()), `${label}: all eight physical source axes missing`);
    for (const [name, range] of Object.entries(result.perJoint ?? {})) jointRange(range, name, label, true);
  }

  // Bind the verifier and the human-readable interpretation too, then re-read
  // every bound source to reject a mixed snapshot changed during verification.
  for (const path of ['work/verify-c5-acceptance.mjs', 'work/C5-ACCEPTANCE.md']) sourceHashes[path] = await currentHash(path);
  for (const [path, hash] of Object.entries(sourceHashes)) check(sha(await readFile(resolve(root, path))) === hash, `Source changed during acceptance verification: ${path}`);
  const rows = [...single, ...repeated], tasks = rows.filter(row => !row.scenario.recovery), stacks = tasks.filter(row => row.scenario.stack);
  const lifts = tasks.flatMap(row => row.scenario.stack ? row.stackEvidence : [row.evidence]);
  const maximum = key => Math.max(...rows.map(row => row[key]));
  const result = { pass: errors.length === 0,
    scope: 'C5 + simulation-mounted Dex1 browser preview: source-bound physical regression, manual-motion safety and native desktop/phone inspection; not hardware certification.',
    reports: { single: single.length, repeat: repeated.length, total: rows.length, sort: tasks.length - stacks.length,
      stack: stacks.length, recovery: rows.length - tasks.length, opening: opening.cases, manual: manual.cases, originalAssetFiles: visuals.sourceFilesUnchanged, staticUI: ui.checks?.length },
    observed: { millisecondSamples: rows.reduce((sum, row) => sum + row.millisecondSamples, 0),
      actualContactSamples: rows.reduce((sum, row) => sum + row.collision.samples, 0),
      maxFingerLimitViolationMetres: maximum('maxFingerLimitViolation'), maxIllegalPenetrationMetres: Math.max(...rows.map(row => row.collision.maxIllegalPenetration)),
      maxActualSourceVelocityRatio: maximum('maxActualVelocityRatio'), maxJointErrorRad: maximum('maxJointError'), maxBaseDriftMetres: maximum('maxBaseDrift'),
      maxFingerForceN: maximum('maxFingerForce'), maxMotorTorqueNm: maximum('maxTorque'), maxObjectTranslationResidualMetres: maximum('maxObjectTranslationResidual'),
      objectEditsDuringTask: rows.reduce((sum, row) => sum + row.objectEditsDuringTask, 0), minimumPhysicalLiftMetres: Math.min(...lifts.map(evidence => evidence.lift)),
      maximumSortPositionErrorMetres: Math.max(...tasks.filter(row => !row.scenario.stack).map(row => row.measurement.distance)),
      sourceVertices: visuals.totalVertices, sourceVisualTriangles: visuals.totalVisualTriangles },
    limits: { actualSamplingSeconds: .001, actualContactSamplingSeconds: .01, realFingerRangeToleranceMetres: .0002,
      revoluteRangeToleranceRadians: .001, illegalPenetrationThresholdMetres: .001, actualSourceVelocityNumericalAllowance: '1% + 0.0001 in the native unit',
      requiredActualLiftMetres: .035, stackStableSeconds: 3, additionalReleasedStackSeconds: 3 },
    nativeScreenshots: { inspectedInline: true, persistedToDisk: browser.rendered.screenshots.persistedToDisk,
      localPaths: browser.rendered.screenshots.localPaths, note: 'No saved screenshot path is required when the documented native API only emits inspected image bytes.' },
    notCertified: ['physical touchscreen gestures', 'arbitrary-scene continuous-time collision freedom', 'real C5/Dex1 adapter fit or camera calibration',
      'real-machine control, commissioning, operational safety or certification', 'an SPDX-specific license grant absent from upstream evidence'],
    officialSource: { repository: manifest.repository, commit: manifest.commit, originalUrdfSha256: officialUrdfSha,
      declaredLicense: manifest.license.declaration, fullLicenseTextSuppliedByUpstream: false }, sourceHashes, reportHashes, errors };
  console.log(JSON.stringify(result, null, 2)); if (!result.pass) process.exitCode = 1;
}
main().catch(error => { console.error(JSON.stringify({ pass: false, error: error.message })); process.exitCode = 1; });
