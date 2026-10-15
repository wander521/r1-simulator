import * as THREE from 'three';
import { OrbitControls } from '../vendor/OrbitControls.js';
import URDFLoader from '../vendor/URDFLoader.js';

const $ = (selector) => document.querySelector(selector);
const canvas = $('#main-view');
const stage = $('#viewport');
const ui = {
  loading: $('#loading-overlay'), loadingText: $('#loading-text'), status: $('#status'), start: $('#start-grasp'), reset: $('#reset'),
  progress: $('#grasp-progress'), progressValue: $('#grasp-progress-value'), stage: $('#grasp-stage'), armPose: $('#panel-hint'),
  width: $('#grasp-force'), trajectory: $('#rms-error'), objectState: $('#contact-count'), toast: null, steps: [],
  armSelect: $('#model-variant'),
  torqueChart: $('#torque-chart'), errorChart: $('#error-chart'), targetButtons: [...document.querySelectorAll('.vision-target')],
  cameraToggle: $('#toggle-camera-views'), cameraGrid: $('#camera-feed-grid'), contactToggle: $('#toggle-contact-visuals'),
};

const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x09111f, 0.22);
const camera = new THREE.PerspectiveCamera(39, 1, 0.02, 12);
camera.position.set(2.5, 1.5, 2.75);
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setClearColor(0x0b1726, 1); renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap; renderer.outputColorSpace = THREE.SRGBColorSpace;
const controls = new OrbitControls(camera, canvas); controls.target.set(0, 0.66, 0); controls.minDistance = 0.8; controls.maxDistance = 4.2; controls.maxPolarAngle = Math.PI * 0.48; controls.update();
const overheadCamera = new THREE.PerspectiveCamera(42, 4 / 3, .02, 12); overheadCamera.position.set(.9, 2.5, 1.2);
const wristCamera = new THREE.PerspectiveCamera(55, 4 / 3, .02, 12);
const visionRenderers = [new THREE.WebGLRenderer({ canvas: $('#overhead-camera-canvas'), antialias: true }), new THREE.WebGLRenderer({ canvas: $('#wrist-camera-canvas'), antialias: true })];
visionRenderers.forEach((view) => { view.setPixelRatio(1); view.outputColorSpace = THREE.SRGBColorSpace; view.setClearColor(0x07121f, 1); });
scene.add(new THREE.HemisphereLight(0xbde9ff, 0x07121d, 3.8));
const key = new THREE.DirectionalLight(0xe9faff, 4.2); key.position.set(1.4, 2.2, 1.5); key.castShadow = true; key.shadow.mapSize.set(1024, 1024); scene.add(key);
const rim = new THREE.PointLight(0x43c8ff, 17, 3.5); rim.position.set(-1.2, 1.4, -1.1); scene.add(rim);
const warm = new THREE.PointLight(0x96efe2, 9, 2.5); warm.position.set(.55, .75, .35); scene.add(warm);

const ground = new THREE.Mesh(new THREE.PlaneGeometry(5, 5), new THREE.MeshStandardMaterial({ color: 0x11283b, metalness: .18, roughness: .7 }));
ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true; scene.add(ground);
const grid = new THREE.GridHelper(3.5, 22, 0x4c9fbd, 0x1c4961); grid.position.y = .004; grid.material.transparent = true; grid.material.opacity = .55; scene.add(grid);

let robot, object, workcell, objectStart, objectHome, isRunning = false, animationFrame, toastTimer, graspValidated = false, selectedTarget = 'blue';
const taskObjects = new Map();
const targetZones = new Map();
const telemetryHistory = [];
let lastJointValues = {}, lastForce = 0, lastContactCount = 0, lastContactDistance = Infinity, lastTelemetryAt = 0;
let mjPhysics = null;
let physicsInitPromise = null;
const gripperPoint = new THREE.Vector3(), gripperQuat = new THREE.Quaternion(), direction = new THREE.Vector3();
const objectRaycaster = new THREE.Raycaster(), objectPointer = new THREE.Vector2(), objectDragPoint = new THREE.Vector3();
const objectDragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -.72);
const tableBounds = { minX: .46, maxX: 1.10, minZ: -.32, maxZ: .32 };
let objectDrag = null;
let runEvidence = null;
const contactDebug = new THREE.Group(); contactDebug.name = 'MuJoCo 接触点'; scene.add(contactDebug);
const threeToMujocoRotation = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
const physicsPadLinks = {
  right_finger_a_pad: 'right_gripper_finger_link1', right_finger_b_pad: 'right_gripper_finger_link2',
  left_finger_a_pad: 'left_gripper_finger_link1', left_finger_b_pad: 'left_gripper_finger_link2',
};
const torsoRest = { torso_joint1: 0, torso_joint2: .06, torso_joint3: 0, torso_joint4: 0 };
// R1 的中段由四个躯干关节串联；抓取时让它们协同前倾，形成下蹲取物动作。
// 中段折叠下蹲，但三段俯仰的合角保持接近 0：胸腔不会后仰。
// 回提一档的下蹲：保留约 -0.1 rad 的轻微前倾，不再蹲到最低位。
const torsoCrouch = { torso_joint1: 1.45, torso_joint2: -2.35, torso_joint3: -.80, torso_joint4: 0 };
// 在“胸腔略向前、深蹲到桌面高度”的躯干姿态下离线求得的末端解。
// 两侧指尖中点距正前方桌面抓取点约 4 mm；它们是固定工作台的稳定种子。
const tablePickPoses = {
  right: { right_arm_joint1: -.244, right_arm_joint2: 3.180, right_arm_joint3: -2.518, right_arm_joint4: .086, right_arm_joint5: -.725, right_arm_joint6: 2.182 },
  left: { left_arm_joint1: .379, left_arm_joint2: 3.180, left_arm_joint3: -2.072, left_arm_joint4: 1.373, left_arm_joint5: 1.108, left_arm_joint6: 2.181 },
};
const rightPoses = {
  // 待命时肩到肘（joint2）和肘到腕（joint3）都折向下；不能只让腕部垂下。
  rest: { right_arm_joint1: -1.57, right_arm_joint2: 2.20, right_arm_joint3: -1.40, right_arm_joint4: 0, right_arm_joint5: 0, right_arm_joint6: 0 },
  spread: { right_arm_joint1: .64, right_arm_joint2: 1.04, right_arm_joint3: -1.35, right_arm_joint4: .1, right_arm_joint5: .3, right_arm_joint6: .02 },
  safe: { right_arm_joint1: .88, right_arm_joint2: 1.13, right_arm_joint3: -1.48, right_arm_joint4: .12, right_arm_joint5: .28, right_arm_joint6: .06 },
  approach: { right_arm_joint1: 1.08, right_arm_joint2: 1.37, right_arm_joint3: -1.77, right_arm_joint4: .1, right_arm_joint5: .17, right_arm_joint6: .08 },
  pick: { right_arm_joint1: 1.25, right_arm_joint2: 1.56, right_arm_joint3: -1.95, right_arm_joint4: .1, right_arm_joint5: .13, right_arm_joint6: .08 },
  transfer: { right_arm_joint1: .72, right_arm_joint2: 1.15, right_arm_joint3: -1.47, right_arm_joint4: .08, right_arm_joint5: .28, right_arm_joint6: .05 },
  place: { right_arm_joint1: .86, right_arm_joint2: 1.36, right_arm_joint3: -1.7, right_arm_joint4: .08, right_arm_joint5: .18, right_arm_joint6: .05 },
};
const leftPoses = {
  // 左臂使用镜像肩关节，肘部关节保持同样的向下折叠。
  rest: { left_arm_joint1: 1.57, left_arm_joint2: 2.20, left_arm_joint3: -1.40, left_arm_joint4: 0, left_arm_joint5: 0, left_arm_joint6: 0 },
  spread: { left_arm_joint1: -.64, left_arm_joint2: 1.04, left_arm_joint3: -1.35, left_arm_joint4: -.1, left_arm_joint5: -.3, left_arm_joint6: -.02 },
  safe: { left_arm_joint1: -.88, left_arm_joint2: 1.13, left_arm_joint3: -1.48, left_arm_joint4: -.12, left_arm_joint5: -.28, left_arm_joint6: -.06 },
  approach: { left_arm_joint1: -1.08, left_arm_joint2: 1.37, left_arm_joint3: -1.77, left_arm_joint4: -.1, left_arm_joint5: -.17, left_arm_joint6: -.08 },
  pick: { left_arm_joint1: -1.25, left_arm_joint2: 1.56, left_arm_joint3: -1.95, left_arm_joint4: -.1, left_arm_joint5: -.13, left_arm_joint6: -.08 },
  transfer: { left_arm_joint1: -.72, left_arm_joint2: 1.15, left_arm_joint3: -1.47, left_arm_joint4: -.08, left_arm_joint5: -.28, left_arm_joint6: -.05 },
  place: { left_arm_joint1: -.86, left_arm_joint2: 1.36, left_arm_joint3: -1.7, left_arm_joint4: -.08, left_arm_joint5: -.18, left_arm_joint6: -.05 },
};
const jointBounds = {
  right: [[-2.75, 2.75], [0, 3.18], [-3.2, 0], [-2.75, 2.75], [-1.62, 1.62], [-2.75, 2.75]],
  left: [[-2.75, 2.75], [0, 3.18], [-3.2, 0], [-2.75, 2.75], [-1.62, 1.62], [-2.75, 2.75]],
};
let activeArm = 'right';
const armJoints = (arm) => Object.keys(arm === 'left' ? leftPoses.rest : rightPoses.rest);
const blend = (from, to, amount) => { const eased = amount * amount * (3 - 2 * amount), result = {}; Object.keys(from).forEach((name) => result[name] = THREE.MathUtils.lerp(from[name], to[name], eased)); return result; };
const armGripper = (arm, value) => ({ [`${arm}_gripper_finger_joint1`]: value, [`${arm}_gripper_finger_joint2`]: value });
function bothArms(right, left, rightFinger = .05, leftFinger = .05, torso = torsoRest) { return { ...torso, ...right, ...left, ...armGripper('right', rightFinger), ...armGripper('left', leftFinger) }; }
function activeLabel() { return activeArm === 'right' ? '右臂' : '左臂'; }
function activePoses() { return activeArm === 'right' ? rightPoses : leftPoses; }
function inactivePoses() { return activeArm === 'right' ? leftPoses : rightPoses; }

function makeMaterial(color, metalness = .28, roughness = .42) { return new THREE.MeshStandardMaterial({ color, metalness, roughness }); }
function createWorkcell() {
  const table = new THREE.Group();
  const top = new THREE.Mesh(new THREE.BoxGeometry(.72, .065, .76), makeMaterial(0x829caf, .48, .3)); top.position.y = -.04; top.castShadow = true; top.receiveShadow = true; table.add(top);
  const mat = new THREE.Mesh(new THREE.BoxGeometry(.62, .008, .66), makeMaterial(0x24516b, .18, .4)); mat.position.y = -.002; table.add(mat);
  const legMaterial = makeMaterial(0x1f3e52, .38, .38);
  [[-.28,-.29],[.28,-.29],[-.28,.29],[.28,.29]].forEach(([x,z]) => { const leg = new THREE.Mesh(new THREE.BoxGeometry(.045,.65,.045), legMaterial); leg.position.set(x,-.365,z); leg.castShadow = true; table.add(leg); });
  return table;
}
// 直接沿用参考场景 rs_grasp_scene.xml 的三种任务物：红色立方体、
// 蓝色长方体与黄色圆柱。材质颜色与参考场景的 rs_red/rs_blue/rs_yellow 一致。
function addReferenceObject(position, spec) {
  const group = new THREE.Group();
  group.name = `reference-${spec.id}`; group.userData.taskId = spec.id;
  const material = makeMaterial(spec.color, .2, .3); material.emissive = new THREE.Color(spec.color).multiplyScalar(.10);
  const geometry = spec.kind === 'cylinder' ? new THREE.CylinderGeometry(.020, .020, .052, 32) : spec.kind === 'block' ? new THREE.BoxGeometry(.052, .038, .032, 2, 2, 2) : new THREE.BoxGeometry(.045, .045, .045, 2, 2, 2);
  const shape = new THREE.Mesh(geometry, material); shape.castShadow = true; shape.receiveShadow = true; shape.userData.taskId = spec.id; group.add(shape);
  const edgeGeometry = spec.kind === 'cylinder' ? new THREE.EdgesGeometry(new THREE.CylinderGeometry(.020, .020, .052, 20)) : new THREE.EdgesGeometry(geometry);
  const edges = new THREE.LineSegments(edgeGeometry, new THREE.LineBasicMaterial({ color: 0xeaffff, transparent: true, opacity: .62 })); edges.userData.taskId = spec.id; group.add(edges);
  const ring = new THREE.Mesh(new THREE.RingGeometry(.034, .041, 32), new THREE.MeshBasicMaterial({ color: 0x4ee8e4, transparent: true, opacity: .72, side: THREE.DoubleSide }));
  ring.name = 'selection-ring'; ring.rotation.x = -Math.PI / 2; ring.position.y = spec.kind === 'cylinder' ? -.023 : spec.kind === 'block' ? -.016 : -.020; ring.visible = false; group.add(ring);
  group.position.copy(position); scene.add(group); return group;
}
function addZone(position, color, label) {
  const zone = new THREE.Group();
  zone.name = `${label}放置区`;
  const plate = new THREE.Mesh(new THREE.BoxGeometry(.105,.003,.105), new THREE.MeshStandardMaterial({ color, transparent: true, opacity: .34, metalness: .35, roughness: .28 })); zone.add(plate);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(plate.geometry), new THREE.LineBasicMaterial({ color, transparent: true, opacity: .95 })); zone.add(edges);
  const marker = new THREE.Mesh(new THREE.RingGeometry(.028, .034, 28), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: .88, side: THREE.DoubleSide })); marker.rotation.x = -Math.PI / 2; marker.position.y = .003; zone.add(marker);
  zone.position.copy(position); scene.add(zone); return zone;
}
function setChineseLabels() {
  document.querySelector('.loading-title').textContent = '正在构建 R1 本地数字孪生';
  ui.loadingText.textContent = '正在加载…'; ui.status.textContent = '正在加载…';
  document.querySelector('.eyebrow').textContent = '视觉 + 物理抓取演示'; document.querySelector('h1').textContent = 'R1 双臂抓取仿真';
  $('#toggle-drag').textContent = '启用末端拖拽'; ui.reset.textContent = '复位'; $('#toggle-guides').textContent = '显示全部引导';
  document.querySelector('.camera-controls h2').textContent = '视觉模式'; ui.cameraToggle.textContent = '隐藏画面'; $('#camera-layout-row').textContent = '并排'; $('#camera-layout-column').textContent = '上下排列'; $('#toggle-camera-model').textContent = '隐藏相机模型';
  document.querySelectorAll('.camera-feed-heading span:first-child')[0].textContent = '全局俯视相机'; document.querySelectorAll('.camera-feed-heading span:first-child')[1].textContent = 'R1 腕部相机 · RGB';
  document.querySelector('.grasp-controls h2').textContent = '一键抓取演示'; document.querySelector('.control-hint').textContent = '双臂先展开，躯干前倾下蹲；所选侧的平行夹爪接触方块后才会闭合搬运。';
  ui.start.textContent = '▶ 开始抓取'; $('#cancel-grasp').textContent = '停止'; document.querySelector('.grasp-stage span').textContent = '阶段'; ui.stage.textContent = '就绪';
  document.querySelector('.telemetry-controls h2').textContent = '接触与受力'; ui.contactToggle.textContent = '隐藏接触点';
  const metricLabels = document.querySelectorAll('.telemetry-metrics span'); metricLabels[0].textContent = '接触数'; metricLabels[1].textContent = '夹持力'; metricLabels[2].textContent = '均方根误差';
  document.querySelectorAll('.telemetry-chart span')[0].textContent = '关节力矩曲线 · N·m'; document.querySelectorAll('.telemetry-chart span')[1].textContent = '关节误差曲线 · rad';
  $('#gripper-open').textContent = '张开'; $('#gripper-close').textContent = '闭合'; document.querySelector('.object-edit-hud-heading strong').textContent = '任务对象'; $('#object-edit-selection').textContent = '已选择：蓝色长方块（可自由拖动）'; document.querySelector('.object-edit-hud p:last-child').textContent = '点中物体后拖动可调整抓取位置；拖动空白处可旋转视角。';
}
function drawTelemetry(canvas, key, ceiling) {
  const ctx = canvas.getContext('2d'), width = canvas.width, height = canvas.height;
  ctx.clearRect(0, 0, width, height); ctx.fillStyle = '#020b18'; ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = 'rgba(81,160,192,.20)'; ctx.lineWidth = 1;
  for (let row = 1; row < 4; row++) { const y = height * row / 4; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
  if (telemetryHistory.length < 2) return;
  const colors = ['#5ee0ff', '#8fd7c1', '#f5d76e', '#ff9f5a', '#ff7ab6', '#7aa2ff'];
  for (let joint = 0; joint < 6; joint++) { ctx.strokeStyle = colors[joint]; ctx.lineWidth = 1.25; ctx.beginPath(); telemetryHistory.forEach((sample, i) => { const x = i * width / Math.max(1, telemetryHistory.length - 1), y = height - 5 - Math.min(1, sample[key][joint] / ceiling) * (height - 10); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); }
}
function sampleTelemetry(now = performance.now()) {
  if (now - lastTelemetryAt < 80) return; lastTelemetryAt = now;
  const names = armJoints(activeArm), current = names.map((name) => Number(lastJointValues[name] || 0));
  const rest = names.map((name) => Math.abs((activePoses().rest[name] || 0) - (lastJointValues[name] || 0)));
  const torque = current.map((value, i) => Math.min(28, Math.abs(value) * 8 + (i === 5 ? lastForce * .25 : lastForce * .08)));
  telemetryHistory.push({ torque, error: rest }); if (telemetryHistory.length > 150) telemetryHistory.shift();
  drawTelemetry(ui.torqueChart, 'torque', 30); drawTelemetry(ui.errorChart, 'error', 3.2);
}
function threeToMujocoPoint(point) { return new THREE.Vector3(point.x, -point.z, point.y); }
function mujocoToThreePoint(values) { return new THREE.Vector3(values[0], values[2], -values[1]); }
function threeToMujocoQuaternion(quaternion) {
  // The loaded URDF root is rotated -90° about X for Three's Y-up scene.
  // Premultiplying by the inverse coordinate rotation recovers MuJoCo's Z-up
  // world frame; do not conjugate, because pad/object local frames are MJCF.
  return threeToMujocoRotation.clone().multiply(quaternion);
}
function mujocoToThreeQuaternion(values) {
  return threeToMujocoRotation.clone().invert().multiply(new THREE.Quaternion(values[1], values[2], values[3], values[0]));
}
function setPhysicsObjectPose(target, position = target.core, quaternion = target.group.quaternion) {
  if (!mjPhysics || !target) return;
  const address = mjPhysics.objectQpos.get(target.id); if (address === undefined) return;
  const point = threeToMujocoPoint(position), rotation = threeToMujocoQuaternion(quaternion);
  const { data } = mjPhysics;
  data.qpos[address] = point.x; data.qpos[address + 1] = point.y; data.qpos[address + 2] = point.z;
  data.qpos[address + 3] = rotation.w; data.qpos[address + 4] = rotation.x; data.qpos[address + 5] = rotation.y; data.qpos[address + 6] = rotation.z;
  const velocity = mjPhysics.objectDof.get(target.id); for (let i = 0; i < 6; i++) data.qvel[velocity + i] = 0;
}
function syncPhysicsPads() {
  if (!mjPhysics || !robot) return;
  const { data, model, padMocap } = mjPhysics;
  Object.entries(physicsPadLinks).forEach(([padName, linkName]) => {
    const link = robot.getObjectByName(linkName), mocapId = padMocap.get(padName); if (!link || mocapId === undefined) return;
    const point = threeToMujocoPoint(link.getWorldPosition(new THREE.Vector3()));
    const rotation = threeToMujocoQuaternion(link.getWorldQuaternion(new THREE.Quaternion()));
    data.mocap_pos[mocapId * 3] = point.x; data.mocap_pos[mocapId * 3 + 1] = point.y; data.mocap_pos[mocapId * 3 + 2] = point.z;
    data.mocap_quat[mocapId * 4] = rotation.w; data.mocap_quat[mocapId * 4 + 1] = rotation.x; data.mocap_quat[mocapId * 4 + 2] = rotation.y; data.mocap_quat[mocapId * 4 + 3] = rotation.z;
    const padGeom = mjPhysics.padGeoms.get(padName);
    const arm = padName.startsWith('right') ? 'right' : 'left';
    const opening = lastJointValues[`${arm}_gripper_finger_joint1`] ?? .05;
    // Collision pads expand only while a parallel gripper closes.  This avoids
    // reporting a grasp merely because an open finger passes near an object.
    const closing = THREE.MathUtils.clamp((.05 - opening) / .047, 0, 1);
    model.geom_size[padGeom * 3] = THREE.MathUtils.lerp(.017, .042, closing);
  });
  ['right', 'left'].forEach((arm) => {
    const mocapId = mjPhysics.supportMocap.get(arm); if (mocapId === undefined) return;
    const enabled = runEvidence?.carrierActive && arm === activeArm;
    // This small compliant support is only moved after the measured two-pad
    // pinch.  It gives the physical object a real load path while preserving
    // free-body collisions and slip instead of parenting it to the gripper.
    let point = new THREE.Vector3(0, 0, -10);
    // Keep the support just below the object bottom.  Spawning it inside the
    // object creates a large solver impulse (the reported "弹飞" symptom).
    if (enabled) { point = threeToMujocoPoint(activeFingerMidpoint(arm).add(new THREE.Vector3(0, -.037, 0))); runEvidence.carrierAnchor = point.clone(); }
    else if (arm === activeArm && runEvidence?.releaseAnchor && runEvidence.releaseAmount < 1) point = runEvidence.releaseAnchor.clone().add(new THREE.Vector3(0, 0, -.12 * runEvidence.releaseAmount));
    data.mocap_pos[mocapId * 3] = point.x; data.mocap_pos[mocapId * 3 + 1] = point.y; data.mocap_pos[mocapId * 3 + 2] = point.z;
    data.mocap_quat[mocapId * 4] = 1; data.mocap_quat[mocapId * 4 + 1] = 0; data.mocap_quat[mocapId * 4 + 2] = 0; data.mocap_quat[mocapId * 4 + 3] = 0;
  });
  mjPhysics.api.mj_forward(model, data);
}
function syncTaskObjectsFromPhysics() {
  if (!mjPhysics) return;
  taskObjects.forEach((target) => {
    const body = mjPhysics.objectBodies.get(target.id); if (body === undefined) return;
    target.group.position.copy(mujocoToThreePoint(mjPhysics.data.xpos.slice(body * 3, body * 3 + 3)));
    if (mjPhysics.data.xquat) target.group.quaternion.copy(mujocoToThreeQuaternion(mjPhysics.data.xquat.slice(body * 4, body * 4 + 4)));
  });
}
function clearContactDebug() { contactDebug.clear(); }
function drawPhysicsContacts() {
  clearContactDebug();
  if (!mjPhysics || ui.contactToggle.getAttribute('aria-checked') !== 'true') return;
  const { data } = mjPhysics;
  for (let index = 0; index < Math.min(data.ncon, data.contact.size()); index++) {
    const contact = data.contact.get(index); if (!contact?.pos) continue;
    const point = mujocoToThreePoint(contact.pos);
    const marker = new THREE.Mesh(new THREE.SphereGeometry(.007, 10, 8), new THREE.MeshBasicMaterial({ color: 0x4ef6da }));
    marker.position.copy(point); contactDebug.add(marker);
  }
}
function physicalContactState(target = taskObjects.get(selectedTarget)) {
  if (!mjPhysics || !target) return { count: 0, force: 0, table: false, speed: Infinity, distance: Infinity };
  const { api, model, data, objectBodies, tableBody, padBodies } = mjPhysics;
  const objectBody = objectBodies.get(target.id); const pads = activeArm === 'right' ? [padBodies.get('right_finger_a_pad'), padBodies.get('right_finger_b_pad')] : [padBodies.get('left_finger_a_pad'), padBodies.get('left_finger_b_pad')];
  const hitPads = new Set(); let table = false, force = 0;
  const normal = new Float64Array(6);
  for (let index = 0; index < Math.min(data.ncon, data.contact.size()); index++) {
    const contact = data.contact.get(index); if (!contact) continue;
    const a = model.geom_bodyid[contact.geom1], b = model.geom_bodyid[contact.geom2];
    if (a === objectBody || b === objectBody) {
      const other = a === objectBody ? b : a;
      if (pads.includes(other)) { hitPads.add(other); normal.fill(0); api.mj_contactForce(model, data, index, normal); force += Math.abs(normal[0]); }
      if (other === tableBody) table = true;
    }
  }
  const dof = mjPhysics.objectDof.get(target.id), speed = Math.hypot(data.qvel[dof], data.qvel[dof + 1], data.qvel[dof + 2]);
  const position = mujocoToThreePoint(data.xpos.slice(objectBody * 3, objectBody * 3 + 3));
  return { count: hitPads.size, force, table, speed, distance: position.distanceTo(target.drop), position };
}
function stepPhysics(frameSteps = 5) {
  if (!mjPhysics) return physicalContactState();
  syncPhysicsPads();
  for (let i = 0; i < frameSteps; i++) mjPhysics.api.mj_step(mjPhysics.model, mjPhysics.data);
  syncTaskObjectsFromPhysics(); drawPhysicsContacts();
  return physicalContactState();
}
function resetPhysicsWorld() {
  if (!mjPhysics) return;
  mjPhysics.api.mj_resetData(mjPhysics.model, mjPhysics.data);
  taskObjects.forEach((target) => setPhysicsObjectPose(target));
  syncPhysicsPads(); syncTaskObjectsFromPhysics(); clearContactDebug();
}
async function initMuJoCoPhysics() {
  const [{ default: createMuJoCo }, xml] = await Promise.all([
    import('./mujoco-runtime.mjs'),
    fetch('./models/r1_grasp_scene.xml').then((response) => { if (!response.ok) throw new Error(`MJCF HTTP ${response.status}`); return response.arrayBuffer(); }),
  ]);
  const api = await createMuJoCo({ locateFile: () => new URL('../Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', import.meta.url).href });
  const vfs = new api.MjVFS(); vfs.addBuffer('r1_grasp_scene.xml', new Uint8Array(xml));
  const names = ['base_link.STL', 'torso_link4.STL', 'right_gripper_link.STL', 'right_gripper_finger_link1.STL', 'right_gripper_finger_link2.STL'];
  await Promise.all(names.map(async (name) => {
    const response = await fetch(`./assets/meshes/${name}`); if (!response.ok) throw new Error(`网格读取失败：${name}`);
    vfs.addBuffer(`../assets/meshes/${name}`, new Uint8Array(await response.arrayBuffer()));
  }));
  const model = api.MjModel.from_xml_path('r1_grasp_scene.xml', vfs), data = new api.MjData(model);
  const kind = api.mjtObj, bodyId = (name) => api.mj_name2id(model, kind.mjOBJ_BODY.value, name);
  const objectNames = { red: 'red_cube', blue: 'blue_block', yellow: 'yellow_cylinder' };
  const objectBodies = new Map(Object.entries(objectNames).map(([id, name]) => [id, bodyId(name)]));
  const objectQpos = new Map(), objectDof = new Map();
  objectBodies.forEach((body, id) => { const joint = model.body_jntadr[body]; objectQpos.set(id, model.jnt_qposadr[joint]); objectDof.set(id, model.jnt_dofadr[joint]); });
  const padBodies = new Map(Object.keys(physicsPadLinks).map((name) => [name, bodyId(name)]));
  const padMocap = new Map(); padBodies.forEach((body, name) => padMocap.set(name, model.body_mocapid[body]));
  const padGeoms = new Map(); padBodies.forEach((body, name) => padGeoms.set(name, model.body_geomadr[body]));
  const supportMocap = new Map(['right', 'left'].map((arm) => [arm, model.body_mocapid[bodyId(`${arm}_grasp_support`)]]));
  api.mj_forward(model, data);
  mjPhysics = { api, model, data, vfs, objectBodies, objectQpos, objectDof, padBodies, padMocap, padGeoms, supportMocap, tableBody: bodyId('task_table') };
  if (robot) resetPhysicsWorld();
  console.info(`R1 MuJoCo ready: ${model.ngeom} colliders, ${model.nu} actuators, ${padMocap.size} URDF-driven fingertip pads`);
}
function toast(message) { ui.status.textContent = message; }
function setStep(index) { ui.steps.forEach((step, i) => step.classList.toggle('is-active', i <= index)); }
function setStage(progress) {
  let name = '双臂展开', status = '双臂从垂直待命姿态展开', step = 0, trajectory = '展开';
  if (progress >= .18 && progress < .34) { name = '躯干前倾下蹲'; status = '中段躯干协同弯曲，降低作业高度'; trajectory = '下蹲'; }
  else if (progress >= .34 && progress < .5) { name = '接近目标'; status = `${activeLabel()}将夹爪移动到方块上方`; step = 1; trajectory = '接近'; }
  else if (progress >= .5 && progress < .59) { name = '下降对准'; status = '夹爪进入目标抓取位'; step = 1; trajectory = '下降'; }
  else if (progress >= .59 && progress < .68) { name = '闭合夹爪'; status = '两根手指滑动闭合，夹住方块'; step = 1; trajectory = '夹持'; }
  else if (progress >= .68 && progress < .77) { name = '抬升目标'; status = `目标随${activeLabel()}离开工作台`; step = 1; trajectory = '抬升'; }
  else if (progress >= .77 && progress < .87) { name = '移向收纳区'; status = `${activeLabel()}平稳搬运目标`; step = 2; trajectory = '搬运'; }
  else if (progress >= .87 && progress < .94) { name = '下降放置'; status = '方块对准收纳区下降'; step = 2; trajectory = '放置'; }
  else if (progress >= .94) { name = '释放并回正'; status = '夹爪展开，躯干与双臂回到垂直待命位'; step = 2; trajectory = '回正'; }
  ui.stage.textContent = name; ui.status.textContent = status; ui.trajectory.textContent = trajectory; setStep(step);
}
function apply(values) { lastJointValues = { ...lastJointValues, ...values }; robot.setJointValues(values); }
function torsoAt(progress) {
  if (progress < .18) return torsoRest;
  if (progress < .34) return blend(torsoRest, torsoCrouch, (progress - .18) / .16);
  if (progress < .94) return torsoCrouch;
  return blend(torsoCrouch, torsoRest, Math.min(1, (progress - .94) / .06));
}
function armMotion(progress) {
  const active = activePoses(), inactive = inactivePoses();
  if (progress < .18) return { active: blend(active.rest, active.spread, progress / .18), inactive: blend(inactive.rest, inactive.spread, progress / .18) };
  if (progress < .34) return { active: blend(active.spread, active.safe, (progress - .18) / .16), inactive: inactive.spread };
  if (progress < .5) return { active: blend(active.safe, active.approach, (progress - .34) / .16), inactive: inactive.spread };
  if (progress < .59) return { active: blend(active.approach, active.pick, (progress - .5) / .09), inactive: inactive.spread };
  if (progress < .68) return { active: active.pick, inactive: inactive.spread };
  if (progress < .77) return { active: blend(active.pick, active.approach, (progress - .68) / .09), inactive: inactive.spread };
  if (progress < .87) return { active: blend(active.approach, active.transfer, (progress - .77) / .1), inactive: inactive.spread };
  if (progress < .94) return { active: blend(active.transfer, active.place, (progress - .87) / .07), inactive: inactive.spread };
  return { active: blend(active.place, active.rest, (progress - .94) / .06), inactive: blend(inactive.spread, inactive.rest, (progress - .94) / .06) };
}
function updateGripper(progress) { const opening = progress < .59 ? .05 : progress < .68 ? THREE.MathUtils.lerp(.05, .003, (progress-.59)/.09) : progress < .94 ? .003 : THREE.MathUtils.lerp(.003,.05,Math.min(1,(progress-.94)/.04)); apply(armGripper(activeArm, opening)); }
function getGripPose() { robot.getObjectByName(`${activeArm}_gripper_link`).getWorldPosition(gripperPoint); robot.getObjectByName(`${activeArm}_gripper_link`).getWorldQuaternion(gripperQuat); }
function activeFingerMidpoint(arm = activeArm) {
  const a = robot.getObjectByName(`${arm}_gripper_finger_link1`).getWorldPosition(new THREE.Vector3());
  const b = robot.getObjectByName(`${arm}_gripper_finger_link2`).getWorldPosition(new THREE.Vector3());
  return a.add(b).multiplyScalar(.5);
}
function applyArmForSolve(arm, pose) {
  apply(bothArms(arm === 'right' ? pose : rightPoses.rest, arm === 'left' ? pose : leftPoses.rest, .05, .05, torsoCrouch));
  robot.updateMatrixWorld(true);
}
function solveTablePickPose(arm, target) {
  // 工作台和目标中心固定在 R1 正前方。此前从站立姿态的旧猜测做数值迭代，
  // 在深蹲时会卡到关节边界；优先使用与当前躯干姿态一致、已校准的种子解。
  const referenceTarget = new THREE.Vector3(.78, .7015, 0);
  if (target && target.distanceTo(referenceTarget) < .004) return { ...tablePickPoses[arm] };
  const names = armJoints(arm);
  // 从经过验证的正前方抓取解开始，而不是从站立姿态开始，允许用户在工作台内拖动目标。
  const pose = { ...tablePickPoses[arm] }, epsilon = .009, error = new THREE.Vector3();
  const evaluate = () => { applyArmForSolve(arm, pose); return activeFingerMidpoint(arm); };
  for (let iteration = 0; iteration < 128; iteration++) {
    const current = evaluate(); error.subVectors(target, current); if (error.length() < .012) break;
    const steps = names.map((name, index) => {
      const initial = pose[name]; pose[name] = initial + epsilon; const plus = evaluate(); pose[name] = initial - epsilon; const minus = evaluate(); pose[name] = initial;
      const derivative = plus.sub(minus).multiplyScalar(1 / (2 * epsilon));
      const delta = THREE.MathUtils.clamp(derivative.dot(error) * .48, -.060, .060), [minimum, maximum] = jointBounds[arm][index];
      return [name, THREE.MathUtils.clamp(initial + delta, minimum, maximum)];
    });
    steps.forEach(([name, value]) => { pose[name] = value; });
  }
  evaluate(); return pose;
}
function fingerContactState() {
  // MuJoCo is authoritative whenever it is available.  The geometric fallback
  // only keeps the UI usable if WebAssembly fails to load.
  if (mjPhysics) {
    const measured = physicalContactState();
    return { count: measured.count, distance: measured.distance, force: measured.force, table: measured.table, speed: measured.speed };
  }
  const fingerA = robot.getObjectByName(`${activeArm}_gripper_finger_link1`), fingerB = robot.getObjectByName(`${activeArm}_gripper_finger_link2`);
  if (!fingerA || !fingerB || !object) return { count: 0, distance: Infinity };
  const a = fingerA.getWorldPosition(new THREE.Vector3()), b = fingerB.getWorldPosition(new THREE.Vector3());
  const midpoint = a.clone().add(b).multiplyScalar(.5), spread = a.distanceTo(b), centered = midpoint.distanceTo(object.position);
  // A valid pinch has the object between the two finger link origins.  This
  // is a conservative collision proxy for the rendered URDF meshes; the MJCF
  // scene uses the corresponding two box colliders for force simulation.
  const pinched = centered < .035 && spread > .018 && spread < .18;
  return { count: pinched ? 2 : 0, distance: centered };
}
function applyPickPose(arm, target) {
  const useCalibratedPick = target.solutions?.[arm]?.pick && target.core.distanceTo(target.start) < .004;
  const solved = useCalibratedPick ? { ...target.solutions[arm].pick } : solveTablePickPose(arm, target.core), poses = arm === 'right' ? rightPoses : leftPoses;
  Object.assign(poses.pick, solved);
  poses.approach = blend(poses.spread, solved, .78); poses.safe = blend(poses.spread, solved, .48);
}
function configureGraspPath(arm, target) {
  applyPickPose(arm, target);
  const poses = arm === 'right' ? rightPoses : leftPoses;
  // 取物与放物分别解末端位姿：搬运段从抬升点连续过渡至该颜色的收纳区。
  const placed = target.solutions?.[arm]?.place ? { ...target.solutions[arm].place } : solveTablePickPose(arm, target.drop);
  poses.place = placed;
  poses.transfer = blend(poses.approach, placed, .56);
}
function markSelectedObject() {
  taskObjects.forEach((target) => {
    const active = target.id === selectedTarget;
    target.group.getObjectByName('selection-ring').visible = active;
    target.group.scale.setScalar(active ? 1.05 : 1);
  });
}
function updateDraggedTarget(target, point, announce = false) {
  target.core.set(THREE.MathUtils.clamp(point.x, tableBounds.minX, tableBounds.maxX), target.core.y, THREE.MathUtils.clamp(point.z, tableBounds.minZ, tableBounds.maxZ));
  target.group.position.copy(target.core); target.group.quaternion.identity(); setPhysicsObjectPose(target); if (mjPhysics) syncPhysicsPads(); objectStart = target.core.clone();
  if (announce) toast(`${target.label}已移动到工作台内的新抓取位置`);
}
function setTarget(id) {
  if (isRunning || !taskObjects.has(id)) return;
  selectedTarget = id;
  const selected = taskObjects.get(id); object = selected.group; objectStart = selected.core.clone(); objectHome = selected.drop.clone();
  document.querySelectorAll('.vision-target').forEach((button) => { const active = button.dataset.target === id; button.classList.toggle('active', active); button.setAttribute('aria-pressed', String(active)); });
  markSelectedObject(); configureGraspPath(activeArm, selected); const title = selected.label; $('#object-edit-selection').textContent = `已选择：${title}（可自由拖动）`; toast(`已选择${title}，将由${activeLabel()}执行抓取`); reset();
}
function reset() {
  cancelAnimationFrame(animationFrame); isRunning = false; runEvidence = null; graspValidated = false; lastForce = 0; lastContactCount = 0;
  apply(bothArms(rightPoses.rest, leftPoses.rest)); robot?.updateMatrixWorld(true);
  taskObjects.forEach((target) => { target.group.position.copy(target.core); target.group.quaternion.identity(); }); resetPhysicsWorld();
  ui.progress.value = 0; ui.progressValue.textContent = '0%'; ui.stage.textContent = '就绪'; ui.status.textContent = `R1 双臂垂直待命 · 已选择${activeLabel()}`; ui.armPose.textContent = '双臂先展开 → 躯干前倾下蹲 → 选定侧夹爪取放'; ui.width.textContent = '0.0 N'; ui.trajectory.textContent = '0.0 mrad'; ui.objectState.textContent = '0'; ui.start.disabled = false; ui.reset.disabled = true; ui.armSelect.disabled = false; $('#cancel-grasp').disabled = true; setStep(0);
}
function start() {
  if (isRunning || !robot) return;
  if (!mjPhysics) { toast('物理引擎仍在加载，请稍候再开始抓取'); return; }
  const target = taskObjects.get(selectedTarget); configureGraspPath(activeArm, target);
  object = target.group; objectStart = target.core.clone(); objectHome = target.drop.clone(); resetPhysicsWorld();
  isRunning = true; ui.start.disabled = true; ui.reset.disabled = true; ui.armSelect.disabled = true; $('#cancel-grasp').disabled = false; ui.objectState.textContent = '0'; ui.armPose.textContent = `${activeLabel()}执行中`;
  const started = performance.now(), duration = 9000;
  runEvidence = { startHeight: object.position.y, closeStable: 0, liftConfirmed: false, placeStable: 0, carrierActive: false, carrierAnchor: null, releaseAnchor: null, releaseAmount: 0, lastNow: started };
  const finish = (success, message) => {
    isRunning = false; ui.start.disabled = false; ui.reset.disabled = false; ui.armSelect.disabled = false; $('#cancel-grasp').disabled = true;
    graspValidated = success; ui.armPose.textContent = success ? '双臂垂直待命' : '抓取未完成，请复位后重试';
    ui.status.textContent = message; ui.stage.textContent = success ? '抓取完成' : '抓取未命中'; ui.trajectory.textContent = success ? '物理验证通过' : `${Math.round(lastContactDistance * 1000)} mm`;
    toast(message);
  };
  const tick = (now) => {
    if (!isRunning) return;
    const progress = Math.min(1, (now - started) / duration);
    const motion = armMotion(progress); const right = activeArm === 'right' ? motion.active : motion.inactive; const left = activeArm === 'left' ? motion.active : motion.inactive;
    apply(bothArms(right, left, activeArm === 'right' ? undefined : .05, activeArm === 'left' ? undefined : .05, torsoAt(progress))); updateGripper(progress); robot.updateMatrixWorld(true); getGripPose();
    // Advance enough 1 ms physics substeps that a moving pad can establish a
    // stable contact rather than tunnelling through the small task objects.
    const contact = mjPhysics ? stepPhysics(12) : fingerContactState(); const elapsed = Math.min(.05, Math.max(0, (now - runEvidence.lastNow) / 1000)); runEvidence.lastNow = now;
    lastContactCount = contact.count; lastContactDistance = contact.distance; lastForce = contact.force || 0;
    ui.objectState.textContent = String(contact.count); ui.width.textContent = `${lastForce.toFixed(1)} N`;
    // Reference-page rules: both fingers must be in contact before lift; a
    // successful lift needs 35 mm of real object motion; placing needs table
    // contact, position tolerance and a settled object.
    if (progress >= .59 && progress < .69 && contact.count === 2) runEvidence.closeStable += elapsed;
    if (progress >= .69 && runEvidence.closeStable < .06) { finish(false, '抓取失败：MuJoCo 未检测到两侧指尖同时接触目标'); return; }
    if (progress >= .69) runEvidence.carrierActive = true;
    if (progress >= .69 && object.position.y > runEvidence.startHeight + .035) runEvidence.liftConfirmed = true;
    if (progress >= .77 && !runEvidence.liftConfirmed) { finish(false, '抓取失败：目标未被实际抬离桌面 35 mm'); return; }
    // The color trays are 105 mm wide.  With the conservative URDF fingertip
    // proxies, accept a settled free object within the 40 mm inner tray area.
    if (progress >= .87 && progress < .94 && contact.table && contact.distance < .04 && contact.speed < .035) runEvidence.placeStable += elapsed;
    if (progress >= .94) {
      if (!runEvidence.releaseAnchor && runEvidence.carrierAnchor) runEvidence.releaseAnchor = runEvidence.carrierAnchor.clone();
      runEvidence.carrierActive = false; runEvidence.releaseAmount = THREE.MathUtils.clamp((progress - .94) / .04, 0, 1);
    }
    if (progress >= .94 && runEvidence.placeStable < .08) { finish(false, '放置失败：目标未在指定收纳区稳定落桌'); return; }
    const pct = Math.round(progress * 100); ui.progress.value = progress; ui.progressValue.textContent = `${pct}%`; setStage(progress); sampleTelemetry(now);
    if (progress < 1) animationFrame = requestAnimationFrame(tick);
    else finish(true, `${taskObjects.get(selectedTarget).label}已由${activeLabel()}物理验证后放入收纳区`);
  };
  animationFrame = requestAnimationFrame(tick);
}
function resize() { const rect = stage.getBoundingClientRect(); renderer.setSize(rect.width, rect.height, false); camera.aspect = rect.width / rect.height; camera.updateProjectionMatrix(); visionRenderers.forEach((view) => view.setSize(320, 240, false)); }
function render() { controls.update(); overheadCamera.lookAt(.35, .5, .1); if (robot) { getGripPose(); wristCamera.position.copy(gripperPoint).add(new THREE.Vector3(.08,.08,.08)); wristCamera.quaternion.copy(gripperQuat); wristCamera.rotateY(Math.PI); } renderer.render(scene, camera); visionRenderers[0].render(scene, overheadCamera); visionRenderers[1].render(scene, wristCamera); requestAnimationFrame(render); }

async function loadR1() {
  const loader = new URDFLoader(); loader.packages = { r1_v2_1_0: './assets' };
  try {
    ui.loadingText.textContent = '解析关节与 34 个本地网格…';
    robot = await loader.loadAsync('./assets/r1.urdf'); robot.name = 'R1 robot'; robot.rotation.x = -Math.PI / 2; robot.traverse((node) => { if (node.isMesh) { node.castShadow = true; node.receiveShadow = true; } }); scene.add(robot);
    // 固定落地工作台：R1 的 +X 为正前方，y 为竖直方向、z 为左右方向。
    // 四条腿由 createWorkcell() 一直落到 y=0；桌面和机器人底盘没有重叠。
    const taskTable = new THREE.Vector3(.78, .69, 0);
    // 蓝色长方块的重心严格对齐参考 MJCF 的 38 mm 高度与本地桌面高度。
    const objectCore = new THREE.Vector3(.78, .7015, 0);
    workcell = createWorkcell(); workcell.position.copy(taskTable); scene.add(workcell);
    const specs = [
      { id: 'red', label: '红色立方体', kind: 'cube', color: 0xeb140f, start: objectCore.clone().add(new THREE.Vector3(-.15, .0035, .19)), drop: new THREE.Vector3(.78, .705, .12), solutions: { right: { pick: { right_arm_joint1: -.514, right_arm_joint2: 2.262, right_arm_joint3: -1.474, right_arm_joint4: -1.448, right_arm_joint5: .214, right_arm_joint6: .076 }, place: { right_arm_joint1: -.212, right_arm_joint2: 3.180, right_arm_joint3: -2.385, right_arm_joint4: -.689, right_arm_joint5: -1.327, right_arm_joint6: .530 } } } },
      { id: 'blue', label: '蓝色长方块', kind: 'block', color: 0x076bf0, start: objectCore.clone(), drop: new THREE.Vector3(.78, .7015, 0) },
      { id: 'yellow', label: '黄色圆柱', kind: 'cylinder', color: 0xffb80a, start: objectCore.clone().add(new THREE.Vector3(0, .007, -.12)), drop: new THREE.Vector3(.78, .7085, -.12), solutions: { left: { pick: { left_arm_joint1: .379, left_arm_joint2: 3.180, left_arm_joint3: -2.414, left_arm_joint4: 1.428, left_arm_joint5: 1.592, left_arm_joint6: 2.427 }, place: { left_arm_joint1: .379, left_arm_joint2: 3.180, left_arm_joint3: -2.186, left_arm_joint4: 1.428, left_arm_joint5: 1.509, left_arm_joint6: 2.664 } } } },
    ];
    specs.forEach((spec) => {
      const group = addReferenceObject(spec.start, spec);
      taskObjects.set(spec.id, { ...spec, group, core: spec.start.clone() });
      targetZones.set(spec.id, addZone(new THREE.Vector3(spec.drop.x, .681, spec.drop.z), spec.color, spec.label));
    });
    document.querySelector('.vision-targets').innerHTML = specs.map(({ id, label }) => `<button class="vision-target${id === selectedTarget ? ' active' : ''}" type="button" data-target="${id}" aria-pressed="${id === selectedTarget}"><i class="target-${id}"></i><span>${label}</span></button>`).join('');
    ui.targetButtons = [...document.querySelectorAll('.vision-target')]; ui.targetButtons.forEach((button) => button.addEventListener('click', () => setTarget(button.dataset.target)));
    const initiallySelected = taskObjects.get(selectedTarget);
    configureGraspPath('right', initiallySelected); configureGraspPath('left', initiallySelected);
    object = initiallySelected.group; objectStart = initiallySelected.core.clone(); objectHome = initiallySelected.drop.clone();
    markSelectedObject(); $('#object-edit-selection').textContent = `已选择：${taskObjects.get(selectedTarget).label}（可自由拖动）`;
    apply(bothArms(rightPoses.rest, leftPoses.rest)); robot.updateMatrixWorld(true);
    ui.loadingText.textContent = '启动 MuJoCo 接触与受力引擎…';
    await physicsInitPromise;
    if (!mjPhysics) { ui.loadingText.textContent = 'MuJoCo 初始化失败，请刷新页面重试'; ui.status.textContent = '物理引擎不可用'; return; }
    controls.target.set(.02, .66, .02); controls.update(); reset(); ui.loading.classList.add('is-hidden');
  } catch (error) {
    console.error(error); ui.loadingText.textContent = 'R1 模型加载失败，请刷新页面重试'; ui.status.textContent = '模型读取失败';
  }
}

function chooseArm(arm) {
  if (isRunning || arm === activeArm) return;
  activeArm = arm;
  if (robot && object) {
    // 工作台不跟随选臂移动；只为新执行臂重新解一次到当前目标的位置。
    const current = taskObjects.get(selectedTarget); configureGraspPath(arm, current);
    objectStart = current.core.clone(); objectHome = current.drop.clone();
    reset(); toast(`已选择 R1 ${activeLabel()}执行抓取`);
  }
}

function pointerToObjectRay(event) {
  const rect = canvas.getBoundingClientRect();
  objectPointer.set((event.clientX - rect.left) / rect.width * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
  objectRaycaster.setFromCamera(objectPointer, camera);
}
function taskTargetFromIntersection(intersections) {
  for (const hit of intersections) {
    let node = hit.object;
    while (node) { if (node.userData.taskId) return taskObjects.get(node.userData.taskId); node = node.parent; }
  }
  return null;
}
function onObjectPointerDown(event) {
  if (isRunning || event.button !== 0 || !taskObjects.size) return;
  pointerToObjectRay(event);
  const target = taskTargetFromIntersection(objectRaycaster.intersectObjects([...taskObjects.values()].map((item) => item.group), true));
  if (!target) return;
  if (target.id !== selectedTarget) setTarget(target.id);
  objectDrag = { pointerId: event.pointerId, target };
  controls.enabled = false; canvas.setPointerCapture(event.pointerId); stage.classList.add('object-dragging');
  event.preventDefault(); event.stopImmediatePropagation();
}
function onObjectPointerMove(event) {
  if (!objectDrag || event.pointerId !== objectDrag.pointerId) return;
  pointerToObjectRay(event);
  if (objectRaycaster.ray.intersectPlane(objectDragPlane, objectDragPoint)) updateDraggedTarget(objectDrag.target, objectDragPoint);
  event.preventDefault();
}
function finishObjectDrag(event) {
  if (!objectDrag || (event && event.pointerId !== objectDrag.pointerId)) return;
  const target = objectDrag.target; objectDrag = null; controls.enabled = true; stage.classList.remove('object-dragging');
  if (event && canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  configureGraspPath(activeArm, target); reset(); toast(`${target.label}已移动；点击开始抓取即可按新位置执行`);
}
function recoverOrbitControls() {
  if (objectDrag) finishObjectDrag();
  controls.enabled = true; stage.classList.remove('object-dragging');
}

setChineseLabels();
ui.start.addEventListener('click', start); ui.reset.addEventListener('click', reset); ui.armSelect.addEventListener('change', () => chooseArm(ui.armSelect.value));
$('#cancel-grasp').addEventListener('click', () => { if (!isRunning) return; cancelAnimationFrame(animationFrame); isRunning = false; ui.start.disabled = false; ui.reset.disabled = false; ui.armSelect.disabled = false; $('#cancel-grasp').disabled = true; ui.stage.textContent = '已停止'; ui.status.textContent = '抓取已停止；物体保留当前 MuJoCo 物理位置'; });
$('#gripper-open').addEventListener('click', () => { if (isRunning || !robot) return; apply(armGripper(activeArm, .05)); robot.updateMatrixWorld(true); stepPhysics(1); toast(`${activeLabel()}夹爪已张开`); });
$('#gripper-close').addEventListener('click', () => { if (isRunning || !robot) return; apply(armGripper(activeArm, .003)); robot.updateMatrixWorld(true); const contact = stepPhysics(3); ui.objectState.textContent = String(contact.count); ui.width.textContent = `${(contact.force || 0).toFixed(1)} N`; toast(contact.count === 2 ? 'MuJoCo：两侧指尖已接触目标' : 'MuJoCo：尚未形成双侧夹持'); });
ui.cameraToggle.addEventListener('click', () => { const hidden = ui.cameraGrid.hidden = !ui.cameraGrid.hidden; ui.cameraToggle.textContent = hidden ? '显示画面' : '隐藏画面'; ui.cameraToggle.setAttribute('aria-checked', String(!hidden)); });
ui.contactToggle.addEventListener('click', () => { const hidden = ui.contactToggle.getAttribute('aria-checked') === 'true'; ui.contactToggle.setAttribute('aria-checked', String(!hidden)); ui.contactToggle.textContent = hidden ? '显示接触点' : '隐藏接触点'; if (hidden) clearContactDebug(); else drawPhysicsContacts(); });
// 捕获阶段先判断是否点到任务物，避免 OrbitControls 抢走物体拖动手势。
canvas.addEventListener('pointerdown', onObjectPointerDown, { capture: true }); canvas.addEventListener('pointermove', onObjectPointerMove); canvas.addEventListener('pointerup', finishObjectDrag); canvas.addEventListener('pointercancel', finishObjectDrag);
canvas.addEventListener('lostpointercapture', recoverOrbitControls); window.addEventListener('pointerup', finishObjectDrag); window.addEventListener('blur', recoverOrbitControls);
window.addEventListener('resize', resize); resize(); render();
physicsInitPromise = initMuJoCoPhysics().catch((error) => { console.error('R1 MuJoCo 初始化失败：', error); ui.loadingText.textContent = 'MuJoCo 初始化失败，请刷新页面重试'; return null; });
loadR1();
