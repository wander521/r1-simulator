import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { STLLoader } from './vendor/STLLoader.js';

const $ = (s) => document.querySelector(s), $$ = (s) => [...document.querySelectorAll(s)];
const canvas = $('#main-view'), stage = $('.stage'), raycaster = new THREE.Raycaster(), pointer = new THREE.Vector2();
const dragPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), dragPoint = new THREE.Vector3(), up = new THREE.Vector3(0, 0, 1);
const targetInfo = {
  red: { label: '红色方块', color: 0xef5260, confidence: '98.7%', initial: [0.42, -0.24, 0.145], home: [0.56, -0.26, 0.145], size: .045 },
  blue: { label: '蓝色方块', color: 0x45aef2, confidence: '97.9%', initial: [0.37, 0.13, 0.138], home: [0.56, 0, 0.138], size: .052 },
  yellow: { label: '黄色圆柱', color: 0xe6d957, confidence: '99.1%', initial: [0.24, 0.28, 0.151], home: [0.56, 0.25, 0.151], size: .042 },
};
const els = { loading: $('#loading'), loadingText: $('#loading-text'), status: $('#status-text'), toast: $('#grasp-toast'), visionToggle: $('#vision-toggle'), feeds: $('#camera-feeds'), overlay: $('#vision-overlay'), start: $('#start-grasp'), stack: $('#start-stack'), reset: $('#reset'), progress: $('#grasp-progress'), progressValue: $('#progress-value'), stage: $('#grasp-stage'), confidence: $('#confidence'), force: $('#grip-force'), error: $('#trajectory-error'), dragState: $('#drag-state'), process: $$('.process-step') };

const scene = new THREE.Scene(); scene.fog = new THREE.FogExp2(0x234a67, .48);
const camera = new THREE.PerspectiveCamera(42, 1, .01, 10); camera.up.copy(up); camera.position.set(.92, -1.12, .68);
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true }); renderer.setClearColor(0x2d6183, 1); renderer.outputColorSpace = THREE.SRGBColorSpace; renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const controls = new OrbitControls(camera, canvas); controls.target.set(.22, 0, .23); controls.minDistance = .55; controls.maxDistance = 1.7; controls.maxPolarAngle = Math.PI * .48; controls.update();
const overheadCamera = new THREE.PerspectiveCamera(38, 4 / 3, .01, 10); overheadCamera.up.copy(up); overheadCamera.position.set(.38, -.02, 1.32); overheadCamera.lookAt(.38, 0, .08);
const wristCamera = new THREE.PerspectiveCamera(48, 4 / 3, .01, 10); wristCamera.up.copy(up);
const visionRenderers = [new THREE.WebGLRenderer({ canvas: $('#overhead-view'), antialias: true, alpha: false }), new THREE.WebGLRenderer({ canvas: $('#wrist-view'), antialias: true, alpha: false })];
visionRenderers.forEach((view) => { view.setPixelRatio(1); view.setClearColor(0x020817, 1); view.outputColorSpace = THREE.SRGBColorSpace; });
scene.add(new THREE.HemisphereLight(0x9bdcff, 0x0a223b, 3.8));
const keyLight = new THREE.DirectionalLight(0xe9faff, 5.2); keyLight.position.set(.25, -.5, 1.2); keyLight.castShadow = true; keyLight.shadow.mapSize.set(1024, 1024); scene.add(keyLight);
const fillLight = new THREE.PointLight(0xb3ecff, 18, 2.4); fillLight.position.set(.82, -.72, .76); scene.add(fillLight);
const rimLight = new THREE.PointLight(0x25bbff, 20, 2.1); rimLight.position.set(-.45, .3, .75); scene.add(rimLight);

function addWorkcell() {
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(2.4, 2.4), new THREE.MeshStandardMaterial({ color: 0x3a6b88, metalness: .12, roughness: .76 })); floor.receiveShadow = true; scene.add(floor);
  const grid = new THREE.GridHelper(1.35, 14, 0xaeeaff, 0x467994); grid.position.set(.36, 0, .006); grid.material.opacity = .68; grid.material.transparent = true; scene.add(grid);
  const table = new THREE.Group(), top = new THREE.Mesh(new THREE.BoxGeometry(.74, 1.04, .055), new THREE.MeshStandardMaterial({ color: 0x7c98a8, metalness: .52, roughness: .31 })); top.position.z = .055; top.castShadow = true; top.receiveShadow = true; table.add(top);
  const inset = new THREE.Mesh(new THREE.BoxGeometry(.67, .97, .009), new THREE.MeshStandardMaterial({ color: 0x214861, metalness: .18, roughness: .4 })); inset.position.z = .087; table.add(inset); table.position.set(.34, 0, 0); scene.add(table);
  Object.entries(targetInfo).forEach(([key, info]) => { const zone = new THREE.Mesh(new THREE.BoxGeometry(.11, .11, .025), new THREE.MeshStandardMaterial({ color: info.color, transparent: true, opacity: .42, metalness: .42, roughness: .25 })); zone.position.set(info.home[0], info.home[1], .106); zone.receiveShadow = true; scene.add(zone); const edge = new THREE.LineSegments(new THREE.EdgesGeometry(zone.geometry), new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: .65 })); edge.position.copy(zone.position); scene.add(edge); });
}
addWorkcell();
function material(color, roughness = .3) { return new THREE.MeshStandardMaterial({ color, metalness: .18, roughness }); }
function addEdges(group, mesh, color) { const edge = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry), new THREE.LineBasicMaterial({ color, transparent: true, opacity: .5 })); edge.position.copy(mesh.position); edge.rotation.copy(mesh.rotation); group.add(edge); }
function createObject(key) {
  const info = targetInfo[key], group = new THREE.Group(); group.userData.targetKey = key; group.userData.radius = info.size; let body;
  if (key === 'yellow') { body = new THREE.Mesh(new THREE.CylinderGeometry(.032, .032, .062, 32), material(info.color, .23)); body.castShadow = true; body.receiveShadow = true; group.add(body); const band = new THREE.Mesh(new THREE.TorusGeometry(.033, .003, 8, 28), material(0xfff6a5, .18)); band.rotation.x = Math.PI / 2; band.position.z = .018; group.add(band); }
  else { const size = key === 'blue' ? [.065, .042, .052] : [.05, .05, .05]; body = new THREE.Mesh(new THREE.BoxGeometry(...size, 3, 3, 3), material(info.color, .25)); body.castShadow = true; body.receiveShadow = true; group.add(body); addEdges(group, body, 0xe6fbff); const cap = new THREE.Mesh(new THREE.BoxGeometry(size[0] * .65, size[1] * .65, .004), material(0xffffff, .42)); cap.position.z = size[2] / 2 + .003; group.add(cap); }
  const halo = new THREE.Mesh(new THREE.RingGeometry(info.size * 1.08, info.size * 1.2, 32), new THREE.MeshBasicMaterial({ color: 0x56f1df, transparent: true, opacity: 0, side: THREE.DoubleSide })); halo.position.z = .006; group.add(halo); group.userData.halo = halo; group.position.set(...info.initial); scene.add(group); return group;
}
const objects = Object.fromEntries(Object.keys(targetInfo).map((key) => [key, createObject(key)]));
let robot, selected = 'red', running = false, visionOn = true, dragging = null, toastTimer, animationId;
let gripperTip = null, heldObject = null, heldOffset = new THREE.Vector3(), runMode = 'single', stackIndex = 0;
const robotJoints = {};
const homePose = { joint1: 0, joint2: 0, joint3: 0, joint4: 0, joint5: 0, joint6: 0 };
function setJoint(name, value) { robotJoints[name]?.set(value); }
function setPose(values) { Object.entries(values).forEach(([name, value]) => setJoint(name, value)); }
function setGripper(width) { setJoint('gripper_joint1', width); setJoint('gripper_joint2', width); }
function updateSelection() { Object.entries(objects).forEach(([key, object]) => { object.userData.halo.material.opacity = key === selected ? .72 : 0; }); $$('.target-choice').forEach((button) => { const active = button.dataset.target === selected; button.classList.toggle('active', active); button.setAttribute('aria-pressed', active); }); const info = targetInfo[selected]; els.confidence.textContent = info.confidence; els.status.textContent = `${info.label} 已锁定 · 可直接拖动调整位置`; }
function selectTarget(key) { if (!running) { selected = key; updateSelection(); } }
function toast(message) { clearTimeout(toastTimer); els.toast.textContent = message; els.toast.classList.add('show'); toastTimer = setTimeout(() => els.toast.classList.remove('show'), 2200); }
function setStage(progress, itemLabel = targetInfo[selected].label, stack = false) { let label = stack ? `叠叠乐 · ${itemLabel}` : '视觉定位', status = `${itemLabel} 已锁定`, step = 0; if (progress >= 20 && progress < 48) { label = stack ? '轨迹规划 · 对准下一层' : '轨迹规划'; status = '正在计算关节无碰撞路径'; step = 1; } if (progress >= 48 && progress < 67) { label = stack ? '夹爪闭合 · 接触中' : '夹爪闭合 · 接触中'; status = '夹爪已贴合物体，正在建立夹持力'; step = 2; } if (progress >= 67 && progress < 100) { label = stack ? '搬运至堆叠位' : '搬运至收纳区'; status = '物体已挂接末端，正在平稳搬运'; step = 2; } if (progress === 100) { label = stack ? '本层完成' : '任务完成'; status = stack ? `${itemLabel} 已稳固放置` : `${itemLabel} 已放入目标收纳格`; step = 2; } els.stage.textContent = label; els.status.textContent = status; els.process.forEach((item, index) => item.classList.toggle('is-active', index <= step)); }
function mixPose(a, b, t) { const smooth = t * t * (3 - 2 * t); return Object.fromEntries(Object.keys(homePose).map((key) => [key, (a[key] ?? 0) + ((b[key] ?? 0) - (a[key] ?? 0)) * smooth])); }
const stackTargets = { blue: [.56, 0, .138], red: [.56, 0, .193], yellow: [.56, 0, .248] };
const armJointNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
const armJointLimits = { joint1: [-2.8, 2.8], joint2: [0, 3.14], joint3: [0, 3.14], joint4: [-1.57, 1.57], joint5: [-1.57, 1.57], joint6: [-3.14, 3.14] };
function clampJoint(name, value) { const [min, max] = armJointLimits[name]; return THREE.MathUtils.clamp(value, min, max); }
function tipPosition(pose) { setPose(pose); scene.updateMatrixWorld(true); return gripperTip.getWorldPosition(new THREE.Vector3()); }
function solveGripperPose(target, seed = homePose) {
  const result = { ...seed }, eps = .014, probe = new THREE.Vector3();
  for (let iteration = 0; iteration < 48; iteration += 1) {
    if (tipPosition(result).distanceTo(target) < .004) break;
    armJointNames.forEach((name) => {
      const here = tipPosition(result), error = target.clone().sub(here), base = result[name], stepped = clampJoint(name, base + eps); result[name] = stepped;
      const after = tipPosition(result); probe.copy(after).sub(here).multiplyScalar(1 / Math.max(.0001, stepped - base));
      result[name] = clampJoint(name, base + THREE.MathUtils.clamp(probe.dot(error) * 1.45, -.12, .12));
    });
  }
  return result;
}
function physicalStage(phase, itemLabel, stack) {
  let label = '安全高位', status = '机械臂先收拢到安全姿态', step = 0;
  if (phase >= .13 && phase < .23) { label = '开爪'; status = '两指张开，准备接近目标'; step = 1; }
  if (phase >= .23 && phase < .42) { label = '接近目标'; status = '夹爪保持张开并在目标上方对准'; step = 1; }
  if (phase >= .42 && phase < .54) { label = '下降'; status = '夹爪正下降到物体两侧'; step = 1; }
  if (phase >= .54 && phase < .64) { label = '闭合夹爪'; status = '两指接触物体后闭合夹持'; step = 2; }
  if (phase >= .64 && phase < .72) { label = '抬升'; status = '已夹持目标，正在离开桌面'; step = 2; }
  if (phase >= .72 && phase < .84) { label = stack ? '移向叠放位' : '移向收纳区'; status = '物体随夹爪末端同步移动'; step = 2; }
  if (phase >= .84 && phase < .91) { label = '下降放置'; status = '正将物体平稳放入目标位置'; step = 2; }
  if (phase >= .91 && phase < .95) { label = '松开目标'; status = '夹爪张开，物体保持在放置位'; step = 2; }
  if (phase >= .95) { label = '安全撤离'; status = '夹爪正在抬离目标'; step = 2; }
  els.stage.textContent = label; els.status.textContent = status; els.process.forEach((item, index) => item.classList.toggle('is-active', index <= step));
}
function finishRun(label, stack = false) { running = false; heldObject = null; els.start.disabled = false; els.stack.disabled = false; els.reset.disabled = false; els.start.innerHTML = '<span>↻</span> 再次演示'; els.stack.innerHTML = '<span>▲</span> 叠叠乐'; els.dragState.textContent = '物体拖动已重新启用'; toast(stack ? `${label} 叠叠乐演示完成` : `${label} 已完成抓取并归位`); }
function runSequence(sequence, stack = false) { if (running || !robot || !gripperTip) return; running = true; runMode = stack ? 'stack' : 'single'; stackIndex = 0; els.start.disabled = true; els.stack.disabled = true; els.reset.disabled = true; els.start.innerHTML = '<span>◌</span> 执行中'; els.stack.innerHTML = '<span>◌</span> 堆叠中'; els.dragState.textContent = stack ? '叠叠乐执行中，依次接触、夹持、叠放' : '抓取执行中，物体会保持在桌面直至两指闭合'; const next = () => { if (stackIndex >= sequence.length) { els.progress.value = 100; els.progressValue.textContent = '100%'; setStage(100, '三块积木', true); finishRun('三块积木', true); return; } const key = sequence[stackIndex], object = objects[key], destination = stack ? stackTargets[key] : targetInfo[key].home, home = new THREE.Vector3(...destination), label = targetInfo[key].label; selected = key; updateSelection(); const contact = object.position.clone(), high = contact.clone().add(new THREE.Vector3(0, 0, .14)), place = home.clone(), placeHigh = place.clone().add(new THREE.Vector3(0, 0, .14)); const safe = solveGripperPose(new THREE.Vector3(.30, 0, .34), homePose), pickHigh = solveGripperPose(high, safe), pick = solveGripperPose(contact, pickHigh), dropHigh = solveGripperPose(placeHigh, pick), drop = solveGripperPose(place, dropHigh); setPose(homePose); const startedAt = performance.now(), duration = stack ? 6100 : 9000; heldObject = null; const tipWorld = new THREE.Vector3(); const poseAt = (phase) => { if (phase < .13) return mixPose(homePose, safe, phase / .13); if (phase < .23) return safe; if (phase < .42) return mixPose(safe, pickHigh, (phase - .23) / .19); if (phase < .54) return mixPose(pickHigh, pick, (phase - .42) / .12); if (phase < .64) return pick; if (phase < .72) return mixPose(pick, pickHigh, (phase - .64) / .08); if (phase < .84) return mixPose(pickHigh, dropHigh, (phase - .72) / .12); if (phase < .91) return mixPose(dropHigh, drop, (phase - .84) / .07); if (phase < .95) return drop; return mixPose(drop, safe, (phase - .95) / .05); }; const tick = (now) => { const local = Math.min(1, (now - startedAt) / duration), progress = stack ? Math.round(((stackIndex + local) / sequence.length) * 100) : Math.round(local * 100); setPose(poseAt(local)); const closeAmount = local < .54 ? .05 : local < .64 ? THREE.MathUtils.lerp(.05, .002, (local - .54) / .10) : local < .91 ? .002 : THREE.MathUtils.lerp(.002, .05, (local - .91) / .04); setGripper(closeAmount); scene.updateMatrixWorld(true); gripperTip.getWorldPosition(tipWorld); if (local >= .64 && local < .91 && !heldObject && tipWorld.distanceTo(object.position) < .035) { heldObject = object; heldOffset.copy(object.position).sub(tipWorld); } if (heldObject) object.position.copy(tipWorld).add(heldOffset); if (local >= .91) { heldObject = null; object.position.copy(home); } els.progress.value = progress; els.progressValue.textContent = `${progress}%`; els.force.textContent = local >= .54 && local < .91 ? '18.4 N' : local >= .91 ? '4.2 N' : '0.0 N'; els.error.textContent = local < .54 ? `${Math.max(.2, 2.0 - local * 3.6).toFixed(1)} mm` : '.2 mm'; physicalStage(local, label, stack); if (local < 1) animationId = requestAnimationFrame(tick); else { stackIndex += 1; if (stack) setTimeout(next, 320); else finishRun(label, false); } }; animationId = requestAnimationFrame(tick); }; next(); }
function runDemo() { runSequence([selected], false); }
function runStack() { runSequence(['blue', 'red', 'yellow'], true); }
function resetDemo() { cancelAnimationFrame(animationId); running = false; heldObject = null; stackIndex = 0; Object.entries(objects).forEach(([key, object]) => object.position.set(...targetInfo[key].initial)); setPose(homePose); setGripper(.05); els.progress.value = 0; els.progressValue.textContent = '0%'; els.stage.textContent = '准备就绪'; els.status.textContent = '系统就绪 · 可调整物体位置'; els.force.textContent = '0.0 N'; els.error.textContent = '0.0 mm'; els.start.innerHTML = '<span>▶</span> 开始抓取'; els.stack.innerHTML = '<span>▲</span> 叠叠乐'; els.start.disabled = false; els.stack.disabled = false; els.reset.disabled = true; els.dragState.textContent = '物体拖动已启用'; els.process.forEach((item, index) => item.classList.toggle('is-active', index === 0)); updateSelection(); }
function pointerPosition(event) { const rect = canvas.getBoundingClientRect(); pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1; pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1; raycaster.setFromCamera(pointer, camera); }
function targetFromHit(hit) { let node = hit.object; while (node && !node.userData.targetKey) node = node.parent; return node; }
canvas.addEventListener('pointerdown', (event) => { if (running) return; pointerPosition(event); const hit = raycaster.intersectObjects(Object.values(objects), true)[0], object = hit && targetFromHit(hit); if (!object) return; dragging = object; selectTarget(object.userData.targetKey); dragPlane.constant = -object.position.z; controls.enabled = false; canvas.setPointerCapture(event.pointerId); canvas.style.cursor = 'grabbing'; els.dragState.textContent = `正在移动${targetInfo[selected].label}`; });
canvas.addEventListener('pointermove', (event) => { if (!dragging) { pointerPosition(event); const hit = raycaster.intersectObjects(Object.values(objects), true)[0]; canvas.style.cursor = hit ? 'grab' : 'default'; return; } pointerPosition(event); if (!raycaster.ray.intersectPlane(dragPlane, dragPoint)) return; const candidate = new THREE.Vector3(THREE.MathUtils.clamp(dragPoint.x, .10, .62), THREE.MathUtils.clamp(dragPoint.y, -.43, .43), dragging.position.z), blocked = Object.values(objects).some((object) => object !== dragging && object.position.distanceTo(candidate) < object.userData.radius + dragging.userData.radius + .025); if (!blocked) dragging.position.copy(candidate); });
function stopDrag() { if (!dragging) return; dragging = null; controls.enabled = true; canvas.style.cursor = 'default'; els.dragState.textContent = '物体拖动已启用'; }
canvas.addEventListener('pointerup', stopDrag); canvas.addEventListener('pointercancel', stopDrag); canvas.addEventListener('pointerleave', (event) => { if (event.buttons === 0) stopDrag(); });
function resize() { const rect = stage.getBoundingClientRect(); renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2)); renderer.setSize(rect.width, rect.height, false); camera.aspect = rect.width / rect.height; camera.updateProjectionMatrix(); visionRenderers.forEach((view) => view.setSize(160, 120, false)); }
function render() { controls.update(); const focused = objects[selected].position; wristCamera.position.set(.18, -.18, .42); wristCamera.lookAt(focused.x, focused.y, focused.z + .02); renderer.render(scene, camera); if (visionOn) { visionRenderers[0].render(scene, overheadCamera); visionRenderers[1].render(scene, wristCamera); } requestAnimationFrame(render); }
function originGroup(position, rpy = [0, 0, 0], axis = null) { const origin = new THREE.Group(); origin.position.set(...position); origin.rotation.set(...rpy); const moving = new THREE.Group(); origin.add(moving); return { origin, moving, axis, set(value) { if (axis) moving.quaternion.setFromAxisAngle(axis, value); else moving.position.z = value; } }; }
function meshMaterial(name) { const colors = { base_link: 0x395a60, link1: 0xc5dddd, link2: 0x4c727a, link3: 0xd0e1df, link4: 0x4c727a, link5: 0xc9d9d8, link6: 0x4c727a, motor_7: 0x141d1e, cnc7: 0xc7d6d5, pla7_green: 0xb8d539, pla_left: 0x253234, cnc_left: 0xc7d6d5, pla_right: 0x253234, cnc_right: 0xc7d6d5 }; return new THREE.MeshStandardMaterial({ color: colors[name] ?? 0x9bb9bc, emissive: 0x16343b, emissiveIntensity: .22, metalness: .38, roughness: .37, envMapIntensity: 1.3 }); }
function loadRobot() {
  const loader = new STLLoader(); const meshRoot = './assets/robot/meshes/shared/';
  const nodes = { base_link: new THREE.Group() };
  const jointSpecs = [
    ['joint1', 'base_link', 'link1', [-.00034283, -.00098683, .075], [0, 0, 0], new THREE.Vector3(0, 0, -1)],
    ['joint2', 'link1', 'link2', [.020343, .027237, .07], [-1.5708, 0, 0], new THREE.Vector3(0, 0, 1)],
    ['joint3', 'link2', 'link3', [-.236, 0, 0], [0, 0, 0], new THREE.Vector3(0, 0, -1)],
    ['joint4', 'link3', 'link4', [.228, -.072746, .0045], [0, 0, 0], new THREE.Vector3(0, 0, -1)],
    ['joint5', 'link4', 'link5', [.087, -.048, -.03075], [-1.5708, 0, 0], new THREE.Vector3(0, 0, -1)],
    ['joint6', 'link5', 'link6', [.0365, 0, .048], [0, 1.5708, 0], new THREE.Vector3(0, 0, -1)],
    ['j_gripper_end', 'link6', 'gripper_end', [0, 0, .16621], [3.1416, -1.5708, 0], null],
    ['gripper_joint1', 'gripper_end', 'gripper_left', [-.041939, -.000073385, 0], [1.5708, -1.5708, 0], null],
    ['gripper_joint2', 'gripper_end', 'gripper_right', [-.041939, .000073385, 0], [-1.5708, -1.5708, 0], null],
  ];
  jointSpecs.forEach(([name, parent, child, position, rpy, axis]) => { nodes[child] = new THREE.Group(); const joint = originGroup(position, rpy, axis); nodes[parent].add(joint.origin); joint.moving.add(nodes[child]); if (name !== 'j_gripper_end') robotJoints[name] = joint; });
  gripperTip = new THREE.Object3D(); gripperTip.name = 'Gripper TCP'; nodes.gripper_end.add(gripperTip);
  robot = nodes.base_link; robot.name = 'ReBot RS'; robot.position.set(.05, 0, .094); robot.rotation.z = .08; robot.scale.setScalar(.86); scene.add(robot);
  const parts = [
    ['base_link', 'base_link'], ['link1', 'link1'], ['link2', 'link2'], ['link3', 'link3'], ['link4', 'link4'], ['link5', 'link5'], ['link6', 'link6'],
    ['motor_7', 'gripper_end'], ['cnc7', 'gripper_end'], ['pla7_green', 'gripper_end'], ['pla_left', 'gripper_left'], ['cnc_left', 'gripper_left'], ['pla_right', 'gripper_right'], ['cnc_right', 'gripper_right'],
  ]; let loaded = 0;
  (async () => { try { for (const [part, node] of parts) { const geometry = await loader.loadAsync(`${meshRoot}${part}.STL`); geometry.computeVertexNormals(); const mesh = new THREE.Mesh(geometry, meshMaterial(part)); mesh.castShadow = true; mesh.receiveShadow = true; nodes[node].add(mesh); loaded += 1; els.loadingText.textContent = `正在读取真实网格 · ${loaded} / ${parts.length}`; } setPose(homePose); setGripper(.05); els.loading.classList.add('is-hidden'); els.status.textContent = 'ReBot RS 模型已就绪 · 可调整物体位置'; updateSelection(); } catch { els.loadingText.textContent = '模型读取失败，请刷新页面重试'; els.status.textContent = '模型加载失败'; } })();
}
$$('.target-choice').forEach((button) => button.addEventListener('click', () => selectTarget(button.dataset.target))); els.start.addEventListener('click', runDemo); els.stack.addEventListener('click', runStack); els.reset.addEventListener('click', resetDemo);
els.visionToggle.addEventListener('click', () => { visionOn = !visionOn; els.visionToggle.setAttribute('aria-checked', String(visionOn)); els.visionToggle.textContent = visionOn ? '双相机开启' : '双相机关闭'; els.feeds.classList.toggle('is-hidden', !visionOn); els.overlay?.classList.toggle('is-hidden', !visionOn); toast(visionOn ? '视觉模式已开启：顶视与腕部相机实时渲染' : '视觉模式已暂停'); });
window.addEventListener('resize', resize); resize(); updateSelection(); loadRobot(); render();
