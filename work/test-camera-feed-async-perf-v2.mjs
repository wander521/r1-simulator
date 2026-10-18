import assert from 'node:assert/strict';
import {readFile, mkdir, writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {performance} from 'node:perf_hooks';
import * as THREE from '../dist/vendor/three.module.min.js';
import {createCameraFeedPass} from '../dist/multi/visuals/camera-feeds.mjs';

// This is a declared renderer contract fixture, not a WebGL implementation.
// Controlled promises let it test asynchronous readback lifetime and immediate
// state restoration without claiming GPU execution or a measured FPS benefit.
const root = new URL('../', import.meta.url), args = process.argv.slice(2);
const reportPath = args.find(value => value.startsWith('--report='))?.slice(9)
  ?? 'work/validation/performance-v2/camera-feed-async-contracts.json';
const sourcePaths = [
  'work/test-camera-feed-async-perf-v2.mjs', 'work/test-multi-camera-feeds.mjs',
  'dist/multi/visuals/camera-feeds.mjs', 'dist/vendor/three.module.min.js',
  'dist/vendor/three.core.min.js',
];
const sha = value => createHash('sha256').update(value).digest('hex');
const hashes = async () => Object.fromEntries(await Promise.all(sourcePaths.map(async path =>
  [path, sha(await readFile(new URL(path, root)))])));
const sourceHashes = await hashes(), startedAt = new Date().toISOString(), started = performance.now();
const results = [], views = new Set(), renderers = new Set();
const toneChunk = THREE.ShaderChunk.tonemapping_pars_fragment;
const colorChunk = THREE.ShaderChunk.colorspace_pars_fragment;

// Independent CPU evaluation of the actual bundled GLSL coefficients. This
// tests buffer identity/color-pipeline contracts, not shader precision or GPU
// pixels; the native-browser before/after recording is a separate requirement.
function matrix(name) {
  const body = toneChunk.match(new RegExp(`const mat3 ${name} = mat3\\(([\\s\\S]*?)\\);`))?.[1];
  assert(body, `Bundled ACES matrix missing: ${name}`);
  const values = [...body.matchAll(/vec3\(([^)]+)\)/g)].flatMap(match => match[1].split(',').map(Number));
  assert.equal(values.length, 9); assert(values.every(Number.isFinite)); return values;
}
const inputMatrix = matrix('ACESInputMat'), outputMatrix = matrix('ACESOutputMat');
const multiply = (m, vector) => [0, 1, 2].map(row =>
  m[row] * vector[0] + m[row + 3] * vector[1] + m[row + 6] * vector[2]);
function displayBytes(linear, exposure, clipping) {
  let value = linear.slice(0, 3).map(channel =>
    (clipping ? Math.max(0, Math.min(1, channel)) : channel) * exposure / .6);
  value = multiply(inputMatrix, value).map(v =>
    (v * (v + .0245786) - .000090537) / (v * (.983729 * v + .4329510) + .238081));
  value = multiply(outputMatrix, value).map(v => Math.max(0, Math.min(1, v)));
  return [...value.map(v => Math.round((v <= .0031308 ? v * 12.92 : Math.pow(v, .41666) * 1.055 - .055) * 255)),
    Math.round(linear[3] * 255)];
}

class RendererContractFixture {
  constructor({floatColorBuffer = true, target = true, asyncReadback = true} = {}) {
    this.capabilityQueries = 0;
    this.extensions = {has: name => {assert.equal(name, 'EXT_color_buffer_float'); this.capabilityQueries++; return floatColorBuffer;}};
    this.capabilities = {maxTextureSize: 4096};
    this.toneMapping = THREE.ACESFilmicToneMapping; this.toneMappingExposure = 1;
    this.outputColorSpace = THREE.SRGBColorSpace;
    this.clearColor = new THREE.Color(.02, .04, .08); this.clearAlpha = .6;
    this.activeClearColor = this.clearColor.clone(); this.activeClearAlpha = this.clearAlpha;
    this.autoClear = true; this.autoClearColor = false; this.autoClearDepth = true; this.autoClearStencil = false;
    this.xr = {enabled: true}; this.shadowMap = {enabled: true, autoUpdate: true, needsUpdate: true};
    this.pixelRatio = 2; this.viewport = new THREE.Vector4(7, 9, 640, 360);
    this.scissor = new THREE.Vector4(11, 13, 307, 201); this.scissorTest = true;
    this.target = target ? new THREE.WebGLRenderTarget(53, 37) : null;
    if (this.target) {this.target.viewport.set(1, 2, 49, 31); this.target.scissor.set(3, 4, 41, 29); this.target.scissorTest = true;}
    this.currentViewport = this.target ? new THREE.Vector4(5, 6, 23, 17) : this.viewport.clone().multiplyScalar(2);
    this.cubeFace = this.target ? 4 : 0; this.mipLevel = this.target ? 1 : 0;
    this.draws = []; this.reads = []; this.clears = []; this.pendingReads = []; this.targetChanges = [];
    this.sourceScene = new THREE.Scene(); this.sourceCamera = new THREE.PerspectiveCamera();
    this.sampleLinearRGBA = [.18, .18, .18, .4]; this.fail = null;
    if (!asyncReadback) this.readRenderTargetPixelsAsync = undefined;
    renderers.add(this);
  }
  getRenderTarget() {return this.target;}
  getActiveCubeFace() {return this.cubeFace;}
  getActiveMipmapLevel() {return this.mipLevel;}
  getCurrentViewport(out) {return out.copy(this.currentViewport);}
  getViewport(out) {return out.copy(this.viewport);}
  getScissor(out) {return out.copy(this.scissor);}
  getScissorTest() {return this.scissorTest;}
  getPixelRatio() {return this.pixelRatio;}
  getClearColor(out) {return out.copy(this.clearColor);}
  getClearAlpha() {return this.clearAlpha;}
  setClearColor(color, alpha) {this.clearColor.copy(color); this.clearAlpha = alpha; this.activeClearColor.copy(color); this.activeClearAlpha = alpha;}
  setRenderTarget(target, cubeFace = 0, mipLevel = 0) {
    this.target = target; this.cubeFace = cubeFace; this.mipLevel = mipLevel;
    this.currentViewport.copy(target ? target.viewport : this.viewport.clone().multiplyScalar(this.pixelRatio));
    this.targetChanges.push(target);
  }
  clear(...flags) {this.clears.push({target: this.target, flags});}
  render(scene, camera) {
    const kind = scene === this.sourceScene ? 'source' : 'display';
    if (kind === 'source') {this.activeClearColor.setRGB(.4, .5, .6); this.activeClearAlpha = 1;}
    const material = kind === 'display' ? scene.children[0].material : null;
    this.draws.push({kind, scene, camera, material, geometry: scene.children[0]?.geometry,
      target: this.target, exposure: material?.uniforms.toneMappingExposure.value,
      toneMapping: this.toneMapping, outputColorSpace: this.outputColorSpace, autoClear: this.autoClear,
      xr: this.xr.enabled, shadowAutoUpdate: this.shadowMap.autoUpdate, shadowNeedsUpdate: this.shadowMap.needsUpdate});
    if (this.fail === kind) throw new Error(`Declared ${kind} draw failure`);
  }
  readRecord(method, target, x, y, width, height, pixels) {
    const draw = this.draws.at(-1), source = this.draws.at(-2);
    const record = {method, target, x, y, width, height, pixels,
      rgba: displayBytes(this.sampleLinearRGBA, draw.exposure, source.target.texture.type === THREE.UnsignedByteType)};
    this.reads.push(record); return record;
  }
  readRenderTargetPixels(target, x, y, width, height, pixels) {
    const record = this.readRecord('sync', target, x, y, width, height, pixels);
    if (this.fail === 'read-submit') throw new Error('Declared readback submission failure');
    for (let index = 0; index < pixels.length; index++) pixels[index] = record.rgba[index % 4];
    return pixels;
  }
  readRenderTargetPixelsAsync(target, x, y, width, height, pixels) {
    const record = this.readRecord('async', target, x, y, width, height, pixels);
    if (this.fail === 'read-submit') throw new Error('Declared readback submission failure');
    return new Promise((resolve, reject) => this.pendingReads.push({...record, resolve, reject}));
  }
  settleRead({error = null} = {}) {
    const pending = this.pendingReads.shift(); assert(pending, 'No controlled readback is pending');
    if (error) {pending.reject(error); return;}
    for (let index = 0; index < pending.pixels.length; index++) pending.pixels[index] = pending.rgba[index % 4];
    pending.resolve(pending.pixels);
  }
}

function snapshot(renderer) {
  return {target: renderer.target, cubeFace: renderer.cubeFace, mipLevel: renderer.mipLevel,
    currentViewport: renderer.currentViewport.toArray(), viewport: renderer.viewport.toArray(),
    scissor: renderer.scissor.toArray(), scissorTest: renderer.scissorTest,
    targetViewport: renderer.target?.viewport.toArray(), targetScissor: renderer.target?.scissor.toArray(),
    targetScissorTest: renderer.target?.scissorTest,
    toneMapping: renderer.toneMapping, exposure: renderer.toneMappingExposure, outputColorSpace: renderer.outputColorSpace,
    autoClear: renderer.autoClear, autoClearColor: renderer.autoClearColor, autoClearDepth: renderer.autoClearDepth,
    autoClearStencil: renderer.autoClearStencil, xr: {...renderer.xr}, shadowMap: {...renderer.shadowMap},
    clearColor: renderer.clearColor.toArray(), clearAlpha: renderer.clearAlpha,
    activeClearColor: renderer.activeClearColor.toArray(), activeClearAlpha: renderer.activeClearAlpha,
    pixelRatio: renderer.pixelRatio};
}
function create(options = {}, dimensions = {width: 4, height: 3}) {
  const renderer = new RendererContractFixture(options), view = createCameraFeedPass(renderer, dimensions);
  // Observe each original rejection without changing the returned Promise or
  // its sync-throw semantics, so a preceding failed assertion cannot leave an
  // unhandled controlled fixture rejection during final cleanup.
  if (typeof view.renderAsync === 'function') {
    const renderAsync = view.renderAsync.bind(view);
    view.renderAsync = (...values) => {const pending = renderAsync(...values); pending?.catch(() => {}); return pending;};
  }
  views.add(view); return {renderer, view, pixels: new Uint8Array(dimensions.width * dimensions.height * 4)};
}
function bounded(promise, label = 'contract operation') {
  let timer;
  return Promise.race([Promise.resolve(promise), new Promise((_, reject) => {
    timer = setTimeout(() => reject(new Error(`${label} did not settle within the bounded fixture`)), 300);
  })]).finally(() => clearTimeout(timer));
}
async function rejects(invoke, pattern) {
  let value;
  try {value = invoke();} catch (error) {assert.match(error.message, pattern); return;}
  await assert.rejects(bounded(value, 'Rejected operation'), pattern);
}
async function complete(renderer, promise) {renderer.settleRead(); return bounded(promise, 'Controlled readback');}
async function check(name, fn) {
  try {const detail = await fn(); results.push({name, pass: true, ...(detail ? {detail} : {})});}
  catch (error) {results.push({name, pass: false, error: {message: error.message, stack: error.stack}});}
}

await check('Async readback submits two passes synchronously and restores every caller state before its promise settles', async () => {
  for (const target of [true, false]) {
    const {renderer, view, pixels} = create({target}), before = snapshot(renderer);
    assert.equal(typeof view.renderAsync, 'function');
    assert.equal(view.readbackMode, 'async-pbo'); assert.equal(view.readbackPending, false);
    let settled = false;
    const pending = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
    assert(pending && typeof pending.then === 'function', 'renderAsync must return a Promise');
    pending.then(() => {settled = true;}, () => {settled = true;});
    assert.deepEqual(snapshot(renderer), before, 'Restore must happen before the caller regains control');
    assert.equal(renderer.draws.length, 2); assert.equal(renderer.reads.length, 1);
    assert.equal(renderer.reads[0].method, 'async'); assert.equal(renderer.pendingReads.length, 1);
    assert.equal(view.readbackPending, true);
    assert.deepEqual([...pixels], Array(pixels.length).fill(0));
    await Promise.resolve(); assert.equal(settled, false);
    assert.equal(await complete(renderer, pending), pixels); assert.equal(settled, true);
    assert.equal(view.readbackPending, false);
    assert.deepEqual(snapshot(renderer), before); view.dispose();
  }
});

await check('Delayed result returns the supplied RGBA8 buffer and uses submission-time ACES exposure without changing later main-view state', async () => {
  const {renderer, view, pixels} = create(), before = snapshot(renderer);
  const pending = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
  const read = renderer.reads[0], source = renderer.draws[0], display = renderer.draws[1];
  assert.equal(read.target, display.target); assert.deepEqual([read.x, read.y, read.width, read.height], [0, 0, 4, 3]);
  assert.equal(read.pixels, pixels); assert.equal(display.material.uniforms.linearFeed.value, source.target.texture);
  assert.equal(source.target.texture.type, THREE.HalfFloatType); assert.equal(display.target.texture.type, THREE.UnsignedByteType);
  assert.equal(source.target.texture.colorSpace, THREE.LinearSRGBColorSpace); assert.equal(display.target.texture.colorSpace, THREE.NoColorSpace);
  assert.deepEqual(snapshot(renderer), before);
  renderer.toneMappingExposure = 2.4; renderer.autoClear = false; renderer.xr.enabled = false;
  renderer.shadowMap.autoUpdate = false; renderer.shadowMap.needsUpdate = false;
  renderer.setRenderTarget(null); renderer.setClearColor(new THREE.Color(.1, .2, .3), .7);
  renderer.currentViewport.set(2, 3, 61, 47); renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
  const changedWhileWaiting = snapshot(renderer);
  assert.equal(await complete(renderer, pending), pixels); assert.deepEqual([...pixels.slice(0, 4)], [127, 127, 127, 102]);
  assert.deepEqual(snapshot(renderer), changedWhileWaiting, 'Promise settlement must not replay stale saved renderer state');
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  const next = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
  assert.equal(renderer.draws.at(-1).exposure, 2.4);
  await complete(renderer, next); assert.deepEqual([...pixels.slice(0, 4)], [188, 188, 188, 102]); view.dispose();
});

await check('One in-flight frame rejects sync/async reentry and disposal until completion without another draw or release', async () => {
  const {renderer, view, pixels} = create(), before = snapshot(renderer);
  const pending = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
  const owned = [renderer.draws[0].target, renderer.draws[1].target, renderer.draws[1].geometry, renderer.draws[1].material];
  let disposed = 0; for (const resource of owned) resource.addEventListener('dispose', () => disposed++);
  await rejects(() => view.render(renderer.sourceScene, renderer.sourceCamera, pixels), /re-enter|pending|in.flight|already|while|render/i);
  await rejects(() => view.renderAsync(renderer.sourceScene, renderer.sourceCamera, new Uint8Array(pixels.length)), /re-enter|pending|in.flight|already|while|render/i);
  await rejects(() => view.dispose(), /while|pending|in.flight|render/i);
  assert.equal(disposed, 0); assert.equal(renderer.draws.length, 2); assert.equal(renderer.reads.length, 1);
  assert.deepEqual(snapshot(renderer), before); await complete(renderer, pending);
  view.render(renderer.sourceScene, renderer.sourceCamera, pixels); assert.equal(renderer.reads.at(-1).method, 'sync');
  view.dispose(); view.dispose(); assert.equal(disposed, 4);
});

await check('Deferred readback rejection restores pending lifetime and allows a subsequent successful frame', async () => {
  const {renderer, view, pixels} = create(), before = snapshot(renderer);
  const pending = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
  assert.deepEqual(snapshot(renderer), before);
  const rejection = assert.rejects(bounded(pending), /Declared asynchronous readback failure/);
  renderer.settleRead({error: new Error('Declared asynchronous readback failure')}); await rejection;
  assert.equal(view.readbackPending, false);
  assert.deepEqual(snapshot(renderer), before);
  assert.deepEqual([...pixels], Array(pixels.length).fill(0));
  const recovery = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
  assert.equal(await complete(renderer, recovery), pixels); assert.deepEqual(snapshot(renderer), before); view.dispose();
});

await check('Source draw, display draw and asynchronous readback submission failures restore state immediately and do not poison reuse', async () => {
  for (const failure of ['source', 'display', 'read-submit']) {
    const {renderer, view, pixels} = create(), before = snapshot(renderer); renderer.fail = failure;
    let invocation;
    try {invocation = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);} catch (error) {invocation = Promise.reject(error);}
    const rejected = assert.rejects(bounded(invocation), /Declared .* failure/);
    assert.deepEqual(snapshot(renderer), before); await rejected;
    assert.equal(renderer.pendingReads.length, 0); renderer.fail = null;
    const recovery = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
    await complete(renderer, recovery); assert.deepEqual(snapshot(renderer), before); view.dispose();
  }
});

await check('A synchronous legacy renderer has an explicit compatible Promise fallback with unchanged color and restoration contracts', async () => {
  for (const floatColorBuffer of [true, false]) {
    const {renderer, view, pixels} = create({asyncReadback: false, floatColorBuffer, target: false});
    assert.equal(view.readbackMode, 'sync-fallback'); assert.equal(view.readbackPending, false);
    renderer.sampleLinearRGBA = [4, 4, 4, 1]; const before = snapshot(renderer);
    const pending = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
    assert(pending && typeof pending.then === 'function'); assert.equal(renderer.reads.length, 1);
    assert.equal(renderer.reads[0].method, 'sync'); assert.equal(renderer.pendingReads.length, 0);
    assert.deepEqual(snapshot(renderer), before); assert.equal(await bounded(pending), pixels);
    assert.deepEqual([...pixels.slice(0, 4)], floatColorBuffer ? [250, 250, 250, 255] : [226, 226, 226, 255]);
    if (!floatColorBuffer) {assert.equal(view.capabilities.hdr, false); assert.match(view.capabilities.limitation, /clips scene values above 1 before ACES/);}
    view.dispose();
  }
});

await check('Fallback readback errors restore all state and preserve later synchronous and asynchronous fallback use', async () => {
  const {renderer, view, pixels} = create({asyncReadback: false}), before = snapshot(renderer);
  renderer.fail = 'read-submit';
  await rejects(() => view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels), /Declared readback submission failure/);
  assert.deepEqual(snapshot(renderer), before); renderer.fail = null;
  assert.equal(await bounded(view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels)), pixels);
  assert.equal(view.render(renderer.sourceScene, renderer.sourceCamera, pixels), pixels); view.dispose();
});

await check('Async buffer/exposure/mapping validation rejects before any renderer mutation or read submission', async () => {
  const {renderer, view} = create({}, {width: 1, height: 1});
  for (const pixels of [null, new Uint8ClampedArray(4), new Float32Array(4), new Uint8Array(3), new Uint8Array(5)]) {
    const before = snapshot(renderer); await rejects(() => view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels), /Uint8Array|bytes|buffer/i);
    assert.deepEqual(snapshot(renderer), before);
  }
  for (const exposure of [NaN, Infinity, -.1]) {
    renderer.toneMappingExposure = exposure; const before = snapshot(renderer);
    await rejects(() => view.renderAsync(renderer.sourceScene, renderer.sourceCamera, new Uint8Array(4)), /exposure/i); assert.deepEqual(snapshot(renderer), before);
  }
  renderer.toneMappingExposure = 1; renderer.toneMapping = THREE.NoToneMapping; const before = snapshot(renderer);
  await rejects(() => view.renderAsync(renderer.sourceScene, renderer.sourceCamera, new Uint8Array(4)), /main view to use ACES/);
  assert.deepEqual(snapshot(renderer), before); assert.equal(renderer.draws.length, 0); assert.equal(renderer.reads.length, 0); view.dispose();
});

await check('Nested entry from either source/display draw cannot corrupt the enclosing asynchronous submission', async () => {
  const {renderer, view, pixels} = create(), draw = renderer.render.bind(renderer), before = snapshot(renderer);
  const rejectedEntries = [];
  renderer.render = (scene, camera) => {
    assert.throws(() => view.render(scene, camera, pixels), /re-enter|pending|render/i);
    rejectedEntries.push(rejects(() => view.renderAsync(scene, camera, pixels), /re-enter|pending|render/i));
    assert.throws(() => view.dispose(), /while|pending|render/i); draw(scene, camera);
  };
  const pending = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
  assert.deepEqual(snapshot(renderer), before); assert.equal(renderer.draws.length, 2); assert.equal(renderer.reads.length, 1);
  await Promise.all(rejectedEntries); await complete(renderer, pending); view.dispose();
});

await check('Sync and async paths reuse identical HDR/display targets, bundled shader, geometry and material across exposure changes', async () => {
  const {renderer, view, pixels} = create();
  assert.equal(view.render(renderer.sourceScene, renderer.sourceCamera, pixels), pixels);
  for (const exposure of [2.4, .7, 1]) {
    renderer.toneMappingExposure = exposure; const before = snapshot(renderer);
    const pending = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
    assert.deepEqual(snapshot(renderer), before); await complete(renderer, pending); assert.deepEqual(snapshot(renderer), before);
  }
  const source = renderer.draws.filter(draw => draw.kind === 'source'), display = renderer.draws.filter(draw => draw.kind === 'display');
  for (const [rows, property] of [[source, 'target'], [display, 'target'], [display, 'material'], [display, 'geometry']]) assert.equal(new Set(rows.map(row => row[property])).size, 1);
  assert.equal(renderer.capabilityQueries, 1);
  const shader = display[0].material.fragmentShader;
  assert.equal(shader.split(toneChunk).length - 1, 1); assert.equal(shader.split(colorChunk).length - 1, 1);
  assert(shader.includes('sRGBTransferOETF(vec4(ACESFilmicToneMapping(samplePixel.rgb), samplePixel.a))'));
  assert(!shader.includes('linearToOutputTexel') && !shader.includes('#include <tonemapping_fragment>'));
  assert.equal(display[0].material.isRawShaderMaterial, true); assert.equal(display[0].material.glslVersion, THREE.GLSL3);
  assert.equal(display[0].material.toneMapped, false); assert.equal(display[0].material.blending, THREE.NoBlending);
  for (const current of renderer.draws) {
    assert.equal(current.toneMapping, THREE.NoToneMapping); assert.equal(current.outputColorSpace, THREE.LinearSRGBColorSpace);
    assert.equal(current.xr, false); assert.equal(current.autoClear, false);
    assert.equal(current.shadowAutoUpdate, false); assert.equal(current.shadowNeedsUpdate, false);
  }
  view.dispose(); return {threeRevision: THREE.REVISION, shaderSHA256: sha(shader), asyncFrames: 3, syncFrames: 1};
});

await check('Successful async disposal is idempotent, owns exactly four resources, and rejects all subsequent entry', async () => {
  const {renderer, view, pixels} = create(), before = snapshot(renderer), pending = view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels);
  const display = renderer.draws[1], owned = [renderer.draws[0].target, display.target, display.geometry, display.material], events = new Map();
  for (const resource of [...owned, before.target]) resource.addEventListener('dispose', () => events.set(resource, (events.get(resource) ?? 0) + 1));
  await complete(renderer, pending); view.dispose(); view.dispose();
  for (const resource of owned) assert.equal(events.get(resource), 1);
  assert.equal(events.get(before.target), undefined); assert.equal(display.scene.children.length, 0);
  assert.deepEqual(snapshot(renderer), before);
  assert.throws(() => view.render(renderer.sourceScene, renderer.sourceCamera, pixels), /disposed/);
  await rejects(() => view.renderAsync(renderer.sourceScene, renderer.sourceCamera, pixels), /disposed/);
});

await check('Pass remains confined to existing renderer/targets with no context, canvas, robot or physical-world ownership', async () => {
  const source = await readFile(new URL('dist/multi/visuals/camera-feeds.mjs', root), 'utf8');
  assert(!/getContext\s*\(|new\s+(?:THREE\.)?WebGLRenderer\s*\(|putImageData\s*\(|\breadPixels\s*\(/.test(source));
  assert(!/from\s+['"][^'"]*(?:robot-runtime|controller|profile|kinematics|dynamics)/.test(source));
  assert.equal((source.match(/new THREE\.WebGLRenderTarget\(/g) ?? []).length, 2);
  assert(source.includes('readRenderTargetPixelsAsync'), 'Production path must use Three asynchronous readback');
});

// Never retain controlled pending reads after a failing contract; reject them
// before resource cleanup so the test suite itself cannot leak/hang forever.
for (const renderer of renderers) while (renderer.pendingReads.length) renderer.settleRead({error: new Error('Fixture cleanup after failed contract')});
await Promise.resolve(); await Promise.resolve();
for (const view of views) {try {view.dispose();} catch {}}
const endSourceHashes = await hashes(), sourceFrozen = JSON.stringify(sourceHashes) === JSON.stringify(endSourceHashes);
const report = {pass: sourceFrozen && results.every(result => result.pass), startedAt, finishedAt: new Date().toISOString(),
  wallSeconds: (performance.now() - started) / 1000, checks: results.length, results, sourceFrozen, sourceHashes, endSourceHashes,
  gpuExecuted: false, shaderCompileExecuted: false, browserExecuted: false,
  scope: 'Actual production camera-feed module with controlled asynchronous renderer-contract fixtures. Immediate restoration, deferred buffer/Promise lifetime, errors, reentry, disposal, fallback, original shader/color pipeline, resource reuse and ownership are tested. CPU-generated fixture bytes are not GPU pixels or a measured FPS improvement.',
  nativeFollowUp: 'A real browser/GPU must verify Three async readback, rendered HDR/ACES/sRGB equivalence, desktop/mobile camera feeds, theme changes, main-view interactions and paired frame/input timings. Sync fallback remains a compatibility path and is not claimed nonblocking.'};
const destination = new URL(reportPath, root); await mkdir(new URL('.', destination), {recursive: true});
await writeFile(destination, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify(report, null, 2)); if (!report.pass) process.exitCode = 1;
