import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import {createCameraFeedPass} from '../dist/multi/visuals/camera-feeds.mjs';

const root = new URL('../', import.meta.url), results = [];
const paths = ['dist/multi/visuals/camera-feeds.mjs', 'work/test-multi-camera-feeds.mjs',
  'dist/vendor/three.module.min.js', 'dist/vendor/three.core.min.js'];
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const hashes = async () => Object.fromEntries(await Promise.all(paths.map(async path =>
  [path, sha(await readFile(new URL(path, root)))])));
const sourceHashes = await hashes();
function check(name, fn) {const detail = fn(); results.push({name, pass: true, ...(detail ? {detail} : {})});}
const toneChunk = THREE.ShaderChunk.tonemapping_pars_fragment;
const colorChunk = THREE.ShaderChunk.colorspace_pars_fragment;

// Independent CPU reference for the bundled GLSL, not an execution of GLSL.
// Matrices come from the real chunk in GLSL's column-major order. The scalar
// coefficients below are asserted against that chunk before golden sampling.
const matrix = name => {
  const body = toneChunk.match(new RegExp(`const mat3 ${name} = mat3\\(([\\s\\S]*?)\\);`))?.[1];
  assert(body, `Bundled ACES matrix missing: ${name}`);
  const values = [...body.matchAll(/vec3\(([^)]+)\)/g)].flatMap(match => match[1].split(',').map(Number));
  assert.equal(values.length, 9); assert(values.every(Number.isFinite)); return values;
};
const inputMatrix = matrix('ACESInputMat'), outputMatrix = matrix('ACESOutputMat');
const multiply = (m, v) => [0, 1, 2].map(row => m[row] * v[0] + m[row + 3] * v[1] + m[row + 6] * v[2]);
function referenceDisplay(linear, exposure = 1, clipLinearToRGBA8 = false) {
  let value = linear.map(channel => (clipLinearToRGBA8 ? Math.max(0, Math.min(1, channel)) : channel) * exposure / .6);
  value = multiply(inputMatrix, value);
  value = value.map(v => (v * (v + .0245786) - .000090537) / (v * (.983729 * v + .4329510) + .238081));
  value = multiply(outputMatrix, value).map(v => Math.max(0, Math.min(1, v)));
  return value.map(v => v <= .0031308 ? v * 12.92 : Math.pow(v, .41666) * 1.055 - .055);
}
const bytes = value => value.map(channel => Math.round(Math.max(0, Math.min(1, channel)) * 255));

class RendererContractFixture {
  constructor({floatColorBuffer = true, target = true} = {}) {
    this.extensions = {has: name => {assert.equal(name, 'EXT_color_buffer_float'); this.capabilityQueries++; return floatColorBuffer;}};
    this.capabilities = {maxTextureSize: 4096}; this.capabilityQueries = 0;
    this.toneMapping = THREE.ACESFilmicToneMapping; this.toneMappingExposure = 1;
    this.outputColorSpace = THREE.SRGBColorSpace;
    this.clearColor = new THREE.Color(.02, .04, .08); this.clearAlpha = .6;
    this.activeClearColor = this.clearColor.clone(); this.activeClearAlpha = this.clearAlpha;
    this.autoClear = true; this.autoClearColor = false; this.autoClearDepth = true; this.autoClearStencil = false;
    this.xr = {enabled: true}; this.shadowMap = {enabled: true, autoUpdate: true, needsUpdate: true};
    this.pixelRatio = 2;
    this.viewport = new THREE.Vector4(7, 9, 640, 360);
    this.scissor = new THREE.Vector4(11, 13, 307, 201); this.scissorTest = true;
    this.target = target ? new THREE.WebGLRenderTarget(53, 37) : null;
    if (this.target) {this.target.viewport.set(1, 2, 49, 31); this.target.scissor.set(3, 4, 41, 29); this.target.scissorTest = true;}
    // A caller may have overridden its active target's viewport after binding.
    this.currentViewport = this.target ? new THREE.Vector4(5, 6, 23, 17) : this.viewport.clone().multiplyScalar(2);
    this.cubeFace = this.target ? 4 : 0; this.mipLevel = this.target ? 1 : 0;
    this.draws = []; this.reads = []; this.clears = []; this.targetChanges = [];
    this.sourceScene = new THREE.Scene(); this.sourceCamera = new THREE.PerspectiveCamera();
    this.sampleLinearRGBA = [.18, .18, .18, .4]; this.fail = null;
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
    // Three's color background changes the active GL clear color without
    // replacing the configured renderer clear color. The pass must restore it.
    if (kind === 'source') {this.activeClearColor.setRGB(.4, .5, .6); this.activeClearAlpha = 1;}
    const material = kind === 'display' ? scene.children[0].material : null;
    this.draws.push({kind, scene, camera, material, geometry: scene.children[0]?.geometry,
      target: this.target, exposure: material?.uniforms.toneMappingExposure.value,
      toneMapping: this.toneMapping, outputColorSpace: this.outputColorSpace, autoClear: this.autoClear,
      xr: this.xr.enabled, shadowAutoUpdate: this.shadowMap.autoUpdate, shadowNeedsUpdate: this.shadowMap.needsUpdate});
    if (this.fail === kind) throw new Error(`Declared ${kind} draw failure`);
  }
  readRenderTargetPixels(target, x, y, width, height, pixels) {
    this.reads.push({target, x, y, width, height, pixels});
    if (this.fail === 'read') throw new Error('Declared readback failure');
    const draw = this.draws.at(-1), source = this.draws.at(-2);
    // A declared contract fixture only. These bytes are a CPU reference and
    // do not constitute shader execution, GPU readback, or rendered evidence.
    const rgba = [...bytes(referenceDisplay(this.sampleLinearRGBA.slice(0, 3), draw.exposure,
      source.target.texture.type === THREE.UnsignedByteType)), Math.round(this.sampleLinearRGBA[3] * 255)];
    for (let i = 0; i < pixels.length; i++) pixels[i] = rgba[i % 4];
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
const fixture = new RendererContractFixture(), pass = createCameraFeedPass(fixture, {width: 4, height: 3});
const original = snapshot(fixture), pixels = new Uint8Array(4 * 3 * 4);
pass.render(fixture.sourceScene, fixture.sourceCamera, pixels);
const sourceDraw = fixture.draws[0], displayDraw = fixture.draws[1];

check('Real display shader embeds bundled ACES and sRGB chunks exactly once without automatic double encoding', () => {
  const shader = displayDraw.material.fragmentShader;
  assert(shader.includes(toneChunk)); assert(shader.includes(colorChunk));
  assert.equal(shader.split(toneChunk).length - 1, 1); assert.equal(shader.split(colorChunk).length - 1, 1);
  assert(shader.includes('sRGBTransferOETF(vec4(ACESFilmicToneMapping(samplePixel.rgb), samplePixel.a))'));
  assert(!shader.includes('linearToOutputTexel') && !shader.includes('#include <tonemapping_fragment>'));
  assert.equal(displayDraw.material.isRawShaderMaterial, true); assert.equal(displayDraw.material.glslVersion, THREE.GLSL3);
  assert.equal(displayDraw.material.toneMapped, false); assert.equal(displayDraw.material.blending, THREE.NoBlending);
  assert.equal(displayDraw.material.depthTest, false); assert.equal(displayDraw.material.depthWrite, false);
  assert.equal(displayDraw.target.texture.colorSpace, THREE.NoColorSpace);
  return {threeRevision: THREE.REVISION, toneChunkSHA256: sha(toneChunk), colorChunkSHA256: sha(colorChunk)};
});

check('CPU reference retains bundled matrix order, ACES exposure scale and exact OETF coefficients', () => {
  for (const fragment of ['v + 0.0245786', '- 0.000090537', '0.983729 * v + 0.4329510', '+ 0.238081', 'toneMappingExposure / 0.6']) assert(toneChunk.includes(fragment));
  for (const fragment of ['vec3( 0.41666 )', '* 1.055', 'vec3( 0.055 )', '* 12.92', 'vec3( 0.0031308 )']) assert(colorChunk.includes(fragment));
  const goldens = [
    {linear: [0, 0, 0], exposure: 1, expected: [0, 0, 0]},
    {linear: [.003, .003, .003], exposure: 1, expected: [1, 1, 1]},
    {linear: [.18, .18, .18], exposure: 1, expected: [127, 127, 127]},
    {linear: [.18, .18, .18], exposure: 2.4, expected: [188, 188, 188]},
    {linear: [1, 1, 1], exposure: 1, expected: [226, 226, 226]},
    {linear: [4, 4, 4], exposure: 1, expected: [250, 250, 250]},
    {linear: [4, .18, .02], exposure: 1, expected: [255, 187, 107]},
    {linear: [.02, .18, 4], exposure: 2.4, expected: [210, 200, 255]},
  ];
  for (const row of goldens) assert.deepEqual(bytes(referenceDisplay(row.linear, row.exposure)), row.expected);
  assert.deepEqual(referenceDisplay([0, 0, 0], 0), [0, 0, 0]);
  return {samples: goldens, scope: 'Independent CPU calculation of bundled GLSL coefficients; no GPU precision/readback claim.'};
});

check('HDR capability uses one linear HalfFloat scene target followed by one explicit RGBA8 display target', () => {
  assert.equal(pass.mode, 'hdr-half-float'); assert.equal(pass.capabilities.hdr, true); assert.equal(pass.capabilities.limitation, null);
  assert.equal(sourceDraw.scene, fixture.sourceScene); assert.equal(sourceDraw.camera, fixture.sourceCamera);
  assert.equal(sourceDraw.target.texture.type, THREE.HalfFloatType);
  assert.equal(sourceDraw.target.texture.colorSpace, THREE.LinearSRGBColorSpace);
  assert.equal(sourceDraw.target.depthBuffer, true); assert.equal(sourceDraw.target.samples, 0);
  assert.equal(displayDraw.target.texture.type, THREE.UnsignedByteType); assert.equal(displayDraw.target.depthBuffer, false);
  assert.equal(displayDraw.material.uniforms.linearFeed.value, sourceDraw.target.texture);
  for (const target of [sourceDraw.target, displayDraw.target]) {
    assert.equal(target.width, 4); assert.equal(target.height, 3);
    assert.equal(target.texture.format, THREE.RGBAFormat); assert.equal(target.texture.generateMipmaps, false);
    assert.equal(target.texture.minFilter, THREE.NearestFilter); assert.equal(target.texture.magFilter, THREE.NearestFilter);
    assert.equal(target.scissorTest, false);
  }
  assert.deepEqual(fixture.clears[0], {target: sourceDraw.target, flags: [true, true, true]});
});

check('Feed draws disable automatic mapping/XR/shadow refresh while original renderer and target state are restored', () => {
  for (const draw of fixture.draws) {
    assert.equal(draw.toneMapping, THREE.NoToneMapping); assert.equal(draw.outputColorSpace, THREE.LinearSRGBColorSpace);
    assert.equal(draw.xr, false); assert.equal(draw.autoClear, false);
    assert.equal(draw.shadowAutoUpdate, false); assert.equal(draw.shadowNeedsUpdate, false);
  }
  assert.deepEqual(snapshot(fixture), original);
  assert.equal(fixture.sourceScene.children.length, 0); assert.equal(fixture.sourceCamera.parent, null);
});

check('Caller receives display-encoded bytes in the same supplied buffer, with no flip or canvas/context operation', () => {
  const read = fixture.reads[0]; assert.equal(read.target, displayDraw.target);
  assert.deepEqual([read.x, read.y, read.width, read.height], [0, 0, 4, 3]); assert.equal(read.pixels, pixels);
  assert.deepEqual([...pixels.slice(0, 4)], [127, 127, 127, 102]);
  const result = pass.render(fixture.sourceScene, fixture.sourceCamera, pixels); assert.equal(result, pixels);
});

check('New theme exposures are read on every render without replacing targets, geometry, material, or renderer', () => {
  fixture.toneMappingExposure = 2.4; const before = snapshot(fixture);
  pass.render(fixture.sourceScene, fixture.sourceCamera, pixels);
  assert.equal(fixture.draws.at(-1).exposure, 2.4); assert.deepEqual([...pixels.slice(0, 4)], [188, 188, 188, 102]);
  assert.equal(new Set(fixture.draws.filter(draw => draw.kind === 'source').map(draw => draw.target)).size, 1);
  assert.equal(new Set(fixture.draws.filter(draw => draw.kind === 'display').map(draw => draw.target)).size, 1);
  assert.equal(new Set(fixture.draws.filter(draw => draw.kind === 'display').map(draw => draw.material)).size, 1);
  assert.equal(new Set(fixture.draws.filter(draw => draw.kind === 'display').map(draw => draw.geometry)).size, 1);
  assert.equal(fixture.capabilityQueries, 1); assert.deepEqual(snapshot(fixture), before);
});

check('RGBA8 fallback still transforms display color and explicitly reports unrecoverable HDR clipping', () => {
  const renderer = new RendererContractFixture({floatColorBuffer: false, target: false});
  const view = createCameraFeedPass(renderer, {width: 1, height: 1}), output = new Uint8Array(4), before = snapshot(renderer);
  assert.equal(view.mode, 'ldr-rgba8-fallback'); assert.equal(view.capabilities.hdr, false);
  assert.match(view.capabilities.limitation, /clips scene values above 1 before ACES/);
  renderer.sampleLinearRGBA = [4, 4, 4, 1]; view.render(renderer.sourceScene, renderer.sourceCamera, output);
  assert.equal(renderer.draws[0].target.texture.type, THREE.UnsignedByteType);
  assert.equal(renderer.draws[1].material.fragmentShader, displayDraw.material.fragmentShader);
  assert.deepEqual([...output], [226, 226, 226, 255]);
  assert.deepEqual(bytes(referenceDisplay([4, 4, 4])), [250, 250, 250]);
  assert.deepEqual(snapshot(renderer), before); view.dispose();
  return {hdrWhite4: [250, 250, 250], fallbackWhite4: [226, 226, 226], limitationExposed: true};
});

check('Absent extension API conservatively selects the same declared LDR fallback', () => {
  const renderer = new RendererContractFixture(); renderer.extensions = undefined;
  const view = createCameraFeedPass(renderer); assert.equal(view.mode, 'ldr-rgba8-fallback'); view.dispose();
});

check('Scene draw, display draw and readback failures restore all tested state and leave the pass reusable', () => {
  for (const failure of ['source', 'display', 'read']) {
    const renderer = new RendererContractFixture(), view = createCameraFeedPass(renderer, {width: 1, height: 1});
    const before = snapshot(renderer), output = new Uint8Array(4); renderer.fail = failure;
    assert.throws(() => view.render(renderer.sourceScene, renderer.sourceCamera, output), /Declared .* failure/);
    assert.deepEqual(snapshot(renderer), before); renderer.fail = null;
    view.render(renderer.sourceScene, renderer.sourceCamera, output); assert.deepEqual(snapshot(renderer), before); view.dispose();
  }
});

check('Invalid dimensions, buffers, exposure and non-ACES main mapping fail before renderer mutations', () => {
  const renderer = new RendererContractFixture();
  for (const options of [{width: 0}, {height: -1}, {width: 1.5}, {width: Infinity}, {width: 4097}]) assert.throws(() => createCameraFeedPass(renderer, options), RangeError);
  const view = createCameraFeedPass(renderer, {width: 1, height: 1});
  for (const output of [null, new Uint8ClampedArray(4), new Float32Array(4), new Uint8Array(3), new Uint8Array(5)]) {
    const before = snapshot(renderer); assert.throws(() => view.render(renderer.sourceScene, renderer.sourceCamera, output), TypeError); assert.deepEqual(snapshot(renderer), before);
  }
  for (const exposure of [NaN, Infinity, -.1]) {renderer.toneMappingExposure = exposure; const before = snapshot(renderer); assert.throws(() => view.render(renderer.sourceScene, renderer.sourceCamera, new Uint8Array(4)), RangeError); assert.deepEqual(snapshot(renderer), before);}
  renderer.toneMappingExposure = 1; renderer.toneMapping = THREE.NoToneMapping; const before = snapshot(renderer);
  assert.throws(() => view.render(renderer.sourceScene, renderer.sourceCamera, new Uint8Array(4)), /main view to use ACES/);
  assert.deepEqual(snapshot(renderer), before); assert.equal(renderer.draws.length, 0); view.dispose();
});

check('Reentrant rendering and in-render disposal are rejected without losing the outer pass state', () => {
  const renderer = new RendererContractFixture(), view = createCameraFeedPass(renderer, {width: 1, height: 1});
  const draw = renderer.render.bind(renderer), before = snapshot(renderer);
  renderer.render = (scene, camera) => {assert.throws(() => view.render(scene, camera, new Uint8Array(4)), /re-entered/); assert.throws(() => view.dispose(), /while it renders/); draw(scene, camera);};
  view.render(renderer.sourceScene, renderer.sourceCamera, new Uint8Array(4)); assert.deepEqual(snapshot(renderer), before); view.dispose();
});

check('Idempotent disposal releases exactly the four owned resources and no scene, renderer or original target resource', () => {
  const owned = [sourceDraw.target, displayDraw.target, displayDraw.geometry, displayDraw.material], events = new Map();
  for (const resource of [...owned, original.target]) resource.addEventListener('dispose', () => events.set(resource, (events.get(resource) ?? 0) + 1));
  const before = snapshot(fixture); pass.dispose(); pass.dispose();
  for (const resource of owned) assert.equal(events.get(resource), 1); assert.equal(events.get(original.target), undefined);
  assert.equal(displayDraw.scene.children.length, 0); assert.deepEqual(snapshot(fixture), before);
  assert.throws(() => pass.render(fixture.sourceScene, fixture.sourceCamera, pixels), /disposed/);
});

const source = await readFile(new URL(paths[0], root), 'utf8');
check('Module ownership and application integration boundary remain restricted to the new visual pass', () => {
  assert(!/getContext\s*\(|new\s+(?:THREE\.)?WebGLRenderer\s*\(|putImageData\s*\(|readPixels\s*\(/.test(source));
  assert(!/from\s+['"][^'"]*(?:robot-runtime|controller|profile|kinematics|dynamics)/.test(source));
  assert.equal((source.match(/new THREE\.WebGLRenderTarget\(/g) ?? []).length, 2);
  const renderSource = source.slice(source.indexOf('  function render('), source.indexOf('\n  return {\n    render, mode'));
  assert(!/new THREE\.(?:WebGLRenderTarget|.*Material|.*Geometry)\(/.test(renderSource));
});
const sourceFrozen = JSON.stringify(await hashes()) === JSON.stringify(sourceHashes); assert(sourceFrozen);
console.log(JSON.stringify({pass: true, checks: results.length, results, sourceFrozen, sourceHashes,
  gpuExecuted: false, shaderCompileExecuted: false,
  scope: 'Actual production shader source embeds bundled THREE r180 chunks. Independent CPU coefficient/golden reference plus declared renderer-contract fixtures verify two-pass ordering, capability fallback, current exposure, state restoration, resource reuse and ownership. Fixture bytes are CPU-generated, not GPU readback.',
  nativeFollowUp: 'Root must exercise night/hangar scenes in the native browser, confirm shader compilation and actual HDR/fallback mode, inspect main/feed exposure and semantic object colors, test theme changes while a task runs, and inspect desktop/phone output. No GPU or pixel pass is claimed here.'}, null, 2));
