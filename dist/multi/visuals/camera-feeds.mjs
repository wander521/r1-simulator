import * as THREE from '../../vendor/three.module.min.js';

// A render target is linear even when the canvas uses an sRGB display space.
// Readback therefore needs its own display pass before ImageData sees bytes.
// Use the bundled renderer's actual functions, including its ACES /0.6 scale
// and sRGB OETF exponent, rather than a similar-looking filmic approximation.
const vertexShader = `
precision highp float;
in vec3 position;
in vec2 uv;
out vec2 feedUv;
void main() {
  feedUv = uv;
  gl_Position = vec4(position.xy, 0.0, 1.0);
}`;
const fragmentShader = `
precision highp float;
uniform highp sampler2D linearFeed;
in vec2 feedUv;
out vec4 displayPixel;
${THREE.ShaderChunk.tonemapping_pars_fragment}
${THREE.ShaderChunk.colorspace_pars_fragment}
void main() {
  vec4 samplePixel = texture(linearFeed, feedUv);
  displayPixel = sRGBTransferOETF(vec4(ACESFilmicToneMapping(samplePixel.rgb), samplePixel.a));
}`;

/**
 * Scene -> linear HDR (or declared LDR fallback) -> ACES/sRGB RGBA8 readback.
 * Uses the existing renderer; the caller retains its row flip / putImageData.
 * There are no robot, camera-frame, physics, or context ownership changes.
 */
export function createCameraFeedPass(renderer, {width = 240, height = 180} = {}) {
  const maximum = renderer.capabilities?.maxTextureSize ?? Infinity;
  if (![width, height].every(value => Number.isSafeInteger(value) && value > 0 && value <= maximum)
      || !Number.isSafeInteger(width * height * 4)) {
    throw new RangeError('Camera feed dimensions must be positive texture-sized integers.');
  }
  const hdr = renderer.extensions?.has('EXT_color_buffer_float') === true;
  const mode = hdr ? 'hdr-half-float' : 'ldr-rgba8-fallback';
  const asynchronous = typeof renderer.readRenderTargetPixelsAsync === 'function';
  const readbackMode = asynchronous ? 'async-pbo' : 'sync-fallback';
  const capabilities = Object.freeze({
    mode, hdr, width, height, readbackMode,
    displayTransform: 'Bundled THREE ACESFilmicToneMapping + sRGBTransferOETF',
    exposure: 'Reads the current renderer.toneMappingExposure for every feed render.',
    limitation: hdr ? null : 'EXT_color_buffer_float is unavailable: linear RGBA8 clips scene values above 1 before ACES. The explicit ACES/sRGB transform remains active, but HDR highlights cannot be recovered.',
  });
  const linearTarget = new THREE.WebGLRenderTarget(width, height, {
    type: hdr ? THREE.HalfFloatType : THREE.UnsignedByteType,
    format: THREE.RGBAFormat, colorSpace: THREE.LinearSRGBColorSpace,
    minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter,
    depthBuffer: true, stencilBuffer: false, generateMipmaps: false,
  });
  linearTarget.texture.name = 'multi-feed-linear-scene';
  // The shader already encodes sRGB. Keep the destination framebuffer linear
  // so the GPU/renderer cannot apply a second transfer before byte readback.
  const displayTarget = new THREE.WebGLRenderTarget(width, height, {
    type: THREE.UnsignedByteType, format: THREE.RGBAFormat,
    colorSpace: THREE.NoColorSpace,
    minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter,
    depthBuffer: false, stencilBuffer: false, generateMipmaps: false,
  });
  displayTarget.texture.name = 'multi-feed-display-srgb-bytes';
  const material = new THREE.RawShaderMaterial({
    name: 'MultiCameraFeedDisplay', glslVersion: THREE.GLSL3,
    uniforms: {linearFeed: {value: linearTarget.texture}, toneMappingExposure: {value: 1}},
    vertexShader, fragmentShader,
    depthTest: false, depthWrite: false, blending: THREE.NoBlending,
    toneMapped: false,
  });
  const geometry = new THREE.PlaneGeometry(2, 2);
  const quad = new THREE.Mesh(geometry, material); quad.frustumCulled = false;
  const displayScene = new THREE.Scene(); displayScene.add(quad);
  const displayCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
  const savedCurrentViewport = new THREE.Vector4(), savedTargetViewport = new THREE.Vector4(), savedClearColor = new THREE.Color();
  let disposed = false, rendering = false, readbackPending = false;

  function render(scene, camera, pixels, asyncRead = false) {
    if (disposed) throw new Error('Camera feed pass is disposed.');
    if (rendering || readbackPending) throw new Error('Camera feed rendering cannot be re-entered.');
    if (!(pixels instanceof Uint8Array) || pixels.length !== width * height * 4) {
      throw new TypeError(`Camera feed readback needs a Uint8Array of ${width * height * 4} bytes.`);
    }
    if (renderer.toneMapping !== THREE.ACESFilmicToneMapping) {
      throw new RangeError('Camera feed display pass requires the main view to use ACESFilmicToneMapping.');
    }
    const exposure = renderer.toneMappingExposure;
    if (!Number.isFinite(exposure) || exposure < 0) throw new RangeError('Camera feed exposure must be finite and nonnegative.');
    const target = renderer.getRenderTarget();
    const cubeFace = renderer.getActiveCubeFace(), mipLevel = renderer.getActiveMipmapLevel();
    const clearAlpha = renderer.getClearAlpha(); renderer.getClearColor(savedClearColor);
    const saved = {
      toneMapping: renderer.toneMapping, exposure, outputColorSpace: renderer.outputColorSpace,
      autoClear: renderer.autoClear, xrEnabled: renderer.xr.enabled,
      shadowAutoUpdate: renderer.shadowMap.autoUpdate, shadowNeedsUpdate: renderer.shadowMap.needsUpdate,
    };
    // setRenderTarget restores the target's own viewport/scissor. Preserve a
    // caller's manually overridden current viewport too, without changing its
    // global viewport, target metadata, pixel ratio, or scissor configuration.
    renderer.getCurrentViewport(savedCurrentViewport);
    if (target) savedTargetViewport.copy(target.viewport);
    rendering = true;
    try {
      renderer.xr.enabled = false;
      renderer.shadowMap.autoUpdate = false; renderer.shadowMap.needsUpdate = false;
      renderer.autoClear = false;
      renderer.toneMapping = THREE.NoToneMapping;
      renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
      material.uniforms.toneMappingExposure.value = exposure;
      renderer.setRenderTarget(linearTarget);
      renderer.clear(true, true, true);
      renderer.render(scene, camera);
      renderer.setRenderTarget(displayTarget);
      renderer.render(displayScene, displayCamera);
      if (asyncRead) {
        // The bundled renderer submits a PBO read and restores its framebuffer
        // before waiting on the GPU fence. Restore ALL pass state below in this
        // same synchronous turn; a later completion must not overwrite the
        // viewport, exposure or render target used by a newer main-view frame.
        readbackPending = true;
        let readback;
        try { readback = renderer.readRenderTargetPixelsAsync(displayTarget, 0, 0, width, height, pixels); }
        catch (error) { readbackPending = false; throw error; }
        return Promise.resolve(readback).then(result => {
          if (result !== pixels) throw new Error('Camera feed readback did not return the requested buffer.');
          return pixels;
        }).finally(() => { readbackPending = false; });
      }
      renderer.readRenderTargetPixels(displayTarget, 0, 0, width, height, pixels);
      return pixels;
    } finally {
      renderer.toneMapping = saved.toneMapping;
      renderer.toneMappingExposure = saved.exposure;
      renderer.outputColorSpace = saved.outputColorSpace;
      renderer.autoClear = saved.autoClear;
      renderer.xr.enabled = saved.xrEnabled;
      renderer.shadowMap.autoUpdate = saved.shadowAutoUpdate;
      renderer.shadowMap.needsUpdate = saved.shadowNeedsUpdate;
      try {
        if (target) target.viewport.copy(savedCurrentViewport);
        renderer.setRenderTarget(target, cubeFace, mipLevel);
      } finally {
        if (target) target.viewport.copy(savedTargetViewport);
        renderer.setClearColor(savedClearColor, clearAlpha);
        rendering = false;
      }
    }
  }

  return {
    render, mode, capabilities, readbackMode,
    get readbackPending() { return readbackPending; },
    renderAsync(scene, camera, pixels) {
      // Preserve the old synchronous API and its exact color path for callers
      // and renderers without asynchronous readback support.
      return asynchronous ? render(scene, camera, pixels, true) : Promise.resolve(render(scene, camera, pixels));
    },
    dispose() {
      if (disposed) return;
      if (rendering || readbackPending) throw new Error('Cannot dispose a camera feed pass while it renders.');
      disposed = true;
      displayScene.remove(quad);
      geometry.dispose(); material.dispose(); linearTarget.dispose(); displayTarget.dispose();
    },
  };
}
