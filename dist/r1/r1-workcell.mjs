// Observed reference: Rebot_Arm_AGV/assets/index-BXF1-FJz.js, nh/th/ih/sh.
// World coordinates here are Three.js Y-up; source MJCF is Z-up.
import {initAreaLightTextures} from './r1-area-light-textures.mjs';

export const WORKCELL_REFERENCE = Object.freeze({
  backgroundBeamFractions: Object.freeze([.035, .18, .5, .82, .965]),
  horizonFraction: .73,
  exposure: 1.02,
  overheadFov: 70,
  wristFov: 62.82,
  wristCant: Math.PI / 6,
});

function canvasTexture(THREE, width, height, draw) {
  const canvas = globalThis.document?.createElement('canvas');
  if (!canvas) return null;
  canvas.width = width; canvas.height = height;
  const context = canvas.getContext('2d');
  if (!context) return null;
  draw(context, width, height);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function gradient(context, x0, y0, x1, y1, stops) {
  const value = context.createLinearGradient(x0, y0, x1, y1);
  for (const [offset, color] of stops) value.addColorStop(offset, color);
  return value;
}

function backgroundTexture(THREE) {
  return canvasTexture(THREE, 1600, 900, (ctx, width, height) => {
    ctx.fillStyle = gradient(ctx, 0, 0, 0, height, [[0, '#010319'], [.56, '#030629'], [.72, '#06154b'], [.76, '#03153d'], [1, '#01030f']]);
    ctx.fillRect(0, 0, width, height);
    const horizon = height * WORKCELL_REFERENCE.horizonFraction;
    ctx.fillStyle = gradient(ctx, 0, horizon - 60, 0, horizon + 60, [[0, 'rgba(0,87,255,0)'], [.42, 'rgba(0,118,255,.24)'], [.5, 'rgba(0,229,255,.98)'], [.58, 'rgba(0,94,255,.30)'], [1, 'rgba(0,30,110,0)']]);
    ctx.fillRect(0, horizon - 60, width, 120);
    ctx.globalCompositeOperation = 'lighter';
    WORKCELL_REFERENCE.backgroundBeamFractions.forEach((fraction, index) => {
      const x = width * fraction;
      ctx.fillStyle = gradient(ctx, x - 22, 0, x + 22, 0, [[0, 'rgba(0,55,255,0)'], [.42, 'rgba(0,84,255,.18)'], [.5, index % 2 ? 'rgba(0,121,255,.78)' : 'rgba(0,199,255,.92)'], [.58, 'rgba(0,84,255,.18)'], [1, 'rgba(0,55,255,0)']]);
      ctx.fillRect(x - 22, 0, 44, horizon + 20);
      ctx.fillStyle = 'rgba(92,220,255,.64)';
      ctx.fillRect(x - 1, 0, 2, horizon + 20);
    });
    ctx.globalCompositeOperation = 'source-over';
    ctx.strokeStyle = 'rgba(16,78,178,.18)'; ctx.lineWidth = 1;
    for (let y = 120; y < horizon; y += 8) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
    }
    const vignette = ctx.createRadialGradient(width / 2, height * .55, height * .18, width / 2, height * .55, width * .62);
    vignette.addColorStop(0, 'rgba(0,0,0,0)'); vignette.addColorStop(1, 'rgba(0,0,14,.64)');
    ctx.fillStyle = vignette; ctx.fillRect(0, 0, width, height);
  });
}

function floorTexture(THREE) {
  const texture = canvasTexture(THREE, 1024, 1024, (ctx, width, height) => {
    ctx.fillStyle = '#020617'; ctx.fillRect(0, 0, width, height);
    const glow = ctx.createRadialGradient(width / 2, height / 2, 0, width / 2, height / 2, width * .7);
    glow.addColorStop(0, 'rgba(8,47,105,.48)'); glow.addColorStop(.5, 'rgba(3,21,56,.18)'); glow.addColorStop(1, 'rgba(2,6,23,0)');
    ctx.fillStyle = glow; ctx.fillRect(0, 0, width, height);
    ctx.strokeStyle = 'rgba(0,126,255,.34)'; ctx.lineWidth = 1; ctx.beginPath();
    for (let i = 1; i < 10; i++) {
      const x = i / 10 * width;
      ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.moveTo(0, x); ctx.lineTo(width, x);
    }
    ctx.stroke(); ctx.shadowColor = 'rgba(0,214,255,.9)'; ctx.shadowBlur = 10;
    ctx.strokeStyle = 'rgba(0,207,255,.96)'; ctx.lineWidth = 3;
    ctx.strokeRect(0, 0, width, height); ctx.shadowBlur = 0;
  });
  if (texture) {
    texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
    // Reference repeats 5 times across a 2 m plane. Preserve that 4 cm grid
    // spacing across the larger R1 floor without enlarging the task table.
    texture.repeat.set(20, 20); texture.anisotropy = 8;
  }
  return texture;
}

function environmentTexture(THREE) {
  const texture = canvasTexture(THREE, 1024, 512, (ctx, width, height) => {
    ctx.fillStyle = gradient(ctx, 0, 0, 0, height, [[0, '#d8f6ff'], [.14, '#3977b8'], [.5, '#081b45'], [.76, '#0b3c75'], [1, '#d8f7ff']]);
    ctx.fillRect(0, 0, width, height);
    [.08, .3, .57, .82].forEach((fraction, index) => {
      const x = width * fraction;
      ctx.fillStyle = gradient(ctx, x - 55, 0, x + 55, 0, [[0, 'rgba(37,99,235,0)'], [.35, 'rgba(61,193,255,.28)'], [.5, index % 2 ? 'rgba(235,252,255,.98)' : 'rgba(74,222,255,.92)'], [.65, 'rgba(61,193,255,.28)'], [1, 'rgba(37,99,235,0)']]);
      ctx.fillRect(x - 55, 0, 110, height);
    });
    ctx.fillStyle = gradient(ctx, 0, height * .62, 0, height * .83, [[0, 'rgba(0,41,120,0)'], [.48, 'rgba(60,220,255,.9)'], [.55, 'rgba(230,252,255,.98)'], [1, 'rgba(0,41,120,0)']]);
    ctx.fillRect(0, height * .62, width, height * .21);
  });
  if (texture) texture.mapping = THREE.EquirectangularReflectionMapping;
  return texture;
}

export function configureRenderer(renderer, {THREE}) {
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = WORKCELL_REFERENCE.exposure;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
}

export function configureOverheadCamera(camera, {THREE, TABLE_HEIGHT, tableCenter = .78}) {
  camera.fov = WORKCELL_REFERENCE.overheadFov;
  camera.near = .02; camera.far = 10;
  camera.position.set(tableCenter, TABLE_HEIGHT + .68, 0);
  camera.up.set(1, 0, 0);
  camera.lookAt(new THREE.Vector3(tableCenter, TABLE_HEIGHT, 0));
  camera.layers.enable(1); camera.updateProjectionMatrix();
}

export function createWorkcell(scene, {THREE, TABLE_HEIGHT, tableCenter = .78}) {
  initAreaLightTextures(THREE);
  const group = new THREE.Group(); group.name = 'r1-reference-workcell';
  scene.add(group);
  const textures = [backgroundTexture(THREE), floorTexture(THREE), environmentTexture(THREE)];
  const previous = {background: scene.background, environment: scene.environment, fog: scene.fog};
  scene.background = textures[0] || new THREE.Color(0x010319);
  if (textures[2]) scene.environment = textures[2];
  scene.fog = null;
  const material = (color, metalness = .12, roughness = .55) => new THREE.MeshStandardMaterial({color, metalness, roughness});
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(8, 8), new THREE.MeshStandardMaterial({map: textures[1], color: 0x41658f, metalness: .1, roughness: .82, envMapIntensity: .4}));
  floor.name = 'reference-blue-grid-floor'; floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; group.add(floor);
  const table = new THREE.Group(); table.name = 'r1-task-table'; table.position.x = tableCenter; group.add(table);
  // Same material conversion as reference Qu(): rs_table linear RGB × .58.
  const tableColor = new THREE.Color().setRGB(.34 * .58, .37 * .58, .36 * .58, THREE.LinearSRGBColorSpace);
  const top = new THREE.Mesh(new THREE.BoxGeometry(.72, .065, .76), material(tableColor));
  top.name = 'task-table-top'; top.position.y = TABLE_HEIGHT - .0325; top.receiveShadow = true; top.castShadow = true; table.add(top);
  const legMaterial = material(0x26343f, .25, .56);
  for (const x of [-.28, .28]) for (const z of [-.29, .29]) {
    const leg = new THREE.Mesh(new THREE.BoxGeometry(.045, TABLE_HEIGHT - .065, .045), legMaterial);
    leg.position.set(x, (TABLE_HEIGHT - .065) / 2, z); leg.castShadow = true; leg.receiveShadow = true; table.add(leg);
  }
  const lights = [];
  const addLight = light => {lights.push(light); group.add(light); return light;};
  addLight(new THREE.AmbientLight(0x6f9bd4, .26));
  addLight(new THREE.HemisphereLight(0xd9f6ff, 0x021026, .48));
  const key = addLight(new THREE.DirectionalLight(0xf2fbff, 1));
  key.position.set(1.2, 2.6, -.8); key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048); key.shadow.camera.near = .1; key.shadow.camera.far = 6;
  key.shadow.camera.left = -1.4; key.shadow.camera.right = 1.4; key.shadow.camera.top = 1.4; key.shadow.camera.bottom = -1.4;
  key.shadow.bias = -.00012; key.shadow.normalBias = .003;
  const fill = addLight(new THREE.DirectionalLight(0x8bdcff, .38)); fill.position.set(-1, 1.4, .5);
  const rim = addLight(new THREE.DirectionalLight(0x286fff, .62)); rim.position.set(-.5, 2.1, 1);
  // sh() uses these three actual area emitters in addition to the environment.
  // Normalize around the reference .50×.90 m table at [.38,0,.10].
  // Scaling X/height by table width and depth by table depth preserves their
  // relative placement around the adapted .72×.76 m R1 work surface.
  const widthScale = .72 / .50, depthScale = .76 / .90;
  const sourcePoint = ([x, y, z]) => new THREE.Vector3(tableCenter + (x - .38) * widthScale, TABLE_HEIGHT + (z - .10) * widthScale, -y * depthScale);
  const areaSpecs = [
    {name: 'reference-softbox-key', color: 0xeaf8ff, intensity: 2, width: 1.6, height: 1.15, position: [.28, -.58, 1.28], target: [.25, 0, .16]},
    {name: 'reference-softbox-cyan', color: 0x10cfff, intensity: 2.8, width: .75, height: 1.05, position: [-.58, .20, .68], target: [.23, 0, .18]},
    {name: 'reference-softbox-blue', color: 0x245cff, intensity: 1.9, width: .70, height: .90, position: [.86, .32, .62], target: [.25, 0, .18]},
  ];
  for (const spec of areaSpecs) {
    const area = addLight(new THREE.RectAreaLight(spec.color, spec.intensity, spec.width * widthScale, spec.height * widthScale));
    area.name = spec.name; area.position.copy(sourcePoint(spec.position)); area.lookAt(sourcePoint(spec.target));
  }
  return {
    group, table, floor, grid: floor, lights,
    dispose() {
      scene.remove(group);
      scene.background = previous.background; scene.environment = previous.environment; scene.fog = previous.fog;
      const geometries = new Set(), materials = new Set();
      group.traverse(node => {if (node.geometry) geometries.add(node.geometry); if (node.material) materials.add(node.material);});
      geometries.forEach(value => value.dispose()); materials.forEach(value => value.dispose());
      textures.forEach(value => value?.dispose()); lights.forEach(value => value.dispose?.());
    },
  };
}

export function createWristVision({scene, THREE, camera}) {
  const model = new THREE.Group(); model.name = 'r1-wrist-camera-mount'; scene.add(model);
  const black = new THREE.MeshStandardMaterial({color: 0x29343b, metalness: .22, roughness: .38});
  const front = new THREE.MeshStandardMaterial({color: 0x061219, metalness: .12, roughness: .34});
  const lens = new THREE.MeshStandardMaterial({color: 0x0569a2, metalness: .1, roughness: .08, emissive: 0x005380, emissiveIntensity: .2});
  const bracketMaterial = new THREE.MeshStandardMaterial({color: 0x313a40, metalness: .3, roughness: .5});
  const lensPosition = new THREE.Vector3(-.035, 0, -.065);
  const sine = Math.sin(WORKCELL_REFERENCE.wristCant), cosine = Math.cos(WORKCELL_REFERENCE.wristCant);
  const direction = new THREE.Vector3(cosine, 0, sine);
  const up = new THREE.Vector3(-sine, 0, -cosine);
  const cameraRotation = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(new THREE.Vector3(0, 1, 0), up, direction.clone().negate()));
  const assembly = new THREE.Group(); assembly.position.copy(lensPosition); assembly.quaternion.copy(cameraRotation); model.add(assembly);
  const housing = new THREE.Mesh(new THREE.BoxGeometry(.042, .042, .023), black); housing.position.z = .0125; assembly.add(housing);
  const face = new THREE.Mesh(new THREE.BoxGeometry(.0404, .0404, .002), front); assembly.add(face);
  for (const x of [-.009, .009]) {
    const imager = new THREE.Mesh(new THREE.CylinderGeometry(.0055, .0055, .0024, 20), lens);
    imager.rotation.x = Math.PI / 2; imager.position.set(x, 0, -.002); assembly.add(imager);
  }
  function bracket(from, to) {
    const a = new THREE.Vector3(...from), b = new THREE.Vector3(...to), delta = b.clone().sub(a);
    const bar = new THREE.Mesh(new THREE.BoxGeometry(.006, .006, delta.length()), bracketMaterial);
    bar.position.copy(a).add(b).multiplyScalar(.5);
    bar.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), delta.normalize()); model.add(bar);
  }
  for (const y of [-.025, .025]) {
    bracket([-.065, y, -.012], [-.065, y, -.05]);
    bracket([-.065, y, -.05], [-.046, y, -.058]);
  }
  // Layer 1 is drawn by main/overhead cameras, excluded from wrist camera.
  // A lens mounted to the wrist should never render its own housing/bracket.
  model.traverse(node => {node.layers.set(1); if (node.isMesh) {node.castShadow = true; node.receiveShadow = true;}});
  camera.layers.disable(1); camera.fov = WORKCELL_REFERENCE.wristFov; camera.near = .008; camera.far = 10; camera.updateProjectionMatrix();
  const linkPosition = new THREE.Vector3(), linkRotation = new THREE.Quaternion(), worldDirection = new THREE.Vector3(), worldUp = new THREE.Vector3(), target = new THREE.Vector3();
  return {
    model,
    get visible() {return model.visible;},
    setVisible(value) {model.visible = Boolean(value); return model.visible;},
    update(link) {
      if (!link) return false;
      link.updateWorldMatrix(true, false); link.getWorldPosition(linkPosition); link.getWorldQuaternion(linkRotation);
      model.position.copy(linkPosition); model.quaternion.copy(linkRotation);
      camera.position.copy(lensPosition).applyQuaternion(linkRotation).add(linkPosition);
      worldDirection.copy(direction).applyQuaternion(linkRotation); worldUp.copy(up).applyQuaternion(linkRotation);
      camera.up.copy(worldUp); target.copy(camera.position).add(worldDirection); camera.lookAt(target); camera.updateMatrixWorld(true);
      return true;
    },
    dispose() {
      scene.remove(model);
      model.traverse(node => node.geometry?.dispose());
      [black, front, lens, bracketMaterial].forEach(value => value.dispose());
    },
  };
}
