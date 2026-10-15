import * as VendorTHREE from '../vendor/three.module.min.js';

const MM_TO_METERS = 0.001;
const IDENTITY_FRAME = [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0];
const TEXTURE_MAP_CHUNKS = new Set([0xa200, 0xa204, 0xa210, 0xa220, 0xa230,
  0xa33a, 0xa33c, 0xa33d, 0xa33e, 0xa340, 0xa342, 0xa344, 0xa346, 0xa348, 0xa34a]);

function fail(message) { throw new Error(`C5 3DS: ${message}`); }

/**
 * Decode the manufacturer's static, untextured C5 visual mesh without replacing
 * it with a collision hull. Original face-material assignments and vertices are
 * retained. The source files are millimeters; URDF transforms remain meters.
 * Non-identity mesh/keyframe transforms and external textures are rejected
 * explicitly rather than silently displaying a displaced or uncolored model.
 */
export function parseC5_3DS(input, THREE = VendorTHREE) {
  const bytes = input instanceof ArrayBuffer ? new Uint8Array(input)
    : ArrayBuffer.isView(input) ? new Uint8Array(input.buffer, input.byteOffset, input.byteLength) : null;
  if (!bytes || bytes.byteLength < 6) fail('missing or truncated binary input');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const materials = new Map();
  const objects = [];
  let masterScale = 1;

  function need(offset, count, end = bytes.byteLength) {
    if (!Number.isInteger(offset) || !Number.isInteger(count) || count < 0 || offset < 0 || offset + count > end) {
      fail('truncated chunk data');
    }
  }
  function f32(offset, end) {
    need(offset, 4, end);
    const value = view.getFloat32(offset, true);
    if (!Number.isFinite(value)) fail('non-finite mesh value');
    return value;
  }
  function cstring(offset, end) {
    let cursor = offset;
    while (cursor < end && bytes[cursor] !== 0) cursor++;
    if (cursor === end) fail('unterminated string');
    return { value: new TextDecoder().decode(bytes.subarray(offset, cursor)), next: cursor + 1 };
  }
  function chunks(start, end, visit) {
    let cursor = start;
    while (cursor < end) {
      need(cursor, 6, end);
      const id = view.getUint16(cursor, true);
      const length = view.getUint32(cursor + 2, true);
      if (length < 6 || cursor + length > end) fail(`invalid chunk length (${id.toString(16)})`);
      if (TEXTURE_MAP_CHUNKS.has(id) || id === 0xa300) fail('external texture maps are unsupported');
      visit({ id, start: cursor + 6, end: cursor + length, length });
      cursor += length;
    }
  }
  function color(start, end) {
    let rgb = null;
    chunks(start, end, chunk => {
      if (chunk.id === 0x0011 || chunk.id === 0x0012) {
        need(chunk.start, 3, chunk.end);
        rgb = [bytes[chunk.start], bytes[chunk.start + 1], bytes[chunk.start + 2]];
      } else if (chunk.id === 0x0010 || chunk.id === 0x0013) {
        const values = [0, 1, 2].map(axis => f32(chunk.start + axis * 4, chunk.end));
        if (values.some(value => value < 0 || value > 1)) fail('invalid material RGB');
        rgb = values.map(value => value * 255);
      }
    });
    if (!rgb) fail('missing material RGB');
    return rgb;
  }
  function material(start, end) {
    const record = { name: null, diffuse: null };
    chunks(start, end, chunk => {
      if (chunk.id === 0xa000) record.name = cstring(chunk.start, chunk.end).value;
      else if (chunk.id === 0xa020) record.diffuse = color(chunk.start, chunk.end);
      // Other material scalars (source Phong shininess, etc.) do not change
      // geometry or source diffuse colors; the workcell uses its PBR finish.
    });
    if (!record.name || !record.diffuse || materials.has(record.name)) fail('missing or duplicate material');
    materials.set(record.name, record);
  }
  function faceArray(start, end, object) {
    need(start, 2, end);
    const count = view.getUint16(start, true);
    need(start + 2, count * 8, end);
    const indices = new Uint16Array(count * 3);
    for (let face = 0; face < count; face++) {
      for (let axis = 0; axis < 3; axis++) indices[face * 3 + axis] = view.getUint16(start + 2 + face * 8 + axis * 2, true);
    }
    object.indices = indices;
    object.faceMaterials = new Array(count).fill(null);
    chunks(start + 2 + count * 8, end, chunk => {
      if (chunk.id === 0x4130) {
        const name = cstring(chunk.start, chunk.end);
        need(name.next, 2, chunk.end);
        const size = view.getUint16(name.next, true);
        need(name.next + 2, size * 2, chunk.end);
        if (name.next + 2 + size * 2 !== chunk.end) fail('invalid face-material group length');
        for (let index = 0; index < size; index++) {
          const face = view.getUint16(name.next + 2 + index * 2, true);
          if (face >= count || object.faceMaterials[face] !== null) fail('invalid or duplicate face-material assignment');
          object.faceMaterials[face] = name.value;
        }
      } else if (chunk.id === 0x4150 && chunk.length !== 6 + count * 4) fail('invalid smoothing-group length');
    });
  }
  function mesh(start, end, object) {
    chunks(start, end, chunk => {
      if (chunk.id === 0x4110) {
        if (object.positions) fail('duplicate vertex array');
        need(chunk.start, 2, chunk.end);
        const count = view.getUint16(chunk.start, true);
        if (chunk.length !== 8 + count * 12) fail('invalid vertex-array length');
        object.positions = new Float32Array(count * 3);
        for (let index = 0; index < count * 3; index++) object.positions[index] = f32(chunk.start + 2 + index * 4, chunk.end) * MM_TO_METERS;
      } else if (chunk.id === 0x4120) {
        if (object.indices) fail('duplicate face array');
        faceArray(chunk.start, chunk.end, object);
      } else if (chunk.id === 0x4160) {
        if (chunk.length !== 54) fail('invalid mesh matrix');
        object.frame = IDENTITY_FRAME.map((value, index) => f32(chunk.start + index * 4, chunk.end));
        if (object.frame.some((value, index) => Math.abs(value - IDENTITY_FRAME[index]) > 1e-7)) {
          fail('non-identity mesh matrix is unsupported');
        }
      } else if (chunk.id === 0x4140) {
        need(chunk.start, 2, chunk.end);
        const count = view.getUint16(chunk.start, true);
        if (chunk.length !== 8 + count * 8) fail('invalid UV-array length');
        for (let index = 0; index < count * 2; index++) f32(chunk.start + 2 + index * 4, chunk.end);
        object.uvCount = count;
      }
    });
  }
  function keyframes(start, end) {
    chunks(start, end, chunk => {
      if (chunk.id === 0xb002) keyframes(chunk.start, chunk.end);
      else if (chunk.id === 0xb013) {
        if (chunk.length !== 18 || [0, 1, 2].some(axis => Math.abs(f32(chunk.start + axis * 4, chunk.end)) > 1e-7)) {
          fail('non-zero keyframe pivot is unsupported');
        }
      } else if (chunk.id === 0xb020 || chunk.id === 0xb021 || chunk.id === 0xb022) {
        need(chunk.start, 14, chunk.end);
        if (view.getUint32(chunk.start + 10, true) !== 0) fail('animated keyframe transforms are unsupported');
      }
    });
  }
  function scene(start, end) {
    chunks(start, end, chunk => {
      if (chunk.id === 0x3d3d) scene(chunk.start, chunk.end);
      else if (chunk.id === 0x0100) {
        masterScale = f32(chunk.start, chunk.end);
        if (Math.abs(masterScale - 1) > 1e-7) fail('non-unit master scale is unsupported');
      } else if (chunk.id === 0xafff) material(chunk.start, chunk.end);
      else if (chunk.id === 0x4000) {
        const name = cstring(chunk.start, chunk.end);
        const object = { name: name.value, frame: [...IDENTITY_FRAME] };
        chunks(name.next, chunk.end, child => {
          if (child.id === 0x4100) mesh(child.start, child.end, object);
        });
        if (object.positions || object.indices) objects.push(object);
      } else if (chunk.id === 0xb000) keyframes(chunk.start, chunk.end);
    });
  }
  if (view.getUint16(0, true) !== 0x4d4d || view.getUint32(2, true) !== bytes.byteLength) fail('invalid 3DS root');
  scene(6, bytes.byteLength);
  if (objects.length === 0 || materials.size === 0) fail('missing visual mesh or materials');

  // A Group is intentional. URDFLoader replaces materials on a root Mesh with
  // the URDF's plain white material, losing the manufacturer's blue/silver faces.
  const group = new THREE.Group();
  group.name = 'C5 official 3DS visual';
  group.userData.c5SourceMaterials = true;
  group.userData.c5SourceUnits = 'millimeter';
  group.userData.c5VertexScaleToMeters = MM_TO_METERS;
  group.userData.c5MasterScale = masterScale;
  group.userData.c5SourceVertices = 0;
  group.userData.c5SourceTriangles = 0;
  for (const object of objects) {
    if (!object.positions || !object.indices || object.indices.length === 0) fail('incomplete mesh');
    const vertexCount = object.positions.length / 3;
    if (object.indices.some(index => index >= vertexCount)) fail('face references invalid vertex');
    if (object.uvCount !== undefined && object.uvCount !== vertexCount) fail('UV and vertex counts differ');
    if (object.faceMaterials.some(name => name === null || !materials.has(name))) fail('unassigned or unknown face material');
    const usedNames = [...new Set(object.faceMaterials)];
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(object.positions, 3));
    const orderedIndices = [];
    const pbr = [];
    usedNames.forEach((name, materialIndex) => {
      const start = orderedIndices.length;
      object.faceMaterials.forEach((faceName, face) => {
        if (faceName === name) orderedIndices.push(...object.indices.subarray(face * 3, face * 3 + 3));
      });
      geometry.addGroup(start, orderedIndices.length - start, materialIndex);
      const source = materials.get(name);
      const diffuse = new THREE.Color().setRGB(...source.diffuse.map(value => value / 255)).convertSRGBToLinear();
      const finish = new THREE.MeshStandardMaterial({ color: diffuse, metalness: 0.42, roughness: 0.36 });
      finish.name = name;
      finish.userData.c5SourceDiffuseSRGB = [...source.diffuse];
      finish.userData.c5SourceMaterials = true;
      pbr.push(finish);
    });
    geometry.setIndex(orderedIndices);
    geometry.computeVertexNormals();
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();
    const visual = new THREE.Mesh(geometry, pbr);
    visual.name = object.name;
    visual.castShadow = true;
    visual.receiveShadow = true;
    visual.userData.c5SourceMaterials = true;
    visual.userData.c5SourceVertices = vertexCount;
    visual.userData.c5SourceTriangles = object.indices.length / 3;
    visual.userData.c5SourceFaceMaterialCounts = Object.fromEntries(usedNames.map(name => [name,
      object.faceMaterials.filter(faceName => faceName === name).length]));
    group.userData.c5SourceVertices += vertexCount;
    group.userData.c5SourceTriangles += object.indices.length / 3;
    group.add(visual);
  }
  return group;
}

/** URDFLoader.loadMeshCb-compatible .3ds branch; delegate other formats upstream. */
export function loadC5Mesh(path, manager, done) {
  const loader = new VendorTHREE.FileLoader(manager);
  loader.setResponseType('arraybuffer');
  return loader.load(path, buffer => {
    try { done(parseC5_3DS(buffer)); }
    catch (error) { manager?.itemError(path); done(null, error); }
  }, undefined, error => done(null, error));
}
