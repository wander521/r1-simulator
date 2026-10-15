import assert from 'node:assert/strict';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import * as THREE from '../dist/vendor/three.module.min.js';
import { parseC5_3DS } from '../dist/c5/c5-3ds.mjs';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const ASSETS = resolve(ROOT, 'dist/c5/assets');
const sha256 = buffer => createHash('sha256').update(buffer).digest('hex');
const manifest = JSON.parse(await readFile(resolve(ASSETS, 'SOURCE_MANIFEST.json'), 'utf8'));
const EXPECTED_VERTICES = [649, 1706, 3383, 2747, 1673, 1673, 3031];
const EXPECTED_TRIANGLES = [316, 2049, 4066, 3406, 2016, 2016, 1600];
const report = { ok: false, model: 'official aubo_C5', sourceCommit: manifest.commit,
  scope: 'Offline source/3DS geometry/material verification. Not a rendered browser or hardware approval.',
  parserSha256: sha256(await readFile(resolve(ROOT, 'dist/c5/c5-3ds.mjs'))),
  sourceFiles: [], links: [], negativeInputs: [], colors: [] };

// Independent, read-only source inspection. All offsets refer to the untouched
// source bytes, not geometry produced by the rendering parser under test.
function sourceChunks(buffer) {
  const found = [];
  function scan(start, end) {
    for (let offset = start; offset < end;) {
      const id = buffer.readUInt16LE(offset);
      const length = buffer.readUInt32LE(offset + 2);
      assert.ok(length >= 6 && offset + length <= end);
      const chunk = { id, offset, start: offset + 6, end: offset + length };
      found.push(chunk);
      if ([0x4d4d, 0x3d3d, 0x4100, 0xafff, 0xb000, 0xb002].includes(id)) scan(chunk.start, chunk.end);
      else if (id === 0x4000) {
        let cursor = chunk.start;
        while (buffer[cursor++]);
        scan(cursor, chunk.end);
      } else if (id === 0x4120) {
        const count = buffer.readUInt16LE(chunk.start);
        scan(chunk.start + 2 + count * 8, chunk.end);
      }
      offset += length;
    }
  }
  scan(0, buffer.length);
  return found;
}
function stlBounds(buffer) {
  const count = buffer.readUInt32LE(80);
  assert.equal(buffer.length, 84 + count * 50);
  const bounds = new THREE.Box3();
  for (let face = 0; face < count; face++) for (let vertex = 0; vertex < 3; vertex++) {
    const offset = 84 + face * 50 + 12 + vertex * 12;
    const values = [0, 1, 2].map(axis => buffer.readFloatLE(offset + axis * 4));
    assert.ok(values.every(Number.isFinite));
    bounds.expandByPoint(new THREE.Vector3(...values));
  }
  return { bounds, triangles: count };
}

for (const file of manifest.files) {
  const buffer = await readFile(resolve(ASSETS, file.path));
  assert.equal(buffer.length, file.bytes);
  assert.equal(sha256(buffer), file.sha256);
  assert.equal(createHash('sha1').update(`blob ${buffer.length}\0`).update(buffer).digest('hex'), file.gitBlobSha);
  report.sourceFiles.push({ path: file.path, bytes: buffer.length, sha256: file.sha256, unchanged: true });
}
assert.equal(manifest.files[0].sha256, 'c87a1b853791f91a7908d77e05bbf5f87496b02016ea1040916e7cad5c1c70df');
const colors = new Set();
let totalVertices = 0;
let totalTriangles = 0;
let totalCollisionTriangles = 0;
for (let link = 0; link < 7; link++) {
  const raw = await readFile(resolve(ASSETS, `meshes/aubo_C5/visual/link${link}.3ds`));
  const before = sha256(raw);
  const chunks = sourceChunks(raw);
  const group = parseC5_3DS(raw, THREE);
  assert.equal(sha256(raw), before, 'parser must not mutate source input');
  assert.ok(group.isGroup && !group.isMesh, 'Group prevents URDFLoader white-material override');
  assert.equal(group.userData.c5SourceMaterials, true);
  assert.deepEqual(group.scale.toArray(), [1, 1, 1]);
  assert.equal(group.children.length, 1);
  const visual = group.children[0];
  assert.equal(visual.userData.c5SourceMaterials, true);
  const position = visual.geometry.attributes.position;
  const normal = visual.geometry.attributes.normal;
  const indices = visual.geometry.index.array;
  assert.equal(position.count, EXPECTED_VERTICES[link]);
  assert.equal(indices.length / 3, EXPECTED_TRIANGLES[link]);
  assert.ok([...position.array, ...normal.array].every(Number.isFinite));
  assert.ok([...indices].every(index => index >= 0 && index < position.count));
  const sourceVertices = chunks.find(chunk => chunk.id === 0x4110);
  let maxVertexError = 0;
  for (let index = 0; index < position.array.length; index++) {
    maxVertexError = Math.max(maxVertexError,
      Math.abs(position.array[index] - raw.readFloatLE(sourceVertices.start + 2 + index * 4) * 0.001));
  }
  assert.ok(maxVertexError < 6e-8, 'every source coordinate must be converted exactly once from mm to m');
  // Face ordering may change to group draw calls, but no source triangle may be
  // dropped, duplicated, rewound, or assigned to a different source material.
  const sourceFaces = chunks.find(chunk => chunk.id === 0x4120);
  const original = new Map();
  for (const faceGroup of chunks.filter(chunk => chunk.id === 0x4130)) {
    let cursor = faceGroup.start;
    while (raw[cursor++]);
    const name = raw.subarray(faceGroup.start, cursor - 1).toString();
    const count = raw.readUInt16LE(cursor);
    cursor += 2;
    for (let index = 0; index < count; index++) {
      const face = raw.readUInt16LE(cursor + index * 2);
      const offset = sourceFaces.start + 2 + face * 8;
      const key = `${name}/${[0, 1, 2].map(axis => raw.readUInt16LE(offset + axis * 2)).join(',')}`;
      original.set(key, (original.get(key) || 0) + 1);
    }
  }
  let covered = 0;
  for (const drawGroup of visual.geometry.groups) {
    assert.equal(drawGroup.start, covered);
    assert.equal(drawGroup.count % 3, 0);
    const material = visual.material[drawGroup.materialIndex];
    assert.ok(material.isMeshStandardMaterial);
    assert.ok(material.userData.c5SourceMaterials);
    const rgb = material.userData.c5SourceDiffuseSRGB;
    colors.add(rgb.join(','));
    const roundtrip = material.color.clone().convertLinearToSRGB().toArray();
    // Three's fast linear->sRGB approximation has ~0.002/255 round-trip error.
    roundtrip.forEach((value, axis) => assert.ok(Math.abs(value * 255 - rgb[axis]) < 0.003));
    for (let index = drawGroup.start; index < drawGroup.start + drawGroup.count; index += 3) {
      const key = `${material.name}/${[indices[index], indices[index + 1], indices[index + 2]].join(',')}`;
      assert.ok(original.get(key) > 0, 'rendered face must retain source material and winding');
      original.set(key, original.get(key) - 1);
    }
    covered += drawGroup.count;
  }
  assert.equal(covered, indices.length);
  assert.ok([...original.values()].every(value => value === 0));
  const visualBounds = new THREE.Box3().setFromObject(group);
  const collision = stlBounds(await readFile(resolve(ASSETS, `meshes/aubo_C5/collision/link${link}.STL`)));
  const visualSize = visualBounds.getSize(new THREE.Vector3());
  const collisionSize = collision.bounds.getSize(new THREE.Vector3());
  assert.ok(Math.max(...visualSize.toArray()) < 0.6 && Math.min(...visualSize.toArray()) > 0.02);
  for (const axis of ['x', 'y', 'z']) {
    assert.ok(Math.abs(visualBounds.min[axis] - collision.bounds.min[axis]) < 0.02, 'visual/collision coordinates share meter-scale frames');
    assert.ok(Math.abs(visualBounds.max[axis] - collision.bounds.max[axis]) < 0.02);
    assert.ok(visualSize[axis] / collisionSize[axis] > 0.75 && visualSize[axis] / collisionSize[axis] < 1.25);
  }
  totalVertices += position.count;
  totalTriangles += indices.length / 3;
  totalCollisionTriangles += collision.triangles;
  report.links.push({ link, vertices: position.count, visualTriangles: indices.length / 3,
    collisionTriangles: collision.triangles, maxMeterVertexError: maxVertexError,
    visualBounds: [visualBounds.min.toArray(), visualBounds.max.toArray()],
    collisionBounds: [collision.bounds.min.toArray(), collision.bounds.max.toArray()],
    materials: visual.material.map(material => ({ name: material.name, diffuseSRGB: material.userData.c5SourceDiffuseSRGB })),
    faceMaterialCounts: visual.userData.c5SourceFaceMaterialCounts, allFacesPreserved: true });
  visual.geometry.dispose();
  visual.material.forEach(material => material.dispose());
}
assert.equal(totalVertices, 14862);
assert.equal(totalTriangles, 15469);
assert.equal(totalCollisionTriangles, 3140);
for (const rgb of ['127,127,127', '217,216,233', '24,76,198', '185,185,185']) assert.ok(colors.has(rgb));
const base = await readFile(resolve(ASSETS, 'meshes/aubo_C5/visual/link2.3ds'));
const chunks = sourceChunks(base);
function reject(name, mutate) {
  let input = Buffer.from(base);
  input = mutate(input) || input;
  assert.throws(() => parseC5_3DS(input), /C5 3DS:/, name);
  report.negativeInputs.push({ name, rejected: true });
}
reject('truncated input', buffer => buffer.subarray(0, 21));
reject('root size overflow', buffer => { buffer.writeUInt32LE(buffer.length + 1, 2); });
reject('non-finite vertex', buffer => { buffer.writeFloatLE(NaN, chunks.find(chunk => chunk.id === 0x4110).start + 2); });
reject('out of range face vertex', buffer => { buffer.writeUInt16LE(65535, chunks.find(chunk => chunk.id === 0x4120).start + 2); });
reject('out of range face-material assignment', buffer => {
  const group = chunks.find(chunk => chunk.id === 0x4130);
  let cursor = group.start;
  while (buffer[cursor++]);
  buffer.writeUInt16LE(65535, cursor + 2);
});
reject('non-identity mesh matrix', buffer => { buffer.writeFloatLE(2, chunks.find(chunk => chunk.id === 0x4160).start); });
reject('non-unit master scale', buffer => { buffer.writeFloatLE(2, chunks.find(chunk => chunk.id === 0x0100).start); });
reject('non-zero keyframe pivot', buffer => { buffer.writeFloatLE(1, chunks.find(chunk => chunk.id === 0xb013).start); });
reject('animated keyframe', buffer => { buffer.writeUInt32LE(1, chunks.find(chunk => chunk.id === 0xb020).start + 10); });
reject('external texture', buffer => {
  const addition = Buffer.alloc(6);
  addition.writeUInt16LE(0xa200, 0);
  addition.writeUInt32LE(6, 2);
  const combined = Buffer.concat([buffer, addition]);
  combined.writeUInt32LE(combined.length, 2);
  return combined;
});
report.ok = true;
report.colors = [...colors];
report.totalVertices = totalVertices;
report.totalVisualTriangles = totalTriangles;
report.totalCollisionTriangles = totalCollisionTriangles;
report.sourceFilesUnchanged = report.sourceFiles.length;
const output = resolve(ROOT, 'work/validation/c5/visuals-final.json');
await mkdir(dirname(output), { recursive: true });
await writeFile(output, `${JSON.stringify(report, null, 2)}\n`);
console.log(JSON.stringify({ ok: true, sourceFilesUnchanged: report.sourceFiles.length, links: report.links.length,
  vertices: totalVertices, triangles: totalTriangles, negativeInputs: report.negativeInputs.length,
  output: 'work/validation/c5/visuals-final.json' }));
