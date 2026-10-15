import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';

// Import only the requested C5, not the large multi-model upstream repository.
// C5_30's mesh directory aliases i5 upstream; it is deliberately not used here.
const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const ASSETS = resolve(ROOT, 'dist/c5/assets');
const REPOSITORY = 'AuboRobot/aubo_description';
const COMMIT = '47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1';
const FILES = [
  ['urdf/aubo_C5.urdf', 'aubo_C5.urdf', 9128, 'eaa931d3449fec363693e69b093bbd5b6ab1a381'],
  ['urdf/aubo_C5.srdf', 'aubo_C5.srdf', 2493, '83705489b8b38ec3318f27e3d24e74cf5b39aaad'],
  ['package.xml', 'source/package.xml', 1201, '67d3dd5222e85fe87ea046acd5c8833d9c6e693f'],
  ['README.md', 'source/README.md', 13054, '380c93076c5bc8f22d3c0ec69d053bb31fd9f248'],
  ['meshes/aubo_C5/collision/link0.STL', 'meshes/aubo_C5/collision/link0.STL', 14184, '8ee490cced5acc62c85b23bbe88624ce399309fd'],
  ['meshes/aubo_C5/collision/link1.STL', 'meshes/aubo_C5/collision/link1.STL', 23384, '00bf11a8b41728f098f46339baf0ae5208bbe346'],
  ['meshes/aubo_C5/collision/link2.STL', 'meshes/aubo_C5/collision/link2.STL', 52284, '44fedeb0098a1ad422c221778a4db51200974ba7'],
  ['meshes/aubo_C5/collision/link3.STL', 'meshes/aubo_C5/collision/link3.STL', 18184, '35e7f691864f0a3b1e145ce3477e858e6ffc777a'],
  ['meshes/aubo_C5/collision/link4.STL', 'meshes/aubo_C5/collision/link4.STL', 17784, '36a9e4a54f321518714369aacdf0fe49e6ed9e37'],
  ['meshes/aubo_C5/collision/link5.STL', 'meshes/aubo_C5/collision/link5.STL', 17784, 'e527bb6be9593812604c0c25891e9d25298393ef'],
  ['meshes/aubo_C5/collision/link6.STL', 'meshes/aubo_C5/collision/link6.STL', 13984, '2e9c032e5ef9830d13190285898f0b11f74971cc'],
  ['meshes/aubo_C5/visual/link0.3ds', 'meshes/aubo_C5/visual/link0.3ds', 17911, '02a71f11c9a2ece3e9d15f80407ae56cd24ebf73'],
  ['meshes/aubo_C5/visual/link1.3ds', 'meshes/aubo_C5/visual/link1.3ds', 63725, '1753f68ac64e21343afdb72901e279e3285efcd0'],
  ['meshes/aubo_C5/visual/link2.3ds', 'meshes/aubo_C5/visual/link2.3ds', 125503, '6abea5a95ce59481a9a93f972b27efe2030d9160'],
  ['meshes/aubo_C5/visual/link3.3ds', 'meshes/aubo_C5/visual/link3.3ds', 103543, 'e8aa845853d3da5f0aeaeaaf3a6413ed507952c6'],
  ['meshes/aubo_C5/visual/link4.3ds', 'meshes/aubo_C5/visual/link4.3ds', 62603, '1d8cafa4612b43e023edb48dba1b2d011df1ddba'],
  ['meshes/aubo_C5/visual/link5.3ds', 'meshes/aubo_C5/visual/link5.3ds', 62603, '084a71964b9e0d719b0291e0fbf28a6d926ffcd6'],
  ['meshes/aubo_C5/visual/link6.3ds', 'meshes/aubo_C5/visual/link6.3ds', 83527, '99083a093d9232135a340241e500cc5688483e24'],
];

function sha256(buffer) { return createHash('sha256').update(buffer).digest('hex'); }
function blobSha(buffer) { return createHash('sha1').update(`blob ${buffer.length}\0`).update(buffer).digest('hex'); }
function validate(buffer, [sourcePath, , bytes, gitBlobSha]) {
  if (buffer.length !== bytes || blobSha(buffer) !== gitBlobSha) {
    throw new Error(`Upstream byte identity mismatch: ${sourcePath}`);
  }
}

if (process.argv.includes('--verify')) {
  const manifest = JSON.parse(await readFile(resolve(ASSETS, 'SOURCE_MANIFEST.json'), 'utf8'));
  if (manifest.repository !== REPOSITORY || manifest.commit !== COMMIT || manifest.files.length !== FILES.length) {
    throw new Error('C5 source manifest does not match the pinned source selection');
  }
  for (const file of FILES) {
    const [, path] = file;
    const buffer = await readFile(resolve(ASSETS, path));
    validate(buffer, file);
    const record = manifest.files.find(item => item.path === path);
    if (!record || record.sha256 !== sha256(buffer) || record.gitBlobSha !== file[3]) {
      throw new Error(`C5 SHA-256 manifest mismatch: ${path}`);
    }
  }
  console.log(JSON.stringify({ ok: true, repository: REPOSITORY, commit: COMMIT, files: FILES.length,
    bytes: FILES.reduce((sum, file) => sum + file[2], 0), originalUrdfSha256: manifest.files[0].sha256 }));
} else {
  const records = [];
  for (const file of FILES) {
    const [sourcePath, path, bytes, gitBlobSha] = file;
    // GitHub blob API has stable binary/base64 semantics, including CRLF in URDF.
    const response = await fetch(`https://api.github.com/repos/${REPOSITORY}/git/blobs/${gitBlobSha}`,
      { headers: { Accept: 'application/vnd.github+json', 'User-Agent': 'r1-simulator-c5-source-import' } });
    if (!response.ok) throw new Error(`C5 source HTTP ${response.status}: ${sourcePath}`);
    const blob = await response.json();
    if (blob.encoding !== 'base64') throw new Error(`Unexpected source encoding: ${sourcePath}`);
    const buffer = Buffer.from(blob.content, 'base64');
    validate(buffer, file);
    await mkdir(dirname(resolve(ASSETS, path)), { recursive: true });
    await writeFile(resolve(ASSETS, path), buffer);
    records.push({ path, sourcePath, bytes, gitBlobSha, sha256: sha256(buffer),
      sourceUrl: `https://github.com/${REPOSITORY}/blob/${COMMIT}/${sourcePath}` });
    console.log(`Imported ${path} (${bytes} bytes)`);
  }
  await writeFile(resolve(ASSETS, 'SOURCE_MANIFEST.json'), `${JSON.stringify({
    repository: REPOSITORY, commit: COMMIT,
    officialPackageVersion: '1.3.3', model: 'aubo_C5',
    license: {
      declaration: 'BSD', source: 'source/package.xml',
      exactVariant: 'Not specified by upstream package.xml',
      fullLicenseTextPresentInSourceRepository: false,
      note: 'Upstream declares BSD but supplies no LICENSE/COPYING/NOTICE file or model copyright header. No additional permissions or exact BSD variant are asserted here.',
    },
    upstreamVisualReferenceIssue: 'Original C5 URDF references visual/link0.DAE through link6.DAE, but the pinned C5 mesh directory contains only link0.3ds through link6.3ds. Preserve this URDF verbatim; repair mesh URLs only in the explicitly derived simulation/visualization model.',
    aliasNotUsed: 'meshes/aubo_C5_30 is an upstream symlink to aubo_i5. This import selects meshes/aubo_C5 directly, not C5_30 or i5 meshes.',
    gripper: 'Not present in the official C5 URDF/SRDF. Any project-specific end effector must retain its own provenance and be labelled as a simulation integration.',
    files: records,
  }, null, 2)}\n`);
}
