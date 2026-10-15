import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile, stat} from 'node:fs/promises';

// Dependency-free, read-only publication checks. This does not instantiate the
// physics runtime or certify rendered pixels, interaction, or successful tasks.
const root = new URL('../dist/', import.meta.url);
const bases = ['http://127.0.0.1:4173/', 'https://wander521.github.io/r1-simulator/'];
const cached = new Map(), directories = new Set(), checks = [], edges = [], exclusions = [];
const sha = data => createHash('sha256').update(data).digest('hex');
const stripQuery = path => path.replace(/[?#].*$/, '');
async function contents(path) {
  if (!cached.has(path)) cached.set(path, await readFile(new URL(path, root)));
  assert(cached.get(path).length > 0, `Empty resource: ${path}`);
  return cached.get(path);
}
async function text(path) { return (await contents(path)).toString('utf8'); }
async function exists(path) {
  const info = await stat(new URL(path, root));
  if (info.isDirectory()) directories.add(path);
  else await contents(path);
  return info;
}
function check(name, detail = {}) { checks.push({name, pass: true, ...detail}); }

async function reference(from, specifier, kind) {
  assert(specifier && !specifier.startsWith('/') && !specifier.includes('\\'), `Non-portable ${kind}: ${from} -> ${specifier}`);
  assert(!/^[a-z][a-z\d+.-]*:/i.test(specifier), `External ${kind}: ${from} -> ${specifier}`);
  const paths = [];
  for (const base of bases) {
    const project = new URL(base), resolved = new URL(specifier, new URL(from, project));
    assert.equal(resolved.origin, project.origin, `${kind} escaped origin: ${specifier}`);
    assert(resolved.pathname.startsWith(project.pathname), `${kind} escaped project base: ${from} -> ${specifier}`);
    const relative = decodeURIComponent(resolved.pathname.slice(project.pathname.length));
    assert(relative && !relative.split('/').includes('..'), `Invalid project path: ${relative}`);
    await exists(relative);
    paths.push(relative);
  }
  assert.equal(paths[0], paths[1], `Different local and GitHub resolution: ${specifier}`);
  edges.push({from, specifier, kind, to: paths[0]});
  return paths[0];
}

// Lex tokens instead of searching raw source: comments, shader templates, and
// quoted prose must not be mistaken for actual module import statements.
function tokens(source) {
  const out = []; let i = 0, previous = '';
  const push = (type, value) => { out.push({type, value}); previous = value; };
  while (i < source.length) {
    const c = source[i], n = source[i + 1];
    if (/\s/.test(c)) { i++; continue; }
    if (c === '/' && n === '/') { i = source.indexOf('\n', i + 2); if (i < 0) break; continue; }
    if (c === '/' && n === '*') { const end = source.indexOf('*/', i + 2); assert(end >= 0, 'Unclosed JavaScript comment'); i = end + 2; continue; }
    if (c === '"' || c === "'") {
      const quote = c; let value = ''; i++;
      while (i < source.length && source[i] !== quote) {
        if (source[i] === '\\') {
          i++;
          const escape = source[i++];
          if (escape === '\n') continue;
          if (escape === '\r') { if (source[i] === '\n') i++; continue; }
          if (escape === 'x' || escape === 'u') {
            const length = escape === 'x' ? 2 : 4;
            if (escape === 'u' && source[i] === '{') {
              const end = source.indexOf('}', ++i); assert(end >= 0, 'Unclosed Unicode escape');
              value += String.fromCodePoint(parseInt(source.slice(i, end), 16)); i = end + 1;
            } else { value += String.fromCodePoint(parseInt(source.slice(i, i + length), 16)); i += length; }
          } else value += ({n: '\n', r: '\r', t: '\t', b: '\b', f: '\f', v: '\v', 0: '\0'})[escape] ?? escape;
        } else value += source[i++];
      }
      assert.equal(source[i++], quote, 'Unclosed JavaScript string'); push('string', value); continue;
    }
    if (c === '`') {
      // No production import/URL resource in this graph is inside a template.
      // Templates are opaque; dynamic computed resource construction is outside
      // this deliberately static test and is disclosed in the report scope.
      i++; while (i < source.length) { if (source[i] === '\\') i += 2; else if (source[i++] === '`') break; }
      push('template', '<template>'); continue;
    }
    if (c === '/' && (!previous || /^(?:=|\(|\[|\{|,|:|;|!|\?|\||&|return|throw|case|=>)$/.test(previous))) {
      let characterClass = false; i++;
      while (i < source.length) {
        if (source[i] === '\\') { i += 2; continue; }
        if (source[i] === '[') characterClass = true;
        else if (source[i] === ']') characterClass = false;
        else if (source[i] === '/' && !characterClass) { i++; break; }
        i++;
      }
      while (/[a-z]/i.test(source[i] || '') && i < source.length) i++;
      push('regex', '<regex>'); continue;
    }
    if (/[a-zA-Z_$]/.test(c)) { const start = i++; while (/[a-zA-Z\d_$]/.test(source[i] || '') && i < source.length) i++; push('word', source.slice(start, i)); continue; }
    push('punctuation', c); i++;
  }
  return out;
}

function moduleReferences(source) {
  const lex = tokens(source), found = [];
  const value = i => lex[i]?.value;
  const literal = i => lex[i]?.type === 'string';
  for (let i = 0; i < lex.length; i++) {
    const current = value(i);
    if ((current === 'import' || current === 'export') && value(i - 1) !== '.') {
      if (current === 'import' && value(i + 1) === '(' && literal(i + 2) && value(i + 3) === ')') {
        found.push({specifier: value(i + 2), kind: 'dynamic-import'}); continue;
      }
      if (current === 'import' && literal(i + 1)) found.push({specifier: value(i + 1), kind: 'static-import'});
      else if (value(i + 1) !== '.' && value(i + 1) !== '(') {
        for (let j = i + 1; j < lex.length && value(j) !== ';'; j++) {
          if (value(j) === 'from' && literal(j + 1)) { found.push({specifier: value(j + 1), kind: 'static-import'}); break; }
          if (['const', 'let', 'class', 'function', 'default'].includes(value(j))) break;
        }
      }
    }
    if (current === 'new' && value(i + 1) === 'URL' && value(i + 2) === '(' && literal(i + 3) &&
        value(i + 4) === ',' && ['import', '.', 'meta', '.', 'url'].every((v, j) => value(i + 5 + j) === v))
      found.push({specifier: value(i + 3), kind: 'import-meta-url'});
  }
  return found;
}

const lexicalFixture = moduleReferences(`// import 'missing-comment.js';
  const prose = "import 'missing-prose.js'";
  import './a.mjs'; import { x } from '../b.mjs'; export { x } from './c.mjs';
  await import('./d.mjs'); new URL('./image.png', import.meta.url);
  const shader = \`import 'missing-shader.js'\`;
`);
assert.deepEqual(lexicalFixture.map(item => item.specifier), ['./a.mjs', '../b.mjs', './c.mjs', './d.mjs', './image.png']);
check('Static reference scanner ignores comments, strings and shader templates');

const home = await text('index.html');
assert(/url=\.\/multi\//.test(home));
assert(/location\.replace\(['"]\.\/multi\/['"]\)/.test(home));
assert(/href=['"]\.\/multi\/['"]/.test(home));
await reference('index.html', './multi/', 'main-entry');
check('Main redirect and fallback use ./multi/ at both project bases');

const importMap = new Map(), entryModules = [];
for (const page of ['multi/index.html', 'r1/index.html', 'g1/index.html', 'c5/index.html']) {
  const html = await text(page); assert(/lang=['"]zh-CN['"]/.test(html), `${page} language missing`);
  const maps = [...html.matchAll(/<script\b[^>]*type=['"]importmap['"][^>]*>([\s\S]*?)<\/script>/gi)];
  for (const map of maps) {
    const imports = JSON.parse(map[1]).imports;
    for (const [name, target] of Object.entries(imports)) {
      const path = await reference(page, target, 'import-map');
      if (page.startsWith('multi/')) importMap.set(name, path);
    }
  }
  for (const resource of html.matchAll(/<(script|link)\b[^>]*(?:src|href)=['"]([^'"]+)['"][^>]*>/gi)) {
    const path = await reference(page, resource[2], 'html-resource');
    if (page.startsWith('multi/') && resource[1].toLowerCase() === 'script' && /type=['"]module['"]/.test(resource[0])) entryModules.push(path);
  }
  check(`${page}: Chinese single page and local script/style/import map resources retained`);
}
assert.equal(entryModules.length, 1); assert(importMap.has('three'));

const queue = [...entryModules], modules = new Set();
while (queue.length) {
  const path = queue.shift(); if (modules.has(path)) continue; modules.add(path);
  const source = await text(path);
  for (const item of moduleReferences(source)) {
    if (item.specifier.startsWith('node:')) {
      assert.equal(path, 'r1/mujoco-runtime.mjs'); assert.equal(item.specifier, 'node:module');
      exclusions.push({from: path, specifier: item.specifier, reason: 'Emscripten Node-only conditional branch, not a browser fetch'}); continue;
    }
    let target;
    if (/^\.?\.\//.test(item.specifier)) target = await reference(path, item.specifier, item.kind);
    else {
      assert(importMap.has(item.specifier), `Unmapped bare browser import: ${path} -> ${item.specifier}`);
      target = importMap.get(item.specifier); edges.push({from: path, ...item, to: target, importMap: true});
    }
    if (item.kind.endsWith('import')) queue.push(stripQuery(target));
  }
}
assert(modules.has('multi/robot-runtime.mjs') && modules.has('vendor/URDFLoader.js'));
for (const id of ['r1', 'g1', 'c5']) assert(modules.has(`${id}/grasp-controller.mjs`) && modules.has(`${id}/${id}-dynamics.mjs`));
check('Complete multi browser module graph resolves under local and GitHub project bases', {modules: modules.size});
check('Literal dynamic URDFLoader import is included in the checked module graph');
check('Bare Three.js loader imports resolve through multi import map');

const runtime = await text('multi/robot-runtime.mjs');
assert(/const url\s*=\s*path\s*=>\s*new URL\(path,\s*import\.meta\.url\)\.href/.test(runtime), 'Asset helper must remain import.meta.url-relative');
const helperResources = [...new Set([...runtime.matchAll(/\burl\(\s*(['"])([^'"]+)\1\s*\)/g)].map(match => match[2]))];
for (const specifier of helperResources) await reference('multi/robot-runtime.mjs', specifier, 'module-relative-asset-helper');
check('Robot URDF/MJCF/package/WASM helper arguments are project-relative', {resources: helperResources.length});

const robots = [
  {id: 'r1', urdf: 'r1/assets/r1.urdf', model: 'r1/models/r1-dynamic-scene.xml'},
  {id: 'g1', urdf: 'g1/assets/g1_29dof_with_dex1_1.urdf', model: 'g1/models/g1-dynamic-scene.xml'},
  {id: 'c5', urdf: 'c5/assets/c5-dex1.urdf', model: 'c5/models/c5-dynamic-scene.xml'},
];
const meshCounts = {};
for (const robot of robots) {
  assert(helperResources.includes('../' + robot.urdf) && helperResources.includes('../' + robot.model), `${robot.id} configured source paths changed`);
  assert(/<mujoco\b/.test(await text(robot.model)), `${robot.model} is not a MJCF scene`);
  const urdf = await text(robot.urdf); assert(/<robot\b/.test(urdf));
  const meshes = [...new Set([...urdf.matchAll(/<mesh\b[^>]*filename=['"]([^'"]+)['"]/g)].map(match => match[1]))];
  assert(meshes.length > 0, `${robot.id} has no source meshes`);
  for (let mesh of meshes) {
    if (mesh.startsWith('package://')) {
      assert.equal(robot.id, 'r1'); assert(mesh.startsWith('package://r1_v2_1_0/'));
      assert(helperResources.includes('../r1/assets'), 'R1 package base missing');
      mesh = './' + mesh.slice('package://r1_v2_1_0/'.length);
    }
    await reference(robot.urdf, mesh, 'urdf-mesh');
  }
  meshCounts[robot.id] = meshes.length;
  check(`${robot.id.toUpperCase()} original standalone core scene and all source mesh references retained`, {meshes: meshes.length});
}

const wasmPath = 'Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', wasm = await contents(wasmPath);
assert.deepEqual([...wasm.subarray(0, 8)], [0, 97, 115, 109, 1, 0, 0, 0]);
assert(WebAssembly.validate(wasm), 'Invalid MuJoCo WebAssembly binary');
assert(helperResources.includes('../' + wasmPath), 'Multi page must locate bundled, relative MuJoCo binary');
assert(edges.some(edge => edge.from === 'r1/mujoco-runtime.mjs' && edge.to === wasmPath && edge.kind === 'import-meta-url'), 'Runtime fallback WASM path not checked');
check('Bundled WASM magic, version, binary validity and both module-relative loading paths', {bytes: wasm.length});

for (const path of [...cached.keys()].filter(path => path.endsWith('.css'))) {
  for (const match of (await text(path)).matchAll(/url\(\s*['"]?([^'"\s)]+)['"]?\s*\)/g)) await reference(path, match[1], 'css-resource');
}
check('Checked page stylesheets have no missing or root-absolute URL assets');

const sourceHashes = {};
for (const [path, bytes] of [...cached].sort(([a], [b]) => a.localeCompare(b))) {
  const key = 'dist/' + path; sourceHashes[key] = sha(bytes);
  assert.equal(sha(await readFile(new URL(path, root))), sourceHashes[key], `Source changed while this check ran: ${key}`);
}
sourceHashes['work/test-multi-pages.mjs'] = sha(await readFile(new URL(import.meta.url)));
check('All inspected source and asset bytes stayed frozen during the check', {files: cached.size});

console.log(JSON.stringify({
  pass: true, generatedAt: new Date().toISOString(), checks: checks.length, results: checks,
  scope: 'Static publication and path integrity only. No browser, rendered-pixel, pointer/touch, runtime loading, physics, grasp, or synchronization acceptance is claimed. JavaScript templates and computed non-literal resources are not statically interpreted; the known import.meta.url helper is checked through every literal call argument.',
  bases, moduleCount: modules.size, modules: [...modules].sort(), resourceEdges: edges.length,
  edges, meshCounts, wasmBytes: wasm.length, checkedFiles: cached.size, checkedDirectories: [...directories].sort(),
  exclusions, sourceHashes,
}, null, 2));
