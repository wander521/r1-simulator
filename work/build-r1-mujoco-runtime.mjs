import { readFile, writeFile } from 'node:fs/promises';

const sourcePath = new URL('../dist/Rebot_Arm_AGV/assets/mujoco-DYuOvsQB.js', import.meta.url);
const destinationPath = new URL('../dist/r1/mujoco-runtime.mjs', import.meta.url);
const source = await readFile(sourcePath, 'utf8');

// The generated Emscripten wrapper imports Vite's preload helper only for its
// Node fallback.  Keeping a tiny local replacement prevents importing the
// reference page's application bundle when this runtime is used by R1.
const runtime = source.replace(
  'import{_ as Gc}from"./index-BXF1-FJz.js";',
  'const Gc=async()=>await import("node:module");',
);
if (runtime === source) throw new Error('MuJoCo runtime import signature changed; aborting extraction.');
await writeFile(destinationPath, runtime);
console.log(`Wrote ${destinationPath.pathname}`);
