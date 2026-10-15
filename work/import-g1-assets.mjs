import {readFile,writeFile,mkdir,copyFile,rename,stat,unlink} from 'node:fs/promises';
import {spawn} from 'node:child_process';
import {createHash} from 'node:crypto';
import {dirname,basename,resolve,join} from 'node:path';
import {fileURLToPath} from 'node:url';

// Import only the exact user-requested portable URDF and its referenced meshes.
// Original bytes are retained. No source MJCF or alternate robot is imported.
export const HF_REVISION='68459ed68f6f68e1f661091dfcb6ebce44681aec';
export const UNITREE_LICENSE_REVISION='5994d4faef0a9cadd3287f8de0199a67eeb2a259';
export const UNITREE_AVP_REVISION='817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b';
const RECORDS=[
  {
    "path": "g1_29dof_with_dex1_1.urdf",
    "bytes": 39614,
    "sha256": "0a25ea0ed77740ca66af74a9f3c2dae649324a369351bf2f6d5cab3555b5f059",
    "gitBlobSHA1": "eb12c1ee5281f65b8df79c478b47021a8f6de7c8",
    "sourcePath": "assets/g1_29dof_with_dex1_1.urdf",
    "cacheName": "urdf"
  },
  {
    "path": "dex1_1/meshes/Dex1_base_link.STL",
    "bytes": 2824984,
    "sha256": "03f1cb02f94d6b280953f6c5a10da2eab6220ea5c301408d3c4decca66c8cdad",
    "gitBlobSHA1": "054f3a9f02891b2441b025ede6c701825cda3cee",
    "sourcePath": "assets/dex1_1/meshes/Dex1_base_link.STL"
  },
  {
    "path": "dex1_1/meshes/Dex1_finger_link_1.STL",
    "bytes": 313884,
    "sha256": "7cd2ff199556e3955547a34c75ed9ea777fb7d704fc98e7318d0f67e4aaedf5a",
    "gitBlobSHA1": "5c191a8c15c5294233c5f0f698f7376b0cf0d77f",
    "sourcePath": "assets/dex1_1/meshes/Dex1_finger_link_1.STL"
  },
  {
    "path": "dex1_1/meshes/Dex1_finger_link_2.STL",
    "bytes": 313884,
    "sha256": "e8297ce8184c64d34d70cd9d6c7b583fba6a3adae2075523e97fb956ada65eb3",
    "gitBlobSHA1": "66e501f16cd67eb03ef0426a97a9de66591d8f74",
    "sourcePath": "assets/dex1_1/meshes/Dex1_finger_link_2.STL"
  },
  {
    "path": "dex1_1/meshes/dex1_col_1.stl",
    "bytes": 75884,
    "sha256": null,
    "gitBlobSHA1": "7066aa69ca07ffc5ec9e4b6f7bc3fbbdeb589f0d",
    "sourcePath": "assets/dex1_1/meshes/dex1_col_1.stl"
  },
  {
    "path": "dex1_1/meshes/dex1_col_2.stl",
    "bytes": 72884,
    "sha256": null,
    "gitBlobSHA1": "fb49684490facc26bf2ad773ecec7cb50aec84c8",
    "sourcePath": "assets/dex1_1/meshes/dex1_col_2.stl"
  },
  {
    "path": "meshes/head_link.STL",
    "bytes": 932784,
    "sha256": "005fb67fbd3eff94aa8bf4a6e83238174e9f91b6721f7111594322f223724411",
    "gitBlobSHA1": "401f822751c33dab53c6cee77eb62890aca5cd8f",
    "sourcePath": "assets/meshes/head_link.STL"
  },
  {
    "path": "meshes/left_ankle_pitch_link.STL",
    "bytes": 71184,
    "sha256": null,
    "gitBlobSHA1": "69de8490184afc698f633e34bb74e65351bd4689",
    "sourcePath": "assets/meshes/left_ankle_pitch_link.STL"
  },
  {
    "path": "meshes/left_ankle_roll_link.STL",
    "bytes": 653384,
    "sha256": "c4092af943141d4d9f74232f3cfa345afc6565f46a077793b8ae0e68b39dc33f",
    "gitBlobSHA1": "cb69f65c6a8059fc6720cb393540f579b4d0c26e",
    "sourcePath": "assets/meshes/left_ankle_roll_link.STL"
  },
  {
    "path": "meshes/left_elbow_link.STL",
    "bytes": 88784,
    "sha256": null,
    "gitBlobSHA1": "1a96d99ba469960173129084ab6dd3bf8a732a71",
    "sourcePath": "assets/meshes/left_elbow_link.STL"
  },
  {
    "path": "meshes/left_hip_pitch_link.STL",
    "bytes": 181684,
    "sha256": "4725168105ee768ee31638ef22b53f6be2d7641bfd7cfefe803488d884776fa4",
    "gitBlobSHA1": "4cf7475bf6bbaef47d5b3f7644a34c73b3d64037",
    "sourcePath": "assets/meshes/left_hip_pitch_link.STL"
  },
  {
    "path": "meshes/left_hip_roll_link.STL",
    "bytes": 192184,
    "sha256": "91f25922ee8a7c3152790051bebad17b4d9cd243569c38fe340285ff93a97acf",
    "gitBlobSHA1": "585f6040c38571177d55519417a0f46ca7b29381",
    "sourcePath": "assets/meshes/left_hip_roll_link.STL"
  },
  {
    "path": "meshes/left_hip_yaw_link.STL",
    "bytes": 296284,
    "sha256": "a16d88aa6ddac8083aa7ad55ed317bea44b1fa003d314fba88965b7ed0f3b55b",
    "gitBlobSHA1": "b46a74134f90c568cbdfb8484eeb83a0503b4504",
    "sourcePath": "assets/meshes/left_hip_yaw_link.STL"
  },
  {
    "path": "meshes/left_knee_link.STL",
    "bytes": 854884,
    "sha256": "8d92b9e3d3a636761150bb8025e32514c4602b91c7028d523ee42b3e632de477",
    "gitBlobSHA1": "2dcf84e4018ab9e4cff48cecdafe433601c48480",
    "sourcePath": "assets/meshes/left_knee_link.STL"
  },
  {
    "path": "meshes/left_shoulder_pitch_link.STL",
    "bytes": 176784,
    "sha256": "f0d1cfd02fcf0d42f95e678eeca33da3afbcc366ffba5c052847773ec4f31d52",
    "gitBlobSHA1": "926d9807f1cbf6ad0d41170904785eb30427c848",
    "sourcePath": "assets/meshes/left_shoulder_pitch_link.STL"
  },
  {
    "path": "meshes/left_shoulder_roll_link.STL",
    "bytes": 400284,
    "sha256": "fb9df21687773522598dc384f1a2945c7519f11cbc8bd372a49170316d6eee88",
    "gitBlobSHA1": "4c6840b93ea15d1ba7b4dac0f18f94655867cab7",
    "sourcePath": "assets/meshes/left_shoulder_roll_link.STL"
  },
  {
    "path": "meshes/left_shoulder_yaw_link.STL",
    "bytes": 249184,
    "sha256": "1aa97e9748e924336567992181f78c7cd0652fd52a4afcca3df6b2ef6f9e712e",
    "gitBlobSHA1": "89b0e0661275ed832c674a46893239ad2e5bef85",
    "sourcePath": "assets/meshes/left_shoulder_yaw_link.STL"
  },
  {
    "path": "meshes/left_wrist_pitch_link.STL",
    "bytes": 85984,
    "sha256": null,
    "gitBlobSHA1": "82cc224a8e41251d879502f9809e31d0988ec7f9",
    "sourcePath": "assets/meshes/left_wrist_pitch_link.STL"
  },
  {
    "path": "meshes/left_wrist_roll_link.STL",
    "bytes": 356184,
    "sha256": "edc387c9a0ba8c2237e9b296d32531426fabeb6f53e58df45c76106bca74148c",
    "gitBlobSHA1": "2097ca3e5bbc0af985d150e5dcda4d8ba9bcf43f",
    "sourcePath": "assets/meshes/left_wrist_roll_link.STL"
  },
  {
    "path": "meshes/left_wrist_yaw_link.STL",
    "bytes": 318684,
    "sha256": "83f8fb3a726bf9613d65dd14f0f447cb918c3c95b3938042a0c9c09749267d3b",
    "gitBlobSHA1": "692f4b07105fa0ec4ffd989dc60be9b2a33ac9c3",
    "sourcePath": "assets/meshes/left_wrist_yaw_link.STL"
  },
  {
    "path": "meshes/logo_link.STL",
    "bytes": 243384,
    "sha256": "8571a0a19bc4916fa55f91449f51d5fdefd751000054865a842449429d5f155b",
    "gitBlobSHA1": "6c25961c594c714ada9d32e922250d326c121f4a",
    "sourcePath": "assets/meshes/logo_link.STL"
  },
  {
    "path": "meshes/pelvis.STL",
    "bytes": 1060884,
    "sha256": "5ba6bbc888e630550140d3c26763f10206da8c8bd30ed886b8ede41c61f57a31",
    "gitBlobSHA1": "f98a88dbf6c7698791ba694e16cbcc180c40663c",
    "sourcePath": "assets/meshes/pelvis.STL"
  },
  {
    "path": "meshes/pelvis_contour_link.STL",
    "bytes": 1805184,
    "sha256": "5cc5c2c7a312329e3feeb2b03d3fc09fc29705bd01864f6767e51be959662420",
    "gitBlobSHA1": "8025bc070cd65550f345e9e97bc44a8ff9740dea",
    "sourcePath": "assets/meshes/pelvis_contour_link.STL"
  },
  {
    "path": "meshes/right_ankle_pitch_link.STL",
    "bytes": 71184,
    "sha256": null,
    "gitBlobSHA1": "e77d8a2fe1e5d56fac049833d254d6ffa4f6b350",
    "sourcePath": "assets/meshes/right_ankle_pitch_link.STL"
  },
  {
    "path": "meshes/right_ankle_roll_link.STL",
    "bytes": 653784,
    "sha256": "4b66222ea56653e627711b56d0a8949b4920da5df091da0ceb343f54e884e3a5",
    "gitBlobSHA1": "3c38507b9060eda2c40e5e2bb1b174ef5bda0390",
    "sourcePath": "assets/meshes/right_ankle_roll_link.STL"
  },
  {
    "path": "meshes/right_elbow_link.STL",
    "bytes": 88784,
    "sha256": null,
    "gitBlobSHA1": "f259e3812efb9985d6463c04d3e8a4b53793a699",
    "sourcePath": "assets/meshes/right_elbow_link.STL"
  },
  {
    "path": "meshes/right_hip_pitch_link.STL",
    "bytes": 181284,
    "sha256": "e4f3c99d7f4a7d34eadbef9461fc66e3486cb5442db1ec50c86317d459f1a9c6",
    "gitBlobSHA1": "064085fca108bfc2a5a365403ceb6cdac4d5cd5f",
    "sourcePath": "assets/meshes/right_hip_pitch_link.STL"
  },
  {
    "path": "meshes/right_hip_roll_link.STL",
    "bytes": 192684,
    "sha256": "4c254ef66a356f492947f360dd931965477b631e3fcc841f91ccc46d945d54f6",
    "gitBlobSHA1": "6544025eeb7dc7d09da549746ae28272e5bcf224",
    "sourcePath": "assets/meshes/right_hip_roll_link.STL"
  },
  {
    "path": "meshes/right_hip_yaw_link.STL",
    "bytes": 296284,
    "sha256": "e479c2936ca2057e9eb2f7dff6c189b7419d7b8484dea0b298cbb36a2a6aa668",
    "gitBlobSHA1": "0ad7bee36419f85b490debe929957c6932df393f",
    "sourcePath": "assets/meshes/right_hip_yaw_link.STL"
  },
  {
    "path": "meshes/right_knee_link.STL",
    "bytes": 852284,
    "sha256": "63c4008449c9bbe701a6e2b557b7a252e90cf3a5abcf54cee46862b9a69f8ec8",
    "gitBlobSHA1": "65e8a708a7e86f1f242cd32e8959a560ea66c2d7",
    "sourcePath": "assets/meshes/right_knee_link.STL"
  },
  {
    "path": "meshes/right_shoulder_pitch_link.STL",
    "bytes": 176784,
    "sha256": "24cdb387e0128dfe602770a81c56cdce3a0181d34d039a11d1aaf8819b7b8c02",
    "gitBlobSHA1": "48a1c46009929341f2c312966acf81c3c733d7ec",
    "sourcePath": "assets/meshes/right_shoulder_pitch_link.STL"
  },
  {
    "path": "meshes/right_shoulder_roll_link.STL",
    "bytes": 401884,
    "sha256": "962b97c48f9ce9e8399f45dd9522e0865d19aa9fd299406b2d475a8fc4a53e81",
    "gitBlobSHA1": "2a5d22f9e17a986ec8c958a6ff7668d65878a907",
    "sourcePath": "assets/meshes/right_shoulder_roll_link.STL"
  },
  {
    "path": "meshes/right_shoulder_yaw_link.STL",
    "bytes": 249984,
    "sha256": "a0b76489271da0c72461a344c9ffb0f0c6e64f019ea5014c1624886c442a2fe5",
    "gitBlobSHA1": "0882a567fc1a11f0cf821182154551058b0517f3",
    "sourcePath": "assets/meshes/right_shoulder_yaw_link.STL"
  },
  {
    "path": "meshes/right_wrist_pitch_link.STL",
    "bytes": 79584,
    "sha256": null,
    "gitBlobSHA1": "da194543c40df9d492abb0553c06cc614a78caae",
    "sourcePath": "assets/meshes/right_wrist_pitch_link.STL"
  },
  {
    "path": "meshes/right_wrist_roll_link.STL",
    "bytes": 356084,
    "sha256": "a7ee9212ff5b94d6cb7f52bb1bbf3f352194d5b598acff74f4c77d340c5b344f",
    "gitBlobSHA1": "77d23a77ad6ab659884db4d540e0bd9fd0675d0f",
    "sourcePath": "assets/meshes/right_wrist_roll_link.STL"
  },
  {
    "path": "meshes/right_wrist_yaw_link.STL",
    "bytes": 341484,
    "sha256": "bc9dece2d12509707e0057ba2e48df8f3d56db0c79410212963a25e8a50f61a6",
    "gitBlobSHA1": "77edbb4135bb5b9b64357a0226431fd665d794b2",
    "sourcePath": "assets/meshes/right_wrist_yaw_link.STL"
  },
  {
    "path": "meshes/torso_link_rev_1_0.STL",
    "bytes": 2570584,
    "sha256": "11ddb46f2098efbbd8816b1d65893632d8e78be936376c7cdcd6771899ccc723",
    "gitBlobSHA1": "836b9923b3edac7a1c06ecd2f11bfbbc0dedf7e0",
    "sourcePath": "assets/meshes/torso_link_rev_1_0.STL"
  },
  {
    "path": "meshes/waist_roll_link_rev_1_0.STL",
    "bytes": 85884,
    "sha256": null,
    "gitBlobSHA1": "a64f330c592582dc31cdf38f4d08cfff06681c5f",
    "sourcePath": "assets/meshes/waist_roll_link_rev_1_0.STL"
  },
  {
    "path": "meshes/waist_yaw_link_rev_1_0.STL",
    "bytes": 619984,
    "sha256": "ec6db442b11f25eed898b5add07940c85d804f300de24dcbd264ccd8be7d554c",
    "gitBlobSHA1": "dc628fbdda04129fda29140243d148cbd2c81c83",
    "sourcePath": "assets/meshes/waist_yaw_link_rev_1_0.STL"
  },
  {
    "path": "licenses/Unitree-BSD-3-Clause.txt",
    "bytes": 1558,
    "sha256": "84aac59fd3246e3ddc49d1387644e8fcf43b0def4f0b9fe687f372e90446df2d",
    "url": "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/5994d4faef0a9cadd3287f8de0199a67eeb2a259/LICENSE",
    "cacheName": "unitree-license"
  },
  {
    "path": "licenses/Apache-2.0.txt",
    "bytes": 11358,
    "sha256": "cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30",
    "url": "https://www.apache.org/licenses/LICENSE-2.0.txt",
    "cacheName": "apache-license"
  },
  {
    "path": "licenses/Unitree-AVP-Apache-Notice.txt",
    "bytes": 1151,
    "sha256": "63762ccebdb836cf2bae6b1ae0de526e7499dbf45e554c790ebc45942cce3e36",
    "url": "https://raw.githubusercontent.com/unitreerobotics/avp_teleoperate/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/LICENSE",
    "cacheName": "avp-license"
  }
];
const ROOT=fileURLToPath(new URL('../dist/g1/assets/',import.meta.url));
const sha=(algorithm,bytes)=>createHash(algorithm).update(bytes).digest('hex');
const urlFor=record=>record.url||`https://huggingface.co/lerobot/unitree-g1-mujoco/resolve/${HF_REVISION}/${record.sourcePath}?download=true`;
const byteCheck=(record,bytes)=>{
  if(bytes.length!==record.bytes)return false;
  if(record.sha256&&sha('sha256',bytes)!==record.sha256)return false;
  // Hub's git oid names an LFS pointer for large meshes, not its binary blob.
  // LFS binaries are checked against the published SHA256 instead.
  if(record.gitBlobSHA1&&!record.sha256&&sha('sha1',Buffer.concat([Buffer.from(`blob ${bytes.length}\0`),bytes]))!==record.gitBlobSHA1)return false;
  return true;
};
async function validated(path,record){try{const bytes=await readFile(path);return byteCheck(record,bytes)?bytes:null;}catch(error){if(error.code==='ENOENT')return null;throw error;}}
function curl(url,path,resume){
  return new Promise((resolvePromise,reject)=>{
    const args=['--http1.1','-L','--fail','--silent','--show-error','--connect-timeout','15','--max-time','120'];
    if(resume)args.push('-C','-');
    args.push(url,'-o',path);
    const child=spawn('curl',args,{stdio:['ignore','ignore','pipe']});let message='';
    child.stderr.on('data',bytes=>{message=(message+bytes.toString()).slice(-2000);});
    child.on('error',reject);child.on('close',code=>resolvePromise({code,message:message.trim()}));
  });
}
async function ensure(record,cache){
  const destination=resolve(ROOT,record.path);if(!destination.startsWith(ROOT))throw new Error('Asset path leaves destination');
  let bytes=await validated(destination,record);if(bytes)return {bytes,from:'existing'};
  try{await stat(destination);throw new Error(`Existing asset differs from pinned source: ${record.path}`);}catch(error){if(error.code!=='ENOENT')throw error;}
  await mkdir(dirname(destination),{recursive:true});
  if(cache){
    for(const candidate of [join(cache,record.path),join(cache,record.cacheName||basename(record.path))]){
      bytes=await validated(candidate,record);if(bytes){await copyFile(candidate,destination);return {bytes,from:'verified-cache'};}
    }
  }
  const partial=destination+'.download';
  for(let attempt=0;attempt<4;attempt++){
    let resume=false;try{resume=(await stat(partial)).size>0;}catch(error){if(error.code!=='ENOENT')throw error;}
    const result=await curl(urlFor(record),partial,resume);bytes=await validated(partial,record);
    if(bytes){await rename(partial,destination);return {bytes,from:'download'};}
    if(result.code===0||result.code===33){try{await unlink(partial);}catch(error){if(error.code!=='ENOENT')throw error;}}
    console.warn(`Retry ${attempt+1}/4 ${record.path}: ${result.message||'source checksum mismatch'}`);
  }
  throw new Error(`Unable to download verified asset ${record.path}`);
}
export async function verifyAssets(){
  const results=[];
  for(const record of RECORDS){const bytes=await validated(resolve(ROOT,record.path),record);if(!bytes)throw new Error(`Missing or mismatched asset: ${record.path}`);results.push({path:record.path,bytes:bytes.length,sha256:sha('sha256',bytes)});}
  return results;
}
async function main(){
  if(process.argv.includes('--verify')){const files=await verifyAssets();console.log(JSON.stringify({verified:files.length,meshFiles:RECORDS.filter(r=>r.path.match(/\.stl$/i)).length,bytes:files.reduce((n,f)=>n+f.bytes,0),revision:HF_REVISION}));return;}
  const cacheIndex=process.argv.indexOf('--cache'),cache=cacheIndex>=0?resolve(process.argv[cacheIndex+1]):null;
  const results=[],failures=[];let next=0,done=0;
  await Promise.all(Array.from({length:6},async()=>{while(next<RECORDS.length){const record=RECORDS[next++];try{const result=await ensure(record,cache);results.push({path:record.path,bytes:result.bytes.length,sha256:sha('sha256',result.bytes),gitObjectSHA1:record.gitBlobSHA1||null,gitObjectKind:record.gitBlobSHA1?(record.sha256&&record.path.toLowerCase().endsWith('.stl')?'lfs-pointer':'blob'):null,source:urlFor(record),sourcePath:record.sourcePath||'LICENSE'});console.log(`${++done}/${RECORDS.length} ${record.path} (${result.from})`);}catch(error){failures.push({path:record.path,error:error.message});console.error(error.message);}}}));
  if(failures.length)throw new Error(JSON.stringify(failures));
  results.sort((a,b)=>a.path.localeCompare(b.path));
  const manifest={repository:'lerobot/unitree-g1-mujoco',revision:HF_REVISION,requestedURDF:'https://huggingface.co/lerobot/unitree-g1-mujoco/resolve/main/assets/g1_29dof_with_dex1_1.urdf',urdfUnmodified:true,unitreeLicenseRevision:UNITREE_LICENSE_REVISION,unitreeAVPRevision:UNITREE_AVP_REVISION,files:results};
  await writeFile(join(ROOT,'SOURCE_MANIFEST.json'),JSON.stringify(manifest,null,2)+'\n');
  await verifyAssets();console.log(`Imported ${results.length} verified original files; no MJCF imported.`);
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))main().catch(error=>{console.error(error.message);process.exitCode=1;});
