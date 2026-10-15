import {PIPE_RADIUS,PIPE_HALF_HEIGHT,PIPE_MASS} from './pipe-layout.mjs';

/** Derive only the task props. Robot/table/contacts/actuators stay byte-identical.
 * Free cylinder bodies use the original yellow cylinder's mass and inertia.
 */
export function buildPipeScene(sourceXml,layout){
  if(typeof sourceXml!=='string'||!sourceXml.includes('<worldbody>'))throw new Error('缺少原始物理场景，不能创建圆管分类任务');
  let xml=sourceXml;let removed=0;
  xml=xml.replace(/^\s*<body name="(?:red_cube|blue_block|yellow_cylinder)"[^\n]*<\/body>\r?\n/gm,()=>{removed++;return'';});
  if(removed!==3)throw new Error('原始任务物体不符合预期，已取消物理场景替换');
  const props=layout.targets.map(t=>`    <body name="${t.body}" pos="${t.start.x} ${-t.start.z} ${t.start.y}"><freejoint/><inertial pos="0 0 0" mass="${PIPE_MASS}" diaginertia=".000013664 .000013664 .0000084"/><geom name="${t.body}_geom" type="cylinder" size="${PIPE_RADIUS} ${PIPE_HALF_HEIGHT}" contype="1" conaffinity="3"/></body>`).join('\n');
  return xml.replace('</worldbody>',`${props}\n  </worldbody>`);
}
