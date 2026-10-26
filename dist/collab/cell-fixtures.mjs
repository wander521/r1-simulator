// One source for visible cabinetry AND collision geometry. Working surfaces,
// docking targets and the mobile aisle retain their validated coordinates.
export function cabinetFixtures(supports) {
  return supports.map(s => ({
    name: `${s.body}_cabinet`,
    position: [s.center[0], s.center[1], (s.tableHeight - .065 + .1)/2],
    half: [s.halfExtents[0] - .035, s.halfExtents[1] - .035, (s.tableHeight - .065 - .1)/2],
    color: '.12 .19 .23 1',
  }));
}

export function cabinetPlinths(supports){
  const local=supports.filter(s=>!s.body.startsWith('c5_')).map(s=>({
    name:`${s.body}_plinth`,position:[s.center[0],s.center[1],.05],
    half:[s.halfExtents[0]-.055,s.halfExtents[1]-.055,.05],color:'.08 .12 .15 1',
  }));
  // One grounding block ties the two C5 worktops and pedestal into a cell.
  return [...local,{name:'c5_cell_plinth',position:[2.65,-.585,.04],half:[.50,.485,.04],color:'.08 .12 .15 1'}];
}

export const CELL_PART_TYPES = Object.freeze([
  {category:'red-cylinder', shape:'cylinder', label:'红色圆柱', color:0xd7514c, rgba:'.75 .19 .17 1'},
  {category:'blue-box', shape:'box', label:'蓝色方块', color:0x438dd5, rgba:'.12 .38 .72 1'},
  {category:'yellow-cylinder', shape:'cylinder', label:'黄色圆柱', color:0xe5b83f, rgba:'.9 .68 .12 1'},
  {category:'green-box', shape:'box', label:'绿色方块', color:0x43ab81, rgba:'.12 .58 .34 1'},
]);

// Backing ends just above the upper shelf; shared by rendering and physics.
export const RACK_STRUCTURE = Object.freeze([
  {name:'wall_rack_back',position:[.67,-.84,.795],half:[.018,.31,.245],color:'.15 .21 .26 1'},
  {name:'wall_rack_foot',position:[.67,-.84,.375],half:[.025,.31,.375],color:'.12 .17 .2 1'},
]);
