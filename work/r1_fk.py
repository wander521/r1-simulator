import math
import xml.etree.ElementTree as ET

SOURCE = 'dist/r1/assets/r1.urdf'
tree = ET.parse(SOURCE)
joints = {}
children = {}
for node in tree.findall('joint'):
    origin = node.find('origin')
    axis = node.find('axis')
    parent = node.find('parent').attrib['link']
    child = node.find('child').attrib['link']
    joints[child] = {
        'name': node.attrib['name'], 'parent': parent,
        'xyz': [float(v) for v in origin.attrib.get('xyz', '0 0 0').split()],
        'rpy': [float(v) for v in origin.attrib.get('rpy', '0 0 0').split()],
        'axis': [float(v) for v in axis.attrib.get('xyz', '0 0 0').split()],
        'type': node.attrib['type'],
    }
    children.setdefault(parent, []).append(child)

def eye(): return [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
def mm(a,b): return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(a,v): return [sum(a[i][k]*v[k] for k in range(3)) for i in range(3)]
def add(a,b): return [a[i]+b[i] for i in range(3)]
def rpy(r,p,y):
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
def axisrot(axis, theta):
    x,y,z=axis; n=math.sqrt(x*x+y*y+z*z)
    if not n: return eye()
    x,y,z=x/n,y/n,z/n; c,s=math.cos(theta),math.sin(theta); d=1-c
    return [[c+x*x*d,x*y*d-z*s,x*z*d+y*s],[y*x*d+z*s,c+y*y*d,y*z*d-x*s],[z*x*d-y*s,z*y*d+x*s,c+z*z*d]]

poses = {
 'torso_joint1': 1.45, 'torso_joint2': -2.35, 'torso_joint3': -.8, 'torso_joint4': 0,
 'right_arm_joint1': -1.57, 'right_arm_joint2': 2.20, 'right_arm_joint3': -1.40, 'right_arm_joint4': 0, 'right_arm_joint5': 0, 'right_arm_joint6': 0,
 'left_arm_joint1': .379, 'left_arm_joint2': 3.18, 'left_arm_joint3': -2.072, 'left_arm_joint4': 1.373, 'left_arm_joint5': 1.108, 'left_arm_joint6': 2.181,
 'right_gripper_finger_joint1': .003, 'right_gripper_finger_joint2': .003, 'left_gripper_finger_joint1': .003, 'left_gripper_finger_joint2': .003,
}
world = {'base_link': (eye(), [0.,0.,0.])}
pending = True
while pending:
    pending = False
    for child,joint in joints.items():
        if child in world or joint['parent'] not in world: continue
        parent_r,parent_p=world[joint['parent']]; local_r=rpy(*joint['rpy']); local_p=joint['xyz'][:]
        angle=poses.get(joint['name'],0.)
        if joint['type'] in ('revolute','continuous'): local_r=mm(local_r,axisrot(joint['axis'],angle))
        if joint['type']=='prismatic': local_p=add(local_p,[angle*a for a in joint['axis']])
        world[child]=(mm(parent_r,local_r),add(parent_p,mv(parent_r,local_p))); pending=True

for name in ['right_gripper_link','right_gripper_finger_link1','right_gripper_finger_link2','left_gripper_link','left_gripper_finger_link1','left_gripper_finger_link2']:
    p=world[name][1]; three=[p[0],p[2],-p[1]]
    print(f'{name:33s} URDF {p[0]: .3f} {p[1]: .3f} {p[2]: .3f}  THREE {three[0]: .3f} {three[1]: .3f} {three[2]: .3f}')
    if name.endswith('gripper_finger_link1') or name.endswith('gripper_finger_link2'):
        print('  rotation', ' '.join(f'{v:.6f}' for row in world[name][0] for v in row))
