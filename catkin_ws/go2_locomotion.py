"""Floating-base Go2 dynamics and RL-SAR/robot_lab locomotion deployment.

Only twelve joint motor torques are applied. Base position/velocity are never
prescribed after reset; forward movement comes from foot/ground contacts.
The policy observation, joint order and PD gains follow the vendored config.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[3]
ROBOT_DIR = ROOT/'third_party/unitree_mujoco/unitree_robots/go2'
POLICY_DIR = ROOT/'third_party/rl_sar_go2/policy/go2/robot_lab'


def make_model(scene=None, meshes=True):
    root = ET.parse(ROBOT_DIR/'go2.xml').getroot()
    root.find('compiler').set('meshdir', str(ROBOT_DIR/'assets'))
    option = root.find('option')
    option.set('timestep','0.005')
    option.set('integrator','implicitfast')
    if not meshes:
        for parent in root.iter():
            for child in list(parent):
                if child.tag == 'mesh' or (child.tag == 'geom' and child.get('mesh')):
                    parent.remove(child)
        root.find(".//default[@class='collision']/geom").set('group','1')
        root.find(".//default[@class='collision']/geom").set('rgba','0.25 0.3 0.35 1')
    root.find(".//default[@class='foot']/geom").set('friction','0.8 0.02 0.01')
    visual = ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='960',offheight='640')
    ET.SubElement(visual,'headlight',ambient='0.5 0.5 0.5',diffuse='0.6 0.6 0.6')
    asset = root.find('asset')
    ET.SubElement(asset,'texture',name='test_grid',type='2d',builtin='checker',
                  rgb1='0.86 0.89 0.92',rgb2='0.71 0.77 0.82',width='512',height='512')
    ET.SubElement(asset,'material',name='test_floor',texture='test_grid',texrepeat='14 8',reflectance='0.03')
    world = root.find('worldbody')
    ET.SubElement(world,'light',pos='4 -3 8',dir='0 0 -1',directional='true')
    ET.SubElement(world,'geom',name='floor',type='plane',size='15 10 0.1',
                  material='test_floor',friction='0.8 0.02 0.01')
    if scene:
        for i,o in enumerate(scene['obstacles']):
            h = o['height']/2
            size = (*o['half'],h) if o['shape']=='box' else (o['radius'],h)
            color = ('0.32 0.40 0.48 0.35' if o['name'].startswith('wall') else
                     '0.90 0.12 0.10 1' if o.get('on_reference') else
                     '0.76 0.31 0.12 1' if o['name'].startswith('shelf') else
                     '0.16 0.48 0.65 1' if o['shape']=='cylinder' else '0.62 0.47 0.27 1')
            ET.SubElement(world,'geom',name=o['name'],type=o['shape'],
                          pos=f"{o['xy'][0]} {o['xy'][1]} {h}",
                          size=' '.join(map(str,size)),rgba=color,group='0')
        ET.SubElement(world,'geom',name='goal',type='cylinder',group='2',
                      pos=f"{scene['goal'][0]} {scene['goal'][1]} 0.015",size='0.3 0.015',
                      rgba='0.12 0.75 0.25 0.8',contype='0',conaffinity='0')
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))


class Go2Locomotion:
    def __init__(self, model):
        assert model.nu == 12 and model.nq == 19 and model.nv == 18
        self.model = model
        ids = model.actuator_trnid[:,0]
        self.qadr = model.jnt_qposadr[ids]
        self.vadr = model.jnt_dofadr[ids]
        self.default = np.tile([0.0,0.8,-1.5],4)
        self.action_scale = np.tile([0.125,0.25,0.25],4)
        self.action = np.zeros(12,np.float32)
        self.target = self.default.copy()
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.policy = ort.InferenceSession(str(POLICY_DIR/'policy.onnx'),options,
                                          providers=['CPUExecutionProvider'])
        self.input_name = self.policy.get_inputs()[0].name
        self.base_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
        self.foot_ids = [mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,n)
                         for n in ['FR','FL','RR','RL']]
        self.obstacle_ids = {i for i in range(model.ngeom)
                             if model.geom_bodyid[i] == 0 and model.geom_contype[i]
                             and mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) != 'floor'}
        self.collision_substeps = 0

    def reset(self, data, xy=(0,0)):
        mujoco.mj_resetData(self.model,data)
        data.qpos[:3] = [*xy,0.34]
        data.qpos[3:7] = [1,0,0,0]
        data.qpos[self.qadr] = self.default
        self.action[:] = 0
        self.collision_substeps = 0
        self.target[:] = self.default
        mujoco.mj_forward(self.model,data)

    def update(self, data, command):
        # Free-joint angular velocity is in the base frame in MuJoCo.
        rotation = data.xmat[self.base_id].reshape(3,3)
        gravity = rotation.T @ np.array([0,0,-1.0])
        obs = np.concatenate([data.qvel[3:6]*0.25,gravity,command,
                              data.qpos[self.qadr]-self.default,
                              data.qvel[self.vadr]*0.05,self.action]).astype(np.float32)
        obs = np.clip(obs,-100,100)
        self.action = np.clip(self.policy.run(None,{self.input_name:obs[None]})[0][0],-100,100)
        self.target = self.default+self.action_scale*self.action

    def torque(self, data):
        data.ctrl[:] = np.clip(20*(self.target-data.qpos[self.qadr])
                               -0.5*data.qvel[self.vadr],-23.5,23.5)

    def step(self, data, command):
        self.update(data,command)
        for _ in range(4):
            self.torque(data)
            mujoco.mj_step(self.model,data)
            if any(((int(c.geom1) in self.obstacle_ids and self.model.geom_bodyid[c.geom2] > 0)
                    or (int(c.geom2) in self.obstacle_ids and self.model.geom_bodyid[c.geom1] > 0))
                   for c in data.contact):
                self.collision_substeps += 1

    def attitude(self, data):
        w,x,y,z = data.qpos[3:7]
        roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch = math.asin(np.clip(2*(w*y-z*x),-1,1))
        yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        return roll,pitch,yaw
