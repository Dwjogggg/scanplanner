#!/usr/bin/env python3
"""Build six selectable, physically renderable scenes; do not start a planner."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from go2_locomotion import make_model
from scan_scene import load_scene, global_route, obstacle_distance


def box(name, x, y, hx, hy, height=0.9, blocker=False):
    return dict(name=name,shape='box',xy=[x,y],half=[hx,hy],height=height,on_reference=blocker)


def post(name, x, y, radius=0.28, height=0.9, blocker=False):
    return dict(name=name,shape='cylinder',xy=[x,y],radius=radius,height=height,on_reference=blocker)


def point_at(route, fraction):
    lengths = [math.dist(a,b) for a,b in zip(route[:-1],route[1:])]
    remaining = sum(lengths)*fraction
    for a,b,length in zip(route[:-1],route[1:],lengths):
        if remaining <= length:
            return [a[j]+(b[j]-a[j])*remaining/length for j in range(2)]
        remaining -= length
    return route[-1]


def candidates():
    base = load_scene()
    walls = [o for o in base['obstacles'] if o['name'].startswith('wall')]
    specs = [
        ('Open aisle','开阔通道', '基础绕障：三个独立障碍，绕行空间较大。'),
        ('Alternating barriers','交错挡板', '连续转向：三道挡板分别伸向通道两侧，形成 S 形绕行。'),
        ('Narrow corridor','货架窄通道', '侧向空间受限：两排长货架之间布置三个小障碍。'),
        ('Doorway approach','门洞通行', '先绕障再对准门洞，出门后再绕开障碍。'),
        ('Dense mixed clutter','密集混合障碍', '箱体和立柱混合，主路径上四个障碍，旁侧还有干扰物。'),
        ('Warehouse corner','仓库转角', '全局参考先绕大货架转弯，转弯路线中再加入三个障碍。'),
    ]
    output = []
    for i,(title,title_zh,description) in enumerate(specs,1):
        scene = dict(name=f'go2_candidate_{i:02d}',description=description,
                     bounds=base['bounds'],start=[0.0,0.0],goal=[9.8,0.0],
                     route_clearance=0.65,seed=2026110500+i,
                     obstacles=copy.deepcopy(walls))
        obstacles = scene['obstacles']
        if i == 1:
            for x in [1.3,3.6,6.2,8.7]:
                obstacles += [box(f'crate_n_{x}',x,2.7,.35,.3),post(f'post_s_{x}',x,-2.7)]
            obstacles += [post('path_blocker_1',2.3,0,.32,blocker=True),
                          box('path_blocker_2',5.0,0,.34,.34,blocker=True),
                          post('path_blocker_3',7.7,0,.32,blocker=True)]
        elif i == 2:
            for n,(x,y) in enumerate([(2.4,.85),(5.0,-.85),(7.6,.85)],1):
                obstacles.append(box(f'path_blocker_{n}',x,y,.24,1.22,1.0,True))
            for n,(x,y) in enumerate([(1.0,-2.8),(3.7,2.9),(6.3,-2.9),(9.0,2.8)],1):
                obstacles.append(box(f'crate_{n}',x,y,.3,.25,.7))
        elif i == 3:
            obstacles += [box('shelf_n',5,2.1,4.3,.35,1.15),box('shelf_s',5,-2.1,4.3,.35,1.15)]
            for n,x in enumerate([2.4,4.9,7.4],1):
                obstacles.append(post(f'path_blocker_{n}',x,0,.27,.8,True))
        elif i == 4:
            obstacles += [box('shelf_gate_n',4.9,2.1,.3,1.3,1.1),
                          box('shelf_gate_s',4.9,-2.1,.3,1.3,1.1)]
            for n,x in enumerate([2.6,7.1],1):
                obstacles.append(box(f'path_blocker_{n}',x,0,.32,.35,.9,True))
            for n,(x,y) in enumerate([(1.1,2.8),(2.8,-2.6),(7.2,2.7),(8.8,-2.7)],1):
                obstacles.append(post(f'post_{n}',x,y,.30))
        elif i == 5:
            for n,x in enumerate([2.0,3.9,6.0,8.0],1):
                obstacles.append(post(f'path_blocker_{n}',x,0,.29,.9,True) if n%2
                                 else box(f'path_blocker_{n}',x,0,.30,.30,.85,True))
            for n,(x,y) in enumerate([(1.0,2.6),(2.9,2.2),(4.7,2.65),(6.9,2.2),(8.9,2.6),
                                      (1.2,-2.65),(3.0,-2.5),(4.9,-2.2),(6.9,-2.65),(8.9,-2.2)],1):
                obstacles.append(post(f'post_{n}',x,y,.29) if n%2
                                 else box(f'crate_{n}',x,y,.30,.33,.8))
        else:
            scene.update(start=[0.0,-2.0],goal=[9.8,2.0])
            obstacles += [box('shelf_main',3.2,1.0,1.8,1.5,1.1),
                          box('shelf_lower',8.7,-2.6,1.0,.4,1.0),
                          post('post_1',1.0,2.9,.24),post('post_2',7.5,2.9,.25),
                          box('crate_1',.2,1.0,.35,.25,.7)]
            reference = global_route(scene)
            for n,fraction in enumerate([.22,.52,.78],1):
                x,y = point_at(reference,fraction)
                obstacles.append(post(f'path_blocker_{n}',x,y,.28,.9,True))
        output.append((scene,title,title_zh))
    return output


def segment_geom(view, a, b, radius, color):
    geom = view.geoms[view.ngeom]
    mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),
                       np.eye(3).ravel(),np.array(color,np.float32))
    mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_CAPSULE,radius,
                        np.array([*a,.045]),np.array([*b,.045]))
    view.ngeom += 1


def render(scene, route, destination, title):
    model = make_model(scene)
    data = mujoco.MjData(model)
    data.qpos[:3] = [*scene['start'],.34]
    data.qpos[3:7] = [1,0,0,0]
    data.qpos[model.jnt_qposadr[model.actuator_trnid[:,0]]] = np.tile([0,.8,-1.5],4)
    mujoco.mj_forward(model,data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [4.8,0,.1]
    camera.distance,camera.azimuth,camera.elevation = 13.6,90,-65
    with mujoco.Renderer(model,height=640,width=960,max_geom=4000) as renderer:
        renderer.update_scene(data,camera=camera)
        for n,(a,b) in enumerate(zip(route[:-1],route[1:])):
            if n%2 == 0:
                segment_geom(renderer.scene,a,b,.024,[.07,.09,.12,1])
        frame = renderer.render().copy()
    count = sum(bool(o.get('on_reference')) for o in scene['obstacles'])
    font = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    overlay = (f"drawtext=fontfile={font}:text='{title}':x=24:y=16:fontsize=30:fontcolor=white:box=1:boxcolor=black@0.75,"
               f"drawtext=fontfile={font}:text='{count} on-path blockers | Candidate preview - algorithm not run':x=24:y=58:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.65,"
               f"drawtext=fontfile={font}:text='Red = path obstacles | Dashed = reference | Robot = start | Green = goal':x=22:y=h-35:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.65")
    subprocess.run(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-y',
                    '-f','rawvideo','-pixel_format','rgb24','-video_size','960x640','-i','pipe:0',
                    '-vf',overlay,'-frames:v','1',str(destination)],input=frame.tobytes(),check=True)


def validate(scene, reference, feasible):
    for point in [scene['start'],scene['goal']]:
        assert min(obstacle_distance(point,o) for o in scene['obstacles']) >= .65
    for blocker in [o for o in scene['obstacles'] if o.get('on_reference')]:
        # Dense samples include segment interiors, not just vertices.
        assert min(obstacle_distance(p,blocker) for p in reference) < 0
    for a,b in zip(feasible[:-1],feasible[1:]):
        n = max(1,math.ceil(math.dist(a,b)/.025))
        for j in range(n+1):
            point = [a[k]+(b[k]-a[k])*j/n for k in range(2)]
            assert min(obstacle_distance(point,o) for o in scene['obstacles']) >= .64


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=Path,required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    manifest = []
    for i,(scene,title,title_zh) in enumerate(candidates(),1):
        background = dict(scene,obstacles=[o for o in scene['obstacles'] if not o.get('on_reference')])
        reference,feasible = global_route(background),global_route(scene)
        validate(scene,reference,feasible)
        scene['layout_hash'] = hashlib.sha256(json.dumps(scene['obstacles'],sort_keys=True).encode()).hexdigest()[:16]
        scene['on_path_blocker_count'] = sum(bool(o.get('on_reference')) for o in scene['obstacles'])
        stem = f'scene_{i:02d}'
        (args.output_dir/f'{stem}.json').write_text(json.dumps(scene,ensure_ascii=False,indent=2)+'\n')
        (args.output_dir/f'{stem}_routes.json').write_text(json.dumps(dict(reference=reference,
                                      diagnostic_feasibility_route=feasible),indent=2)+'\n')
        render(scene,reference,args.output_dir/f'{stem}.png',f'{i:02d} | {title}')
        manifest.append(dict(id=i,title=title,title_zh=title_zh,description=scene['description'],
                             scene=f'{stem}.json',preview=f'{stem}.png',
                             obstacle_count=len(scene['obstacles']),on_path_blockers=scene['on_path_blocker_count'],
                             geometric_feasibility_checked=True,algorithm_run=False,
                             reference_length=sum(math.dist(a,b) for a,b in zip(reference[:-1],reference[1:])),
                             feasible_length=sum(math.dist(a,b) for a,b in zip(feasible[:-1],feasible[1:]))))
        print(f'{stem}: {title}, geometry passed, {len(scene["obstacles"])} obstacles',flush=True)
    (args.output_dir/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    cmd = ['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-y']
    for i in range(1,7):
        cmd += ['-i',str(args.output_dir/f'scene_{i:02d}.png')]
    cmd += ['-filter_complex','[0:v][1:v][2:v][3:v][4:v][5:v]xstack=inputs=6:layout=0_0|w0_0|0_h0|w0_h0|0_h0+h2|w0_h0+h2[out]',
            '-map','[out]','-frames:v','1',str(args.output_dir/'overview.png')]
    subprocess.run(cmd,check=True)
    print(args.output_dir/'overview.png',flush=True)


if __name__ == '__main__':
    main()
