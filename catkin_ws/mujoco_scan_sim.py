#!/usr/bin/env python3
"""SCAN local planning + floating-base Go2 locomotion in a cluttered warehouse."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import secrets
import socket
import subprocess
import time

import mujoco
import numpy as np
from scipy.interpolate import BSpline

from go2_locomotion import Go2Locomotion, make_model
from scan_scene import randomized_episode, fixed_episode, PATH_OBSTACLE_SCENE, obstacle_distance, route_distance


class VideoRecorder:
    """Finalize each episode/60-second part; active fragmented MP4 is readable."""
    def __init__(self, directory, fps=20):
        self.directory = directory
        self.fps = fps
        self.process = None
        self.frames = 0
        self.part = 0
        self.episode = None
        self.log = None
        self.path = None
        self.scene_name = ''

    def write(self, frame, episode, seed):
        if self.process is None or episode != self.episode or self.frames >= self.fps*60:
            self.close()
            self.part = self.part+1 if episode == self.episode else 1
            self.episode = episode
            self.path = self.directory / f"episode_{episode:04d}_part_{self.part:03d}.mp4"
            self.log = open(self.path.with_suffix(".ffmpeg.log"), "ab")
            overlay = ("drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:"
                       f"text='SCAN | GPU 4 | Go2 | {self.scene_name} | Episode {episode}':"
                       "x=18:y=16:fontsize=20:fontcolor=white:box=1:boxcolor=black@0.65,"
                       "drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:"
                       "text='Dashed reference | Red blockers | Cyan plan | Yellow track | Green goal':"
                       "x=18:y=h-34:fontsize=17:fontcolor=white:box=1:boxcolor=black@0.65")
            self.process = subprocess.Popen([
                "/usr/bin/ffmpeg", "-hide_banner", "-loglevel", "warning", "-y",
                "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "960x640",
                "-framerate", str(self.fps), "-i", "pipe:0", "-an", "-vf", overlay,
                "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-threads", "2",
                "-pix_fmt", "yuv420p", "-g", str(self.fps),
                "-movflags", "+frag_keyframe+empty_moov+default_base_moof", str(self.path),
            ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log,
                start_new_session=True)
            self.frames = 0
            print(f"Recording {self.path}", flush=True)
        self.process.stdin.write(frame.tobytes())
        self.frames += 1

    def close(self):
        if self.process is None:
            return
        try:
            self.process.stdin.close()
        except BrokenPipeError:
            pass
        try:
            code = self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            code = self.process.wait(timeout=5)
        self.log.close()
        self.process = None
        if code:
            raise RuntimeError(f"Video encoder exited with status {code}: {self.path}")
        print(f"Finalized {self.path} ({self.frames} frames)", flush=True)


def draw_path(scene, points, color, width=4, world_width=False, dashed=False):
    kind = mujoco.mjtGeom.mjGEOM_CAPSULE if world_width else mujoco.mjtGeom.mjGEOM_LINE
    for index,(a, b) in enumerate(zip(points[:-1], points[1:])):
        if dashed and index%2:
            continue
        if scene.ngeom >= scene.maxgeom:
            break
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, kind, np.zeros(3),
                           np.zeros(3), np.eye(3).ravel(), np.array(color, np.float32))
        mujoco.mjv_connector(geom, kind, width,
                            np.asarray(a, np.float64), np.asarray(b, np.float64))
        scene.ngeom += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--episode-seconds',type=float,default=180)
    parser.add_argument('--seed',type=int,default=None,help='First layout seed; incremented each episode.')
    parser.add_argument('--scene-file',type=Path,default=PATH_OBSTACLE_SCENE)
    parser.add_argument('--randomize',action='store_true',help='Optional random-scene mode; default is the fixed test scene.')
    parser.add_argument('--episodes',type=int,default=1,help='Number of tests, then exit (default: one).')
    parser.add_argument('--primitive-visuals',action='store_true',
                        help='Render collision primitives; joint dynamics stay identical.')
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error('--episodes must be at least 1')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    videos = args.output_dir/'videos'
    videos.mkdir(exist_ok=True)
    base_seed = args.seed if args.seed is not None else secrets.randbits(32)
    scene,route,feasible_route = randomized_episode(base_seed) if args.randomize else fixed_episode(args.scene_file)
    running = True
    def stop(_signum,_frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    model = make_model(scene,meshes=not args.primitive_visuals)
    data = mujoco.MjData(model)
    locomotion = Go2Locomotion(model)
    joint_names = [mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,int(j))
                   for j in model.actuator_trnid[:,0]]
    obstacle_ids = locomotion.obstacle_ids
    geomid = np.array([-1],np.int32)
    # Only world geometry is group 0; robot visual/collision groups are 1/2/3.
    ray_groups = np.array([1,0,0,0,0,0],np.uint8)
    sensor = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    commands = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    commands.bind(('127.0.0.1',45872))
    commands.setblocking(False)
    renderer = mujoco.Renderer(model,height=640,width=960,max_geom=4000)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [4.8,0,0.1]
    camera.distance = 13.6
    camera.azimuth = 90
    camera.elevation = -65
    follow_camera = mujoco.MjvCamera()
    follow_camera.distance = 2.0
    follow_camera.elevation = -22
    recorder = VideoRecorder(videos)
    telemetry = open(args.output_dir/'telemetry.csv','w',buffering=1)
    csvlog = csv.writer(telemetry)
    csvlog.writerow(['wall_time','episode','elapsed','x','y','z','roll','pitch','yaw',
                     'vx_world','vy_world','cmd_vx_body','cmd_vy_body','cmd_wz',
                     'goal_distance','collision_substeps','reached_goal','command_age',
                     'FR_contact','FL_contact','RR_contact','RL_contact',*joint_names])
    summary = open(args.output_dir/'episodes.jsonl','a',buffering=1)
    episode = 0
    goal = np.array(scene['goal'])
    global_path = [[x,y,0.025] for x,y in route]
    print(f"Go2: floating base, {model.nu} joint motors, RL locomotion 50 Hz, physics 200 Hz",flush=True)
    print(f"Scene {scene['name']}: {len(scene['obstacles'])} obstacles including walls, goal={scene['goal']}",flush=True)
    try:
        while running and episode < args.episodes:
            episode += 1
            if episode > 1:
                # Recompile all world geometry and collision bounds, not only visuals.
                scene,route,feasible_route = (randomized_episode(base_seed+episode-1)
                                            if args.randomize else fixed_episode(args.scene_file))
                renderer.close()
                model = make_model(scene,meshes=not args.primitive_visuals)
                data = mujoco.MjData(model)
                locomotion = Go2Locomotion(model)
                renderer = mujoco.Renderer(model,height=640,width=960,max_geom=4000)
            seed = scene['seed']
            recorder.scene_name = scene['name']
            goal = np.array(scene['goal'])
            global_path = [[x,y,0.025] for x,y in route]
            episode_dir = args.output_dir/'episodes'/f'episode_{episode:04d}'
            episode_dir.mkdir(parents=True,exist_ok=True)
            configuration = {'episode':episode,'seed':seed,'layout_hash':scene['layout_hash'],
                             'scene_mode':'randomized' if args.randomize else 'fixed',
                             'scene':scene,'reference_route':route,'feasibility_route':feasible_route,
                             'reference_excludes_on_path_blockers':True}
            (episode_dir/'configuration.json').write_text(json.dumps(configuration,indent=2))
            (args.output_dir/'scene.json').write_text(json.dumps(scene,indent=2))
            (args.output_dir/'global_route.json').write_text(json.dumps(route))
            locomotion.reset(data,scene['start'])
            start = deadline = time.monotonic()
            last_command = 0.0
            velocity = np.zeros(3)
            filtered_velocity = np.zeros(3)
            planned = []
            trail = [[*scene['start'],0.045]]
            path_length = max_lateral = max_tilt = 0.0
            max_reference_deviation = 0.0
            min_clearance = 5.0
            min_height,max_height = 1.0,0.0
            trajectories = 0
            finished = reached = fallen = False
            finish_time = None
            termination_reason = None
            finish_elapsed = None
            next_sensor = next_frame = next_status = start
            print(f"Episode {episode} started: seed={seed}, layout={scene['layout_hash']}, "
                  f"obstacles={len(scene['obstacles'])}, on-path blockers={scene['on_path_blocker_count']}",flush=True)
            while running:
                now = time.monotonic()
                elapsed = now-start
                while True:
                    try:raw,_ = commands.recvfrom(65507)
                    except BlockingIOError:break
                    message = json.loads(raw)
                    if message.get('episode') != episode:continue
                    if message['kind'] == 'command':
                        velocity = np.asarray(message['velocity'],dtype=float)
                        if velocity.shape != (3,) or not np.isfinite(velocity).all():velocity = np.zeros(3)
                        velocity = np.clip(velocity,[-0.4,-0.25,-0.7],[0.4,0.25,0.7])
                        if not message['enabled']:velocity[:] = 0
                        last_command = now
                    elif message['kind'] == 'trajectory':
                        spline = BSpline(message['knots'],message['points'],message['order'])
                        knots = message['knots']
                        planned = spline(np.linspace(knots[message['order']],knots[-message['order']-1],80)).tolist()
                        for point in planned:point[2] = 0.07
                        trajectories += 1
                if now-last_command > 0.5 or finished:velocity[:] = 0
                # Smooth commands into the locomotion policy; emergency timeout still brakes.
                filtered_velocity += np.clip(velocity-filtered_velocity,[-0.025,-0.025,-0.05],[0.025,0.025,0.05])
                locomotion.step(data,filtered_velocity)
                xy = data.qpos[:2].copy()
                height = float(data.qpos[2])
                roll,pitch,yaw = locomotion.attitude(data)
                tilt = max(abs(roll),abs(pitch))
                max_tilt = max(max_tilt,tilt)
                min_height,max_height = min(min_height,height),max(max_height,height)
                distance = float(np.linalg.norm(xy-goal))
                # Conservative horizontal disk bound on the moving robot footprint.
                clearance = min(obstacle_distance(xy,o) for o in scene['obstacles'])-0.43
                min_clearance = min(min_clearance,clearance)
                max_lateral = max(max_lateral,abs(float(xy[1])))
                max_reference_deviation = max(max_reference_deviation,route_distance(xy,route))
                if np.linalg.norm(xy-np.array(trail[-1][:2])) > 0.025:
                    path_length += float(np.linalg.norm(xy-np.array(trail[-1][:2])))
                    trail.append([float(xy[0]),float(xy[1]),0.045])
                if not finished and (distance < 0.30 or elapsed > args.episode_seconds or height < 0.14 or tilt > 1.0):
                    fallen = height < 0.14 or tilt > 1.0
                    reached = distance < 0.30 and not fallen
                    finished,finish_time = True,now
                    finish_elapsed = elapsed
                    termination_reason = 'fall' if fallen else 'goal' if reached else 'timeout'
                    print(f'Episode {episode}: reached={reached}, fallen={fallen}, distance={distance:.3f}, '
                          f'collision_substeps={locomotion.collision_substeps}',flush=True)
                foot_contacts = [False]*4
                for contact in data.contact:
                    for i,foot in enumerate(locomotion.foot_ids):
                        if contact.geom1 == foot or contact.geom2 == foot:foot_contacts[i] = True
                if now >= next_sensor:
                    rotation = data.xmat[locomotion.base_id].reshape(3,3)
                    origin = data.qpos[:3]+rotation @ np.array([0.0,0.0,0.30])
                    points = []
                    for vertical in (-12,0,12):
                        elev = math.radians(vertical)
                        for degrees in range(-180,180,2):
                            azimuth = math.radians(degrees)
                            local = np.array([math.cos(elev)*math.cos(azimuth),
                                              math.cos(elev)*math.sin(azimuth),math.sin(elev)])
                            direction = rotation @ local
                            ray_distance = mujoco.mj_ray(model,data,origin,direction,ray_groups,1,-1,geomid)
                            if 0 <= ray_distance <= 5.0:
                                hit = origin+ray_distance*direction
                                if hit[2] > 0.06:points.append(np.round(hit,4).tolist())
                    yaw_rate = float((rotation @ data.qvel[3:6])[2])
                    sample = {'episode':episode,'finished':finished,'xyz':data.qpos[:3].tolist(),
                              'seed':seed,'layout_hash':scene['layout_hash'],'reference_route':route,
                              'quaternion':data.qpos[3:7].tolist(),'yaw':yaw,
                              'velocity_world':data.qvel[:3].tolist(),'yaw_rate':yaw_rate,'points':points,
                              'joint_names':joint_names,'joint_positions':data.qpos[locomotion.qadr].tolist(),
                              'joint_velocities':data.qvel[locomotion.vadr].tolist()}
                    sensor.sendto(json.dumps(sample,separators=(',',':')).encode(),('127.0.0.1',45871))
                    csvlog.writerow([time.time(),episode,elapsed,*xy,height,roll,pitch,yaw,*data.qvel[:2],
                                     *filtered_velocity,distance,locomotion.collision_substeps,reached,
                                     now-last_command,*foot_contacts,*data.qpos[locomotion.qadr]])
                    next_sensor = now+0.1
                if now >= next_frame:
                    renderer.update_scene(data,camera=camera)
                    draw_path(renderer.scene,global_path,[0.07,0.09,0.12,1],.024,world_width=True,dashed=True)
                    draw_path(renderer.scene,planned,[0,0.8,0.9,1])
                    draw_path(renderer.scene,trail[-1600:],[1,0.7,0,1],5)
                    frame = renderer.render().copy()
                    follow_camera.lookat[:] = data.qpos[:3]
                    follow_camera.azimuth = math.degrees(yaw)+135
                    renderer.update_scene(data,camera=follow_camera)
                    inset = renderer.render()[::3,::3]
                    ih,iw = inset.shape[:2]
                    frame[640-60-ih:640-60,960-16-iw:960-16] = inset
                    recorder.write(frame,episode,seed)
                    next_frame += 1/recorder.fps
                    if next_frame < now-0.1:next_frame = now+1/recorder.fps
                if now >= next_status:
                    status = {'episode':episode,'elapsed':elapsed,'xyz':data.qpos[:3].tolist(),
                              'seed':seed,'layout_hash':scene['layout_hash'],
                              'scene_mode':'randomized' if args.randomize else 'fixed',
                              'on_path_blocker_count':scene['on_path_blocker_count'],
                              'max_reference_deviation':max_reference_deviation,
                              'configuration':str(episode_dir/'configuration.json'),
                              'rpy':[roll,pitch,yaw],'goal_distance':distance,'reached_goal':reached,
                              'fallen':fallen,'collision_substeps':locomotion.collision_substeps,
                              'finished':finished,'termination_reason':termination_reason,
                              'finish_elapsed':finish_elapsed,
                              'min_footprint_clearance':min_clearance,'footprint_bound_radius':0.43,
                              'max_tilt_radians':max_tilt,
                              'base_height_range':[min_height,max_height],'max_lateral_displacement':max_lateral,
                              'path_length':path_length,'trajectories':trajectories,'command_age':now-last_command,
                              'foot_contacts':foot_contacts,'video':str(recorder.path),'gpu_index':4,
                              'model':'Unitree Go2, floating base, 12 joint-torque actuators',
                              'locomotion_policy':'rl_sar/go2/robot_lab','scene':scene['name'],
                              'obstacle_count':len(scene['obstacles']),'wall_time':time.time()}
                    temp = args.output_dir/'status.tmp'
                    temp.write_text(json.dumps(status,indent=2));os.replace(temp,args.output_dir/'status.json')
                    next_status = now+1
                if finished and now-finish_time > 2:
                    recorder.close()
                    status.update(reached_goal=reached,fallen=fallen,goal_distance=distance,
                                  collision_substeps=locomotion.collision_substeps,elapsed=elapsed,
                                  path_length=path_length,max_tilt_radians=max_tilt,
                                  xyz=data.qpos[:3].tolist(),rpy=[roll,pitch,yaw],
                                  max_reference_deviation=max_reference_deviation)
                    temp = args.output_dir/'status.tmp'
                    temp.write_text(json.dumps(status,indent=2));os.replace(temp,args.output_dir/'status.json')
                    summary.write(json.dumps(status)+'\n')
                    (episode_dir/'executed_track.json').write_text(json.dumps(trail))
                    break
                deadline += 0.02
                time.sleep(max(0,deadline-time.monotonic()))
                if deadline < time.monotonic()-0.2:deadline = time.monotonic()
    finally:
        try:
            recorder.close()
        finally:
            telemetry.close();summary.close();renderer.close();commands.close();sensor.close()
        print('Go2 simulator stopped; recording finalized',flush=True)


if __name__ == '__main__':
    main()
