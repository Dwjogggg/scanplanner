#!/usr/bin/env python3
"""Bidirectional MuJoCo/ROS bridge, with fresh-trajectory command gating."""
import json
import argparse
import math
import os
from pathlib import Path as FilePath
import signal
import socket
import struct
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from scan_planner.msg import Bspline
from sensor_msgs.msg import JointState, PointCloud2, PointField


def point_cloud(points, stamp):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = "world"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("fff", *p) for p in points)
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=FilePath,required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("mujoco_scan_ros_bridge")
    odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=1)
    cloud_pub = rospy.Publisher("/test/cloud", PointCloud2, queue_size=1)
    path_pub = rospy.Publisher("/initial_path", Path, queue_size=1)
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    route = None
    planner = None
    planner_log = open(args.output_dir/'planner.log','ab',buffering=0)

    def stop_planner():
        nonlocal planner
        if planner is None:
            return
        # Own process group only: never kill a shared ROS master or other jobs.
        for sig,timeout in [(signal.SIGINT,6),(signal.SIGTERM,2),(signal.SIGKILL,2)]:
            try:
                os.killpg(planner.pid,sig)
            except ProcessLookupError:
                break
            try:
                planner.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue
        planner = None

    def restart_planner(episode):
        nonlocal planner
        stop_planner()
        planner_log.write(f'\n=== Fresh planner/map/controller for episode {episode} ===\n'.encode())
        planner = subprocess.Popen(['roslaunch','--sigint-timeout=2','--sigterm-timeout=2',
                                    str(FilePath(__file__).with_name('go2_cluttered.launch')),
                                    'execute:=true'],stdout=planner_log,stderr=subprocess.STDOUT,
                                   start_new_session=True)
        (args.output_dir/'roslaunch.pid').write_text(str(planner.pid)+'\n')

    # Let rospy stop its timer and finalize the managed planner on TERM as well.
    signal.signal(signal.SIGTERM,lambda *_: rospy.signal_shutdown('Supervisor stop'))
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 45871))
    rx.settimeout(0.2)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lock = threading.Lock()
    state = {"episode": None, "episode_start": 0.0, "goal_time": 0.0,
             "last_sensor": 0.0, "last_cmd": 0.0, "trajectory_ready": False,
             "finished": False, "cmd": [0.0, 0.0, 0.0]}

    def command_callback(msg):
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not all(math.isfinite(v) for v in values):
            return
        with lock:
            state["cmd"] = values
            state["last_cmd"] = time.monotonic()

    def trajectory_callback(msg):
        with lock:
            if not state["goal_time"] or state["finished"]:
                return
            if msg.start_time.to_sec() < state["goal_time"]:
                return
            state["trajectory_ready"] = True
            packet = {"kind": "trajectory", "episode": state["episode"],
                      "order": msg.order, "knots": list(msg.knots),
                      "points": [[p.x, p.y, p.z] for p in msg.pos_pts],
                      "traj_id": msg.traj_id}
        tx.sendto(json.dumps(packet, separators=(",", ":")).encode(), ("127.0.0.1", 45872))

    def send_command(_event):
        now = time.monotonic()
        with lock:
            enabled = (state["trajectory_ready"] and not state["finished"]
                       and now-state["last_sensor"] < 0.5 and now-state["last_cmd"] < 0.5)
            packet = {"kind": "command", "episode": state["episode"],
                      "enabled": enabled, "velocity": state["cmd"] if enabled else [0, 0, 0]}
        tx.sendto(json.dumps(packet, separators=(",", ":")).encode(), ("127.0.0.1", 45872))

    subscribers = [
        rospy.Subscriber("/scan_sim/cmd_vel", Twist, command_callback, queue_size=1),
        rospy.Subscriber("/planning/bspline", Bspline, trajectory_callback, queue_size=1),
    ]
    timer = rospy.Timer(rospy.Duration(0.02), send_command)
    rospy.loginfo("Bidirectional bridge: sensors UDP 45871, execution UDP 45872")
    try:
        while not rospy.is_shutdown():
            try:
                packet, _ = rx.recvfrom(65507)
            except socket.timeout:
                continue
            sample = json.loads(packet)
            now = time.monotonic()
            with lock:
                if sample["episode"] != state["episode"]:
                    if 'reference_route' not in sample:
                        raise RuntimeError('Episode missing simulator-owned reference route')
                    route = sample['reference_route']
                    state.update(episode=sample["episode"], episode_start=now,
                                 goal_time=0.0, trajectory_ready=False, finished=False,
                                 last_cmd=0.0,cmd=[0.0,0.0,0.0])
                    # Clear previous map, progress and trajectory before teleporting
                    # odometry. Otherwise the native planner can replan the old goal
                    # from the reset start and hang inside its path search.
                    restart_planner(sample['episode'])
                    rospy.loginfo("Episode %d: waiting for fresh map and goal", sample["episode"])
                if planner is not None and planner.poll() is not None:
                    raise RuntimeError(f'Planner exited with code {planner.returncode}')
                state["last_sensor"] = now
                state["finished"] = sample["finished"]
                need_goal = (not state["goal_time"] and now-state["episode_start"] > 2.0
                             and path_pub.get_num_connections() > 0)
            stamp = rospy.Time.now()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "world"
            odom.child_frame_id = "base"
            x, y, z = sample["xyz"]
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = z
            quat = sample["quaternion"]
            odom.pose.pose.orientation.w = quat[0]
            odom.pose.pose.orientation.x = quat[1]
            odom.pose.pose.orientation.y = quat[2]
            odom.pose.pose.orientation.z = quat[3]
            # SCAN's odometry callback expects world-frame linear velocity.
            odom.twist.twist.linear.x = sample["velocity_world"][0]
            odom.twist.twist.linear.y = sample["velocity_world"][1]
            odom.twist.twist.linear.z = sample["velocity_world"][2]
            odom.twist.twist.angular.z = sample["yaw_rate"]
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud(sample["points"], stamp))
            joints = JointState()
            joints.header.stamp = stamp
            joints.name = sample["joint_names"]
            joints.position = sample["joint_positions"]
            joints.velocity = sample["joint_velocities"]
            joint_pub.publish(joints)
            if need_goal:
                path = Path()
                path.header.stamp = stamp
                path.header.frame_id = "world"
                for px, py in route:
                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position.x = px
                    pose.pose.position.y = py
                    pose.pose.position.z = z
                    pose.pose.orientation.w = 1.0
                    path.poses.append(pose)
                with lock:
                    state["goal_time"] = stamp.to_sec()
                path_pub.publish(path)
                rospy.loginfo("Episode %d: published %d-point global A* reference to %s (%d live cloud points)",
                              sample["episode"], len(route), route[-1], len(sample["points"]))
    finally:
        timer.shutdown()
        stop_planner()
        planner_log.close()
        rx.close()
        tx.close()


if __name__ == "__main__":
    main()
