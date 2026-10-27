#!/usr/bin/env python3
"""Read-only telemetry for the current course; usage: observe_run.py output.json seconds.

Distance/scan-hit estimates use wheel odometry aligned to the initial world pose,
so they are diagnostics, not ground-truth clearance. Contact events come from
Gazebo's moving-box sensor. This does not monitor every static-world collision.
"""
import json, math, sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import TwistStamped
from ros_gz_interfaces.msg import Contacts

rclpy.init()
n = Node('obstacle_test_observer')
state = {'robot': None, 'obstacle_y': None, 'speed': 0.0, 'scan_hits': 0}
report = {'samples': [], 'plans': [], 'contacts': [], 'joint_messages': 0, 'scan_messages': 0}
def odom(msg):
    p,q=msg.pose.pose.position,msg.pose.pose.orientation
    state['robot']=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    state['sim_time']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
def joint(msg):
    if 'slider' in msg.name:
        state['obstacle_y']=1.35+msg.position[msg.name.index('slider')]
        report['joint_messages']+=1
def cmd(msg): state['speed']=msg.twist.linear.x
def plan(msg):
    poses=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
    report['plans'].append({'sim_time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'poses':poses})
def contact(msg):
    for c in msg.contacts:
        report['contacts'].append({'a':c.collision1.name,'b':c.collision2.name,'depths':list(c.depths), 'sim_time':state.get('sim_time')})
def scan(msg):
    report['scan_messages']+=1
    if state['robot'] is None or state['obstacle_y'] is None:return
    x,y,yaw=state['robot']; sx=x-.064*math.cos(yaw); sy=y-.064*math.sin(yaw)
    hits=0
    for i,r in enumerate(msg.ranges):
        if not math.isfinite(r):continue
        a=yaw+msg.angle_min+i*msg.angle_increment
        px,py=sx+r*math.cos(a),sy+r*math.sin(a)
        if abs(px-4.7)<.275+.06 and abs(py-state['obstacle_y'])<.325+.06:hits+=1
    state['scan_hits']=hits
subs=[n.create_subscription(Odometry,'/odom',odom,10),n.create_subscription(JointState,'/demo/obstacle_joint_states',joint,10),n.create_subscription(TwistStamped,'/cmd_vel',cmd,10),n.create_subscription(NavPath,'/plan',plan,10),n.create_subscription(Contacts,'/demo/obstacle_contacts',contact,100),n.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)]
start=time.monotonic(); next_sample=start; next_log=start
try:
    while rclpy.ok() and time.monotonic()-start<float(sys.argv[2]):
        rclpy.spin_once(n,timeout_sec=.05)
        now=time.monotonic()
        if now>=next_sample and state['robot'] is not None and state['obstacle_y'] is not None:
            r=state['robot']; d=math.hypot(max(abs(r[0]-4.7)-.275,0),max(abs(r[1]-state['obstacle_y'])-.325,0))
            report['samples'].append({**state,'robot':r[:],'distance_to_obstacle_box':d})
            next_sample=now+.2
        if now>=next_log:
            print(json.dumps(state),flush=True);next_log=now+5
except KeyboardInterrupt:pass
finally:
    Path(sys.argv[1]).write_text(json.dumps(report))
    samples=report['samples']
    print(json.dumps({'samples':len(samples),'joint_messages':report['joint_messages'],'scan_messages':report['scan_messages'],'contacts':len(report['contacts']),'plans':len(report['plans']),'min_box_distance':min((s['distance_to_obstacle_box'] for s in samples),default=None),'obstacle_y_range':[min((s['obstacle_y'] for s in samples),default=None),max((s['obstacle_y'] for s in samples),default=None)],'max_scan_hits':max((s['scan_hits'] for s in samples),default=0)}),flush=True)
    n.destroy_node();rclpy.shutdown()
