#!/usr/bin/env python3
"""Send one endpoint to Nav2; no intermediate waypoints or velocity commands."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    rclpy.init()
    nav = BasicNavigator(node_name='demo_mission')
    nav.declare_parameter('goal_x', 6.0)
    nav.declare_parameter('goal_y', 0.0)
    nav.declare_parameter('goal_yaw', 0.0)
    nav.declare_parameter('timeout_sec', 180.0)
    endpoint = nav.create_publisher(PoseStamped, '/demo/goal_pose',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    observed = {'cells': 0}

    def on_map(msg):
        observed['cells'] = sum(value >= 0 for value in msg.data)

    subscription = nav.create_subscription(OccupancyGrid, '/map', on_map,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    buffer = Buffer()
    listener = TransformListener(buffer, nav)
    try:
        nav.waitUntilNav2Active(localizer='slam_toolbox')
        ready_deadline = time.monotonic() + 30.0
        while rclpy.ok():
            rclpy.spin_once(nav, timeout_sec=0.1)
            if observed['cells'] and buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                break
            if time.monotonic() > ready_deadline:
                nav.error('No live SLAM map/transform after 30 seconds; goal not sent.')
                return 1

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x = nav.get_parameter('goal_x').value
        goal.pose.position.y = nav.get_parameter('goal_y').value
        yaw = nav.get_parameter('goal_yaw').value
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        endpoint.publish(goal)
        initial_cells = observed['cells']
        start = time.monotonic()
        task = nav.goToPose(goal)
        if task is None:
            return 1
        next_report = start
        while not nav.isTaskComplete(task):
            now = time.monotonic()
            if now - start > nav.get_parameter('timeout_sec').value:
                nav.error('Mission timed out; canceling navigation.')
                nav.cancelTask()
                return 1
            if now >= next_report:
                feedback = nav.getFeedback(task)
                if feedback:
                    nav.info(f'Remaining path: {feedback.distance_remaining:.2f} m; '
                             f'recoveries: {feedback.number_of_recoveries}; '
                             f'observed map cells: {observed["cells"]}')
                next_report = now + 5.0

        result = nav.getResult()
        nav.info(f'Mission result: {result.name}; elapsed wall time: {time.monotonic()-start:.1f}s; '
                 f'observed map cells: {initial_cells} -> {observed["cells"]}')
        try:
            tf = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            p = tf.transform.translation
            error = math.hypot(p.x - goal.pose.position.x, p.y - goal.pose.position.y)
            nav.info(f'Final map pose: ({p.x:.3f}, {p.y:.3f}); goal position error: {error:.3f} m')
        except TransformException as exc:
            nav.warn(str(exc))
        if result != TaskResult.SUCCEEDED:
            nav.error(f'Nav2 error: {nav.getTaskError()}')
        return 0 if result == TaskResult.SUCCEEDED else 1
    except KeyboardInterrupt:
        nav.cancelTask()
        return 130
    finally:
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
