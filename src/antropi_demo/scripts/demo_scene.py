#!/usr/bin/env python3
"""Display mission endpoints and animate the test obstacle, not the robot."""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray


class DemoScene(Node):
    def __init__(self):
        super().__init__('demo_scene')
        self.declare_parameter('dynamic_obstacle', True)
        self.declare_parameter('obstacle_period', 36.0)
        self.moving = self.get_parameter('dynamic_obstacle').value
        self.period = self.get_parameter('obstacle_period').value
        if self.period <= 0:
            raise ValueError('obstacle_period must be positive')
        self.elapsed = 0.0
        self.previous_time = None
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'
        self.goal.pose.position.x = 6.0
        self.goal.pose.orientation.w = 1.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/demo/markers', latched)
        self.command = self.create_publisher(Float64, '/demo/obstacle_target', 1)
        self.create_subscription(PoseStamped, '/demo/goal_pose', self.set_goal, latched)
        self.create_subscription(PoseStamped, '/goal_pose', self.set_goal, 10)
        self.create_service(SetBool, '/demo/set_obstacle_motion', self.set_motion)
        self.create_timer(0.1, self.move_obstacle)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            f'Obstacle motion {"enabled" if self.moving else "paused"}; '
            f'period={self.period:.1f}s. Nav2 receives its geometry only through /scan.')

    def set_goal(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning('Goal marker expects the map frame; ignoring other frame.')
            return
        self.goal = msg
        self.publish_markers()

    def set_motion(self, request, response):
        self.moving = request.data
        response.success = True
        response.message = 'Obstacle motion resumed' if self.moving else 'Obstacle held at current target'
        self.get_logger().info(response.message)
        return response

    def move_obstacle(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0:
            return
        if self.previous_time is not None and self.moving:
            self.elapsed += max(0.0, now - self.previous_time)
        self.previous_time = now
        # This is a test-scene motion script. It has no robot pose, path,
        # costmap or velocity subscription and cannot steer the TurtleBot.
        self.command.publish(Float64(data=0.85 * math.sin(2 * math.pi * self.elapsed / self.period)))

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        result = MarkerArray()

        def marker(index, kind, x, y, z, sx, sy, sz, color, text=''):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'mission_endpoints'
            m.id, m.type, m.action = index, kind, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.text = text
            result.markers.append(m)
            return m

        green, blue = (0.1, 0.9, 0.3, 0.9), (0.1, 0.5, 1.0, 0.95)
        marker(0, Marker.CYLINDER, 0.0, 0.0, 0.025, 0.65, 0.65, 0.02, green)
        marker(1, Marker.TEXT_VIEW_FACING, 0.0, -0.5, 0.4, 0.0, 0.0, 0.28, green, 'START')
        p = self.goal.pose.position
        marker(2, Marker.CYLINDER, p.x, p.y, 0.025, 0.65, 0.65, 0.02, (0.1, 0.5, 1.0, 0.35))
        arrow = marker(3, Marker.ARROW, p.x, p.y, 0.12, 0.7, 0.13, 0.13, blue)
        arrow.pose.orientation = self.goal.pose.orientation
        marker(4, Marker.TEXT_VIEW_FACING, p.x, p.y-0.5, 0.4, 0.0, 0.0, 0.28,
               blue, f'GOAL ({p.x:.1f}, {p.y:.1f})')
        self.markers.publish(result)


def main():
    rclpy.init()
    node = DemoScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
