#!/usr/bin/env python3

import math
import carla

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from carla_interface import carla_utils


def carla_rotation_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    Convert a CARLA rotation (left-handed, degrees) to a ROS quaternion (right-handed).
    Conversion: negate pitch and yaw to go from left-hand to right-hand coordinate system.
    """
    roll  = math.radians(roll_deg)
    pitch = math.radians(-pitch_deg)
    yaw   = math.radians(-yaw_deg)

    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class HeroTFBroadcaster(Node):
    def __init__(self):
        super().__init__('hero_tf_broadcaster')

        # Parameters
        self.server_host = self.declare_parameter('server_connection.host', 'localhost').get_parameter_value().string_value
        self.server_port = self.declare_parameter('server_connection.port', 2000).get_parameter_value().integer_value
        self.timeout     = self.declare_parameter('server_connection.timeout', 10.0).get_parameter_value().double_value

        # TF broadcaster — map is the root frame (no parent needed)
        self._dynamic_broadcaster = TransformBroadcaster(self)

        # Connect to CARLA and find the hero vehicle
        self.get_logger().info(f'Connecting to CARLA server at {self.server_host}:{self.server_port}')
        client = carla.Client(self.server_host, self.server_port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()

        self.hero = carla_utils.get_hero_vehicle(self.world, timeout=self.timeout)
        if self.hero is None:
            self.get_logger().error('Hero vehicle not found - no hero TF will be published')
            return

        self.get_logger().info('Hero vehicle found, starting TF broadcast')

        # Use a ROS timer so TF is published from within the ROS executor (thread-safe)
        self.create_timer(0.05, self._publish_hero_tf)  # 20 Hz


    def _publish_hero_tf(self):
        """Timer callback: publishes map -> hero TF at 20 Hz."""
        try:
            carla_tf = self.hero.get_transform()
        except Exception as e:
            self.get_logger().warn(f'Failed to get hero transform: {e}')
            return

        loc = carla_tf.location
        rot = carla_tf.rotation

        # Convert CARLA (left-handed) coordinates to ROS (right-handed)
        ros_x  =  loc.x
        ros_y  = -loc.y
        ros_z  =  loc.z
        qx, qy, qz, qw = carla_rotation_to_quaternion(rot.roll, rot.pitch, rot.yaw)

        t = TransformStamped()
        t.header.stamp            = self.get_clock().now().to_msg()
        t.header.frame_id         = 'map'
        t.child_frame_id          = 'hero'
        t.transform.translation.x = ros_x
        t.transform.translation.y = ros_y
        t.transform.translation.z = ros_z
        t.transform.rotation.x    = qx
        t.transform.rotation.y    = qy
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw

        self._dynamic_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = HeroTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
