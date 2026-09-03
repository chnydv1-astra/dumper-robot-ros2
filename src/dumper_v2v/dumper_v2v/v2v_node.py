import copy
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class V2VNode(Node):

    def __init__(self):
        super().__init__('v2v_node')

        self.declare_parameter('odom_topic', '/dumper1/odom')
        self.declare_parameter('peer_topic', '/dumper2/v2v/state')
        self.declare_parameter('world_offset_x', 0.0)
        self.declare_parameter('world_offset_y', 0.0)
        self.declare_parameter('yaw_offset', 0.0)

        odom_topic = self.get_parameter('odom_topic').value
        peer_topic = self.get_parameter('peer_topic').value

        self.offset_x = float(
            self.get_parameter('world_offset_x').value
        )
        self.offset_y = float(
            self.get_parameter('world_offset_y').value
        )
        self.yaw_offset = float(
            self.get_parameter('yaw_offset').value
        )

        self.latest_state = None
        self.peer_state = None

        self.state_pub = self.create_publisher(
            Odometry,
            'v2v/state',
            10
        )

        self.state_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.state_callback,
            10
        )

        self.peer_sub = self.create_subscription(
            Odometry,
            peer_topic,
            self.peer_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.process_v2v
        )

    def state_callback(self, msg):
        self.latest_state = msg

    def peer_callback(self, msg):
        self.peer_state = msg

    def process_v2v(self):

        if self.latest_state is None:
            return

        local_x = self.latest_state.pose.pose.position.x
        local_y = self.latest_state.pose.pose.position.y

        cos_yaw = math.cos(self.yaw_offset)
        sin_yaw = math.sin(self.yaw_offset)

        world_x = (
            cos_yaw * local_x
            - sin_yaw * local_y
            + self.offset_x
        )

        world_y = (
            sin_yaw * local_x
            + cos_yaw * local_y
            + self.offset_y
        )

        state = Odometry()

        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'mine_world'
        state.child_frame_id = self.latest_state.child_frame_id

        state.pose.pose = copy.deepcopy(
            self.latest_state.pose.pose
        )

        state.pose.pose.position.x = world_x
        state.pose.pose.position.y = world_y

        local_vx = self.latest_state.twist.twist.linear.x
        local_vy = self.latest_state.twist.twist.linear.y

        world_vx = (
            cos_yaw * local_vx
            - sin_yaw * local_vy
        )

        world_vy = (
            sin_yaw * local_vx
            + cos_yaw * local_vy
        )

        state.twist = copy.deepcopy(
            self.latest_state.twist
        )

        state.twist.twist.linear.x = world_vx
        state.twist.twist.linear.y = world_vy

        self.state_pub.publish(state)


def main(args=None):

    rclpy.init(args=args)

    node = V2VNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()