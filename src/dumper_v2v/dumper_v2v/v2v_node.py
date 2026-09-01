import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class V2VNode(Node):

    def __init__(self):
        super().__init__("v2v_node")

        self.declare_parameter("peer_topic", "/dumper2/v2v/state")
        self.declare_parameter("world_offset_x", 0.0)
        self.declare_parameter("world_offset_y", 0.0)
        self.declare_parameter("warning_distance", 10.0)

        self.peer_topic = self.get_parameter("peer_topic").value
        self.offset_x = self.get_parameter("world_offset_x").value
        self.offset_y = self.get_parameter("world_offset_y").value
        self.warning_distance = self.get_parameter("warning_distance").value

        self.latest_state = None
        self.peer_state = None

        self.state_pub = self.create_publisher(
            Odometry,
            "v2v/state",
            10
        )

        # Use EKF filtered state for Dumper 1.
        # If Dumper 2 does not have EKF, its normal odom will be used.
        self.state_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.state_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )

        self.peer_sub = self.create_subscription(
            Odometry,
            self.peer_topic,
            self.peer_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.process_v2v
        )

        self.get_logger().info(
            f"V2V active. Receiving peer state from {self.peer_topic}"
        )

    def state_callback(self, msg):
        self.latest_state = msg

    def odom_callback(self, msg):
        # Fallback for Dumper 2 or when EKF is unavailable.
        if self.latest_state is None:
            self.latest_state = msg

    def peer_callback(self, msg):
        self.peer_state = msg

    def process_v2v(self):

        if self.latest_state is None:
            return

        x = (
            self.latest_state.pose.pose.position.x
            + self.offset_x
        )

        y = (
            self.latest_state.pose.pose.position.y
            + self.offset_y
        )

        state = Odometry()

        state.header = self.latest_state.header
        state.header.frame_id = "mine_world"

        state.child_frame_id = self.latest_state.child_frame_id

        state.pose.pose = self.latest_state.pose.pose
        state.pose.pose.position.x = x
        state.pose.pose.position.y = y

        state.twist = self.latest_state.twist

        self.state_pub.publish(state)

        if self.peer_state is None:
            return

        peer_x = self.peer_state.pose.pose.position.x
        peer_y = self.peer_state.pose.pose.position.y

        distance = math.hypot(
            peer_x - x,
            peer_y - y
        )

        if distance < self.warning_distance:
            self.get_logger().warn(
                f"V2V WARNING: Dumper distance = {distance:.2f} m"
            )


def main(args=None):

    rclpy.init(args=args)

    node = V2VNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()