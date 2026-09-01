import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class V2VNode(Node):
    def __init__(self):
        super().__init__("v2v_node")

        self.declare_parameter("peer_topic", "/dumper2/v2v/state")
        self.declare_parameter("warning_distance", 10.0)
        self.declare_parameter("world_offset_x", 0.0)
        self.declare_parameter("world_offset_y", 0.0)

        peer_topic = self.get_parameter("peer_topic").value
        self.warning_distance = self.get_parameter("warning_distance").value
        self.offset_x = self.get_parameter("world_offset_x").value
        self.offset_y = self.get_parameter("world_offset_y").value

        self.latest_odom = None
        self.peer_state = None

        self.state_pub = self.create_publisher(
            Odometry,
            "v2v/state",
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
            peer_topic,
            self.peer_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.process_v2v
        )

        self.get_logger().info(
            f"V2V active. Receiving peer state from {peer_topic}"
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def peer_callback(self, msg):
        self.peer_state = msg

    def process_v2v(self):
        if self.latest_odom is None:
            return

        # Convert local odometry position to world position
        world_x = (
            self.latest_odom.pose.pose.position.x
            + self.offset_x
        )

        world_y = (
            self.latest_odom.pose.pose.position.y
            + self.offset_y
        )

        state = Odometry()
        state.header = self.latest_odom.header
        state.header.frame_id = "mine_world"
        state.child_frame_id = self.latest_odom.child_frame_id

        state.pose.pose = self.latest_odom.pose.pose
        state.pose.pose.position.x = world_x
        state.pose.pose.position.y = world_y

        state.twist = self.latest_odom.twist

        self.state_pub.publish(state)

        if self.peer_state is None:
            return

        x1 = world_x
        y1 = world_y

        x2 = self.peer_state.pose.pose.position.x
        y2 = self.peer_state.pose.pose.position.y

        distance = math.sqrt(
            (x2 - x1) ** 2 +
            (y2 - y1) ** 2
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