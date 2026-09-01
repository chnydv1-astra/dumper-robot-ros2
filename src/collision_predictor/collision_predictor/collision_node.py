import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class CollisionPredictor(Node):

    def __init__(self):
        super().__init__("collision_predictor")

        self.declare_parameter(
            "peer_topic",
            "/dumper2/v2v/state"
        )

        self.declare_parameter(
            "warning_distance",
            15.0
        )

        self.declare_parameter(
            "critical_distance",
            8.0
        )

        self.declare_parameter(
            "critical_ttc",
            3.0
        )

        peer_topic = self.get_parameter(
            "peer_topic"
        ).value

        self.warning_distance = self.get_parameter(
            "warning_distance"
        ).value

        self.critical_distance = self.get_parameter(
            "critical_distance"
        ).value

        self.critical_ttc = self.get_parameter(
            "critical_ttc"
        ).value

        self.my_state = None
        self.peer_state = None

        self.my_sub = self.create_subscription(
            Odometry,
            "/dumper1/odometry/filtered",
            self.my_state_callback,
            10
        )

        self.peer_sub = self.create_subscription(
            Odometry,
            peer_topic,
            self.peer_state_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.calculate_collision_risk
        )

        self.get_logger().info(
            "Collision Predictor active"
        )

        self.get_logger().info(
            f"Monitoring peer: {peer_topic}"
        )

    def my_state_callback(self, msg):
        self.my_state = msg

    def peer_state_callback(self, msg):
        self.peer_state = msg

    def calculate_collision_risk(self):

        if self.my_state is None:
            return

        if self.peer_state is None:
            return

        # Current positions
        my_x = self.my_state.pose.pose.position.x
        my_y = self.my_state.pose.pose.position.y

        peer_x = self.peer_state.pose.pose.position.x
        peer_y = self.peer_state.pose.pose.position.y

        # Relative position
        dx = peer_x - my_x
        dy = peer_y - my_y

        distance = math.hypot(dx, dy)

        if distance < 0.001:
            return

        # Relative velocity
        my_vx = self.my_state.twist.twist.linear.x
        my_vy = self.my_state.twist.twist.linear.y

        peer_vx = self.peer_state.twist.twist.linear.x
        peer_vy = self.peer_state.twist.twist.linear.y

        relative_vx = peer_vx - my_vx
        relative_vy = peer_vy - my_vy

        # Unit vector from Dumper 1 to Dumper 2
        ux = dx / distance
        uy = dy / distance

        # Closing speed
        closing_speed = -(
            relative_vx * ux +
            relative_vy * uy
        )

        # TTC calculation
        if closing_speed > 0.01:
            ttc = distance / closing_speed
        else:
            ttc = float("inf")

        # Risk classification
        if (
            distance <= self.critical_distance
            or ttc <= self.critical_ttc
        ):
            risk = "CRITICAL"

        elif distance <= self.warning_distance:
            risk = "WARNING"

        else:
            risk = "SAFE"

        # Print only when useful
        if risk == "CRITICAL":

            self.get_logger().error(
                f"COLLISION RISK: {risk} | "
                f"Distance={distance:.2f} m | "
                f"Closing Speed={closing_speed:.2f} m/s | "
                f"TTC={ttc:.2f} s"
            )

        elif risk == "WARNING":

            self.get_logger().warn(
                f"COLLISION RISK: {risk} | "
                f"Distance={distance:.2f} m | "
                f"TTC={ttc:.2f} s"
            )

        else:

            self.get_logger().info(
                f"Collision status: SAFE | "
                f"Distance={distance:.2f} m"
            )


def main(args=None):

    rclpy.init(args=args)

    node = CollisionPredictor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
