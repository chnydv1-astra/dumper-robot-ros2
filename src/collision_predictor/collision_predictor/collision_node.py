import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import tf2_ros


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

        self.peer_state = None

        # TF2 listener for SLAM pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
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
            "Dumper1 position source: SLAM map frame"
        )

        self.get_logger().info(
            f"Monitoring peer: {peer_topic}"
        )

    def peer_state_callback(self, msg):
        self.peer_state = msg

    def get_dumper1_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "dumper1/base_link",
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return x, y

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            return None

    def calculate_collision_risk(self):

        if self.peer_state is None:
            return

        # -------------------------------------------------
        # Dumper1 position from SLAM
        # -------------------------------------------------

        dumper1_pose = self.get_dumper1_pose()

        if dumper1_pose is None:
            self.get_logger().warn(
                "Waiting for SLAM pose..."
            )
            return

        map_x, map_y = dumper1_pose

        # -------------------------------------------------
        # IMPORTANT:
        # Current SLAM map and mine_world have the same
        # orientation in our simulation.
        #
        # We establish the mine_world origin from the
        # initial SLAM pose.
        # -------------------------------------------------

        if not hasattr(self, "slam_origin_x"):

            self.slam_origin_x = map_x
            self.slam_origin_y = map_y

            self.get_logger().info(
                f"SLAM reference initialized: "
                f"map=({map_x:.2f}, {map_y:.2f})"
            )

        # Convert SLAM displacement into mine_world
        my_x = map_x - self.slam_origin_x
        my_y = map_y - self.slam_origin_y

        # -------------------------------------------------
        # Dumper2 V2V position
        # -------------------------------------------------

        peer_x = self.peer_state.pose.pose.position.x
        peer_y = self.peer_state.pose.pose.position.y

        # -------------------------------------------------
        # Relative position
        # -------------------------------------------------

        dx = peer_x - my_x
        dy = peer_y - my_y

        distance = math.hypot(dx, dy)

        if distance < 0.001:
            return

        # -------------------------------------------------
        # Relative velocity
        # -------------------------------------------------

        peer_vx = self.peer_state.twist.twist.linear.x
        peer_vy = self.peer_state.twist.twist.linear.y

        # SLAM position is used for position.
        # Dumper1 velocity comes from the V2V state.
        #
        # The V2V velocity is already in mine_world.
        #
        # This keeps the existing velocity/TTC system.
        # -------------------------------------------------

        # Estimate Dumper1 velocity from SLAM position
        current_time = self.get_clock().now()

        if not hasattr(self, "previous_x"):

            self.previous_x = my_x
            self.previous_y = my_y
            self.previous_time = current_time

            return

        dt = (
            current_time - self.previous_time
        ).nanoseconds / 1e9

        if dt <= 0.0 or dt > 1.0:
            self.previous_x = my_x
            self.previous_y = my_y
            self.previous_time = current_time
            return

        my_vx = (my_x - self.previous_x) / dt
        my_vy = (my_y - self.previous_y) / dt

        self.previous_x = my_x
        self.previous_y = my_y
        self.previous_time = current_time

        # -------------------------------------------------
        # Relative velocity
        # -------------------------------------------------

        relative_vx = peer_vx - my_vx
        relative_vy = peer_vy - my_vy

        # Unit vector Dumper1 -> Dumper2
        ux = dx / distance
        uy = dy / distance

        # Closing speed
        closing_speed = -(
            relative_vx * ux +
            relative_vy * uy
        )

        # TTC
        if closing_speed > 0.01:
            ttc = distance / closing_speed
        else:
            ttc = float("inf")

        # -------------------------------------------------
        # Risk classification
        # -------------------------------------------------

        if (
            distance <= self.critical_distance
            or ttc <= self.critical_ttc
        ):
            risk = "CRITICAL"

        elif distance <= self.warning_distance:
            risk = "WARNING"

        else:
            risk = "SAFE"

        # -------------------------------------------------
        # Output
        # -------------------------------------------------

        if risk == "CRITICAL":

            self.get_logger().error(
                f"COLLISION RISK: {risk} | "
                f"Distance={distance:.2f} m | "
                f"Closing Speed={closing_speed:.2f} m/s | "
                f"TTC={ttc:.2f} s | "
                f"SLAM=({my_x:.2f}, {my_y:.2f})"
            )

        elif risk == "WARNING":

            self.get_logger().warn(
                f"COLLISION RISK: {risk} | "
                f"Distance={distance:.2f} m | "
                f"TTC={ttc:.2f} s | "
                f"SLAM=({my_x:.2f}, {my_y:.2f})"
            )

        else:

            self.get_logger().info(
                f"Collision status: SAFE | "
                f"Distance={distance:.2f} m | "
                f"SLAM=({my_x:.2f}, {my_y:.2f})"
            )


def main(args=None):

    rclpy.init(args=args)

    node = CollisionPredictor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()