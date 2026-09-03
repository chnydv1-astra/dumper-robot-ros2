import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import tf2_ros


class CollisionPredictor(Node):

    def __init__(self):
        super().__init__("collision_predictor")

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

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

        self.warning_distance = float(
            self.get_parameter("warning_distance").value
        )

        self.critical_distance = float(
            self.get_parameter("critical_distance").value
        )

        self.critical_ttc = float(
            self.get_parameter("critical_ttc").value
        )

        # -------------------------------------------------
        # V2V state
        # -------------------------------------------------

        self.dumper1_state = None
        self.dumper2_state = None

        self.dumper1_sub = self.create_subscription(
            Odometry,
            "/dumper1/v2v/state",
            self.dumper1_callback,
            10
        )

        self.dumper2_sub = self.create_subscription(
            Odometry,
            peer_topic,
            self.dumper2_callback,
            10
        )

        # -------------------------------------------------
        # TF2 / SLAM
        # -------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # -------------------------------------------------
        # Emergency stop publishers
        # -------------------------------------------------

        self.dumper1_cmd_pub = self.create_publisher(
            Twist,
            "/dumper1/cmd_vel",
            10
        )

        self.dumper2_cmd_pub = self.create_publisher(
            Twist,
            "/dumper2/cmd_vel",
            10
        )

        # -------------------------------------------------
        # Emergency-stop latch
        # -------------------------------------------------

        self.emergency_stop = False

        # -------------------------------------------------
        # Timer
        # -------------------------------------------------

        self.timer = self.create_timer(
            0.1,
            self.calculate_collision_risk
        )

        self.get_logger().info(
            "Collision Predictor active"
        )

        self.get_logger().info(
            "Collision frame: mine_world"
        )

        self.get_logger().info(
            "SLAM TF: map -> dumper1/base_link"
        )

        self.get_logger().info(
            f"Monitoring peer: {peer_topic}"
        )

    # =====================================================
    # Callbacks
    # =====================================================

    def dumper1_callback(self, msg):
        self.dumper1_state = msg

    def dumper2_callback(self, msg):
        self.dumper2_state = msg

    # =====================================================
    # Get SLAM pose
    # =====================================================

    def get_slam_pose(self):

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

    # =====================================================
    # Emergency stop
    # =====================================================

    def stop_dumpers(self):

        stop = Twist()

        # Dumper 1
        self.dumper1_cmd_pub.publish(stop)

        # Dumper 2
        self.dumper2_cmd_pub.publish(stop)

    # =====================================================
    # Collision calculation
    # =====================================================

    def calculate_collision_risk(self):

        if self.dumper1_state is None:
            return

        if self.dumper2_state is None:
            return

        # -------------------------------------------------
        # If emergency stop has already been triggered,
        # continuously publish zero velocity.
        # -------------------------------------------------

        if self.emergency_stop:

            self.stop_dumpers()

            self.get_logger().error(
                "EMERGENCY STOP LATCHED | "
                "Both dumpers commanded to STOP"
            )

            return

        # -------------------------------------------------
        # Get SLAM pose
        #
        # SLAM is monitored here, but collision coordinates
        # remain in mine_world because both V2V states are
        # already correctly aligned there.
        # -------------------------------------------------

        slam_pose = self.get_slam_pose()

        if slam_pose is None:

            self.get_logger().warn(
                "SLAM TF unavailable"
            )

        # -------------------------------------------------
        # Dumper1 position
        # -------------------------------------------------

        my_x = (
            self.dumper1_state
            .pose.pose.position.x
        )

        my_y = (
            self.dumper1_state
            .pose.pose.position.y
        )

        # -------------------------------------------------
        # Dumper2 position
        # -------------------------------------------------

        peer_x = (
            self.dumper2_state
            .pose.pose.position.x
        )

        peer_y = (
            self.dumper2_state
            .pose.pose.position.y
        )

        # -------------------------------------------------
        # Relative position
        # -------------------------------------------------

        dx = peer_x - my_x
        dy = peer_y - my_y

        distance = math.hypot(
            dx,
            dy
        )

        if distance < 0.001:
            return

        # -------------------------------------------------
        # Velocities
        # -------------------------------------------------

        my_vx = (
            self.dumper1_state
            .twist.twist.linear.x
        )

        my_vy = (
            self.dumper1_state
            .twist.twist.linear.y
        )

        peer_vx = (
            self.dumper2_state
            .twist.twist.linear.x
        )

        peer_vy = (
            self.dumper2_state
            .twist.twist.linear.y
        )

        # -------------------------------------------------
        # Relative velocity
        # -------------------------------------------------

        relative_vx = peer_vx - my_vx
        relative_vy = peer_vy - my_vy

        # -------------------------------------------------
        # Closing speed
        # -------------------------------------------------

        ux = dx / distance
        uy = dy / distance

        closing_speed = -(
            relative_vx * ux +
            relative_vy * uy
        )

        # -------------------------------------------------
        # Time To Collision
        # -------------------------------------------------

        if closing_speed > 0.01:

            ttc = (
                distance /
                closing_speed
            )

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
        # Emergency stop
        # -------------------------------------------------

        if risk == "CRITICAL":

            self.emergency_stop = True

            self.stop_dumpers()

            slam_text = "N/A"

            if slam_pose is not None:

                slam_text = (
                    f"({slam_pose[0]:.2f}, "
                    f"{slam_pose[1]:.2f})"
                )

            self.get_logger().error(
                f"🚨 EMERGENCY STOP | "
                f"Distance={distance:.2f} m | "
                f"Closing Speed={closing_speed:.2f} m/s | "
                f"TTC={ttc:.2f} s | "
                f"SLAM={slam_text}"
            )

        elif risk == "WARNING":

            self.get_logger().warn(
                f"COLLISION RISK: WARNING | "
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

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()