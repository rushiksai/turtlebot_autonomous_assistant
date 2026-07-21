import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

class DoorMonitorNode(Node):
    def _init_(self):
        super()._init_('door_monitor_node')

        # Parameters
        self.door_angle_center = -90  # Door is assumed to be straight ahead
        self.door_window = 5          # +/- degrees around center
        self.door_open_threshold = 2.0  # Difference from closed distance to consider door open
        self.door_wait_time_sec = 5.0
        self.forward_speed = 0.1
        self.forward_duration = 3.0  # seconds

        # Internal state
        self.d_closed_reference = None
        self.door_open_start_time = None
        self.door_confirmed_open = False
        self.forward_start_time = None
        self.already_drove = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/door_marker', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def get_distance_at_angle(self, msg, angle_deg):
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)
        index = max(0, min(len(msg.ranges) - 1, index))
        distance = msg.ranges[index]
        return distance if math.isfinite(distance) else float('inf')

    def get_mean_distance(self, msg):
        distances = [
            self.get_distance_at_angle(msg, self.door_angle_center + offset)
            for offset in range(-self.door_window, self.door_window + 1)
        ]
        valid = [d for d in distances if math.isfinite(d)]
        return sum(valid) / len(valid) if valid else float('inf')

    def scan_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        current_mean = self.get_mean_distance(msg)

        if self.d_closed_reference is None:
            self.d_closed_reference = current_mean
            self.get_logger().info(f"Tür-Referenz gesetzt: {self.d_closed_reference:.2f} m")
            return

        if current_mean > self.d_closed_reference + self.door_open_threshold:
            if self.door_open_start_time is None:
                self.door_open_start_time = current_time
            elif current_time - self.door_open_start_time >= self.door_wait_time_sec:
                self.door_confirmed_open = True
        else:
            self.door_open_start_time = None
            self.door_confirmed_open = False

        # --- Zusätzliche vollständige Hindernis-Visualisierung ---
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad)
                self.publish_marker(msg, angle_deg, distance)

    def control_loop(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        twist = Twist()

        if self.door_confirmed_open and not self.already_drove:
            if self.forward_start_time is None:
                self.forward_start_time = now
                self.get_logger().info("🚪 Tür offen – Starte Vorwärtsbewegung.")
            elif now - self.forward_start_time < self.forward_duration:
                twist.linear.x = self.forward_speed
            else:
                self.get_logger().info("✅ Vorwärtsbewegung abgeschlossen.")
                self.already_drove = True
        else:
            twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

    def publish_marker(self, scan_msg, angle_deg, distance):

        angle_rad = math.radians(angle_deg)

        # Punkt im Sensor-Frame berechnen
        x_laser = distance * math.cos(angle_rad)
        y_laser = distance * math.sin(angle_rad)

        point_in_laser = PointStamped()
        point_in_laser.header = scan_msg.header # frame_id = z.B. laser
        point_in_laser.point.x = x_laser
        point_in_laser.point.y = y_laser
        point_in_laser.point.z = 0.0

        try:
            # 2. Transformation in map-Frame
            transform = self.tf_buffer.lookup_transform('map', scan_msg.header.frame_id, rclpy.time.Time())
            point_in_map = do_transform_point(point_in_laser, transform)

            # 3. Marker erstellen
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacle_cylinders"
            marker.id = int(angle_deg) % 360  # eindeutige ID für jeden Winkelgrad
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.lifetime = Duration(sec=10)  # Optional: verschwindet nach 10s

            marker.pose.position.x = point_in_map.point.x
            marker.pose.position.y = point_in_map.point.y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05  # Durchmesser
            marker.scale.y = 0.05
            marker.scale.z = 0.2   # Höhe

            marker.color.r = 0.8
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.9

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f"Transformationsfehler: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DoorMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
