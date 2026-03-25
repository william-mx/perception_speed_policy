import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy

from vision_msgs.msg import Detection2DArray, LabelInfo
from std_msgs.msg import Float32

from ros2_pydata import from_detection2d_array, from_label_info


class PerceptionSpeedGuard(Node):
    def __init__(self):
        super().__init__('perception_speed_guard')

        # QoS for sensor data
        self.qos_profile = qos_profile_sensor_data
        self.qos_profile.depth = 1

        # QoS for label mapping (transient local — latched)
        qos_transient = QoSProfile(depth=1)
        qos_transient.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Optional label mapping subscription
        self.label_sub = self.create_subscription(
            LabelInfo,
            '/label_mapping',
            self.label_mapping_callback,
            qos_transient,
        )

        # Detection subscription
        self.det_sub = self.create_subscription(
            Detection2DArray,
            '/detections_2d',
            self.detection_callback,
            self.qos_profile,
        )

        # Speed limit publisher
        self.speed_limit_pub = self.create_publisher(
            Float32,
            '/speed_limit',
            self.qos_profile,
        )

    def label_mapping_callback(self, msg: LabelInfo):
        self.id2label = from_label_info(msg)
        self.label2id = {lbl: idx for idx, lbl in self.id2label.items()}
        self.get_logger().info(f"Label mapping received: {self.id2label}")

    def detection_callback(self, msg: Detection2DArray):
        all_detections = from_detection2d_array(msg)

        for detection in all_detections:
            if detection.score > 0.0:
                self.get_logger().info(f"{detection}")


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSpeedGuard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("KeyboardInterrupt: shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()