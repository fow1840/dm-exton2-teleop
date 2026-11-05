import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class JointToPoseAdapter(Node):
    def __init__(self):
        super().__init__('joint_to_pose_adapter')

        self.base_frame = self.declare_parameter('base_frame', 'waist_pitch_link').value
        self.left_ee_frame = self.declare_parameter('left_ee_frame', 'left_wrist_roll_link').value
        self.right_ee_frame = self.declare_parameter('right_ee_frame', 'right_wrist_roll_link').value
        self.publish_ee_state = bool(
            self.declare_parameter('publish_ee_state', True).value
        )
        # 按下 Clutch 时改由 ik_node 发指令末端，避免抢发
        self.yield_when_clutch = bool(
            self.declare_parameter('yield_when_clutch', True).value
        )

        self.clutch = {'left': False, 'right': False}

        self.left_state_pub = self.create_publisher(PoseStamped, '/target_robot/left_ee/state', 10)
        self.right_state_pub = self.create_publisher(PoseStamped, '/target_robot/right_ee/state', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, '/client_robot/joint_states', self.joint_callback, 10)
        self.create_subscription(Bool, '/clutch/left', lambda m: self._clutch_cb(m, 'left'), 10)
        self.create_subscription(Bool, '/clutch/right', lambda m: self._clutch_cb(m, 'right'), 10)

        self.get_logger().info(
            f'FK adapter active (publish_ee_state={self.publish_ee_state}, '
            f'yield_when_clutch={self.yield_when_clutch})'
        )

    def _clutch_cb(self, msg, side):
        self.clutch[side] = bool(msg.data)

    def joint_callback(self, msg):
        if not self.publish_ee_state:
            return
        if self.yield_when_clutch and any(self.clutch.values()):
            return
        self.publish_ee_pose(self.left_ee_frame, self.left_state_pub)
        self.publish_ee_pose(self.right_ee_frame, self.right_state_pub)

    def publish_ee_pose(self, ee_frame, publisher):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                ee_frame,
                rclpy.time.Time(),
            )
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.base_frame
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation
            publisher.publish(pose_msg)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = JointToPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
