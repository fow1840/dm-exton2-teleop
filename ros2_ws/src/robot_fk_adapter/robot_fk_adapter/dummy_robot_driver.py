import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import threading

class DummyRobotDriver(Node):
    def __init__(self):
        super().__init__('dummy_robot_driver')
        
        # 1. 声明参数
        self.declare_parameter('urdf_path', '')
        # 【新增】声明频率参数，默认给一个值（比如 100.0）
        self.declare_parameter('publish_frequency', 100.0) 
        
        urdf_path = self.get_parameter('urdf_path').value
        # 【新增】获取 Launch 文件传进来的频率
        freq = self.get_parameter('publish_frequency').value
        
        self.get_logger().info(f"Using URDF: {urdf_path}")
        self.get_logger().info(f"Publish Frequency: {freq} Hz") # 打印确认

        # 解析 URDF 获取所有非固定关节名称
        self.all_joint_names = self.parse_urdf_joints(urdf_path)
        
        # 初始化所有关节为 0.0
        self.current_positions = {name: 0.0 for name in self.all_joint_names}
        self.lock = threading.Lock()

        # 发布全量状态
        self.pub_state = self.create_publisher(JointState, '/client_robot/joint_states', 10)

        # 接收 IK 产生的关节目标指令 
        self.sub_cmd = self.create_subscription(
            JointState, '/target_robot/joint_commands', self.command_callback, 10)

        # 【修复】使用参数计算周期 (Period = 1.0 / Frequency)
        timer_period = 1.0 / freq
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def parse_urdf_joints(self, path):
        joints = []
        try:
            # 注意：此处建议增加 path 检查，防止空路径报错
            if not path:
                self.get_logger().warn("URDF path is empty!")
                return []
            root = ET.parse(path).getroot()
            for j in root.findall('joint'):
                if j.get('type') in ['revolute', 'continuous', 'prismatic']:
                    joints.append(j.get('name'))
        except Exception as e:
            self.get_logger().error(f"URDF Error: {e}")
        return joints

    def command_callback(self, msg):
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.current_positions:
                    self.current_positions[name] = msg.position[i] 

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = self.all_joint_names 
            # 确保字典键值对齐
            msg.position = [self.current_positions[n] for n in self.all_joint_names]
        self.pub_state.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DummyRobotDriver())
    rclpy.shutdown()