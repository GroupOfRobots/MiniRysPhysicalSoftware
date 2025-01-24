#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from linefollower_module import LineFollowerModule, LineFollowerModuleError

class LineFollowerNoCamm(Node):
    bridge = CvBridge()
    is_working = True

    def __init__(self):
        super().__init__('line_follower_no_cam')
        self.logger = self.get_logger()
        self.declare_parameters(
            namespace="",
            parameters=[
                ('timer_period', Parameter.Type.DOUBLE),
                ('maxU', Parameter.Type.DOUBLE),
                ('K', Parameter.Type.DOUBLE),
                ('Ti', Parameter.Type.DOUBLE),
                ('Td', Parameter.Type.DOUBLE),
                ('turnOffsetParam', Parameter.Type.DOUBLE),
                ('linearSpeed', Parameter.Type.DOUBLE),
                ('processedImage', Parameter.Type.BOOL)
            ]
        )
        self.logger.info("LLL")
        self.maxU = self.get_parameter("maxU").get_parameter_value().double_value
        self.minU = -self.maxU
        self.speed = self.get_parameter("linearSpeed").get_parameter_value().double_value
        self.turnOffsetParam = self.get_parameter("turnOffsetParam").get_parameter_value().double_value
        processedImage = self.get_parameter("processedImage").get_parameter_value().bool_value
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value
        K = self.get_parameter("K").get_parameter_value().double_value
        Ti = self.get_parameter("Ti").get_parameter_value().double_value
        Td = self.get_parameter("Td").get_parameter_value().double_value

        self.logger.info(f"timer_period: { timer_period}, maxU: {self.maxU}, K: {K}, Ti: {Ti}")
        self.logger.info(f"Td: {Td}, turnOffsetParam: {self.turnOffsetParam}, speed: {self.speed}"
                         + f", processedImage: {processedImage}, namespace:{self.get_namespace()}")

        self.line_follower = LineFollowerModule(timer_period, K, Ti, Td, self.minU, self.maxU, self.turnOffsetParam, self.get_logger())
        self.line_follower_input_mode = LineFollowerModule.InputMode.PROCESSED if processedImage else LineFollowerModule.InputMode.RAW

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(Image, '/cv_video_frames',
            self.listener_callback, 10)

        self.publisher3_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Bool, 'is_line_follower', self.subscr_callback, 10)
        self.publisher2_ = self.create_publisher(Image, 'mod_img', 10)

    def listener_callback(self, msg):
        thresh = self.bridge.imgmsg_to_cv2(msg)
        self.line_follower.set_image(thresh, self.line_follower_input_mode)

    def timer_callback(self):
        if not self.is_working:
            return

        # publishing linear and angular robot speed
        cmd = Twist()
        cmd.linear.x = self.speed
        u = self.line_follower.compute_angular_velocity()
        if u is not None:
            cmd.angular.z = u
        self.publisher3_.publish(cmd)

        try:
            msg = self.bridge.cv2_to_imgmsg(self.line_follower.get_image_debug())
            self.publisher2_.publish(msg)
        except LineFollowerModuleError as e:
            self.logger.info(f'Failed to obtain debug image, the error is \"{e}\"')

    def subscr_callback(self, msg):
        self.is_working = msg.data

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = LineFollowerNoCamm()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
