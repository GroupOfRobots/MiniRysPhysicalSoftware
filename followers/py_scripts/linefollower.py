#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from picamera2 import Picamera2
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from linefollower_module import LineFollowerModule, LineFollowerModuleError

class LineFollower(Node):
    bridge = CvBridge()
    is_working = True
    picam2 = None

    def __init__(self):
        super().__init__('line_follower')
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
                ('linearSpeed', Parameter.Type.DOUBLE)
            ]
        )
        self.maxU = self.get_parameter("maxU").get_parameter_value().double_value
        self.minU = -self.maxU
        self.speed = self.get_parameter("linearSpeed").get_parameter_value().double_value
        self.turnOffsetParam = self.get_parameter("turnOffsetParam").get_parameter_value().double_value
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value
        K = self.get_parameter("K").get_parameter_value().double_value
        Ti = self.get_parameter("Ti").get_parameter_value().double_value
        Td = self.get_parameter("Td").get_parameter_value().double_value

        self.logger.info(f"timer_period: { timer_period}, maxU: {self.maxU}, K: {K}, Ti:{Ti}")
        self.logger.info(f"Td: {Td},turnOffsetParam: {self.turnOffsetParam}, speed: {self.speed}, namespace:{self.get_namespace()}")

        self.line_follower = LineFollowerModule(timer_period, K, Ti, Td, self.minU, self.maxU, self.turnOffsetParam, self.get_logger())

        # Publishers
        self.publisher1_ = self.create_publisher(Image, 'binn_img', 10)
        self.publisher2_ = self.create_publisher(Image, 'mod_img', 10)
        self.publisher3_ = self.create_publisher(Twist, 'cmd_vel', 10)

        #Subscriber
        self.subscriber = self.create_subscription(Bool, 'is_line_follower', self.subscr_callback, 10)
        # Timer

        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Camera configuration
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(lores={"size": (640,480)})
        self.picam2.configure(config)
        self.picam2.start()
        #self.logger.info(f"end")

    def timer_callback(self):
        #self.logger.info(f"running{self.isWorking}")
        if not self.is_working:
            return

        yuv = self.picam2.capture_array("lores")
        image = cv2.cvtColor(yuv, cv2.COLOR_YUV420p2RGB)[0:450, 100:540]

        # publishing linear and angular robot speed
        cmd = Twist()
        cmd.linear.x = self.speed
        self.line_follower.set_image(image, LineFollowerModule.InputMode.RAW)
        u = self.line_follower.compute_angular_velocity()
        if u is not None:
            cmd.angular.z = u
        self.publisher3_.publish(cmd)

        try:
            msg = self.bridge.cv2_to_imgmsg(self.line_follower.get_image_binary())
            self.publisher1_.publish(msg)
        except LineFollowerModuleError as e:
            self.logger.info(f'Failed to obtain binary image, the error is \"{e}\"')

        try:
            msg = self.bridge.cv2_to_imgmsg(self.line_follower.get_image_debug())
            self.publisher2_.publish(msg)
        except LineFollowerModuleError as e:
            self.logger.info(f'Failed to obtain debug image, the error is \"{e}\"')

    def subscr_callback(self, msg):
        self.is_working = msg.data

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = LineFollower()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
