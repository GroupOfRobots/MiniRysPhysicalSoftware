#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from picamera2 import Picamera2
from pid import PID
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

class LineFollower(Node):

    img = None
    picam2 = None
    pid = None
    #points = [(220, 270),(220, 230),(220, 200),(220, 170)]
    points = [(220, 200)]
    leftT = (435, 270)
    rightT = (5, 270)
    y = 0

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
        # PID
        self.pid = PID(timer_period,K,Ti,Td)
        self.is_working = True
        #self.logger.info(f"end")



    def timer_callback(self):
        #self.logger.info(f"running{self.isWorking}")
        if self.is_working:
            yuv = self.picam2.capture_array("lores")
            self.img = cv2.cvtColor(yuv, cv2.COLOR_YUV420p2RGB)[0:450, 100:540]
            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            kernel = np.ones((5, 5), np.uint8)
            img_erosion = cv2.erode(gray, kernel, iterations=1)
            ret,thresh = cv2.threshold(img_erosion, 127, 255, cv2.THRESH_BINARY_INV)
            #Negative binn
            #thresh = cv2.bitwise_not(thresh)
            br = CvBridge()
            msg = br.cv2_to_imgmsg(thresh)
            self.publisher1_.publish(msg)
            # contours
            contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
            img2 = self.img
            # biggest contour
            areas = [cv2.contourArea(c) for c in contours]

            # getting position where robot should be
            turnOfset = 0
            if not np.size(areas) == 0:
                max_index = np.argmax(areas)
                cnt=contours[max_index]

                sum_dist = 0
                for p in self.points:
                    x_value = 0
                    y_value = p[1]
                    i = 0
                    for point in cnt:
                        if point[0][1] == y_value:
                            x_value = point[0][0] + x_value
                            i = i +1
                    if not i == 0:
                        sum_dist = sum_dist + x_value/i - p[0]
                        cv2.circle(img2, p, radius=5, color=(0, 0, 255), thickness=-1)
                        cv2.circle(img2, (int(x_value/i), y_value), radius=5, color=(255, 0, 0), thickness=-1)
                self.y = sum_dist/len(self.points)

                # recognizing 90 turn
                if cv2.pointPolygonTest(cnt, self.leftT, False) >= 0 and cv2.pointPolygonTest(cnt, self.rightT, False) < 0 :
                    turnOfset = -self.turnOffsetParam#-1.0 #-1
                elif cv2.pointPolygonTest(cnt, self.leftT, False) < 0 and cv2.pointPolygonTest(cnt, self.rightT, False) >= 0 :
                    turnOfset = self.turnOffsetParam#1.0 #1
            u = self.pid.pid(self.y,0) + turnOfset

            if u > self.maxU:
                u = self.maxU

            if u < self.minU:
                u = self.minU


            # publishing linear and angular robot speed
            cmd = Twist()
            cmd.linear.x = self.speed
            cmd.angular.z = u
            self.publisher3_.publish(cmd)
            if not np.size(areas) == 0:
                cv2.drawContours(image=img2, contours=[cnt], contourIdx=-1, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
            msg = br.cv2_to_imgmsg(img2)
            self.publisher2_.publish(msg)

    def subscr_callback(self, msg):
        self.is_working = msg.data



def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = LineFollower()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
