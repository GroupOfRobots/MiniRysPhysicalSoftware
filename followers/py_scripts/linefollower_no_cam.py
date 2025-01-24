#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.parameter import Parameter
from pid import PID
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class LineFollowerNoCamm(Node):
    img = None
    pid = None
    #minU = -5.14
    #maxU = 5.14
    #points = [(220, 270),(220, 230),(220, 200),(220, 170)]
    points = [(220, 200)]
    leftT = (435, 270)
    rightT = (5, 270)
    y = 0

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
                ('linearSpeed', Parameter.Type.DOUBLE)
            ]
        )
        self.logger.info("LLL")
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

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.thresh = None

        self.subscription = self.create_subscription(Image, '/cv_video_frames',
            self.listener_callback, 10)

        self.pid = PID(timer_period,K,Ti,Td)

        self.publisher3_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Bool, 'is_line_follower', self.subscr_callback, 10)

        self.is_working = True


    def listener_callback(self, msg):
        br = CvBridge()
        self.thresh = br.imgmsg_to_cv2(msg)

    def timer_callback(self):
        if self.thresh is not None and self.is_working:
            thresh = self.thresh
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
                    print("prawy")
                    turnOfset = -self.turnOffsetParam #-1
                elif cv2.pointPolygonTest(cnt, self.leftT, False) < 0 and cv2.pointPolygonTest(cnt, self.rightT, False) >= 0 :
                    print("lewy")
                    turnOfset = self.turnOffsetParam #1
            u = self.pid.pid(self.y,0) + turnOfset
            print(turnOfset)
            if u > self.maxU:
                u = self.maxU

            if u < self.minU:
                u = self.minU

            # publikowanie u
            cmd = Twist()
            cmd.linear.x = self.speed
            cmd.angular.z = u
            self.publisher3_.publish(cmd)

    def subscr_callback(self, msg):
        self.is_working = msg.data


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = LineFollowerNoCamm()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
