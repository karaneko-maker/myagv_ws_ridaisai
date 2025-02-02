#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
# import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
# import tf_transformations
import numpy as np
import math
from sklearn import linear_model
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
import time

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('Lidar Node Started')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dist = {}
        self.inliers = {}
        self.ns = 30
        self.nc = 0
        self.c_thresh = 0
        
        self.right_side_D_subscription = self.create_subscription(Bool, '/right_side_D_msg',  self.right_side_RED_callback, 10)
        self.right_side_C_subscription = self.create_subscription(Bool, '/right_side_C_msg',  self.right_side_C_callback, 10)
        self.right_side_B_subscription = self.create_subscription(Bool, '/right_side_B_msg',  self.right_side_B_callback, 10)
        self.right_side_A_subscription = self.create_subscription(Bool, '/right_side_A_msg',  self.right_side_A_callback, 10)
        self.right_side_F_subscription = self.create_subscription(Bool, '/right_side_F_msg',  self.right_side_F_callback, 10)
        self.right_side_subscription = self.create_subscription(Bool, '/right_side_msg',  self.right_side_D_callback, 10)

        self.move_forward_subscription = self.create_subscription(Bool,'/move_msg',  self.move_forward_callback, 10)
        self.is_moving_forward = False
        self.camera_move_publisher = self.create_publisher(Bool, '/camera_move_msg', 10)
        self.D_stop_publisher = self.create_publisher(Bool, '/D_stop_msg', 10)
        self.camera_move = False

        self.now = 0
        self.count_to_curve = 0
        self.flont_far = 0
        self.count_to_straight = 0
        self.check_curve_finish = 0
        self.camera_on_modes = [3, 4, 8, 15, 16, 19]
        self.last_R, self.last_C, self.last_B, self.last_A, self.last_F, self.last_D = False, False, False, False, False, False

    def right_side_RED_callback(self, msg):
        if not self.last_R and msg.data:
            self.now = 9
        self.last_R = msg.data

    def right_side_C_callback(self, msg):
        if not self.last_C and msg.data:
            self.now = 13
            msgD = Bool()
            msgD.data = False
            self.D_stop_publisher.publish(msgD)
        self.last_C = msg.data
    
    def right_side_B_callback(self, msg):
        if not self.last_B and msg.data:
            self.now = 16
        self.last_B = msg.data
    
    def right_side_A_callback(self, msg):
        if not self.last_A and msg.data:
            self.now = 17
            print(F"{self.now}")
        self.last_A = msg.data
    
    def right_side_F_callback(self, msg):
        if not self.last_F and msg.data:
            self.now = 1
        self.last_F = msg.data

    def right_side_D_callback(self, msg):
        if not self.last_D and msg.data:
            self.now = 6
        self.last_D = msg.data
    
    def move_forward_callback(self, msg):
        self.is_moving_forward = msg.data
        
        if self.is_moving_forward:
            self.get_logger().info("Move!")
        else:
            v = 0.0
            theta = 0.0
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = theta
            self.publisher.publish(twist)
            self.get_logger().info("ransac stop!")
            
    def lidar_callback(self, msg): 
        start = time.time()  
        if not self.is_moving_forward:
            return

        r = np.array(msg.ranges)
        num_points = len(r)
        xf = np.zeros(num_points)
        yf = np.zeros(num_points)
        xr = np.zeros(num_points)
        yr = np.zeros(num_points)
        f_average = 0
        r_average = 0
        for i in range(num_points):
            theta = msg.angle_min + msg.angle_increment * i
            if math.pi/4 < theta < math.pi/4*3:
                xf[i] = r[i] * np.cos(theta)
                yf[i] = r[i] * np.sin(theta)
            elif -math.pi/4 < theta or theta < math.pi/4:
                xr[i] = r[i] * np.cos(theta)
                yr[i] = r[i] * np.sin(theta)
        
        valid_indices = np.isfinite(yf)
        xf = xf[valid_indices]
        yf = yf[valid_indices]
        
        valid_indices = np.abs(yf) < 1e10
        xf = xf[valid_indices]
        yf = yf[valid_indices]

        valid_indices = ~((xf < 0.27) & (xf > -0.14) & (yf < 0.30) & (yf > -0.17))
        xf = xf[valid_indices]
        yf = yf[valid_indices]

        valid_indices = np.isfinite(yr)
        xr = xr[valid_indices]
        yr = yr[valid_indices]
        
        valid_indices = np.abs(yr) < 1e10
        xr = xr[valid_indices]
        yr = yr[valid_indices]

        valid_indices = ~((xr < 0.27) & (xr > -0.14) & (yr < 0.30) & (yr > -0.17))
        xr = xr[valid_indices]
        yr = yr[valid_indices]

        self.ransacf(xf, yf)
        self.ransacr(xr, yr)
        end =time.time()
        print(end-start)
        print(f"{self.dist, self.theta_right}")
        self.publish_v(self.dist)
    
    def ransacf(self, xf, yf, num_inliers_thresh=20):
        ransac = linear_model.RANSACRegressor(min_samples=2, residual_threshold=0.05, max_trials=200000)
        while True:
            if len(xf) < 50:
                break
            ransac.fit(xf.reshape(-1, 1), yf)
            a = ransac.estimator_.coef_
            b = ransac.estimator_.intercept_
            self.theta = math.atan(a)
            inlier_mask = ransac.inlier_mask_
            if abs(math.degrees(self.theta)) < 45 and b > 0:
                n = "front"
                self.dist[n] = abs(b)/(a**2+1)**0.5
                break
            xf = np.delete(xf, inlier_mask)
            yf = np.delete(yf, inlier_mask)
    
    def ransacr(self, xr, yr, num_inliers_thresh=20):
        ransac = linear_model.RANSACRegressor(min_samples=2, residual_threshold=0.05, max_trials=200000)
        while True:
            if len(xr) < 50:
                break
            ransac.fit(xr.reshape(-1, 1), yr)
            a = ransac.estimator_.coef_
            b = ransac.estimator_.intercept_
            self.theta = math.atan(a)
            inlier_mask = ransac.inlier_mask_
            if abs(math.degrees(self.theta)) > 45:
                if -b/a > 0:
                    n = "right"
                    point = (0.27, 0.30)
                    self.theta_right = self.theta
                    self.dist[n] = abs(a*point[0] - point[1] + b)/(a**2+1)**0.5
                    break
            xr = np.delete(xr, inlier_mask)
            yr = np.delete(yr, inlier_mask)

    def publish_v(self, dist):
        v = 0.0
        theta = 0.0
        dl = 0
        V, TH = 0, 0
        A, B = 0, 0
        R = 0.25

        if "front" in dist:
            front_OK = True
            df = dist["front"][0]
        else:
            front_OK = False
        if "right" in dist:
            right_OK = True
            dr = dist["right"][0]
        else:
            right_OK = False
        
        if right_OK:
            if self.theta_right < 0:
                theta_now = -math.pi/2 - self.theta_right
            else:
                theta_now = math.pi/2 - self.theta_right
        
        if self.now == 1:
            A = 1
            dl = 0.1
            B = 1.5
            if front_OK and right_OK:
                if df > 0.9 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.4
                elif df > 0.6 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1 and self.flont_far > 20:
                        self.now += 1
                        self.count_to_curve = 0
                        self.flont_far = 0
                    return
                if df > 0.6 + R:
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                self.flont_far += 1
                v = 0.3
            else:
                return
            theta = A*(dl-dr) - B*math.sin(theta_now) + math.pi/10
        
        elif self.now == 2:
            if -math.pi/2 < self.theta_right < -math.pi/3:
                if self.count_to_straight > 5:
                    self.count_to_straight = 0
                    self.now += 1
                    return
            else:
                self.count_to_straight += 1
            v = 0.4
            theta = 0.6 + self.count_to_straight*0.03
        
        elif self.now == 3:
            A = 1
            dl = 0.1
            B = 1.5
            if right_OK:
                if dl - dr > 0.1:
                    A = 3
                v = 0.4
            else:
                return
            theta = A*(dl-dr) - B*math.sin(theta_now) + math.pi/12
        
        elif self.now == 6:
            V = 0.35
            dl = 0.15
            Th = 3
            if front_OK and right_OK:
                if df > 0.9 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.4
                elif df > 0.55 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1 and self.flont_far > 20:
                        self.now += 1
                        self.count_to_curve = 0
                        self.flont_far = 0
                    return
                if df > 0.6 + R:
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                self.flont_far += 1
                v = 0.4
            else:
                return
            theta = V*(dl-dr) - Th*theta_now
        
        elif self.now == 7:
            if -math.pi/2 < self.theta_right < -math.pi/3:
                if self.count_to_straight > 5:
                    self.count_to_straight = 0
                    self.now += 1
                    return
            else:
                self.count_to_straight += 1
            v = 0.2
            theta = 0.8
            print(f"----self.theta_right:{self.theta_right}, self.count_to_straight:{self.count_to_straight}----")

        
        elif self.now == 8:
            V = 0.35
            dl = 0.05
            Th = 0.5
            if right_OK:
                v = 0.4
            else:
                return
            theta = V*(dl-dr) - Th*theta_now
        
        elif self.now == 9:
            V = 0.75
            dl = 0.05
            Th = 1.25
            if front_OK and right_OK:
                if df > 1.5 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.5
                elif df > 1.0 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.4
                elif df > 0.85 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1 and self.flont_far > 20:
                        self.now += 1
                        self.count_to_curve = 0
                        self.flont_far = 0
                    return
                if df > 0.85 + R:
                    df -= self.check_curve_finish*0.005
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                self.flont_far += 1
                v = 0.3
            else:
                return
            theta = V*(dl-dr) - Th*theta_now
        
        elif self.now == 10:
            if math.pi/3 < self.theta_right < math.pi/2:
                if self.count_to_straight > 5:
                    self.count_to_straight = 0
                    self.now += 1
                    return
            else:
                self.count_to_straight += 1
            v = 0.175
            theta = -0.5
        
        elif self.now == 11:
            V = 0.1
            dl = 0.15
            Th = 1
            if front_OK and right_OK:
                if df > 1.15 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.3
                elif df > 0.65 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.275
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1 and self.flont_far > 20:
                        self.now += 1
                        self.count_to_curve = 0.0
                        self.flont_far = 0
                    return
                if df > 0.65 + R:
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                self.flont_far += 1
                v = 0.3
            else:
                return
            theta = V*(dl-dr) - Th*theta_now  + math.pi/24
            print(f"----self.count_to_curve:{self.count_to_curve}, self.flont_far:{self.flont_far}, dl-dr:{dl-dr}, theta_now:{theta_now}----")
        
        elif self.now == 12:
            if -math.pi/2 < self.theta_right < -math.pi/3:
                if self.count_to_straight > 5:
                    self.count_to_straight = 0
                    v = 0.0
                    theta = 0.0
                    msg = Bool()
                    msg.data = True
                    self.D_stop_publisher.publish(msg)
                    return
            else:
                self.count_to_straight += 1
            v = 0.175
            theta = 0.5
        
        elif self.now == 13:
            print(f"mode_13")
            if -math.pi/2 < self.theta_right < -math.pi/3:
                if self.count_to_straight > 5:
                    self.count_to_straight = 0
                    self.now += 1
                    return
            else:
                self.count_to_straight += 1
            v = 0.175
            theta = 0.5
            print(f"----self.theta_right:{self.theta_right}, self.count_to_straight:{self.count_to_straight}----")
        
        elif self.now == 14:
            if self.count_to_curve < 41:
                v = 0.3
                theta = -0.4 + 0.02 * self.count_to_curve
                self.count_to_curve += 1.0
            else:
                self.count_to_curve = 0
                self.now += 1
        
        elif self.now == 15:
            A = 1
            dl = 0
            B = 0.5
            if right_OK:
                if dl - dr > 0.1:
                    A = 3
                v = 0.3
            else:
                return
            theta = A*(dl-dr) - B*math.sin(theta_now) + math.pi/12
        
        elif self.now == 16:
            A = 1
            dl = 0
            B = 0.5
            if right_OK:
                if dl - dr > 0.1:
                    A = 3
                v = 0.4
            else:
                return
            theta = A*(dl-dr) - B*math.sin(theta_now) + math.pi/12
        
        elif self.now == 17:
            A = 1
            dl = 0.2
            B = 1
            if front_OK and right_OK:
                if df > 0.8 + R:
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.4
                elif df > 0.52 + R - 0.25*math.sin(theta_now):
                    self.count_to_curve = 0
                    self.flont_far += 1
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1 and self.flont_far > 20:
                        self.now += 1
                        self.count_to_curve = 0
                        self.flont_far = 0
                    return
                if df > 0.52 + R:
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                self.flont_far += 1
                v = 0.3
            else:
                return
            theta = A*(dl-dr) - B*math.sin(theta_now)  + math.pi/12
            print(f"----self.count_to_curve:{self.count_to_curve}, self.flont_far:{self.flont_far}, dl-dr:{dl-dr}, theta_now:{theta_now}----")
        
        elif self.now == 18:
            if -math.pi/2 < self.theta_right < -math.pi/3:
                if self.count_to_straight > 5:
                    self.count_to_straight = 0
                    self.now += 1
                    return
            else:
                self.count_to_straight += 1
            v = 0.2
            theta = 0.8
            print(f"----self.theta_right:{self.theta_right}, self.count_to_straight:{self.count_to_straight}----")
        
        elif self.now == 19:
            A = 1
            dl = 0.05
            B = 1
            if right_OK:
                if dl - dr > 0.1:
                    A = 3
                v = 0.4
            else:
                return
            theta = A*(dl-dr) - B*math.sin(theta_now) - math.pi/12
        
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = theta
        self.publisher.publish(twist)
        self.get_logger().info(f"v : {v}, theta : {theta}, possition : {self.now}")
    
        if self.now in self.camera_on_modes:
            self.camera_move = True
        else:
            self.camera_move = False
        msg = Bool()
        msg.data = self.camera_move
        self.camera_move_publisher.publish(msg)
            
def main():
    rclpy.init()
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()