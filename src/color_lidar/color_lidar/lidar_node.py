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
        self.mode_n = 0
        self.straight1 = [6, 8, 9, 11]
        self.straight2 = [1, 3, 15, 16, 17]
        self.curve = [2, 7, 12, 13, 18]

    def right_side_RED_callback(self, msg):
        if mag.data:
            self.now = 9

    def right_side_C_callback(self, msg):
        if mag.data:
            self.now = 13
    
    def right_side_B_callback(self, msg):
        if mag.data:
            self.now = 16
    
    def right_side_A_callback(self, msg):
        if mag.data:
            self.now = 17
    
    def right_side_F_callback(self, msg):
        if mag.data:
            self.now = 1

    def right_side_D_callback(self, msg):
        if mag.data:
            self.now = 6
    
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
        if not self.is_moving_forward:
            return

        r = np.array(msg.ranges)
        num_points = len(r)
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        for i in range(num_points):
            theta = msg.angle_min + msg.angle_increment * i
            x[i] = -r[i] * np.cos(theta)
            y[i] = -r[i] * np.sin(theta)
        
        valid_indices = np.isfinite(y)
        x = x[valid_indices]
        y = y[valid_indices]
        
        valid_indices = np.abs(y) < 1e10
        x = x[valid_indices]
        y = y[valid_indices]

        valid_indices = ~((x > -0.27) & (x < 0.14) & (y > -0.30) & (y < 0.17))
        x = x[valid_indices]
        y = y[valid_indices]
        
        x, y = self.sampling(x, y)
        self.ransac(x, y)

    def sampling(self, x, y, sample_size=300):
        indices = np.random.choice(len(x), size=sample_size, replace=False)
        x_sample = x[indices]
        y_sample = y[indices]
        return x_sample, y_sample
    
    def ransac(self, x, y, num_inliers_thresh=20):
        ransac = linear_model.RANSACRegressor(min_samples=2, residual_threshold=0.05, max_trials=200000)
        while True:
            if len(x) < 50:
                break
            ransac.fit(x.reshape(-1, 1), y)
            a = ransac.estimator_.coef_
            b = ransac.estimator_.intercept_
            self.theta = math.atan(a)
            inlier_mask = ransac.inlier_mask_
            num_inliers = np.sum(inlier_mask)
            if abs(math.degrees(self.theta)) < 45:
                if b < 0:
                    n = "front"
                else:
                    n = "rear"
                if n in self.inliers:
                    if self.inliers[n] > num_inliers or abs(b)/(a**2+1)**0.5 < self.lidar_position[1]-0.05:
                        pass
                    else:
                        self.inliers[n] = num_inliers
                        self.dist[n] = abs(b)/(a**2+1)**0.5
                else:
                    if abs(b)/(a**2+1)**0.5 < self.lidar_position[1]-0.05:
                        pass
                    else:
                        self.inliers[n] = num_inliers
                        self.dist[n] = abs(b)/(a**2+1)**0.5
            else:
                if -b/a < 0:
                    n = "right"
                    point = (-0.27, -0.30)
                else:
                    n = "left"
                    point = (0.14, -0.30)
                if n in self.inliers:
                    if self.inliers[n] > num_inliers:
                        pass
                    else:
                        self.theta_right = self.theta
                        self.inliers[n] = num_inliers
                        self.dist[n] = abs(a*point[0] - point[1] + b)/(a**2+1)**0.5
                else:
                    self.theta_right = self.theta
                    self.inliers[n] = num_inliers
                    self.dist[n] = abs(a*point[0] - point[1] + b)/(a**2+1)**0.5
            pub = self.create_publisher(PointCloud2, f'/lidar/wall_{n}', 10)
            
            if num_inliers < num_inliers_thresh:
                break
            x_inlier = x[inlier_mask]
            y_inlier = y[inlier_mask]
            x = np.delete(x, inlier_mask)
            y = np.delete(y, inlier_mask)

        self.publish_v(self.dist)
        self.dist = {}
        self.inliers = {}

    def publish_v(self, dist):
        v = 0
        theta = 0
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
                    v = 0.4
                elif df > 0.65 + R:
                    self.count_to_curve = 0
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1:
                        self.now += 1
                        self.count_to_curve = 0
                    return
                if df > 0.5 + R
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                v = 0.3
            else:
                return
            theta = A*(dl-dr) - B*math.sin(theta_now) + math.pi/12
        
        if self.now == 3:
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
        
        if self.now == 6:
            V = 0.35
            df = 0.05
            Th = 3
            if front_OK and right_OK:
                if df > 0.9 + R:
                    self.count_to_curve = 0
                    v = 0.4
                elif df > 0.65 + R:
                    self.count_to_curve = 0
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1:
                        self.now += 1
                        self.count_to_curve = 0
                    return
                if df > 0.65 + R
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                v = 0.4
            else:
                return
            theta = V*(dl-dr) - Th*theta_now
        
        if self.now == 8:
            V = 0.35
            df = 0.05
            Th = 3
            if right_OK:
                if dl - dr > 0.1:
                    A = 3
                v = 0.4
            else:
                return
            theta = V*(dl-dr) - Th*theta_now
        
        if self.now == 9:
            V = 0.75
            df = 0.05
            Th = 1.25
            if front_OK and right_OK:
                if df > 1.5 + R:
                    self.count_to_curve = 0
                    v = 0.5
                elif df > 1.2 + R:
                    self.count_to_curve = 0
                    v = 0.4
                elif df > 0.97 + R:
                    self.count_to_curve = 0
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1:
                        self.now += 1
                        self.count_to_curve = 0
                    return
                if df > 0.97 + R:
                    df -= self.check_curve_finish*0.005
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                v = 0.3
            else:
                return
            theta = V*(dl-dr) - Th*theta_now
        
        if self.now == 11:
            V = 0.35
            df = 0.05
            Th = 3
            if front_OK and right_OK:
                if df > 1.15 + R:
                    self.count_to_curve = 0
                    v = 0.4
                elif df > 0.8 + R:
                    self.count_to_curve = 0
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1:
                        self.now += 1
                        self.count_to_curve = 0
                    return
                if df > 0.8 + R:
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                v = 0.3
            else:
                return
            theta = V*(dl-dr) - Th*theta_now
        
        if self.now == 15 or self.now == 16:
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
        
        if self.now == 17:
            A = 1
            dl = 0.3
            B = 0.5
            if front_OK and right_OK:
                if df > 0.8 + R:
                    self.count_to_curve = 0
                    v = 0.4
                elif df > 0.52 + R - 0.25*math.sin(theta_now):
                    self.count_to_curve = 0
                    v = 0.3
                else:
                    self.count_to_curve += 1
                    if self.count_to_curve > 1:
                        self.now += 1
                        self.count_to_curve = 0
                    return
                if df > 0.52 + R
                    self.check_curve_finish += 1
            elif right_OK:
                self.count_to_curve = 0
                v = 0.3
            else:
                return
            theta = A*(dl-dr) - B*math.sin(theta_now) + math.pi/12
            
def main():
    rclpy.init()
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()