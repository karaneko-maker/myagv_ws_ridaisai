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
        # self.br = self.create_transform_broadcaster()
        self.dist = {}
        self.inliers = {}
        self.tag = False
        self.lidar_position = [0, 0.25]
        self.v_coefficient = 0.75   # 0.75
        self.theta_coefficient = 1.5    #1
        self.straight_OK = True
        self.ns = 30
        self.nc = 0
        self.c_thresh = 0
        self.dl = 0.2
        
        self.right_side_D_subscription = self.create_subscription(Bool, '/right_side_D_msg',  self.right_side_D_callback, 10)
        self.right_side_C_subscription = self.create_subscription(Bool, '/right_side_C_msg',  self.right_side_C_callback, 10)
        self.right_side_subscription = self.create_subscription(Bool, '/right_side_msg',  self.right_side_callback, 10)

        self.move_forward_subscription = self.create_subscription(Bool,'/move_msg',  self.move_forward_callback, 10)
        self.is_moving_forward = False
        self.is_right_side_D = False
        self.is_right_side_C = False
        self.is_right_side = False
        self.w_theta = 0

        self.camera_move_publisher = self.create_publisher(Bool, '/camera_move_msg', 10)
        self.camera_move = False
        self.D_camera_stop = False
        self.regular = True
        self.catch = True

        # Send a static transform for lidar
        # transform = TransformStamped()
        # transform.header.stamp = self.get_clock().now().to_msg()
        # transform.header.frame_id = 'map'
        # transform.child_frame_id = 'lidar'
        # transform.transform.translation.x = 0.0
        # transform.transform.translation.y = 0.0
        # transform.transform.translation.z = 0.0
        # transform.transform.rotation = tf_transformations.quaternion_from_euler(0, 0, 0)
        # self.br.sendTransform(transform)
    
    ### Bool 値定義 ###
    def right_side_D_callback(self, msg):
        if self.catch:
            self.is_right_side_D = msg.data
            if self.is_right_side_D:
                self.regular = False
        
        # if self.is_right_side_D:
        #     self.get_logger().info("Right Side D!")
        # else:
        #     self.get_logger().info("Normal")
    
    def right_side_C_callback(self, msg):
        self.is_right_side_C = msg.data
        
        # if self.is_right_side_C:
        #     self.get_logger().info("Right Side C!")
        # else:
        #     self.get_logger().info("Normal")
    
    def right_side_callback(self, msg):
        self.is_right_side = msg.data
        
        # if self.is_right_side:
        #     self.get_logger().info("Right Side!")
        # else:
        #     self.get_logger().info("Normal")
    
    def move_forward_callback(self, msg):
        self.is_moving_forward = msg.data
        
        if self.is_moving_forward:
            self.get_logger().info("Move!")
        else:
            self.get_logger().info("ransac stop!")
        
        # if self.is_moving_forward:
        #     self.get_logger().info("Move Forward Triggered!")
        # else:
        #     self.get_logger().info("Stop Movement")
    ####################


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
    
    def create_point_cloud_msg(self, x_array, y_array):
        z_array = np.zeros_like(x_array)
        points = np.vstack((x_array, y_array, z_array)).T
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        # cloud_msg = pc2.create_cloud_xyz32(header, points)
        # return cloud_msg
    
    def ransac(self, x, y, num_inliers_thresh=20):
        ransac = linear_model.RANSACRegressor(min_samples=2, residual_threshold=0.05, max_trials=200000)
        while True:
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
                    if self.inliers[n] > num_inliers:
                        pass
                    else:
                        self.inliers[n] = num_inliers
                        self.dist[n] = abs(b)/(a**2+1)**0.5
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
            # self.get_logger().info(f"theta_{n} : {self.theta}, dist_{n} : {self.dist[n]}")
            
            if num_inliers < num_inliers_thresh:
                break
            x_inlier = x[inlier_mask]
            y_inlier = y[inlier_mask]
            x = np.delete(x, inlier_mask)
            y = np.delete(y, inlier_mask)
            # point_cloud_msg = self.create_point_cloud_msg(x_inlier, y_inlier)
            # pub.publish(point_cloud_msg)
        # self.get_logger().info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        self.get_logger().info(f"nc : {self.nc}, ns : {self.ns}, theta : {self.theta_right}, dist : {self.dist} D : {self.is_right_side_D} , Catch : {self.catch}, regular : {self.regular}")
        self.publish_v(self.dist)
        self.dist = {}
        self.inliers = {}

    def publish_v(self, dist):
        if self.tag:
            x_thresh = -0.095
        else:
            x_thresh = 0
        if "front" in dist:
            df = dist["front"][0]
            front_OK = False
        else:
            front_OK = True
            df = 100
        
        if self.straight_OK or self.is_right_side_C:
            if self.theta_right < 0:
                theta_now = -math.pi/2 - self.theta_right
            else:
                theta_now = math.pi/2 - self.theta_right
            if "right" in dist:
                dr = dist["right"][0]
            else:
                return
            
            if self.ns < 20:
                self.w_theta = 0
                v = 0.4
                self.theta_coefficient = 1.5
                if "left" in dist:
                    dl = dist["left"][0]
                else:
                    dl = self.dr0
                if front_OK or df > 0.85 + self.lidar_position[1]:
                    self.ns += 1
            else:
                self.w_theta = 0
                self.theta_coefficient = 1.5
                if "left" in dist:
                    dl = dist["left"][0]
                else:
                    return
                
                # print(f"is_right_side = {self.is_right_side_D}")  # 現在の値を出力する

                if self.is_right_side_D:
                    self.D_camera_stop = True
                    # self.get_logger().info("{YELLOW}right_side_D mode")
                    if front_OK or df > 1.5 + self.lidar_position[1]:
                        v = 0.5
                    elif df > 1.2 + self.lidar_position[1]:
                        v = 0.4
                    elif df > 0.97 + self.lidar_position[1]:
                        v = 0.3
                        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        # v = 0.1 + (df - self.lidar_position[1] -0.65)
                    # elif df > 0.9 + self.lidar_position[1]:
                    #     v = 0.3
                    #     self.w_theta = 0.65 - df
                    else:
                        self.c_thresh += 1
                        if self.c_thresh > 2:
                            self.straight_OK = False
                        v = 0.4
                        self.nc = 0
                else:
                    self.D_camera_stop = False
                    if front_OK or df > 0.85 + self.lidar_position[1]:
                        v = 0.4
                        self.w_theta = 0
                    elif df > 0.65 + self.lidar_position[1]:
                        v = 0.2 + (df - self.lidar_position[1] -0.65)
                        self.w_theta = math.pi/9
                    elif df > 0.5 + self.lidar_position[1]:
                        v = 0.2
                        self.w_theta = math.pi/9
                    else:
                        self.c_thresh += 1
                        if self.c_thresh > 2:
                            self.straight_OK = False
                        v = 0.4
                        self.nc = 0
                    if not self.catch:
                        v = 0.1
            
            if self.is_right_side_D or self.is_right_side_C or self.is_right_side:
                # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                if self.dl < 0.05 or self.regular:
                    self.dl = 0.05
                else:
                    self.dl -= 0.005
                self.v_coefficient = 1.5
                theta = (dl - dr) * self.v_coefficient - theta_now * self.theta_coefficient +self.w_theta
            else:
                theta = (dl - dr) * self.v_coefficient - theta_now * self.theta_coefficient
        
        if theta:
            if self.catch:
                thm = 0.8
            else:
                thm = 0.4
            if theta > thm:
                theta = thm
            elif theta < -thm:
                theta = -thm

            # theta = (dl - dr) * self.v_coefficient - theta_now * self.theta_coefficient
        
        # self.get_logger().info(f"{self.theta_right}")
        
        if not self.straight_OK and not self.is_right_side_C:
            if self.is_right_side_D:
                # print("!!!!!!!!!!!!!!!!!!!!!")
                if self.nc < 5 or not math.pi/3 < self.theta_right < math.pi/2:
                    v = 0.175
                    theta = -0.5
                    if math.pi/4 < self.theta_right < math.pi/2:
                        pass
                    else:
                        self.nc += 1
                else:
                    self.regular = True
                    self.D_camera_stop = False
                    self.is_right_side_D = False
                    self.is_right_side = True
                    self.catch = False
                    self.dr0 = dist["right"][0]
                    self.straight_OK = True
                    self.c_thresh = 0
                    self.ns = 0
                    v = 0.4
                    theta =  math.pi / 6
                    self.dl = 0.2
            else:
                if self.nc < 5 or not -math.pi/2 < self.theta_right < -math.pi/3:
                    v = 0.2
                    theta = 0.8

                    if -math.pi/2 < self.theta_right < -math.pi/3:
                        pass
                    else:
                        self.nc += 1
                else:
                    self.catch = True
                    self.dr0 = dist["right"][0]
                    self.straight_OK = True
                    self.c_thresh = 0
                    self.ns = 0
                    v = 0.4
                    theta = - 1 *math.pi / 6
                    self.dl = 0.2

        if self.straight_OK and not self.D_camera_stop:
            self.get_logger().info("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
            if not locals().get('dr'):
                self.camera_move = False
            elif dr > 0.3:
                self.camera_move = False
            else:
                self.camera_move = True
        else:
            if not self.straight_OK:
                self.get_logger().info("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC")
            self.camera_move = False
        msg = Bool()
        msg.data = self.camera_move
        self.camera_move_publisher.publish(msg)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = theta
        self.publisher.publish(twist)
        self.get_logger().info(f"v : {v}, theta : {theta}")

def main():
    rclpy.init()
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()