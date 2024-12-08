#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
import time
import math

class JetAutoController():

    PI = 3.14159265359
    SPEED = 0.25
    
    pose_x = 0
    pose_y = 0
    yaw = 0
    

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.yaw = yaw
        # print("Yaw: %f" % (yaw))
        # print("position: (%f %f %f)" % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))


    def __init__(self):
        rospy.init_node('jetauto_teleop', anonymous=True)
        self.publisher_ = rospy.Publisher('/jetauto_controller/cmd_vel', Twist,  queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.odom_pub = rospy.Publisher('/set_odom', Odometry, queue_size=10)    

    # def reset_odom(self):
    #     reset = Odometry()
    #     reset.pose.pose.position.x = 0.0
    #     reset.pose.pose.position.y = 0.0
    #     reset.pose.pose.position.z = 0.0
    #     reset.pose.pose.orientation.x = 0.0
    #     reset.pose.pose.orientation.y = 0.0
    #     reset.pose.pose.orientation.z = 0.0
    #     reset.pose.pose.orientation.w = 1.0
    #     reset.twist.twist.linear.x = 0.0
    #     reset.twist.twist.linear.y = 0.0
    #     reset.twist.twist.linear.z = 0.0
    #     reset.twist.twist.angular.x = 0.0
    #     reset.twist.twist.angular.y = 0.0
    #     reset.twist.twist.angular.z = 0.0
    #     print("Reseting odom...")
    #     self.odom_pub.publish(reset)

    def reset_odom(self):
        reset = Pose2D()
        reset.x = 0.0
        reset.y = 0.0
        reset.theta = 0.0
        print("Reseting odom...")
        self.odom_pub.publish(reset)

    # def reset_odom(self):
    #     rospy.wait_for_service('/set_pose')
    #     set_post_ser = rospy.ServiceProxy('/set_pose', PoseStamped)

    #     reset = PoseStamped()
    #     reset.header.frame_id = "odom"
    #     reset.header.stamp = rospy.Time.now()

    #     reset.pose.position.x = 0.0
    #     reset.pose.position.y = 0.0
    #     reset.pose.position.z = 0.0

    #     reset.pose.orientation.x = 0.0
    #     reset.pose.orientation.y = 0.0
    #     reset.pose.orientation.z = 0.0
    #     reset.pose.orientation.w = 1.0
    #     print("Reseting...")
    #     set_post_ser(reset)
       
        
    def getTwist(self, linear_x=0, angular_z=0, linear_y=0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z

        return msg

    def start(self):
        self.reset_odom()
        msg = self.getTwist()
        self.run(msg, 1)
    

    def run(self, msg, duration = 0):
        self.publisher_.publish(msg)
        print("Publishing: start.")
        time.sleep(duration)
    
    def stop(self):
        msg = self.getTwist(0.0, 0.0, 0.0)
        self.publisher_.publish(msg)
        print("Publishing: Stop.")
    
    def run_lab(self):
        original_x = self.pose_x
        diff = abs(self.pose_x - original_x)
        msg = self.getTwist(0.3, linear_y=0)
        self.publisher_.publish(msg)
        print('running forward')
        while diff < 1.0:
            diff = abs(self.pose_x - original_x)
            # continue
        # self.stop()

        original_y = self.pose_y
        diff = abs(self.pose_y - original_y)
        msg = self.getTwist(0, linear_y=0.3)
        self.publisher_.publish(msg)
        while diff < 1.0:
            diff = abs(self.pose_y - original_y)
            print("f diff/: (%f)" % (diff))
            # continue
        # self.stop()

        original_yaw = self.yaw
        diff = abs(self.yaw - original_yaw)
        msg = self.getTwist(angular_z=-0.3)
        self.publisher_.publish(msg)
        while diff < 1.5708:
            diff = abs(self.yaw - original_yaw)
            # continue
        # self.stop()

        original_x = self.pose_x
        diff = abs(self.pose_x - original_x)
        msg = self.getTwist(0, linear_y=-0.3)
        self.publisher_.publish(msg)
        while diff < 1.0:
            diff = abs(self.pose_x - original_x)
        # while self.pose_x > 0:
        #     continue
        # self.stop()

        # self.drift()
    
    def drift(self):
        current_yaw = self.yaw
        current_y = self.pose_y
        end = False
        while not end:
            yaw_diff = self.yaw - current_yaw
            y_diff = self.pose_y - current_y
            v = 0.18
            v_x = v * math.cos(yaw_diff)
            v_y = v * math.sin(yaw_diff)
            msg = self.getTwist(linear_x=v_x, linear_y=-v_y, angular_z=0.26)
            self.publisher_.publish(msg)
            end = abs(yaw_diff) > 1.5708
        
        self.stop()

    # def drift(self):
    #     one_time_rad = math.pi / 200.0
    #     vx = 0.25
    #     speed_in_rad = math.pi/10.0 # 90degree in 5 second
    #     sleep_dur = 0.05 # 5 second into 100 

    #     for i in range(0, 101):

    #         theta = i * one_time_rad
    #         x = vx / (-pow(math.sin(theta),2)/math.cos(theta) + math.cos(theta))
    #         x = max(x, 0.7)
    #         y = -math.sin(theta)*x / math.cos(theta)
    #         y = max(y, 0.7)
    #         msg = self.getTwist(x, speed_in_rad, y)
    #         self.publisher_.publish(msg)
    #         time.sleep(sleep_dur)


def main(args=None):

    controller = JetAutoController()
    controller.start()
    # controller.run_square()
    # controller.run_lab()
    # controller.drift()
    controller.stop()
    controller.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

