#!/usr/bin/env python3

import rospy
import signal
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos
from kinematics.search_kinematics_solutions import SearchKinematicsSolutions

class MoveServoNode:
    def __init__(self, name):
        # rospy.init_node(name, anonymous=True)
        self.z_dis = 0.17
        self.y_init = 0.39
        signal.signal(signal.SIGINT, self.shutdown)

        self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(0.2)        
        self.search_kinemactis_solutions = SearchKinematicsSolutions()
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/hiwonder_servo_manager/running') and rospy.get_param('/joint_states_publisher/running'):
                    break
            except:
                rospy.sleep(0.1)
        # self.move_servo()

    def shutdown(self, signum, frame):
        rospy.loginfo('shutdown')

    def move_servo(self):
                                                            # outward  z: height
        res = self.search_kinemactis_solutions.solveIK((0, self.y_init, self.z_dis), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)

            print(res)
                                                # gripper  
            set_servos(self.joints_pub, 1500, ((1, 300), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
            rospy.sleep(1.8)

    
    def initPose(self):
        print("init pose")          
                                                           # y   z
        res = self.search_kinemactis_solutions.solveIK((0, 0.25, 0.35), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 200), (2, 198), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
            rospy.sleep(1)

    def _pickup(self):
        print("picking up")
        res = self.search_kinemactis_solutions.solveIK((0, 0.395, 0.16), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 200), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
            rospy.sleep(1)

    def _closeGripper(self):
        print("gripping..")
        res = self.search_kinemactis_solutions.solveIK((0, 0.395, 0.16), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 550), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
            rospy.sleep(1)
    
    def _lift(self):
        print("lifting ... ")
        res = self.search_kinemactis_solutions.solveIK((0, 0.25, 0.35), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 550), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
            rospy.sleep(1)
    
    def _rotate(self):
        print("rotating ... ")
        res = self.search_kinemactis_solutions.solveIK((0, 0.25, 0.35), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 550), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, 300)))
            rospy.sleep(1)
    
    def _dropOff(self):
        print("dropping off ... ")
        res = self.search_kinemactis_solutions.solveIK((0, 0.395, 0.17), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 550), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
            rospy.sleep(1)

    def _dropOffSide(self):
        print("dropping off on the side... ")
        res = self.search_kinemactis_solutions.solveIK((0, 0.395, 0.16), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 550), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, 300)))
            rospy.sleep(1)

    def _openGripSide(self):
        print("Opening grip ... ")
        res = self.search_kinemactis_solutions.solveIK((0, 0.395, 0.16), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 200), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, 300)))
            rospy.sleep(1)

    def _openGrip(self):
        print("Opening grip ... ")
        res = self.search_kinemactis_solutions.solveIK((0, 0.395, 0.16), 0, -90, 90)
        if res:
            joint_data = res[1]
            rospy.sleep(0.5)
                                                # gripper # joint 4
            set_servos(self.joints_pub, 1500, ((1, 200), (2, 245), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
            rospy.sleep(1)
    
    def flow(self):
        # pick uo abd drop off on the side
        self.initPose()
        self._pickup()
        self._closeGripper()
        self._lift()
        self._rotate()
        self._dropOff()
        self._openGrip()
        self.initPose()
    
    def pickup(self):
        self._pickup()
        self._closeGripper()
        self._lift()


    def dropOff(self):
        self._dropOff()
        self._openGrip()
        self.initPose()

# if __name__ == '__main__':
#     msm = MoveServoNode('move_servo')
#     msm.initPose()
#     msm.pickup()
#     msm.closeGripper()
#     msm.lift()
#     msm.rotate()
#     msm.dropOff()
#     msm.openGrip()
#     msm.initPose()


"""
init pose

z_dis = 0.35
y_init = 0.25
joint 4 (2, 198)
gripper 200


To Pick up pose:

self.z_dis = 0.17
self.y_init = 0.39
joint 4 (2, 245)
gripper 500


hold 
self.z_dis = 0.35
self.y_init = 0.25
joint 4 (2, 245)
gripper 500



"""