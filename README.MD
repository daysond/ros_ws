# JetAuto Pro Robot Simple Inventory System

The robot will navigate to a pick up location to pick up either red, green or blue cube and drop it off at the designated drop off location.

## Demo Video

<video width="400" controls>
  <source src="./sea700_demo.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Set up the work space

After connecting to the robot via Wi-Fi, under the parent directory of the ```ros_ws```, run command ```rsync -av ./ros_ws/ jetauto@192.168.149.1:~/ros_ws/``` to copy the workplace onto the robot. 

## Run the code

After SSH into the robot do the following: 

1. Stop start app service *(New Terminal)*

```sudo systemctl stop start_app_node.service```

2. Start basic wheel control *(Same Terminal as above)*

```roslaunch jetauto_controller jetauto_controller.launch```

3. Start servo control *(New Terminal)*

```roslaunch hiwonder_servo_controllers start.launch```

***** **The following steps require using [NO MACHINE](https://www.nomachine.com) or the actual robot for visualization** *****

4. Start navigation service *(New Terminal)*

```roslaunch jetauto_navigation navigation.launch sim:=true map:=<your_map>```

5. Start Rviz navigation *(New Terminal)*

```roslaunch jetauto_navigation rviz_navigation.launch```

6. Source ros_ws then Run program *(New Terminal)*

```source ~/ros_ws/devel/setup.bash``` 

```rosrun py_jetauto_controller program.py``` 

## Known Issues

After running Rviz Navigation, the camera devices will be occupied and not available for shape and cube detection. Due to limited access to robot, this issue is not fixed yet.

However, the following might be helpful:

```py
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Display the image (or process it)
        cv2.imshow("Camera Frame", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

def main():
    rospy.init_node('camera_reader', anonymous=True)
    # Replace '/camera/image_raw' with your topic
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

```

This is the sample code to subscribe to the ```camera_node``` (Depth Camera) and the ```usb_cam_node```(Arm Camera), read the raw data from the camera and then process it into frames. To see the exact topic names, run command ```rostopic list``` and it should output something similar to ```/camera/image_raw``` ```/usb_cam/image_raw```.

In summary, instead of reading the feed directly from the cameras, we should:

1. Subscribe to the camera nodes
2. Get the raw data
3. Process the raw data and turn it into image frame
4. Use the frame to perform CV tasks


## Code Reference

1.  ```servo.py``` is modification of [SEA700 Lab](https://seneca-bsa.github.io/bsa/sea700/lab5/) material under Lab 5, section **JetAuto Robot Arm Control (Without MoveIt)**.
2.  ```cube_finder.py``` is inspired from the example code in ```jetauto_example/color_detect```.
3. ```navigator.py``` is based on the article [Sending Simple Goals](https://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) on Ros.org.

## Note

The SLAM algorithm used in this project was **gmapping**. It performs well but sometimes the robot has problem localizing itself given the 2D feature is not strong enough at some location. Another method that we wanted to try was [ORB SLAM](https://github.com/shanpenghui/ORB_SLAM3_Fixed). Initially, we used [another repository](https://github.com/UZ-SLAMLab/ORB_SLAM3?tab=readme-ov-file) but encountered error building the code. This [ORB SLAM](https://github.com/shanpenghui/ORB_SLAM3_Fixed) is recommended from HiWonder, in case anyone is continuing in improving the project, they could try the ORBSLAM3 mapping. 