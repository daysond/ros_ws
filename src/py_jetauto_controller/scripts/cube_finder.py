#!/usr/bin/env python3
# encoding: utf-8

import cv2
import time
import numpy as np
from functools import reduce
import signal
import sys
from wheel_controller import JetAutoController


TARGET = (340, 375)
THRESHOLD = 7

class CubeFinder:

    def __init__(self, color_to_detect='r'):

        self.wheel_controller = JetAutoController()
        self.cap = None
        self.color_to_detect = color_to_detect
        self.lock_x = False
        self.lock_y = False
        self.fully_locked = False
        self.color_ranges = {
            "red": [
                (np.array([0, 150, 50]), np.array([10, 255, 255])),
                (np.array([170, 150, 50]), np.array([180, 255, 255]))
            ],
            "green": [
                (np.array([30, 100, 50]), np.array([90, 255, 255]))  # Broader range for green
            ],
            "blue": [
                (np.array([90, 100, 50]), np.array([150, 255, 255]))  # Broader range for blue
            ]
        }

        self.moving_dir = None
    
    def unlock(self):
        self.lock_x = False
        self.lock_y = False
        self.fully_locked = False

    
    def set_detection_color(self, color):
        self.color_to_detect = color
    
    def stop(self):
        if self.cap is not None:
            self.cap.release()
            cv2.destroyAllWindows()

        self.wheel_controller.stop()
        

    def stop_wheel(self):
        self.wheel_controller.stop()
        self.moving_dir = None

    def rotate_cw_90(self):
        self.wheel_controller.rotate(-1)
        time.sleep(8)
        self.wheel_controller.stop()

    def start(self):

        target_y = TARGET[1]  #380
        target_x = TARGET[0]  #350

        if self.color_to_detect == 'b':
            self.wheel_controller.start()
            self.wheel_controller.moveSideWay(-1)
            time.sleep(2.5)
            self.wheel_controller.stop()

        self.cap = cv2.VideoCapture("/dev/usb_cam")
        
        while not self.fully_locked:
            ret, frame = self.cap.read()
            if ret:

                # frame = cv2.resize(frame, (640, 480))
                image, centers = self.run(frame, self.color_to_detect)
                cv2.imshow('image', image)
                
                if centers and not self.fully_locked:
                    # print(f"centers {centers}")
                    # print(f" in pox x {is_in_position_x} in pos y {is_in_position_y}")

                    for cx, cy in centers:
                        self.lock_x = not ((cx < target_x - THRESHOLD) or (cx > target_x + THRESHOLD))
                        self.lock_y = not ((cy < target_y - THRESHOLD) or (cy > target_y + THRESHOLD))
                        self.fully_locked = self.lock_x and self.lock_y
                        # * Lock X axis first
                        if not self.lock_x:
                            if cx < target_x - THRESHOLD:
                                print("Left")
                                if self.moving_dir != "left":
                                    self.stop_wheel()
                                    self.wheel_controller.moveSideWay(1)
                                    self.moving_dir = "left"
                        
                            elif cx > target_x + THRESHOLD:
                                print("Right")
                                if self.moving_dir != "right":
                                    self.stop_wheel()
                                    self.wheel_controller.moveSideWay(-1)
                                    self.moving_dir = "right"
                
                            else:
                                if self.moving_dir is not None:
                                    # self.stop_wheel()
                                    self.moving_dir = None
                                print("Center x")
                        
                        # * Then Lock Y axis
                        if self.lock_x and not self.lock_y:
                            if cy > target_y + THRESHOLD:
                                print("top")
                                if self.moving_dir != "backward":
                                    self.stop_wheel()
                                    self.wheel_controller.moveForward(-1)
                                    self.moving_dir = "backward"
                            elif cy < target_y - THRESHOLD:
                                print("down")
                                if self.moving_dir != "forward":
                                    self.stop_wheel()
                                    self.wheel_controller.moveForward(1)
                                    self.moving_dir = "forward"
                            else:
                                print("Center y")
                                if self.moving_dir is not None:
                                    self.stop_wheel()
                                    self.moving_dir = None
                else:
                    print("No center")
                    self.stop_wheel()
       
                key = cv2.waitKey(1)
                if key == 27:  # Press 'Esc' to exit
                    break

                time.sleep(0.02)


    def run(self, image, color_to_detect):
        """
        Detects a specific colored cube in an image, draws bounding boxes and center points,
        and returns the modified image and center point locations.

        Parameters:
            image (numpy.ndarray): Input image
            color_to_detect (str): The color to detect ('r', 'g', or 'b')

        Returns:
            result_image (numpy.ndarray): Image with bounding boxes and center points
            center_points (list): List of tuples containing center point coordinates (x, y)
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Map the input to the corresponding color name
        color_map_input = {'r': 'red', 'g': 'green', 'b': 'blue'}
        if color_to_detect not in color_map_input:
            raise ValueError("Invalid color_to_detect. Choose 'r', 'g', or 'b'.")

        color_name = color_map_input[color_to_detect]

        # Define HSV ranges for each color

        # Create a mask for the selected color
        masks = [cv2.inRange(hsv, lower, upper) for lower, upper in self.color_ranges[color_name]]
        mask = reduce(cv2.bitwise_or, masks)

        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Prepare output
        center_points = []
        result_image = image.copy()

        color_bgr = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}

        for contour in contours:
            if cv2.contourArea(contour) > 2000 and cv2.contourArea(contour) < 5000:  
                x, y, w, h = cv2.boundingRect(contour)
                # print(f"{x} {y} {w} {h}")
                # Draw the rectangle
                cv2.rectangle(result_image, (x, y), (x + w, y + h), color_bgr[color_name], 2)
                # Calculate and draw the center point
                cx, cy = x + w // 2, y + h // 2
                cv2.circle(result_image, (cx, cy), 5, color_bgr[color_name], -1)
                cv2.circle(result_image, (x, y), 5, color_bgr['red'], -1)
                # Add the center point to the list
                center_points.append((cx, cy))
                # print(f" {cx} {cy}")
                # Display the center coordinates
                cv2.putText(result_image, f"{color_name} ({cx}, {cy})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr[color_name], 2)

        cv2.circle(result_image, TARGET, 5, color_bgr["green"], -1)

        return result_image, center_points


