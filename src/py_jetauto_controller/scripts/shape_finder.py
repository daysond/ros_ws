#!/usr/bin/env python3
# encoding: utf-8
import cv2
from math import sqrt
import time
import numpy as np
from functools import reduce
from enum import Enum

from wheel_controller import JetAutoController

#  CONSTANTS
TARGET_TRIANGLE = (281, 422)
TARGET_SQUARE = (312, 355)
TARGET_CURRENT = None

SQUARE = 1
TRIANGLE = 2


# HELPER FUNCTIONS
def preprocess_image(image, use_lines_for_square=False):
    """Preprocess the image and optionally enhance edges for square detection."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    edges = cv2.Canny(binary, 50, 150, apertureSize=3)

    if use_lines_for_square:
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=10)
    else: 
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=30, maxLineGap=10)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(binary, (x1, y1), (x2, y2), 255, 2)
    kernel = np.ones((10, 10), np.uint8)
    return cv2.erode(binary, kernel, iterations=1)

def get_shape_center(image, detected_shape):
    binary = preprocess_image(image, use_lines_for_square=True) if detected_shape == SQUARE else preprocess_image(image)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result_image = image.copy()
    _centers = []
    if detected_shape == SQUARE:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500 or area > 4000:
                continue

            # Approximate the contour to a polygon
            epsilon = 0.05 * cv2.arcLength(contour, True)  # 5% of contour perimeter
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the polygon has 4 sides (quadrilateral)
            if len(approx) == 4:
                # Calculate the bounding rectangle
                x, y, w, h = cv2.boundingRect(approx)
                print(w, h)

                # Filter by aspect ratio (approximately square) and size
                aspect_ratio = float(w) / h
                if 0.9 <= aspect_ratio <= 1.1 and w > 20 and h > 20:  # Adjust size thresholds as needed
                    # Draw the square on the result image
                    cv2.drawContours(result_image, [approx], -1, (0, 255, 0), 2)

                    # Calculate the center of the square
                    cx, cy = x + w // 2, y + h // 2
                    _centers.append((cx, cy))

                    # Mark the center of the square
                    cv2.circle(result_image, (cx, cy), 5, (255, 0, 0), -1)

                    # Coordinates for the ground truth point - SQUARE
                    gt_x, gt_y = TARGET_SQUARE[0], TARGET_SQUARE[1] 
                    cv2.circle(result_image, (gt_x, gt_y), 5, (0, 0, 255), -1)

    if detected_shape == TRIANGLE:
        for contour in contours:
            area = cv2.contourArea(contour)
            print(area)
            if area < 300 or area > 4000:
                continue

            # Approximate the contour to a polygon
            epsilon = 0.03 * cv2.arcLength(contour, True)  # Slightly higher epsilon for noisy contours
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the polygon has 3 sides (triangle)
            if len(approx) == 3:
                # Calculate edge lengths
                a = sqrt((approx[0][0][0] - approx[1][0][0])**2 + (approx[0][0][1] - approx[1][0][1])**2)
                b = sqrt((approx[1][0][0] - approx[2][0][0])**2 + (approx[1][0][1] - approx[2][0][1])**2)
                c = sqrt((approx[2][0][0] - approx[0][0][0])**2 + (approx[2][0][1] - approx[0][0][1])**2)

                # Sort edges to identify the base and height
                edges = sorted([a, b, c])

                # Check triangle properties: isosceles or equilateral
                if abs(edges[0] - edges[1]) < 0.1 * edges[0] and edges[2] < 1.5 * edges[0]:
                    # Calculate the center of the triangle using moments
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        _centers.append((cx, cy))

                        # Draw the triangle on the result image
                        cv2.drawContours(result_image, [approx], -1, (0, 255, 0), 2)
                        cv2.circle(result_image, (cx, cy), 5, (255, 0, 0), -1)

                        # Coordinates for the ground truth point - TRIANGLE
                        gt_x, gt_y = TARGET_TRIANGLE[0], TARGET_TRIANGLE[1] 
                        cv2.circle(result_image, (gt_x, gt_y), 5, (0, 0, 255), -1)
    else:
        print("get_shape_center(): No shape detected")

    return result_image, _centers


# SHAPER FINDER
class ShapeFinder:
    def __init__(self):
        # Initialization
        self.wheel_controller = JetAutoController()
        self.lock_x = False
        self.lock_y = False
        self.fully_locked = False
        self.moving_dir = None
        self.isRunning = True
        self.cap = None

        
    def stop(self):
        if self.cap is not None:
            self.cap.release()
            cv2.destroyAllWindows()
        self.stop_wheel()

    def stop_wheel(self):
        self.wheel_controller.stop()
        self.moving_dir = None

    def align(self, centers, TARGET):

        target_y = TARGET[1]  #380
        target_x = TARGET[0]  #350

        OFFSET = 5
                
        if centers and not self.fully_locked:
            for cx, cy in centers:
                self.lock_x = not ((cx < target_x - 10) or (cx > target_x + 10))
                self.lock_y = not ((cy < target_y - 10) or (cy > target_y + 10))
                self.fully_locked = self.lock_x and self.lock_y
                # * Lock X axis first
                if not self.lock_x:
                    if cx < target_x - OFFSET:
                        print("Left")
                        if self.moving_dir != "left":
                            self.stop_wheel()
                            self.wheel_controller.moveSideWay(1)
                            self.moving_dir = "left"
                
                    elif cx > target_x + OFFSET:
                        print("Right")
                        if self.moving_dir != "right":
                            self.stop_wheel()
                            self.wheel_controller.moveSideWay(-1)
                            self.moving_dir = "right"
        
                    else:
                        if self.moving_dir is not None:
                            self.stop_wheel()
                            self.moving_dir = None
                        print("Center x")
                
                # * Then Lock Y axis
                if self.lock_x and not self.lock_y:
                    if cy > target_y + OFFSET:
                        print("top")
                        if self.moving_dir != "backward":
                            self.stop_wheel()
                            self.wheel_controller.moveForward(-1)
                            self.moving_dir = "backward"
                    elif cy < target_y - OFFSET:
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

    def calibrate(self, display=True):
        self.cap = cv2.VideoCapture("/dev/astrapro")
        while True:
            ret, frame = self.cap.read()
            if ret:
                shape_to_detect = SQUARE
                name = "Square"  if shape_to_detect == SQUARE else "triangle"
                image = None
                if shape_to_detect is not None:
                    image, center = get_shape_center(frame, shape_to_detect)
                    print(f"Calibration complete. Center of {shape_to_detect}: {center}")
                else:
                    print("No shape provided for calibration.")


                if display & image is not None:
                    text = f"Shape: {name}, Center: {center}"
                    cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    cv2.imshow('Calibration', image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        self.cap.release()
        cv2.destroyAllWindows()
                    
    def rotate_cw_90(self):
        self.wheel_controller.rotate(-1)
        time.sleep(8)
        self.wheel_controller.stop()

    def recognize_pickup_location(self, display=True):
        # Target shape: Triangle
        self.fully_locked = False
        try:
            self.cap = cv2.VideoCapture("/dev/astrapro")
            while not self.fully_locked:
                ret, frame = self.cap.read()
                if ret:
                    shape_to_detect = TRIANGLE # pickup location
                    
                    image, center = get_shape_center(frame, shape_to_detect)
                    if center is None:
                        print("Failed to find the center of the pick up shape")
                        continue

                    TARGET_CURRENT = TARGET_TRIANGLE
                    self.align(centers=center, TARGET=TARGET_CURRENT)

                    if image is not None and display:
                        cv2.imshow('Pickup', image)
                        key = cv2.waitKey(1)
                        if key == 27:  # Press 'Esc' to exit
                            break
                    
                    time.sleep(0.02)
            
            self.stop()
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            print(f"Error ooccurred:  {e}")
    
    def recognize_dropOff_location(self, display=True):
        # Target shape: Square
        self.fully_locked = False
        try:
            self.cap = cv2.VideoCapture("/dev/astrapro")
            while not self.fully_locked:
                ret, frame = self.cap.read()
                if ret:
                    shape_to_detect = SQUARE # pickup location
                    
                    image, center = get_shape_center(frame, shape_to_detect)
                    if center is None:
                        print("Failed to find the center of the drop off shape")
                        continue

                    TARGET_CURRENT = TARGET_SQUARE
                    self.align(centers=center, TARGET=TARGET_CURRENT)

                    if image is not None and display:
                        cv2.imshow('Pickup', image)
                        key = cv2.waitKey(1)
                        if key == 27:  # Press 'Esc' to exit
                            break
                    
                    time.sleep(0.02)
            
            self.stop()
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            print(f"Error ooccurred {e}")

