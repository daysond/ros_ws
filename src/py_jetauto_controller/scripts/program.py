#!/usr/bin/env python3
# encoding: utf-8
import signal
import sys
from cube_finder import CubeFinder
from servo import MoveServoNode
from shape_finder import ShapeFinder
from navigation import Navigator
import argparse


# CLASSES
cubeFinder = CubeFinder()
move = MoveServoNode('move_servo')
shapeFinder = ShapeFinder()



# EXIT HANDLING
def clean_up(*args):
    print("Exiting...")
    cubeFinder.stop()
    shapeFinder.stop()
    sys.exit(0)  # Exit the script safely

signal.signal(signal.SIGINT, clean_up)  # Ctrl+C
signal.signal(signal.SIGTSTP, clean_up)  # Ctrl+Z


# CLI ARGUMENTS
parser = argparse.ArgumentParser(description="Run the robot program with color argument.")
parser.add_argument('color', choices=['r', 'g', 'b'], help="Specify the color: 'r' (red), 'g' (green), or 'b' (blue).")
parser.add_argument("--calibrate", action="store_true", help="Perform calibration before starting")

args = parser.parse_args()


if __name__ == '__main__':
    try:
        if args.calibrate:
            print("Running Shapes calibration...")
            ## Calibrate the position of the camera and get the center point of the shape
            shapeFinder.calibrate() 
        else:
            # Set detection color
            print(f"Selected color: {args.color}")
            cubeFinder.set_detection_color(args.color)

            # * Localize itself
                # * Search for pick up 
            nav = Navigator()
            nav.navigate_to_checkpoints_for_pickup()
            nav.navigate_to_pickup()
            # # * Get to pick up position
                # * Using: astropro camera, usb camera is off
            # ! -------------------------------------------------
            shapeFinder.recognize_pickup_location()
            shapeFinder.stop()

            # # # # * Lock the cube
            cubeFinder.start()
            cubeFinder.stop()

            # # # # * pick up and hold
            move.pickup()    

            # ! --------------------------------------------------

            nav = Navigator()
            nav.navigate_to_checkpoints_for_drop_off()
            nav.navigate_to_dropoff()


            # ! ---------------------------
            cubeFinder.rotate_cw_90()
            # # # * Recognize Drop Off location
            shapeFinder.recognize_dropOff_location()
            # # # * Drop off
            move.dropOff()

            # navigator.logComplete()

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Cleaning up...")
        clean_up()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        clean_up()

