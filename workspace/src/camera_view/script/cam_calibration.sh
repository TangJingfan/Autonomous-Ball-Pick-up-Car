#!/bin/bash
rosrun camera_calibration cameracalibrator.py --size 10x6 --square 0.025 image:=/usb_cam/image_raw
