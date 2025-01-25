#!/bin/bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.1 image:=/usb_cam/image_raw camera:=/head_camera --no-service-check
