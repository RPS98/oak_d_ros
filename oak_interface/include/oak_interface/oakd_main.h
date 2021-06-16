#ifndef OAKD_MAIN
#define OAKD_MAIN

#include <string>
#include <stdio.h>
#include <pthread.h>
#include <chrono>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

#include <camera_info_manager/camera_info_manager.h>
#include "sensor_msgs/Image.h"
#include "oak_interface/BoundingBoxes.h"
#include "oak_interface/BoundingBox.h"
#include "sensor_msgs/Imu.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <functional>

// OpenCV library
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Include from depthai library
#include "depthai/depthai.hpp"
#include <oak_interface/depthai_bridge/ImageConverter.hpp>

// Tasks
#include "oakd_process.h"
#include "oakd_utils.h"

#endif