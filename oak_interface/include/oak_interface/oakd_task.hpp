#ifndef OAKD_TASK
#define OAKD_TASK

#include "oakd_main.h"

// List of publishers
struct OakPublishList
{
    bool publish_mono = false;
    bool publish_depth = false;
    bool publish_rectified = false;
    bool publish_rgb = false;
    bool publish_detections = false;
    bool publish_imu = false;
    bool publish_stereo_neural_inference = false;
};

// List of task
struct OakUseList
{
    bool use_mono = false;
    bool use_depth = false;
    bool use_rectified = false;
    bool use_rgb = false;
    bool use_detections = false;
    bool use_imu = false;
    bool use_stereo_neural_inference = false;
};

// List of queue index
struct OakQueueIndex
{
    int inx_mono_left  = -1;
    int inx_mono_right = -1;
    int inx_depth = -1;
    int inx_rectified_left = -1;
    int inx_rectified_right = -1;
    int inx_rgb = -1;
    int inx_imu = -1;
    // OakDTaskDetections
    int inx_detections = -1;
    int inx_bbDepthMapping = -1;
    int inx_video = -1;
    // OakDTaskStereoNeuralInference
    int inx_detections_right = -1;
    int inx_detections_left = -1;
    int inx_imgManip_right = -1;
    int inx_imgManip_left = -1;
};


class OakDTask
{
public:
    virtual void start(ros::NodeHandle& nh) = 0;
    virtual void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                     OakQueueIndex& queue_index, std_msgs::Header header) = 0;
    virtual void stop() = 0;
};

#endif