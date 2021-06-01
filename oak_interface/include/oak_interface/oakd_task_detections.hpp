#ifndef OAKD_TASK_DETECTIONS
#define OAKD_TASK_DETECTIONS

#include "oakd_task.hpp"

class OakDTaskDetections : public OakDTask
{
public:
    OakDTaskDetections() = default;
    ~OakDTaskDetections() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index);
    void stop();

private:
    static const std::vector<std::string> label_map;

    std::shared_ptr<dai::ImgFrame> imgFrame;
    std::shared_ptr<dai::ImgFrame> depth;
    std::shared_ptr<dai::SpatialImgDetections> det;
    std::shared_ptr<dai::SpatialLocationCalculatorConfig> boundingBoxMapping;

    // ROS Messages
    oak_interface::BoundingBox bbox;
    oak_interface::BoundingBoxes msg;
    sensor_msgs::Image frame_msg;
    sensor_msgs::Image frame_depth_msg;
    // ROS Publisher
    ros::Publisher detections_pub;
    ros::Publisher image_detections_pub;
    ros::Publisher depth_pub;

    // Other necessary variables
    std::chrono::_V2::steady_clock::time_point startTime;
    int counter_;
    float fps_;
    cv::Scalar color_;

};

#endif