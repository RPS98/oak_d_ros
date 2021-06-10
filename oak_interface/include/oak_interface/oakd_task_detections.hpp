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
             OakQueueIndex& queue_index, std_msgs::Header header);
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

    // For RGB Image
    bool publish_rgb = false;
    // Cameras info
    std::unique_ptr<camera_info_manager::CameraInfoManager> rgb_camInfoManager;
    sensor_msgs::CameraInfo rgb_CameraInfo;
    // Image
    sensor_msgs::Image rgb_image_msg;
    // Publishers
    ros::Publisher rgb_pub;
    ros::Publisher rgb_info_pub;

    // Other necessary variables
    std::chrono::_V2::steady_clock::time_point startTime;
    int counter_;
    float fps_;
    cv::Scalar color_;

};

#endif