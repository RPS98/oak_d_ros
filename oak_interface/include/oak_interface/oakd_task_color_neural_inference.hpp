#ifndef OAKD_TASK_COLOR_NEURAL_INFERENCE
#define OAKD_TASK_COLOR_NEURAL_INFERENCE

#include "oakd_task.hpp"

class OakDTaskColorNeuralInference : public OakDTask
{
public:
    OakDTaskColorNeuralInference() = default;
    ~OakDTaskColorNeuralInference() = default;

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

    // ROS Subscriber
    ros::Subscriber color_image_sub;

    // Other necessary variables
    std::chrono::_V2::steady_clock::time_point startTime;
    int counter_;
    float fps_;
    cv::Scalar color_;

public:
    void colorImageCallback(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat color_frame;
    
};

#endif