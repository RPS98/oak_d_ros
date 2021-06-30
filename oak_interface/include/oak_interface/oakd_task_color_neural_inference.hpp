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

    // DETECTIONS
    static const std::vector<std::string> label_map;

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



    // COLOR IMAGE PUBLISHER
    // Cameras info
    std::unique_ptr<camera_info_manager::CameraInfoManager> color_camInfoManager;
    sensor_msgs::CameraInfo color_CameraInfo;
    // Image
    sensor_msgs::Image color_image_msg;
    std::shared_ptr<dai::ImgFrame> color_frame;
    // Publishers
    ros::Publisher color_pub;
    ros::Publisher color_info_pub;

    // Use interleaved
    bool publish_color = false;
    bool use_interleaved = false;
    void get_RosMsg(std::shared_ptr<dai::ImgFrame> color_frame,
                    sensor_msgs::Image &color_image_msg,
                    std_msgs::Header header);



    // RUN VARIABLES
    int labelIndex;
    std::string labelStr;
    int x1;
    int y1;
    int x2;
    int y2;
    std::stringstream confStr;
    std::stringstream depthX;
    std::stringstream depthY;
    std::stringstream depthZ;
    std::stringstream fpsStr;
};

#endif