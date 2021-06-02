#ifndef OAKD_TASK_STEREO_NEURAL_INFERENCE
#define OAKD_TASK_STEREO_NEURAL_INFERENCE

#include "oakd_task.hpp"

struct detections{
    std::string type;
    float prob; 
    int x1, y1; // Bounding box coordinates
    int x2, y2; // Bounding box coordinates
    float cx, cy; // Center coordinates
    float area;
};

class OakDTaskStereoNeuralInference : public OakDTask
{
public:
    OakDTaskStereoNeuralInference() = default;
    ~OakDTaskStereoNeuralInference() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index);
    void stop();

private:
    static const std::vector<std::string> label_map;
    detections dets;
    std::vector<detections> detsRight;
    std::vector<detections> detsLeft;

    std::shared_ptr<dai::ImgFrame> right;
    std::shared_ptr<dai::ImgFrame> left;
    std::shared_ptr<dai::ImgDetections> det_right;
    std::shared_ptr<dai::ImgDetections> det_left;
    std::shared_ptr<dai::ImgFrame> rectified_left;
    std::shared_ptr<dai::ImgFrame> rectified_right;

    // ROS Messages
    oak_interface::BoundingBox bbox;
    oak_interface::BoundingBoxes msg;
    sensor_msgs::Image image_right_with_det;
    sensor_msgs::Image image_left_with_det;

    // ROS Publisher
    ros::Publisher detections_pub;
    ros::Publisher image_right_det_pub;
    ros::Publisher image_left_det_pub;

    // Other necessary variables
    std::chrono::_V2::steady_clock::time_point startTime;
    int counter_;
    float fps_;
    cv::Scalar color_;

};

#endif