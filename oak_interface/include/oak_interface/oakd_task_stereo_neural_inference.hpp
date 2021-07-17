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
    float aspect_ratio;
};

#define F_LEFT 872.2578152369078                // Focal length for left camera in pixels
#define F_RIGHT 875.2617250748619               // Focal length for right camera in pixels
#define FOCAL_LENGTH (F_LEFT + F_RIGHT)/2.0     // Focal length for both cameras in pixels
#define B 75                                    // Distance between left and right cameras in mm
#define CX_LEFT 644.2604132944807               // CX for left camera in pixels
#define CY_LEFT 370.415032403672                // CY for left camera in pixels                   
#define CX_RIGHT 641.3521672332931              // CX for right camera in pixels
#define CY_RIGHT 372.6642906480663              // CY for right camera in pixels
#define N_REDUCTIONS 2                          // Number of reductions for computing depth average

class OakDTaskStereoNeuralInference : public OakDTask
{
public:
    OakDTaskStereoNeuralInference() = default;
    ~OakDTaskStereoNeuralInference() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index, std_msgs::Header header);
    void stop();
    static bool wayToSort(const detections& i, const detections& j);

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

    // ROS Publishers
    ros::Publisher detections_pub;
    ros::Publisher image_right_det_pub;
    ros::Publisher image_left_det_pub;

    // Other necessary variables
    std::chrono::_V2::steady_clock::time_point startTime;
    int counter_;
    float fps_;
    cv::Scalar color_;

    // RUN VARIABLES
    int mono_camera_resolution = 800;
    int resolution_width = 1280;
    int resolution_height = 800;
    int NN_input_width = 300;
    int NN_input_high = 300;

    // Bounding boxes
    int bottle_counter_R = 0;
    int monitor_counter_R = 0;
    int table_counter_R = 0;
    int chair_counter_R = 0;

    int bottle_counter_L = 0;
    int monitor_counter_L = 0;
    int table_counter_L = 0;
    int chair_counter_L = 0;

    int labelIndex;
    std::string labelStr;
    int x1;
    int y1;
    int x2;
    int y2;

    int x1R;
    int y1R;
    int x2R;
    int y2R;

    int x1L;
    int y1L;
    int x2L;
    int y2L;

    std::stringstream confStr;

    // Disparity
    //float dist_threshold = 20000000.0;
    float area_threshold = 0.4; // From 0 to 1
    float depth = 0.0, depth_sum = 0.0, depth_avg = 0.0;
    float reduction = 0.0;
    float disparity;
    int disparity_null_counter = 0;
    int n_points;
    int y_threshold = 10;  // In pixels
    float ratio_threshold = 0.4; // From 0 to 1
    int i = 0;

    float distance;
    float area_error;
    float ratio_error;

    float Z_centroid;
    float X_centroid;
    float Y_centroid;

    std::stringstream XStr;
    std::stringstream YStr;
    std::stringstream ZStr;
};

#endif