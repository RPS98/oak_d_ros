#ifndef OAKD_TASK_MONO
#define OAKD_TASK_MONO

#include "oakd_task.hpp"

class OakDTaskMono : public OakDTask
{
public:
    OakDTaskMono() = default;
    ~OakDTaskMono() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index);
    void stop();

private:
    // Left
    // Cameras info
    std::unique_ptr<camera_info_manager::CameraInfoManager> left_camInfoManager;
    sensor_msgs::CameraInfo left_CameraInfo;
    // Image
    sensor_msgs::Image left_image_msg;
    std::shared_ptr<dai::ImgFrame> left_frame;
    // Publishers
    ros::Publisher left_pub;
    ros::Publisher left_info_pub;

    // Right
    // Cameras info
    std::unique_ptr<camera_info_manager::CameraInfoManager> right_camInfoManager;
    sensor_msgs::CameraInfo right_CameraInfo;
    // Image
    sensor_msgs::Image right_image_msg;
    std::shared_ptr<dai::ImgFrame> right_frame;
    // Publishers
    ros::Publisher right_pub;
    ros::Publisher right_info_pub;
};

#endif