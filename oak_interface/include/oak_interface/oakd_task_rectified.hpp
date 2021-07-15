#ifndef OAKD_TASK_RECTIDIED
#define OAKD_TASK_RECTIDIED

#include "oakd_task.hpp"

class OakDTaskRectified : public OakDTask
{
public:
    OakDTaskRectified() = default;
    ~OakDTaskRectified() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index, std_msgs::Header header);
    void stop();

private:
    // Left
    // Cameras info
    std::unique_ptr<camera_info_manager::CameraInfoManager> left_rectified_camInfoManager;
    sensor_msgs::CameraInfo left_rectified_CameraInfo;
    // Image
    sensor_msgs::Image left_rectified_image_msg;
    std::shared_ptr<dai::ImgFrame> left_rectified_frame;
    // Publishers
    ros::Publisher left_rectified_pub;
    ros::Publisher left_rectified_info_pub;

    // Right
    // Cameras info
    std::unique_ptr<camera_info_manager::CameraInfoManager> right_rectified_camInfoManager;
    sensor_msgs::CameraInfo right_rectified_CameraInfo;
    // Image
    sensor_msgs::Image right_rectified_image_msg;
    std::shared_ptr<dai::ImgFrame> right_rectified_frame;
    // Publishers
    ros::Publisher right_rectified_pub;
    ros::Publisher right_rectified_info_pub;
};

#endif