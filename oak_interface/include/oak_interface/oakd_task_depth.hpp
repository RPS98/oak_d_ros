#ifndef OAKD_TASK_DEPTH
#define OAKD_TASK_DEPTH

#include "oakd_task.hpp"

class OakDTaskDepth : public OakDTask
{
public:
    OakDTaskDepth() = default;
    ~OakDTaskDepth() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index);
    void stop();

private:
    // Cameras info
    std::unique_ptr<camera_info_manager::CameraInfoManager> depth_camInfoManager;
    sensor_msgs::CameraInfo depth_CameraInfo;
    // Image
    sensor_msgs::Image depth_image_msg;
    std::shared_ptr<dai::ImgFrame> depth_frame;
    // Publishers
    ros::Publisher depth_pub;
    ros::Publisher depth_info_pub;
};

#endif