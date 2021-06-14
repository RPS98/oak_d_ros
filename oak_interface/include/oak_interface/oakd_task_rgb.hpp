#ifndef OAKD_TASK_RGB
#define OAKD_TASK_RGB

#include "oakd_task.hpp"

class OakDTaskRGB : public OakDTask
{
public:
    OakDTaskRGB() = default;
    ~OakDTaskRGB() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index, std_msgs::Header header);
    void stop();

private:
    // Cameras info
    std::unique_ptr<camera_info_manager::CameraInfoManager> rgb_camInfoManager;
    sensor_msgs::CameraInfo rgb_CameraInfo;
    // Image
    sensor_msgs::Image rgb_image_msg;
    std::shared_ptr<dai::ImgFrame> rgb_frame;
    // Publishers
    ros::Publisher rgb_pub;
    ros::Publisher rgb_info_pub;

    // Use interleaved
    bool interleaved = true;
};

#endif