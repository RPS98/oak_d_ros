#ifndef OAKD_TASK_COLOR
#define OAKD_TASK_COLOR

#include "oakd_task.hpp"

class OakDTaskColor : public OakDTask
{
public:
    OakDTaskColor() = default;
    ~OakDTaskColor() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index, std_msgs::Header header);
    void stop();

private:
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
    bool use_interleaved = true;
    void get_RosMsg(std::shared_ptr<dai::ImgFrame> color_frame,
                    sensor_msgs::Image &color_image_msg,
                    std_msgs::Header header);
};

#endif