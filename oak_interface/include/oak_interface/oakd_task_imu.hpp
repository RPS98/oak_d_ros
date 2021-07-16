#ifndef OAKD_TASK_IMU
#define OAKD_TASK_IMU

#include "oakd_task.hpp"

class OakDTaskIMU : public OakDTask{

public: 
    OakDTaskIMU() = default;
    ~OakDTaskIMU() = default;

public:
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index, std_msgs::Header header);
    void stop();

private:
    sensor_msgs::Imu imu_msg;
    std::shared_ptr<dai::IMUData> imu_queue;

    //ROS Publisher
    ros::Publisher imu_pub;
};

#endif




