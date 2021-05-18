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
             OakQueueIndex& queue_index);
    void stop();

private:
    sensor_msgs::Imu imu_msg;
    std::shared_ptr<dai::IMUData> imu_queue;

    //ROS Publisher
    ros::Publisher imu_pub;

    //Otras variables

    //Descomentar si necesitamos el tiempo
    //  std::chrono::_V2::steady_clock::time_point baseTs;
    //  std::chrono::_V2::steady_clock::time_point acceleroTs1;
    //  std::chrono::_V2::steady_clock::time_point acceleroTs;
    //  std::chrono::_V2::steady_clock::time_point gyroTs1;
    //  std::chrono::_V2::steady_clock::time_point gyroTs;
    //  std::chrono::_V2::steady_clock::time_point rvTs1;
    //  std::chrono::_V2::steady_clock::time_point rvTs;
    // int firstTs;
};

#endif




