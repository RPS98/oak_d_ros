#ifndef OAKD_INTERFACE
#define OAKD_INTERFACE

#include "oakd_task.hpp"
#include "oakd_pipeline.hpp"

#include "oakd_task_mono.hpp"
#include "oakd_task_depth.hpp"
#include "oakd_task_rectified.hpp"
#include "oakd_task_rgb.hpp"
#include "oakd_task_detections.hpp"
#include "oakd_task_imu.hpp"

class OakDInterface : public OakDProcess
{
public:
    OakDInterface() = default;
    ~OakDInterface() = default;

private: // OakDProcess
    void ownSetUp();
    void ownStart();
    void ownRun();
    void ownStop();

private:
    std::vector<OakDTask*> tasks_list_; // List of tasks

    ros::NodeHandle nh;
    OakDPipeline oakd_pipeline;
    std::vector<std::shared_ptr<dai::DataOutputQueue>> streams_queue_; // queue with Oak-D conexion

    OakQueueIndex queue_index_; // Index of streams_queue_

    
    // Start functions
    void read_param(OakPublishList& publish_list);
    void create_use_list(OakUseList& use_list, OakPublishList& publish_list);

    int seq = 1;
    std_msgs::Header header;
};

#endif