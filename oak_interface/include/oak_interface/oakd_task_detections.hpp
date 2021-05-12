#ifndef OAKD_TASK_DETECTIONS
#define OAKD_TASK_DETECTIONS

#include "oakd_task.hpp"

class OakDTaskDetections : public OakDTask
{
public:
    OakDTaskDetections() = default;
    ~OakDTaskDetections() = default;

public: // OakDTask
    void start(ros::NodeHandle& nh);
    void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index);
    void stop();

private:

};

#endif