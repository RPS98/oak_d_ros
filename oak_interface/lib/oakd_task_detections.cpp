#include <oak_interface/oakd_task_detections.hpp>

void OakDTaskDetections::start(ros::NodeHandle& nh){};

void OakDTaskDetections::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index){};

void OakDTaskDetections::stop(){};