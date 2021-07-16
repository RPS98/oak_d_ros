#ifndef OAKD_PIPELINE
#define OAKD_PIPELINE

#include "oakd_task.hpp"

class OakDPipeline
{
public:
    OakDPipeline() = default;
    ~OakDPipeline() = default;

public:
    void start(OakUseList& use_list, 
               std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue,
               OakQueueIndex& queue_index);

    void stop();

private:
    std::unique_ptr<dai::Device> dev_;
    dai::Pipeline pipeline_;
};

#endif