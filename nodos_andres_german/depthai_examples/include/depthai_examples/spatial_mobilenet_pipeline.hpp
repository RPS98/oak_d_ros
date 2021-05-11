#pragma once

#include <vector>
#include "depthai/depthai.hpp"

class SpatialMobileNetDetectionExample{

    public:

    static const std::vector<std::string> label_map ;

    SpatialMobileNetDetectionExample() = default;
    ~SpatialMobileNetDetectionExample() = default;

    dai::Pipeline createSpatialNNPipeline(std::string nnPath); // new

    //std::unique_ptr<dai::Device> _dev; 
    dai::Pipeline _p;

};