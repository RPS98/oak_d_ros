
#include <depthai_examples/spatial_mobilenet_pipeline.hpp>
#include "depthai/depthai.hpp"

const std::vector<std::string> SpatialMobileNetDetectionExample::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"}; 

dai::Pipeline SpatialMobileNetDetectionExample::createSpatialNNPipeline(std::string nnPath){ // new

    bool syncNN = true;

    // Create the nodes
    auto colorCam = _p.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = _p.create<dai::node::MobileNetSpatialDetectionNetwork>(); // new
    auto monoLeft = _p.create<dai::node::MonoCamera>(); // new
    auto monoRight = _p.create<dai::node::MonoCamera>(); // new
    auto stereo = _p.create<dai::node::StereoDepth>(); // new

    // Create xlink connections
    auto xlinkOut = _p.create<dai::node::XLinkOut>(); // xlinOut = xoutRgb
    auto nnOut = _p.create<dai::node::XLinkOut>(); // nnOut = xoutNN
    auto xoutBoundingBoxDepthMapping = _p.create<dai::node::XLinkOut>();// new
    auto xoutDepth = _p.create<dai::node::XLinkOut>(); // new
    // auto detectionNetwork = _p.create<dai::node::MobileNetDetectionNetwork>(); // deleted
    

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");
    xoutBoundingBoxDepthMapping->setStreamName("boundingBoxDepthMapping"); // new
    xoutDepth->setStreamName("depth"); // new

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(40);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P); // new
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT); // new
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P); // new
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT); // new

    // setting node configs
    stereo->setOutputDepth(true); // new
    stereo->setConfidenceThreshold(255); // new

    spatialDetectionNetwork->setBlobPath(nnPath); // new
    spatialDetectionNetwork->setConfidenceThreshold(0.5f); // new
    spatialDetectionNetwork->input.setBlocking(false); // new
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5); // new
    spatialDetectionNetwork->setDepthLowerThreshold(100); // new
    spatialDetectionNetwork->setDepthUpperThreshold(5000); // new

    // testing MobileNet DetectionNetwork (deleted)
    //detectionNetwork->setConfidenceThreshold(0.5f);
    //detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left); // new
    monoRight->out.link(stereo->right); // new

    // Link plugins CAM -> NN -> XLINK
    // colorCam->preview.link(detectionNetwork->input); // deleted
    colorCam->preview.link(spatialDetectionNetwork->input); // new
    if(syncNN) {
        // detectionNetwork->passthrough.link(xlinkOut->input);
        spatialDetectionNetwork->passthrough.link(xlinkOut->input); // new
    }
    else colorCam->preview.link(xlinkOut->input); // xlinkOut = xoutRgb


    // detectionNetwork->out.link(nnOut->input); // deleted
    spatialDetectionNetwork->out.link(nnOut->input); // new
    spatialDetectionNetwork->boundingBoxMapping.link(xoutBoundingBoxDepthMapping->input); // new

    stereo->depth.link(spatialDetectionNetwork->inputDepth); // new
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input); // new

    return _p;

    /*
    _dev = std::make_unique<dai::Device>(_p);
    _dev->startPipeline();

    _opImageStreams.push_back(_dev->getOutputQueue("preview", 30, false));
    _opNNetStreams.push_back(_dev->getOutputQueue("detections", 30, false));
    _opImageStreamsDepth.push_back(_dev->getOutputQueue("depth", 30, false)); // new
    _opBoundingBoxDepthMapping.push_back(_dev->getOutputQueue("boundingBoxDepthMapping", 30, false)); // new */

}