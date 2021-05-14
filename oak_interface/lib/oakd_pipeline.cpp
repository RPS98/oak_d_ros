#include <oak_interface/oakd_interface.hpp>

void OakDPipeline::start(OakUseList& use_list, 
                         std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue,
                         OakQueueIndex& queue_index){

    // Mono Camera
    if(use_list.use_mono){
        // XLinkOut
        auto xoutLeft  = pipeline_.create<dai::node::XLinkOut>();
        auto xoutRight = pipeline_.create<dai::node::XLinkOut>();

        // Parameters
        int mono_camera_resolution = 720;
        if (ros::param::has("/mono_camera_resolution")) {
            ros::param::get("/mono_camera_resolution", mono_camera_resolution);
        }
        float mono_camera_fps = 60.0;
        if (ros::param::has("/mono_camera_fps")) {
            ros::param::get("/mono_camera_fps", mono_camera_fps);
        }

        // MonoCamera node
        auto monoLeft = pipeline_.create<dai::node::MonoCamera>();
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        auto monoRight = pipeline_.create<dai::node::MonoCamera>();
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        if(mono_camera_resolution == 720){
            monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
            monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        } else if(mono_camera_resolution == 800){
            monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
            monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P); 
        } else if(mono_camera_resolution == 400){
            monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
            monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P); 
        } else {
            ROS_INFO_STREAM("Default mono camera resolution 720");
            monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
            monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        }

        monoLeft->setFps(mono_camera_fps);
        monoRight->setFps(mono_camera_fps);
        // monoRight->setImageOrientation(CameraImageOrientationimageOrientation);

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft ->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);

        // Data from device to host via XLink
        xoutLeft ->setStreamName("left");
        xoutRight->setStreamName("right");
    

        // Depth Camera
        if(use_list.use_depth){
            // XLinkOut
            auto xoutDepth   = pipeline_.create<dai::node::XLinkOut>();

            // StereoDepth node
            auto stereo = pipeline_.create<dai::node::StereoDepth>();

            //stereo->setInputResolution(int width, int height); (Optional if MonoCamera exists, otherwise necessary)
            if(!use_list.use_mono){
                stereo->setInputResolution(1280, 800);
            }

            //stereo->loadCalibrationFile(const std::string &path);
            //stereo->loadCalibrationData(const std::vector<std::uint8_t> &data);
            //stereo->setEmptyCalibration();
            //stereo->setMedianFilter(Properties::MedianFilter median);
            //stereo->setDepthAlign(Properties::DepthAlign align);
            stereo->setConfidenceThreshold(200);
            //stereo->setLeftRightCheck(bool enable);
            //stereo->setSubpixel(bool enable);
            //stereo->setExtendedDisparity(bool enable);
            stereo->setRectifyEdgeFillColor(0);
            //stereo->setRectifyMirrorFrame(bool enable);
            //stereo->setOutputRectified(oakTasksList.use_rectified); // DEPRECATED
            //stereo->setOutputDepth(oakTasksList.use_depth); // DEPRECATED

            // Needed to work
            monoLeft->out.link(stereo->left);
            monoRight->out.link(stereo->right);
            
            // Link plugins CAM -> STEREO -> XLINK
            stereo->depth.link(xoutDepth->input); // RAW16 encoded (0..65535) depth data in millimeters
            //stereo->disparity.link(xoutDepth->input);   // RAW8 / RAW16 encoded disparity data
            //stereo->syncedLeft.link(xoutLeft->input);  // Passthrough ImgFrame message from ‘left’ Input
            //stereo->syncedRight.link(xoutRight->input); // Passthrough ImgFrame message from right Input

            // Data from device to host via XLink
            xoutDepth->setStreamName("depth");

            // Rectified Images
            // If use rectified
            if(use_list.use_rectified){
                // XLinkOut
                auto xoutRectifL = pipeline_.create<dai::node::XLinkOut>();
                auto xoutRectifR = pipeline_.create<dai::node::XLinkOut>();

                // Link plugins CAM -> STEREO -> XLINK
                stereo->rectifiedLeft.link(xoutRectifL->input);
                stereo->rectifiedRight.link(xoutRectifR->input);

                // Data from device to host via XLink
                xoutRectifL ->setStreamName("rectified_left");
                xoutRectifR ->setStreamName("rectified_right");
            }

            // Spatial Detection Network
            // If use detections
            if(use_list.use_detections){
/*
                // XLinkOut
                //auto xoutMN_SDN = pipeline_.create<dai::node::XLinkOut>();

                // ColorCamera node
                auto mobilenetSpatial = pipeline_.create<dai::node::MobileNetSpatialDetectionNetwork>();
                
                std::string nnBlobPath = "default";
                if (ros::param::has("/nnBlobPath")) {
                    ros::param::get("/nnBlobPath", nnBlobPath);
                }
                mobilenetSpatial->setBlobPath(nnBlobPath);
                // Will ingore all detections whose confidence is below 50%
                mobilenetSpatial->setConfidenceThreshold(0.5f);
                mobilenetSpatial->input.setBlocking(false);
                // How big the ROI will be (smaller value can provide a more stable reading)
                mobilenetSpatial->setBoundingBoxScaleFactor(0.5f);
                // Min/Max threshold. Values out of range will be set to 0 (invalid)
                mobilenetSpatial->setDepthLowerThreshold(100);
                mobilenetSpatial->setDepthUpperThreshold(5000);

                // Link depth from the StereoDepth node
                stereo->depth.link(mobilenetSpatial->inputDepth);
*/

            }
        }
    }

    // RGB Camera
    if(use_list.use_rgb){
        // XLinkOut
        auto xoutRGB = pipeline_.create<dai::node::XLinkOut>();

        // Parameters
        int rgb_camera_resolution = 1080;
        if (ros::param::has("/rgb_camera_resolution")) {
            ros::param::get("/rgb_camera_resolution", rgb_camera_resolution);
        }
        float rgb_camera_fps = 60.0;
        if (ros::param::has("/rgb_camera_fps")) {
            ros::param::get("/rgb_camera_fps", rgb_camera_fps);
        }

        // ColorCamera node
        auto colorCam = pipeline_.create<dai::node::ColorCamera>();
        colorCam->setPreviewSize(1280, 720);
        colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);

        if(rgb_camera_resolution == 1080){
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        } else if(rgb_camera_resolution == 3840){
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
        } else if(rgb_camera_resolution == 2826){
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_12_MP);
        } else {
            ROS_INFO_STREAM("Default rgb camera resolution 720");
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        }
        
        //colorCam->setInterleaved(false); // Function getRosMsg do not support
        colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
        colorCam->setFps(rgb_camera_fps);
        // colorCam->setCamId(int64_t id);
        // colorCam->setImageOrientation(CameraImageOrientationimageOrientation);
        // colorCam->setInterleaved(bool interleaved);
        // colorCam->setPreviewSize(int width, int height);
        // colorCam->setVideoSize(int width, int height);
        // colorCam->setStillSize(int width, int height);
        // colorCam->setIspScale(int numerator, int denominator);
        // colorCam->setPreviewKeepAspectRatio(bool keep);

        // Link plugins CAM -> STEREO -> XLINK
        colorCam->preview.link(xoutRGB->input); // BGR/RGB planar/interleaved encoded
        //colorCam->video.link(xoutRGB->input); // NV12 encoded (YUV420, UV plane interleaved)
        //colorCam->still.link(xoutRGB->input); // NV12 encoded (YUV420, UV plane interleaved) when inputControl
        //colorCam->isp.link(xoutRGB->input);   // YUV420 planar (I420/IYUV)
        //colorCam->raw.link(xoutRGB->input);   // RAW10-packed (MIPI CSI-2 format)

        // Data from device to host via XLink
        xoutRGB->setStreamName("rgb");
    }

    // CONNECT TO DEVICE
    dev_ = std::make_unique<dai::Device>(pipeline_);
    dev_->startPipeline();

    int counter = 0;
    int queueSize = 1;

    if(use_list.use_mono){
        streams_queue.push_back(dev_->getOutputQueue("left", queueSize, false));
        queue_index.inx_mono_left = counter; counter++;
        streams_queue.push_back(dev_->getOutputQueue("right", queueSize, false));
        queue_index.inx_mono_right = counter; counter++;
    }
    if(use_list.use_depth){
        streams_queue.push_back(dev_->getOutputQueue("depth", queueSize, false));
        queue_index.inx_depth = counter; counter++;
    }
    if(use_list.use_rectified){
        streams_queue.push_back(dev_->getOutputQueue("rectified_left", queueSize, false));
        queue_index.inx_rectified_left = counter; counter++;
        streams_queue.push_back(dev_->getOutputQueue("rectified_right", queueSize, false));
        queue_index.inx_rectified_right = counter; counter++;
    }
    if(use_list.use_rgb){
        streams_queue.push_back(dev_->getOutputQueue("rgb", queueSize, true));
        queue_index.inx_rgb = counter; counter++;
    }
/*
    if(use_list.use_detections){
        streams_queue.push_back(dev_->getOutputQueue("detections", queueSize, true));
        queue_index.inx_detections = counter; counter++;
    }
*/
}

void OakDPipeline::stop(){
    dev_->close();
}
