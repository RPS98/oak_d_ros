#include <oak_interface/oakd_interface.hpp>

void OakDPipeline::start(OakUseList& use_list, 
                         std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue,
                         OakQueueIndex& queue_index){
    
    // NODES DECLARATION

    // MonoCamera nodes
    std::shared_ptr<dai::node::MonoCamera> monoLeft = nullptr;
    std::shared_ptr<dai::node::MonoCamera> monoRight = nullptr;

    // StereoDepth node
    std::shared_ptr<dai::node::StereoDepth> stereo = nullptr;

    // ColorCamera node
    std::shared_ptr<dai::node::ColorCamera> colorCam = nullptr;

    // Imu node
    std::shared_ptr<dai::node::IMU> imu = nullptr;

    // MobileNetSpatialDetectionNetwork node for color neural inference
    std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> mobilenetSpatialDetectionNetwork_color = nullptr;

    // MobilenetDetectionNetwork nodes for stereo neural inference
    std::shared_ptr<dai::node::MobileNetDetectionNetwork> mobilenetDetectionNetwork_right = nullptr;
    std::shared_ptr<dai::node::MobileNetDetectionNetwork> mobilenetDetectionNetwork_left  = nullptr;

    // ImagesManip nodes for stereo neural inference
    std::shared_ptr<dai::node::ImageManip> imageManip_right = nullptr;
    std::shared_ptr<dai::node::ImageManip> imageManip_left  = nullptr;


    // XLINKOUT DECLARATION

    // XLinkOut nodes
    std::shared_ptr<dai::node::XLinkOut> xoutLeft    = nullptr; // Left mono image
    std::shared_ptr<dai::node::XLinkOut> xoutRight   = nullptr; // Right mono image
    std::shared_ptr<dai::node::XLinkOut> xoutDepth   = nullptr; // Depth image
    std::shared_ptr<dai::node::XLinkOut> xoutRectifL = nullptr; // Left mono image rectified
    std::shared_ptr<dai::node::XLinkOut> xoutRectifR = nullptr; // Right mono image rectified
    std::shared_ptr<dai::node::XLinkOut> xoutColor   = nullptr; // Color image
    std::shared_ptr<dai::node::XLinkOut> xoutIMU     = nullptr; // IMU

    // XLinkOut nodes for color neural inference
    std::shared_ptr<dai::node::XLinkOut> nnColorOutDetections        = nullptr;
    std::shared_ptr<dai::node::XLinkOut> xoutBoundingBoxDepthMapping = nullptr;

    // XLinkOut nodes for stereo neural inference
    std::shared_ptr<dai::node::XLinkOut> nnRightOut     = nullptr;
    std::shared_ptr<dai::node::XLinkOut> nnLeftOut      = nullptr;
    std::shared_ptr<dai::node::XLinkOut> xoutManipRight = nullptr;
    std::shared_ptr<dai::node::XLinkOut> xoutManipLeft  = nullptr;

    // Mono Camera
    if(use_list.use_mono){
        // XLinkOut
        xoutLeft  = pipeline_.create<dai::node::XLinkOut>();
        xoutRight = pipeline_.create<dai::node::XLinkOut>();

        // Parameters
        int mono_camera_resolution = 720;
        if (ros::param::has("/mono_camera_resolution")) {
            ros::param::get("/mono_camera_resolution", mono_camera_resolution);
        }
        float mono_camera_fps = 60.0;
        if (ros::param::has("/mono_camera_fps")) {
            ros::param::get("/mono_camera_fps", mono_camera_fps);
        }

        // MonoCamera node properties
        monoLeft = pipeline_.create<dai::node::MonoCamera>();
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight = pipeline_.create<dai::node::MonoCamera>();
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

        if (ros::param::has("/image_orientation")) {
            int image_orientation;
            ros::param::get("/image_orientation", image_orientation);
            if(image_orientation == 1){
                monoLeft->setImageOrientation(dai::CameraImageOrientation::AUTO);
                monoRight->setImageOrientation(dai::CameraImageOrientation::AUTO);
            } else if(image_orientation == 2){
                monoLeft->setImageOrientation(dai::CameraImageOrientation::NORMAL);
                monoRight->setImageOrientation(dai::CameraImageOrientation::NORMAL);
            } else if(image_orientation == 3){
                monoLeft->setImageOrientation(dai::CameraImageOrientation::HORIZONTAL_MIRROR);
                monoRight->setImageOrientation(dai::CameraImageOrientation::HORIZONTAL_MIRROR);
            } else if(image_orientation == 4){
                monoLeft->setImageOrientation(dai::CameraImageOrientation::VERTICAL_FLIP);
                monoRight->setImageOrientation(dai::CameraImageOrientation::VERTICAL_FLIP);
            } else if(image_orientation == 5){
                monoLeft->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
                monoRight->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
            }
        }
        // monoRight->setImageOrientation(CameraImageOrientationimageOrientation);

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);

        // Data from device to host via XLink
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
    

        // Depth Camera
        if(use_list.use_depth){
            // XLinkOut
            xoutDepth = pipeline_.create<dai::node::XLinkOut>();

            // StereoDepth node
            stereo = pipeline_.create<dai::node::StereoDepth>();

            //stereo->setInputResolution(int width, int height); (Optional if MonoCamera exists, otherwise necessary)
            if(!use_list.use_mono){
                if(mono_camera_resolution == 720){
                    stereo->setInputResolution(1280, 720);
                } else if(mono_camera_resolution == 800){
                    stereo->setInputResolution(1280, 800);
                } else if(mono_camera_resolution == 400){
                    stereo->setInputResolution(640, 480); 
                } else {
                    ROS_INFO_STREAM("Default mono camera resolution 720");
                    stereo->setInputResolution(1280, 720);
                }
            }

            std::string path;
            stereo->loadCalibrationFile(path);
            //stereo->loadCalibrationData(const std::vector<std::uint8_t> &data);
            //stereo->setEmptyCalibration();
            stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::KERNEL_7x7);
            //stereo->setDepthAlign(Properties::DepthAlign align);
            stereo->setConfidenceThreshold(100);
            //stereo->setLeftRightCheck(bool enable);
            //stereo->setSubpixel(bool enable);
            //stereo->setExtendedDisparity(bool enable);
            //stereo->setRectifyEdgeFillColor(0);
            //stereo->setRectifyMirrorFrame(bool enable);
            //stereo->setOutputRectified(use_list.use_rectified); // DEPRECATED
            //stereo->setOutputDepth(use_list.use_depth); // DEPRECATED

            // Needed to work
            monoLeft->out.link(stereo->left);
            monoRight->out.link(stereo->right);
            
            // Link plugins CAM -> STEREO -> XLINK
            if(!use_list.use_color_detections){
                stereo->depth.link(xoutDepth->input); // RAW16 encoded (0..65535) depth data in millimeters
            }
            
            //stereo->disparity.link(xoutDepth->input);   // RAW8 / RAW16 encoded disparity data
            //stereo->syncedLeft.link(xoutLeft->input);  // Passthrough ImgFrame message from ‘left’ Input
            //stereo->syncedRight.link(xoutRight->input); // Passthrough ImgFrame message from right Input

            // Data from device to host via XLink
            xoutDepth->setStreamName("depth");

            // Rectified Images
            // If use rectified
            if(use_list.use_rectified){
                // XLinkOut
                xoutRectifL = pipeline_.create<dai::node::XLinkOut>();
                xoutRectifR = pipeline_.create<dai::node::XLinkOut>();

                // Link plugins CAM -> STEREO -> XLINK
                stereo->rectifiedLeft.link(xoutRectifL->input);
                stereo->rectifiedRight.link(xoutRectifR->input);

                // Data from device to host via XLink
                xoutRectifL->setStreamName("rectified_left");
                xoutRectifR->setStreamName("rectified_right");
            }
        }
    }

    // IMU
    if(use_list.use_imu){
        // XLinkout
        xoutIMU = pipeline_.create<dai::node::XLinkOut>();
        // IMU node
        imu = pipeline_.create<dai::node::IMU>();
        
        dai::IMUSensorConfig sensorConfig;
        // sensorConfig.reportIntervalUs = 2500; // 400 Hz (el maximo por ahora)
        sensorConfig.sensorId = dai::IMUSensor::ACCELEROMETER_RAW;
        imu->enableIMUSensor(sensorConfig);
        sensorConfig.sensorId = dai::IMUSensor::GYROSCOPE_RAW;
        imu->enableIMUSensor(sensorConfig);
        sensorConfig.sensorId = dai::IMUSensor::ROTATION_VECTOR;            
        imu->enableIMUSensor(sensorConfig);

        imu->setBatchReportThreshold(1);
        imu->setMaxBatchReports(5);
        
        // Link
        imu->out.link(xoutIMU->input);

        // Data from device to host via XLink
        xoutIMU->setStreamName("imu");
    }

    // Color  Camera
    if(use_list.use_color){
        // XLinkOut
        xoutColor  = pipeline_.create<dai::node::XLinkOut>();

        // Parameters
        int color_camera_resolution = 1080;
        if (ros::param::has("/color_camera_resolution")) {
            ros::param::get("/color_camera_resolution", color_camera_resolution);
        }
        float color_camera_fps = 60.0;
        if (ros::param::has("/color_camera_fps")) {
            ros::param::get("/color_camera_fps", color_camera_fps);
        }
        bool use_BGR = true;
        if (ros::param::has("/use_BGR")) {
            ros::param::get("/use_BGR", use_BGR);
        }

        // ColorCamera node
        colorCam = pipeline_.create<dai::node::ColorCamera>();
        colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);

        if(color_camera_resolution == 1080){
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
            colorCam->setPreviewSize(1920, 1080);
        } else if(color_camera_resolution == 3840){
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
            colorCam->setPreviewSize(3840, 2160);
        } else if(color_camera_resolution == 2826){
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_12_MP);
            colorCam->setPreviewSize(4247, 2826);
        } else {
            ROS_INFO_STREAM("Default color camera resolution 720");
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
            colorCam->setPreviewSize(1920, 1080);
        }

        if(use_BGR){
            colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
        } else {
            colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
        }

        bool color_interleaved = true;
        if (ros::param::has("/color_interleaved")) {
            ros::param::get("/color_interleaved", color_interleaved);
        }
        colorCam->setInterleaved(color_interleaved);
        
        colorCam->setFps(color_camera_fps);

        if (ros::param::has("/image_orientation")) {
            int image_orientation;
            ros::param::get("/image_orientation", image_orientation);
            if(image_orientation == 1){
                colorCam->setImageOrientation(dai::CameraImageOrientation::AUTO);
            } else if(image_orientation == 2){
                colorCam->setImageOrientation(dai::CameraImageOrientation::NORMAL);
            } else if(image_orientation == 3){
                colorCam->setImageOrientation(dai::CameraImageOrientation::HORIZONTAL_MIRROR);
            } else if(image_orientation == 4){
                colorCam->setImageOrientation(dai::CameraImageOrientation::VERTICAL_FLIP);
            } else if(image_orientation == 5){
                colorCam->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
            }
        }

        // colorCam->setCamId(int64_t id);
        // colorCam->setImageOrientation(dai::CameraImageOrientation::HORIZONTAL_MIRROR);
        // colorCam->setPreviewSize(int width, int height);
        // colorCam->setVideoSize(int width, int height);
        // colorCam->setStillSize(int width, int height);
        // colorCam->setIspScale(int numerator, int denominator);
        // colorCam->setPreviewKeepAspectRatio(bool keep);

        // Link plugins CAM -> STEREO -> XLINK
        if(!use_list.use_color_detections){
            colorCam->preview.link(xoutColor->input); // BGR/RGB planar/interleaved encoded
        }
        //colorCam->video.link(xoutColor->input); // NV12 encoded (YUV420, UV plane interleaved)
        //colorCam->still.link(xoutColor->input); // NV12 encoded (YUV420, UV plane interleaved) when inputControl
        //colorCam->isp.link(xoutColor->input);   // YUV420 planar (I420/IYUV)
        //colorCam->raw.link(xoutColor->input);   // RAW10-packed (MIPI CSI-2 format)

        // Data from device to host via XLink
        xoutColor->setStreamName("color");
    }

    // Spatial Detection Network
    // If use detections
    if(use_list.use_color_detections){

        std::string nnBlobPath = "default";
        if (ros::param::has("/nnBlobPath")) {
            ros::param::get("/nnBlobPath", nnBlobPath);
        }

        int NN_input_width = 300;
        if (ros::param::has("/NN_input_width")) {
            ros::param::get("/NN_input_width", NN_input_width);
        }
        int NN_input_high = 300;
        if (ros::param::has("/NN_input_high")) {
            ros::param::get("/NN_input_high", NN_input_high);
        }
        colorCam->setPreviewSize(NN_input_width, NN_input_high);

        // XLinkOut nodes
        nnColorOutDetections = pipeline_.create<dai::node::XLinkOut>();
        nnColorOutDetections->setStreamName("detections_color");

        xoutBoundingBoxDepthMapping = pipeline_.create<dai::node::XLinkOut>();
        xoutBoundingBoxDepthMapping->setStreamName("boundingBoxDepthMapping");

        // MobileNetSpatialDetectionNetwork node
        mobilenetSpatialDetectionNetwork_color = pipeline_.create<dai::node::MobileNetSpatialDetectionNetwork>();
        mobilenetSpatialDetectionNetwork_color->setBlobPath(nnBlobPath);
        mobilenetSpatialDetectionNetwork_color->setConfidenceThreshold(0.5f); 
        mobilenetSpatialDetectionNetwork_color->input.setBlocking(false); 
        mobilenetSpatialDetectionNetwork_color->setBoundingBoxScaleFactor(0.5f); 
        mobilenetSpatialDetectionNetwork_color->setDepthLowerThreshold(100); 
        mobilenetSpatialDetectionNetwork_color->setDepthUpperThreshold(5000); 

        // Link color camera with neural network
        colorCam->preview.link(mobilenetSpatialDetectionNetwork_color->input);
        mobilenetSpatialDetectionNetwork_color->passthrough.link(xoutColor->input);

        mobilenetSpatialDetectionNetwork_color->out.link(nnColorOutDetections->input);
        mobilenetSpatialDetectionNetwork_color->boundingBoxMapping.link(xoutBoundingBoxDepthMapping->input);

        // Link depth from the StereoDepth node
        stereo->depth.link(mobilenetSpatialDetectionNetwork_color->inputDepth);
        mobilenetSpatialDetectionNetwork_color->passthroughDepth.link(xoutDepth->input);


    }

    if(use_list.use_stereo_detections){

        std::string nnBlobPath = "default";
        if (ros::param::has("/nnBlobPath")) {
            ros::param::get("/nnBlobPath", nnBlobPath);
        }

        // NN for the right camera
        mobilenetDetectionNetwork_right = pipeline_.create<dai::node::MobileNetDetectionNetwork>();
        mobilenetDetectionNetwork_right->setConfidenceThreshold(0.5f);
        mobilenetDetectionNetwork_right->setBlobPath(nnBlobPath);
        mobilenetDetectionNetwork_right->setNumInferenceThreads(2);
        mobilenetDetectionNetwork_right->input.setBlocking(false);

        // NN for hte left camera
        mobilenetDetectionNetwork_left = pipeline_.create<dai::node::MobileNetDetectionNetwork>();
        mobilenetDetectionNetwork_left->setConfidenceThreshold(0.5f);
        mobilenetDetectionNetwork_left->setBlobPath(nnBlobPath);
        mobilenetDetectionNetwork_left->setNumInferenceThreads(2);
        mobilenetDetectionNetwork_left->input.setBlocking(false);

        int NN_input_width = 300;
        if (ros::param::has("/NN_input_width")) {
            ros::param::get("/NN_input_width", NN_input_width);
        }
        int NN_input_high = 300;
        if (ros::param::has("/NN_input_high")) {
            ros::param::get("/NN_input_high", NN_input_high);
        }
        colorCam->setPreviewSize(NN_input_width, NN_input_high);

        // ImageManip for the right camera image
        imageManip_right = pipeline_.create<dai::node::ImageManip>();
        imageManip_right->initialConfig.setResize(NN_input_width, NN_input_high);
        imageManip_right->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888p); // The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)

        // ImageManip for the left camera image
        imageManip_left = pipeline_.create<dai::node::ImageManip>();
        imageManip_left->initialConfig.setResize(NN_input_width, NN_input_high);
        imageManip_left->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888p); // The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)

        // Set stream names for connection with PC
        nnRightOut = pipeline_.create<dai::node::XLinkOut>();
        nnRightOut->setStreamName("detections_right");

        nnLeftOut = pipeline_.create<dai::node::XLinkOut>();
        nnLeftOut->setStreamName("detections_left");

        xoutManipRight = pipeline_.create<dai::node::XLinkOut>();
        xoutManipRight->setStreamName("manip_right");

        xoutManipLeft = pipeline_.create<dai::node::XLinkOut>();
        xoutManipLeft->setStreamName("manip_left");

        // Connections among nodes
        stereo->rectifiedLeft.link(imageManip_left->inputImage);
        stereo->rectifiedRight.link(imageManip_right->inputImage);
        imageManip_right->out.link(mobilenetDetectionNetwork_right->input);
        imageManip_left->out.link(mobilenetDetectionNetwork_left->input);
        mobilenetDetectionNetwork_right->passthrough.link(xoutManipRight->input);
        //imageManip_right->out.link(xoutManipRight->input);
        mobilenetDetectionNetwork_right->out.link(nnRightOut->input);
        mobilenetDetectionNetwork_left->passthrough.link(xoutManipLeft->input);
        mobilenetDetectionNetwork_left->out.link(nnLeftOut->input);
        //imageManip_left->out.link(xoutManipLeft->input);

    }


    // CONNECT TO DEVICE
    dev_ = std::make_unique<dai::Device>(pipeline_);
    // dev_->startPipeline(); // DEPRECATED

    int counter = 0;
    int queueSize = 4;

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
    if(use_list.use_color){
        streams_queue.push_back(dev_->getOutputQueue("color", queueSize, false));
        queue_index.inx_color = counter; counter++;
    }
    if(use_list.use_imu){
        streams_queue.push_back(dev_->getOutputQueue("imu", queueSize, false));
        queue_index.inx_imu = counter; counter++;
    }
    if(use_list.use_color_detections){
        streams_queue.push_back(dev_->getOutputQueue("detections_color", queueSize, false));
        queue_index.inx_detections_color = counter; counter++; 
        streams_queue.push_back(dev_->getOutputQueue("boundingBoxDepthMapping", queueSize, false));
        queue_index.inx_bbDepthMapping = counter; counter++;
    }
    if(use_list.use_stereo_detections){
        streams_queue.push_back(dev_->getOutputQueue("detections_right", queueSize, false));
        queue_index.inx_detections_right = counter; counter++; 
        streams_queue.push_back(dev_->getOutputQueue("detections_left", queueSize, false));
        queue_index.inx_detections_left = counter; counter++;
        streams_queue.push_back(dev_->getOutputQueue("manip_right", queueSize, false));
        queue_index.inx_imgManip_right = counter; counter++; 
        streams_queue.push_back(dev_->getOutputQueue("manip_left", queueSize, false));
        queue_index.inx_imgManip_left = counter; counter++;
    }

    std::cout << "Pipeline initialized correctly" << std::endl;
}

void OakDPipeline::stop(){
    dev_->close();
}
