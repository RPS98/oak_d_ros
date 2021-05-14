#include <oak_interface/oakd_task_detections.hpp>


const std::vector<std::string> OakDTaskDetections::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"}; 

void OakDTaskDetections::start(ros::NodeHandle& nh){
    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";

    if (ros::param::has("/camera_name")) {
        ros::param::get("/camera_name", deviceName);
    }
    if (ros::param::has("/camera_param_uri")) {
        ros::param::get("/camera_param_uri", camera_param_uri);
    }
    
    // ROS
    // Publishers
    detections_pub = nh.advertise<oak_interface::BoundingBoxes>("/detections", 1);

    // Initialization of some atributes
    startTime = std::chrono::steady_clock::now();
    counter_ = 0;
    fps_ = 0;
    color_ = cv::Scalar(255, 255, 255);
};

void OakDTaskDetections::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index){
    
    imgFrame = streams_queue[queue_index.inx_rgb]->tryGet<dai::ImgFrame>();
    det = streams_queue[queue_index.inx_detections]->tryGet<dai::SpatialImgDetections>();
    depth = streams_queue[queue_index.inx_depth]->tryGet<dai::ImgFrame>();

    OakDUtils utils;

    auto dets = det->detections;
    cv::Mat depthFrame = utils.getFrame(depth,false);
    cv::Mat depthFrameColor;
    cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
    cv::equalizeHist(depthFrameColor, depthFrameColor);
    cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

    if(!dets.empty()) {
        boundingBoxMapping = streams_queue[queue_index.inx_bbDepthMapping]->tryGet<dai::SpatialLocationCalculatorConfig>();
        auto roiDatas = boundingBoxMapping->getConfigData();

        for(auto roiData : roiDatas) {
            auto roi = roiData.roi;
            roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
            auto topLeft = roi.topLeft();
            auto bottomRight = roi.bottomRight();
            auto xmin = (int)topLeft.x;
            auto ymin = (int)topLeft.y;
            auto xmax = (int)bottomRight.x;
            auto ymax = (int)bottomRight.y;

            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color_, cv::FONT_HERSHEY_SIMPLEX);
        }
    }
    counter_++;
    auto currentTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - startTime);
    if(elapsed > std::chrono::seconds(1)) {
        fps_ = counter_ / elapsed.count();
        counter_ = 0;
        startTime = currentTime;
    }

    cv::Mat frame = utils.getCvFrame(imgFrame);

    for(const auto& d : dets) {
        int x1 = d.xmin * frame.cols;
        int y1 = d.ymin * frame.rows;
        int x2 = d.xmax * frame.cols;
        int y2 = d.ymax * frame.rows;

        int labelIndex = d.label;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < label_map.size()) {
            labelStr = label_map[labelIndex];
        }
        cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
        cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

        std::stringstream depthX;
        depthX << "X: " << (int)d.spatialCoordinates.x << " mm";
        cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream depthY;
        depthY << "Y: " << (int)d.spatialCoordinates.y << " mm";
        cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream depthZ;
        depthZ << "Z: " << (int)d.spatialCoordinates.z << " mm";
        cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

        cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

        bbox.Class = (std::string)labelStr;
        bbox.probability = (float)d.confidence;
        bbox.xmin = (int)x1;
        bbox.ymin = (int)y1;
        bbox.xmax = (int)x2;
        bbox.ymax = (int)y2;
        bbox.depth = (float)d.spatialCoordinates.z;

        msg.bounding_boxes.push_back(bbox);

    }

    std::stringstream fpsStr;
    fpsStr << std::fixed << std::setprecision(2) << fps_;
    cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color_);
    cv::imshow("depth", depthFrameColor);
    cv::imshow("preview", frame);

    // Publishing the info of detections in the topic 
    detections_pub.publish(msg);
    msg.bounding_boxes.clear();

};

void OakDTaskDetections::stop(){
    detections_pub.shutdown();
};