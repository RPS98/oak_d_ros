#include <oak_interface/oakd_task_color_neural_inference.hpp>

const std::vector<std::string> OakDTaskColorNeuralInference::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"}; 

void OakDTaskColorNeuralInference::start(ros::NodeHandle& nh){

    // ROS Publisher
    detections_pub = nh.advertise<oak_interface::BoundingBoxes>("/detections/bounding_boxes", 1);
    image_detections_pub = nh.advertise<sensor_msgs::Image>("/detections/image", 1);
    // Initialization of some atributes
    startTime = std::chrono::steady_clock::now();
    counter_ = 0;
    fps_ = 0;
    color_ = cv::Scalar(255, 255, 255);


    // COLOR IMAGE PUBLISHER
    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";

    if (ros::param::has("/camera_name")) {
        ros::param::get("/camera_name", deviceName);
    }
    if (ros::param::has("/camera_param_uri")) {
        ros::param::get("/camera_param_uri", camera_param_uri);
    }
    if (ros::param::has("/color_interleaved")) {
        ros::param::get("/color_interleaved", use_interleaved);
    }
    if (ros::param::has("/publish_color")) {
        ros::param::get("/publish_color", publish_color);
    }
    std::string color_uri = camera_param_uri + "/" + "color.yaml";
    
    // ROS
    // Publishers
    color_pub = nh.advertise<sensor_msgs::Image>("color/image", 1);
    color_info_pub = nh.advertise<sensor_msgs::CameraInfo>("color/info", 1);

    color_camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, deviceName, color_uri);
    color_CameraInfo = color_camInfoManager->getCameraInfo();
};

void OakDTaskColorNeuralInference::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index, std_msgs::Header header){
                
    using namespace std::chrono;
    color_frame = streams_queue[queue_index.inx_color]->tryGet<dai::ImgFrame>();

    if(!(color_frame == nullptr)){
        
        if(publish_color){
            // Send image
            OakDTaskColorNeuralInference::get_RosMsg(color_frame,color_image_msg, header);
            color_image_msg.header.stamp = header.stamp;
            color_pub.publish(color_image_msg);

            // Send info
            color_CameraInfo.header.seq = color_frame->getSequenceNum();
            color_CameraInfo.header.stamp = ros::Time::now();
            color_info_pub.publish(color_CameraInfo);
        }
        
        det = streams_queue[queue_index.inx_detections_color]->get<dai::SpatialImgDetections>();

        counter_++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps_ = counter_ / elapsed.count();
            counter_ = 0;
            startTime = currentTime;
        }

        cv::Mat frame = OakDUtils::getCvFrame(color_frame);

        for(const auto& d : det->detections) {
            labelIndex = d.label;
            labelStr = std::to_string(labelIndex);
            if(labelIndex < label_map.size()) {
                labelStr = label_map[labelIndex];
            }
            // Process just the objects that will be sent
            if (labelStr == "diningtable" || labelStr == "tvmonitor" || labelStr == "bottle" || labelStr == "chair"){
                // Get rid of outliers
                if ((float)d.spatialCoordinates.z/1000.0 > 0.7 && (float)d.spatialCoordinates.z/1000.0 < 3.0){

                    x1 = d.xmin * frame.cols;
                    y1 = d.ymin * frame.rows;
                    x2 = d.xmax * frame.cols;
                    y2 = d.ymax * frame.rows;


                    cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

                    confStr.str("");
                    confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
                    cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

                    depthX.str("");
                    depthX << "X: " << std::fixed << std::setprecision(3) << (float)d.spatialCoordinates.x/1000 << " m";
                    cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                    
                    depthY.str("");
                    depthY << "Y: " << std::fixed << std::setprecision(3) << (float)d.spatialCoordinates.y/1000 << " m";
                    cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                    
                    depthZ.str("");
                    depthZ << "Z: " << std::fixed << std::setprecision(3) << (float)d.spatialCoordinates.z/1000 << " m";
                    cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

                    cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

                    bbox.Class = (std::string)labelStr;
                    bbox.probability = (float)d.confidence;
                    bbox.xmin = (int)x1;
                    bbox.ymin = (int)y1;
                    bbox.xmax = (int)x2;
                    bbox.ymax = (int)y2;
                    bbox.depth = (float)d.spatialCoordinates.z/1000;
                    bbox.x_centroid = (float)d.spatialCoordinates.x/1000;
                    bbox.y_centroid = (float)d.spatialCoordinates.y/1000;
                    msg.header.stamp = header.stamp;

                    msg.bounding_boxes.push_back(bbox);
                }
            }
        }

        // Print fps
        fpsStr.str("");
        fpsStr << std::fixed << std::setprecision(2) << fps_;
        cv::putText(frame, fpsStr.str(), cv::Point(2, color_frame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color_);

        // From cv::Mat to cv_bridge::CvImage 
        cv_bridge::CvImage frame_cv_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8, frame);
        // From cv_bridge::CvImage to sensor_msgs::Image
        frame_cv_bridge.toImageMsg(frame_msg);
        // Publishing the image with detections in the topic "/detections/image"
        frame_msg.header.stamp = header.stamp;
        image_detections_pub.publish(frame_msg);
        // Publishing the info of detections in the topic "/detections/bounding_boxes"
        detections_pub.publish(msg); 
        msg.bounding_boxes.clear();
        
    }
    
};

void OakDTaskColorNeuralInference::stop(){
    detections_pub.shutdown();
    image_detections_pub.shutdown();
};

void OakDTaskColorNeuralInference::get_RosMsg(std::shared_ptr<dai::ImgFrame> color_frame,
                                              sensor_msgs::Image &color_image_msg, 
                                              std_msgs::Header header){

    if(use_interleaved){
        OakDUtils::getRosMsg(color_frame, color_image_msg, use_interleaved);
    } else {
        cv::Mat frame_cv2 = OakDUtils::getCvFrame(color_frame);
         cv_bridge::CvImage frame_cv_bridge = cv_bridge::CvImage(header ,sensor_msgs::image_encodings::BGR8, frame_cv2);
         frame_cv_bridge.toImageMsg(color_image_msg);
    }
}