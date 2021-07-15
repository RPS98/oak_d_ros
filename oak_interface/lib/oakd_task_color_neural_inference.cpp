#include <oak_interface/oakd_task_color_neural_inference.hpp>

const std::vector<std::string> OakDTaskColorNeuralInference::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"}; 

void OakDTaskColorNeuralInference::start(ros::NodeHandle& nh){
    std::cout << "Starting OakDTaskDetections" << std::endl;

    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";

    // ROS Publisher
    detections_pub = nh.advertise<oak_interface::BoundingBoxes>("/detections", 1);
    image_detections_pub = nh.advertise<sensor_msgs::Image>("/rgb_image_detections", 1);
    // Initialization of some atributes
    startTime = std::chrono::steady_clock::now();
    counter_ = 0;
    fps_ = 0;
    color_ = cv::Scalar(255, 255, 255);
};

void OakDTaskColorNeuralInference::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index, std_msgs::Header header){
    using namespace std::chrono;
    imgFrame = streams_queue[queue_index.inx_color]->get<dai::ImgFrame>();
    det = streams_queue[queue_index.inx_detections_color]->get<dai::SpatialImgDetections>();

    auto dets = det->detections;

    counter_++;
    auto currentTime = steady_clock::now();
    auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
    if(elapsed > seconds(1)) {
        fps_ = counter_ / elapsed.count();
        counter_ = 0;
        startTime = currentTime;
    }

    cv::Mat frame = OakDUtils::getCvFrame(imgFrame);

    for(const auto& d : dets) {

        int labelIndex = d.label;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < label_map.size()) {
            labelStr = label_map[labelIndex];
        }
        // Process just the objects that will be sent
        if (labelStr == "diningtable" || labelStr == "tvmonitor" || labelStr == "bottle" || labelStr == "chair"){
            // Get rid of outliers
            if ((float)d.spatialCoordinates.z/1000.0 > 0.7 && (float)d.spatialCoordinates.z/1000.0 < 3.0){

                int x1 = d.xmin * frame.cols;
                int y1 = d.ymin * frame.rows;
                int x2 = d.xmax * frame.cols;
                int y2 = d.ymax * frame.rows;


                cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                std::stringstream confStr;
                confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
                cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

                std::stringstream depthX;
                depthX << "X: " << std::fixed << std::setprecision(3) << (float)d.spatialCoordinates.x/1000 << " m";
                cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                std::stringstream depthY;
                depthY << "Y: " << std::fixed << std::setprecision(3) << (float)d.spatialCoordinates.y/1000 << " m";
                cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                std::stringstream depthZ;
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
    std::stringstream fpsStr;
    fpsStr << std::fixed << std::setprecision(2) << fps_;
    cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color_);

    // From cv::Mat to cv_bridge::CvImage 
    cv_bridge::CvImage frame_cv_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8, frame);
    // From cv_bridge::CvImage to sensor_msgs::Image
    frame_cv_bridge.toImageMsg(frame_msg);
    // Publishing the image with detections in the topic "/rgb_image_detections"
    image_detections_pub.publish(frame_msg);
    // Publishing the info of detections in the topic "/detections"
    detections_pub.publish(msg); 
    msg.bounding_boxes.clear();
};

void OakDTaskColorNeuralInference::stop(){
    detections_pub.shutdown();
    image_detections_pub.shutdown();
};
