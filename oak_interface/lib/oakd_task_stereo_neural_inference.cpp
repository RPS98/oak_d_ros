#include <oak_interface/oakd_task_stereo_neural_inference.hpp>
#include <cmath>

#define f 196 // focal distance 196 mm
#define B 75  // distance between left and right monocameras
#define CX 150
#define pixel_to_mm 0.003 // 1 pixel = 3 um = 0.003 mm
#define pixel 3.7795752 // 1 mm

const std::vector<std::string> OakDTaskStereoNeuralInference::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"}; 

void OakDTaskStereoNeuralInference::start(ros::NodeHandle& nh){
    
    std::cout << "Starting OakDTaskStereoNeuralInference" << std::endl;

    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";

    int bad_params = 0;

    bad_params += !nh.getParam("camera_name", deviceName);
    bad_params += !nh.getParam("camera_param_uri", camera_param_uri);
    /*
    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    } */
    
    // ROS Publisher
    detections_pub = nh.advertise<oak_interface::BoundingBoxes>("/detections", 1);
    image_right_det_pub = nh.advertise<sensor_msgs::Image>("/detections_image_right", 1);
    image_left_det_pub = nh.advertise<sensor_msgs::Image>("/detections_image_left", 1);

    // Initialization of some atributes
    startTime = std::chrono::steady_clock::now();
    counter_ = 0;
    fps_ = 0;
    color_ = cv::Scalar(255, 255, 255);
};

void OakDTaskStereoNeuralInference::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index){
    
    using namespace std::chrono;
    //std::cout << "Running OakDTaskStereoNeuralInference" << std::endl;

    right = streams_queue[queue_index.inx_imgManip_right]->get<dai::ImgFrame>();
    left = streams_queue[queue_index.inx_imgManip_left]->get<dai::ImgFrame>();
    det_right = streams_queue[queue_index.inx_detections_right]->get<dai::ImgDetections>();
    det_left = streams_queue[queue_index.inx_detections_left]->get<dai::ImgDetections>();
    
    auto dets_right = det_right->detections;
    auto dets_left = det_left->detections;

    counter_++;
    auto currentTime = steady_clock::now();
    auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
    if(elapsed > seconds(1)) {
        fps_ = counter_ / elapsed.count();
        counter_ = 0;
        startTime = currentTime;
    }

    cv::Mat frame_right = OakDUtils::getCvFrame(right);

    for(const auto& d : dets_right) {
        // x1, x2, x3 , x4 -> coordenadas laterales en pixeles de las 2 esquinas de la bbox
        int x1 = d.xmin * frame_right.cols;
        int y1 = d.ymin * frame_right.rows;
        int x2 = d.xmax * frame_right.cols;
        int y2 = d.ymax * frame_right.rows;

        int labelIndex = d.label;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < label_map.size()) {
            labelStr = label_map[labelIndex];
        }
        cv::putText(frame_right, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
        cv::putText(frame_right, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

        cv::rectangle(frame_right, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

        dets.type = (std::string)labelStr;
        dets.prob = (float)d.confidence;
        dets.x1 = (int)x1;
        dets.y1 = (int)y1;
        dets.x2 = (int)x2;
        dets.y2 = (int)y2;
        dets.cx = (float)(x1 + x2)/2;
        dets.cy = (float)(y1 + y2)/2;
        dets.area = abs(x2-x1)*abs(y2-y1);

        detsRight.push_back(dets);

        /*bbox.Class = (std::string)labelStr;
        bbox.probability = (float)d.confidence;
        bbox.xmin = (int)x1;
        bbox.ymin = (int)y1;
        bbox.xmax = (int)x2;
        bbox.ymax = (int)y2;
        bbox.depth = (float)0.0;

        right_detections.bounding_boxes.push_back(bbox);*/

    }

    cv::Mat frame_left = OakDUtils::getCvFrame(left);

    for(const auto& d : dets_left) {
        int x1 = d.xmin * frame_left.cols;
        int y1 = d.ymin * frame_left.rows;
        int x2 = d.xmax * frame_left.cols;
        int y2 = d.ymax * frame_left.rows;

        int labelIndex = d.label;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < label_map.size()) {
            labelStr = label_map[labelIndex];
        }
        cv::putText(frame_left, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
        cv::putText(frame_left, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

        cv::rectangle(frame_left, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

        dets.type = (std::string)labelStr;
        dets.prob = (float)d.confidence;
        dets.x1 = (int)x1;
        dets.y1 = (int)y1;
        dets.x2 = (int)x2;
        dets.y2 = (int)y2;
        dets.cx = (float)((x2 + x1)/2);
        dets.cy = (float)((y2 + y1)/2);
        dets.area = abs(x2-x1)*abs(y2-y1);
        
        detsLeft.push_back(dets);

        // bbox.Class = (std::string)labelStr;
        // bbox.probability = (float)d.confidence;
        // bbox.xmin = (int)x1;
        // bbox.ymin = (int)y1;
        // bbox.xmax = (int)x2;
        // bbox.ymax = (int)y2;
        // bbox.depth = (float)0.0;

        // left_detections.bounding_boxes.push_back(bbox);

    }

    // Compute the depth of the detections which are in both cameras
    float dist_threshold = 5.0;
    float area_threshold = 10.0;

    for(const auto& dR : detsRight){
        for(const auto& dL : detsLeft){
            if(dR.type == dL.type){
                float distance = sqrt(pow(dL.cx-dR.cx,2)+pow(dL.cy-dR.cy,2));
                if((distance <= dist_threshold) && (abs(dR.area-dL.area) <= area_threshold)){
                    bbox.Class = dR.type;
                    bbox.probability = dR.prob;
                    bbox.xmin = (int)dR.x1;
                    bbox.ymin = (int)dR.y1;
                    bbox.xmax = (int)dR.x2;
                    bbox.ymax = (int)dR.y2;

                    // Computing the depth
                    std::cout << ((dR.x1-CX)-(dL.x1-CX))*pixel_to_mm << std::endl;
                    bbox.depth = 1;//(f*B)/(dR.x1-dL.x1); // Stereo model

                    msg.bounding_boxes.push_back(bbox);
                }
            }   
        }
    }
    
    
    std::stringstream fpsStr;
    fpsStr << std::fixed << std::setprecision(2) << fps_;
    cv::putText(frame_right, fpsStr.str(), cv::Point(2, right->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color_);
    cv::putText(frame_left, fpsStr.str(), cv::Point(2, left->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color_);
    //cv::imshow("depth", depthFrameColor);
    //cv::imshow("preview", frame);

    // From cv::Mat to cv_bridge::CvImage 
    std_msgs::Header header;
    cv_bridge::CvImage frame_cv_bridge_right = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8, frame_right);
    cv_bridge::CvImage frame_cv_bridge_left = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8, frame_left);

    // From cv_bridge::CvImage to sensor_msgs::Image
    frame_cv_bridge_right.toImageMsg(image_right_with_det);
    frame_cv_bridge_left.toImageMsg(image_left_with_det);

    // Publishing the both right and left images with detections in the topics
    image_right_det_pub.publish(image_right_with_det);
    image_left_det_pub.publish(image_left_with_det);

    // Publishing the info of detections in the topic "/detections"
    detections_pub.publish(msg);
    msg.bounding_boxes.clear();
    //right_detections.bounding_boxes.clear();
    //left_detections.bounding_boxes.clear();

};

void OakDTaskStereoNeuralInference::stop(){
    detections_pub.shutdown();
    image_right_det_pub.shutdown();
    image_left_det_pub.shutdown();
};