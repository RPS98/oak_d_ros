#include <oak_interface/oakd_task_stereo_neural_inference.hpp>
#include <cmath>

#define F_LEFT 872.2578152369078                // Focal length for left camera in pixels
#define F_RIGHT 875.2617250748619               // Focal length for right camera in pixels
#define FOCAL_LENGTH (F_LEFT + F_RIGHT)/2.0     // Focal length for both cameras in pixels
#define B 75                                    // Distance between left and right cameras in mm
#define CX_LEFT 644.2604132944807               // CX for left camera in pixels
#define CY_LEFT 370.415032403672                // CY for left camera in pixels                   
#define CX_RIGHT 641.3521672332931              // CX for right camera in pixels
#define CY_RIGHT 372.6642906480663              // CY for right camera in pixels
#define N_REDUCTIONS 2                          // Number of reductions for computing depth average

const std::vector<std::string> OakDTaskStereoNeuralInference::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"}; 

void OakDTaskStereoNeuralInference::start(ros::NodeHandle& nh){
    
    std::cout << "Starting OakDTaskStereoNeuralInference" << std::endl;

    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";
    
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

bool OakDTaskStereoNeuralInference::wayToSort(const detections& i, const detections& j){ 
    return i.cx < j.cx;
};

void OakDTaskStereoNeuralInference::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index, std_msgs::Header header){
    
    using namespace std::chrono;
    
    //std::cout << "Running OakDTaskStereoNeuralInference" << std::endl;

    rectified_left = streams_queue[queue_index.inx_rectified_left]->get<dai::ImgFrame>();   // rectified left image 1280x800
    rectified_right = streams_queue[queue_index.inx_rectified_right]->get<dai::ImgFrame>(); // rectified right image 1280x800
    right = streams_queue[queue_index.inx_imgManip_right]->get<dai::ImgFrame>();            // rectified right image 300x300
    left = streams_queue[queue_index.inx_imgManip_left]->get<dai::ImgFrame>();              // rectified left image 300x300
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
    cv::Mat frame_rectified_right = OakDUtils::getCvFrame(rectified_right); // In order to view rectified right image with its original size with detections

    // Test for knowing what setResize function actually performs
    /*cv::Rect crop_region(240,0,800,800);
    cv::Mat image_cropped=frame_rectified_right(crop_region);
    cv::imshow("right", frame_right);
    cv::imshow("rectified_right", image_cropped);
    cv::waitKey(1); */

    int bottle_counter_R = 0;
    int monitor_counter_R = 0;
    int table_counter_R = 0;
    int chair_counter_R = 0;

    for(const auto& d : dets_right) {

        int labelIndex = d.label;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < label_map.size()) {
            labelStr = label_map[labelIndex];
        }

        // Process just the objects that will be sent
        if (labelStr == "diningtable" || labelStr == "tvmonitor" || labelStr == "bottle" || labelStr == "chair"){
            // Lateral coordinates in pixels of the top left and bottom right corners of bbox
            int x1 = d.xmin * frame_right.cols;
            int y1 = d.ymin * frame_right.rows;
            int x2 = d.xmax * frame_right.cols;
            int y2 = d.ymax * frame_right.rows;

            // Bounding box lateral coordinates of 1280x800 images
            int x1R = (x1-150) * 800/300 + CX_RIGHT;
            int y1R = (y1-150) * 800/300 + CY_RIGHT;
            int x2R = (x2-150) * 800/300 + CX_RIGHT;
            int y2R = (y2-150) * 800/300 + CY_RIGHT;

            // Putting text in 1280x800 image and printing bbox
            // cv::putText(frame_rectified_right, labelStr, cv::Point(x1R + 10, y1R + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            // std::stringstream confStr_;
            // confStr_ << std::fixed << std::setprecision(2) << d.confidence * 100;
            // cv::putText(frame_rectified_right, confStr_.str(), cv::Point(x1R + 10, y1R + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            // cv::rectangle(frame_rectified_right, cv::Rect(cv::Point(x1R, y1R), cv::Point(x2R, y2R)), color_, cv::FONT_HERSHEY_SIMPLEX);

            // Putting text in 300x300 image and printing bbox
            cv::putText(frame_right, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
            cv::putText(frame_right, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            cv::rectangle(frame_right, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

            dets.type = (std::string)labelStr;
            dets.prob = (float)d.confidence;
            dets.x1 = (int)(x1R - CX_RIGHT);
            dets.y1 = (int)(y1R - CY_RIGHT);
            dets.x2 = (int)(x2R - CX_RIGHT);
            dets.y2 = (int)(y2R - CY_RIGHT);
            dets.cx = (float)(dets.x1 + dets.x2)/2;
            dets.cy = (float)(dets.y1 + dets.y2)/2;
            dets.area = abs(dets.x2-dets.x1)*abs(dets.y2-dets.y1);
            dets.aspect_ratio = abs((dets.y2-dets.y1)/(dets.x2-dets.x1));
            if (dets.type == "bottle"){
                bottle_counter_R++;
                detsRight.push_back(dets);
            } 
            else if (dets.type == "chair"){
                chair_counter_R++;
                detsRight.push_back(dets);
            } 
            else if (dets.type == "tvmonitor") {
                monitor_counter_R++;
                detsRight.push_back(dets);
            }
            else if (dets.type == "diningtable") {
                table_counter_R++;
                detsRight.push_back(dets);
            } 
        }

    }

    cv::Mat frame_left = OakDUtils::getCvFrame(left);
    cv::Mat frame_rectified_left = OakDUtils::getCvFrame(rectified_left); // In order to view rectified left image with its original size with detections

    int bottle_counter_L = 0;
    int monitor_counter_L = 0;
    int table_counter_L = 0;
    int chair_counter_L = 0;

    for(const auto& d : dets_left) {

        int labelIndex = d.label;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < label_map.size()) {
            labelStr = label_map[labelIndex];
        }

        // Process just the objects that will be sent
        if (labelStr == "diningtable" || labelStr == "tvmonitor" || labelStr == "bottle" || labelStr == "chair"){

            // Lateral coordinates in pixels of the top left and bottom right corners of bbox
            int x1 = d.xmin * frame_left.cols;
            int y1 = d.ymin * frame_left.rows;
            int x2 = d.xmax * frame_left.cols;
            int y2 = d.ymax * frame_left.rows;

            // Bounding box lateral coordinates of 1280x800 images
            int x1L = (x1-150) * 800/300 + CX_LEFT;
            int y1L = (y1-150) * 800/300 + CY_LEFT;
            int x2L = (x2-150) * 800/300 + CX_LEFT;
            int y2L = (y2-150) * 800/300 + CY_LEFT;
            
            // Putting text in 1280x800 image and printing bbox
            // cv::putText(frame_rectified_left, labelStr, cv::Point(x1L + 10, y1L + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            // std::stringstream confStr_;
            // confStr_ << std::fixed << std::setprecision(2) << d.confidence * 100;
            // cv::putText(frame_rectified_left, confStr_.str(), cv::Point(x1L + 10, y1L + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            // cv::rectangle(frame_rectified_left, cv::Rect(cv::Point(x1L, y1L), cv::Point(x2L, y2L)), color_, cv::FONT_HERSHEY_SIMPLEX);

            // Putting text in 300x300 image and printing bbox
            cv::putText(frame_left, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
            cv::putText(frame_left, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            cv::rectangle(frame_left, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

            dets.type = (std::string)labelStr;
            dets.prob = (float)d.confidence;
            dets.x1 = (int)x1L - CX_LEFT;
            dets.y1 = (int)y1L - CY_LEFT;
            dets.x2 = (int)x2L - CX_LEFT;
            dets.y2 = (int)y2L - CY_LEFT;
            dets.cx = (float)((dets.x2 + dets.x1)/2);
            dets.cy = (float)((dets.y2 + dets.y1)/2);
            dets.area = abs(dets.x2-dets.x1)*abs(dets.y2-dets.y1);
            dets.aspect_ratio = abs((dets.y2-dets.y1)/(dets.x2-dets.x1));

            if (dets.type == "bottle"){
                bottle_counter_L++;
                detsLeft.push_back(dets);
            } 
            else if (dets.type == "chair"){
                chair_counter_L++;
                detsLeft.push_back(dets);
            } 
            else if (dets.type == "tvmonitor") {
                monitor_counter_L++;
                detsLeft.push_back(dets);
            }
            else if (dets.type == "diningtable") {
                table_counter_L++;
                detsLeft.push_back(dets);
            } 
        }

    }

    //float dist_threshold = 20000000.0;
    float area_threshold = 0.4; // From 0 to 1
    float depth = 0.0, depth_sum = 0.0, depth_avg = 0.0;
    float reduction = 0.0;
    float disparity;
    int disparity_null_counter = 0;
    int n_points;
    int y_threshold = 10;  // In pixels
    float ratio_threshold = 0.4; // From 0 to 1
    int i = 0;

    // First filter: erase objects whose counters of each image don't match
    if (bottle_counter_R != bottle_counter_L){
        i = 0;
        for(const auto& dR : detsRight){
            if (dR.type == "bottle") detsRight.erase(detsRight.begin() + i);
            else i++;
        }
        i = 0;
        for(const auto& dL : detsLeft){
            if (dL.type == "bottle") detsLeft.erase(detsLeft.begin() + i);
            else i++;
        }
    }

    if (chair_counter_R != chair_counter_L){
        i = 0;
        for(const auto& dR : detsRight){
            if (dR.type == "chair") detsRight.erase(detsRight.begin() + i);
            else i++;
        }
        i = 0;
        for(const auto& dL : detsLeft){
            if (dL.type == "chair") detsLeft.erase(detsLeft.begin() + i);
            else i++;
        }
    }

    if (table_counter_R != table_counter_L){
        i = 0;
        for(const auto& dR : detsRight){
            if (dR.type == "diningtable") detsRight.erase(detsRight.begin() + i);
            else i++;
        }
        i = 0;
        for(const auto& dL : detsLeft){
            if (dL.type == "diningtable") detsLeft.erase(detsLeft.begin() + i);
            else i++;
        }
    }

    if (monitor_counter_R != monitor_counter_L) {
        i = 0;
        for(const auto& dR : detsRight){
            if (dR.type == "tvmonitor") detsRight.erase(detsRight.begin() + i);
            else i++;
        }
        i = 0;
        for(const auto& dL : detsLeft){
            if (dL.type == "tvmonitor") detsLeft.erase(detsLeft.begin() + i);
            else i++;
        }
    }

    // Sorting bounding boxes of each images depending on their X center coordinates
    std::sort(detsRight.begin(), detsRight.end(), OakDTaskStereoNeuralInference::wayToSort);
    std::sort(detsLeft.begin(), detsLeft.end(), OakDTaskStereoNeuralInference::wayToSort);

    // Corresponding detected objects in both images to compute their spatial coordinates
    for(const auto& dR : detsRight){
        i = 0;
        for(const auto& dL : detsLeft){
            // Second filter: compare objects type
            if(dR.type == dL.type){
                float distance = sqrt(pow(dL.cx-dR.cx,2)+pow(dL.cy-dR.cy,2));
                float area_error = abs(dR.area-dL.area)/(dR.area+dL.area);
                float ratio_error = abs(dR.aspect_ratio-dL.aspect_ratio)/(dR.aspect_ratio+dL.aspect_ratio);

                // Third filter: Bbox matching by comparing areas and Y center coordinates
                if((area_error <= area_threshold) && (ratio_error <= ratio_threshold) && (dL.cy <= (dR.cy + y_threshold)) && (dL.cy >= (dR.cy - y_threshold))){
                    // Computing the depth
                    // Depth calculation by reducing the size of the bbox and computing the average depth value 
                    for (int i=0;i<=N_REDUCTIONS;i++){
                        disparity = (dR.x1 + reduction*dR.x1 - CX_RIGHT/2)-(dL.x1 + reduction*dL.x1 - CX_LEFT/2);
                        if (disparity <= 0.001) disparity_null_counter += 1;
                        else {
                            depth = (FOCAL_LENGTH*B)/disparity;
                            depth_sum = depth_sum + depth;
                        }

                        disparity = (dR.x2 - reduction*dR.x2 - CX_RIGHT/2)-(dL.x2 - reduction*dL.x2 - CX_LEFT/2);
                        if (disparity <= 0.001) disparity_null_counter += 1;
                        else {
                            depth = (FOCAL_LENGTH*B)/disparity;
                            depth_sum = depth_sum + depth;
                        }
                        reduction += 0.05;
                    }
                    
                    // Average of calculated depths when disparity isn't always 0
                    if (disparity_null_counter > 0 && depth_sum == 0) 
                        std::cout<<"WARNING: Disparity null"<<std::endl;
                    else{
                        n_points = (N_REDUCTIONS+1)*2-disparity_null_counter;
                        depth_avg = depth_sum/n_points;
                    }
                    // Reset counters for the next detection
                    disparity_null_counter = 0;
                    depth_sum = 0;
                    reduction = 0;

                    // Depth centroid calculation 
                    float Z_centroid = depth_avg; // mm

                    // Get rid of outliers
                    if (Z_centroid/1000.0 > 0.2 && Z_centroid/1000.0 < 3.0){
                        
                        // Spatial centroid coordinates (the reference system is placed on the left monocamera)
                        float X_centroid = B/2 + (Z_centroid*dL.cx)/FOCAL_LENGTH; // mm 
                        float Y_centroid = (Z_centroid*dL.cy)/FOCAL_LENGTH; // mm

                        // Print spatial X inside the bbox
                        std::stringstream XStr;
                        XStr << std::fixed << std::setprecision(3) << "X: " << X_centroid/1000 << " m";;
                        cv::putText(frame_right, XStr.str(), cv::Point(dR.x1*300/800 + 150 + 10, dR.y1*300/800 + 150 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                        cv::putText(frame_left, XStr.str(), cv::Point(dL.x1*300/800 + 150 + 10, dL.y1*300/800 + 150 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);  

                        // Print spatial Y inside the bbox
                        std::stringstream YStr;
                        YStr << std::fixed << std::setprecision(3) << "Y: "<< Y_centroid/1000 << " m";;
                        cv::putText(frame_right, YStr.str(), cv::Point(dR.x1*300/800 + 150 + 10, dR.y1*300/800 + 150 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                        cv::putText(frame_left, YStr.str(), cv::Point(dL.x1*300/800 + 150 + 10, dL.y1*300/800 + 150 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

                        // Print spatial Z inside the bbox
                        std::stringstream ZStr;
                        ZStr << std::fixed << std::setprecision(3) << "Z: " << Z_centroid/1000 << " m";;
                        cv::putText(frame_right, ZStr.str(), cv::Point(dR.x1*300/800 + 150 + 10, dR.y1*300/800 + 150 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                        cv::putText(frame_left, ZStr.str(), cv::Point(dL.x1*300/800 + 150 + 10, dL.y1*300/800 + 150 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                        
                        // ROS message for bounding boxes
                        bbox.Class = dR.type;
                        bbox.probability = dR.prob;
                        bbox.xmin = (int)dR.x1;
                        bbox.ymin = (int)dR.y1;
                        bbox.xmax = (int)dR.x2;
                        bbox.ymax = (int)dR.y2;
                        bbox.depth = Z_centroid/1000;        // m
                        bbox.x_centroid = X_centroid/1000;   // m
                        bbox.y_centroid = Y_centroid/1000;   // m
                        msg.bounding_boxes.push_back(bbox);
                        msg.header.stamp = header.stamp;
                    }

                    // Erase matched detected object
                    detsLeft.erase(detsLeft.begin() + i);
                    break;
                }
            }
            i++;  
        }
    }

    // Visualize 1280x800 rectified images with openCV
    //imshow("rectified_right", frame_rectified_right);
    //imshow("rectified_left", frame_rectified_left);
    //cv::waitKey(1);

    // From cv::Mat to cv_bridge::CvImage 
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
    ;
    // Cleaning vectors for the next iteration
    msg.bounding_boxes.clear();
    detsLeft.clear();
    detsRight.clear();

};

void OakDTaskStereoNeuralInference::stop(){
    detections_pub.shutdown();
    image_right_det_pub.shutdown();
    image_left_det_pub.shutdown();
};