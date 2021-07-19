#include <oak_interface/oakd_task_stereo_neural_inference.hpp>
#include <cmath>


const std::vector<std::string> OakDTaskStereoNeuralInference::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"}; 

void OakDTaskStereoNeuralInference::start(ros::NodeHandle& nh){
    
    std::cout << "Starting OakDTaskStereoNeuralInference" << std::endl;

    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";

    // Parameters
    if (ros::param::has("/mono_camera_resolution")) {
        ros::param::get("/mono_camera_resolution", mono_camera_resolution);
    }
    if(mono_camera_resolution == 720){
        resolution_width = 1280;
        resolution_height = 720;
    } else if(mono_camera_resolution == 800){
        resolution_width = 1280;
        resolution_height = 800;
    } else if(mono_camera_resolution == 400){
        resolution_width = 640;
        resolution_height = 400;
    } else {
        ROS_INFO_STREAM("Default mono camera resolution 720");
        resolution_width = 1280;
        resolution_height = 800;
    }

    if (ros::param::has("/NN_input_width")) {
        ros::param::get("/NN_input_width", NN_input_width);
    }
    if (ros::param::has("/NN_input_high")) {
        ros::param::get("/NN_input_high", NN_input_high);
    }

    // ROS Publisher
    detections_pub = nh.advertise<oak_interface::BoundingBoxes>("/detections/bounding_boxes", 1);
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

    det_right = streams_queue[queue_index.inx_detections_right]->get<dai::ImgDetections>();
    det_left = streams_queue[queue_index.inx_detections_left]->get<dai::ImgDetections>();
    
    if(!(det_right == nullptr) && !(det_left == nullptr)){
        rectified_left = streams_queue[queue_index.inx_rectified_left]->get<dai::ImgFrame>();   // rectified left image  with full resolution
        rectified_right = streams_queue[queue_index.inx_rectified_right]->get<dai::ImgFrame>(); // rectified right image with full resolution
        right = streams_queue[queue_index.inx_imgManip_right]->get<dai::ImgFrame>();            // rectified right image with net input resolution
        left = streams_queue[queue_index.inx_imgManip_left]->get<dai::ImgFrame>();              // rectified left image  with net input resolution

        cv::Mat frame_right = OakDUtils::getCvFrame(right);
        cv::Mat frame_rectified_right = OakDUtils::getCvFrame(rectified_right); // In order to view rectified right image with its original size with detections

        bottle_counter_R = 0;
        monitor_counter_R = 0;
        table_counter_R = 0;
        chair_counter_R = 0;

        counter_++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps_ = counter_ / elapsed.count();
            counter_ = 0;
            startTime = currentTime;
        }

        for(const auto& d : det_right->detections) {

            labelIndex = d.label;
            labelStr = std::to_string(d.label);
            if(labelIndex < label_map.size()) {
                labelStr = label_map[labelIndex];
            }

            // Process just the objects that will be sent
            if (labelStr == "diningtable" || labelStr == "tvmonitor" || labelStr == "bottle" || labelStr == "chair"){
                // Lateral coordinates in pixels of the top left and bottom right corners of bbox
                x1 = d.xmin * frame_right.cols;
                y1 = d.ymin * frame_right.rows;
                x2 = d.xmax * frame_right.cols;
                y2 = d.ymax * frame_right.rows;

                // Bounding box lateral coordinates of 1280x800 images
                x1R = (x1-NN_input_width/2) * resolution_width /NN_input_width + CX_RIGHT;
                y1R = (y1-NN_input_high/2)  * resolution_height/NN_input_high  + CY_RIGHT;
                x2R = (x2-NN_input_width/2) * resolution_width /NN_input_width + CX_RIGHT;
                y2R = (y2-NN_input_high/2)  * resolution_height/NN_input_high  + CY_RIGHT;

                // Putting text in net input image and printing bbox
                cv::putText(frame_right, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                confStr.str("");
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

        bottle_counter_L = 0;
        monitor_counter_L = 0;
        table_counter_L = 0;
        chair_counter_L = 0;

        for(const auto& d : det_left->detections) {

            labelIndex = d.label;
            labelStr = std::to_string(labelIndex);
            if(labelIndex < label_map.size()) {
                labelStr = label_map[labelIndex];
            }

            // Process just the objects that will be sent
            if (labelStr == "diningtable" || labelStr == "tvmonitor" || labelStr == "bottle" || labelStr == "chair"){

                // Lateral coordinates in pixels of the top left and bottom right corners of bbox
                x1 = d.xmin * frame_left.cols;
                y1 = d.ymin * frame_left.rows;
                x2 = d.xmax * frame_left.cols;
                y2 = d.ymax * frame_left.rows;

                // Bounding box lateral coordinates of 1280x800 images
                x1L = (x1-NN_input_width/2) * resolution_width /NN_input_width + CX_LEFT;
                y1L = (y1-NN_input_high/2)  * resolution_height/NN_input_high  + CY_LEFT;
                x2L = (x2-NN_input_width/2) * resolution_width /NN_input_width + CX_LEFT;
                y2L = (y2-NN_input_high/2)  * resolution_height/NN_input_high  + CY_LEFT;

                // Putting text in 300x300 image and printing bbox
                cv::putText(frame_left, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                confStr.str("");
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

        disparity_null_counter = 0;
        i = 0;
        reduction = 0.0;
        depth = 0.0;
        depth_sum = 0.0;
        depth_avg = 0.0;

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
                    distance = sqrt(pow(dL.cx-dR.cx,2)+pow(dL.cy-dR.cy,2));
                    area_error = abs(dR.area-dL.area)/(dR.area+dL.area);
                    ratio_error = abs(dR.aspect_ratio-dL.aspect_ratio)/(dR.aspect_ratio+dL.aspect_ratio);

                    // Third filter: Bbox matching by comparing areas and Y center coordinates
                    if((area_error <= area_threshold) && (ratio_error <= ratio_threshold) && (dL.cy <= (dR.cy + y_threshold)) && (dL.cy >= (dR.cy - y_threshold))){
                        // Computing the depth
                        // Depth calculation by reducing the size of the bbox and computing the average depth value 
                        for (int j=0;j<=N_REDUCTIONS;j++){
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
                        Z_centroid = depth_avg; // mm

                        // Get rid of outliers
                        if (Z_centroid/1000.0 > 0.2 && Z_centroid/1000.0 < 3.0){
                            
                            // Spatial centroid coordinates (the reference system is placed on the left monocamera)
                            X_centroid = B/2 + (Z_centroid*dL.cx)/FOCAL_LENGTH; // mm 
                            Y_centroid = (Z_centroid*dL.cy)/FOCAL_LENGTH; // mm

                            // Print spatial X inside the bbox
                            XStr.str("");
                            XStr << std::fixed << std::setprecision(3) << "X: " << X_centroid/1000 << " m";;
                            cv::putText(frame_right, XStr.str(), cv::Point(dR.x1*NN_input_width/resolution_height + NN_input_width/2 + 10, dR.y1*NN_input_width/resolution_height + NN_input_width/2 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                            cv::putText(frame_left, XStr.str(), cv::Point(dL.x1*NN_input_width/resolution_height + NN_input_width/2 + 10, dL.y1*NN_input_width/resolution_height + NN_input_width/2 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);  

                            // Print spatial Y inside the bbox
                            YStr.str("");
                            YStr << std::fixed << std::setprecision(3) << "Y: "<< Y_centroid/1000 << " m";;
                            cv::putText(frame_right, YStr.str(), cv::Point(dR.x1*NN_input_width/resolution_height + NN_input_width/2 + 10, dR.y1*NN_input_width/resolution_height + NN_input_width/2 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                            cv::putText(frame_left, YStr.str(), cv::Point(dL.x1*NN_input_width/resolution_height + NN_input_width/2 + 10, dL.y1*NN_input_width/resolution_height + NN_input_width/2 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

                            // Print spatial Z inside the bbox
                            ZStr.str("");
                            ZStr << std::fixed << std::setprecision(3) << "Z: " << Z_centroid/1000 << " m";;
                            cv::putText(frame_right, ZStr.str(), cv::Point(dR.x1*NN_input_width/resolution_height + NN_input_width/2 + 10, dR.y1*NN_input_width/resolution_height + NN_input_width/2 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                            cv::putText(frame_left, ZStr.str(), cv::Point(dL.x1*NN_input_width/resolution_height + NN_input_width/2 + 10, dL.y1*NN_input_width/resolution_height + NN_input_width/2 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                            
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
        msg.header.stamp = header.stamp;
        detections_pub.publish(msg);
        
        // Cleaning vectors for the next iteration
        msg.bounding_boxes.clear();
        detsLeft.clear();
        detsRight.clear();
    }
};

void OakDTaskStereoNeuralInference::stop(){
    detections_pub.shutdown();
    image_right_det_pub.shutdown();
    image_left_det_pub.shutdown();
};