#include <oak_interface/oakd_task_stereo_neural_inference.hpp>
#include <cmath>

#define f_left 872.2578152369078 // focal length for left camera in pixels
#define f_right 875.2617250748619 // focal lenght for right camera in pixels
#define B 75  // distance between left and right cameras
#define CX_left 644.2604132944807 // CX for left camera in pixels
#define CX_right 641.3521672332931 // CX for right camera in pixels

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
    cont = 0;
    cont_depth_avg = 0;
    depth_avg_total = 0.0;
};

void OakDTaskStereoNeuralInference::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index){
    
    using namespace std::chrono;
    //std::cout << "Running OakDTaskStereoNeuralInference" << std::endl;

    rectified_left = streams_queue[queue_index.inx_rectified_left]->get<dai::ImgFrame>();
    rectified_right = streams_queue[queue_index.inx_rectified_right]->get<dai::ImgFrame>();
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

    // Prueba para saber que hace la funcion setResize
    cv::Mat frame_rectified_right = OakDUtils::getCvFrame(rectified_right);
    
    /*cv::Rect crop_region(240,0,800,800);
    cv::Mat image_cropped=frame_rectified_right(crop_region);
    cv::imshow("right", frame_right);
    cv::imshow("rectified_right", image_cropped);
    cv::waitKey(1); */


    for(const auto& d : dets_right) {
        // x1, x2, x3 , x4 -> coordenadas laterales en pixeles de las 2 esquinas de la bbox
        int x1 = d.xmin * frame_right.cols;
        int y1 = d.ymin * frame_right.rows;
        int x2 = d.xmax * frame_right.cols;
        int y2 = d.ymax * frame_right.rows;

        // coordenadas de la bbox para la imagen 1280x800
        int x1R = (x1 - 150) * 800/300 + 644.2604;
        int y1R = (y1 - 150) * 800/300 + 370.4150;
        int x2R = (x2 - 150) * 800/300 + 644.2604;
        int y2R = (y2 - 150) * 800/300 + 370.4150;

        int labelIndex = d.label;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < label_map.size()) {
            labelStr = label_map[labelIndex];
        }

        // Image 1280x800 
        cv::putText(frame_rectified_right, labelStr, cv::Point(x1R + 10, y1R + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream confStr_;
        confStr_ << std::fixed << std::setprecision(2) << d.confidence * 100;
        cv::putText(frame_rectified_right, confStr_.str(), cv::Point(x1R + 10, y1R + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

        cv::rectangle(frame_rectified_right, cv::Rect(cv::Point(x1R, y1R), cv::Point(x2R, y2R)), color_, cv::FONT_HERSHEY_SIMPLEX);

        // Image 300x300
        cv::putText(frame_right, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
        cv::putText(frame_right, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

        cv::rectangle(frame_right, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

        dets.type = (std::string)labelStr;
        dets.prob = (float)d.confidence;
        dets.x1 = (int)x1R;
        dets.y1 = (int)y1R;
        dets.x2 = (int)x2R;
        dets.y2 = (int)y2R;
        dets.cx = (float)(x1R + x2R)/2;
        dets.cy = (float)(y1R + y2R)/2;
        dets.area = abs(x2R-x1R)*abs(y2R-y1R);

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
    cv::Mat frame_rectified_left = OakDUtils::getCvFrame(rectified_left);

    for(const auto& d : dets_left) {
        int x1 = d.xmin * frame_left.cols;
        int y1 = d.ymin * frame_left.rows;
        int x2 = d.xmax * frame_left.cols;
        int y2 = d.ymax * frame_left.rows;

        int x1L = (x1 - 150) * 800/300 + 641.3522;
        int y1L = (y1 - 150) * 800/300 + 372.6643;;
        int x2L = (x2 - 150) * 800/300 + 641.3522;
        int y2L = (y2 - 150) * 800/300 + 372.6643;;

        int labelIndex = d.label;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < label_map.size()) {
            labelStr = label_map[labelIndex];
        }
        
        // Image 1280x800
        cv::putText(frame_rectified_left, labelStr, cv::Point(x1L + 10, y1L + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream confStr_;
        confStr_ << std::fixed << std::setprecision(2) << d.confidence * 100;
        cv::putText(frame_rectified_left, confStr_.str(), cv::Point(x1L + 10, y1L + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

        cv::rectangle(frame_rectified_left, cv::Rect(cv::Point(x1L, y1L), cv::Point(x2L, y2L)), color_, cv::FONT_HERSHEY_SIMPLEX);

        // Image 300x300
        cv::putText(frame_left, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
        cv::putText(frame_left, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

        cv::rectangle(frame_left, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

        dets.type = (std::string)labelStr;
        dets.prob = (float)d.confidence;
        dets.x1 = (int)x1L;
        dets.y1 = (int)y1L;
        dets.x2 = (int)x2L;
        dets.y2 = (int)y2L;
        dets.cx = (float)((x2L + x1L)/2);
        dets.cy = (float)((y2L + y1L)/2);
        dets.area = abs(x2L-x1L)*abs(y2L-y1L);
        
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
    float dist_threshold = 20000000.0;
    float area_threshold = 70000000.0;

    float focal_length = (f_right + f_left)/2; 
    float depth, cont_depth = 0, depth_avg = 0.0;
    int reducciones = 2;
    float r = 0;
    float disparity;
    int e = 0;
    int n_puntos;

    for(const auto& dR : detsRight){
        for(const auto& dL : detsLeft){
            if(dR.type == dL.type && dR.type == "bottle"){
                float distance = sqrt(pow(dL.cx-dR.cx,2)+pow(dL.cy-dR.cy,2));
                float area = abs(dR.area-dL.area);
                
                if((distance <= dist_threshold) && (area <= area_threshold)){
                    bbox.Class = dR.type;
                    bbox.probability = dR.prob;
                    bbox.xmin = (int)dR.x1;
                    bbox.ymin = (int)dR.y1;
                    bbox.xmax = (int)dR.x2;
                    bbox.ymax = (int)dR.y2;

                    // Computing the depth
                    // int disparity = (int)abs((dR.x1-CX_right/2)-(dL.x1-CX_left/2));
                    // if(disparity == 0) disparity = 1;
                    // bbox.depth = (focal_length*B)/disparity; // Stereo model
                    // std::cout<<"class: "<<dR.type<<std::endl;
                    // std::cout<<"distance: "<<distance<<std::endl;
                    // std::cout<<"area: "<<area<<std::endl;
                    // std::cout<<"disparity: "<<disparity<<std::endl;
                    // std::cout<<"depth: "<<bbox.depth/1000<<std::endl;
                    // std::cout<<" "<<std::endl;
                    e = 0;
                    r = 0;
                    cont_depth = 0;
                    for (int i=0;i<=reducciones;i++){
                        disparity = abs((dR.x1+r*dR.x1 - CX_right/2)-(dL.x1 + r*dL.x1 - CX_left/2));
                        if (disparity <= 0.001) e += 1;
                        else {
                            depth = (focal_length*B)/disparity;
                            cont_depth = cont_depth + depth;
                        }


                        disparity = abs((dR.x2 - r*dR.x2 - CX_right/2)-(dL.x2 - r*dL.x2 - CX_left/2));
                        if (disparity <= 0.001) e += 1;
                        else {
                            depth = (focal_length*B)/disparity;
                            cont_depth = cont_depth + depth;
                        }
                        
                        r += 0.05;
                    }

                    // A: average of calculated depths when disparity isn't always 0
                    if (e > 0 && cont_depth == 0) 
                        std::cout<<"Disparidad 0 "<<std::endl;
                    else{
                        n_puntos = (reducciones+1)*2-e;
                        depth_avg = cont_depth/n_puntos;
                    }

                        
                    //cont_depth_avg = cont_depth_avg + depth_avg;
                


                    // int disparity3 = (int)abs((dR.x1-CX_right/2)-(dL.x1-CX_left/2));
                    // if (disparity3 <= 0.0001 && disparity3 >= -0.0001) depth1 = 0.0000;
                    // else depth1 = (focal_length*B)/disparity1;

                    // int disparity2 = (int)abs((dR.x2-CX_right/2)-(dL.x2-CX_left/2));
                    // if (disparity2 <= 0.01 && disparity2 >= -0.01) depth2 = 0.0000;
                    // else depth2 = (focal_length*B)/disparity2;
                    std::stringstream avgStr;

                    /*if (cont == 10) {
                        depth_avg_total = depth_avg/(cont+1);
                        cont = 0;
                        cont_depth_avg = 0;
                    } */                    
                    //cont = cont + 1;

                    avgStr << std::fixed << std::setprecision(3) << depth_avg/10;
                    cv::putText(frame_rectified_right, avgStr.str(), cv::Point(dR.x1 + 10, dR.y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
                    cv::putText(frame_rectified_left, avgStr.str(), cv::Point(dL.x1 + 10, dL.y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);  

                    std::cout<<"depth_avg: "<< depth_avg/10 <<std::endl;

                    msg.bounding_boxes.push_back(bbox);
                }

            }   
        }
    }


    std::stringstream fpsStr;
    fpsStr << std::fixed << std::setprecision(2) << fps_;
    cv::putText(frame_rectified_right, fpsStr.str(), cv::Point(2, right->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color_);
    cv::putText(frame_rectified_left, fpsStr.str(), cv::Point(2, left->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color_);

    cv::imshow("rectified_right_dets", frame_rectified_right);
    cv::imshow("rectified_left_dets", frame_rectified_left);
    cv::waitKey(1);

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
    detsLeft.clear();
    detsRight.clear();

    
    //right_detections.bounding_boxes.clear();
    //left_detections.bounding_boxes.clear();

};

void OakDTaskStereoNeuralInference::stop(){
    detections_pub.shutdown();
    image_right_det_pub.shutdown();
    image_left_det_pub.shutdown();
};