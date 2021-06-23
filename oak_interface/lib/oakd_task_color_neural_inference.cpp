#include <oak_interface/oakd_task_color_neural_inference.hpp>

const std::vector<std::string> OakDTaskColorNeuralInference::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"}; 

void OakDTaskColorNeuralInference::start(ros::NodeHandle& nh){
    
    std::cout << "Starting OakDTaskColorNeuralInference" << std::endl;

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
    detections_pub = nh.advertise<oak_interface::BoundingBoxes>("/detections_spatial", 1);
    image_detections_pub = nh.advertise<sensor_msgs::Image>("/detections_images", 1);
    depth_pub = nh.advertise<sensor_msgs::Image>("/detections_depth", 1);

    // ROS Subscriber
    color_image_sub = nh.subscribe("color/image", 1, &OakDTaskColorNeuralInference::colorImageCallback, this);

    // Initialization of some atributes
    startTime = std::chrono::steady_clock::now();
    counter_ = 0;
    fps_ = 0;
    color_ = cv::Scalar(255, 255, 255);
};

void OakDTaskColorNeuralInference::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index, std_msgs::Header header){
  
    using namespace std::chrono;

    std::cout << "Running OakDTaskColorNeuralInference" << std::endl;

    if(!color_frame.empty()){
    
        // imgFrame = streams_queue[queue_index.inx_detections_color_image]->get<dai::ImgFrame>();
        det      = streams_queue[queue_index.inx_detections_color]->get<dai::SpatialImgDetections>();
        depth    = streams_queue[queue_index.inx_depth]->get<dai::ImgFrame>();

        //OakDUtils utils;
        auto dets = det->detections;
        cv::Mat depthFrame = OakDUtils::getFrame(depth,false); // depth->getFrame();
        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        if(!dets.empty()) {
            boundingBoxMapping = streams_queue[queue_index.inx_bbDepthMapping]->get<dai::SpatialLocationCalculatorConfig>();
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
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps_ = counter_ / elapsed.count();
            counter_ = 0;
            startTime = currentTime;
        }

        // cv::Mat frame = OakDUtils::getCvFrame(imgFrame);

        for(const auto& d : dets) {
            int x1 = d.xmin * color_frame.cols;
            int y1 = d.ymin * color_frame.rows;
            int x2 = d.xmax * color_frame.cols;
            int y2 = d.ymax * color_frame.rows;

            int labelIndex = d.label;
            std::string labelStr = std::to_string(labelIndex);
            if(labelIndex < label_map.size()) {
                labelStr = label_map[labelIndex];
            }
            cv::putText(color_frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
            cv::putText(color_frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

            std::stringstream depthX;
            depthX << "X: " << (int)d.spatialCoordinates.x/1000 << " m";
            cv::putText(color_frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            std::stringstream depthY;
            depthY << "Y: " << (int)d.spatialCoordinates.y/1000 << " m";
            cv::putText(color_frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)d.spatialCoordinates.z/1000 << " m";
            cv::putText(color_frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color_);

            cv::rectangle(color_frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color_, cv::FONT_HERSHEY_SIMPLEX);

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

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps_;
        //cv::putText(color_frame, fpsStr.str(), cv::Point(2, mat.size().height - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color_);
        //cv::imshow("depth", depthFrameColor);
        //cv::imshow("preview", frame);
        //cv::waitKey(1);
        
        // From cv::Mat to cv_bridge::CvImage 
        //std_msgs::Header header;
        cv_bridge::CvImage frame_cv_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8, color_frame);
        //cv_bridge::CvImage depth_cv_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8, depthFrameColor);

        // From cv_bridge::CvImage to sensor_msgs::Image
        frame_cv_bridge.toImageMsg(frame_msg);
        //depth_cv_bridge.toImageMsg(frame_depth_msg);

        // Publishing the image with detections in the topic "/detections_images"
        image_detections_pub.publish(frame_msg);
        //depth_pub.publish(frame_depth_msg);

        // Publishing the info of detections in the topic "/detections"
        detections_pub.publish(msg); 
        msg.bounding_boxes.clear();
    }
};

void OakDTaskColorNeuralInference::stop(){
    detections_pub.shutdown();
    image_detections_pub.shutdown();
    depth_pub.shutdown();
};

void OakDTaskColorNeuralInference::colorImageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //cv_bridge::toCvShare(msg, "bgr8")->image;
    color_frame = cv_ptr->image;
}