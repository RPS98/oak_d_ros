#include <oak_interface/oakd_task_color.hpp>

void OakDTaskColor::start(ros::NodeHandle& nh){
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

    std::string color_uri = camera_param_uri + "/" + "color.yaml";
    
    // ROS
    // Publishers
    color_pub = nh.advertise<sensor_msgs::Image>("color/image", 1);
    color_info_pub = nh.advertise<sensor_msgs::CameraInfo>("color/info", 1);

    color_camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, deviceName, color_uri);
    color_CameraInfo = color_camInfoManager->getCameraInfo();
}

void OakDTaskColor::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index, std_msgs::Header header){

    color_frame = streams_queue[queue_index.inx_color]->tryGet<dai::ImgFrame>();

    if(!(color_frame == nullptr)){

        // Send image
        OakDTaskColor::get_RosMsg(color_frame,color_image_msg, header);
        color_image_msg.header.stamp = header.stamp;
        color_pub.publish(color_image_msg);

        // Send info
        color_CameraInfo.header.seq = color_frame->getSequenceNum();
        color_CameraInfo.header.stamp = ros::Time::now();
        color_info_pub.publish(color_CameraInfo);    
    } 

}

void OakDTaskColor::stop(){
    color_pub.shutdown();
    color_info_pub.shutdown();
}

void OakDTaskColor::get_RosMsg(std::shared_ptr<dai::ImgFrame> color_frame,
                sensor_msgs::Image &color_image_msg, std_msgs::Header header){

    if(use_interleaved){
        OakDUtils::getRosMsg(color_frame, color_image_msg, use_interleaved);
    } else {
        cv::Mat frame_cv2 = OakDUtils::getCvFrame(color_frame);
         cv_bridge::CvImage frame_cv_bridge = cv_bridge::CvImage(header ,sensor_msgs::image_encodings::BGR8, frame_cv2);
         frame_cv_bridge.toImageMsg(color_image_msg);
    }
}
