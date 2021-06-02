#include <oak_interface/oakd_task_mono.hpp>

void OakDTaskMono::start(ros::NodeHandle& nh){

    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";

    if (ros::param::has("/camera_name")) {
        ros::param::get("/camera_name", deviceName);
    }
    if (ros::param::has("/camera_param_uri")) {
        ros::param::get("/camera_param_uri", camera_param_uri);
    }

    std::string left_uri = camera_param_uri +"/" + "left.yaml";
    std::string right_uri = camera_param_uri + "/" + "right.yaml";
    
    // ROS
    // Publishers
    left_pub = nh.advertise<sensor_msgs::Image>("left/image", 1);
    left_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/info", 1);

    right_pub = nh.advertise<sensor_msgs::Image>("right/image", 1);
    right_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/info", 1);

    left_camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, deviceName, left_uri);
    right_camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, deviceName, right_uri);

    left_CameraInfo = left_camInfoManager->getCameraInfo();
    right_CameraInfo = right_camInfoManager->getCameraInfo();
}

void OakDTaskMono::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                       OakQueueIndex& queue_index){

    left_frame = streams_queue[queue_index.inx_mono_left]->tryGet<dai::ImgFrame>();
    if(!(left_frame == nullptr)){
        // Send image
        OakDUtils::getRosMsg(left_frame,left_image_msg, true);
        left_pub.publish(left_image_msg);

        // Send info
        left_CameraInfo.header.seq = left_frame->getSequenceNum();
        left_CameraInfo.header.stamp = ros::Time::now();
        left_info_pub.publish(left_CameraInfo);
    } 

    right_frame = streams_queue[queue_index.inx_mono_right]->tryGet<dai::ImgFrame>();
    if(!(right_frame == nullptr)){
        // Send image
        OakDUtils::getRosMsg(right_frame,right_image_msg, true);
        right_pub.publish(right_image_msg);

        // Send info
        right_CameraInfo.header.seq = right_frame->getSequenceNum();
        right_CameraInfo.header.stamp = ros::Time::now();
        right_info_pub.publish(right_CameraInfo);
    }
}

void OakDTaskMono::stop(){
    left_pub.shutdown();
    left_info_pub.shutdown();
    right_pub.shutdown();
    right_info_pub.shutdown();
}