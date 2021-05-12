#include <oak_interface/oakd_task_rectified.hpp>

void OakDTaskRectified::start(ros::NodeHandle& nh){
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
    left_rectified_pub = nh.advertise<sensor_msgs::Image>("left_rectified/image", 1);
    left_rectified_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left_rectified/info", 1);

    right_rectified_pub = nh.advertise<sensor_msgs::Image>("right_rectified/image", 1);
    right_rectified_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right_rectified/info", 1);

    left_rectified_camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, deviceName, left_uri);
    right_rectified_camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, deviceName, right_uri);

    left_rectified_CameraInfo = left_rectified_camInfoManager->getCameraInfo();
    right_rectified_CameraInfo = right_rectified_camInfoManager->getCameraInfo();
}

void OakDTaskRectified::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index){

    left_rectified_frame = streams_queue[queue_index.inx_rectified_left]->tryGet<dai::ImgFrame>();

    if(!(left_rectified_frame == nullptr)){
        // Send image
        OakDUtils::getRosMsg(left_rectified_frame,left_rectified_image_msg);
        left_rectified_pub.publish(left_rectified_image_msg);

        // Send info
        left_rectified_CameraInfo.header.seq = left_rectified_frame->getSequenceNum();
        left_rectified_CameraInfo.header.stamp = ros::Time::now();
        left_rectified_info_pub.publish(left_rectified_CameraInfo);

    } 

    right_rectified_frame = streams_queue[queue_index.inx_rectified_right]->tryGet<dai::ImgFrame>();

    if(!(right_rectified_frame == nullptr)){
        // Send image
        OakDUtils::getRosMsg(right_rectified_frame,right_rectified_image_msg);
        right_rectified_pub.publish(right_rectified_image_msg);

        // Send info
        right_rectified_CameraInfo.header.seq = right_rectified_frame->getSequenceNum();
        right_rectified_CameraInfo.header.stamp = ros::Time::now();
        right_rectified_info_pub.publish(right_rectified_CameraInfo);
    } 
}

void OakDTaskRectified::stop(){
    left_rectified_pub.shutdown();
    left_rectified_info_pub.shutdown();
    right_rectified_pub.shutdown();
    right_rectified_info_pub.shutdown();
}
