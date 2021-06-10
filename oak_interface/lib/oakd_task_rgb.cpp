#include <oak_interface/oakd_task_rgb.hpp>

void OakDTaskRGB::start(ros::NodeHandle& nh){
    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";

    if (ros::param::has("/camera_name")) {
        ros::param::get("/camera_name", deviceName);
    }
    if (ros::param::has("/camera_param_uri")) {
        ros::param::get("/camera_param_uri", camera_param_uri);
    }

    if (ros::param::has("/publish_detections"))
        ros::param::get("/publish_detections", interleaved);

    std::string color_uri = camera_param_uri + "/" + "color.yaml";
    
    // ROS
    // Publishers
    rgb_pub = nh.advertise<sensor_msgs::Image>("rgb/image", 1);
    rgb_info_pub = nh.advertise<sensor_msgs::CameraInfo>("rgb/info", 1);

    rgb_camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, deviceName, color_uri);
    rgb_CameraInfo = rgb_camInfoManager->getCameraInfo();
}

void OakDTaskRGB::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index, std_msgs::Header header){

    rgb_frame = streams_queue[queue_index.inx_rgb]->tryGet<dai::ImgFrame>();

    if(!(rgb_frame == nullptr)){
        // Send image
        OakDUtils::getRosMsg(rgb_frame,rgb_image_msg, !interleaved);
        rgb_image_msg.header.stamp = header.stamp;
        rgb_pub.publish(rgb_image_msg);

        // Send info
        rgb_CameraInfo.header.seq = rgb_frame->getSequenceNum();
        rgb_CameraInfo.header.stamp = ros::Time::now();
        rgb_info_pub.publish(rgb_CameraInfo);    
    } 

}

void OakDTaskRGB::stop(){
    rgb_pub.shutdown();
    rgb_info_pub.shutdown();
}
