#include <oak_interface/oakd_task_depth.hpp>

void OakDTaskDepth::start(ros::NodeHandle& nh){
    std::string deviceName = "oak";
    std::string camera_param_uri = "package://oak_interface/params/camera";

    if (ros::param::has("/camera_name")) {
        ros::param::get("/camera_name", deviceName);
    }
    if (ros::param::has("/camera_param_uri")) {
        ros::param::get("/camera_param_uri", camera_param_uri);
    }

   std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    
    // ROS
    // Publishers
    depth_pub = nh.advertise<sensor_msgs::Image>("depth/image", 1);
    depth_info_pub = nh.advertise<sensor_msgs::CameraInfo>("depth/info", 1);

    depth_camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(nh, deviceName, stereo_uri); 
    depth_CameraInfo = depth_camInfoManager->getCameraInfo();
}

void OakDTaskDepth::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
                      OakQueueIndex& queue_index, std_msgs::Header header){

    depth_frame = streams_queue[queue_index.inx_depth]->tryGet<dai::ImgFrame>();
     

    if(!(depth_frame == nullptr)){
        // Send image
        OakDUtils::getRosMsg(depth_frame,depth_image_msg, true);
        depth_image_msg.header.stamp = header.stamp;
        depth_image_msg.header.frame_id = "camera_link";
        depth_pub.publish(depth_image_msg);

        // Send info
        // depth_CameraInfo.header.seq = depth_frame->getSequenceNum();
        depth_CameraInfo = depth_camInfoManager->getCameraInfo();
        depth_CameraInfo.P[3] /= 100.0f;
        depth_CameraInfo.header.stamp = header.stamp;
        depth_CameraInfo.header.frame_id = "camera_link";
        depth_info_pub.publish(depth_CameraInfo);
    }
}

void OakDTaskDepth::stop(){
    depth_pub.shutdown();
    depth_info_pub.shutdown();
}
