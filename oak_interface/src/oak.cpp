#include <oak_interface/oakd_interface.hpp>

int main(int argc, char** argv){

    // ROS
    ros::init(argc, argv, "oak");

    ros::Time::init();

    ros::NodeHandle nh;

    OakDInterface oakd_interface;

    oakd_interface.setUp();
    oakd_interface.start();
    ros::Duration(2).sleep();
    ros::Rate loop_rate(60); // Frequency in Hz
    while(ros::ok()){   
        oakd_interface.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    oakd_interface.stop();

    return 0;
}






/*

    Oak_interface oak_interface;
    
    oak_interface.Start();

    ros::Time::init();
    ros::Rate loop_rate(120); // Frequency in Hz
    
    // Start pipeline
    OakPipeline oak_pipeline;
    oak_pipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = oak_pipeline.getExposedImageStreams();

    // Camera info
    auto left_CameraInfo            = oak_interface.left_camInfoManager->getCameraInfo();
    auto right_CameraInfo           = oak_interface.right_camInfoManager->getCameraInfo();
    auto depth_CameraInfo           = oak_interface.depth_camInfoManager->getCameraInfo();
    auto left_rectified_CameraInfo  = oak_interface.left_rectified_camInfoManager->getCameraInfo();
    auto right_rectified_CameraInfo = oak_interface.right_rectified_camInfoManager->getCameraInfo();
    auto rgb_rectified_CameraInfo   = oak_interface.color_camInfoManager->getCameraInfo();

    while(ros::ok()){

        std_msgs::Header header; // empty header
        header.stamp = ros::Time::now(); // time

        // Get std::shared_ptr<dai::ImgFrame> Frames
        if(oak_pipeline.camera_mono){
            auto left_frame = imageDataQueues[oak_pipeline.left_position]->tryGet<dai::ImgFrame>();
            auto right_frame = imageDataQueues[oak_pipeline.right_position]->tryGet<dai::ImgFrame>();

            if(!(left_frame == nullptr)){
                // Send image
                // ROS_INFO("LEFT read");
                oak_interface.getRosMsg(left_frame,oak_interface.left_image);
                oak_interface.left_pub.publish(oak_interface.left_image);

                // Send info
                left_CameraInfo.header.seq = left_frame->getSequenceNum();
                left_CameraInfo.header.stamp =  header.stamp;
                oak_interface.left_info_pub.publish(left_CameraInfo);
            }

            if(!(right_frame == nullptr)){
                // Send image
                // ROS_INFO("RIGHT read");
                oak_interface.getRosMsg(right_frame,oak_interface.right_image);
                oak_interface.right_pub.publish(oak_interface.right_image);

                // Send info
                right_CameraInfo.header.seq = right_frame->getSequenceNum();
                right_CameraInfo.header.stamp =  header.stamp;
                oak_interface.right_info_pub.publish(right_CameraInfo);
            }
        }
        if(oak_pipeline.camera_rectified){
            auto left_rect_frame = imageDataQueues[oak_pipeline.left_rectified_position]->tryGet<dai::ImgFrame>();
            auto right_rect_frame = imageDataQueues[oak_pipeline.right_rectified_position]->tryGet<dai::ImgFrame>();

            if(!(left_rect_frame == nullptr)){
                // Send image
                // ROS_INFO("RIGHT RECTIFIED read");
                oak_interface.getRosMsg(left_rect_frame,oak_interface.left_image_rectified);
                oak_interface.left_rectified_pub.publish(oak_interface.left_image_rectified);

                // Send info
                left_rectified_CameraInfo.header.seq = left_rect_frame->getSequenceNum();
                left_rectified_CameraInfo.header.stamp =  header.stamp;
                oak_interface.left_info_rectified_pub.publish(left_rectified_CameraInfo);
            }

            if(!(right_rect_frame == nullptr)){
                // Send image
                // ROS_INFO("DEPTH RECTIFIED read");
                oak_interface.getRosMsg(right_rect_frame,oak_interface.right_image_rectified);
                oak_interface.right_rectified_pub.publish(oak_interface.right_image_rectified);

                // Send info
                right_rectified_CameraInfo.header.seq = right_rect_frame->getSequenceNum();
                right_rectified_CameraInfo.header.stamp =  header.stamp;
                oak_interface.right_info_rectified_pub.publish(right_rectified_CameraInfo);
            }
        }
        if(oak_pipeline.camera_depth){
            auto depth_frame = imageDataQueues[oak_pipeline.depth_position]->tryGet<dai::ImgFrame>();

            if(!(depth_frame == nullptr)){
                // Send image
                // ROS_INFO("DEPTH read");
                oak_interface.getRosMsg(depth_frame,oak_interface.depth_image);
                oak_interface.depth_pub.publish(oak_interface.depth_image);

                // Send info
                depth_CameraInfo.header.seq = depth_frame->getSequenceNum();
                depth_CameraInfo.header.stamp =  header.stamp;
                oak_interface.depth_info_pub.publish(depth_CameraInfo);
            }
        }
        if(oak_pipeline.camera_rgb){
            auto rgb_frame = imageDataQueues[oak_pipeline.rgb_position]->tryGet<dai::ImgFrame>();

            if(!(rgb_frame == nullptr)){
                // Send image
                // ROS_INFO("RGB read");
                oak_interface.getRosMsg(rgb_frame,oak_interface.rgb_image);
                oak_interface.rgb_pub.publish(oak_interface.rgb_image);

                // Send info
                rgb_rectified_CameraInfo.header.seq = rgb_frame->getSequenceNum();
                rgb_rectified_CameraInfo.header.stamp =  header.stamp;
                oak_interface.rgb_info_pub.publish(rgb_rectified_CameraInfo);
            } 
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    oak_interface.Stop();

    return 0;
}
*/