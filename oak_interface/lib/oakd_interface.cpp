#include <oak_interface/oakd_interface.hpp>

void OakDInterface::ownSetUp(){
    // List of information that must be published from params
    OakPublishList publish_list; 
    read_param(publish_list);

    // List of Oak-D features that must be initialize for publish
    OakUseList use_list;  
    create_use_list(use_list, publish_list);

    // Create a vector of tasks
    if(publish_list.publish_mono){
        tasks_list_.push_back(new OakDTaskMono());
    }

    if(publish_list.publish_depth){
        tasks_list_.push_back(new OakDTaskDepth());
    }
    if(publish_list.publish_rectified){
        tasks_list_.push_back(new OakDTaskRectified());
    }

    if(publish_list.publish_rgb){
        tasks_list_.push_back(new OakDTaskRGB());
    } 

    if(publish_list.publish_rgb_detections){
        tasks_list_.push_back(new OakDTaskRGBDetections());
    }
    if(publish_list.publish_imu){
        tasks_list_.push_back(new OakDTaskIMU());
    }
    if(publish_list.publish_stereo_detections){
        tasks_list_.push_back(new OakDTaskStereoDetections());
    }    


    oakd_pipeline.start(use_list, streams_queue_, queue_index_);
}

void OakDInterface::ownStart(){
    for(auto task:tasks_list_){
        task->start(nh);
    }
}

void OakDInterface::ownRun(){
    
    header.stamp = ros::Time::now();
    header.seq = seq;
    for(auto task:tasks_list_){
        task->run(streams_queue_, queue_index_, header);
    }
    seq = seq + 1;
}

void OakDInterface::ownStop(){
    for(auto task:tasks_list_){
        task->stop();
    }
}

// G: This function reads params from the launch file and stores them in variables of the struct OakPublishList
void OakDInterface::read_param(OakPublishList& publish_list){

    // List of publishers
    if (ros::param::has("/publish_mono"))
        ros::param::get("/publish_mono", publish_list.publish_mono);

    if (ros::param::has("/publish_depth"))
        ros::param::get("/publish_depth", publish_list.publish_depth);

    if (ros::param::has("/publish_rectified"))
        ros::param::get("/publish_rectified", publish_list.publish_rectified);

    if (ros::param::has("/publish_rgb"))
        ros::param::get("/publish_rgb", publish_list.publish_rgb);

    if (ros::param::has("/publish_rgb_detections")) {
        ros::param::get("/publish_rgb_detections", publish_list.publish_rgb_detections);
        if(publish_list.publish_rgb_detections){
            publish_list.publish_rgb = false;
            // Because detections do, not oak_task_rgb
        }
    }
        
    if (ros::param::has("/publish_imu"))
        ros::param::get("/publish_imu", publish_list.publish_imu);

    if (ros::param::has("/publish_stereo_detections"))
        ros::param::get("/publish_stereo_detections", publish_list.publish_stereo_detections);
}

// G: This function reads params from the launch file and stores them in order to know what nodes are necessary to define the pipeline
void OakDInterface::create_use_list(OakUseList& use_list, OakPublishList& publish_list){


    if(publish_list.publish_mono){ 
        use_list.use_mono = true;
    }
    if(publish_list.publish_depth){
        use_list.use_mono = true;
        use_list.use_depth = true;
    }
    if(publish_list.publish_rectified){
        use_list.use_mono = true;
        use_list.use_depth = true;
        use_list.use_rectified = true;
    }
    if(publish_list.publish_rgb){
        use_list.use_rgb = true;
    }
    // if(publish_list.publish_detections){
    //     use_list.use_mono = true;
    //     use_list.use_depth = true;     
    //     use_list.use_rgb = true;
    //     use_list.use_detections = true; 
    // }
    if(publish_list.publish_imu){
        use_list.use_imu = true;
    }

    // OakDTaskDetections and OakDTaskStereoNeuralInference can't be used at the same time
    if((publish_list.publish_rgb_detections == true) && (publish_list.publish_stereo_detections == false)){
        use_list.use_mono = true;
        use_list.use_depth = true;     
        use_list.use_rgb = true;
        use_list.use_rgb_detections = true; 
    }
    if((publish_list.publish_rgb_detections == false) && (publish_list.publish_stereo_detections == true)){
        use_list.use_mono = true;
        use_list.use_depth = true;
        use_list.use_rectified = true;
        use_list.use_stereo_detections = true; 
    }


}