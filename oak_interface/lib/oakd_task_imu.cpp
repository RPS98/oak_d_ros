#include <oak_interface/oakd_task_imu.hpp>

void OakDTaskIMU::start(ros::NodeHandle& nh){
    imu_pub = nh.advertise<sensor_msgs::Imu>("/Imu", 1);

    std::vector<double> orientation_covariance;
    if(!nh.getParam("/imu_covariance/orientation_covariance/data", orientation_covariance))
        ROS_ERROR("Failed to get imu_covariance parameter from server.");
    for(int i=0; i<9; i++){
        imu_msg.orientation_covariance[i] = orientation_covariance[i];
    }

    std::vector<double> angular_velocity_covariance;
    if(!nh.getParam("/imu_covariance/angular_velocity_covariance/data", angular_velocity_covariance))
        ROS_ERROR("Failed to get angular_velocity_covariance parameter from server.");
    for(int i=0; i<9; i++){
        imu_msg.angular_velocity_covariance[i] = angular_velocity_covariance[i];
    }

    std::vector<double> linear_acceleration_covariance;
    if(!nh.getParam("/imu_covariance/linear_acceleration_covariance/data", linear_acceleration_covariance))
        ROS_ERROR("Failed to get linear_acceleration_covariance parameter from server.");
    for(int i=0; i<9; i++){
        imu_msg.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
    }
    
};

void OakDTaskIMU::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, OakQueueIndex& queue_index, std_msgs::Header header){

    imu_queue = streams_queue[queue_index.inx_imu]->tryGet<dai::IMUData>();
    if(!(imu_queue == nullptr)){
        for(auto& imuPacket : imu_queue->packets) {
            //Orientación en cuaternios
            imu_msg.orientation.x = imuPacket.rotationVector.i;
            imu_msg.orientation.y = imuPacket.rotationVector.j;
            imu_msg.orientation.z = imuPacket.rotationVector.k;
            imu_msg.orientation.w = imuPacket.rotationVector.real;

            //Velocidad angular en rad/s
            imu_msg.angular_velocity.x = imuPacket.gyroscope.x;
            imu_msg.angular_velocity.y = imuPacket.gyroscope.y;
            imu_msg.angular_velocity.z = imuPacket.gyroscope.z;

            //Aceleración lineal en m/s²
            imu_msg.linear_acceleration.x = imuPacket.acceleroMeter.x;
            imu_msg.linear_acceleration.y = imuPacket.acceleroMeter.y;
            imu_msg.linear_acceleration.z = imuPacket.acceleroMeter.z;

            imu_msg.header.stamp = ros::Time::now();
            // imu_msg.header.stamp = header.stamp;
            imu_pub.publish(imu_msg);
        }  
    }                           
};

void OakDTaskIMU::stop(){
    imu_pub.shutdown();
};