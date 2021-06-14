#include <oak_interface/oakd_task_imu.hpp>

void OakDTaskIMU::start(ros::NodeHandle& nh){
    std::cout << "Starting IMU task" << std::endl;
    imu_pub = nh.advertise<sensor_msgs::Imu>("/Imu", 1);
    //Descomentar si necesitamos el tiempo
    // baseTs = steady_clock::now();
    // firstTs = false;
};

void OakDTaskIMU::run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, OakQueueIndex& queue_index, std_msgs::Header header){

    imu_queue = streams_queue[queue_index.inx_imu]->get<dai::IMUData>();

    imuPackets = imu_queue->packets;

    for(auto& imuPacket : imuPackets) {
        //Descomentar si necesitamos el tiempo
        // acceleroTs1 = imuPacket.rawAcceleroMeter.timestamp.getTimestamp();
        // gyroTs1 = imuPacket.rawGyroscope.timestamp.getTimestamp();
        // rvTs1 = imuPacket.rotationVector.timestamp.getTimestamp();
        // if(!firstTs){
        //     baseTs = std::min(std::min(acceleroTs1, gyroTs1), rvTs1);
        //     firstTs = true;
        // }
        // acceleroTs = acceleroTs1 - baseTs;  
        // gyroTs = gyroTs1 - baseTs;
        // rvTs = rvTs1 - baseTs;

        //Orientación en cuaternios
        imu_msg.orientation.x = imuPacket.rotationVector.i;
        imu_msg.orientation.y = imuPacket.rotationVector.j;
        imu_msg.orientation.z = imuPacket.rotationVector.k;
        imu_msg.orientation.w = imuPacket.rotationVector.real;
        imu_msg.orientation_covariance = {0.1,0,0,
                                          0,0.1,0,
                                          0,0,0.1}; //THIS IS INVENTED, NEEDS TO BE CALCULATED
        //Velocidad angular en rad/s
        imu_msg.angular_velocity.x = imuPacket.gyroscope.x;
        imu_msg.angular_velocity.y = imuPacket.gyroscope.y;
        imu_msg.angular_velocity.z = imuPacket.gyroscope.z;
        imu_msg.angular_velocity_covariance = {0.0072255522344464793,0,0,
                                               0,0.0061319799724541104,0,
                                               0,0,0.0081724950866878237};

        //Aceleración lineal en m/s²
        imu_msg.linear_acceleration.x = imuPacket.acceleroMeter.x;
        imu_msg.linear_acceleration.y = imuPacket.acceleroMeter.y;
        imu_msg.linear_acceleration.z = imuPacket.acceleroMeter.z;
        imu_msg.linear_acceleration_covariance = {0.027129571855099233,0,0,
                                              0,0.025032181819875311,0,
                                              0,0,0.028286689365978803};

        // imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.stamp = header.stamp;
        imu_pub.publish(imu_msg);
    }                             
};

void OakDTaskIMU::stop(){
    imu_pub.shutdown();
};