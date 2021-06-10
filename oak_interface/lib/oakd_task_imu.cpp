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

        //Velocidad angular en rad/s
        imu_msg.angular_velocity.x = imuPacket.gyroscope.x;
        imu_msg.angular_velocity.y = imuPacket.gyroscope.y;
        imu_msg.angular_velocity.z = imuPacket.gyroscope.z;
        //Falta la matriz de covarianza de la velocidad angular (no puede ser 0)
        //imu_msg.angular_velocity_orientation_covariance = {0,0,0,0,0,0,0,0,0};

        //Aceleración lineal en m/s²
        imu_msg.linear_acceleration.x = imuPacket.acceleroMeter.x;
        imu_msg.linear_acceleration.y = imuPacket.acceleroMeter.y;
        imu_msg.linear_acceleration.z = imuPacket.acceleroMeter.z;
        //Falta la matriz de covarianza de la aceleración lineal (no puede ser 0)
        //msg.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};

        // imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.stamp = header.stamp;
        imu_pub.publish(imu_msg);
    }                             
};

void OakDTaskIMU::stop(){
    imu_pub.shutdown();
};