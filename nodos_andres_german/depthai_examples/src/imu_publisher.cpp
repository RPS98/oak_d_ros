#include <cstdio>
#include <iostream>

//#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


//ROS
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"



// Deberíamos crear una librería de pipeline

dai::Pipeline createCameraPipeline() {
    dai::Pipeline p;

    auto imu = p.create<dai::node::IMU>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("imu");
    dai::IMUSensorConfig sensorConfig;
    sensorConfig.reportIntervalUs = 2500; //400 Hz
    sensorConfig.sensorId = dai::IMUSensorId::RAW_ACCELEROMETER;
    imu->enableIMUSensor(sensorConfig);
    sensorConfig.sensorId = dai::IMUSensorId::RAW_GYROSCOPE;
    imu->enableIMUSensor(sensorConfig);
    sensorConfig.sensorId = dai::IMUSensorId::ROTATION_VECTOR;
    imu->enableIMUSensor(sensorConfig);

    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(5);

    imu->out.link(xlinkOut->input);

    return p;
}

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;

    //ROS 
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh("~");

    // Publisher
    ros::Publisher node_publisher = nh.advertise<sensor_msgs::Imu>("/Imu", 10);        
    ros::Rate loop_rate(10);
    sensor_msgs::Imu msg;
    //----

    auto baseTs = steady_clock::now();
    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);
    d.startPipeline();

    //Porque tenemos varias medidas de parámetros
    int firstTs = false;

    auto imuQueue = d.getOutputQueue("imu", 50, false);


    while (ros::ok()) {
        auto imuPacket = imuQueue->get<dai::IMUData>();
        auto imuDatas = imuPacket->imuDatas;

        for(auto& imuData : imuDatas) {
            auto acceleroTs1 = imuData.rawAcceleroMeter.timestamp.getTimestamp();
            auto gyroTs1 = imuData.rawGyroscope.timestamp.getTimestamp();
            auto rvTs1 = imuData.rotationVector.timestamp.getTimestamp();
            if(!firstTs) {
                //probar si esto sirve o hay q incluir algorithm
                //baseTs = std::min({acceleroTs1, gyroTs1, rvTs1});
                baseTs = std::min(std::min(acceleroTs1, gyroTs1), rvTs1);
                firstTs = true;
                }
            auto acceleroTs = acceleroTs1 - baseTs;
            auto gyroTs = gyroTs1 - baseTs;
            auto rvTs = rvTs1 - baseTs;
        
            
            //aqui publicamos
            //Orientación en cuaternios
            msg.orientation.x = imuData.rotationVector.i;
            msg.orientation.y = imuData.rotationVector.j;
            msg.orientation.z = imuData.rotationVector.k;
            msg.orientation.w = imuData.rotationVector.real;
            //Falta la matriz de covarianza de la orientación (no puede ser 0)
            //msg.orientation_covariance = {0,0,0,0,0,0,0,0,0};

            //Velocidad angular en rad/s
            msg.angular_velocity.x = imuData.rawGyroscope.x;
            msg.angular_velocity.y = imuData.rawGyroscope.y;
            msg.angular_velocity.z = imuData.rawGyroscope.z;
            //Falta la matriz de covarianza de la velocidad angular (no puede ser 0)
            //msg.angular_velocity_orientation = {0,0,0,0,0,0,0,0,0};
            
            //Aceleración lineal en m/s²
            msg.linear_acceleration.x = imuData.rawAcceleroMeter.x;
            msg.linear_acceleration.y = imuData.rawAcceleroMeter.y;
            msg.linear_acceleration.z = imuData.rawAcceleroMeter.z;
            //Falta la matriz de covarianza de la aceleración lineal (no puede ser 0)
            //msg.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};

            node_publisher.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

