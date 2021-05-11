
#include <cstdio>
#include <iostream>

//#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


//NEW
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

static bool testCamera = true;

dai::Pipeline createCameraPipeline() {
    dai::Pipeline p;

    auto imu = p.create<dai::node::IMU>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("imu");

    dai::IMUSensorConfig sensorConfig;
    sensorConfig.reportIntervalUs = 2500;  // 400hz
    sensorConfig.sensorId = dai::IMUSensorId::ROTATION_VECTOR;
    imu->enableIMUSensor(sensorConfig);
    // above this threshold packets will be sent in batch of X, if the host is not blocked
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    imu->setMaxBatchReports(5);
    // WARNING, temporarily 6 is the max

    // Link plugins CAM -> XLINK
    imu->out.link(xlinkOut->input);

    return p;
}

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;


    ros::init(argc, argv, "imu_test_node");
    ros::NodeHandle nh("~");


    auto baseTs = steady_clock::now();
    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);
    d.startPipeline();

    auto imuQueue = d.getOutputQueue("imu", 50, false);


    // Publisher
    ros::Publisher node_publisher = nh.advertise<sensor_msgs::Imu>("/Imu_data", 10);        
    ros::Rate loop_rate(10);

    while (ros::ok()){
        sensor_msgs::Imu msg;
        auto imuPacket = imuQueue->get<dai::IMUData>();

        auto imuDatas = imuPacket->imuDatas;
        for(auto& imuData : imuDatas) {
            auto rvTs = imuData.rotationVector.timestamp.getTimestamp() - baseTs;
            auto& rVvalues = imuData.rotationVector;
            printf("Rotation vector timestamp: %ld ms\n", duration_cast<milliseconds>(rvTs).count());

            printf(
                "Quaternion: i: %.3f j: %.3f k: %.3f real: %.3f\n"
                "Accuracy (rad): %.3f \n",
                rVvalues.i,
                rVvalues.j,
                rVvalues.k,
                rVvalues.real,
                rVvalues.accuracy);

            msg.orientation.x = rVvalues.i;
            msg.orientation.y = rVvalues.j;
            msg.orientation.z = rVvalues.k;
            msg.orientation.w = rVvalues.real;
            node_publisher.publish(msg);
        }





//        int key = cv::waitKey(1);
//        if(key == 'q') {
//            return 0;
//        }

        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}