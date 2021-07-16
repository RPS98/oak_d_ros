#include <oak_interface/oakd_interface.hpp>

int main(int argc, char** argv){

    // ROS
    ros::init(argc, argv, "oak");

    ros::Time::init();

    ros::NodeHandle nh;

    OakDInterface oakd_interface;

    oakd_interface.setUp();
    oakd_interface.start();
    ros::Duration(3).sleep();
    ros::Rate loop_rate(300); // Frequency in Hz
    while(ros::ok()){   
        oakd_interface.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    oakd_interface.stop();

    return 0;
}