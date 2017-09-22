#include <ros/ros.h>
#include <ros/node_handle.h>

#include <dynamic_reconfigure/server.h>
#include <RNE_dehazing/RNE_dehazingConfig.h>
#include <RNE_dehazing/rosthread.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


int main(int argc, char *argv[])
{
    // Ros Initilize
    ros::init(argc, argv, "rne_dehazing");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // ROS main loop
    RosThread rosth(nh, private_nh);
    rosth.run();

    return 0;
}
