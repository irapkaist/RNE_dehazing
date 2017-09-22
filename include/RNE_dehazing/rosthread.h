#ifndef ROSTHREAD_H
#define ROSTHREAD_H

#include <ros/ros.h>
#include <iostream>

#include <dynamic_reconfigure/server.h>
#include <RNE_dehazing/RNE_dehazingConfig.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
typedef RNE_dehazing::RNE_dehazingConfig RNE_config;


class RosThread
{

public:
    RosThread(ros::NodeHandle &nh, ros::NodeHandle& pnh);

    void
    run ();

    void get_cv_img (cv::Mat &cv_img) { cv_img = move(img_); }


private:
    bool global_init_;
    cv::Mat img_;
    cv::Mat dehazed_img_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher image_pub_;
    ros::Publisher image_ori_pub_;

    ros::Subscriber img_subs_;
    dynamic_reconfigure::Server<RNE_config> cfg_server_;
    RNE_config cfg_;

    void config_callback(RNE_config& config, int level);

    void image_callback(const sensor_msgs::Image::ConstPtr &msg);

    cv::Mat image_enhancement(const cv::Mat in_img);


};

#endif // ROSTHREAD_H
