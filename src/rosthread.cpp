#include <RNE_dehazing/rosthread.h>


RosThread::RosThread(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), cfg_server_(pnh)
{
    string pub_name, pub_ori_name, sub_name;
    pnh_.param("sub_name", sub_name, std::string("dehaze_image"));

    pub_name = sub_name + "_dehaze";
    pub_ori_name = sub_name + "_original";
    img_subs_ = nh_.subscribe<sensor_msgs::Image>(sub_name, 2, boost::bind(&RosThread::image_callback, this, _1));
    image_pub_ = nh_.advertise<sensor_msgs::Image>(pub_name, 2);
    image_ori_pub_ = nh_.advertise<sensor_msgs::Image>(pub_ori_name, 2);
    cfg_server_.setCallback(boost::bind(&RosThread::config_callback, this, _1, _2));
}

void RosThread::run()
{
    ros::spin();
}

void RosThread::config_callback(RNE_config &config, int level)
{
    cfg_ = config;
    ROS_INFO("Dyn config (clip limit): %d", cfg_.clip_limit);
    ROS_INFO("Dyn config (box size): %d", cfg_.box_size);
}

void
RosThread::image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        cout << "Subscribe image" << endl;
        cv::Mat img_color;
        img_color = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::cvtColor(img_color, img_, CV_BGR2GRAY);
        dehazed_img_ = image_enhancement(img_);

        // Convert cv::mat to ros image
        cv_bridge::CvImage img_dehaze = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, dehazed_img_);
        cv_bridge::CvImage img_original = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, img_);

        // Publish
        image_pub_.publish(img_dehaze.toImageMsg());
        image_ori_pub_.publish(img_original.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

cv::Mat RosThread::image_enhancement(const cv::Mat in_img)
{
    cv::Mat dehazed_img;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(cfg_.clip_limit, cv::Size(cfg_.box_size,cfg_.box_size));
    clahe->apply(in_img, dehazed_img);

    return dehazed_img;
}
