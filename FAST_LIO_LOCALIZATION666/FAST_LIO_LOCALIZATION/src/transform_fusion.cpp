#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <mutex>
#include <thread>
#include <clocale>
#include <atomic>

#include "localization_utils.h"

using fast_lio_localization_cpp::odomMsgToMat;
using fast_lio_localization_cpp::matToOdom;
using fast_lio_localization_cpp::poseMsgToMat;

namespace
{
class TransformFusionNode
{
public:
    TransformFusionNode()
      : nh_()
      , pnh_("~")
    {
        freq_pub_ = pnh_.param("freq_pub_localization", 50.0);
        sub_odom_ = nh_.subscribe("/Odometry", 1, &TransformFusionNode::odomCallback, this);
        sub_map_to_odom_ = nh_.subscribe("/map_to_odom", 1, &TransformFusionNode::mapToOdomCallback, this);
        sub_localization_valid_ = nh_.subscribe("/localization_valid", 1, &TransformFusionNode::localizationValidCallback, this);
        pub_localization_ = nh_.advertise<nav_msgs::Odometry>("/localization", 1);
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(freq_pub_, 1.0)), &TransformFusionNode::timerCallback, this);
    }

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cur_odom_ = *msg;
        have_odom_ = true;
    }

    void localizationValidCallback(const std_msgs::BoolConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        localization_valid_ = msg->data;
    }

    void mapToOdomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cur_map_to_odom_ = *msg;
        have_map_to_odom_ = true;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const Eigen::Matrix4d T_map_to_odom = have_map_to_odom_ ? odomMsgToMat(cur_map_to_odom_) : Eigen::Matrix4d::Identity();
        const Eigen::Quaterniond q(T_map_to_odom.block<3, 3>(0, 0));
        tf::Transform tf_map_to_odom;
        tf_map_to_odom.setOrigin(tf::Vector3(T_map_to_odom(0, 3), T_map_to_odom(1, 3), T_map_to_odom(2, 3)));
        tf_map_to_odom.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        br_.sendTransform(tf::StampedTransform(tf_map_to_odom, ros::Time::now(), "map", "camera_init"));

        if (!have_odom_) {
            return;
        }

        if (!localization_valid_) {
            return;
        }

        const Eigen::Matrix4d T_odom_to_base_link = odomMsgToMat(cur_odom_);
        const Eigen::Matrix4d T_map_to_base_link = T_map_to_odom * T_odom_to_base_link;
        nav_msgs::Odometry localization = matToOdom(T_map_to_base_link, cur_odom_.header, "body");
        localization.header.frame_id = "map";
        localization.header.stamp = cur_odom_.header.stamp;
        pub_localization_.publish(localization);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_map_to_odom_;
    ros::Subscriber sub_localization_valid_;
    ros::Publisher pub_localization_;
    ros::Timer timer_;
    tf::TransformBroadcaster br_;
    std::mutex mutex_;
    nav_msgs::Odometry cur_odom_;
    nav_msgs::Odometry cur_map_to_odom_;
    bool have_odom_{false};
    bool have_map_to_odom_{false};
    bool localization_valid_{false};
    double freq_pub_{50.0};

};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_fusion");
    setlocale(LC_ALL, "");
    ROS_INFO("位姿融合节点已启动");
    TransformFusionNode node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
