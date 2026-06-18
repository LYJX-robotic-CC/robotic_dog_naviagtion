#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <clocale>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <mutex>

#include "localization_utils.h"

using fast_lio_localization_cpp::odomMsgToMat;

namespace
{
constexpr double LIDAR_OFFSET_X = 0.45;
constexpr double LIDAR_OFFSET_Y = 0.0;
constexpr double LIDAR_OFFSET_Z = 0.0;

class OdomTfPublisher
{
public:
    OdomTfPublisher()
    {
        sub_ = nh_.subscribe("/localization", 1, &OdomTfPublisher::callback, this);
        ROS_INFO("TF发布节点已启动");
        ROS_INFO("监听 /localization (雷达在 map 中的位姿)...");
        ROS_INFO("发布 map -> base_link 的 TF 变换...");
    }

private:
    void callback(const nav_msgs::OdometryConstPtr& msg)
    {
        const Eigen::Matrix4d T_map_to_lidar = odomMsgToMat(*msg);
        const Eigen::Quaterniond q_map_to_lidar(T_map_to_lidar.block<3, 3>(0, 0));
        const Eigen::Vector3d p_lidar_to_base(-LIDAR_OFFSET_X, -LIDAR_OFFSET_Y, -LIDAR_OFFSET_Z);
        const Eigen::Vector3d rotated_offset = q_map_to_lidar * p_lidar_to_base;

        tf::Transform tf_map_to_base;
        tf_map_to_base.setOrigin(tf::Vector3(
            T_map_to_lidar(0, 3) + rotated_offset.x(),
            T_map_to_lidar(1, 3) + rotated_offset.y(),
            T_map_to_lidar(2, 3) + rotated_offset.z()));
        tf_map_to_base.setRotation(tf::Quaternion(q_map_to_lidar.x(), q_map_to_lidar.y(), q_map_to_lidar.z(), q_map_to_lidar.w()));
        br_.sendTransform(tf::StampedTransform(tf_map_to_base, ros::Time::now(), "map", "base_link"));
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster br_;
};
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_base_link_tf_publisher_tf2");
    setlocale(LC_ALL, "");
    OdomTfPublisher node;
    ros::spin();
    return 0;
}
