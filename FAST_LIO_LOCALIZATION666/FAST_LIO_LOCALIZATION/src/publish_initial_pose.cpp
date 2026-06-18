#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <cstdlib>
#include <clocale>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_initial_pose");
    setlocale(LC_ALL, "");
    ros::NodeHandle nh;

    if (argc < 7) {
        ROS_ERROR("Usage: publish_initial_pose x y z yaw pitch roll");
        return 1;
    }

    const double x = std::atof(argv[1]);
    const double y = std::atof(argv[2]);
    const double z = std::atof(argv[3]);
    const double yaw = std::atof(argv[4]);
    const double pitch = std::atof(argv[5]);
    const double roll = std::atof(argv[6]);

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    ros::Duration(1.0).sleep();
    ROS_INFO("Initial Pose: %f %f %f %f %f %f", x, y, z, yaw, pitch, roll);
    pub.publish(msg);
    ros::Duration(0.2).sleep();
    return 0;
}
