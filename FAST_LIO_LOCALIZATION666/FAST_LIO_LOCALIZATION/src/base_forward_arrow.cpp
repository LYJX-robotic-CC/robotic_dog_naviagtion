#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_forward_arrow");
    ros::NodeHandle nh("~");

    const std::string frame_id = nh.param<std::string>("frame_id", "base_link");
    const std::string topic = nh.param<std::string>("marker_topic", "/fast_lio_localization/base_forward_arrow");
    const double length = nh.param("length", 0.75);
    const double shaft_diameter = nh.param("shaft_diameter", 0.08);
    const double head_diameter = nh.param("head_diameter", 0.22);
    const double publish_rate = nh.param("publish_rate", 10.0);

    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(topic, 1);
    ros::Rate rate(publish_rate);

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "base_forward_arrow";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = shaft_diameter;
    marker.scale.y = head_diameter;
    marker.scale.z = head_diameter;
    marker.color.r = 1.0f;
    marker.color.g = 0.2f;
    marker.color.b = 0.75f;
    marker.color.a = 1.0f;
    marker.points.resize(2);
    marker.points[0].x = 0.0;
    marker.points[0].y = 0.0;
    marker.points[0].z = 0.18;
    marker.points[1].x = length;
    marker.points[1].y = 0.0;
    marker.points[1].z = 0.18;

    while (ros::ok()) {
        marker.header.stamp = ros::Time::now();
        pub.publish(marker);
        rate.sleep();
    }

    return 0;
}
