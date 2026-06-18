#ifndef FAST_LIO_LOCALIZATION_UTILS_H
#define FAST_LIO_LOCALIZATION_UTILS_H

#include "common_lib.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

namespace fast_lio_localization_cpp
{

using LocPointT = pcl::PointXYZI;
using LocCloudT = pcl::PointCloud<LocPointT>;
using RawMapPointT = pcl::PointXYZ;
using RawMapCloudT = pcl::PointCloud<RawMapPointT>;

inline Eigen::Matrix4d poseMsgToMat(const geometry_msgs::Pose& pose)
{
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    mat(0, 3) = pose.position.x;
    mat(1, 3) = pose.position.y;
    mat(2, 3) = pose.position.z;
    return mat;
}

inline Eigen::Matrix4d poseMsgToMat(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    return poseMsgToMat(msg.pose.pose);
}

inline Eigen::Matrix4d odomMsgToMat(const nav_msgs::Odometry& msg)
{
    return poseMsgToMat(msg.pose.pose);
}

inline geometry_msgs::Pose matToPose(const Eigen::Matrix4d& mat)
{
    geometry_msgs::Pose pose;
    Eigen::Quaterniond q(mat.block<3, 3>(0, 0));
    q.normalize();
    pose.position.x = mat(0, 3);
    pose.position.y = mat(1, 3);
    pose.position.z = mat(2, 3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

inline nav_msgs::Odometry matToOdom(const Eigen::Matrix4d& mat, const std_msgs::Header& header, const std::string& child_frame_id)
{
    nav_msgs::Odometry odom;
    odom.header = header;
    odom.child_frame_id = child_frame_id;
    odom.pose.pose = matToPose(mat);
    return odom;
}

inline Eigen::Matrix4d makeTransform(double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = (rz * ry * rx).toRotationMatrix();
    mat(0, 3) = x;
    mat(1, 3) = y;
    mat(2, 3) = z;
    return mat;
}

inline Eigen::Matrix4d inverseSe3(const Eigen::Matrix4d& trans)
{
    Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
    inv.block<3, 3>(0, 0) = trans.block<3, 3>(0, 0).transpose();
    inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * trans.block<3, 1>(0, 3);
    return inv;
}

inline double translationDistance(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

inline double rotationDistanceDeg(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    Eigen::Quaterniond qa(a.block<3, 3>(0, 0));
    Eigen::Quaterniond qb(b.block<3, 3>(0, 0));
    qa.normalize();
    qb.normalize();
    double dot = std::abs(qa.dot(qb));
    dot = std::max(-1.0, std::min(1.0, dot));
    return std::acos(dot) * 360.0 / M_PI;
}

inline Eigen::Matrix4d interpolateTransform(const Eigen::Matrix4d& from, const Eigen::Matrix4d& to, double alpha)
{
    const double a = std::max(0.0, std::min(1.0, alpha));
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.block<3, 1>(0, 3) = (1.0 - a) * from.block<3, 1>(0, 3) + a * to.block<3, 1>(0, 3);
    Eigen::Quaterniond q_from(from.block<3, 3>(0, 0));
    Eigen::Quaterniond q_to(to.block<3, 3>(0, 0));
    q_from.normalize();
    q_to.normalize();
    Eigen::Quaterniond q = q_from.slerp(a, q_to);
    q.normalize();
    out.block<3, 3>(0, 0) = q.toRotationMatrix();
    return out;
}

inline LocCloudT::Ptr voxelDownsample(const LocCloudT::ConstPtr& cloud, double leaf_size)
{
    pcl::VoxelGrid<LocPointT> vg;
    vg.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));
    vg.setInputCloud(cloud);
    LocCloudT::Ptr out(new LocCloudT());
    vg.filter(*out);
    return out;
}

inline LocCloudT::Ptr rosMsgToLocCloudXYZ(const sensor_msgs::PointCloud2& msg)
{
    RawMapCloudT raw;
    pcl::fromROSMsg(msg, raw);
    LocCloudT::Ptr out(new LocCloudT());
    out->reserve(raw.size());
    for (const auto& p : raw.points) {
        LocPointT q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = 0.0f;
        out->push_back(q);
    }
    return out;
}

inline LocCloudT::Ptr cropCloudAroundCenter(
    const LocCloudT::ConstPtr& cloud,
    const Eigen::Vector3d& center,
    double radius_xy,
    double radius_z)
{
    LocCloudT::Ptr out(new LocCloudT());
    if (!cloud || cloud->empty()) {
        return out;
    }
    const double r2 = radius_xy * radius_xy;
    out->reserve(cloud->size());
    for (const auto& p : cloud->points) {
        const double dx = p.x - center.x();
        const double dy = p.y - center.y();
        const double dz = p.z - center.z();
        if ((dx * dx + dy * dy) <= r2 && std::abs(dz) <= radius_z) {
            out->push_back(p);
        }
    }
    return out;
}

inline double confidenceFromFitness(double fitness_score)
{
    if (!std::isfinite(fitness_score) || fitness_score < 0.0) {
        return 0.0;
    }
    return 1.0 / (1.0 + fitness_score);
}

inline std::pair<Eigen::Matrix4d, double> icpRegister(
    const LocCloudT::ConstPtr& source,
    const LocCloudT::ConstPtr& target,
    const Eigen::Matrix4d& initial,
    double source_leaf,
    double target_leaf,
    double max_corr,
    int max_iterations)
{
    LocCloudT::Ptr source_ds = voxelDownsample(source, source_leaf);
    LocCloudT::Ptr target_ds = voxelDownsample(target, target_leaf);
    if (!source_ds || !target_ds || source_ds->empty() || target_ds->empty()) {
        return {Eigen::Matrix4d::Identity(), std::numeric_limits<double>::infinity()};
    }

    pcl::IterativeClosestPoint<LocPointT, LocPointT> icp;
    icp.setInputSource(source_ds);
    icp.setInputTarget(target_ds);
    icp.setMaximumIterations(max_iterations);
    icp.setMaxCorrespondenceDistance(static_cast<float>(max_corr));
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);

    LocCloudT aligned;
    fflush(stderr);
    const int saved_stderr = dup(STDERR_FILENO);
    const int null_fd = open("/dev/null", O_WRONLY);
    if (saved_stderr >= 0 && null_fd >= 0) {
        dup2(null_fd, STDERR_FILENO);
    }
    icp.align(aligned, initial.cast<float>());
    fflush(stderr);
    if (saved_stderr >= 0) {
        dup2(saved_stderr, STDERR_FILENO);
        close(saved_stderr);
    }
    if (null_fd >= 0) {
        close(null_fd);
    }

    Eigen::Matrix4d result = icp.getFinalTransformation().cast<double>();
    double score = std::numeric_limits<double>::infinity();
    if (icp.hasConverged()) {
        score = icp.getFitnessScore(static_cast<float>(max_corr));
    }
    return {result, score};
}

inline sensor_msgs::PointCloud2 makePointCloudMsg(
    const LocCloudT::ConstPtr& cloud,
    const std_msgs::Header& header)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    return msg;
}

}  // namespace fast_lio_localization_cpp

#endif  // FAST_LIO_LOCALIZATION_UTILS_H
