#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <clocale>
#include <atomic>
#include "localization_utils.h"
using fast_lio_localization_cpp::confidenceFromFitness;
using fast_lio_localization_cpp::cropCloudAroundCenter;
using fast_lio_localization_cpp::icpRegister;
using fast_lio_localization_cpp::inverseSe3;
using fast_lio_localization_cpp::interpolateTransform;
using fast_lio_localization_cpp::rotationDistanceDeg;
using fast_lio_localization_cpp::translationDistance;
using fast_lio_localization_cpp::makePointCloudMsg;
using fast_lio_localization_cpp::makeTransform;
using fast_lio_localization_cpp::matToOdom;
using fast_lio_localization_cpp::odomMsgToMat;
using fast_lio_localization_cpp::poseMsgToMat;
using fast_lio_localization_cpp::voxelDownsample;
namespace
{
using CloudT = pcl::PointCloud<pcl::PointXYZI>;
using CloudTPtr = CloudT::Ptr;
using DescriptorT = Eigen::MatrixXf;
using RingKeyT = Eigen::VectorXf;
enum class LocalizationRequestSource
{
    Automatic,
    Manual,
};

struct ScanContextMatch
{
    std::size_t index{0};
    double distance{std::numeric_limits<double>::infinity()};
    int sector_shift{0};
};

std::string matToString(const Eigen::Matrix4d& mat)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(4);
    oss << "[[" << mat(0, 0) << ", " << mat(0, 1) << ", " << mat(0, 2) << ", " << mat(0, 3) << "], "
        << "[" << mat(1, 0) << ", " << mat(1, 1) << ", " << mat(1, 2) << ", " << mat(1, 3) << "], "
        << "[" << mat(2, 0) << ", " << mat(2, 1) << ", " << mat(2, 2) << ", " << mat(2, 3) << "], "
        << "[" << mat(3, 0) << ", " << mat(3, 1) << ", " << mat(3, 2) << ", " << mat(3, 3) << "]]";
    return oss.str();
}
std::vector<double> candidateYaws()
{
    return {0.0, M_PI / 4.0, M_PI / 2.0, 3.0 * M_PI / 4.0, M_PI, 5.0 * M_PI / 4.0, 3.0 * M_PI / 2.0, 7.0 * M_PI / 4.0};
}
Eigen::Matrix4d makeInitial(double x, double y, double z, double roll, double pitch, double yaw)
{
    return makeTransform(x, y, z, roll, pitch, yaw);
}

double yawFromTransform(const Eigen::Matrix4d& mat)
{
    return std::atan2(mat(1, 0), mat(0, 0));
}
}  // namespace
class GlobalLocalizationNode
{
public:
    GlobalLocalizationNode()
      : nh_()
      , pnh_("~")
      , global_map_(new CloudT())
      , cur_scan_(new CloudT())
      , initialized_(false)
      , have_map_(false)
      , have_scan_(false)
      , have_odom_(false)
      , T_map_to_odom_(Eigen::Matrix4d::Identity())
      , manual_initial_pose_(Eigen::Matrix4d::Identity())
    {
        map_voxel_size_ = pnh_.param("map_voxel_size", 0.4);
        scan_voxel_size_ = pnh_.param("scan_voxel_size", 0.1);
        freq_localization_ = pnh_.param("freq_localization", 0.5);
        localization_th_ = pnh_.param("localization_th", 0.95);
        global_recall_trigger_th_ = pnh_.param("global_recall_trigger_th", 0.99);
        global_recall_success_th_ = pnh_.param("global_recall_success_th", 0.85);
        failures_before_recovery_ = pnh_.param("failures_before_recovery", 2);
        enable_global_recall_ = pnh_.param("enable_global_recall", true);
        auto_init_ = pnh_.param("enable_auto_init", true);
        fov_ = pnh_.param("fov", 6.28);
        fov_far_ = pnh_.param("fov_far", 30.0);
        recall_grid_step_ = pnh_.param("recall_grid_step", 6.0);
        recall_z_step_ = pnh_.param("recall_z_step", 1.5);
        recall_crop_radius_ = pnh_.param("recall_crop_radius", 22.0);
        recall_crop_radius_z_ = pnh_.param("recall_crop_radius_z", 3.0);
        recall_map_voxel_size_ = pnh_.param("recall_map_voxel_size", 0.8);
        recall_scan_scale_ = pnh_.param("recall_scan_scale", 3);
        recall_coarse_scale_ = pnh_.param("recall_coarse_scale", 4);
        enable_scan_context_ = pnh_.param("enable_scan_context", true);
        global_search_grid_ = pnh_.param("global_search_grid", 2.0);
        global_search_max_candidates_ = pnh_.param("global_search_max_candidates", 100);
        global_search_support_radius_bins_ = pnh_.param("global_search_support_radius_bins", 2);
        global_search_min_points_per_bin_ = pnh_.param("global_search_min_points_per_bin", 1);
        global_search_submap_radius_ = pnh_.param("global_search_submap_radius", 25.0);
        scan_context_num_rings_ = pnh_.param("scan_context_num_rings", 20);
        scan_context_num_sectors_ = pnh_.param("scan_context_num_sectors", 60);
        scan_context_max_radius_ = pnh_.param("scan_context_max_radius", 25.0);
        scan_context_z_offset_ = pnh_.param("scan_context_z_offset", 2.0);
        scan_context_z_clip_ = pnh_.param("scan_context_z_clip", 10.0);
        scan_context_ringkey_topk_ = pnh_.param("scan_context_ringkey_topk", 12);
        scan_context_refine_topk_ = pnh_.param("scan_context_refine_topk", 8);
        local_crop_radius_xy_ = pnh_.param("local_crop_radius_xy", 8.0);
        local_crop_radius_z_ = pnh_.param("local_crop_radius_z", 3.0);
        stabilize_hold_translation_ = pnh_.param("stabilize_hold_translation", 0.03);
        stabilize_hold_rotation_deg_ = pnh_.param("stabilize_hold_rotation_deg", 0.25);
        stabilize_small_translation_ = pnh_.param("stabilize_small_translation", 0.10);
        stabilize_small_rotation_deg_ = pnh_.param("stabilize_small_rotation_deg", 1.0);
        stabilize_alpha_small_ = pnh_.param("stabilize_alpha_small", 0.08);
        stabilize_alpha_large_ = pnh_.param("stabilize_alpha_large", 0.20);
        stabilize_confidence_gate_ = pnh_.param("stabilize_confidence_gate", 0.95);
        odom_error_log_period_ = pnh_.param("odom_error_log_period", 1.0);
        drift_reset_linear_speed_th_ = pnh_.param("drift_reset_linear_speed_th", 0.05);
        drift_reset_angular_speed_th_ = pnh_.param("drift_reset_angular_speed_th", 0.10);
        fastlio_reset_cooldown_ = pnh_.param("fastlio_reset_cooldown", 3.0);
        low_confidence_frames_before_invalidate_ = pnh_.param("low_confidence_frames_before_invalidate", 3);
        strong_local_recall_xy_step_ = pnh_.param("strong_local_recall_xy_step", 1.0);
        strong_local_recall_xy_radius_ = pnh_.param("strong_local_recall_xy_radius", 3.0);
        strong_local_recall_yaw_step_deg_ = pnh_.param("strong_local_recall_yaw_step_deg", 30.0);
        strong_local_recall_yaw_radius_deg_ = pnh_.param("strong_local_recall_yaw_radius_deg", 90.0);
        strong_local_recall_trigger_streak_ = pnh_.param("strong_local_recall_trigger_streak", 3);
        recall_fine_topk_ = pnh_.param("recall_fine_topk", 6);
        recall_min_crop_points_ = pnh_.param("recall_min_crop_points", 200);
        manual_local_search_xy_step_ = pnh_.param("manual_local_search_xy_step", 0.5);
        manual_local_search_xy_radius_ = pnh_.param("manual_local_search_xy_radius", 0.5);
        manual_local_search_z_step_ = pnh_.param("manual_local_search_z_step", 1.0);
        manual_local_search_z_radius_ = pnh_.param("manual_local_search_z_radius", 1.0);
        manual_local_search_yaw_step_deg_ = pnh_.param("manual_local_search_yaw_step_deg", 22.5);
        manual_local_search_yaw_radius_deg_ = pnh_.param("manual_local_search_yaw_radius_deg", 45.0);
        manual_local_search_topk_ = pnh_.param("manual_local_search_topk", 3);
        manual_pose_pending_.store(false);
        manual_pose_seq_.store(0);
        manual_pose_applied_seq_.store(-1);
        consecutive_failures_ = 0;
        relocalization_mode_ = false;
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
        pub_pc_in_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/cur_scan_in_map", 1);
        pub_submap_ = nh_.advertise<sensor_msgs::PointCloud2>("/submap", 1);
        pub_map_to_odom_ = nh_.advertise<nav_msgs::Odometry>("/map_to_odom", 1);
        pub_localization_valid_ = nh_.advertise<std_msgs::Bool>("/localization_valid", 1, true);
        pub_fastlio_reset_ = nh_.advertise<std_msgs::Empty>("/fast_lio_reset", 1, false);
        publishLocalizationValid(false);
        sub_scan_ = nh_.subscribe("/cloud_registered", 1, &GlobalLocalizationNode::scanCallback, this);
        sub_odom_ = nh_.subscribe("/Odometry", 1, &GlobalLocalizationNode::odomCallback, this);
        sub_initial_ = nh_.subscribe("/initialpose", 1, &GlobalLocalizationNode::initialPoseCallback, this);
        ROS_WARN("等待全局地图...");
        auto map_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/map", nh_, ros::Duration(30.0));
        if (!map_msg) {
            throw std::runtime_error("等待 /map 超时");
        }
        loadGlobalMap(*map_msg);
        ROS_INFO("重定位节点已启动");
        ROS_INFO("全局召回开关：%s", enable_global_recall_ ? "开启" : "关闭");
        localization_thread_ = std::thread(&GlobalLocalizationNode::localizationLoop, this);
        manual_localization_thread_ = std::thread(&GlobalLocalizationNode::manualLocalizationLoop, this);
    }
    void localizationLoop()
    {
        while (ros::ok() && !stop_thread_) {
            ros::Duration(1.0 / std::max(freq_localization_, 0.1)).sleep();
            ros::TimerEvent event;
            timerCallback(event);
        }
    }

    void manualLocalizationLoop()
    {
        while (ros::ok() && !stop_thread_) {
            {
                std::unique_lock<std::mutex> lock(manual_request_mutex_);
                manual_request_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
                    return stop_thread_.load() ||
                           (manual_pose_pending_.load() && manual_pose_seq_.load() != manual_pose_applied_seq_.load());
                });
            }
            if (stop_thread_) {
                break;
            }
            const bool manual_pending = manual_pose_pending_.load();
            const int manual_seq = manual_pose_seq_.load();
            const int manual_applied_seq = manual_pose_applied_seq_.load();
            if (!manual_pending || manual_seq == manual_applied_seq) {
                continue;
            }

            LocalizationSnapshot snapshot = captureSnapshot();
            if (!snapshot.have_map || !snapshot.have_scan || !snapshot.cur_scan) {
                continue;
            }

            Eigen::Matrix4d manual_pose = Eigen::Matrix4d::Identity();
            {
                std::lock_guard<std::mutex> manual_lock(manual_pose_mutex_);
                manual_pose = manual_initial_pose_;
            }

            if (globalLocalization(snapshot, manual_pose, LocalizationRequestSource::Manual, manual_seq)) {
                manual_pose_applied_seq_.store(manual_seq);
                manual_pose_pending_.store(false);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    auto_init_ = false;
                }
                ROS_INFO("手动初始位姿已生效，自动初始化已停止");
            }
        }
    }

    ~GlobalLocalizationNode()
    {
        stop_thread_ = true;
        manual_request_cv_.notify_all();
        if (localization_thread_.joinable()) {
            localization_thread_.join();
        }
        if (manual_localization_thread_.joinable()) {
            manual_localization_thread_.join();
        }
    }

private:
    struct LocalizationSnapshot
    {
        CloudTPtr global_map;
        CloudTPtr cur_scan;
        nav_msgs::Odometry latest_odom;
        bool initialized{false};
        bool have_map{false};
        bool have_scan{false};
        bool have_odom{false};
        bool relocalization_mode{false};
        int consecutive_failures{0};
        int low_confidence_streak{0};
        Eigen::Matrix4d T_map_to_odom{Eigen::Matrix4d::Identity()};
    };

    LocalizationSnapshot captureSnapshot()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        LocalizationSnapshot snapshot;
        snapshot.global_map = global_map_;
        snapshot.cur_scan = cur_scan_;
        snapshot.latest_odom = latest_odom_;
        snapshot.initialized = initialized_;
        snapshot.have_map = have_map_;
        snapshot.have_scan = have_scan_;
        snapshot.have_odom = have_odom_;
        snapshot.relocalization_mode = relocalization_mode_;
        snapshot.consecutive_failures = consecutive_failures_;
        snapshot.low_confidence_streak = low_confidence_streak_;
        snapshot.T_map_to_odom = T_map_to_odom_;
        return snapshot;
    }

    void loadGlobalMap(const sensor_msgs::PointCloud2& msg)
    {
        CloudT::Ptr raw = fast_lio_localization_cpp::rosMsgToLocCloudXYZ(msg);
        *global_map_ = *voxelDownsample(raw, map_voxel_size_);
        have_map_ = true;
        buildScanContextDatabase();
        ROS_INFO("全局地图已接收：%zu 个点", global_map_->size());
    }
    void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_scan_msg_ = *msg;
        latest_scan_msg_.header.frame_id = "camera_init";
        latest_scan_msg_.header.stamp = ros::Time::now();
        pub_pc_in_map_.publish(latest_scan_msg_);
        CloudT::Ptr scan = fast_lio_localization_cpp::rosMsgToLocCloudXYZ(*msg);
        cur_scan_ = scan;
        have_scan_ = true;
    }
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_odom_ = *msg;
        have_odom_ = true;
    }
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
        Eigen::Matrix4d manual_pose = poseMsgToMat(*msg);
        {
            std::lock_guard<std::mutex> manual_lock(manual_pose_mutex_);
            manual_initial_pose_ = manual_pose;
        }
        manual_pose_seq_.fetch_add(1);
        manual_pose_pending_.store(true);
        manual_request_cv_.notify_one();
        ROS_INFO("收到手动初始位姿，等待匹配生效");
    }
    ros::Time publishStamp() const
    {
        return have_odom_ ? latest_odom_.header.stamp : ros::Time::now();
    }
    void publishLocalizationValid(bool valid)
    {
        std_msgs::Bool msg;
        msg.data = valid;
        pub_localization_valid_.publish(msg);
    }

    bool shouldResetFastLio() const
    {
        if (!initialized_ || !have_odom_) {
            return false;
        }
        const auto& twist = latest_odom_.twist.twist;
        const double linear_speed = std::sqrt(
            twist.linear.x * twist.linear.x +
            twist.linear.y * twist.linear.y +
            twist.linear.z * twist.linear.z);
        const double angular_speed = std::sqrt(
            twist.angular.x * twist.angular.x +
            twist.angular.y * twist.angular.y +
            twist.angular.z * twist.angular.z);
        return linear_speed >= drift_reset_linear_speed_th_ || angular_speed >= drift_reset_angular_speed_th_;
    }

    void triggerFastLioResetIfNeeded()
    {
        if (!shouldResetFastLio()) {
            return;
        }
        const ros::Time now = ros::Time::now();
        if (!last_fastlio_reset_time_.isZero() && (now - last_fastlio_reset_time_).toSec() < fastlio_reset_cooldown_) {
            return;
        }
        std_msgs::Empty msg;
        pub_fastlio_reset_.publish(msg);
        last_fastlio_reset_time_ = now;
        ROS_WARN("检测到定位失效且里程计仍有速度，已触发 FAST_LIO 重置");
    }

    void publishMapToOdom(const Eigen::Matrix4d& T_map_to_odom, const ros::Time& stamp)
    {
        nav_msgs::Odometry out = matToOdom(T_map_to_odom, std_msgs::Header(), "odom");
        out.header.stamp = stamp;
        out.header.frame_id = "map";
        out.child_frame_id = "odom";
        pub_map_to_odom_.publish(out);
    }
    CloudTPtr cropGlobalMapInFOV(const LocalizationSnapshot& snapshot, const Eigen::Matrix4d& pose_estimation)
    {
        CloudTPtr crop(new CloudT());
        if (!snapshot.have_map || !snapshot.global_map || snapshot.global_map->empty() || !snapshot.have_odom) {
            if (snapshot.global_map) {
                *crop = *snapshot.global_map;
            }
            return crop;
        }
        const Eigen::Matrix4d T_odom_to_base = odomMsgToMat(snapshot.latest_odom);
        const Eigen::Matrix4d T_map_to_base = pose_estimation * T_odom_to_base;
        const Eigen::Matrix4d T_base_to_map = inverseSe3(T_map_to_base);
        // First crop a local submap around the current pose estimate, then apply the FOV filter.
        // This keeps normal tracking precise and avoids matching against a too-large map slice.
        CloudTPtr local_map = cropCloudAroundCenter(
            snapshot.global_map,
            Eigen::Vector3d(T_map_to_base(0, 3), T_map_to_base(1, 3), T_map_to_base(2, 3)),
            local_crop_radius_xy_,
            local_crop_radius_z_);
        if (!local_map || local_map->empty()) {
            local_map = snapshot.global_map;
        }
        CloudTPtr transformed(new CloudT());
        pcl::transformPointCloud(*local_map, *transformed, T_base_to_map.cast<float>());
        crop->reserve(transformed->size());
        const bool is_cylindrical = (fov_ > M_PI);
        const double half_fov = fov_ * 0.5;
        const double far2 = fov_far_ * fov_far_;
        for (std::size_t i = 0; i < transformed->points.size(); ++i) {
            const auto& p_local = transformed->points[i];
            const double x = p_local.x;
            const double y = p_local.y;
            if (is_cylindrical) {
                if (x < fov_far_ && std::abs(std::atan2(y, x)) < half_fov) {
                    crop->push_back(local_map->points[i]);
                }
            } else {
                if (x > 0.0 && x < fov_far_ && (x * x + y * y) < far2 && std::abs(std::atan2(y, x)) < half_fov) {
                    crop->push_back(local_map->points[i]);
                }
            }
        }
        if (!crop->empty()) {
            sensor_msgs::PointCloud2 msg = makePointCloudMsg(crop, snapshot.latest_odom.header);
            msg.header.frame_id = "map";
            pub_submap_.publish(msg);
        }
        return crop;
    }

    std::vector<Eigen::Vector2d> buildGlobalSearchCandidateCenters() const
    {
        std::vector<Eigen::Vector2d> centers;
        if (!global_map_ || global_map_->empty() || global_search_grid_ <= 1e-6) {
            return centers;
        }

        std::map<std::pair<int, int>, int> occupied;
        int min_ix = std::numeric_limits<int>::max();
        int min_iy = std::numeric_limits<int>::max();
        int max_ix = std::numeric_limits<int>::lowest();
        int max_iy = std::numeric_limits<int>::lowest();
        for (const auto& p : global_map_->points) {
            const int ix = static_cast<int>(std::floor(static_cast<double>(p.x) / global_search_grid_));
            const int iy = static_cast<int>(std::floor(static_cast<double>(p.y) / global_search_grid_));
            occupied[{ix, iy}] += 1;
            min_ix = std::min(min_ix, ix);
            min_iy = std::min(min_iy, iy);
            max_ix = std::max(max_ix, ix);
            max_iy = std::max(max_iy, iy);
        }

        std::vector<std::pair<std::pair<int, int>, int>> bins;
        bins.reserve(occupied.size());
        for (int ix = min_ix; ix <= max_ix; ++ix) {
            for (int iy = min_iy; iy <= max_iy; ++iy) {
                int support = 0;
                for (int dx = -global_search_support_radius_bins_; dx <= global_search_support_radius_bins_; ++dx) {
                    for (int dy = -global_search_support_radius_bins_; dy <= global_search_support_radius_bins_; ++dy) {
                        auto it = occupied.find({ix + dx, iy + dy});
                        if (it != occupied.end()) {
                            support += it->second;
                        }
                    }
                }
                if (support >= global_search_min_points_per_bin_) {
                    bins.push_back({{ix, iy}, support});
                }
            }
        }

        if (bins.empty()) {
            for (const auto& kv : occupied) {
                bins.push_back({kv.first, kv.second});
            }
        }

        std::sort(bins.begin(), bins.end(), [](const auto& a, const auto& b) {
            return a.second > b.second;
        });

        const std::size_t limit = std::min<std::size_t>(static_cast<std::size_t>(std::max(1, global_search_max_candidates_)), bins.size());
        centers.reserve(limit);
        for (std::size_t i = 0; i < limit; ++i) {
            const auto& bin = bins[i].first;
            centers.emplace_back((static_cast<double>(bin.first) + 0.5) * global_search_grid_,
                                 (static_cast<double>(bin.second) + 0.5) * global_search_grid_);
        }
        return centers;
    }

    DescriptorT makeScanContextDescriptor(const CloudTPtr& cloud, const Eigen::Vector2d* center_xy = nullptr) const
    {
        DescriptorT desc = DescriptorT::Zero(scan_context_num_rings_, scan_context_num_sectors_);
        if (!cloud || cloud->empty()) {
            return desc;
        }

        float min_z = std::numeric_limits<float>::infinity();
        for (const auto& p : cloud->points) {
            min_z = std::min(min_z, p.z);
        }

        for (const auto& p : cloud->points) {
            double x = static_cast<double>(p.x);
            double y = static_cast<double>(p.y);
            if (center_xy != nullptr) {
                x -= (*center_xy)(0);
                y -= (*center_xy)(1);
            }
            const double radius = std::sqrt(x * x + y * y);
            if (radius <= 1e-3 || radius >= scan_context_max_radius_) {
                continue;
            }
            const double azimuth_deg = std::fmod(std::atan2(y, x) * 180.0 / M_PI + 360.0, 360.0);
            int ring_idx = static_cast<int>(radius / scan_context_max_radius_ * scan_context_num_rings_);
            int sector_idx = static_cast<int>(azimuth_deg / 360.0 * scan_context_num_sectors_);
            ring_idx = std::max(0, std::min(scan_context_num_rings_ - 1, ring_idx));
            sector_idx = std::max(0, std::min(scan_context_num_sectors_ - 1, sector_idx));
            const float z_value = static_cast<float>(
                std::max(0.0, std::min(scan_context_z_clip_, static_cast<double>(p.z - min_z) + scan_context_z_offset_)));
            desc(ring_idx, sector_idx) = std::max(desc(ring_idx, sector_idx), z_value);
        }
        return desc;
    }

    RingKeyT makeScanContextRingKey(const DescriptorT& desc) const
    {
        RingKeyT key(desc.rows());
        for (int r = 0; r < desc.rows(); ++r) {
            key(r) = desc.row(r).mean();
        }
        return key;
    }

    std::pair<double, int> scanContextDistance(const DescriptorT& a, const DescriptorT& b) const
    {
        Eigen::VectorXf a_norm = a.colwise().norm().transpose();
        double best_dist = std::numeric_limits<double>::infinity();
        int best_shift = 0;
        for (int shift = 0; shift < scan_context_num_sectors_; ++shift) {
            double sim_sum = 0.0;
            int sim_count = 0;
            for (int c = 0; c < scan_context_num_sectors_; ++c) {
                const int bc = (c + shift) % scan_context_num_sectors_;
                const float b_norm = b.col(bc).norm();
                if (a_norm(c) <= 1e-6f || b_norm <= 1e-6f) {
                    continue;
                }
                sim_sum += static_cast<double>(a.col(c).dot(b.col(bc))) /
                           (static_cast<double>(a_norm(c)) * static_cast<double>(b_norm));
                sim_count += 1;
            }
            if (sim_count == 0) {
                continue;
            }
            const double dist = 1.0 - sim_sum / static_cast<double>(sim_count);
            if (dist < best_dist) {
                best_dist = dist;
                best_shift = shift;
            }
        }
        return {best_dist, best_shift};
    }

    void buildScanContextDatabase()
    {
        global_search_candidate_centers_ = buildGlobalSearchCandidateCenters();
        global_scan_context_descriptors_.clear();
        global_scan_context_ring_keys_.clear();

        if (!enable_scan_context_ || global_search_candidate_centers_.empty()) {
            return;
        }

        std::vector<Eigen::Vector2d> kept_centers;
        kept_centers.reserve(global_search_candidate_centers_.size());
        for (const auto& center : global_search_candidate_centers_) {
            CloudTPtr submap = cropCloudAroundCenter(
                global_map_,
                Eigen::Vector3d(center(0), center(1), 0.0),
                global_search_submap_radius_,
                1e6);
            if (!submap || static_cast<int>(submap->size()) < recall_min_crop_points_) {
                continue;
            }
            DescriptorT desc = makeScanContextDescriptor(submap, &center);
            if (desc.size() == 0 || desc.maxCoeff() <= 0.0f) {
                continue;
            }
            kept_centers.push_back(center);
            global_scan_context_descriptors_.push_back(desc);
            global_scan_context_ring_keys_.push_back(makeScanContextRingKey(desc));
        }
        if (!kept_centers.empty()) {
            global_search_candidate_centers_ = std::move(kept_centers);
        }
    }

    std::vector<ScanContextMatch> searchScanContextCandidates(const CloudTPtr& scan) const
    {
        std::vector<ScanContextMatch> candidates;
        if (!enable_scan_context_ || global_scan_context_descriptors_.empty() || !scan || scan->empty()) {
            return candidates;
        }

        DescriptorT scan_desc = makeScanContextDescriptor(scan, nullptr);
        if (scan_desc.size() == 0 || scan_desc.maxCoeff() <= 0.0f) {
            return candidates;
        }

        RingKeyT scan_key = makeScanContextRingKey(scan_desc);
        std::vector<std::pair<double, std::size_t>> key_errors;
        key_errors.reserve(global_scan_context_ring_keys_.size());
        for (std::size_t i = 0; i < global_scan_context_ring_keys_.size(); ++i) {
            const double err = (global_scan_context_ring_keys_[i] - scan_key).norm();
            key_errors.emplace_back(err, i);
        }
        if (key_errors.empty()) {
            return candidates;
        }

        std::sort(key_errors.begin(), key_errors.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
        const int shortlist = std::min<int>(std::max(1, scan_context_ringkey_topk_), static_cast<int>(key_errors.size()));
        for (int i = 0; i < shortlist; ++i) {
            const std::size_t idx = key_errors[static_cast<std::size_t>(i)].second;
            const auto dist_shift = scanContextDistance(scan_desc, global_scan_context_descriptors_[idx]);
            if (!std::isfinite(dist_shift.first)) {
                continue;
            }
            candidates.push_back(ScanContextMatch{idx, dist_shift.first, dist_shift.second});
        }

        std::sort(candidates.begin(), candidates.end(), [](const ScanContextMatch& a, const ScanContextMatch& b) {
            return a.distance < b.distance;
        });
        if (static_cast<int>(candidates.size()) > scan_context_refine_topk_) {
            candidates.resize(static_cast<std::size_t>(scan_context_refine_topk_));
        }
        return candidates;
    }

    bool globalRecallSearch(const LocalizationSnapshot& snapshot,
                          const CloudTPtr& scan,
                          Eigen::Matrix4d& best_transformation,
                          double& best_score)
    {
        if (!snapshot.have_map || !snapshot.global_map || !scan || scan->empty() || snapshot.global_map->empty()) {
            return false;
        }
        CloudTPtr recall_map = voxelDownsample(snapshot.global_map, recall_map_voxel_size_);
        CloudTPtr scan_ds = voxelDownsample(scan, scan_voxel_size_ * recall_scan_scale_);
        if (recall_map->empty() || scan_ds->empty()) {
            return false;
        }
        struct Candidate {
            double coarse_conf;
            double coarse_score;
            Eigen::Matrix4d coarse_tf;
            CloudTPtr crop;
        };
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*recall_map, min_pt, max_pt);
        best_score = 0.0;
        bool found = false;
        CloudTPtr best_crop(new CloudT());

        if (enable_scan_context_ && !global_scan_context_descriptors_.empty()) {
            ROS_WARN("召回模式：Scan Context 候选搜索");
            const auto sc_candidates = searchScanContextCandidates(scan_ds);
            for (const auto& cand : sc_candidates) {
                if (cand.index >= global_search_candidate_centers_.size()) {
                    continue;
                }
                const auto& center = global_search_candidate_centers_[cand.index];
                CloudTPtr crop = cropCloudAroundCenter(
                    recall_map,
                    Eigen::Vector3d(center(0), center(1), 0.0),
                    global_search_submap_radius_,
                    1e6);
                if (!crop || static_cast<int>(crop->size()) < recall_min_crop_points_) {
                    continue;
                }

                double sum_z = 0.0;
                for (const auto& p : crop->points) {
                    sum_z += static_cast<double>(p.z);
                }
                const double mean_z = crop->empty() ? 0.0 : (sum_z / static_cast<double>(crop->size()));
                const double yaw_shift = -2.0 * M_PI * static_cast<double>(cand.sector_shift) / static_cast<double>(scan_context_num_sectors_);
                Eigen::Matrix4d initial = makeInitial(center(0), center(1), mean_z, 0.0, 0.0, yaw_shift);

                auto coarse = icpRegister(scan_ds, crop, initial, scan_voxel_size_ * recall_coarse_scale_, recall_map_voxel_size_ * recall_coarse_scale_, 1.0 * recall_coarse_scale_, 20);
                if (!std::isfinite(coarse.second)) {
                    continue;
                }
                auto fine = icpRegister(scan_ds, crop, coarse.first, scan_voxel_size_, recall_map_voxel_size_, 1.0, 20);
                const double score = std::isfinite(fine.second) ? fine.second : coarse.second;
                const double conf = confidenceFromFitness(score);
                if (conf > best_score) {
                    best_score = conf;
                    best_transformation = std::isfinite(fine.second) ? fine.first : coarse.first;
                    best_crop = crop;
                    found = true;
                }
            }
            if (found && !best_crop->empty()) {
                sensor_msgs::PointCloud2 msg = makePointCloudMsg(best_crop, snapshot.latest_odom.header);
                msg.header.frame_id = "map";
                pub_submap_.publish(msg);
                return true;
            }

            return false;
        }

        ROS_WARN("Scan Context 候选未命中，回退到全图网格搜索");
        std::vector<Candidate> coarse_candidates;
        coarse_candidates.reserve(64);

        for (double z = min_pt[2]; z <= max_pt[2]; z += recall_z_step_) {
            for (double x = min_pt[0]; x <= max_pt[0]; x += recall_grid_step_) {
                for (double y = min_pt[1]; y <= max_pt[1]; y += recall_grid_step_) {
                    CloudTPtr crop = cropCloudAroundCenter(recall_map, Eigen::Vector3d(x, y, z), recall_crop_radius_, recall_crop_radius_z_);
                    if (!crop || static_cast<int>(crop->size()) < recall_min_crop_points_) {
                        continue;
                    }
                    for (double yaw : candidateYaws()) {
                        Eigen::Matrix4d initial = makeInitial(x, y, z, 0.0, 0.0, yaw);
                        auto coarse = icpRegister(scan_ds, crop, initial, scan_voxel_size_ * recall_coarse_scale_, recall_map_voxel_size_ * recall_coarse_scale_, 1.0 * recall_coarse_scale_, 20);
                        if (!std::isfinite(coarse.second)) {
                            continue;
                        }
                        const double coarse_conf = confidenceFromFitness(coarse.second);
                        coarse_candidates.push_back(Candidate{coarse_conf, coarse.second, coarse.first, crop});
                    }
                }
            }
        }

        if (coarse_candidates.empty()) {
            return false;
        }
        std::sort(coarse_candidates.begin(), coarse_candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.coarse_conf > b.coarse_conf;
        });
        const int topk = std::min<int>(std::max(1, recall_fine_topk_), static_cast<int>(coarse_candidates.size()));
        for (int i = 0; i < topk; ++i) {
            const Candidate& cand = coarse_candidates[i];
            auto fine = icpRegister(scan_ds, cand.crop, cand.coarse_tf, scan_voxel_size_, recall_map_voxel_size_, 1.0, 20);
            const double score = std::isfinite(fine.second) ? fine.second : cand.coarse_score;
            const double conf = confidenceFromFitness(score);
            if (conf > best_score) {
                best_score = conf;
                best_transformation = std::isfinite(fine.second) ? fine.first : cand.coarse_tf;
                best_crop = cand.crop;
                found = true;
            }
        }
        if (found && !best_crop->empty()) {
            sensor_msgs::PointCloud2 msg = makePointCloudMsg(best_crop, snapshot.latest_odom.header);
            msg.header.frame_id = "map";
            pub_submap_.publish(msg);
        }
        return found;
    }

    bool strongLocalRecallSearch(const LocalizationSnapshot& snapshot, const Eigen::Matrix4d& pose_estimation, Eigen::Matrix4d& best_transformation, double& best_confidence, double& best_score)
    {
        if (!snapshot.have_map || !snapshot.global_map || !snapshot.cur_scan || snapshot.cur_scan->empty() || snapshot.global_map->empty()) {
            return false;
        }
        const double base_x = pose_estimation(0, 3);
        const double base_y = pose_estimation(1, 3);
        const double base_z = pose_estimation(2, 3);
        const double base_yaw = yawFromTransform(pose_estimation);
        CloudTPtr scan_ds = voxelDownsample(snapshot.cur_scan, scan_voxel_size_ * 2.0);
        if (!scan_ds || scan_ds->empty()) {
            return false;
        }
        best_confidence = 0.0;
        best_score = std::numeric_limits<double>::infinity();
        bool found = false;
        CloudTPtr best_crop(new CloudT());
        const double yaw_step = strong_local_recall_yaw_step_deg_ * M_PI / 180.0;
        const double yaw_radius = strong_local_recall_yaw_radius_deg_ * M_PI / 180.0;
        for (double dx = -strong_local_recall_xy_radius_; dx <= strong_local_recall_xy_radius_ + 1e-6; dx += strong_local_recall_xy_step_) {
            for (double dy = -strong_local_recall_xy_radius_; dy <= strong_local_recall_xy_radius_ + 1e-6; dy += strong_local_recall_xy_step_) {
                for (double dyaw = -yaw_radius; dyaw <= yaw_radius + 1e-6; dyaw += yaw_step) {
                    const double cx = base_x + dx;
                    const double cy = base_y + dy;
                    const double cyaw = base_yaw + dyaw;
                    Eigen::Matrix4d initial = makeInitial(cx, cy, base_z, 0.0, 0.0, cyaw);
                    CloudTPtr crop = cropCloudAroundCenter(snapshot.global_map, Eigen::Vector3d(cx, cy, base_z), recall_crop_radius_, recall_crop_radius_z_);
                    if (!crop || crop->empty()) {
                        continue;
                    }
                    auto coarse = icpRegister(scan_ds, crop, initial, scan_voxel_size_ * 3.0, map_voxel_size_ * 3.0, 3.0, 20);
                    if (!std::isfinite(coarse.second)) {
                        continue;
                    }
                    auto fine = icpRegister(snapshot.cur_scan, crop, coarse.first, scan_voxel_size_, map_voxel_size_, 1.0, 20);
                    const double score = std::isfinite(fine.second) ? fine.second : coarse.second;
                    const double conf = confidenceFromFitness(score);
                    if (conf > best_confidence) {
                        best_confidence = conf;
                        best_score = score;
                        best_transformation = std::isfinite(fine.second) ? fine.first : coarse.first;
                        best_crop = crop;
                        found = true;
                    }
                }
            }
        }
        if (found && !best_crop->empty()) {
            sensor_msgs::PointCloud2 msg = makePointCloudMsg(best_crop, snapshot.latest_odom.header);
            msg.header.frame_id = "map";
            pub_submap_.publish(msg);
        }
        return found;
    }

    bool manualPoseLocalSearch(const LocalizationSnapshot& snapshot,
                               const Eigen::Matrix4d& manual_pose,
                               int manual_request_seq,
                               Eigen::Matrix4d& best_transformation,
                               double& best_confidence,
                               double& best_score)
    {
        if (!snapshot.have_map || !snapshot.global_map || !snapshot.cur_scan || snapshot.cur_scan->empty() || snapshot.global_map->empty()) {
            return false;
        }
        const double base_x = manual_pose(0, 3);
        const double base_y = manual_pose(1, 3);
        const double base_z = manual_pose(2, 3);
        const double base_yaw = yawFromTransform(manual_pose);
        CloudTPtr scan_ds = voxelDownsample(snapshot.cur_scan, scan_voxel_size_ * 2.0);
        if (!scan_ds || scan_ds->empty()) {
            return false;
        }

        CloudTPtr crop = cropCloudAroundCenter(
            snapshot.global_map,
            Eigen::Vector3d(base_x, base_y, base_z),
            std::max(local_crop_radius_xy_, manual_local_search_xy_radius_ + 1.0),
            std::max(local_crop_radius_z_, manual_local_search_z_radius_ + 0.5));
        if (!crop || static_cast<int>(crop->size()) < recall_min_crop_points_) {
            return false;
        }
        CloudTPtr crop_ds = voxelDownsample(crop, map_voxel_size_ * 2.0);
        if (!crop_ds || crop_ds->empty()) {
            return false;
        }

        struct Candidate {
            double conf;
            double score;
            Eigen::Matrix4d tf;
        };
        std::vector<Candidate> coarse_candidates;
        coarse_candidates.reserve(128);

        const double xy_step = manual_local_search_xy_step_;
        const double xy_radius = manual_local_search_xy_radius_;
        const double yaw_step = manual_local_search_yaw_step_deg_ * M_PI / 180.0;
        const double yaw_radius = manual_local_search_yaw_radius_deg_ * M_PI / 180.0;
        const double z_step = std::max(1e-6, manual_local_search_z_step_);

        for (double dx = -xy_radius; dx <= xy_radius + 1e-6; dx += xy_step) {
            if (isManualRequestStale(manual_request_seq)) {
                return false;
            }
            for (double dy = -xy_radius; dy <= xy_radius + 1e-6; dy += xy_step) {
                if (isManualRequestStale(manual_request_seq)) {
                    return false;
                }
                for (double dz = -manual_local_search_z_radius_; dz <= manual_local_search_z_radius_ + 1e-6; dz += z_step) {
                    if (isManualRequestStale(manual_request_seq)) {
                        return false;
                    }
                    for (double dyaw = -yaw_radius; dyaw <= yaw_radius + 1e-6; dyaw += yaw_step) {
                        if (isManualRequestStale(manual_request_seq)) {
                            return false;
                        }
                        const double cx = base_x + dx;
                        const double cy = base_y + dy;
                        const double cz = base_z + dz;
                        const double cyaw = base_yaw + dyaw;
                        Eigen::Matrix4d initial = makeInitial(cx, cy, cz, 0.0, 0.0, cyaw);
                        auto coarse = icpRegister(scan_ds, crop_ds, initial, scan_voxel_size_ * 2.0, map_voxel_size_ * 2.0, 2.0, 8);
                        if (!std::isfinite(coarse.second)) {
                            continue;
                        }
                        coarse_candidates.push_back(Candidate{confidenceFromFitness(coarse.second), coarse.second, coarse.first});
                    }
                }
            }
        }
        if (coarse_candidates.empty()) {
            return false;
        }
        std::sort(coarse_candidates.begin(), coarse_candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.conf > b.conf;
        });

        best_confidence = 0.0;
        best_score = std::numeric_limits<double>::infinity();
        bool found = false;
        const int topk = std::min<int>(std::max(1, manual_local_search_topk_), static_cast<int>(coarse_candidates.size()));
        for (int i = 0; i < topk; ++i) {
            if (isManualRequestStale(manual_request_seq)) {
                return false;
            }
            const Candidate& cand = coarse_candidates[i];
            auto fine = icpRegister(snapshot.cur_scan, crop, cand.tf, scan_voxel_size_, map_voxel_size_, 1.0, 12);
            const double score = std::isfinite(fine.second) ? fine.second : cand.score;
            const double conf = confidenceFromFitness(score);
            if (conf > best_confidence) {
                best_confidence = conf;
                best_score = score;
                best_transformation = std::isfinite(fine.second) ? fine.first : cand.tf;
                found = true;
            }
        }
        if (found) {
            sensor_msgs::PointCloud2 msg = makePointCloudMsg(crop, snapshot.latest_odom.header);
            msg.header.frame_id = "map";
            pub_submap_.publish(msg);
        }
        return found;
    }

    void publishOdomErrorIfNeeded()
    {
        if (!initialized_) {
            return;
        }
        const ros::Time now = ros::Time::now();
        if (last_odom_error_log_.isZero() || (now - last_odom_error_log_).toSec() >= odom_error_log_period_) {
            ROS_WARN("Odometry error!!!");
            last_odom_error_log_ = now;
        }
    }

    bool localMatch(const LocalizationSnapshot& snapshot, const Eigen::Matrix4d& pose_estimation, Eigen::Matrix4d& result, double& confidence, double& score)
    {
        CloudTPtr target = cropGlobalMapInFOV(snapshot, pose_estimation);
        if (!target || target->empty()) {
            return false;
        }
        auto coarse = icpRegister(snapshot.cur_scan, target, pose_estimation, scan_voxel_size_ * 3.0, map_voxel_size_ * 3.0, 3.0, 20);
        if (!std::isfinite(coarse.second)) {
            return false;
        }
        auto fine = icpRegister(snapshot.cur_scan, target, coarse.first, scan_voxel_size_, map_voxel_size_, 1.0, 20);
        const double final_score = std::isfinite(fine.second) ? fine.second : coarse.second;
        result = std::isfinite(fine.second) ? fine.first : coarse.first;
        score = final_score;
        confidence = confidenceFromFitness(final_score);
        return true;
    }
    bool shouldDiscardAutomaticResult(int observed_manual_applied_seq) const
    {
        return manual_pose_applied_seq_.load() != observed_manual_applied_seq;
    }

    bool isManualRequestStale(int request_seq) const
    {
        return manual_pose_seq_.load() != request_seq;
    }

    bool globalLocalization(const LocalizationSnapshot& snapshot,
                            const Eigen::Matrix4d& pose_estimation,
                            LocalizationRequestSource source,
                            int observed_manual_applied_seq)
    {
        if (!snapshot.have_map || !snapshot.have_scan || !snapshot.cur_scan) {
            return false;
        }
        auto start = std::chrono::steady_clock::now();
        ROS_INFO(source == LocalizationRequestSource::Manual ? "开始进行手动局部重定位（粗位姿邻域配准）..." : "开始进行全局重定位（scan-to-map 配准）...");
        Eigen::Matrix4d transformation = pose_estimation;
        double confidence = 0.0;
        double score = std::numeric_limits<double>::infinity();

        const bool force_global_init = !snapshot.initialized;
        const bool use_strong_local_recall = snapshot.initialized && !snapshot.relocalization_mode &&
                                             snapshot.low_confidence_streak >= strong_local_recall_trigger_streak_;
        const int manual_request_seq = observed_manual_applied_seq;

        if (source == LocalizationRequestSource::Manual) {
            if (!manualPoseLocalSearch(snapshot, pose_estimation, manual_request_seq, transformation, confidence, score)) {
                auto end = std::chrono::steady_clock::now();
                ROS_INFO(source == LocalizationRequestSource::Manual ? "手动局部重定位耗时：%.3fs" : "本次配准耗时：%.3fs", std::chrono::duration<double>(end - start).count());
                ROS_WARN("手动粗位姿附近没有找到有效候选");
                return false;
            }
        } else if ((force_global_init || snapshot.relocalization_mode) && enable_global_recall_) {
            ROS_WARN("召回模式：在整张地图上做多候选全局搜索");
            if (!globalRecallSearch(snapshot, snapshot.cur_scan, transformation, confidence)) {
                if (source == LocalizationRequestSource::Automatic && shouldDiscardAutomaticResult(observed_manual_applied_seq)) {
                    return false;
                }
                auto end = std::chrono::steady_clock::now();
                ROS_INFO("本次配准耗时：%.3fs", std::chrono::duration<double>(end - start).count());
                ROS_WARN("整图搜索没有找到有效候选");
                ROS_WARN("实时置信度：0.0%%（匹配分数=0.0000，越大越好）");
                publishLocalizationValid(false);
                triggerFastLioResetIfNeeded();
                publishOdomErrorIfNeeded();
                low_confidence_streak_ = 0;
                relocalization_stable_success_count_ = 0;
                { std::lock_guard<std::mutex> lock(mutex_); consecutive_failures_ += 1; if (enable_global_recall_ && consecutive_failures_ >= failures_before_recovery_) global_recall_success_log_emitted_ = false; }
                return false;
            }
            score = (confidence > 0.0) ? (1.0 / confidence - 1.0) : std::numeric_limits<double>::infinity();
        } else if (use_strong_local_recall) {
            ROS_WARN("连续低置信度，进入强局部召回搜索");
            if (!strongLocalRecallSearch(snapshot, pose_estimation, transformation, confidence, score)) {
                if (source == LocalizationRequestSource::Automatic && shouldDiscardAutomaticResult(observed_manual_applied_seq)) {
                    return false;
                }
                auto end = std::chrono::steady_clock::now();
                ROS_INFO("本次配准耗时：%.3fs", std::chrono::duration<double>(end - start).count());
                ROS_WARN("强局部召回没有找到有效候选");
                publishLocalizationValid(false);
                triggerFastLioResetIfNeeded();
                publishOdomErrorIfNeeded();
                { std::lock_guard<std::mutex> lock(mutex_); consecutive_failures_ += 1; relocalization_mode_ = enable_global_recall_ && consecutive_failures_ >= failures_before_recovery_; if (relocalization_mode_) global_recall_success_log_emitted_ = false; }
                return false;
            }
        } else {
            if (!localMatch(snapshot, pose_estimation, transformation, confidence, score)) {
                if (source == LocalizationRequestSource::Automatic && shouldDiscardAutomaticResult(observed_manual_applied_seq)) {
                    return false;
                }
                auto end = std::chrono::steady_clock::now();
                ROS_INFO("本次配准耗时：%.3fs", std::chrono::duration<double>(end - start).count());
                ROS_WARN("当前帧与地图匹配不通过");
                ROS_WARN("变换矩阵：%s", matToString(transformation).c_str());
                ROS_WARN("实时置信度：0.0%%（匹配分数=0.0000，越大越好）");
                publishLocalizationValid(false);
                triggerFastLioResetIfNeeded();
                publishOdomErrorIfNeeded();
                low_confidence_streak_ = 0;
                relocalization_stable_success_count_ = 0;
                { std::lock_guard<std::mutex> lock(mutex_); consecutive_failures_ += 1; relocalization_mode_ = enable_global_recall_ && consecutive_failures_ >= failures_before_recovery_; if (relocalization_mode_) global_recall_success_log_emitted_ = false; }
                return false;
            }
            score = (confidence > 0.0) ? (1.0 / confidence - 1.0) : std::numeric_limits<double>::infinity();
        }

        if (source == LocalizationRequestSource::Automatic && shouldDiscardAutomaticResult(observed_manual_applied_seq)) {
            return false;
        }
        auto end = std::chrono::steady_clock::now();
        ROS_INFO("本次配准耗时：%.3fs", std::chrono::duration<double>(end - start).count());
        const double confidence_percent = confidence * 100.0;
        const double success_th = ((force_global_init || snapshot.relocalization_mode) && enable_global_recall_) ? global_recall_success_th_ : localization_th_;
        if (confidence_percent >= success_th * 100.0) {
            bool use_stabilizer = snapshot.initialized && !snapshot.relocalization_mode && !use_strong_local_recall && manual_pose_seq_.load() == manual_pose_applied_seq_.load();
            if (use_stabilizer) {
                const double dtrans = translationDistance(snapshot.T_map_to_odom, transformation);
                const double drot = rotationDistanceDeg(snapshot.T_map_to_odom, transformation);
                const bool hold_motion = dtrans <= stabilize_hold_translation_ && drot <= stabilize_hold_rotation_deg_ && confidence >= stabilize_confidence_gate_;
                const bool small_motion = dtrans <= stabilize_small_translation_ && drot <= stabilize_small_rotation_deg_ && confidence >= stabilize_confidence_gate_;
                if (!hold_motion) {
                    const double alpha = small_motion ? stabilize_alpha_small_ : stabilize_alpha_large_;
                    transformation = interpolateTransform(snapshot.T_map_to_odom, transformation, alpha);
                }
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                T_map_to_odom_ = transformation;
                consecutive_failures_ = 0;
                relocalization_mode_ = false;
                initialized_ = true;
            }
            publishMapToOdom(transformation, snapshot.have_odom ? snapshot.latest_odom.header.stamp : ros::Time::now());
            ROS_INFO("实时置信度：%.1f%%（匹配分数=%.4f，越小越好）", confidence_percent, score);

            if (confidence_percent >= 95.0) {
                low_confidence_streak_ = 0;
                publishLocalizationValid(true);
            } else {
                low_confidence_streak_ += 1;
                if (low_confidence_streak_ >= low_confidence_frames_before_invalidate_) {
                    publishLocalizationValid(false);
                    triggerFastLioResetIfNeeded();
                    publishOdomErrorIfNeeded();
                } else {
                    publishLocalizationValid(true);
                }
            }

            if (confidence_percent >= 95.0) {
                relocalization_stable_success_count_ += 1;
            } else {
                relocalization_stable_success_count_ = 0;
            }

            if (!localization_success_log_emitted_ && (confidence_percent >= 97.0 || relocalization_stable_success_count_ >= 2)) {
                ROS_INFO("Initialize successfully!!!!!!");
                localization_success_log_emitted_ = true;
            }

            if (snapshot.relocalization_mode && !global_recall_success_log_emitted_) {
                ROS_INFO("全局召回成功");
                global_recall_success_log_emitted_ = true;
            }
            return true;
        }

        ROS_WARN("重定位失败：原因=位姿跳变过大，置信度=%.1f%%，匹配分数=%.4f，连续失败=%d，召回模式=%s，全局召回=%s",
                 confidence_percent, score, consecutive_failures_ + 1, relocalization_mode_ ? "True" : "False", enable_global_recall_ ? "True" : "False");
        ROS_WARN("变换矩阵：%s", matToString(transformation).c_str());
        ROS_WARN("实时置信度：%.1f%%（匹配分数=%.4f，越小越好）", confidence_percent, score);
        publishLocalizationValid(false);
        triggerFastLioResetIfNeeded();
        publishOdomErrorIfNeeded();
        low_confidence_streak_ = 0;
        relocalization_stable_success_count_ = 0;
        { std::lock_guard<std::mutex> lock(mutex_); consecutive_failures_ += 1; relocalization_mode_ = enable_global_recall_ && consecutive_failures_ >= failures_before_recovery_; }
        return false;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        LocalizationSnapshot snapshot = captureSnapshot();
        if (!snapshot.have_map || !snapshot.have_scan || !snapshot.cur_scan) {
            return;
        }

        const int observed_manual_applied_seq = manual_pose_applied_seq_.load();

        if (!snapshot.initialized) {
            bool auto_init_enabled = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                auto_init_enabled = auto_init_;
            }
            if (auto_init_enabled) {
                if (globalLocalization(snapshot, Eigen::Matrix4d::Identity(), LocalizationRequestSource::Automatic, observed_manual_applied_seq)) {
                    ROS_INFO("初始化成功");
                }
            }
            return;
        }
        if (enable_global_recall_ && snapshot.relocalization_mode) {
            ROS_WARN("进入召回模式，尝试全局召回...");
            if (globalLocalization(snapshot, Eigen::Matrix4d::Identity(), LocalizationRequestSource::Automatic, observed_manual_applied_seq)) {
                std::lock_guard<std::mutex> lock(mutex_);
                relocalization_mode_ = false;
            }
            return;
        }
        globalLocalization(snapshot, snapshot.T_map_to_odom, LocalizationRequestSource::Automatic, observed_manual_applied_seq);
    }
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_initial_;
    ros::Publisher pub_pc_in_map_;
    ros::Publisher pub_submap_;
    ros::Publisher pub_map_to_odom_;
    ros::Publisher pub_localization_valid_;
    ros::Publisher pub_fastlio_reset_;
    std::thread localization_thread_;
    std::thread manual_localization_thread_;
    std::atomic<bool> stop_thread_{false};
    std::mutex mutex_;
    std::mutex manual_pose_mutex_;
    std::mutex manual_request_mutex_;
    std::condition_variable manual_request_cv_;
    CloudTPtr global_map_;
    CloudTPtr cur_scan_;
    sensor_msgs::PointCloud2 latest_scan_msg_;
    nav_msgs::Odometry latest_odom_;
    bool initialized_;
    bool have_map_;
    bool have_scan_;
    bool have_odom_;
    bool enable_global_recall_;
    bool enable_scan_context_;
    bool auto_init_;
    std::atomic<bool> manual_pose_pending_{false};
    bool relocalization_mode_;
    int failures_before_recovery_;
    int consecutive_failures_;
    std::atomic<int> manual_pose_seq_{0};
    std::atomic<int> manual_pose_applied_seq_{-1};
    double map_voxel_size_;
    double scan_voxel_size_;
    double freq_localization_;
    double localization_th_;
    double global_recall_trigger_th_;
    double global_recall_success_th_;
    double fov_;
    double fov_far_;
    double recall_grid_step_;
    double recall_z_step_;
    double recall_crop_radius_;
    double recall_crop_radius_z_;
    double local_crop_radius_xy_;
    double local_crop_radius_z_;
    double stabilize_hold_translation_;
    double stabilize_hold_rotation_deg_;
    double stabilize_small_translation_;
    double stabilize_small_rotation_deg_;
    double stabilize_alpha_small_;
    double stabilize_alpha_large_;
    double stabilize_confidence_gate_;
    double recall_map_voxel_size_;
    double global_search_grid_;
    double global_search_submap_radius_;
    double scan_context_max_radius_;
    double scan_context_z_offset_;
    double scan_context_z_clip_;
    double odom_error_log_period_;
    double drift_reset_linear_speed_th_;
    double drift_reset_angular_speed_th_;
    double fastlio_reset_cooldown_;
    int recall_scan_scale_;
    int recall_coarse_scale_;
    int global_search_max_candidates_;
    int global_search_support_radius_bins_;
    int global_search_min_points_per_bin_;
    int scan_context_num_rings_;
    int scan_context_num_sectors_;
    int scan_context_ringkey_topk_;
    int scan_context_refine_topk_;
    int low_confidence_frames_before_invalidate_;
    double manual_local_search_xy_step_;
    double manual_local_search_xy_radius_;
    double manual_local_search_z_step_;
    double manual_local_search_z_radius_;
    double manual_local_search_yaw_step_deg_;
    double manual_local_search_yaw_radius_deg_;
    int manual_local_search_topk_;
    double strong_local_recall_xy_step_;
    double strong_local_recall_xy_radius_;
    double strong_local_recall_yaw_step_deg_;
    double strong_local_recall_yaw_radius_deg_;
    int strong_local_recall_trigger_streak_;
    int recall_fine_topk_;
    int recall_min_crop_points_;
    bool localization_success_log_emitted_{false};
    bool global_recall_success_log_emitted_{false};
    int relocalization_stable_success_count_{0};
    int low_confidence_streak_{0};
    ros::Time last_odom_error_log_;
    ros::Time last_fastlio_reset_time_;
    Eigen::Matrix4d T_map_to_odom_;
    Eigen::Matrix4d manual_initial_pose_;
    std::vector<Eigen::Vector2d> global_search_candidate_centers_;
    std::vector<DescriptorT> global_scan_context_descriptors_;
    std::vector<RingKeyT> global_scan_context_ring_keys_;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_lio_localization");
    setlocale(LC_ALL, "");
    try {
        GlobalLocalizationNode node;
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_ERROR("fast_lio_localization 退出：%s", e.what());
        return 1;
    }
    return 0;
}
