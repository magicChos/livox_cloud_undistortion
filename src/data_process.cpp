#include "livox_dedistortion_pkg/data_process.h"
#include <nav_msgs/Odometry.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

using Sophus::SE3d;
using Sophus::SO3d;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudtmp(new pcl::PointCloud<pcl::PointXYZI>());

ImuProcess::ImuProcess() : b_first_frame_(true), last_lidar_(nullptr), last_imu_(nullptr)
{
    Eigen::Quaterniond q(1, 0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    T_i_l = Sophus::SE3d(q, t);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
    ROS_WARN("Reset ImuProcess");

    b_first_frame_ = true;
    last_lidar_ = nullptr;
    last_imu_ = nullptr;

    gyr_int_.Reset(-1, nullptr);

    cur_pcl_in_.reset(new PointCloudXYZI());
    cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu)
{
    /// Reset gyr integrator
    gyr_int_.Reset(last_lidar_->header.stamp.toSec(), last_imu_);
    /// And then integrate all the imu measurements
    for (const auto &imu : v_imu)
    {
        gyr_int_.Integrate(imu);
    }
    ROS_INFO("integrate rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
             gyr_int_.GetRot().angleX() * 180.0 / M_PI,
             gyr_int_.GetRot().angleY() * 180.0 / M_PI,
             gyr_int_.GetRot().angleZ() * 180.0 / M_PI);
}

void ImuProcess::UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out,
                              double dt_be, const Sophus::SE3d &Tbe)
{
    // 结束帧到起始帧的平移量
    const Eigen::Vector3d &tbe = Tbe.translation();
    // 结束帧到起始帧的旋转量
    Eigen::Vector3d rso3_be = Tbe.so3().log();
    for (auto &pt : pcl_in_out->points)
    {
        int ring = int(pt.intensity);
        float dt_bi = pt.intensity - ring;

        if (dt_bi == 0)
            laserCloudtmp->push_back(pt);

        // 当前帧距离起始帧的时间间隔占全部时间的比例
        double ratio_bi = dt_bi / dt_be;
        // 当前帧距离结束帧的时间间隔占全部时间的比例
        double ratio_ie = 1 - ratio_bi;

        Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
        // 结束帧到当前得变换矩阵
        SO3d Rie = SO3d::exp(rso3_ie);

        /// Transform to the 'end' frame, using only the rotation
        /// Note: Compensation direction is INVERSE of Frame's moving direction
        /// So if we want to compensate a point at timestamp-i to the frame-e
        /// P_compensate = R_ei * Pi + t_ei

        // 结束帧和当前帧的平移量
        Eigen::Vector3d tie = ratio_ie * tbe;
        // 当前点坐标
        Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
        // v_pt_i = Rie * v_pt_comp_e + tie
        Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

        /// Undistorted point
        pt.x = v_pt_comp_e.x();
        pt.y = v_pt_comp_e.y();
        pt.z = v_pt_comp_e.z();
    }
}

void ImuProcess::Process(const MeasureGroup &meas)
{
    ROS_ASSERT(!meas.imu.empty());
    ROS_ASSERT(meas.lidar != nullptr);
    ROS_DEBUG("Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
              meas.lidar->header.stamp.toSec(), meas.imu.size(),
              meas.imu.front()->header.stamp.toSec(),
              meas.imu.back()->header.stamp.toSec());

    // 当前帧lidar数据
    auto pcl_in_msg = meas.lidar;

    if (b_first_frame_)
    {
        /// The very first lidar frame

        /// Reset
        Reset();

        /// Record first lidar, and first useful imu
        last_lidar_ = pcl_in_msg;
        last_imu_ = meas.imu.back();

        ROS_WARN("The very first lidar frame");

        /// Do nothing more, return
        b_first_frame_ = false;
        return;
    }

    // 积分所有的imu消息
    IntegrateGyr(meas.imu);

    // 利用imu旋转补偿lidar点
    // 来自imu的初始位姿

    // imu当前帧到上一帧imu之间的变换矩阵
    SE3d T_l_c(gyr_int_.GetRot(), Eigen::Vector3d::Zero());
    dt_l_c_ = pcl_in_msg->header.stamp.toSec() - last_lidar_->header.stamp.toSec();
    //// Get input pcl
    pcl::fromROSMsg(*pcl_in_msg, *cur_pcl_in_);

    /// Undistort points
    // lidar坐标系->imu坐标系->上一帧imu坐标系->上一帧Lidar坐标系，上一帧lidar坐标系即是当前帧lidar坐标系的起始时刻
    Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;
    pcl::copyPointCloud(*cur_pcl_in_, *cur_pcl_un_);
    clock_t t1, t2;
    t1 = clock();
    UndistortPcl(cur_pcl_un_, dt_l_c_, T_l_be);
    t2 = clock();
    printf("time is: %f\n", 1000.0 * (t2 - t1) / CLOCKS_PER_SEC);

    {
        static ros::Publisher pub_UndistortPcl =
            nh.advertise<sensor_msgs::PointCloud2>("/livox_first_point", 100);
        sensor_msgs::PointCloud2 pcl_out_msg;
        pcl::toROSMsg(*laserCloudtmp, pcl_out_msg);
        pcl_out_msg.header = pcl_in_msg->header;
        pcl_out_msg.header.frame_id = "/livox_frame";
        pub_UndistortPcl.publish(pcl_out_msg);
        laserCloudtmp->clear();
    }

    {
        static ros::Publisher pub_UndistortPcl =
            nh.advertise<sensor_msgs::PointCloud2>("/livox_undistort", 100);
        sensor_msgs::PointCloud2 pcl_out_msg;
        pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
        pcl_out_msg.header = pcl_in_msg->header;
        pcl_out_msg.header.frame_id = "/livox_frame";
        pub_UndistortPcl.publish(pcl_out_msg);
    }

    {
        static ros::Publisher pub_UndistortPcl =
            nh.advertise<sensor_msgs::PointCloud2>("/livox_origin", 100);
        sensor_msgs::PointCloud2 pcl_out_msg;
        std::cout << "point size: " << cur_pcl_in_->points.size() << "\n";
        pcl::toROSMsg(*cur_pcl_in_, pcl_out_msg);
        pcl_out_msg.header = pcl_in_msg->header;
        pcl_out_msg.header.frame_id = "/livox_frame";
        pub_UndistortPcl.publish(pcl_out_msg);
    }

    /// Record last measurements
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();
    cur_pcl_in_.reset(new PointCloudXYZI());
    cur_pcl_un_.reset(new PointCloudXYZI());
}
