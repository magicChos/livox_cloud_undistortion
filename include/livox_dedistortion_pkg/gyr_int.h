#ifndef GYR_INT_H
#define GYR_INT_H

#include <sensor_msgs/Imu.h>
#include "sophus/so3.hpp"

class GyrInt
{
public:
    GyrInt();
    void Integrate(const sensor_msgs::ImuConstPtr &imu);

    /**
     * @brief
     *
     * @param start_timestamp lidar数据的时间戳
     * @param lastimu lidar数据前一时刻imu数据
     */
    void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

    /**
     * @brief 获取当前帧和上一帧imu之间的变换矩阵
     *
     * @return const Sophus::SO3d
     */
    const Sophus::SO3d GetRot() const;

private:
    // Sophus::SO3d r_;
    /// last_imu_ is

    // 记录的是lidar数据的时间戳
    double start_timestamp_;

    sensor_msgs::ImuConstPtr last_imu_;
    /// Making sure the equal size: v_imu_ and v_rot_
    //  记录内插出的imu数据
    std::vector<sensor_msgs::ImuConstPtr> v_imu_;
    // 记录imu的积分结果（旋转变换）
    std::vector<Sophus::SO3d> v_rot_;
};

#endif