#ifndef POSE_H
#define POSE_H

#include <Eigen/Dense>
#include "ieskf_slam/type/timestamp.h"

namespace IESKFSlam
{
    struct Pose{
        TimeStamp time_stamp;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
    };
} // namespace IESKFSlam


#endif