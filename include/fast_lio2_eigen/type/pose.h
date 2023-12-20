#ifndef POSE_H
#define POSE_H

#include <Eigen/Dense>
#include "fast_lio2_eigen/type/timestamp.h"

namespace IESKFSlam
{
    struct Pose{
        TimeStamp time_stamp;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
    };
} // namespace IESKFSlam


#endif