#ifndef MEARSERE_GROUP_H
#define MEARSERE_GROUP_H
#include "imu.h"
#include "point_cloud.h"
#include <deque>
namespace IESKFSlam{
    struct measure_group
    {
        double lidar_beg_time;
        double lidar_end_time;
        PointCloud point_cloud_measure;
        std::deque<IMU> imus_measure;
    };
    
}

#endif