#ifndef COMMON_LIDAR_PROCESS_INTERFACE_H
#define COMMON_LIDAR_PROCESS_INTERFACE_H
#include <sensor_msgs/PointCloud2.h>
#include "ieskf_slam/type/base_type.h"
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver2/CustomMsg.h>
namespace ROSNoetic{
    class CommonLidarProcessInterface{
        public:
            double blind = 0;
            virtual bool process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, IESKFSlam::PointCloud &cloud) = 0;
    };
}

#endif