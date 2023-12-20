#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <pcl/impl/pcl_base.hpp>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "fast_lio2_eigen/type/timestamp.h"

namespace IESKFSlam{
    using Point = pcl::PointXYZINormal;
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
    using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;

    struct PointCloud{
        using Ptr = std::shared_ptr<PointCloud>;
        TimeStamp time_stamp;
        PCLPointCloudPtr cloud_ptr;
        PointCloud(){
            cloud_ptr = pcl::make_shared<PCLPointCloud>();
        }
    };
}


#endif