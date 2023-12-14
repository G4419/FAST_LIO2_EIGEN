#ifndef BASE_TYPE_H
#define BASE_TYPE_H

#include "ieskf_slam/type/point_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace IESKFSlam
{
    //体素滤波器
    using VoxFilter = pcl::VoxelGrid<Point>;
    //KD树
    using KDtree = pcl::KdTreeFLANN<Point>;
    using KDtreePtr = KDtree::Ptr;
    using KDtreeConstPtr = KDtree::ConstPtr;

    const double GRIVITY = 9.81;
    template <typename _first, typename _second, typename _third, typename _fourth>
    struct triple{
        _first first;
        _second second;
        _third third;
        _fourth fourth;
    };

} // namespace IESKFSlam



#endif