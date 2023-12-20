#ifndef BASE_TYPE_H
#define BASE_TYPE_H

#include "fast_lio2_eigen/type/point_cloud.h"
#include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <ikd-Tree/ikd_Tree.h>
namespace IESKFSlam
{
    //体素滤波器
    using VoxFilter = pcl::VoxelGrid<Point>;
    //KD树
    using IKDtree = KD_TREE;
    using IKDtreePtr = std::shared_ptr<IESKFSlam::IKDtree>;

    const double GRIVITY = 9.81;
    template <typename _first, typename _second, typename _third, typename _fourth>
    struct quadruple{
        _first first;
        _second second;
        _third third;
        _fourth fourth;
    };

} // namespace IESKFSlam



#endif