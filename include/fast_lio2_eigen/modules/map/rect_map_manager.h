#ifndef RECT_MAP_MANAGER_H
#define RECT_MAP_MANAGER_H
#include "fast_lio2_eigen/modules/module_base.h"
#include "fast_lio2_eigen/type/point_cloud.h"
#include <pcl/common/transforms.h>
#include "fast_lio2_eigen/type/base_type.h"
#include "fast_lio2_eigen/modules/ieskf/ieskf.h"
#include <omp.h>
namespace IESKFSlam{
    class RectMapManager : private ModuleBase{
        private:
            IKDtreePtr ikdtree_ptr;
            float map_side_length_2;
            float map_resolution;
            Eigen::Vector3d local_map_center = Eigen::Vector3d::Zero();
            vector<BoxPointType> box_rm;
        public:
            RectMapManager(const std::string &config_file_path, const std::string &prefix);
            ~RectMapManager();
            void reset();
            void addScan(PCLPointCloudPtr cur_scan, IESKF::Ptr ieskf_ptr);
            IKDtreePtr readKdtree();
            double det_range;
            double map_time = 0;
    };
}

#endif