#ifndef RECT_MAP_MANAGER_H
#define RECT_MAP_MANAGER_H
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/point_cloud.h"
#include <pcl/common/transforms.h>
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/modules/ieskf/ieskf.h"
namespace IESKFSlam{
    class RectMapManager : private ModuleBase{
        private:
            PCLPointCloudPtr local_map_ptr;
            KDtreePtr kdtree_ptr;
            float map_side_length_2;
            float map_resolution;
        public:
            RectMapManager(const std::string &config_file_path, const std::string &prefix);
            ~RectMapManager();
            void reset();
            void addScan(PCLPointCloudPtr cur_scan, IESKF::Ptr ieskf_ptr);
            PCLPointCloudConstPtr getLocalMap();
            KDtreeConstPtr readKdtree();
    };
}

#endif