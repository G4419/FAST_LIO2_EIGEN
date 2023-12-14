#include "ieskf_slam/modules/map/rect_map_manager.h"

#include "ieskf_slam/math/math.h"
namespace IESKFSlam
{
    RectMapManager::RectMapManager(const std::string &config_file_path, const std::string &prefix) : ModuleBase(config_file_path, prefix, "RectMapManager") {
        //创建局部地图（点云）和kd树的智能指针
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KDtree>();
        readParam("map_side_length_2", map_side_length_2, 500.0f);
        readParam("map_resolution", map_resolution, 0.5f);
        print_table();
    }
    RectMapManager::~RectMapManager(){

    }
    //清楚地图指针指向的内容
    void RectMapManager::reset(){
        local_map_ptr->clear();
    }
    //向地图指针指向的点云中添加点云，cur_scan：需要变换的点云，q和t变换矩阵，scan经过变换后的点云
    void RectMapManager::addScan(PCLPointCloudPtr cur_scan, const Eigen::Quaterniond &q, const Eigen::Vector3d &t, IESKF::Ptr ieskf_ptr){
        PCLPointCloud scan;
    
        auto x = ieskf_ptr->getX();
        //先转换到imu坐标系下，再转换到世界坐标系下
        pcl::transformPointCloud(*cur_scan, scan, compositeTransform(x.extrin_r, x.extrin_t).cast<float>());
        pcl::transformPointCloud(scan, scan, compositeTransform(q, t).cast<float>());
        //如果地图为空，则直接赋值
        if(local_map_ptr->empty()) *local_map_ptr = scan;
        else{
            for(auto &&point : scan){
                std::vector<int> ind;
                std::vector<float> distance;
                kdtree_ptr->nearestKSearch(point, 5, ind, distance);
                //大于这个分辨率才添加
                if(distance[0] > map_resolution) local_map_ptr->push_back(point);
            }
            //将地图范围限制在一个以机器人为中心的正方形框里，如果边缘的点超过了这个框，则删除这些点
            //具体的是将超出边界的点放到points尾部，然后resize删除
            int left = 0, right = local_map_ptr->size() - 1;
            while(right > left){
                while(right > left && abs(local_map_ptr->points[left].x - t.x()) < map_side_length_2 &&
                                    abs(local_map_ptr->points[left].y - t.y()) < map_side_length_2 &&
                                    abs(local_map_ptr->points[left].z - t.z()) < map_side_length_2) left++;
                while(right > left && abs(local_map_ptr->points[right].x - t.x()) > map_side_length_2 &&
                    abs(local_map_ptr->points[right].y - t.y()) > map_side_length_2 &&
                    abs(local_map_ptr->points[right].z - t.z()) > map_side_length_2) right--;
                std::swap(local_map_ptr->points[left], local_map_ptr->points[right]);
            }
            local_map_ptr->resize(right+1);
        }
        //重新构建kd树
        kdtree_ptr->setInputCloud(local_map_ptr);
    }
    PCLPointCloudConstPtr RectMapManager::getLocalMap(){
        return local_map_ptr;
    }
    KDtreeConstPtr RectMapManager::readKdtree(){
        return kdtree_ptr;
    }
} // namespace IESKFSlam
