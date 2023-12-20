#include "fast_lio2_eigen/modules/map/rect_map_manager.h"
#include "fast_lio2_eigen/math/math.h"
namespace IESKFSlam
{
    RectMapManager::RectMapManager(const std::string &config_file_path, const std::string &prefix) : ModuleBase(config_file_path, prefix, "RectMapManager") {
        //创建局部地图（点云）和kd树的智能指针
        ikdtree_ptr = std::make_shared<IKDtree>();
        readParam("map_side_length_2", map_side_length_2, 500.0f);
        readParam("map_resolution", map_resolution, 0.5f);
        print_table();
    }
    RectMapManager::~RectMapManager(){
        // std::cout << map_time << std::endl;
    }
    //清楚地图指针指向的内容
    void RectMapManager::reset(){
        ikdtree_ptr->InitializeKDTree();
    }
    //向地图指针指向的点云中添加点云，cur_scan：需要变换的点云，q和t变换矩阵，scan经过变换后的点云
    void RectMapManager::addScan(PCLPointCloudPtr cur_scan, IESKF::Ptr ieskf_ptr){
        PCLPointCloud scan;
    
        auto x = ieskf_ptr->getX();
        //先转换到imu坐标系下，再转换到世界坐标系下
        pcl::transformPointCloud(*cur_scan, scan, compositeTransform(x.extrin_r, x.extrin_t).cast<float>());
        pcl::transformPointCloud(scan, scan, compositeTransform(x.rotation, x.position).cast<float>());
        PointVector pointcloud_need_downsample, pointcloud_noneed_downsample, point_vector;
        for(auto &&point : scan){
            point_vector.push_back(point);
        }
        // double begin_time = omp_get_wtime();
        //始终维护map_side_length_2 × 2的边长的立方体为局部地图，当移动到地图边缘时，调整局部地图。
        if(ikdtree_ptr->Root_Node == NULL) ikdtree_ptr->Build(point_vector);
        else{
            PointVector nearest_points;
            
            for(auto &point : point_vector){
                std::vector<float> distance;
                ikdtree_ptr->Nearest_Search(point, 1, nearest_points, distance, map_resolution);
                if(distance.empty()) pointcloud_noneed_downsample.push_back(point);
            }
            ikdtree_ptr->Add_Points(pointcloud_noneed_downsample, false);
        }
        BoxPointType boxpoints;
        boxpoints.vertex_min[0] = local_map_center(0) - map_side_length_2;
        boxpoints.vertex_min[1] = local_map_center(1) - map_side_length_2;
        boxpoints.vertex_min[2] = local_map_center(2) - map_side_length_2;
        boxpoints.vertex_max[0] = local_map_center(0) + map_side_length_2;
        boxpoints.vertex_max[1] = local_map_center(1) + map_side_length_2;
        boxpoints.vertex_max[2] = local_map_center(2) + map_side_length_2;
        vector<BoxPointType> BoxPoints;
        for(int i = 0; i < 3; i++){
            BoxPointType temp_boxpoints = boxpoints;
            if(x.position(i) + det_range > boxpoints.vertex_max[i]){
                temp_boxpoints.vertex_max[i] = x.position(i) - map_side_length_2;
            }
            else if(x.position(i) - det_range < boxpoints.vertex_min[i]){
                temp_boxpoints.vertex_min[i] = x.position(i) + map_side_length_2;
            }else continue;
            local_map_center(i) = x.position(i);
            BoxPoints.push_back(temp_boxpoints);
        }
        if(BoxPoints.size() > 0) ikdtree_ptr->Delete_Point_Boxes(BoxPoints);
        // map_time += omp_get_wtime() - begin_time;
    }

    IKDtreePtr RectMapManager::readKdtree(){
        return ikdtree_ptr;
    }
} // namespace IESKFSlam
