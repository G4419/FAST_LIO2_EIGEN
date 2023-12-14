#ifndef FRONTEND_H
#define FRONTEND_H
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/globedefine.h"
#include <deque>
#include "ieskf_slam/type/pose.h"
#include "ieskf_slam/type/point_cloud.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/type/measure_group.h"
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/modules/map/rect_map_manager.h"
#include "ieskf_slam/modules/frontbackPropagate/frontback_propagate.h"
#include "ieskf_slam/math/math.h"
#include "ieskf_slam/modules/frontend/lio_zh_model.h"
#include <chrono>
#include <fstream>
namespace IESKFSlam{
    class FrontEnd : private ModuleBase{
        public:
            using Ptr = std::shared_ptr<FrontEnd>;
        private:
            //imu队列，imu发送的数据先存储到队列中
            std::deque<IMU> imu_deque;
            //点云队列，雷达发送的数据经过处理后先存储到队列中
            std::deque<PointCloud> pointcloud_deque;
            //跟ieskf相关的指针，如状态量的定义，状态的预测更新等
            std::shared_ptr<IESKF> ieskf_ptr;
            //地图指针，局部地图，kd树等
            std::shared_ptr<RectMapManager> map_ptr;
            //前后向传播指针，包含定义的imu姿态，进行imu位姿的传播以及激光点畸变的矫正
            std::shared_ptr<FrontBackPropagate> fb_propagate_ptr;
            //体素滤波器
            VoxFilter voxfilter;
            //计算Z和H矩阵的指针
            LIOZHModel::Ptr lio_zh_model_ptr;
            //进行点云的过滤
            PCLPointCloudPtr filter_point_cloud_ptr;
            //imu初始化标志
            bool imu_inited = false;
            //判断imu角速度的单位，统一为m/s^2
            double imu_scale = 1;
            
            
            bool enable_record = false;
            std::string record_file_name;
            std::fstream record_file;
        public:
            FrontEnd(const std::string &config_file_path, const std::string &prefix);
            ~FrontEnd();
            //应用在imu回调函数中
            void addImu(const IMU &imu);
            
            //应用在lidar回调函数中
            void addPointCloud(const PointCloud &pointclod);
            
            bool track();
            const PCLPointCloud &readCurrentPointCloud();
            const PCLPointCloud &readCurrentLocalMap();
            bool syncMeasureGroup(measure_group &mg);
            void initState(measure_group &mg);
            IESKF::State24 readState();
    };
}

#endif