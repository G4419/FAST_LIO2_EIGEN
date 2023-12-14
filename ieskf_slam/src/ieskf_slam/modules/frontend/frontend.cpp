#include "ieskf_slam/modules/frontend/frontend.h"
namespace IESKFSlam{
    FrontEnd::FrontEnd(const std::string &config_file_path, const std::string &prefix) : ModuleBase(config_file_path, prefix, "Front End Module"){
        
        float leaf_size;
        readParam("filter_leaf_size", leaf_size, 0.5f);
        //设置体素滤波器，单位米
        voxfilter.setLeafSize(leaf_size, leaf_size, leaf_size);
        Eigen::Quaterniond extrin_r;
        Eigen::Vector3d extrin_t;
        extrin_r.setIdentity();
        extrin_t.setZero();
        //读取外参信息
        std::vector<double> temp_v;
        readParam("extrin_r", temp_v, std::vector<double>());
        if (temp_v.size() == 9)
        {
            Eigen::Matrix3d temp_r_33;
            temp_r_33 << temp_v[0], temp_v[1], temp_v[2], temp_v[3], temp_v[4], temp_v[5],
                temp_v[6], temp_v[7], temp_v[8];
            extrin_r = temp_r_33;
        }else if(temp_v.size() == 4){
            extrin_r.x() = temp_v[0];
            extrin_r.y() = temp_v[1];
            extrin_r.z() = temp_v[2];
            extrin_r.w() = temp_v[3];
        }
        readParam("extrin_t", temp_v, std::vector<double>());
        if(temp_v.size() == 3) extrin_t << temp_v[0], temp_v[1], temp_v[2];
        
        
        readParam("enable_record", enable_record, false);
        readParam("record_file_name", record_file_name, std::string("default.txt"));
        //打开文件写入流
        if(enable_record){
            record_file.open(RESULT_DIR + record_file_name, std::ios::app | std::ios::app);
            if(!record_file.is_open()) std::cout << "unable to open file" << std::endl;
        }
        //打印参数
        print_table();
        
        ieskf_ptr = std::make_shared<IESKF>(config_file_path, "ieskf");
        auto x = ieskf_ptr->getX();
        x.extrin_r = extrin_r;
        x.extrin_t = extrin_t;
        ieskf_ptr->setX(x);
        map_ptr = std::make_shared<RectMapManager>(config_file_path, "map");
        fb_propagate_ptr = std::make_shared<FrontBackPropagate>();
        lio_zh_model_ptr = std::make_shared<LIOZHModel>();
        //为了调用lio_zh_model的计算Z和H的函数
        ieskf_ptr->cal_ZH_ptr = lio_zh_model_ptr;
        //对点云进行下采样处理
        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        //进行指针传递，分别对应 global_map_kdtree_ptr，current_cloud_ptr，local_map_ptr;
        lio_zh_model_ptr->prepare(map_ptr->readKdtree(),filter_point_cloud_ptr, map_ptr->getLocalMap());
    };

    FrontEnd::~FrontEnd(){
        if (record_file.is_open())
        {
            record_file.close();
        }
        
        
    };
    void FrontEnd::addImu(const IMU &imu){
        imu_deque.push_back(imu);

    }
    void FrontEnd::addPointCloud(const PointCloud &pointclod){
        pointcloud_deque.push_back(pointclod);
    }
    //跟踪函数：包括了取数据，初始化imu，添加点云（地图），传播
    bool FrontEnd::track(){
        measure_group mg;
        
        //从队列中取数据
        bool isSynced = syncMeasureGroup(mg);
        
        if(isSynced){
            
            if(!imu_inited){
                map_ptr->reset();
                //将初始的点云数据添加到地图中
                map_ptr->addScan(mg.point_cloud_measure.cloud_ptr, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), ieskf_ptr);
                initState(mg);
                return false;
            }
            
            //前后向传播
            fb_propagate_ptr->propagate(mg, ieskf_ptr);
            
            auto x = ieskf_ptr->getX();

            //对当前点云进行下采样
            voxfilter.setInputCloud(mg.point_cloud_measure.cloud_ptr);
            voxfilter.filter(*filter_point_cloud_ptr);//filter_point_cloud_ptr与current_cloud_ptr指针是相关连的
            
            ieskf_ptr->update();

            //输出位姿到文件中
            x = ieskf_ptr->getX();
            if(enable_record && record_file.is_open()){
                record_file << std::setprecision(15) << mg.lidar_end_time << " "
                            << x.position.x() << " " << x.position.y() << " " << x.position.z()<< " "
                            << x.rotation.x() << " " << x.rotation.y() << " " << x.rotation.z()<< " "
                            << x.rotation.w() << std::endl;
            }

            //将点云加载到local_map_ptr（地图）中
            map_ptr->addScan(filter_point_cloud_ptr, x.rotation, x.position, ieskf_ptr);
            return true;
        }
        return false;


    }
    const PCLPointCloud &FrontEnd::readCurrentPointCloud(){
        return *filter_point_cloud_ptr;
    }
    const PCLPointCloud &FrontEnd::readCurrentLocalMap(){
        return *map_ptr->getLocalMap();
    }
    //取一组点云数据及对应的imu数据
    bool FrontEnd::syncMeasureGroup(measure_group &mg){
        //每次从队列中取数据时都会将mg清空
        mg.imus_measure.clear();
        mg.point_cloud_measure.cloud_ptr->clear();
        //队列为空则返回false
        if(imu_deque.empty() || pointcloud_deque.empty()) return false;

        double imu_start_time = imu_deque.front().time_stamp.sec();
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double pointcloud_start_time = pointcloud_deque.front().time_stamp.sec();
        double pointcloud_end_time = pointcloud_deque.front().cloud_ptr->back().offset_time/1e9 + pointcloud_start_time;
        //imu数据不够
        if(imu_end_time < pointcloud_end_time) return false;
        //雷达点云结束时间早于imu数据开始时间，弹出雷达数据并返回false
        if(pointcloud_end_time < imu_start_time){
            pointcloud_deque.pop_front();
            return false;
        }

        mg.point_cloud_measure = pointcloud_deque.front();
        pointcloud_deque.pop_front();
        mg.lidar_beg_time = pointcloud_start_time;
        mg.lidar_end_time = pointcloud_end_time;
        //将这一帧点云结束前的所有imu数据添加到mg中
        while(!imu_deque.empty()){
            if(imu_deque.front().time_stamp.sec() < pointcloud_end_time){
                mg.imus_measure.push_back(imu_deque.front());
                imu_deque.pop_front();
            }else break;
        }
        //imu数据太少
        if(mg.imus_measure.size() <= 5) return false;
        return true;

    }
    //imu初始化
    void FrontEnd::initState(measure_group &mg){
        //声明为static是为了如果初始化没有成功，还可以继续累计。
        static int imu_counts = 0;
        //加速用来初始化重力，角速度用来初始化陀螺仪零偏
        static Eigen::Vector3d mean_acc{0, 0, 0};
        static Eigen::Vector3d mean_angle_velocity{0, 0, 0};
        auto &ieskf = *ieskf_ptr;
        
        for(size_t i = 0; i < mg.imus_measure.size(); i++){
            mean_acc += mg.imus_measure[i].acceleration;
            mean_angle_velocity += mg.imus_measure[i].gyroscope;
            imu_counts++;
        }
        //imu_counts大于等于5就算初始化成功
        if(imu_counts >= 5){
            auto x = ieskf.getX();
            mean_acc /= double(imu_counts);
            mean_angle_velocity /= double(imu_counts);
            //根据imu加速度单位不同计算scale
            imu_scale = GRIVITY / mean_acc.norm();

            fb_propagate_ptr->imu_scale = imu_scale;
            fb_propagate_ptr->last_imu = mg.imus_measure.back();
            fb_propagate_ptr->last_lidar_end_time_ = mg.lidar_end_time;
            //统一成m/s^2
            x.grivity = -mean_acc / mean_acc.norm() * GRIVITY;
            x.b_g = mean_angle_velocity;
            ieskf.setX(x);
            imu_inited = true;
        }
    }
    IESKF::State24 FrontEnd::readState(){
        return ieskf_ptr->getX();
    }
    

}