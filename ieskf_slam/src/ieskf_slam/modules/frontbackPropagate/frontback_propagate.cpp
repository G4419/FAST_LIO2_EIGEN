#include "ieskf_slam/modules/frontbackPropagate/frontback_propagate.h"

namespace IESKFSlam{
    
    FrontBackPropagate::FrontBackPropagate(){
        // debug_file.open("/home/ryan/slam/test_ws/src/ieskf_slam/debug_file.txt", std::ios::out);
        // debug_file.clear();
    };

    FrontBackPropagate::~FrontBackPropagate(){
        // debug_file.close();
    };
    
    void FrontBackPropagate::propagate(measure_group &mg, IESKF::Ptr ieskf_ptr){
        //使用lambda表达式进行mg数据点云中点的排序
        std::sort(mg.point_cloud_measure.cloud_ptr->begin(), mg.point_cloud_measure.cloud_ptr->end(), [](Point x, Point y){
            return x.offset_time < y.offset_time;
        });
        //存储前向传播中imu的状态量
        std::vector<ImuPose6D> imu_pose6d;
        //将上一个imu加入进来
        mg.imus_measure.push_front(last_imu);
        double lidar_beg_time = mg.lidar_beg_time;
        double lidar_end_time = mg.lidar_end_time;
        double imu_beg_time = mg.imus_measure.front().time_stamp.sec();
        double imu_end_time = mg.imus_measure.back().time_stamp.sec();   
        //后续会将pcl_out->points做矫正
        auto &pcl_out = *mg.point_cloud_measure.cloud_ptr;
        imu_pose6d.clear();
        //利用上一次的雷达结束点的位姿初始imu_pose
        auto state =  ieskf_ptr->getX();
        imu_pose6d.push_back({0.0, last_accelaration, last_angle_velocity, state.velocity, state.position, state.rotation});
        
        //if(mg.point_cloud_measure.cloud_ptr->begin() == mg.point_cloud_measure.cloud_ptr->end()) return;
        
        //依次为平均加速度，平均角速度
        Eigen::Vector3d acce_avr, gyro_avr;
        
        double dt = 0, off_time;
        IMU in;
        //取两个imu数据进行平均作为测量得到的imu数据并进行预测
        for(auto it_imu = mg.imus_measure.begin(); it_imu != mg.imus_measure.end()-1; it_imu++){
            //取一前一后两个imu
            auto &&head = *(it_imu);
            auto &&tail = *(it_imu+1);

            //取平均
            acce_avr = (head.acceleration + tail.acceleration) * 0.5;
            gyro_avr = (head.gyroscope + tail.gyroscope) * 0.5 ;
            //转换为m/s^2单位
            acce_avr = acce_avr * imu_scale;
            
            if(head.time_stamp.sec() < last_lidar_end_time_) 
                //从上一帧点云结束点开始递推
                dt = tail.time_stamp.sec() - last_lidar_end_time_;
            //从前一个imu开始递推
            else dt = tail.time_stamp.sec() - head.time_stamp.sec();
            
            in.acceleration = acce_avr;
            in.gyroscope = gyro_avr;
            //利用imu数据进行预测
            ieskf_ptr->predict(in, dt);
            state = ieskf_ptr->getX();
            //存储世界坐标系下的加速度和imu角速度作为矫正时刻的参考值
            last_accelaration = state.rotation * (acce_avr - state.b_a);
            last_angle_velocity = gyro_avr - state.b_g;
            //减去重力
            for(size_t i = 0; i < 3; i++) last_accelaration[i] += state.grivity[i];
            //与点云开始时间的偏移
            off_time = tail.time_stamp.sec() - lidar_beg_time;
            imu_pose6d.push_back({off_time, last_accelaration, last_angle_velocity, state.velocity, state.position, state.rotation});
        }
        //利用最后一次平均得到imu数据进行再一次预测，得到结束时刻点云的状态
        dt = mg.lidar_end_time - imu_end_time;
        ieskf_ptr->predict(in, dt);
        state = ieskf_ptr->getX();

        //将mg中最后一个imu存到last_imu中
        last_imu = mg.imus_measure.back();
        last_lidar_end_time_ = mg.lidar_end_time;
        //按照雷达点左边的imu数据作为状态，以下的值全是左边imu的值
        Eigen::Vector3d vel_imu, pos_imu, acc_imu, angle_velocity_imu;
        Eigen::Matrix3d R_imu;
        if(pcl_out.begin() == pcl_out.end()) return;
        auto pcl_it = pcl_out.points.end() - 1;
        
        for(auto imu_pose_it = imu_pose6d.end()-1; imu_pose_it != imu_pose6d.begin() ; imu_pose_it--){//last_lidar_end_time_与pcl_beg_time的点云也会用到 为啥呢
            auto head = imu_pose_it-1;
            auto tail = imu_pose_it;
            R_imu = head->rotation.toRotationMatrix();
            vel_imu = head->velocity;
            pos_imu = head->position;
            acc_imu = tail->acceleration;
            angle_velocity_imu = tail->angle_velocity;
            //公式9
            for(; pcl_it >= pcl_out.points.begin() && head->time < pcl_it->offset_time/1e9 ; pcl_it-- ){
                dt = pcl_it->offset_time/1e9 - head->time;
                Eigen::Matrix3d R_i(R_imu* so3Exp(angle_velocity_imu*dt));
                Eigen::Vector3d p_i = {pcl_it->x, pcl_it->y, pcl_it->z};
                Eigen::Vector3d T_ei(pos_imu + vel_imu*dt + 0.5* acc_imu*dt*dt - state.position);
                Eigen::Vector3d P_compensate = state.extrin_r.conjugate() *
                                    (state.rotation.conjugate() * (R_i * (state.extrin_r * p_i + state.extrin_t) + T_ei)
                                    - state.extrin_t);
                pcl_it->x = P_compensate(0);
                pcl_it->y = P_compensate(1);
                pcl_it->z = P_compensate(2);
                // debug_file  << pcl_it->x << " " <<  pcl_it->y << " " << pcl_it->z << std::endl;
            }
        }
    }
}