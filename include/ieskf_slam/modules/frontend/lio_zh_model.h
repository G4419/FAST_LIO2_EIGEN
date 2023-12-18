#ifndef LIO_ZH_MODEL_H
#define LIO_ZH_MODEL_H

#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/math/geometry.h"

namespace IESKFSlam
{
    class LIOZHModel : public IESKF::calZHInterface{
        private:
            const int NEAR_POINTS_NUM = 5;
            
            //1) point_imu 2) normal_vector 3)distance 4)point_lidar
            using loss_type = triple<Eigen::Vector3d, Eigen::Vector3d, double, Eigen::Vector3d>;
            KDtreeConstPtr global_map_kdtree_ptr;
            PCLPointCloudPtr current_cloud_ptr;
            PCLPointCloudConstPtr local_map_ptr;
        public:
            using Ptr =std::shared_ptr<LIOZHModel>;
            bool extrinsic_est_en = false;
            void prepare(KDtreeConstPtr kd_tree, PCLPointCloudPtr current_cloud, PCLPointCloudConstPtr local_map){
                global_map_kdtree_ptr = kd_tree;
                current_cloud_ptr = current_cloud;
                local_map_ptr = local_map;
            }
            bool calculate(const IESKF::State24 &state, Eigen::MatrixXd &Z, Eigen::MatrixXd &H) override {
                //std::vector<loss_type> loss_v;
                //loss_v.resize(current_cloud_ptr->size());
                //std::vector<bool> is_effect_point(current_cloud_ptr->size(), false);
                std::vector<loss_type> loss_real;
                std::vector<loss_type> loss_v;
                loss_v.resize(current_cloud_ptr->size());
                std::vector<bool> is_effect_point(current_cloud_ptr->size(), false);
                //并行化计算z和h
                #ifdef MP_EN
                    omp_set_num_threads(MP_PROC_NUM);
                    #pragma omp parallel for
                #endif
                /**
                 * 有效点的判断
                 * 1. 将当前点变换到世界系下
                 * 2. 在局部地图上使用kd_tree 临近搜索NEAR_POINTS_NUM个点
                 * 3. 判断这些点是否构成平面
                 * 4. 判断点离这个平面够不够近(达到阈值)
                 * 5. 满足上述条件，设置为有效点。
                */
               //std::cout << current_cloud_ptr->size() << std::endl;
                for(size_t i = 0; i < current_cloud_ptr->size(); i++){
                    Point point_lidar = current_cloud_ptr->points[i];
                    Point point_imu = transformPoint(point_lidar, state.extrin_r, state.extrin_t);
                    Point point_world = transformPoint(point_imu, state.rotation, state.position);
                    std::vector<int> point_index;
                    std::vector<float> point_distance;//nearestKSearch传入的必须是float
                    global_map_kdtree_ptr->nearestKSearch(point_world, NEAR_POINTS_NUM, point_index, point_distance);
                    // . 是否搜索到足够的点以及最远的点到当前点的距离足够小(太远，就不认为这俩在一个平面)
                    if(point_distance.size() <NEAR_POINTS_NUM || point_distance[NEAR_POINTS_NUM-1] > 5){
                        //std::cout <<"1 not enough" << std::endl;
                        continue;
                    } 
                    // . 判断这些点够不够成平面
                    std::vector<Point> planar_points;
                    for(auto index : point_index){
                        planar_points.push_back(local_map_ptr->at(index));
                    }
                    Eigen::Vector4d pabcd;
                    
                    //1) point_imu 2) normal_vector 3)distance
                    // . 如果构成平面
                    if(planarCheck(planar_points, pabcd, 0.1)){
                        double pd = point_world.x * pabcd(0) + point_world.y * pabcd(1) + point_world.z * pabcd(2) +pabcd(3);
                        loss_type loss;
                        loss.first = {point_imu.x, point_imu.y, point_imu.z};
                        loss.second = {pabcd(0), pabcd(1), pabcd(2)};
                        loss.third = pd;
                        loss.fourth = {point_lidar.x, point_lidar.y, point_lidar.z};
                        if(isnan(pd) || isnan(loss.second(0)) || isnan(loss.second(1)) || isnan(loss.second(2))){
                            //std::cout<< "invalid" << std::endl;
                            continue;
                        } 
                        // .计算点和平面的夹角，夹角越小S越大。
                        double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm());
                        if(s > 0.9){
                            loss_v[i] = loss;
                            is_effect_point[i] = true;
                        }
                   }
                }
                for(size_t i = 0; i < current_cloud_ptr->size(); i++){
                    if (is_effect_point[i])
                    {
                        loss_real.push_back(loss_v[i]);
                    }
                }
                int valid_point_nums = loss_real.size();
                H = Eigen::MatrixXd::Zero(valid_point_nums, 24);
                Z.resize(valid_point_nums ,1);
                //点到平面的距离y = u^T * (R_I*(R_L * p_l + t_L)+t_I-q)
                //分别对R_I和t_I求导分别为H.block<1,3>(i,3)和H.block<1,3>(i, 0)
                //分别对R_L和t_L求导分别为H.block<1,3>(i,18)和H.block<1,21>(i, 0)
                //1) point_imu 2) normal_vector 3)distance 4)point_lidar
                for(size_t i = 0; i < valid_point_nums; i++){
                    Eigen::Vector3d dr = -loss_real[i].second.transpose() * state.rotation.toRotationMatrix() * skewSymmetric(loss_real[i].first);
                    H.block<1,3>(i, 0) = dr;
                    H.block<1,3>(i,3) = loss_real[i].second.transpose();
                    if(extrinsic_est_en){
                        H.block<1,3>(i,18) = -loss_real[i].second.transpose() * state.rotation.toRotationMatrix() * state.extrin_r.toRotationMatrix() * skewSymmetric(loss_real[i].fourth);
                        H.block<1,3>(i,21) = loss_real[i].second.transpose() * state.rotation.toRotationMatrix();
                    }
                    Z(i,0) = loss_real[i].third;
                }
               return true;
            }
    };
    
} // namespace IESKFSlam



#endif