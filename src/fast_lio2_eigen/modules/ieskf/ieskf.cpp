#include "fast_lio2_eigen/modules/ieskf/ieskf.h"
#include "fast_lio2_eigen/math/SO3.h"
namespace IESKFSlam{
    IESKF::IESKF(const std::string &config_path, const std::string &prefix) :ModuleBase(config_path, prefix, "IESKF") {
        //初始化P和Q以及状态量
        P.setIdentity();
        P.block<3,3>(9,9).diagonal() = Eigen::Vector3d{0.0001, 0.0001, 0.0001};
        P.block<3,3>(12,12).diagonal() = Eigen::Vector3d{0.001, 0.001, 0.001};
        P.block<3,3>(15,15).diagonal() = Eigen::Vector3d{0.00001, 0.00001, 0.00001};
        P.block<3,3>(18,18).diagonal() = Eigen::Vector3d{0.00001, 0.00001, 0.00001};
        P.block<3,3>(21,21).diagonal() = Eigen::Vector3d{0.00001, 0.00001, 0.00001};
        double cov_gyroscope, cov_acceleration, cov_bias_gyroscope, cov_bias_acceleration;
        //readParam：从ModuleBase里继承的读取参数的函数
        readParam("cov_gyroscope", cov_gyroscope, 0.1);
        readParam("cov_acceleration", cov_acceleration, 0.1);
        readParam("cov_bias_gyroscope", cov_bias_gyroscope, 0.1);
        readParam("cov_bias_acceleration", cov_bias_acceleration, 0.1);
        print_table();
        Q.block<3,3>(0,0).diagonal() = Eigen::Vector3d{cov_gyroscope, cov_gyroscope, cov_gyroscope};
        Q.block<3,3>(3,3).diagonal() = Eigen::Vector3d{cov_acceleration, cov_acceleration, cov_acceleration};
        Q.block<3,3>(6,6).diagonal() = Eigen::Vector3d{cov_bias_gyroscope, cov_bias_gyroscope, cov_bias_gyroscope};
        Q.block<3,3>(9,9).diagonal() = Eigen::Vector3d{cov_bias_acceleration, cov_bias_acceleration, cov_bias_acceleration};
        X.b_a.setZero();
        X.b_g.setZero();
        X.grivity.setZero();
        X.position.setZero();
        X.rotation.setIdentity();
        X.velocity.setZero();
    };
    IESKF::~IESKF(){}
    const IESKF::State24& IESKF::getX(){
        return X;
    }
    
    //公式2和协方差矩阵P
    void IESKF::predict(IMU imu, double dt){
        //omega_hat和a_hat
        imu.acceleration -= X.b_a;
        imu.gyroscope -= X.b_g;
        //对应公式6
        auto rotation = X.rotation.toRotationMatrix();
        X.rotation = Eigen::Quaterniond(rotation* so3Exp(imu.gyroscope * dt));
        X.rotation.normalize();
        X.position += X.velocity*dt;
        X.velocity += (rotation * imu.acceleration + X.grivity) * dt;
        //Fx和Fw
        Eigen::Matrix<double, 24, 24> Fx;
        Eigen::Matrix<double, 24, 12> Fw;
        Fx.setIdentity();
        Fw.setZero();

        Fx.block<3,3>(0,0) = so3Exp(-imu.gyroscope*dt);
        Fx.block<3,3>(0,9) = -A_T(-imu.gyroscope*dt)*dt;
        Fx.block<3,3>(3,6) = Fx.block<3,3>(6,15) = Eigen::Matrix3d::Identity()*dt;
        Fx.block<3,3>(6,0) = -rotation*skewSymmetric(imu.acceleration)*dt;
        Fx.block<3,3>(6,12) = -rotation*dt;

        Fw.block<3,3>(0,0) = -A_T(-imu.gyroscope*dt) * dt;
        Fw.block<3,3>(6,3) = -rotation*dt;
        Fw.block<3,3>(9,6) = Fw.block<3,3>(12,9) = Eigen::Matrix3d::Identity() *dt;
        P = Fx*P*Fx.transpose() +Fw*Q*Fw.transpose();
        
    }
    void IESKF::setX(const IESKF::State24 &x_in){
        X = x_in;
    }
    Eigen::Matrix<double, 24, 1> IESKF::getErrorState(const IESKF::State24 &s1, const IESKF::State24 &s2){
        Eigen::Matrix<double, 24, 1> ans;
        ans.block<3,1>(0,0) = SO3Log(s2.rotation.toRotationMatrix().transpose()*s1.rotation.toRotationMatrix());
        ans.block<3,1>(3,0) = s1.position - s2.position;
        ans.block<3,1>(6,0) = s1.velocity - s2.velocity;
        ans.block<3,1>(9,0) = s1.b_g - s2.b_g;
        ans.block<3,1>(12,0) = s1.b_a - s2.b_a;
        ans.block<3,1>(15,0) = s1.grivity - s2.grivity;
        ans.block<3,1>(18,0) = SO3Log(s2.extrin_r.toRotationMatrix().transpose()*s1.extrin_r.toRotationMatrix());
        ans.block<3,1>(21,0) = s1.extrin_t - s2.extrin_t;
        return ans;
    }
    bool IESKF::update(){
        auto x_k_k = X;
        Eigen::Matrix<double,24,24> P_in_update;
        Eigen::MatrixXd Z;
        Eigen::MatrixXd H;
        Eigen::MatrixXd K;
        bool converge = true;
        for(size_t i = 0; i < max_iter; i++){
            auto error_state = IESKF::getErrorState(x_k_k, X);
            //公式11
            Eigen::Matrix<double, 24, 24> J_inv;
            J_inv.setIdentity();
            J_inv.block<3,3>(0,0) = A_T(error_state.block<3,1>(0,0));
            J_inv.block<3,3>(18,18) = A_T(error_state.block<3,1>(18,0));
            P_in_update = J_inv * P * J_inv.transpose();
            //计算z和h
            cal_ZH_ptr->calculate(x_k_k, Z, H);
            
            Eigen::MatrixXd H_t = H.transpose();
            
            //公式14 R写成定值0.001
            K = (H_t * H + (P_in_update/ 0.001).inverse()).inverse() * H_t;
            //公式18
            Eigen::Matrix<double, 24, 1> left = -K * Z;
            Eigen::Matrix<double, 24, 1> right = -(Eigen::Matrix<double, 24, 24>::Identity() - K*H)*J_inv*error_state;
            Eigen::Matrix<double, 24, 1> update_X = left + right;

            converge = true;
            for(size_t j = 0; j < 24; j++){
                if(update_X(j, 0) > 0.001){
                    converge = false;
                    break;
                }
            }
            //将更新量加到名义变量上去
            x_k_k.rotation = x_k_k.rotation.toRotationMatrix()*so3Exp(update_X.block<3,1>(0,0));
            x_k_k.rotation.normalize();
            x_k_k.position = x_k_k.position + update_X.block<3,1>(3,0);
            x_k_k.velocity = x_k_k.velocity + update_X.block<3,1>(6,0);
            x_k_k.b_g = x_k_k.b_g + update_X.block<3,1>(9,0);
            x_k_k.b_a = x_k_k.b_a + update_X.block<3,1>(12,0);
            x_k_k.grivity = x_k_k.grivity + update_X.block<3,1>(15,0);
            x_k_k.extrin_r = x_k_k.extrin_r.toRotationMatrix() * so3Exp(update_X.block<3,1>(18,0));
            x_k_k.extrin_r.normalize();
            x_k_k.extrin_t = x_k_k.extrin_t + update_X.block<3,1>(21,0);

            if (converge)
            {
                break;
            }

        }
        //公式15
        X = x_k_k;
        P = (Eigen::Matrix<double, 24, 24>::Identity() - K * H ) * P_in_update;
        return converge;
    }
    


}