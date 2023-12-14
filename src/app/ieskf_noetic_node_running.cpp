#include "wrapper/ros_noetic/ieskf_frontend_wrapper_noetic.h"

int main(int argc, char* argv[]){
    //节点入口
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;
    //声明<ROSNoetic::IESKFFrontEndWrapper>类型的智能指针
    std::shared_ptr<ROSNoetic::IESKFFrontEndWrapper> front_end_ptr;
    //创建智能指针并调用构造函数
    front_end_ptr = std::make_shared<ROSNoetic::IESKFFrontEndWrapper>(nh);
    return 0;
}