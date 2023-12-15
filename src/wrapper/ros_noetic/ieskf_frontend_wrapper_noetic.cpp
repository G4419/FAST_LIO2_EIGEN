#include "wrapper/ros_noetic/ieskf_frontend_wrapper_noetic.h"
#include "ieskf_slam/globedefine.h"
namespace ROSNoetic{
    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh){
        //从ros参数服务器里读取数据
        //文件名，雷达话题，imu话题
        std::string config_file_name, lidar_topic, imu_topic;
        //雷达类型
        int lidar_type = 0;
        //视野盲区
        double blid = 0;
        nh.param<std::string>("wrapper/config_file_name", config_file_name, " ");
        nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
        nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        nh.param<double>("wrapper/blind", blid, 0);
        std::cout <<  "lidar_type: " <<lidar_type << std::endl;

        std::cout << "lidar_topic: " << lidar_topic << std::endl;
        std::cout << "imu_topic: " << imu_topic << std::endl;
        //创建前端智能指针，里面包含ieskf、map和前后向传播的指针。
        frontend_ptr = std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");
        if(lidar_type == AVIA){
            lidar_process_ptr = std::make_shared<AVIAProcess>();
            lidar_process_ptr->blind = blid;
        }else{
            std::cout << "unsupport lidar type" << std::endl;
            exit(100);
        }
        //imu回调函数
        imu_sub = nh.subscribe(imu_topic, 200000, &IESKFFrontEndWrapper::imuMsgsCallback, this);
        //雷达点云回调函数
        cloud_sub = nh.subscribe(lidar_topic, 200000, &IESKFFrontEndWrapper::cloudMsgsCallback, this);
        
        cur_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100000);
        local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100000);
        path_pub = nh.advertise<nav_msgs::Path>("path", 100000);
        odometry_pub = nh.advertise<nav_msgs::Odometry>("odometry", 100000);
        //运行
        run();

    }
    IESKFFrontEndWrapper::~IESKFFrontEndWrapper(){}

    void IESKFFrontEndWrapper::cloudMsgsCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg){
        //将雷达点云信息添加到点云队列中
        IESKFSlam::PointCloud point_cloud;
        lidar_process_ptr->process(msg, point_cloud);
        frontend_ptr->addPointCloud(point_cloud);
    }

    void IESKFFrontEndWrapper::imuMsgsCallback(const sensor_msgs::ImuPtr &msg){
        //将imu信息添加到imu队列中
        IESKFSlam::IMU imu;
        imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
        imu.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        imu.acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        frontend_ptr->addImu(imu);
    }
    void IESKFFrontEndWrapper::run(){
        ros::Rate r(500);
        while(ros::ok()){   
            //跟踪线程
            if(frontend_ptr->track()){
                //发布信息
                publishMsg();
            }
            r.sleep();
            ros::spinOnce();
        }
    }
    //发布数据
    void IESKFFrontEndWrapper::publishMsg(){
        
        auto X = frontend_ptr->readState();
        path.header.frame_id = "map";

        //发布path信息
        geometry_msgs::PoseStamped psd;
        psd.pose.position.x = X.position.x();
        psd.pose.position.y = X.position.y();
        psd.pose.position.z = X.position.z();
        path.poses.push_back(psd);
        path_pub.publish(path);

        //发布odometry信息
        odometry.header.frame_id = "map";
        odometry.child_frame_id = "odometry";
        odometry.header.stamp = ros::Time().fromSec(frontend_ptr->getLidarLastTime());
        odometry.pose.pose.position.x = X.position(0);
        odometry.pose.pose.position.y = X.position(1);
        odometry.pose.pose.position.z = X.position(2);
        odometry.pose.pose.orientation.w = X.rotation.w();
        odometry.pose.pose.orientation.x = X.rotation.x();
        odometry.pose.pose.orientation.y = X.rotation.y();
        odometry.pose.pose.orientation.z = X.rotation.z();
        odometry_pub.publish(odometry);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(odometry.pose.pose.position.x, 
                                        odometry.pose.pose.position.y, 
                                        odometry.pose.pose.position.z));
        q.setW(odometry.pose.pose.orientation.w);
        q.setX(odometry.pose.pose.orientation.x);
        q.setY(odometry.pose.pose.orientation.y);
        q.setZ(odometry.pose.pose.orientation.z);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, odometry.header.stamp, "map", "odometry"));
        
        
        //发布当前扫描的点云信息
        IESKFSlam::PCLPointCloud cloud = frontend_ptr->readCurrentPointCloud();
        pcl::transformPointCloud(cloud, cloud, IESKFSlam::compositeTransform(X.extrin_r, X.extrin_t).cast<float>());
        pcl::transformPointCloud(cloud, cloud, IESKFSlam::compositeTransform(X.rotation, X.position).cast<float>());
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        cur_cloud_pub.publish(msg);

        //发布地图信息
        cloud = frontend_ptr->readCurrentLocalMap();
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        local_map_pub.publish(msg);

        
    }

}

