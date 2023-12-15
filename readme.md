## 这是利用Eigen进行重构FAST_LIO2的简洁实现。

### 参考链接

[FAST_LIO2](https://github.com/hku-mars/FAST_LIO) 

[动手写Fast-Lio](https://zhuanlan.zhihu.com/p/635702243)

[Fast-Lio2论文与代码详解](https://icv.51cto.com/posts/2160)
### 使用教程
1. 安装[Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
2. 下载[livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)
3. 将livox_ros_driver和FAST_LIO2_EIGEN放到一起catkin_make
### TO-DO LIST
- [x] 重力对齐 
- [ ] 使用idk-tree代替kd-tree
