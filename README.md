# 激光雷达-相机标定工具
## 一、介绍
### 1.1 动机
学术界提出了许多新颖、先进的激光雷达-相机target-free/target-based标定算法[1-2]，但目前仍没有一种在任意场景数据上均能得到较好标定效果的算法，在工业界往往需要使用标定工具进行内外参数微调。

### 1.2 开发细节
本仓库在ROS环境下开发了一个针对高空间分辨率固态激光雷达与可见光相机的手动无目标标定软件工具(Manual Target-less Calibration Software)，可以对相机内参fx,fy,cx,cy,k1,k2,p1,p2以及激光雷达相对相机位姿tx,ty,tz,roll,pitch,yaw进行调整,软件界面见第三章。

技术细节方面，使用C++开发主要功能(点云滤波、投影、伪彩色增强、融合等操作)并编译封装成.so动态连接库；使用python开发软件界面，通过ctypes库调用动态链接库中的函数与用户进行交互；最后将标定参数保存在本地yaml文件。

## 二、环境与依赖
1. 环境：Ubuntu ROS
2. 依赖：ROS(主要用来读取.bag文件，若文件格式为.pcd等其它点云格式，可以去除ROS), PCL, OpenCV, Eigen, pyqt5
3. 编译运行过程
```
# 编译C++动态链接库
mkdir build & cd build
cmake ..
make
# 运行python代码
cd ../app
python main.py
```

## 三、标定效果

<p align="center"><img src="./resources/init_intrinsic.png" width=40%><img src="./resources/init_extrinsic.png" width=40%></p>
<h6 align="center">初始相机内外参数</h6>

<p align="center"><img src="./resources/result_intrinsic.png" width=40%><img src="./resources/result_extrinsic.png" width=40%></p>
<h6 align="center">手动调整相机内外参数结果</h6>

## 四、参考资料
[1] Yan, G., He, F., Shi, C., Cai, X., & Li, Y. (2022). Joint Camera Intrinsic and LiDAR-Camera Extrinsic Calibration. ArXiv, abs/2202.13708. [github-project](https://github.com/opencalib/jointcalib)

[2] Yan, G., Liu, Z., Wang, C., Shi, C., Wei, P., Cai, X., Ma, T., Liu, Z., Zhong, Z., Liu, Y., Zhao, M., Ma, Z., & Li, Y. (2022). OpenCalib: A Multi-sensor Calibration Toolbox for Autonomous Driving. arXiv preprint arXiv:2205.14087. [github-project](https://github.com/PJLab-ADG/SensorsCalibration)
