# snake_arm_visual功能包使用说明

- 这个包的功能实现并不好。本文档最后列出了一些待优化项。
- 在硬件上依赖realsense2。在launch文件中默认不使用硬件进行数据采集，而是使用data中预先采集的默认数据。

## 功能包依赖

### ros-melodic-imu-tools

- 可通过apt-get 进行安装，不过目前程序里没有再用这个包了

### realsense2_camera

- sudo apt install ros-melodic-librealsense2*
- sudo apt install ros-melodic-realsense2-*

### PCL - surface_on_nurbs

- 该功能包在安装PCL（或者随ROS安装）时默认不编译，因此可能需要重新安装。
-　参考：<https://blog.csdn.net/bianjingyang/article/details/116133046>

1. 安装依赖(无法定位的不用管)
sudo apt-get install g++ cmake
sudo apt-get install doxygen
sudo apt-get install mpi-default-dev
sudo apt-get install openmpi-bin
sudo apt-get install openmpi-common
sudo apt-get install libflann1.8
sudo apt-get install libflann-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libvtk6-dev
sudo apt-get install libvtk6.2
sudo apt-get install libvtk6.2-qt
sudo apt-get install 'libqhull*'
sudo apt-get install libusb-dev
sudo apt-get install libgtest-dev
sudo apt-get install freeglut3-dev
sudo apt-get install pkg-config
sudo apt-get install build-essential
sudo apt-get install libxmu-dev
sudo apt-get install libxi-dev
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install graphviz
sudo apt-get install mono-complete
sudo apt-get install qt-sdk
sudo apt-get install openjdk-9-jdk
sudo apt-get install openjdk-9-jre
sudo apt-get install phonon-backend-gstreamer
sudo apt-get install phonon-backend-vlc
sudo apt-get install libopenni-dev
sudo apt-get install libopenni2-dev

1. 下载1.8.1源码
<https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.8.1.zip>

1. 解压并在pcl-1.8.1文件夹下创建/build文件夹

```shell
cd XXXX/pcl-1.8.1   （XXXX为自己解压源码后的路径）
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=None ..
cmake-gui ..
"勾选BUILD_surface_on_nurbs， 将CMAKE_INSTALL_PREFIX设置为/usr/local/pcl-1.8，依次点击configure、enerate"
make -jX  (X为编译时的线程数，不超过CPU核数)
sudo make install
```

- 安装完毕后，如果自定义了其它安装路径，则需要在[{PACKAGE_PATH}/src/snake_arm_visual/CMakeLists.txt]中对**set(PCL_DIR "/usr/local/pcl-1.8/share/pcl-1.8")**进行相应修改。

## 关于运行中的输出文件

### 路径

- 程序中点云捕获与拼接、曲面拟合、路径规划中产生的文件均位于指定路径下，可通过 [{PACKAGE_PATH}/config/snake_arm_visual.yaml/data_save_path] 进行配置；
- 推荐设置为[data/]路径下的子文件夹；

### 类型说明

- **.pcd**
  - 为一种通用的点云文件，可通过VSCode中的 [pcd-viewer] 插件进行快速预览;

- **.g2**
  - 为[SISL-B样条曲线库] 所采用的一种储存 点序列、曲线、曲线的文件格式，可通过SISL提供的工具[sisl_view_demo]进行查看。
  - [sisl_view_demo] 的可执行文件路径为 **{WORK_SPACE_PATH}/doc/SISL B样条库/build/viewer/sisl_view_demo**；
  - 一个简单的使用示例为： `.{PATH}/sisl_view_demo c {FILE_PATH}/CURVE_FILENAME.g2 s {FILE_PATH}/SURFACE_FILENAME.g2`；
  - 关于该文件的格式说明，可见文件 **{WORK_SPACE_PATH}/doc/SISL B样条库/SISL库.g2文件格式说明.md**；

## 一些注意事项

- 由于realsense提供的点云数据单位为[米]，所以该包中所有的距离单位若无具体说明，均为[米]。

## 一些可以优化的方向

### 整体代码

- 尽管在写代码的时候已经尽量将模块之间通过生成过程文件解耦开了，但是一个类里面完成的功能太多了，看起来还是不够清爽。按理说可以再拆成好几个类。
- 加一些多线程的内容进来是不是可以加快运行速度？

### 点云拼接部分

- 现在的icp配准分了两个阶段，可能有的时候不够用。其实可以把icp_MaxCorrespondenceDistance改成一个数组，这样就可以根据yaml人以控制阶段的数量了。甚至可以探究一下如何根据输入点云的质量自适应计算MaxCorrespondenceDistance。（貌似已经有人做这样的工作了）
- 关于点云数据的精简，这个有点意思，不知道可不可以借鉴一下，来加快匹配速度<https://blog.csdn.net/qq_45006390/article/details/118356423>

### 聚类部分

- 现在的算法对参数极其敏感，很难受，不知道怎么优化。感觉PCL里提供的主曲率计算方法受边缘影响很大，可能也对聚类效果影响比较大；

### 曲面拟合与路径规划

- 现在边界曲线没有用起来，不过现在排除离群点的策略也还是挺不错的，完全可以替代。不过也可是进一步的优化：每获得一个扫查的点位，就检测其相对于点云是否是游离点。如果是，就将其从扫描点位中删去，并标记该边界点为入点/出点。若一条曲线上先出现出点后出现入点，则可认为曲线上有洞。所有曲线完成离散后，汇总其洞的位置，并将洞两侧的边界点分别进行首尾相连，从而避开了对洞的扫查。
- 但是现在的方法在处理凹数据的时候效果不好，更没法处理有洞的面。对此，还有一个感觉更可靠的思路：完成面的拟合之后，对拟合曲面进行采样，并与原始点云求交集，不相交的那部分采样点即是位于洞中的点。对这些点聚类之后分别进行曲面拟合，即可获得洞的样条曲面表达，进一步也可以获取洞的边界曲线。之后，进行路径规划和扫差点离散时，只要遇见这些洞就转向，或者通过这些洞对整体曲面进行分区，每个区内部是无洞的、可分别进行路径规划。

### 扫查点的位姿计算

- 主要是姿态一致性的问题。在曲率变化比较大的地方，相距很近的点间也会有很大的姿态变化，在扫查的时候显然是没法用的。这个问题好像在曲面求法线的时候是个很常见的问题，叫做“定向”，也有一些论文可以参考一下。<http://geometryhub.net/notes/pointcloudnormal>

- 还有一个很天马行空的想法：一系列的姿态其实就是一系列的信号，为了使扫查时的姿态过渡更平稳，是不是可以对这些姿态进行低通滤波？甚至还可以通过统计学分析做噪声过滤？（随手搜了一下好像还真有类似的做法<https://www.cnblogs.com/bforever/p/13152894.html>）
