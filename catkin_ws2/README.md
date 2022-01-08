# 2021-12-14 崔善尧

这个工作空间主要是为了解决车道线感知部分的问题而创建的，该工作空间的具体创建请参考：https://blog.csdn.net/weixin_42675603/article/details/107785376

```
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge

mkdir -p catkin_workspace/src
cd catkin_workspace
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
catkin config --install

git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
apt-cache show ros-melodic-cv-bridge | grep Version
cd src/vision_opencv/
git checkout 1.13.0
cd ../../

catkin build 或者 catkin build cv_bridge
```




# 2021-12-01 崔善尧

该工作空间中的包，编译需要依赖 opencv-3.3.1 && opencv-contrib-3.3.1
我的编译命令如下：
```
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=<YOUR_PATH_TO_OPENCV_CONTRIB_FOLDER>/opencv_contrib-3.3.1/modules -D BUILD_EXAMPLES=ON ..
```

