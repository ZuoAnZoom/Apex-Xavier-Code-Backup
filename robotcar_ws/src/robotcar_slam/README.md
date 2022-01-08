# 2021-12-01 崔善尧

添加 README 文件，用于描述包的依赖项目

1. 模板类 sophus，include 部分为 .hpp

```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build && cmake .. && make -j8
sudo make install
```
	
2. Pangolin

```
git clone https://github.com/stevenlovegrove/Pangolin.git
git checkout -b v0.6
cd Pangolin
mkdir build && cd build && cmake .. && make -j8
sudo make install
```

如果提示缺少 glew，请 `sudo apt install libglew-dev`

3. g2o

```
git clone https://github.com/RainerKuemmerle/g2o.git
git checkout -b 20201223_git
cd g2o
mkdir build && cd build && cmake .. && make -j8
sudo make install
```

# 2021-12-03 崔善尧

本包依赖 opencv 和 cuda，于是编译了 opencv4.5.0+contrib+cuda, 编译的命令如下：

AGX Xavier 的算力，经过查询为 7.2

```bash
cmake -D WITH_CUDA=ON -D WITH_CUDNN=ON -D CUDA_ARCH_BIN="7.2" -D CUDA_ARCH_PTX="" -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH=/home/nvidia/workspace/third_lib/opencv4/opencv_contrib-4.5.0/modules -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python2=ON -D BUILD_opencv_python3=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
```

另外修改了 CMakeLists.txt 文件中的链接库部分，之前没有链接 fmt 导致编译出现 undefined reference 的问题，遂添加 fmt 的链接。

