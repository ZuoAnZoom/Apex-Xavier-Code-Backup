# 2022-01-01 崔善尧

修改了 `1_node_with_4_cameras.launch`, `2_node_with_4_cameras.launch`, `2_nodes_with_8_cameras.launch`，增加了 fps 的参数设置，可以在 launch 的时候使用 `fps_A:=`，`fps_B:=`， `fps_AB:=` 来指定开启相机的 fps。

# 2021-12-18 崔善尧

修改了 gmsl demo 的源码，结合 stereo_image_proc 发布校正后的双目图像。但是，在 run_perception.sh 的时候，会因为 catkin_ws2 环境变量，导致 stereo_image_proc 启动失败。 流程上需要注意。



# 2021-12-03 崔善尧

gmsl_camera 功能包原本从 jetpack 4.2 版本的迁移而来，但是发现在 jetpack 4.4 版本中无法使用，因为系统大版本迭代依赖项变化比较大，github 上的代码是为 4.2 的编写的。经过询问米文，现在 gmsl 相机的 ros demo 程序在系统 /opt/miivii/features 目录下，于是拷贝出来，并编译。但是编译过程中出现 opencv 链接的一些问题，米文也无法解决，经过思考发现，原先 jetpack 4.4 版本自带 opencv4.1.1，也就是说该相机功能包是在 opencv4 的环境下编译通过的，但是因为其他包依赖 opencv3，于是我之前也安装了 opencv3.3.1+contrib，导致现在在编译功能包的时候，可能找到是 opencv3，从而导致现在编译存在链接问题，那么 CMakeList 中必然没有指定 opencv 的版本号，于是打开 CMake 文件，果然印证猜想，遂指定 opencv 版本为 4，编译即可通过。

