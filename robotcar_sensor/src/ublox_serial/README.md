## 2020-12-17

- 注意要修改串口的权限，`sudo chmod a+rw /dev/ttyACM0`，不然可能会导致程序打不开串口
- 注意 `catkin make`的文件修改时间滞后的问题，可以解决。
- kinetic 和 melodic 版本需要安装 ros-<distro>-serial 包。
- noetic 版本没有serial 包，需要自行下载安装，方法如下

## noetic 版本安装和使用 serial

1、安装 Serial 包
ros-noetic 没有 serial 包，所以需要自行下载安装。步骤如下

首先进入 Downloads 目录

```
cd ~/Downloads
```

然后下载 serial 源码

```
git clone https://github.com/wjwwood/serial.git
```

进入 serial 目录，安装编译

```
cd serial
ccmake .
```

按 c 键，将 CMAKE_INSTALL_PREFIX 变量值改为 `/opt/ros/noetic`，再按 c 键

如果提示一长串以下开头

```
CMake Warning (dev) at CMakeLists.txt:2 (project):
Policy CMP0048 is not set: project() command manages VERSION variables.
Run “cmake --help-policy CMP0048” for policy details. Use the cmake_policy
command to set the policy and suppress this warning.

The following variable(s) would be set to empty:

  CMAKE_PROJECT_VERSION
  CMAKE_PROJECT_VERSION_MAJOR
  CMAKE_PROJECT_VERSION_MINOR
  CMAKE_PROJECT_VERSION_PATCH

This warning is for project developers. Use -Wno-dev to suppress it.
```

就按提示操作按 e

然后再按 g

之后回到命令行

```
make
sudo make install
```

安装完毕

测试，输入

```
roscd serial
```

若自动跳转到 serial 包内，则表示安装成功

2、使用 Serial 包
在 Serial 包下找到 CMakeLists.txt 的这一段

```
## Include headers

include_directories(include)
```

将其改为

```
## Include headers

include_directories(include  ${catkin_INCLUDE_DIRS})
```


在其他要用 Serial 的功能包中的 package.xml 文件中加入

```
<build_depend>serial</build_depend>
<build_export_depend>serial</build_export_depend>
```

在该包的 CMakeLists.txt 中的 find_package 中加入 serial 参数

然后就可以正常使用了
