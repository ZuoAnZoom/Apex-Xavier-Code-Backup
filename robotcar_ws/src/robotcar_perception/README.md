# 2021-12-05 崔善尧

和白丽娟修改了她的 障碍物检测部分。

确定依赖 opencv3，本机器上使用的版本为 opencv3.3.1+contrib3.3.1, 可以正常编译使用。

目前需要进一步验证在双目标定下的实时运行结果。



# 2021-12-03 崔善尧

本功能包内包含白丽娟的 传统双目障碍物检测 和 淑涛的 深度学习车道线检测。

障碍物检测代码中注释掉了条件编译的部分，即非 opencv4 情况下的代码，并且修改了 94 行的一处错误。编译的环境为 opencv4.5.0+contrib+cuda

车道线检测部分的代码目前还没有运行，需要安装 tensorflow，联系淑涛知他所需的环境为 tf1.10 版本。

再 AGX Xavier 上安装 tenorflow 可以参考 Nvidia 的官方教程，[链接](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html)。

这里，为了防止系统的 python 环境混乱，我选择使用虚拟环境进行安装。虚拟环境选用 virtualenv，而没有使用 conda。（考虑到 virtualenv 就够用了，conda 安装还会占用为数不多的空间）

安装的步骤如下：

```
$ sudo apt-get update
$ sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
$ sudo apt-get install python3-pip
$ sudo pip3 install -U pip testresources setuptools==49.6.0 
$ sudo pip3 install -U --no-deps numpy==1.19.4 future==0.18.2 mock==3.0.5 keras_preprocessing==1.1.2 keras_applications==1.0.8 gast==0.4.0 protobuf pybind11 cython pkgconfig
$ sudo env H5PY_SETUP_REQUIRES=0 pip3 install -U h5py==3.1.0

# Set up the Virtual Environment
$ cd Documents\
$ python3 -m virtualenv -p python3 venv-lane-det
$ source venv-lane-det/bin/activate

# Install the desired version of TensorFlow and its dependencies:
$ pip3 install -U numpy grpcio absl-py py-cpuinfo psutil portpicker six mock requests gast h5py astor termcolor protobuf keras-applications keras-preprocessing wrapt google-pasta setuptools testresources
$ pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v$JP_VERSION tensorflow==$TF_VERSION+nv$NV_VERSION
$ pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 tensorflow==1.15.2+nv20.06
```

但是，上面的步骤会有问题，在安装 h5py 的时候会出现 build 失败，搜索了全网，方案都失败。

最终，返回米文论坛，按照 [2楼和3楼](https://forum.miivii.com/forum.php?mod=viewthread&tid=272&highlight=tensorflow)的方法，安装成功。其安装在了系统 python3 环境中。

安装流程如下：

```
sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install python3-pip

# 换 pip 源，修改～/.pip/pip.conf，如果没有这个文件，就创建一个。内容如下：
----------
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
[install]
trusted-host=mirrors.aliyun.com
----------

sudo pip3 install -U pip testresources setuptools
sudo pip3 install -U numpy==1.16.1 future==0.17.1 mock==3.0.5 h5py==2.9.0 keras_preprocessing==1.0.5 keras_applications==1.0.8 gast==0.2.2 futures protobuf pybind11
sudo pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 tensorflow==1.15.2+nv20.06
```

由此安装成功，但是，在 venv 中还没有安装成果，等待继续尝试。

