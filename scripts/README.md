# 2022-01-07 崔善尧

- 修改 `open_lidar.sh`, 把两个 lidar 的 frame_id 均改成了 `livox_frame`

# 2021-12-30 崔善尧

- 新增 `open-imu.sh` 脚本，用来播发 razor_imu_9dof 的数据。


# 2021-12-15 崔善尧

- 新增 `open_gps_nmea.sh` 脚本，脚本采用 nmea_navsat_driver 包来自动播发解析 nmea 后的定位结果等信息。


# 2021-12-13 崔善尧

- 修改 `open_car_chassi.sh` 脚本，判断 roscore 是否在运行，选择性执行 roscore
- 增加 shell 打开 gnome-terminal，免去手动。注意，如果需要 new tab 驻留，在命令的最后加入 exec bash
- 修改了 bring up 的代码，在程序中配置用户密码为运行参数，密码从脚本传入到程序中，在执行时候自动输入管理员密码。
- 修改 `start_key_control.sh` 脚本名称为 `open_key_control.sh`，自动打开新 tab，并且不再选择是否退出底盘控制。
- 修改 `start_frp_remote.sh` 脚本名称为 `open_frp_remote.sh`
- 修改 `run_percepiton.sh` 脚本内容，自动新建一个 tab
- 新增 `open_gps.sh` 脚本，会自动打开 gps 并播发数据

脚本的命名规则统一：`open_xxx.sh` 为打开什么设备或程序，一般为单个部分。`record_xxx.sh` 为记录什么什么数据，可以根据任务自己定制。`run_xxx.sh` 为启动什么什么任务，一般为多个部分的组合任务。


# 2021-12-06 崔善尧

新增 `run_perception.sh` 脚本，用于开启感知模块，包括双目障碍物检测和车道线检测，目前车道线检测还没能完全迁移，在 launch 文件中已经被我注释掉了。
新增 `start_frp_remote.sh` 脚本，用于开启 frp 远程客户端，方便远程操控米文大脑。

