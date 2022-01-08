# 2021-12-01 cuishanyao

测试通过：10个

robotcar_bringup;robotcar_general;robotcar_teleop

imu-atomic-pps;robotcar_collect_data;robotcar_controller;robotcar_map;robotcar_motion_planner;robotcar_sim

gmsl_camera: 本包为新包，和历史的版本不一样，因为 4.4 的依赖项目和 4.2 区别比较大。但是 launch 文件和保存相机标定文件的 config 文件夹都是从 4.2 迁移而来


```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="robotcar_bringup;robotcar_general;robotcar_teleop;imu-atomic-pps;robotcar_collect_data;robotcar_controller;robotcar_map;robotcar_motion_planner;robotcar_sim"
```


存在问题: 2个

robotcar_perception
robotcar_slam(应该是少 cuda)


