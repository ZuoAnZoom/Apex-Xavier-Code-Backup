[toc]

# lslidar_c16
The `lslidar_c16` package is a linux ROS driver for lslidar c16 from Shenzhen Leishen Intelligence System Co, Ltd.

This package is tested on Ubuntu 14.04 with ROS indigo and Ubuntu 16.04 with ROS kinetic.


## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
```

# ROS 

## lslidar_c16_driver

### Parameters

- `lidar_ip` (`string`, `default: 192.168.1.200`)
By default, the IP address of the device is 192.168.1.200.

- `device_port` (`int`, `default: 2368`)
By default, the Port address of the device is 2368.

- `frame_id` (`string`, `default: laser`)
The frame ID entry for the sent messages.

- `add_multicast` (`bool`, `default: false`)
Whether using group multicast or not

- `group_ip` (`string`, `default: 234.2.3.2`)
The group IP address of the device. Only useful when setting add_multicast to true

### Published Topics

- `lslidar_packets` (`lslidar_c16_msgs/LslidarC16Packet`)
Each message corresponds to a lslidar packet sent by the device through the Ethernet.

## lslidar_c16_decoder

## Parameters

- `min_range` (`double`, `0.15`)

- `max_range` (`double`, `150.0`)
Points outside this range will be removed.

- `frequency` (`frequency`, `10.0`)
Note that the driver does not change the frequency of the sensor. 

- `publish_point_cloud` (`bool`, `true`)
If set to true, the decoder will send out a full revolution point cloud data

- `use_gps_ts` (`bool`, `false`)
If set to true, the decoder synchronized with GPS timestamp
## Published Topics

- `lslidar_sweep` (`lslidar_c16_msgs/LslidarC16Sweep`)
The message arranges the points within each sweep based on its scan index and azimuth.

- `lslidar_point_cloud` (`sensor_msgs/PointCloud2`)
This is only published when the `publish_point_cloud` is set to `true` in the launch file.

- `angle3_disable_min` (`double`, `-1`)
Ignore 3D point cloud data from angle3_disable_min to angle3_disable_max; Default -1 means disable this function; Value should from 0 to 2*pi

- `angle3_disable_max` (`double`, `-1`)
Ignore 3D point cloud data from angle3_disable_min to angle3_disable_max; Default -1 means disable this function; Value should from 0 to 2*pi



# Usage
Before running lslidar-c16 driver, make sure you set up IP address correctly.
Using following command to check if device is communicating. Replace `eth0` with your correct computer network interface

``` shell
sudo tcpdump -i eth0
```

Running following commmand to launch lslidar c16 driver

``` shell
roslaunch lslidar_c16_decoder lslidar_c16.launch --screen
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.



# FAQ
## Cannot open UDP port...
Make Sure your IP setting for bot device IP and computer IP is correct.
Default device IP is 192.168.1.200 and port is 2368.
Default computer IP is 192.168.1.102


## lslidar poll() timeout
 Make Sure your IP setting for bot device IP and computer IP is correct.
Default device IP is 192.168.1.200 and port is 2368.
Default computer IP is 192.168.1.102


# version track
Author: Yutong

## ver1.2 Yutong
1. Add group multicast function
2. Add truncate 3D angle area data


## ver1.1  Yutong
Using new message type to distinguish different channel data
- topic name: scan_channel
- topic type: LslidarC16Layer
- details: LslidarC16Layer is consist of 16 sets data for different channel, each set of data is represented by Sensor_msgs/LaserScan rosmessage type

Usage: 
``` shell
rostopic echo /scan_channel  will output all 16 channels data
rostopic echo /scan_channel/scan_channel[*]  (* can be from 0 to 15 represents channel num)  --> output data will be sensor_msgs/LaserScan message type
```
Example: 
There is an example script to show you how to obtain each channel data, located at /lslidar_c16_decoder/scripts/Test_MultiChannel.py
You will need python package numpy and matplotlib.pyplot(optional) to fully run this script


## ver1.05 Yutong
Using rostopic to select the channel you wish to output
topic name: layer_num
topic type: std_msgs/Int8
details: send layer number to topic layer_num 
Usage: rostopic pub /layer_num std_msgs/Int8 "data: 5"  --> output channel 5 data to topic /scan, message type is sensor_msgs/LaserScan . data number can only from 0 to 15




