#include "myslam/System.h"
#include "myslam/Viewer.h"
#include "myslam/Map.h"
#include "myslam/ObjDetector_mod.h"
#include "myslam/ObjSegment_mod.h"
#include "myslam/Odometry_mod.h"
#include "myslam/Mapping_mod.h"
#include "myslam/Optimization_mod.h"
#include "myslam/LoopClosing_mod.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace message_filters;

std::vector<unsigned char> left_data_;
std::vector<unsigned char> right_data_;
ros::Time stamp_;

std::mutex mu;
bool recieve = false;

void imageCallback(const sensor_msgs::ImageConstPtr &left_image, const sensor_msgs::ImageConstPtr &right_image)
{   
    mu.lock();
    left_data_ = left_image->data;
    right_data_ = right_image->data;
    stamp_ = left_image->header.stamp;
    recieve = true;
    mu.unlock();
}

void spin()
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotcar_slam");

    ros::NodeHandle nh;
    tf::TransformBroadcaster tf_bc;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    
    tf::TransformListener tf(ros::Duration(10));

    message_filters::Subscriber<sensor_msgs::Image> left_cam_sub(nh, "/miivii_gmsl_ros_A/camera1", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> right_cam_sub(nh, "/miivii_gmsl_ros_A/camera2", 1, ros::TransportHints().tcpNoDelay());

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), left_cam_sub, right_cam_sub);  

    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    thread spin_topic(spin);
    spin_topic.detach();

    myslam::System::Ptr system(new myslam::System);

    system->initSystem();

    myslam::Map::Ptr local_map(new myslam::Map);    // ????????????
    myslam::Viewer::Ptr viewer(new myslam::Viewer); // ??????

    myslam::Segment::Ptr segment(new myslam::Segment);    // ????????????
    myslam::Detector::Ptr detector(new myslam::Detector); // ????????????

    myslam::Odometry::Ptr odometry(new myslam::Odometry);          // ?????????
    myslam::Mapping::Ptr mapping(new myslam::Mapping);             // ??????
    myslam::Optimization::Ptr optimizer(new myslam::Optimization); // ??????
    myslam::LoopClosing::Ptr loopclosing(new myslam::LoopClosing); // ??????

    // myslam::MotionClassifier::Ptr classifier(new myslam::MotionClassifier); // ??????????????????

    // // ???????????????
    // if (!system->initNetWork(detector, segment))
    //     return false;

    // ?????????
    thread thread_view(&myslam::Viewer::displayGUI, viewer, system);

    // ????????????????????????
    int i = 0;
    system->t1_ = clock();
    // while ((system->file_type_ == myslam::System::FileType::IMAGE) ? i < system->img0_files_.size() : system->isVideoOpen())
    while (ros::ok())
    {
        if (recieve == false)
        {
            continue;
        }

        mu.lock();
        vector<unsigned char> left_data = left_data_;
        vector<unsigned char> right_data = right_data_;
        ros::Time stamp = stamp_;
        recieve = false;
        mu.unlock();

        cout << "*********** Image: " << i << " ***********" << endl;

        // ??????????????????
        myslam::Frame::Ptr curr = myslam::Frame::createFrame();
        if (!system->readImage(i, curr, left_data, right_data))
            break;

        // ??????????????????????????????
        // system->readTruth(curr);

        // ?????????????????????
        odometry->trackFrame(curr, local_map);

        // ???????????????????????????
        // system->runNetWork(detector, segment, curr);

        // ??????????????????
        // classifier->detectDynaObjects(system->last_, curr, local_map);

        // ????????????
        mapping->buildMap(curr, system->last_, local_map);

        // ????????????
        // optimizer->optimiseMap(curr, local_map);

        // ??????GUI
        system->updateGUI(curr, local_map);

        // // ??????????????????
        // system->calcCurrError(curr);

        // ???????????????????????????
        // viewer->displayImage(system);

        // SE3 res = curr->T_wc_;
        // Sophus::Vector3d pos = res.translation();
        // float dx = pos[2];
        // float dy = -1 * pos[0];
        // float dyaw = -1 * res.angleY();
        // float yaw = yaw0 + dyaw;
        // float x = x0 + dx * cos(yaw0) - dy * sin(yaw0);
        // float y = y0 + dx * sin(yaw0) + dy * cos(yaw0);

        // // ????????????
        // tf::Transform transform;
        // transform.setOrigin(tf::Vector3(x, y, 0.0));
        // tf::Quaternion q;
        // q.setRPY(0, 0, yaw);
        // transform.setRotation(q);
        // tf_bc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "test"));

        // geometry_msgs::Point p;
        // p.x = x;
        // p.y = y;
        // traj.header.stamp = ros::Time::now();
        // traj.points.push_back(p);

        // if (traj.points.size() == 9999)
        //     traj.points.clear();

        // marker_pub.publish(traj);

        SE3 res = curr->T_wc_;
        Sophus::Vector3d pos = res.translation();
        float dx = pos[2];
        float dy = -1 * pos[0];
        float dyaw = -1 * res.angleY();
        float dis = sqrt(1.057 * 1.057 + 0.200 * 0.200);
        float ang = 3.14 / 4 + atan2(0.200, 1.057) + dyaw;

        // ??????????????????
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(dx - dis * cos(ang) + 3, dy - dis * sin(ang) + 3, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, dyaw);
        transform.setRotation(q);
        tf_bc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

        // ??????odom??????
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = dx - dis * cos(ang) + 3;
        odom.pose.pose.position.y = dy - dis * sin(ang) + 3;
        odom.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion q_;
        q_.x = q.getX();
        q_.y = q.getY();
        q_.z = q.getZ();
        q_.w = q.getW();
        odom.pose.pose.orientation = q_;
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;
        odom_pub.publish(odom);

        // ?????????
        system->updateFrame(curr);

        i++;
    }

    // ??????????????????
    system->outputResult();

    // ??????????????????
    // system->calcTotalRMSE();

    thread_view.join();
    return 0;
}
