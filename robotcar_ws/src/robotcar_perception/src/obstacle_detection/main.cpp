#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <camera_info_manager/camera_info_manager.h>
#include <iostream>
#include <fstream>
#include <string>
 
#include "obstacle_detection/StereoVisionForADAS.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <robotcar_map/obstacles.h>

using namespace std;
using namespace message_filters;

void imageCallback(const sensor_msgs::ImageConstPtr &left_image, const sensor_msgs::ImageConstPtr &right_image,
									 StereoCamParam_t objParam, CStereoVisionForADAS objStereoVision, ros::Publisher res_pub,
									ros::Publisher image_pub, ros::Publisher image_stixel_pub)
{
	robotcar_map::obstacles msg;
	msg.stamp = ros::Time::now();
	
	ROS_INFO_STREAM("befor cv::Mat");
	cv::Mat imgLeft(left_image->data);
	cv::Mat imgRight(right_image->data);

	imgLeft = imgLeft.reshape(4, 720).clone();
	imgRight = imgRight.reshape(4, 720).clone();

	cv::Mat img_split_l[4], img_merge_l[3], image_l;
	cv::split(imgLeft, img_split_l);
	img_merge_l[0] = img_split_l[0];
	img_merge_l[1] = img_split_l[1];
	img_merge_l[2] = img_split_l[2];
	cv::merge(img_merge_l, 3, image_l);
	imgLeft = image_l;

	cv::Mat img_split_r[4], img_merge_r[3], image_r;
	cv::split(imgRight, img_split_r);
	img_merge_r[0] = img_split_r[0];
	img_merge_r[1] = img_split_r[1];
	img_merge_r[2] = img_split_r[2];
	cv::merge(img_merge_r, 3, image_r);
	imgRight = image_r;

	// imshow("l", imgLeft);
	// imshow("r", imgRight);
	// waitKey(0);

	if (imgLeft.empty()) {waitKey();return;}

	int64 t = getTickCount();//GetTickcount函数：它返回从操作系统启动到当前所经过的毫秒数

	// 图片进行降采样
	pyrDown(imgLeft, imgLeft, Size(imgLeft.cols / 2, imgLeft.rows / 2));
	pyrDown(imgRight, imgRight, Size(imgRight.cols / 2, imgRight.rows / 2));

	// procesing
	objStereoVision.Objectness(imgLeft, imgRight); 

	// 发布检测结果
	msg.num = objStereoVision.m_vecobjBB.size();
	for (int i = 0; i < msg.num; i++)
	{
		msg.x_min.push_back(objStereoVision.m_vecobjBB[i].dZ);
		msg.x_max.push_back(objStereoVision.m_vecobjBB[i].zmaxDist);
		msg.y_min.push_back(objStereoVision.m_vecobjBB[i].xmaxDist);
		msg.y_max.push_back(objStereoVision.m_vecobjBB[i].xminDist);
	}
	res_pub.publish(msg);

	Mat imgDisp8;
	objStereoVision.m_matDisp16.convertTo(imgDisp8, CV_8U, 255 / (objParam.m_nNumberOfDisp*16.));
	//深度为１６bit转换为深度为８bit的图像

	Mat imgG;
	bitwise_and(objStereoVision.m_imgGround, imgDisp8, imgG);
	//对二进制数据进行“与”操作，即对图像（灰度图像或彩色图像均可）每个像素值进行二进制“与”操作，
	//1&1=1，1&0=0，0&1=0，0&0=0

	//objSuNoVeMap.Compute(imgDisp8); //创建地表法线
	printf("Time elapsed: %.3fms\n", (getTickCount() - t) * 1000 / getTickFrequency());
	
	Mat imgResult = imgLeft.clone();
	Mat imgStixel = imgLeft.clone();

	objStereoVision.Display(imgResult, imgStixel);
	
	// resize(imgLeft, imgLeft, Size(imgLeft.cols/2, imgLeft.rows /2));
	// resize(imgDisp8, imgDisp8, Size(imgDisp8.cols / 2, imgDisp8.rows / 2));
	// resize(imgDispColor, imgDispColor, Size(imgDispColor.cols / 2, imgDispColor.rows / 2));
	resize(imgResult, imgResult, Size(imgLeft.cols, imgLeft.rows));
	resize(imgStixel, imgStixel, Size(imgLeft.cols, imgLeft.rows));

	//imshow("left", imgLeft);
	// imshow("disp8", imgDisp8);
	//imshow("vec", imgDispColor);

	sensor_msgs::Image image_msg, image_stixel_msg;
	image_msg = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResult).toImageMsg());
	image_stixel_msg = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgStixel).toImageMsg());
	image_pub.publish(image_msg);
	image_stixel_pub.publish(image_stixel_msg);
	// imshow("result", imgResult);
	// imshow("stixel", imgStixel);

	int nWaitTime = 10;
	char chKey = waitKey(nWaitTime);
	if (chKey == 27) return;
	if (chKey == ' ') nWaitTime = !nWaitTime;
}

void spin()
{
  ros::spin();
}

// #define VIZ
#ifdef VIZ
#include <opencv2/viz.hpp>
#endif

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robotcar_vo");

	ros::NodeHandle nh;
	
	message_filters::Subscriber<sensor_msgs::Image> left_cam_sub(nh, "/miivii_gmsl_ros_A/camera1", 1, ros::TransportHints().tcpNoDelay());
	message_filters::Subscriber<sensor_msgs::Image> right_cam_sub(nh, "/miivii_gmsl_ros_A/camera2", 1, ros::TransportHints().tcpNoDelay());

	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
	Synchronizer<syncPolicy> sync(syncPolicy(10), left_cam_sub, right_cam_sub);  

	ros::Publisher obstacle_pub = nh.advertise<robotcar_map::obstacles>("perception/obstacles", 1);
	ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("perception/obstacles_image", 1);
	ros::Publisher image_stixel_pub = nh.advertise<sensor_msgs::Image>("perception/obstacles_stixel_image", 1);
	
	StereoCamParam_t objParam = CStereoVisionForADAS::InitStereoParam(benben3);//获取数据集的参数，
	//包括视差数，窗口大小，基线长度和最远距离参数
	CStereoVisionForADAS objStereoVision(objParam); //stixel constructor　stixel构造函数
	// CSuNoVeMap objSuNoVeMap(objParam); //SNV　根据视差图创建表面法线

	sync.registerCallback(boost::bind(&imageCallback, _1, _2, objParam, objStereoVision, obstacle_pub, image_pub, image_stixel_pub));

	ros::spin();
    
	// int cntFrame = 980;
	// char chLeftImageName[150] = {};
	// char chRightImageName[150] = {};
	// char chOxtsName[150] = {};
	// int nWaitTime = 10;
	
	// while (1){
		// (!plot3d.wasStopped()){
		// cout << cntFrame << ", ";

		// sprintf(chLeftImageName, "/media/bjer/KINGSTON/障碍物检测/201110/left/%d.png", cntFrame);
		// sprintf(chRightImageName, "/media/bjer/KINGSTON/障碍物检测/201110/right/%d.png", cntFrame);

		 //sprintf(chLeftImageName, "./picture/left/%d.jpg", cntFrame);
		 //sprintf(chRightImageName, "./picture/right/ht/%d.jpg", cntFrame);

		// sprintf(chLeftImageName, "./picture2/left/%06d.png", cntFrame);
		// sprintf(chRightImageName, "./picture2/right/%06d.png", cntFrame);


		// sprintf(chLeftImageName, "./data/left/%010d.png", cntFrame);
		// sprintf(chRightImageName, "./data/right/%010d.png", cntFrame);
		
		// cntFrame++;

		// Mat imgLeft = imread(chLeftImageName, 1);
		// Mat imgRight = imread(chRightImageName, 1);
		// if (imgLeft.empty()) {waitKey();break;}
		
		// //imshow("right", imgRight);
		
		// int64 t = getTickCount();//GetTickcount函数：它返回从操作系统启动到当前所经过的毫秒数

		// // procesing
		// objStereoVision.Objectness(imgLeft, imgRight); 

		// Mat imgDisp8;
		// objStereoVision.m_matDisp16.convertTo(imgDisp8, CV_8U, 255 / (objParam.m_nNumberOfDisp*16.));
		// //深度为１６bit转换为深度为８bit的图像

		// Mat imgG;
		// bitwise_and(objStereoVision.m_imgGround, imgDisp8, imgG);
		// //对二进制数据进行“与”操作，即对图像（灰度图像或彩色图像均可）每个像素值进行二进制“与”操作，
		// //1&1=1，1&0=0，0&1=0，0&0=0

	  // //objSuNoVeMap.Compute(imgDisp8); //创建地表法线
		// printf("Time elapsed: %.3fms\n", (getTickCount() - t) * 1000 / getTickFrequency());
		
		// Mat imgResult = imgLeft.clone();
		// Mat imgStixel = imgLeft.clone();

		// objStereoVision.Display(imgResult, imgStixel);
		
		// // resize(imgLeft, imgLeft, Size(imgLeft.cols/2, imgLeft.rows /2));
		// // resize(imgDisp8, imgDisp8, Size(imgDisp8.cols / 2, imgDisp8.rows / 2));
		// //resize(imgDispColor, imgDispColor, Size(imgDispColor.cols / 2, imgDispColor.rows / 2));
		// resize(imgResult, imgResult, Size(imgLeft.cols, imgLeft.rows));
		// resize(imgStixel, imgStixel, Size(imgLeft.cols, imgLeft.rows));
	
		// //imshow("left", imgLeft);
		// //imshow("disp8", imgDisp8);
	  // //imshow("vec", imgDispColor);
		// imshow("result", imgResult);
		// imshow("stixel", imgStixel);

		// char chKey = waitKey(nWaitTime);
		// if (chKey == 27) return 1;
		// if (chKey == ' ') nWaitTime = !nWaitTime;
		// if (chKey == '[') cntFrame -= 2;
		// }

	return 0;
}
