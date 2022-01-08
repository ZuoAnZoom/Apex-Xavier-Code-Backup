/**
 * @file /include/robotcar_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robotcar_gui_QNODE_HPP_
#define robotcar_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/Twist.h> //用来订阅 speed
#include "gps.h" //用来订阅 gps info
#include <map>
#include <QDebug>
// 用于显示图像需要包含的头文件 __add_by_csy_20-12-20
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <QImage> // 用于图像数据编码格式转换的

////camera__add_by_csy_20-12-22
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <QImage>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotcar_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

    // 创建接口函数，订阅压缩图像数据
    void sub_image(QString topic_name);
    void sub_image_2(QString topic_name);
    void sub_camera_1(QString topic_name);
    void sub_camera_2(QString topic_name);
    void sub_camera_3(QString topic_name);
    void sub_camera_4(QString topic_name);
    void sub_camera_5(QString topic_name);
    void sub_camera_6(QString topic_name);
    void sub_camera_7(QString topic_name);
    void sub_camera_8(QString topic_name);


//    //camera__add_by_csy_20-12-22
//    void myCallback_img(const sensor_msgs::ImageConstPtr& msg);//camera callback function
//    QImage image;

    //获取话题列表
    QMap<QString, QString> get_topic_list();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    //自定义信号，发送压缩图像数据
    void image_vel(QImage);
    void image_vel_2(QImage);
    void camera_vel_1(QImage);
    void camera_vel_2(QImage);
    void camera_vel_3(QImage);
    void camera_vel_4(QImage);
    void camera_vel_5(QImage);
    void camera_vel_6(QImage);
    void camera_vel_7(QImage);
    void camera_vel_8(QImage);

    //自定义信号 发送 speed 给 Mainwindow
    void speed_vel(float, float);
    //自定义信号 发送 gps info 给 Mainwindow
    void gps_vel(QString, QString, QString);

//    //camera__add_by_csy_20-12-22
//    void loggingCamera();//发出设置相机图片信号

private:
	int init_argc;
	char** init_argv;
    //ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    //声明 压缩 图像订阅 回调函数
    ros::Subscriber image_suber;
    void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg);
    ros::Subscriber image_suber_2;
    void imageCallback_2(const sensor_msgs::CompressedImageConstPtr &msg);
    ros::Subscriber camera_suber_1;
    ros::Subscriber camera_suber_2;
    ros::Subscriber camera_suber_3;
    ros::Subscriber camera_suber_4;
    ros::Subscriber camera_suber_5;
    ros::Subscriber camera_suber_6;
    ros::Subscriber camera_suber_7;
    ros::Subscriber camera_suber_8;
    void cameraCallback_1(const sensor_msgs::CompressedImageConstPtr &msg);
    void cameraCallback_2(const sensor_msgs::CompressedImageConstPtr &msg);
    void cameraCallback_3(const sensor_msgs::CompressedImageConstPtr &msg);
    void cameraCallback_4(const sensor_msgs::CompressedImageConstPtr &msg);
    void cameraCallback_5(const sensor_msgs::CompressedImageConstPtr &msg);
    void cameraCallback_6(const sensor_msgs::CompressedImageConstPtr &msg);
    void cameraCallback_7(const sensor_msgs::CompressedImageConstPtr &msg);
    void cameraCallback_8(const sensor_msgs::CompressedImageConstPtr &msg);



    //转换图像数据编码格式
    QImage Mat2QImage(cv::Mat const& src);


    //声明 订阅速度 回调函数
    ros::Subscriber speed_sub;
    void speed_callback(const geometry_msgs::Twist &msg);
    //声明 订阅GPS信息 回调函数
    ros::Subscriber gps_sub;
    void gps_callback(const ublox_serial::gps::ConstPtr &msg);

//    //camera__add_by_csy_20-12-22
//    image_transport::Subscriber image_sub;
//    cv::Mat img;

};

}  // namespace robotcar_gui

#endif /* robotcar_gui_QNODE_HPP_ */
