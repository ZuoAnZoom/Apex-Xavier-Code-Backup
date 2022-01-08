/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robotcar_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotcar_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"robotcar_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    // 订阅速度
    speed_sub = n.subscribe("/cmd_vel", 1000, &QNode::speed_callback, this);
    // 订阅GPS
    gps_sub = n.subscribe("/gps_info", 1000, &QNode::gps_callback, this);

    start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"robotcar_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    // 订阅速度
    speed_sub = n.subscribe("/cmd_vel", 1000, &QNode::speed_callback, this);
    // 订阅GPS
    gps_sub = n.subscribe("/gps_info", 1000, &QNode::gps_callback, this);

    start();
	return true;
}

// 订阅压缩图像数据, 包含接口函数和回调函数
// image 1
void QNode::sub_image(QString topic_name)
{
    ros::NodeHandle n;
    image_suber = n.subscribe(topic_name.toStdString(), 1, &QNode::imageCallback, this);
    ros::spinOnce();
}
void QNode::imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
//            qDebug()<<"___in cv_bridge___";
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
//            qDebug()<<"___in cv_bridge___end___";
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit image_vel(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("video 1 exception: "+QString(e.what())).toStdString());
        return;
    }
//    clock_t start = clock();
//    clock_t end = clock();
//    qDebug()<<"____in_image_callback____"<<(end - start)*1000.0/CLOCKS_PER_SEC<<"\tms";
}

// image 2
void QNode::sub_image_2(QString topic_name)
{
    ros::NodeHandle n;
    image_suber_2 = n.subscribe(topic_name.toStdString(), 1, &QNode::imageCallback_2, this);
    ros::spinOnce();
}
void QNode::imageCallback_2(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit image_vel_2(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("video 2 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//camear1
void QNode::sub_camera_1(QString topic_name)
{
    ros::NodeHandle n;
    camera_suber_1 = n.subscribe(topic_name.toStdString(), 1, &QNode::cameraCallback_1, this);
    ros::spinOnce();
}
void QNode::cameraCallback_1(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit camera_vel_1(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("camera 1 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//camear2
void QNode::sub_camera_2(QString topic_name)
{
    ros::NodeHandle n;
    camera_suber_2 = n.subscribe(topic_name.toStdString(), 1, &QNode::cameraCallback_2, this);
    ros::spinOnce();
}
void QNode::cameraCallback_2(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit camera_vel_2(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("camera 2 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//camear3
void QNode::sub_camera_3(QString topic_name)
{
    ros::NodeHandle n;
    camera_suber_3 = n.subscribe(topic_name.toStdString(), 1, &QNode::cameraCallback_3, this);
    ros::spinOnce();
}
void QNode::cameraCallback_3(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit camera_vel_3(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("camera 3 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//camear4
void QNode::sub_camera_4(QString topic_name)
{
    ros::NodeHandle n;
    camera_suber_4 = n.subscribe(topic_name.toStdString(), 1, &QNode::cameraCallback_4, this);
    ros::spinOnce();
}
void QNode::cameraCallback_4(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit camera_vel_4(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("camera 4 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//camear5
void QNode::sub_camera_5(QString topic_name)
{
    ros::NodeHandle n;
    camera_suber_5 = n.subscribe(topic_name.toStdString(), 1, &QNode::cameraCallback_5, this);
    ros::spinOnce();
}
void QNode::cameraCallback_5(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit camera_vel_5(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("camera 5 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//camear6
void QNode::sub_camera_6(QString topic_name)
{
    ros::NodeHandle n;
    camera_suber_6 = n.subscribe(topic_name.toStdString(), 1, &QNode::cameraCallback_6, this);
    ros::spinOnce();
}
void QNode::cameraCallback_6(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit camera_vel_6(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("camera 6 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//camear7
void QNode::sub_camera_7(QString topic_name)
{
    ros::NodeHandle n;
    camera_suber_7 = n.subscribe(topic_name.toStdString(), 1, &QNode::cameraCallback_7, this);
    ros::spinOnce();
}
void QNode::cameraCallback_7(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit camera_vel_7(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("camera 7 exception: "+QString(e.what())).toStdString());
        return;
    }
}
//camear8
void QNode::sub_camera_8(QString topic_name)
{
    ros::NodeHandle n;
    camera_suber_8 = n.subscribe(topic_name.toStdString(), 1, &QNode::cameraCallback_8, this);
    ros::spinOnce();
}
void QNode::cameraCallback_8(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(msg->format.substr(0,4)=="rgba")
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);  //深拷贝转换为opencv类型
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
        }
        QImage im=Mat2QImage(cv_ptr->image);
        emit camera_vel_8(im);
    }
    catch (std::runtime_error& e)
    {
        log(Error,("camera 8 exception: "+QString(e.what())).toStdString());
        return;
    }
}

//定义图像数据格式转换
QImage QNode::Mat2QImage(cv::Mat const& src)
{
    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    const float scale = 255.0;
    if (src.depth() == CV_8U) {
        if (src.channels() == 1) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    int level = src.at<quint8>(i, j);
                    dest.setPixel(j, i, qRgb(level, level, level));
                }
            }
        } else if (src.channels() == 3) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
                    dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                }
            }
        }
    } else if (src.depth() == CV_32F) {
        if (src.channels() == 1) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    int level = scale * src.at<float>(i, j);
                    dest.setPixel(j, i, qRgb(level, level, level));
                }
            }
        } else if (src.channels() == 3) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
                    dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                }
            }
        }
    }
    return dest;
}


QMap<QString, QString> QNode::get_topic_list()
{
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    QMap<QString, QString> res;
    for (auto topic:topic_list)
    {
        res.insert(QString::fromStdString(topic.name), QString::fromStdString(topic.datatype));
    }
    return res;
}

//回调函数 订阅gps
void QNode::gps_callback(const ublox_serial::gps::ConstPtr &msg)
{
    QString q_gpsTime = QString::fromStdString(msg->gpsTime);
    QString q_lon = QString::fromStdString(msg->lon);
    QString q_lat = QString::fromStdString(msg->lat);
    emit gps_vel(q_gpsTime, q_lon, q_lat);
}

//回调函数 订阅速度
void QNode::speed_callback(const geometry_msgs::Twist &msg)
{
    float speed_x = msg.linear.x;
    float angular_z = msg.angular.z;
    emit speed_vel(speed_x, angular_z); //把这个speed信号发出去，记得要在 MainWindow 链接这个信号
}


void QNode::run() {
    ros::Rate loop_rate(30);//原来是 1 改为 30
    log(Info,"I'm running!");
//    int count = 0;
    while ( ros::ok() ) {
//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << "hello world " << count;
//        msg.data = ss.str();
//        chatter_publisher.publish(msg);
//        log(Info,std::string("I sent: ")+msg.data);

        ros::spinOnce();
        loop_rate.sleep();
//        ++count;
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace robotcar_gui
