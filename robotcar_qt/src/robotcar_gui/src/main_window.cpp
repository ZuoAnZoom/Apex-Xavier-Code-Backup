/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robotcar_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotcar_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindowDesign),
    qnode(argc,argv)
{
    ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
//    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    initUIs();//初始化UI__add_by_csy_20-12-21
    initData();

    ReadSettings();//读取配置文件
    setWindowIcon(QIcon("://images/car.png"));
    ui->tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
//    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
    ui->view_logging->setModel(qnode.loggingModel());
//    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    addtopic_form = new AddTopics();
    connect(addtopic_form, SIGNAL(Topic_choose(QTreeWidgetItem *, QString)), this, SLOT(slot_choose_topic(QTreeWidgetItem *, QString)));

    /*********************
    ** Auto Start
    **********************/
    if ( ui->checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    // 调用所有的链接 connect
    connections();
}



// 管理所有的 connect
void MainWindow::connections()
{
    // Rviz
    //MoveCamera
    connect(ui->move_camera_btn,SIGNAL(clicked()),this,SLOT(slot_move_camera_btn()));
    //Select
    connect(ui->set_select_btn,SIGNAL(clicked()),this,SLOT(slot_set_select()));
    //2D Pose
    connect(ui->set_pos_btn,SIGNAL(clicked()),this,SLOT(slot_set_2D_Pos()));
    //2D goal
    connect(ui->set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_2D_Goal()));

    // 显示压缩图像数据
    //image2
    connect(ui->pushButton_sub_image_2, SIGNAL(clicked()), this, SLOT(slot_sub_image_2()));
    connect(&qnode, SIGNAL(image_vel_2(QImage)), this, SLOT(slot_update_image_2(QImage)));
    //image1
    connect(ui->pushButton_sub_image, SIGNAL(clicked()), this, SLOT(slot_sub_image()));
    connect(&qnode, SIGNAL(image_vel(QImage)), this, SLOT(slot_update_image(QImage)));
    //camera1
    connect(ui->button_camera_1, SIGNAL(clicked()), this, SLOT(slot_sub_camera_1()));
    connect(&qnode, SIGNAL(camera_vel_1(QImage)), this, SLOT(slot_update_camera_1(QImage)));
    //camera2
    connect(ui->button_camera_2, SIGNAL(clicked()), this, SLOT(slot_sub_camera_2()));
    connect(&qnode, SIGNAL(camera_vel_2(QImage)), this, SLOT(slot_update_camera_2(QImage)));
    //camera3
    connect(ui->button_camera_3, SIGNAL(clicked()), this, SLOT(slot_sub_camera_3()));
    connect(&qnode, SIGNAL(camera_vel_3(QImage)), this, SLOT(slot_update_camera_3(QImage)));
    //camera4
    connect(ui->button_camera_4, SIGNAL(clicked()), this, SLOT(slot_sub_camera_4()));
    connect(&qnode, SIGNAL(camera_vel_4(QImage)), this, SLOT(slot_update_camera_4(QImage)));
    //camera5
    connect(ui->button_camera_5, SIGNAL(clicked()), this, SLOT(slot_sub_camera_5()));
    connect(&qnode, SIGNAL(camera_vel_5(QImage)), this, SLOT(slot_update_camera_5(QImage)));
    //camera6
    connect(ui->button_camera_6, SIGNAL(clicked()), this, SLOT(slot_sub_camera_6()));
    connect(&qnode, SIGNAL(camera_vel_6(QImage)), this, SLOT(slot_update_camera_6(QImage)));
    //camera7
    connect(ui->button_camera_7, SIGNAL(clicked()), this, SLOT(slot_sub_camera_7()));
    connect(&qnode, SIGNAL(camera_vel_7(QImage)), this, SLOT(slot_update_camera_7(QImage)));
    //camera8
    connect(ui->button_camera_8, SIGNAL(clicked()), this, SLOT(slot_sub_camera_8()));
    connect(&qnode, SIGNAL(camera_vel_8(QImage)), this, SLOT(slot_update_camera_8(QImage)));

    //termianl 1 2
    connect(ui->button_terminal1, SIGNAL(clicked()), this, SLOT(slot_cmd1_clicked()));
    connect(ui->button_terminal2, SIGNAL(clicked()), this, SLOT(slot_cmd2_clicked()));
    //system time
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(slot_update_system_time()));
    timer->start(1000);
    //cat light
    connect(ui->button_car_light, SIGNAL(clicked()), this, SLOT(slot_car_light_toogle()));
    //disconnect
    connect(ui->button_disconnect, SIGNAL(clicked()), this, SLOT(slot_disconnect()));
    //dashBoard
    connect(&qnode, SIGNAL(speed_vel(float, float)), this, SLOT(slot_update_dashBoard(float, float)));
    //gps
    connect(&qnode, SIGNAL(gps_vel(QString, QString, QString)), this, SLOT(slot_update_gps_info(QString, QString, QString)));
    //topiclist
    connect(ui->refresh_topic_btn, SIGNAL(clicked()), this, SLOT(slot_refreshTopicList()));
    // quit button
    connect(ui->quit_button, SIGNAL(clicked()), qApp, SLOT(quit())); //直接退出程序

}


// 初始化 UI
void MainWindow::initUIs()
{
    /**初始化控件显示**/
    ui->tabWidget->setCurrentIndex(0);
    ui->tab_manager->setCurrentIndex(0);
    // refresh topic list
    ui->refresh_topic_btn->setEnabled(false);
    //在线离线显示
    ui->label_robot_state_text->setStyleSheet("color:red;");
    //GPS
    ui->label_gps_time->setText("No data!");
    ui->label_gps_lat->setText("No data!");
    ui->label_gps_lon->setText("No data!");
    //power
    ui->progressBar_power->setValue(100);
    ui->label_power->setText("40.00V");
    //dashBoard
    speed_dashBoard = new CCtrlDashBoard(ui->widget_speed);
    speed_dashBoard->setGeometry(ui->widget_speed->rect());
    speed_dashBoard->setValue(20);
    //turn left right
    ui->label_linear_x->setText("0 km/h");
    ui->label_angular_z->setText("0 rad/s");
    ui->label_turn_left->setPixmap(QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
    ui->label_turn_right->setPixmap(QPixmap::fromImage(QImage("://images/turnRight_l.png")));
    // 压缩 图像显示
    ui->lineEdit_image_topic->setPlaceholderText("输入压缩图像话题名称");
    ui->lineEdit_image_topic_2->setPlaceholderText("输入压缩图像话题名称");
    ui->lineEdit_image_topic->setText("/miivii_gmsl_ros_A/camera3/compressed");
    ui->lineEdit_image_topic_2->setText("/miivii_gmsl_ros_A/camera4/compressed");
    ui->pushButton_sub_image->setEnabled(false);
    ui->pushButton_sub_image_2->setEnabled(false);
    ui->topic_camera_1->setPlaceholderText("输入压缩图像话题名称");
    ui->topic_camera_2->setPlaceholderText("输入压缩图像话题名称");
    ui->topic_camera_3->setPlaceholderText("输入压缩图像话题名称");
    ui->topic_camera_4->setPlaceholderText("输入压缩图像话题名称");
    ui->topic_camera_5->setPlaceholderText("输入压缩图像话题名称");
    ui->topic_camera_6->setPlaceholderText("输入压缩图像话题名称");
    ui->topic_camera_7->setPlaceholderText("输入压缩图像话题名称");
    ui->topic_camera_8->setPlaceholderText("输入压缩图像话题名称");
    ui->topic_camera_1->setText("/miivii_gmsl_ros_A/camera1/compressed"); //前左
    ui->topic_camera_2->setText("/miivii_gmsl_ros_A/camera3/compressed"); //前中
    ui->topic_camera_3->setText("/miivii_gmsl_ros_A/camera2/compressed"); //前右
    ui->topic_camera_4->setText("/miivii_gmsl_ros_B/camera1/compressed"); //Left
    ui->topic_camera_5->setText("/miivii_gmsl_ros_B/camera2/compressed"); //Right
    ui->topic_camera_6->setText("/miivii_gmsl_ros_B/camera3/compressed");
    ui->topic_camera_7->setText("/miivii_gmsl_ros_A/camera4/compressed"); //后中
    ui->topic_camera_8->setText("/miivii_gmsl_ros_B/camera4/compressed");
    ui->button_camera_1->setEnabled(false);
    ui->button_camera_2->setEnabled(false);
    ui->button_camera_3->setEnabled(false);
    ui->button_camera_4->setEnabled(false);
    ui->button_camera_5->setEnabled(false);
    ui->button_camera_6->setEnabled(false);
    ui->button_camera_7->setEnabled(false);
    ui->button_camera_8->setEnabled(false);

    // rviz 设置界面
    ui->move_camera_btn->setEnabled(false);
    ui->set_select_btn->setEnabled(false);
    ui->set_pos_btn->setEnabled(false);
    ui->set_goal_btn->setEnabled(false);
    ui->pushButton_add_topic->setEnabled(false);
    ui->pushButton_rename_topic->setEnabled(false);
    ui->pushButton_remove_topic->setEnabled(false);
    ui->pushButton_rvizReadDisplaySet->setEnabled(false);
    ui->pushButton_rvizSaveDisplaySet->setEnabled(false);
}


// 初始化 topiclist
void MainWindow::initTopicList()
{
    ui->topic_listWidget->clear();
    ui->topic_listWidget->addItem(QString("%1    (%2)").arg("Name", "Type"));
    QMap<QString, QString> topic_list = qnode.get_topic_list();
    for (QMap<QString, QString>::iterator iter = topic_list.begin(); iter!=topic_list.end(); iter++)
    {
        ui->topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
    }
}


void MainWindow::initData()
{
    m_mapRvizDisplays.insert("Axes", RVIZ_DISPLAY_AXES);
    m_mapRvizDisplays.insert("Camera", RVIZ_DISPLAY_CAMERA);
    m_mapRvizDisplays.insert("DepthCloud", RVIZ_DISPLAY_DEPTHCLOUD);
    m_mapRvizDisplays.insert("Effort", RVIZ_DISPLAY_EFFORT);
    m_mapRvizDisplays.insert("FluidPressure", RVIZ_DISPLAY_FLUIDPRESSURE);
    m_mapRvizDisplays.insert("Grid", RVIZ_DISPLAY_GRID);
    m_mapRvizDisplays.insert("GridCells", RVIZ_DISPLAY_GRIDCELLS);
    m_mapRvizDisplays.insert("Group", RVIZ_DISPLAY_GROUP);
    m_mapRvizDisplays.insert("Illuminance", RVIZ_DISPLAY_ILLUMINANCE);
    m_mapRvizDisplays.insert("Image", RVIZ_DISPLAY_IMAGE);
    m_mapRvizDisplays.insert("InterativerMarker", RVIZ_DISPLAY_INTERATIVEMARKER);
    m_mapRvizDisplays.insert("LaserScan", RVIZ_DISPLAY_LASERSCAN);
    m_mapRvizDisplays.insert("Map", RVIZ_DISPLAY_MAP);
    m_mapRvizDisplays.insert("Marker", RVIZ_DISPLAY_MARKER);
    m_mapRvizDisplays.insert("MarkerArray", RVIZ_DISPLAY_MARKERARRAY);
    m_mapRvizDisplays.insert("Odometry", RVIZ_DISPLAY_ODOMETRY);
    m_mapRvizDisplays.insert("Path", RVIZ_DISPLAY_PATH);
    m_mapRvizDisplays.insert("PointCloud", RVIZ_DISPLAY_POINTCLOUD);
    m_mapRvizDisplays.insert("PointCloud2", RVIZ_DISPLAY_POINTCLOUD2);
    m_mapRvizDisplays.insert("PointStamped", RVIZ_DISPLAY_POINTSTAMPED);
    m_mapRvizDisplays.insert("Polygon", RVIZ_DISPLAY_POLYGON);
    m_mapRvizDisplays.insert("Pose", RVIZ_DISPLAY_POSE);
    m_mapRvizDisplays.insert("PoseArray", RVIZ_DISPLAY_POSEARRAY);
    m_mapRvizDisplays.insert("PoseWithCovariance", RVIZ_DISPLAY_POSEWITHCOVARIANCE);
    m_mapRvizDisplays.insert("Range", RVIZ_DISPLAY_RANGE);
    m_mapRvizDisplays.insert("RelativeHumidity", RVIZ_DISPLAY_RELATIVEHUMIDITY);
    m_mapRvizDisplays.insert("RobotModel", RVIZ_DISPLAY_ROBOTMODEL);
    m_mapRvizDisplays.insert("TF", RVIZ_DISPLAY_TF);
    m_mapRvizDisplays.insert("Temperature", RVIZ_DISPLAY_TEMPERATURE);
    m_mapRvizDisplays.insert("WrenchStamped", RVIZ_DISPLAY_WRENCHSTAMPED);
}



void MainWindow::initRviz()
{
    ui->label_rvizShow->hide();
    map_rviz_=new QRviz(ui->verticalLayout_build_map,"qrviz");
    connect(map_rviz_, &QRviz::ReturnModelSignal, this, &MainWindow::RvizGetModel);
    map_rviz_->GetDisplayTreeModel();
    QMap<QString, QVariant> namevalue;
    namevalue.insert("Line Style", "Billboards");
    namevalue.insert("Color", QColor(160, 160, 160));
    namevalue.insert("Plane Cell Count", 10);
    map_rviz_->DisplayInit(RVIZ_DISPLAY_GRID, "Grid", true, namevalue);

    ui->pushButton_add_topic->setEnabled(true);
    ui->pushButton_rvizReadDisplaySet->setEnabled(true);
    ui->pushButton_rvizSaveDisplaySet->setEnabled(true);
}

void MainWindow::RvizGetModel(QAbstractItemModel *model)
{
    m_modelRvizDisplay = model;
    ui->treeView_rvizDisplayTree->setModel(model);
    ui->treeView_rvizDisplayTree->setColumnWidth(0, 180); //在设置模型之后设置列宽才能起作用
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots][add by csy]
*****************************************************************************/

//设置导航当前位置按钮的槽函数
void MainWindow::slot_set_2D_Pos()
{
 map_rviz_->Set_Pos();
// ui->label_map_msg->setText("请在地图中选择机器人的初始位置");
}
//设置导航目标位置按钮的槽函数
void MainWindow::slot_set_2D_Goal()
{
  map_rviz_->Set_Goal();
//  ui->label_map_msg->setText("请在地图中选择机器人导航的目标位置");
}

void MainWindow::slot_move_camera_btn()
{
    map_rviz_->Set_MoveCamera();
    qDebug()<<"move camera";
}
void MainWindow::slot_set_select()
{
    map_rviz_->Set_Select();
}

void MainWindow::slot_choose_topic(QTreeWidgetItem *choose, QString name)
{
    QString ClassID = choose->text(0);
    // 检查重名
    name = JudgeDisplayNewName(name);

    qDebug() << "choose topic ClassID:" << ClassID << ", name:" << name;
    QMap<QString, QVariant> namevalue;
    namevalue.clear();
    map_rviz_->DisplayInit(m_mapRvizDisplays[ClassID], name, true, namevalue);
}

///
/// \brief 检查重名
/// \param name
/// \return
///
QString MainWindow::JudgeDisplayNewName(QString name)
{
    if (m_modelRvizDisplay != nullptr)
    {
        bool same = true;
        while (same)
        {
            same = false;
            for (int i = 0; i < m_modelRvizDisplay->rowCount(); i++)
            {
                //m_sRvizDisplayChooseName = index.data().value<QString>();
                if (m_modelRvizDisplay->index(i, 0).data().value<QString>() == name)
                {
                    if (name.indexOf("_") != -1)
                    {
                        int num = name.section("_", -1, -1).toInt();
                        name = name.left(name.length() - name.section("_", -1, -1).length() - 1);
                        if (num <= 1)
                        {
                            num = 2;
                        }
                        else
                        {
                            num++;
                        }
                        name = name + "_" + QString::number(num);
                    }
                    else {
                      name = name + "_2";
                    }
                    same = true;
                    break;
                }
            }
        }
    }
    return name;
}


//订阅 压缩 图像
//image 1
void MainWindow::slot_sub_image()
{
    qnode.sub_image(ui->lineEdit_image_topic->text());
    qDebug()<<"__qDebugInfo__"<<ui->lineEdit_image_topic->text();
}
void MainWindow::slot_update_image(QImage im)
{
    ui->label_image->setPixmap(QPixmap::fromImage(im).scaled(ui->label_image->width(),ui->label_image->height()));
}
//image 2
void MainWindow::slot_sub_image_2()
{
    qnode.sub_image_2(ui->lineEdit_image_topic_2->text());
    qDebug()<<"__qDebugInfo__"<<ui->lineEdit_image_topic_2->text();
}
void MainWindow::slot_update_image_2(QImage im)
{
    ui->label_image_2->setPixmap(QPixmap::fromImage(im).scaled(ui->label_image_2->width(),ui->label_image_2->height()));
}

//camera1
void MainWindow::slot_sub_camera_1()
{
    qnode.sub_camera_1(ui->topic_camera_1->text());
    qDebug()<<"__qDebugInfo__"<<ui->topic_camera_1->text();
}
void MainWindow::slot_update_camera_1(QImage im)
{
    ui->label_camera_1->setPixmap(QPixmap::fromImage(im).scaled(ui->label_camera_1->width(),ui->label_camera_1->height()));
}
//camera2
void MainWindow::slot_sub_camera_2()
{
    qnode.sub_camera_2(ui->topic_camera_2->text());
    qDebug()<<"__qDebugInfo__"<<ui->topic_camera_2->text();
}
void MainWindow::slot_update_camera_2(QImage im)
{
    ui->label_camera_2->setPixmap(QPixmap::fromImage(im).scaled(ui->label_camera_2->width(),ui->label_camera_2->height()));
}
//camera3
void MainWindow::slot_sub_camera_3()
{
    qnode.sub_camera_3(ui->topic_camera_3->text());
    qDebug()<<"__qDebugInfo__"<<ui->topic_camera_3->text();
}
void MainWindow::slot_update_camera_3(QImage im)
{
    ui->label_camera_3->setPixmap(QPixmap::fromImage(im).scaled(ui->label_camera_3->width(),ui->label_camera_3->height()));
}
//camera4
void MainWindow::slot_sub_camera_4()
{
    qnode.sub_camera_4(ui->topic_camera_4->text());
    qDebug()<<"__qDebugInfo__"<<ui->topic_camera_4->text();
}
void MainWindow::slot_update_camera_4(QImage im)
{
    ui->label_camera_4->setPixmap(QPixmap::fromImage(im).scaled(ui->label_camera_4->width(),ui->label_camera_4->height()));
}
//camera5
void MainWindow::slot_sub_camera_5()
{
    qnode.sub_camera_5(ui->topic_camera_5->text());
    qDebug()<<"__qDebugInfo__"<<ui->topic_camera_5->text();
}
void MainWindow::slot_update_camera_5(QImage im)
{
    ui->label_camera_5->setPixmap(QPixmap::fromImage(im).scaled(ui->label_camera_5->width(),ui->label_camera_5->height()));
}
//camera6
void MainWindow::slot_sub_camera_6()
{
    qnode.sub_camera_6(ui->topic_camera_6->text());
    qDebug()<<"__qDebugInfo__"<<ui->topic_camera_6->text();
}
void MainWindow::slot_update_camera_6(QImage im)
{
    ui->label_camera_6->setPixmap(QPixmap::fromImage(im).scaled(ui->label_camera_6->width(),ui->label_camera_6->height()));
}
//camera7
void MainWindow::slot_sub_camera_7()
{
    qnode.sub_camera_7(ui->topic_camera_7->text());
    qDebug()<<"__qDebugInfo__"<<ui->topic_camera_7->text();
}
void MainWindow::slot_update_camera_7(QImage im)
{
    ui->label_camera_7->setPixmap(QPixmap::fromImage(im).scaled(ui->label_camera_7->width(),ui->label_camera_7->height()));
}
//camera8
void MainWindow::slot_sub_camera_8()
{
    qnode.sub_camera_8(ui->topic_camera_8->text());
    qDebug()<<"__qDebugInfo__"<<ui->topic_camera_8->text();
}
void MainWindow::slot_update_camera_8(QImage im)
{
    ui->label_camera_8->setPixmap(QPixmap::fromImage(im).scaled(ui->label_camera_8->width(),ui->label_camera_8->height()));
}



//termianl 1 2
void MainWindow::slot_cmd1_clicked()
{
    user_cmd1 = new QProcess;
    user_cmd1->start("bash");
    user_cmd1->write(ui->textEdit_terminal_input1->toPlainText().toLocal8Bit()+"\n");
    connect(user_cmd1, SIGNAL(readyReadStandardError()), this, SLOT(slot_quick_output1())); // !!!这个信号是,有错误输出时，会跳入后面的槽函数中
    connect(user_cmd1, SIGNAL(readyReadStandardOutput()),this, SLOT(slot_quick_output1())); // !!!这个信号是，有正常输出时，跳入后面的槽函数中
}
void MainWindow::slot_quick_output1()
{
    ui->textEdit_terminal_output1->append("<font color=\"#FF0000\">"+user_cmd1->readAllStandardError()+"</font>");//错误输出，红色
    ui->textEdit_terminal_output1->append("<font color=\"#FFFFFF\">"+user_cmd1->readAllStandardOutput()+"</font>");//正常输出，白色
}
void MainWindow::slot_cmd2_clicked()
{
    user_cmd2 = new QProcess;
    user_cmd2->start("bash");
    user_cmd2->write(ui->textEdit_terminal_input2->toPlainText().toLocal8Bit()+"\n");
    connect(user_cmd2, SIGNAL(readyReadStandardError()), this, SLOT(slot_quick_output2())); // !!!这个信号是,有错误输出时，会跳入后面的槽函数中
    connect(user_cmd2, SIGNAL(readyReadStandardOutput()),this, SLOT(slot_quick_output2())); // !!!这个信号是，有正常输出时，跳入后面的槽函数中
}
void MainWindow::slot_quick_output2()
{
    ui->textEdit_terminal_output2->append("<font color=\"#FF0000\">"+user_cmd2->readAllStandardError()+"</font>");//错误输出，红色
    ui->textEdit_terminal_output2->append("<font color=\"#FFFFFF\">"+user_cmd2->readAllStandardOutput()+"</font>");//正常输出，白色
}


void MainWindow::slot_update_system_time()
{
    QDateTime time = QDateTime::currentDateTime();
    QString qstr = time.toString("yyyy-MM-dd hh:mm:ss dddd");
    ui->label_system_time->setText(qstr);
}

void MainWindow::slot_car_light_toogle()
{
    if (!flag_carlight)
    {
        flag_carlight = true;
        ui->label_car_light_image->setPixmap(QPixmap::fromImage(QImage("://images/switch-on (1).png")));
    }
    else
    {
        flag_carlight = false;
        ui->label_car_light_image->setPixmap(QPixmap::fromImage(QImage("://images/switch-off (1).png")));
    }
}

void MainWindow::slot_disconnect()
{
    ros::shutdown();
    slot_rosShutdown();

    map_rviz_->quit();
    delete map_rviz_;
    map_rviz_ = nullptr;
    ui->pushButton_add_topic->setEnabled(false);
    ui->pushButton_remove_topic->setEnabled(false);
    ui->pushButton_rename_topic->setEnabled(false);
    ui->pushButton_rvizReadDisplaySet->setEnabled(false);
    ui->pushButton_rvizSaveDisplaySet->setEnabled(false);
    ui->label_rvizShow->show();
}



void MainWindow::slot_refreshTopicList()
{
    initTopicList();
}

void MainWindow::slot_update_gps_info(QString gpsTime, QString lon, QString lat)
{
    ui->label_gps_time->setText(gpsTime);
    ui->label_gps_lon->setText(lon);
    ui->label_gps_lat->setText(lat);
}

void MainWindow::slot_update_dashBoard(float speed_value, float angular_value)
{
    ui->label_linear_x->setText(QString::number(speed_value*3.6)+" km/s");
    ui->label_angular_z->setText(QString::number(angular_value)+" rad/s");
    speed_dashBoard->setValue(abs(speed_value)); // !!!目前单位 m/s 等待单位转换为 km/h
    // 这里可能要对 angular_value 进行一些转换
    if (angular_value > car_turnThre)
    {
        ui->label_turn_left->setPixmap(QPixmap::fromImage(QImage("://images/turnLeft_hl.png")));
        ui->label_turn_right->setPixmap(QPixmap::fromImage(QImage("://images/turnRight_l.png")));
    }
    else if (angular_value < car_turnThre)
    {
        ui->label_turn_right->setPixmap(QPixmap::fromImage(QImage("://images/turnRight_hl.png")));
        ui->label_turn_left->setPixmap(QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
    }
    else
    {
        ui->label_turn_left->setPixmap(QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
        ui->label_turn_right->setPixmap(QPixmap::fromImage(QImage("://images/turnRight_l.png")));
    }
}

void MainWindow::slot_rosShutdown()
{
    // 修改 shutdown 后的状态显示
    // 关闭连接后 rviz 按钮修改
    ui->move_camera_btn->setEnabled(false);
    ui->set_select_btn->setEnabled(false);
    ui->set_pos_btn->setEnabled(false);
    ui->set_goal_btn->setEnabled(false);
    // 关闭后, 修改订阅图像按钮显示
    ui->pushButton_sub_image->setEnabled(false);
    ui->pushButton_sub_image_2->setEnabled(false);
    ui->button_camera_1->setEnabled(false);
    ui->button_camera_2->setEnabled(false);
    ui->button_camera_3->setEnabled(false);
    ui->button_camera_4->setEnabled(false);
    ui->button_camera_5->setEnabled(false);
    ui->button_camera_6->setEnabled(false);
    ui->button_camera_7->setEnabled(false);
    ui->button_camera_8->setEnabled(false);
    // 在线 掉线 显示
    ui->label_robot_state_image->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
    ui->label_robot_state_text->setStyleSheet("color:red;");
    ui->label_robot_state_text->setText("离线");
    ui->button_connect->setEnabled(true);
    // gps 显示
    ui->label_gps_time->setText("No data!");
    ui->label_gps_lat->setText("No data!");
    ui->label_gps_lon->setText("No data!");
    // dashboard 显示 turn left right speed angular
    speed_dashBoard->setValue(0);
    ui->label_linear_x->setText("0 km/h");
    ui->label_angular_z->setText("0 rad/s");
    ui->label_turn_left->setPixmap(QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
    ui->label_turn_right->setPixmap(QPixmap::fromImage(QImage("://images/turnRight_l.png")));
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
//	msgBox.setText("Couldn't find the ros master.");
    msgBox.setWindowTitle("错误提示");
    msgBox.setIconPixmap(QPixmap::fromImage(QImage("://images/error.png")));
    msgBox.setText("未找到 ROS Master!");
	msgBox.exec();
//    close(); //主界面不关闭
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */
void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui->checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            showNoMasterMessage();
        } else {
            ui->button_connect->setEnabled(false);
            //modi 3 修改连接后图标状态显示__add_by_csy_20-12-22
            ui->label_robot_state_image->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
            ui->label_robot_state_text->setText("在线");
            ui->label_robot_state_text->setStyleSheet("color:green;");
            // 修改 连接后订阅图像按钮显示
            ui->pushButton_sub_image->setEnabled(true);
            ui->pushButton_sub_image_2->setEnabled(true);
            ui->button_camera_1->setEnabled(true);
            ui->button_camera_2->setEnabled(true);
            ui->button_camera_3->setEnabled(true);
            ui->button_camera_4->setEnabled(true);
            ui->button_camera_5->setEnabled(true);
            ui->button_camera_6->setEnabled(true);
            ui->button_camera_7->setEnabled(true);
            ui->button_camera_8->setEnabled(true);
            // 修改连接后 topic list 刷新按钮显示
            ui->refresh_topic_btn->setEnabled(true);
            // 连接后修改 rviz 按钮显示
            ui->move_camera_btn->setEnabled(true);
            ui->set_select_btn->setEnabled(true);
            ui->set_pos_btn->setEnabled(true);
            ui->set_goal_btn->setEnabled(true);
        }
    } else {
        if ( ! qnode.init(ui->line_edit_master->text().toStdString(),
                   ui->line_edit_host->text().toStdString()) ) {
            showNoMasterMessage();
        } else {
            ui->button_connect->setEnabled(false);
            ui->line_edit_master->setReadOnly(true);
            ui->line_edit_host->setReadOnly(true);
            ui->line_edit_topic->setReadOnly(true);
            //modi 3 修改连接后图标状态显示__add_by_csy_20-12-22
            ui->label_robot_state_image->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
            ui->label_robot_state_text->setText("在线");
            ui->label_robot_state_text->setStyleSheet("color:green;");
            // 修改 连接后订阅图像按钮显示
            ui->pushButton_sub_image->setEnabled(true);
            ui->pushButton_sub_image_2->setEnabled(true);
            ui->button_camera_1->setEnabled(true);
            ui->button_camera_2->setEnabled(true);
            ui->button_camera_3->setEnabled(true);
            ui->button_camera_4->setEnabled(true);
            ui->button_camera_5->setEnabled(true);
            ui->button_camera_6->setEnabled(true);
            ui->button_camera_7->setEnabled(true);
            ui->button_camera_8->setEnabled(true);
            // 修改连接后 topic list 刷新按钮显示
            ui->refresh_topic_btn->setEnabled(true);
            // 连接后修改 rviz 按钮显示
            ui->move_camera_btn->setEnabled(true);
            ui->set_select_btn->setEnabled(true);
            ui->set_pos_btn->setEnabled(true);
            ui->set_goal_btn->setEnabled(true);
        }
    }
    //初始化rviz
    initRviz();
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
    ui->line_edit_master->setEnabled(enabled);
    ui->line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui->view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

//void MainWindow::on_actionAbout_triggered() {
//    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
//}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robotcar_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui->line_edit_master->setText(master_url);
    ui->line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui->checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui->checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui->line_edit_master->setEnabled(false);
        ui->line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}


void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robotcar_gui");
    settings.setValue("master_url",ui->line_edit_master->text());
    settings.setValue("host_url",ui->line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui->checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui->checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}


void MainWindow::inform(QString strdata)
{
    QMessageBox m_r;
    m_r.setWindowTitle("提示");
    m_r.setText(strdata);
    m_r.exec();
}
bool MainWindow::AskInform(QString strdata)
{
    QMessageBox m_r;

    m_r.setWindowTitle("提示");
    m_r.setText(strdata);
    m_r.addButton(tr("确认"), QMessageBox::ActionRole);
    m_r.addButton(tr("取消"), QMessageBox::ActionRole);

    int isok = m_r.exec();
    if (isok == 1)
    {
        return false;
    }
    else
    {
        return true;
    }
}




}  // namespace robotcar_gui


//------------------在 namespace 之外了---------------------------



///
/// \brief 增加topic
///
void robotcar_gui::MainWindow::on_pushButton_add_topic_clicked()
{
    addtopic_form->show();
}

///
/// \brief 删除topic
///
void robotcar_gui::MainWindow::on_pushButton_remove_topic_clicked()
{
    if (ui->treeView_rvizDisplayTree->currentIndex().row() >= 0)
    {
        m_sRvizDisplayChooseName_ = ui->treeView_rvizDisplayTree->currentIndex().data().value<QString>();
        map_rviz_->RemoveDisplay(m_sRvizDisplayChooseName_);
        if (ui->treeView_rvizDisplayTree->currentIndex().row() >= 0)
        {
            on_treeView_rvizDisplayTree_clicked(ui->treeView_rvizDisplayTree->currentIndex());
        }
        else
        {
            m_sRvizDisplayChooseName_.clear();
        }
    }
    else
    {
        inform("请选择Display后再执行此操作");
    }
}

///
/// \brief 重命名topic
///
void robotcar_gui::MainWindow::on_pushButton_rename_topic_clicked()
{
    if (ui->treeView_rvizDisplayTree->currentIndex().row() < 0)
    {
        inform("请选择Display后再执行此操作");
        return ;
    }
    QString dlgTitle = "重命名";
    QString txtlabel = "请输入名字：";
    QString defaultInupt = m_sRvizDisplayChooseName_;
    QLineEdit::EchoMode echoMode = QLineEdit::Normal;
    bool ok = false;
    QString newname = QInputDialog::getText(this, dlgTitle, txtlabel, echoMode, defaultInupt, &ok);
    if (ok && !newname.isEmpty())
    {
        if (newname != defaultInupt)
        {
            QString nosamename = JudgeDisplayNewName(newname);
            map_rviz_->RenameDisplay(defaultInupt, nosamename);
            m_sRvizDisplayChooseName_ = nosamename;
            if (nosamename != newname)
            {
                inform("命名重复！命名已自动更改为" + nosamename);
            }
        }
    }
    else if (ok)
    {
        inform("输入内容为空，重命名失败");
    }
}


void robotcar_gui::MainWindow::on_treeView_rvizDisplayTree_clicked(const QModelIndex &index)
{
    m_sRvizDisplayChooseName_ = index.data().value<QString>();
    if (index.parent().row() == -1)   // Display 的根节点
    {
        if (index.row() > 1)
        {
            ui->pushButton_remove_topic->setEnabled(true);
            ui->pushButton_rename_topic->setEnabled(true);
        }
        else
        {
            ui->pushButton_remove_topic->setEnabled(false);
            ui->pushButton_rename_topic->setEnabled(false);
        }
    }
    else
    {
        ui->pushButton_remove_topic->setEnabled(false);
        ui->pushButton_rename_topic->setEnabled(false);
    }
}

void robotcar_gui::MainWindow::on_pushButton_rvizReadDisplaySet_clicked()
{
    if (map_rviz_ == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getOpenFileName(this, "导入 RVIZ Display 配置", "/home/" + getUserName() + "/", "YAML(*.yaml);;ALL(*.*)");
    if (!path.isEmpty())
    {
        map_rviz_->ReadDisplaySet(path);
    }
}

void robotcar_gui::MainWindow::on_pushButton_rvizSaveDisplaySet_clicked()
{
    if (map_rviz_ == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getSaveFileName(this, "导出 RVIZ Display 配置", "/home/" + getUserName() + "/", "YAML(*.yaml)");

    if (!path.isEmpty())
    {
        if (path.section('/', -1, -1).indexOf('.') < 0)
        {
            path = path + ".yaml";
        }
        map_rviz_->OutDisplaySet(path);
    }
}

QString robotcar_gui::MainWindow::getUserName()
{
    QString userPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    QString userName = userPath.section("/", -1, -1);
    return userName;
}
