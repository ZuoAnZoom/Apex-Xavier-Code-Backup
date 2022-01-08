/**
 * @file /include/robotcar_gui/main_window.hpp
 *
 * @brief Qt based gui for robotcar_gui.
 *
 * @date November 2010
 **/
#ifndef robotcar_gui_MAIN_WINDOW_H
#define robotcar_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "qrviz.hpp"
#include "addtopics.h"
//仪表盘头文件
#include "CCtrlDashBoard.h"
//显示系统时间
#include <QTimer>
#include <QDateTime>
//termianl
#include <QProcess>
#include <QComboBox>
#include <QStandardItemModel>

#include <QTreeWidgetItem>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QInputDialog>
#include <QFileDialog>
#include <map>
//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include<rviz/tool.h>


namespace Ui {
class MainWindow;
}

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robotcar_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing
	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

    void initUIs(); // 初始化UI
    void initTopicList(); // 初始化 topicList
    void initRviz();//初始化 Rviz


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
//	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    //rviz
    void RvizGetModel(QAbstractItemModel *model);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    //自己声明的槽函数
    //订阅压缩图像
    void slot_sub_image();
    void slot_update_image(QImage);
    void slot_sub_image_2();
    void slot_update_image_2(QImage);
    void slot_sub_camera_1();
    void slot_sub_camera_2();
    void slot_sub_camera_3();
    void slot_sub_camera_4();
    void slot_sub_camera_5();
    void slot_sub_camera_6();
    void slot_sub_camera_7();
    void slot_sub_camera_8();
    void slot_update_camera_1(QImage);
    void slot_update_camera_2(QImage);
    void slot_update_camera_3(QImage);
    void slot_update_camera_4(QImage);
    void slot_update_camera_5(QImage);
    void slot_update_camera_6(QImage);
    void slot_update_camera_7(QImage);
    void slot_update_camera_8(QImage);
    //terminal
    void slot_cmd1_clicked();
    void slot_quick_output1();
    void slot_cmd2_clicked();
    void slot_quick_output2();
    //system time
    void slot_update_system_time();
    //car light
    void slot_car_light_toogle();
    //disconnet
    void slot_disconnect(); //点击断开连接按钮
    void slot_rosShutdown();
    //dashBoard
    void slot_update_dashBoard(float, float);
    //gps
    void slot_update_gps_info(QString, QString, QString);
    //topiclist
    void slot_refreshTopicList();
    //rviz
    void slot_move_camera_btn();
    void slot_set_select();
    void slot_choose_topic(QTreeWidgetItem *choose, QString name);
    void slot_set_2D_Goal();
    void slot_set_2D_Pos();



private slots:
    // Rviz
    void on_pushButton_add_topic_clicked();
    void on_pushButton_remove_topic_clicked();
    void on_pushButton_rename_topic_clicked();
    void on_treeView_rvizDisplayTree_clicked(const QModelIndex &index);
    /// \brief 导入rviz Display配置
    void on_pushButton_rvizReadDisplaySet_clicked();
    /// \brief 导出rviz Display配置
    void on_pushButton_rvizSaveDisplaySet_clicked();


private:
    Ui::MainWindowDesign *ui;
	QNode qnode;
    // 所有链接
    void connections();
    //terminal
    QProcess *user_cmd1;
    QProcess *user_cmd2;
    //仪表盘
    CCtrlDashBoard* speed_dashBoard;
    //car light
    bool flag_carlight = false;
    //car turn left right Thre
    float car_turnThre = 0.0;
    // rviz
    void initData();
    void inform(QString);
    bool AskInform(QString);
    QString JudgeDisplayNewName(QString name);
    QString getUserName();
    QRviz *map_rviz_ = nullptr;
    QStandardItemModel* treeView_rviz_model = nullptr;
    AddTopics *addtopic_form = nullptr;
    QAbstractItemModel* m_modelRvizDisplay;
    QMap<QString, QString> m_mapRvizDisplays;
    QString m_sRvizDisplayChooseName_;

};

}  // namespace robotcar_gui

#endif // robotcar_gui_MAIN_WINDOW_H
