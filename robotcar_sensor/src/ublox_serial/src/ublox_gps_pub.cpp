#include "ros/ros.h" 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>//string转化为double
#include <iomanip>//保留有效小数
#include <fstream>
#include <iostream>
#include <time.h>
#include "ublox_serial/gps.h"
#include "ublox_serial/ublox.h"


serial::Serial ser; //声明串口对象


int GPS_Data_Check(std::string hex_n, int dex)
{
    std::string dex_con_hex;
    std::string temp;
    int j=0;
    if(dex<=15)
    {
        std::string temp="";
        if(dex<10)
        {
            temp+=dex+'0';
        }
        else
        {
            temp+=dex-10+'A';
        }
        dex_con_hex=temp;
        for(int i=temp.length()-1;i>=0;i--)
        {
            dex_con_hex[j]=temp[i];
            j++;
        }
    }
    else
    {
        std::string temp="";
        do
        {
            int dex_tmp=dex%16;
            if(dex_tmp<10)
            {
                temp+=dex_tmp+'0';
            }
            else
            {
                temp+=dex_tmp-10+'A';
            }
            dex/=16;
        }while(dex>=16);
        if(dex<10)
        {
            temp+=dex+'0';
        }
        else
        {
            temp+=dex-10+'A';
        }
        dex_con_hex=temp;
        for(int i=temp.length()-1;i>=0;i--)
        {
            dex_con_hex[j]=temp[i];
            j++;
        }
    }
    int x;
    int y;
    if(dex_con_hex.compare(hex_n))//如果两个字符串相等则为0
    {
        return 0;
    }
    else
    {
        return 1;
        
    }
}

int GPS_Data_Check(char a[], int length)
{
    int head_n=6;
    std::string head_str; //帧头
    std::string CS_str; //数据帧校验位
    int flag=1; //数据标志位
    int CS_dex; 
    unsigned char CS=0;
    int chang=0;
    for(int i=0;i<length-2;i++)
    {
        if(i<head_n)
        {
            head_str+=a[i];
        }
        if(flag==3)
        {
            CS_str+=a[i];//在一帧数据中本来存在的校验位
        }
        if(a[i]=='*')
        {
            flag=3;
        }
        if(flag==2)
        {
            CS^=a[i];//异或出来的校验位
            chang++;
        }
        if(a[i]=='$')
        {
            flag=2;
        }
    }
    CS_dex=CS;  //十进制ASCLL码
    if((head_str=="$GPFPD")&&(GPS_Data_Check(CS_str,CS_dex)))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int decode_RMC(uint8_t data[], int length, RMC &outData)
{
    int num = 0;
    int flag = 0;
    
    // if (GPS_Data_Check(data, length) == 0 )
    // {
    //     std::cout<<"failed"<<std::endl;
    //     return -1; //校验不通过
    // }
    
    for (int i=0; i<length; i++)
    {
        if (data[i] == ',')
        {
            num++; //统计逗号
        }
        else 
        {
            if (num == 0)      {outData.header += data[i];}
            else if (num == 1) {outData.gpsTime += data[i];}
            else if (num == 2) {outData.status += data[i];}
            else if (num == 3) {outData.lat += data[i];}
            else if (num == 4) {outData.nsIndicator += data[i];}
            else if (num == 5) {outData.lon += data[i];}
            else if (num == 6) {outData.ewIndicator += data[i];}
            else if (num == 7) {outData.spd += data[i];}
            else if (num == 8) {outData.cog += data[i];}
            else if (num == 9) {outData.date += data[i];}
            else if (num == 10){outData.mv += data[i];}
            else if (num == 11){outData.mvEW += data[i];}
            else if (num == 12){outData.poseMode += data[i];}
            else if (num == 13)
            {
                if (flag < 1) {outData.navStatus += data[i];}
                else if (flag>=2 && flag<=3) {outData.checksum += data[i];}
                else if (flag >=4) {outData.CrLf += data[i];}
                flag++;
            }
        }
        outData.str += data[i];
    }
    return 0;
}


int main(int argc, char** argv)
{
    time_t now = time(NULL);
    struct  tm *p;
    p = gmtime(&now);
    char filename[256] = {0};
    char adr[]={"/home/nvidia/workspace/DATA_RECORD/gps_data/"};
    sprintf(filename, "gps-data-%s%d-%d-%d-%d-%d-%d.txt", adr, 1900+p->tm_year,1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min,p->tm_sec);
    std::fstream UBXdata;
    
    ros::init(argc, argv, "ublox_gps_pub"); //解析参数，命名节点
    ros::NodeHandle nh; //创建节点句柄
    ublox_serial::gps msg; //创建 gps 消息 msg
    ros::Publisher pub = nh.advertise<ublox_serial::gps>("/gps_info", 1000); //创建 pub

    try
    {
      //设置串口
      ser.setPort("/dev/ttyACM0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open(); //打开串口
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Serial Port !");
        return -1;
    }
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port is opened.");
    }
    else
    {
        return -1;
    }

    // 串口设置成功才创建并打开文件
    UBXdata.open(filename, std::ios::app|std::ios::binary|std::ios::out); //打开文件

    ros::Rate loop_rate(50); //定义循环发布的频率

    clock_t startTime, endTime;
    while (ros::ok())
    {
		//计时开始
        startTime = clock();

        //获取缓冲区内的字节数
        size_t n = ser.available();

        if (n!=0)
        {
            //ROS_INFO("__Reading from serial port__\n");

            //通过ROS串口对象读取串口信息，并打印到屏幕
            uint8_t buffer[1024] = {0};
            n = ser.read(buffer, n); //返回 read 的字节数

            for (int i=0; i<n; i++)  //ASCII 打印
            {
                //std::cout << buffer[i];
                UBXdata << buffer[i];  //写入到文件
            }
            //std::cout << "\n";
            //for (int i=0; i<n; i++)  //十六进制打印
            //{
            //    std::cout << std::hex << (buffer[i] & 0xff) << " ";
            //}
            //std::cout << "\n";

            //解析数据
            RMC resultRMC;
            decode_RMC(buffer, n, resultRMC);
            if (resultRMC.header == "$GNRMC" | resultRMC.header == "$GPRMC")
            {
                // std::cout<<"str = "<<resultRMC.str.c_str()<<std::endl;
                // std::cout<<"header = "<<resultRMC.header<<std::endl;
                // std::cout<<"gpsTime = "<<resultRMC.gpsTime<<std::endl;
                // std::cout<<"status = "<<resultRMC.status<<std::endl;
                // std::cout<<"lat = "<<resultRMC.lat<<std::endl;
                // std::cout<<"nsIndicator = "<<resultRMC.nsIndicator<<std::endl;
                // std::cout<<"lon = "<<resultRMC.lon<<std::endl;
                // std::cout<<"ewIndicator = "<<resultRMC.ewIndicator<<std::endl;
                // std::cout<<"spd = "<<resultRMC.spd<<std::endl;
                // std::cout<<"cog = "<<resultRMC.cog<<std::endl;
                // std::cout<<"date = "<<resultRMC.date<<std::endl;
                // std::cout<<"mv = "<<resultRMC.mv<<std::endl;
                // std::cout<<"mvEW = "<<resultRMC.mvEW<<std::endl;
                // std::cout<<"poseMode = "<<resultRMC.poseMode<<std::endl;
                // std::cout<<"navStatus = "<<resultRMC.navStatus<<std::endl;
                // std::cout<<"checksum = "<<resultRMC.checksum<<std::endl;
                // std::cout<<"CrLf = "<<resultRMC.CrLf<<std::endl;

                // UBXdata<<"RMC = "<<resultRMC.str<<std::endl;
                // UBXdata<<"header = "<<resultRMC.header<<std::endl;
                // UBXdata<<"gpsTime = "<<resultRMC.gpsTime<<std::endl;
                // UBXdata<<"status = "<<resultRMC.status<<std::endl;
                // UBXdata<<"lat = "<<resultRMC.lat<<std::endl;
                // UBXdata<<"nsIndicator = "<<resultRMC.nsIndicator<<std::endl;
                // UBXdata<<"lon = "<<resultRMC.lon<<std::endl;
                // UBXdata<<"ewIndicator = "<<resultRMC.ewIndicator<<std::endl;
                // UBXdata<<"spd = "<<resultRMC.spd<<std::endl;
                // UBXdata<<"cog = "<<resultRMC.cog<<std::endl;
                // UBXdata<<"date = "<<resultRMC.date<<std::endl;
                // UBXdata<<"mv = "<<resultRMC.mv<<std::endl;
                // UBXdata<<"mvEW = "<<resultRMC.mvEW<<std::endl;
                // UBXdata<<"poseMode = "<<resultRMC.poseMode<<std::endl;
                // UBXdata<<"navStatus = "<<resultRMC.navStatus<<std::endl;
                // UBXdata<<"checksum = "<<resultRMC.checksum<<std::endl;
                // UBXdata<<"CrLf = "<<resultRMC.CrLf<<std::endl;

                msg.header = resultRMC.header;
                msg.gpsTime = resultRMC.gpsTime;
                msg.status = resultRMC.status;
                msg.lat = resultRMC.lat;
                msg.nsIndicator = resultRMC.nsIndicator;
                msg.lon = resultRMC.lon;
                msg.ewIndicator = resultRMC.ewIndicator;
                msg.spd = resultRMC.spd;
                msg.cog = resultRMC.cog;
                msg.date = resultRMC.date;
                msg.mv = resultRMC.mv;
                msg.mvEW = resultRMC.mvEW;
                msg.poseMode = resultRMC.poseMode;
                msg.navStatus = resultRMC.navStatus;
                msg.checksum = resultRMC.checksum;
                msg.CrLf = resultRMC.CrLf;

                ROS_INFO("Header: %s", msg.header.c_str());
                ROS_INFO("gpsTime: %s", msg.gpsTime.c_str());
                ROS_INFO("status: %s", msg.status.c_str());
                ROS_INFO("lat: %s", msg.lat.c_str());
                ROS_INFO("nsIndicator: %s", msg.nsIndicator.c_str());
                ROS_INFO("lon: %s", msg.lon.c_str());
                ROS_INFO("ewIndicator: %s", msg.ewIndicator.c_str());
                ROS_INFO("spd: %s", msg.spd.c_str());
                ROS_INFO("cog: %s", msg.cog.c_str());
                ROS_INFO("date: %s", msg.date.c_str());
                ROS_INFO("mv: %s", msg.mv.c_str());
                ROS_INFO("mvEW: %s", msg.mvEW.c_str());
                ROS_INFO("poseMode: %s", msg.poseMode.c_str());
                ROS_INFO("navStatus: %s", msg.navStatus.c_str());
                ROS_INFO("checksum: %s", msg.checksum.c_str());
                //ROS_INFO("CrLf: %s", msg.CrLf.c_str());

                pub.publish(msg); //发布消息
            }
            memset(buffer, 0, 1024);
            
            endTime = clock();//计时结束
            ROS_INFO("The run time is: %f s\n", (double)(endTime-startTime)/CLOCKS_PER_SEC);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}


