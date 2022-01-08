#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#define PI 3.141592

#ifndef WIN32
#define sprintf_s snprintf
#endif

using namespace cv;

enum { Daimler, KITTI, HICAM, CityScape ,benben,benben2,benben3};
//enum { Ped, Car, TrafficSign, TrafficLight, Else};
enum { TP, FN, FP, TP_S, FN_S}; //p代表正类，N代表负类 T代表true F代表flase
enum { STEREO_BM = 0 }; //, STEREO_SGBM = 1
enum { GRAY, COLOR };
//构造棒状参数
struct stixel_t
{
	int nGround;		///< Ground bound point　接地点 
	int nHeight;		///< Upper bound point　上限点
	uchar chDisparity;	///< Disparity(normalized 255)　
	int nCol;			///< column　列
	double dZ;			///< distance(meter)
	double dY;			///< meter
	double dX;			///< meter
	stixel_t(){
		nGround = -1;
		nHeight = -1;
		chDisparity = 0;
		dZ = 0.;
		dY = 0.;
		dX = 0.;
	}
};

struct Object_t
{
	Rect rectBB;
	double dZ; // 最小z
	double dX; //平均x
	double dY; // 最大y
	double zmaxDist; //最大z
	double xminDist; // 最小x
	double xmaxDist; // 最大x
	std::vector<stixel_t> vecobjStixels;
	int nClass;
	double dClassScore;
	double dCollisionRisk;
	Object_t(){
		rectBB = Rect(0, 0, 0, 0);
		dZ = 0.;
		vecobjStixels.clear();
		nClass = TP;
		dClassScore = 0.;
		dCollisionRisk = 1.;
	}
	Object_t(Rect rect, double dz){
		nClass = TP;
		dClassScore = 0.;
		dCollisionRisk = 1.;
		rectBB = rect; dZ = dz;
		vecobjStixels.clear();
	}
	Object_t(Rect rect, stixel_t objStixel){
		nClass = TP;
		dClassScore = 0.;
		dCollisionRisk = 1.;
		rectBB = rect; dZ = objStixel.dZ;
		vecobjStixels.push_back(objStixel);//该函数将一个新的元素加到vector的最后面
	}
};

//构造相机参数
struct CameraParam_t
{
	double m_dPitchDeg;	///< unit : degree　仰角　单位：度
	double m_dYawDeg; ///< unit : degree　偏角　单位：度
	double m_dFocalLength; ///< unit : pixels　焦距　单位：像素
	double m_dOx; ///< unit : pixels　
	double m_dOy; ///< unit : pixels
	double m_dCameraHeight; ///< cam height from ground　相机距离地面高度
	double m_dFOVvDeg; ///< vertical FOV　垂直视场
	double m_dFOVhDeg; ///< horizontal FOV　水平视场
	int m_nVanishingY; ///< vanishing line location　消失线位置
	cv::Size m_sizeSrc;
	CameraParam_t(){
		m_dPitchDeg = 0.;
		m_dYawDeg = 0.;
		m_dFocalLength = 0.;
		m_dOx = 0.;
		m_dOy = 0.;
		m_dCameraHeight = 0.;
		m_dFOVvDeg = 0.;
		m_dFOVhDeg = 0.;
		m_sizeSrc = cv::Size(0, 0);
	}
};

//构造双目参数
struct StereoCamParam_t
{
	int m_nNumberOfDisp; ///< number of disparity.　视差数
	int m_nWindowSize; ///< window size. It must be odd number.奇数
	double m_dBaseLine; ///< baseline, unit : meters1
	double m_dMaxDist; ///< Maximum distance value,　单位是米
	CameraParam_t objCamParam;
	StereoCamParam_t(){
		m_nNumberOfDisp = 80;
		m_nWindowSize = 9;
		m_dBaseLine = 0.;
		m_dMaxDist = 50.0;
	}
};

