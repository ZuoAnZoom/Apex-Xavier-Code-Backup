#pragma once

// #include <opencv2\opencv.hpp>
#include <iostream>
#include <map> 

#include "DefStruct.h"
#include "StixelEstimation.h"

using namespace std;
using namespace cv;

typedef enum Seg_error { GOOD, SIZ_ERR } SEG_ERROR;//typedef enum自定义一个枚举类型
//其声明的变量SEG_ERROR只能设置为枚举中的值

class CStixelSegmentation{
private:
	//----------------input--------------------
	vector<stixel_t> m_vecobjStixels;

	//----------------param--------------------
	StereoCamParam_t m_objStereoParam;

	//-------------member image----------------
	Mat m_imgDebug;
	
	//---------------function------------------
	//============processing part==============

	//old version
	//SEG_ERROR StixelZClustering(vector<stixel_t>& objStixels, vector<boundingbox_t>& objBBcandidate); ///< Z ���� Distance ��� �з�, Stixel -> BB candidate
	//SEG_ERROR StixelXClustering(vector<boundingbox_t>& objBBinput, vector<boundingbox_t>& objBBOutput); ///< X ���� Distance ��� �з�, BB candidate -> BB result
	//SEG_ERROR StixelBBboxOptimization(vector<boundingbox_t>& objBBinput, vector<boundingbox_t>& objBBOutput); ///< bounding box ����ȭ

	SEG_ERROR StixelZClustering(vector<stixel_t>& objStixels, vector<Object_t>& objBBcandidate); ///< Z ���� Distance ��� �з�, Stixel -> BB candidate
	SEG_ERROR StixelXClustering(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput); ///< X ���� Distance ��� �з�, BB candidate -> BB result
	SEG_ERROR StixelBBboxOptimization(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput); ///< bounding box ����ȭ

public:
	//----------------output-------------------
	vector<Object_t> m_vecobjBB;

	//---------------function------------------
	CStixelSegmentation(StereoCamParam_t objStereoParam);
	void SetDebugImg(Mat imgTemp);//调试图

	//wrapping function
	SEG_ERROR SegmentStixel(vector<stixel_t>& objStixels); ///< Stixel �з��۾�
	
	
};
