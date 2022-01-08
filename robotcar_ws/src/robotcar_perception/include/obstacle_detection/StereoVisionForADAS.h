#pragma once 

#include "opencv2/opencv.hpp"
#include <iostream>


#include "StereoMatching.h"
#include "StixelEstimation.h"
#include "StixelSegmenation.h"
#include "DefStruct.h"

using namespace std;
using namespace cv;

/**
	@class CStereoVisionForADAS
	@brief stixel wrapping class
*/
class CStereoVisionForADAS{
private:
	//----------------input--------------------
	Mat m_imgLeftInput;		///< rectified image
	Mat m_imgRightInput;	///< rectified image

	//----------------param--------------------
	StereoCamParam_t m_objStereoParam; //构造参数结构体，存储视差数，窗口大小，基线长度和最远距离参数
	CStereoMatching m_objStereoMatching;//构造视差图
	CStixelEstimation m_objStixelEstimation; //创建棒状像素
	CStixelSegmentation m_objStixelSegmentation; //分割棒状像素

	MATCHING_ERROR match_err;
	STIXEL_ERROR stixel_err;
	SEG_ERROR seg_err;

	//LUT
	unsigned char m_pseudoColorLUT[256][3]; ///< RGB pseudo color

	//-------------member image----------------
	Mat m_imgColorDisp; ///< 8bit 3ch disparity image
	Mat m_imgStixelGray;
	Mat m_imgTopView;

	//---------------function------------------
	//Pseudo color Disparity　伪颜色视差图
	void MakePseudoColorLUT(); ///< pseudo color LUT
	void cvtPseudoColorImage(Mat& srcGray, Mat& dstColor); ///输入 :灰度图, 输出:彩色图
	
	void TopViewStixel(vector<stixel_t>& objStixelInROI);
	void TopViewLane(Mat& imgTopView, float fLaneInterval, int nCenterPointX);

public:
	//----------------output-------------------
	/// Matching output　匹配结果
	Mat m_matDisp16;
	Mat m_imgDisp8;

	/// Stixel output　
	vector<stixel_t> m_vecobjStixels; ///< stixel output
	vector<stixel_t> m_vecobjStixelInROI; ///< stixel output in 3D ROI
	Mat m_imgGround;

	/// segmentation output
	vector<Object_t> m_vecobjBB;

	//---------------function------------------
	CStereoVisionForADAS(StereoCamParam_t& objStereoParam);
	int Objectness(Mat& imgLeft, Mat& imgRight);
	int Objectness(Mat& imgLeft, Mat& imgRight, Mat& imgDisp8);

	// depth
	int RectToDisp(Rect& rectBox, Mat& matRect);
	int Disp16ToDepth(const uchar fDisparity, float& fDistanceMeter);

	/// display
	void Display();
	void Display(Mat& imgDisplay);
	void Display(Mat& imgDisplay, Mat& imgStixelResult);

	// display, draw
	void DrawLane(Mat& imgResult, StereoCamParam_t& objStereoParam);
	void DrawStixel(Mat& imgResult, vector<stixel_t>& vecobjStixels);
	void DrawGround(Mat& imgResult, Mat& imgGround);

	static StereoCamParam_t InitStereoParam(int nDatasetName);
	static int PitchDegToVanishingLine(StereoCamParam_t& objStereoParam);
	
};
