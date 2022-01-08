#include "obstacle_detection/StereoMatching.h"
// Ptr<StereoBM> bm = StereoBM::create(48, 9); ///< default construct

CStereoMatching::CStereoMatching(StereoCamParam_t& objStereoParam)
{
	SetParamOCVStereo(objStereoParam);
}
CStereoMatching::CStereoMatching(Mat& imgLeftInput, Mat& imgRightInput, StereoCamParam_t& objStereoParam)
{
	SetParamOCVStereo(objStereoParam);
	m_imgLeftInput = imgLeftInput;
	m_imgRightInput = imgRightInput;
}
void CStereoMatching::SetParamOCVStereo(StereoCamParam_t& objStereoParam)
{
	m_objStereoParam = objStereoParam;
	

#if CV_MAJOR_VERSION==3
/*	bm->create(m_objStereoParam.m_nNumberOfDisp, m_objStereoParam.m_nWindowSize);
	//                            视差数　　　　　　　　　　　　匹配块
	bm->setPreFilterCap(31);
	bm->setBlockSize(m_objStereoParam.m_nWindowSize > 0 ? m_objStereoParam.m_nWindowSize : 9);//SAD窗口大小，设为奇数
	bm->setMinDisparity(0);//确定匹配搜索从哪里开始  默认值是0
	bm->setNumDisparities(m_objStereoParam.m_nNumberOfDisp);//在该数值确定的视差范围内进行搜索,视差窗口，即最大视差值与最小视差值之差, 大小必须是16的整数倍
	bm->setTextureThreshold(10);		/// SAD window response threshold : default=12
	bm->setUniquenessRatio(5);			/// >(match_val - min_match)/min_match　使用匹配功能模式 
	bm->setSpeckleWindowSize(100);		//25;//9;检查视差连通区域变化度的窗口大小, 值为0时取消 speckle 检查  
	bm->setSpeckleRange(32);			//4;　视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零  
	//bm->setSmallerBlockSize(9);
	//bm->setDisp12MaxDiff(1); */

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
    int numberOfDisparities = (m_objStereoParam.m_nNumberOfDisp);//((imgSize.width / 8) + 15) & -16;
    sgbm->create(0, m_objStereoParam.m_nNumberOfDisp, m_objStereoParam.m_nWindowSize);
    sgbm->setPreFilterCap(63); //水平sobel预处理后，映射滤波器大小。默认为15
    int SADWindowSize = 13;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);  //越小视差图的噪声越大；越大视差图的误匹配增多，空洞增多
    int cn =1;// imgLeftInput.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
	int alg = STEREO_SGBM;
    if (alg == STEREO_HH)
        sgbm->setMode(cv::StereoSGBM::MODE_HH);
    else if (alg == STEREO_SGBM)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    else if (alg == STEREO_3WAY)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

#else
	bm.state->preFilterCap = 31;
	bm.state->SADWindowSize = m_objStereoParam.m_nWindowSize > 0 ? m_objStereoParam.m_nWindowSize : 9;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = m_objStereoParam.m_nNumberOfDisp;
	bm.state->textureThreshold = 10;		//SAD window response threshold : default=12
	bm.state->uniquenessRatio = 15;		// > (match_val - min_match)/min_match
	bm.state->speckleWindowSize = 100;//25;//9;
	bm.state->speckleRange = 32;//4;
	//bm.state->disp12MaxDiff = 1;
#endif

}

//判断输入图像的格式是否合适
MATCHING_ERROR CStereoMatching::SetImage(Mat& imgLeft, Mat& imgRight){
	if (imgLeft.size() != imgRight.size()) return IMGSCALE_ERR;
	if (imgLeft.size() != m_objStereoParam.objCamParam.m_sizeSrc)
	{
		cout << imgLeft.size() << " " << m_objStereoParam.objCamParam.m_sizeSrc << endl;
		return IMGSCALE_ERR;
	}
	if (imgLeft.channels() == 3){
		cvtColor(imgLeft, m_imgLeftInput, CV_BGR2GRAY);
		cvtColor(imgRight, m_imgRightInput, CV_BGR2GRAY);
		return NO_PROB;
	}
	m_imgLeftInput = imgLeft;
	m_imgRightInput = imgRight;
	
	return NO_PROB;
}

MATCHING_ERROR CStereoMatching::MakeDisparity()
{
	MakeDisparity(m_imgLeftInput, m_imgRightInput, m_matDisp16, m_matDisp16_r);
	m_matDisp16.convertTo(m_imgDisp8, CV_8U, 255.0 / (m_objStereoParam.m_nNumberOfDisp*16.));
	// m_matDisp16_r.convertTo(m_imgDisp8_r, CV_8U,  255 / (m_objStereoParam.m_nNumberOfDisp*16.));
	// double minVal, maxVal;
	// minMaxLoc(m_imgDisp8_r, &minVal, &maxVal);
	// cout<<"maxVal:"<<maxVal<<endl;;
	
	// imshow("disp", m_imgDisp8);
	// cout<<"m_imgDisp8:"<<m_imgDisp8<<endl;
	
	// imshow("disp_r", m_imgDisp8_r);

	return NO_PROB;
}
MATCHING_ERROR CStereoMatching::MakeDisparity(Mat& imgLeft, Mat& imgRight, bool flgUseWLSFilter)//main
{
#if CV_MAJOR_VERSION==2
	flgUseWLSFilter = false;
#endif
	MATCHING_ERROR Error;
	Error = SetImage(imgLeft, imgRight); 
	//cout << Error << endl;
	if (flgUseWLSFilter == false){
		MakeDisparity();
		ImproveDisparity_Naive(m_imgDisp8);
		//ImproveDisparity_eightdirections(m_imgDisp8);
			
	}
}

//计算左右影像差异
MATCHING_ERROR CStereoMatching::MakeDisparity(Mat& imgLeft, Mat& imgRight, Mat& matDisp16, Mat& matDisp16_r){
	//Mat matDisp16_r;
#if CV_MAJOR_VERSION==3
	// cout << "SADWindowSize : " << bm->getBlockSize() << endl;
	// cout << "NumOfDisparity : " << bm->getNumDisparities() << endl;
	//bm->compute(imgLeft, imgRight, matDisp16);//计算差异
	// cout << "complete" << endl;
    sgbm->compute(imgLeft, imgRight, matDisp16);
	//imshow("matDisp16:",matDisp16);
	// sgbm->setMinDisparity(-64);
	// sgbm->compute( imgRight, imgLeft, matDisp16_r);
	// sgbm->setMinDisparity(0);	
	// matDisp16_r = abs(matDisp16_r);
	//cout<<matDisp16_r<<endl;
#else
	sgbm(imgLeft, imgRight, matDisp16, CV_16S);
	// sgbm(imgRight,imgLeft, matDisp16_r, CV_16S);
	// matDisp16_r = abs(matDisp16_r);
#endif
	m_matDisp16 = matDisp16;
	return NO_PROB;
}

//初步处理视差图
MATCHING_ERROR CStereoMatching::ImproveDisparity_Naive(Mat& imgDisp8) //用周围的视差去填充空洞
{
	uchar chTempCur = 0;
	uchar chTempPrev = 0;
	
	int cnt = 1;
	for (int v = 0; v < imgDisp8.rows; v++){
		for (int u = m_objStereoParam.m_nNumberOfDisp; u < imgDisp8.cols; u++){
			chTempCur = imgDisp8.at<uchar>(v, u);//v行u列的这个像素
			//shTempCur = m_matDisp16.at<short>(v, u);
			if (chTempCur == 0) {
				imgDisp8.at<uchar>(v, u) = chTempPrev;
				//m_matDisp16.at<short>(v, u) = shTempPrev;
			}
			else {
				chTempPrev = chTempCur;
				//shTempPrev = shTempCur;
			}
		}
	}
	Size size(7, 7);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);//getStructuringElement函数会返回指定形状和尺寸的结构元素
	//MORPH_RECT是矩形

	morphologyEx(imgDisp8, imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 2);//morphologyEx这个函数可以方便的对图像进行一系列的膨胀腐蚀组合
	//(源图像；目标图像；先腐蚀后膨胀；用于膨胀操作的结构元素，如果取值为Mat(),那么默认使用一个3 x 3 的方形结构元素；锚点的位置；迭代使用 dilate() 的次数)
	return NO_PROB;
}

//以像素为中心，等角度往外发射8条射线，收集每条射线碰到的第一个有效像素，取平均值进行空洞填充
MATCHING_ERROR CStereoMatching::ImproveDisparity_eightdirections(Mat& imgDisp8) 
{
	float pi = 3.1415926;
	float angle1[8] = { pi, 3 * pi / 4, pi / 2, pi / 4, 0, 7 * pi / 4, 3 * pi / 2, 5 * pi / 4 };
	//float angle1[4] = { pi, pi / 2, 0,  3 * pi / 2};
	//float angle2[8] = { pi, 5 * pi / 4, 3 * pi / 2, 7 * pi / 4, 0, pi / 4, pi / 2, 3 * pi / 4 };
	float *angle = angle1;
	std::vector<std::pair<int, int>> inv_pixels; //定义无效像素点集 即空洞
	std::vector<std::pair<int, int>> inv_pixels2;
	std::vector<float> disp_collects;
	uchar chTempCur = 0;
	//uchar chTempPrev = 0;
	for (int v = 0; v < imgDisp8.rows; v++){
		for (int u = m_objStereoParam.m_nNumberOfDisp; u < imgDisp8.cols; u++){
			chTempCur = imgDisp8.at<uchar>(v, u);//v行u列的这个像素
			//shTempCur = m_matDisp16.at<short>(v, u);
			if (chTempCur == 0) 
			{
				//imgDisp8.at<uchar>(v, u) = chTempPrev;
				inv_pixels.emplace_back(v, u);
			}
		}
	}
	for(int i = 0;i<inv_pixels.size();i++)
	{
		if(imgDisp8.at<uchar>(inv_pixels[i].first,inv_pixels[i].second) != 0)
			continue;
		for (int j = 0; j < 8; j++) 
		{
				const float ang = angle[j];
				const float sina = sin(ang);
				const float cosa = cos(ang);
				for (int n = 1; ; n++) 
				{
					const int yy = inv_pixels[i].first + n * sina;  //行
					const int xx = inv_pixels[i].second + n * cosa; //列
					if (yy<0 || yy >= imgDisp8.rows || xx<0 || xx >= imgDisp8.cols) 
					{
						break;
					}
					float m_disp;
					m_disp = imgDisp8.at<uchar>(yy,xx);
					if(m_disp == 0)
					{
						inv_pixels2.emplace_back(yy,xx);
					}

					//if(m_disp != 0)
					else
					{
						disp_collects.emplace_back(m_disp);
						break;
					}
				}
		}
		for (int s = 0 ; s < inv_pixels2.size(); s++ )
		{
			imgDisp8.at<uchar>(inv_pixels2[s].first,inv_pixels2[s].second) = disp_collects[disp_collects.size()/2];
		}
		// 取中值
		imgDisp8.at<uchar>(inv_pixels[i].first,inv_pixels[i].second) = disp_collects[disp_collects.size()/2];
		//cout<<"空洞填充值"<<disp_collects[disp_collects.size()/2]<<endl;
		disp_collects.clear();
		inv_pixels2.clear();

	}

	Size size(7, 7);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);//getStructuringElement函数会返回指定形状和尺寸的结构元素
	//MORPH_RECT是矩形

	morphologyEx(imgDisp8, imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 2);//morphologyEx这个函数可以方便的对图像进行一系列的膨胀腐蚀组合
	//(源图像；目标图像；先腐蚀后膨胀；用于膨胀操作的结构元素，如果取值为Mat(),那么默认使用一个3 x 3 的方形结构元素；锚点的位置；迭代使用 dilate() 的次数)
	return NO_PROB;
}



//对视差图进行后处理，滤波  //没用
MATCHING_ERROR CStereoMatching::ImproveDisparity_WLSFilter(Mat& imgDisp8)
{
 #if CV_MAJOR_VERSION==3
	Mat matDispLeft16;
	Mat matDispRight16;
	/*Mat conf_map = Mat(m_imgLeftInput.rows, m_imgLeftInput.cols, CV_8U);
	conf_map = Scalar(255);*/

	wls_filter = createDisparityWLSFilter(sgbm);//该方法创建DisparityWLSFilter的实例，并根据匹配器实例自动设置所有相关的过滤器参数。
	Ptr<StereoMatcher> right_sgbm = createRightMatcher(sgbm);//设置匹配器以计算右视图视差图的便利方法
	
	wls_filter->setLambda(8000.);
	wls_filter->setSigmaColor(1.5);

	sgbm->compute(m_imgLeftInput, m_imgRightInput, matDispLeft16);
	right_sgbm->compute(m_imgRightInput, m_imgLeftInput, matDispRight16);
	wls_filter->filter(matDispLeft16, m_imgLeftInput, m_matDisp16, matDispRight16);
	/*conf_map = wls_filter->getConfidenceMap();

	m_rectFilterROI = wls_filter->getROI();*/

	m_matDisp16.convertTo(imgDisp8, CV_8U, 255 / (m_objStereoParam.m_nNumberOfDisp*16.));
 #endif
	return NO_PROB;
}



// void CStereoMatching::LRCheck()
// {
//     const int width = m_imgDisp8.cols;
//     const int height = m_imgDisp8.rows;
// 	int Invalid_Float = std::numeric_limits<float>::infinity();

//     const int& threshold = 1;

// 	// 遮挡区像素和误匹配区像素
// 	auto& occlusions = occlusions_;
// 	auto& mismatches = mismatches_;
// 	occlusions.clear();
// 	mismatches.clear();

//     // ---左右一致性检查
//     for (int i = 0; i < height; i++) {
//         for (int j = 0; j < width; j++) {
//             // 左影像视差值
//         	//auto& disp = disp_left_[i * width + j];
// 			auto& disp = m_imgDisp8.at<uchar>(i,j);
// 			if(disp == Invalid_Float)
// 			{
// 				mismatches.emplace_back(i, j);
// 				continue;
// 			}

//             // 根据视差值找到右影像上对应的同名像素
//         	//const auto col_right = static_cast<sint32>(j - disp + 0.5);
// 			const auto col_right = int (j - disp + 0.5);
            
//         	if(col_right >= 0 && col_right < width) {
//                 // 右影像上同名像素的视差值
//                 //const auto& disp_r = disp_right_[i * width + col_right];
// 				const auto& disp_r = m_imgDisp8_r.at<uchar>(i,col_right);
                
//         		// 判断两个视差值是否一致（差值在阈值内）
//         		if (abs(disp - disp_r) > threshold) {
// 					// 区分遮挡区和误匹配区
// 					// 通过右影像视差算出在左影像的匹配像素，并获取视差disp_rl
// 					// if(disp_rl > disp) 
//         			//		pixel in occlusions
// 					// else 
//         			//		pixel in mismatches
// 					//const int col_rl = static_cast<sint32>(col_right + disp_r + 0.5);
// 					const int col_rl = int(col_right + disp_r + 0.5);
// 					if(col_rl > 0 && col_rl < width){
// 						//const auto& disp_l = disp_left_[i*width + col_rl];
// 						const auto& disp_l = m_imgDisp8.at<uchar>(i,col_rl);
// 						if(disp_l > disp) {
// 							occlusions.emplace_back(i, j);
// 						}
// 						else {
// 							mismatches.emplace_back(i, j);
// 						}
// 					}
// 					else{
// 						mismatches.emplace_back(i, j);
// 					}

//                     // 让视差值无效
// 					disp = Invalid_Float;
//                 }
//             }
//             else{
//                 // 通过视差值在右影像上找不到同名像素（超出影像范围）
//                 disp = Invalid_Float;
// 				mismatches.emplace_back(i, j);
//             }
//         }
//     }
// 	//cout<<mismatches.size()<<endl;
// 	//cout<<occlusions.size()<<endl;

// }

// void CStereoMatching::FillHolesInDispMap()
// {
// 	const int width = m_imgDisp8.cols;
//     const int height = m_imgDisp8.rows;
// 	int Invalid_Float = std::numeric_limits<float>::infinity();

// 	std::vector<float> disp_collects;

// 	// 定义8个方向
// 	const float pi = 3.1415926f;
// 	float angle1[8] = { pi, 3 * pi / 4, pi / 2, pi / 4, 0, 7 * pi / 4, 3 * pi / 2, 5 * pi / 4 };
// 	float angle2[8] = { pi, 5 * pi / 4, 3 * pi / 2, 7 * pi / 4, 0, pi / 4, pi / 2, 3 * pi / 4 };
// 	float *angle = angle1;
//     // 最大搜索行程，没有必要搜索过远的像素
//     const int max_search_length = 1.0*std::max(abs(m_objStereoParam.m_nNumberOfDisp), abs(0));

// 	//float* disp_ptr = disp_left_;
// 	for (int k = 0; k < 3; k++) {
// 		// 第一次循环处理遮挡区，第二次循环处理误匹配区
// 		auto& trg_pixels = (k == 0) ? occlusions_ : mismatches_;
//         if (trg_pixels.empty()) {
//             continue;
//         }
// 		std::vector<float> fill_disps(trg_pixels.size());
// 		std::vector<std::pair<int, int>> inv_pixels;
// 		if (k == 2) {
// 			//  第三次循环处理前两次没有处理干净的像素
// 			for (int i = 0; i < height; i++) {
// 				for (int j = 0; j < width; j++) {
// 					if (m_imgDisp8.at<uchar>(i,j) == Invalid_Float) {
// 						inv_pixels.emplace_back(i, j);
// 					}
// 				}
// 			}
// 			trg_pixels = inv_pixels;
// 		}

// 		// 遍历待处理像素
//         for (auto n = 0u; n < trg_pixels.size(); n++) {
//             auto& pix = trg_pixels[n];
//             const int y = pix.first;
//             const int x = pix.second;

// 			if (y == height / 2) {
// 				angle = angle2; 
// 			}

// 			// 收集8个方向上遇到的首个有效视差值
// 			disp_collects.clear();
// 			for (int s = 0; s < 8; s++) {
// 				const float ang = angle[s];
// 				const float sina = float(sin(ang));
// 				const float cosa = float(cos(ang));
// 				for (int m = 1; m < max_search_length; m++) {
// 					const int yy = lround(y + m * sina);
// 					const int xx = lround(x + m * cosa);
// 					if (yy<0 || yy >= height || xx<0 || xx >= width) {
// 						break;
// 					}
// 					//const auto& disp = *(disp_ptr + yy*width + xx);
// 					const auto& disp = m_imgDisp8.at<uchar>(yy,xx);
// 					if (disp != Invalid_Float) {
// 						disp_collects.push_back(disp);
// 						break;
// 					}
// 				}
// 			}
// 			if(disp_collects.empty()) {
// 				continue;
// 			}

// 			std::sort(disp_collects.begin(), disp_collects.end());

// 			// 如果是遮挡区，则选择第二小的视差值
// 			// 如果是误匹配区，则选择中值
// 			if (k == 0) {
// 				if (disp_collects.size() > 1) {
//                     fill_disps[n] = disp_collects[1];
// 				}
// 				else {
//                     fill_disps[n] = disp_collects[0];
// 				}
// 			}
// 			else{
//                 fill_disps[n] = disp_collects[disp_collects.size() / 2];
// 			}
// 		}
//         for (auto n = 0u; n < trg_pixels.size(); n++) {
//             auto& pix = trg_pixels[n];
//             const int y = pix.first;
//             const int x = pix.second;
//             //disp_ptr[y * width + x] = fill_disps[n];
// 			m_imgDisp8.at<uchar>(y,x) = fill_disps[n];
//         }
// 	}
// }
