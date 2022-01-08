/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warranty, support or any guarantee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OPENCV WEBSITES:
     Homepage:      http://opencv.org
     Online docs:   http://docs.opencv.org
     Q&A forum:     http://answers.opencv.org
     GitHub:        https://github.com/opencv/opencv/
   ************************************************** */

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

static int print_help()
{
    std::cout << " Given a list of chessboard images, the number of corners (nx, ny)\n"
                 " on the chessboards, and a flag: useCalibrated for \n"
                 "   calibrated (0) or\n"
                 "   uncalibrated \n"
                 "     (1: use stereoCalibrate(), 2: compute fundamental\n"
                 "         matrix separately) stereo. \n"
                 " Calibrate the cameras and display the\n"
                 " rectified results along with the computed disparity images.   \n"
              << endl;
    std::cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> -s=<square_size default=1.0> <image list XML/YML file default=stereo_calib.xml>\n"
              << endl;
    return 0;
}

static void
StereoCalib(const vector<string> &imagelist, Size boardSize, float squareSize, bool displayCorners = false, bool useCalibrated = true, bool showRectified = true)
{
    if (imagelist.size() % 2 != 0)
    {
        std::cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    // 遍历所有图像对
    cout << "开始遍历图像对" << endl;
    Size imageSize;
    const int maxScale = 2;
    int i, j, k, nimages = (int)imagelist.size() / 2;
    vector<vector<Point2f>> imagePoints[2];
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<vector<Point3f>> objectPoints;
    vector<string> goodImageList;
    for (i = j = 0; i < nimages; i++)
    {
        // 遍历左右目
        vector<float> pix_vector;
        for (k = 0; k < 2; k++)
        {
            const string &filename = imagelist[i * 2 + k];
            Mat img = imread(filename, 0);
            if (img.empty())
            {
                std::cout << filename << "没有图像，跳过" << endl;
                break;
            }
            if (imageSize == Size())
                imageSize = img.size();
            else if (img.size() != imageSize)
            {
                std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }

            // 如果第一次没有找到角点，则放大图像再找一遍
            bool found = false;
            vector<Point2f> &corners = imagePoints[k][j];
            for (int scale = 1; scale <= maxScale; scale++)
            {
                Mat timg;
                if (scale == 1)
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale, INTER_LINEAR_EXACT);
                found = findChessboardCorners(timg, boardSize, corners,
                                              CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                if (found)
                {
                    if (scale > 1)
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1. / scale;
                    }
                    break;
                }
            }

            // 显示角点
            if (displayCorners)
            {
                std::cout << filename << endl;
                Mat cimg;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                cv::imshow("corners", cimg);
                cv::waitKey(0);
            }
            else
                putchar('.');
            if (!found)
                break;

            // 角点坐标精化
            cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
                                      30, 0.01));

            // 计算角点排列方向
            cv::Point2f pt0 = imagePoints[k][j][0];
            cv::Point2f ptw = imagePoints[k][j][boardSize.width];
            pix_vector.push_back(pt0.x * ptw.y - ptw.x * pt0.y);
        }
        if (k == 2 && i % 10 == 0)
        {
            // 判断角点排列是否镜像
            if (pix_vector[0] * pix_vector[1] < 0)
            {
                cout << "角点排列镜像，跳过" << endl;
                continue;
            }

            goodImageList.push_back(imagelist[i * 2]);
            goodImageList.push_back(imagelist[i * 2 + 1]);
            j++;
        }
    }
    std::cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if (nimages < 2)
    {
        std::cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    // 计算角点世界坐标
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
    for (i = 0; i < nimages; i++)
    {
        for (j = 0; j < boardSize.height; j++)
            for (k = 0; k < boardSize.width; k++)
                objectPoints[i].push_back(Point3f(k * squareSize, j * squareSize, 0));
    }

    std::cout << "Running stereo calibration ...\n";

    // 初始化相机内参
    std::cout << "initCameraMatrix2D" << endl;
    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);

    // 双目外参标定
    std::cout << "stereoCalibrate" << endl;
    Mat R, T, E, F;
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 CALIB_FIX_ASPECT_RATIO +
                                     CALIB_ZERO_TANGENT_DIST +
                                     CALIB_USE_INTRINSIC_GUESS +
                                     CALIB_SAME_FOCAL_LENGTH +
                                     CALIB_RATIONAL_MODEL +
                                     CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

    std::cout << "done with RMS error=" << rms << endl;

    // 标定质量检验（极线约束）
    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for (i = 0; i < nimages; i++)
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for (k = 0; k < 2; k++)
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
        }
        for (j = 0; j < npt; j++)
        {
            double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] +
                                imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x * lines[0][j][0] +
                                imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    std::cout << "average epipolar err = " << err / npoints << endl;

    // 保存内参
    FileStorage fs("intrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        std::cout << "Error: can not save the intrinsic parameters\n";

    // 双目校正
    std::cout << "stereoRectify" << endl;
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, imageSize, &validRoi[0], &validRoi[1]);

    // 保存外参
    fs.open("extrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        std::cout << "Error: can not save the extrinsic parameters\n";

    // COMPUTE AND DISPLAY RECTIFICATION
    if (!showRectified)
        return;

    Mat rmap[2][2];
    // IF BY CALIBRATED (BOUGUET'S METHOD)
    if (useCalibrated)
    {
        // we already computed everything
    }
    // OR ELSE HARTLEY'S METHOD
    else
    // use intrinsic parameters of each camera, but
    // compute the rectification transformation directly
    // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for (k = 0; k < 2; k++)
        {
            for (i = 0; i < nimages; i++)
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv() * H1 * cameraMatrix[0];
        R2 = cameraMatrix[1].inv() * H2 * cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    Mat canvas;
    double sf;
    int w, h;
    for (i = 0; i < nimages; i++)
    {
        Mat img1 = imread(goodImageList[i * 2 + 0], 0);
        Mat img2 = imread(goodImageList[i * 2 + 1], 0);
        Mat rimg1, rimg2, rimg;
        // 重映射
        remap(img1, rimg1, rmap[0][0], rmap[0][1], INTER_LINEAR);
        remap(img2, rimg2, rmap[1][0], rmap[1][1], INTER_LINEAR);
        cv::hconcat(rimg1, rimg2, rimg);
        cvtColor(rimg, canvas, COLOR_GRAY2BGR);
        // Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
        // resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
        // if (useCalibrated)
        // {
        //     Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
        //               cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
        //     rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
        // }

        if (!isVerticalStereo)
            for (j = 0; j < canvas.rows; j += 16)
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for (j = 0; j < canvas.cols; j += 16)
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        cv::waitKey(0);
    }
}

static bool readStringList(const string &filename, vector<string> &l)
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
        l.push_back((string)*it);
    return true;
}

int main(int argc, char **argv)
{
    Size boardSize;
    string imagelistfn;
    bool showRectified;
    cv::CommandLineParser parser(argc, argv, "{w|8|}{h|6|}{s|0.072|}{nr||}{help||}{@input|test/calib.xml|}");
    if (parser.has("help"))
        return print_help();
    showRectified = !parser.has("nr");
    imagelistfn = samples::findFile(parser.get<string>("@input"));
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    float squareSize = parser.get<float>("s");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    vector<string> imagelist;
    bool ok = readStringList(imagelistfn, imagelist);
    if (!ok || imagelist.empty())
    {
        std::cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
        return print_help();
    }

    // 双目标定
    StereoCalib(imagelist, boardSize, squareSize, false, false, showRectified);
    return 0;
}