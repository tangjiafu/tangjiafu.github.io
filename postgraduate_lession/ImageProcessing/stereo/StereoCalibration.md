#  更多内容请前往Github搜索tangjiafu或者ImageProcessing_Tang   #
####特别感谢OPENCV提供的源码####      
      #include "opencv2/calib3d/calib3d.hpp"
      #include "opencv2/imgcodecs.hpp"
      #include "opencv2/highgui/highgui.hpp"
      #include "opencv2/imgproc/imgproc.hpp"	//open3.1库的头文件   
      
      #include <vector>	 //向量头文件	
      #include <string>	 //字符串头文件
      #include <algorithm>   //C++标准模版库STL头文件，提供了大量基于迭代器的非成员模版函数
      #include <iostream>	   //输入输出流头文件
      #include <iterator>	   //迭代器
      #include <stdio.h>		//输入输出函数头文件
      #include <stdlib.h>		//C++函数库
      #include <ctype.h>		 //字符转换
      
      using namespace cv;	   //去除opencv中的CV
      using namespace std;   //标准命名空间
      
      static int print_help()   //帮助函数
      {
      	cout <<
      		" Given a list of chessboard images, the number of corners (nx, ny)\n"
      		" on the chessboards, and a flag: useCalibrated for \n"
      		"   calibrated (0) or\n"
      		"   uncalibrated \n"
      		" (1: use cvStereoCalibrate(), 2: compute fundamental\n"
      		" matrix separately) stereo. \n"
      		" Calibrate the cameras and display the\n"
      		" rectified results along with the computed disparity images.   \n" << endl;
      	cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> <image list XML/YML file default=../data/stereo_calib.xml>\n" << endl;
      	return 0;
      }
      
      //立体标定
      static void StereoCalib(const vector<string>& imagelist, Size boardSize, bool displayCorners = false, bool useCalibrated = true, bool showRectified = true)
      {
      	if (imagelist.size() % 2 != 0)//该if语句判断imagelist字符串数组是否是2的倍数，用以检测是否为2组图片
      	{
      		cout << "Error: the image list contains odd (non-even) number of elements\n";
      		return;
      	}
      
      	const int maxScale = 2;
      	const float squareSize = 1.f;  // Set this to your actual square size，设置小方格的大小，一般为正方形方格
      	// ARRAY AND VECTOR STORAGE:
      
      	vector<vector<Point2f> > imagePoints[2];
      	vector<vector<Point3f> > objectPoints;
      	Size imageSize;
      
      	int i, j, k, nimages = (int)imagelist.size() / 2;//imagelist字符串数组大小的一半，即一组图片有多少张
      
      	imagePoints[0].resize(nimages);//重新定义两组图片地址存储数组
      	imagePoints[1].resize(nimages);
      	vector<string> goodImageList;
      
      	for (i = j = 0; i < nimages; i++)
      	{
      		for (k = 0; k < 2; k++)
      		{
      			const string& filename = imagelist[i * 2 + k];
      			Mat img = imread(filename, 0);//读取图片，0表示以灰度图片存储
      			if (img.empty())//保证不是图片存在不是空的
      				break;
      			if (imageSize == Size())//保证每张图片大小一定且相等
      				imageSize = img.size();
      			else if (img.size() != imageSize)
      			{
      				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
      				break;
      			}
      			bool found = false;
      			vector<Point2f>& corners = imagePoints[k][j];
      			for (int scale = 1; scale <= maxScale; scale++)
      			{
      				Mat timg;
      				if (scale == 1)
      					timg = img;
      				else
      					resize(img, timg, Size(), scale, scale);
      				found = findChessboardCorners(timg, boardSize, corners,
      					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);//寻找角点
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
      			if (displayCorners)
      			{
      				cout << filename << endl;
      				Mat cimg, cimg1;
      				cvtColor(img, cimg, COLOR_GRAY2BGR);
      				drawChessboardCorners(cimg, boardSize, corners, found);//画出角点
      				double sf = 640. / MAX(img.rows, img.cols);
      				resize(cimg, cimg1, Size(), sf, sf);
      				imshow("corners", cimg1);
      				char c = (char)waitKey(500);
      				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
      					exit(-1);
      			}
      			else
      				putchar('.');
      			if (!found)
      				break;
      			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
      				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
      				30, 0.01));
      		}
      		if (k == 2)
      		{
      			goodImageList.push_back(imagelist[i * 2]);
      			goodImageList.push_back(imagelist[i * 2 + 1]);
      			j++;
      		}
      	}
      	cout << j << " pairs have been successfully detected.\n";
      	nimages = j;
      	if (nimages < 2)
      	{
      		cout << "Error: too little pairs to run the calibration\n";
      		return;
      	}
      
      	imagePoints[0].resize(nimages);
      	imagePoints[1].resize(nimages);
      	objectPoints.resize(nimages);
      
      	for (i = 0; i < nimages; i++)
      	{
      		for (j = 0; j < boardSize.height; j++)
      		for (k = 0; k < boardSize.width; k++)
      			objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));//角点三维空间坐标
      	}
      
      	cout << "Running stereo calibration ...\n";
      
      	Mat cameraMatrix[2], distCoeffs[2];
      	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);//内参数矩阵
      	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
      	Mat R, T, E, F; //R，T矩阵，本质矩阵，基础矩阵
      
      	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],//立体标定程序
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
      	cout << "done with RMS error=" << rms << endl;//输出RMS误差
      
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
      			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);//矫正没涨图片
      			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);////计算对应点的外极线epilines是一个三元组(a,b,c)，表示点在另一视图中对应的外极线ax+by+c=0;  
      		}
      		for (j = 0; j < npt; j++)//画线部分
      		{
      			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
      				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
      				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
      				imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
      			err += errij;
      		}
      		npoints += npt;
      	}
      	cout << "average epipolar err = " << err / npoints << endl;//平均误差
      
      	// save intrinsic parameters
      	FileStorage fs("intrinsics.xml", FileStorage::WRITE);//将标定结果输入XML文件
      	if (fs.isOpened())
      	{
      		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
      			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];//写入内部参数矩阵
      		fs.release();
      	}
      	else
      		cout << "Error: can not save the intrinsic parameters\n";
      
      	Mat R1, R2, P1, P2, Q;
      	Rect validRoi[2];
      
      	stereoRectify(cameraMatrix[0], distCoeffs[0],//校正第一组标定图片用于可视化查看标定结果
      		cameraMatrix[1], distCoeffs[1],
      		imageSize, R, T, R1, R2, P1, P2, Q,
      		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
      
      	fs.open("extrinsics.xml", FileStorage::WRITE);
      	if (fs.isOpened())
      	{
      		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;//写入外部参数
      		fs.release();
      	}
      	else
      		cout << "Error: can not save the extrinsic parameters\n";
      
      	// OpenCV can handle left-right
      	// or up-down camera arrangements
      	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
      
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
      		Mat H1, H2;//计算单应性矩阵
      		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);//利用单应性矩阵进行校正
      
      		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
      		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
      		P1 = cameraMatrix[0];
      		P2 = cameraMatrix[1];
      	}
      
      	//Precompute maps for cv::remap()
      	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);//校正
      	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);//校正
      
      	Mat canvas;
      	double sf;
      	int w, h;
      	if (!isVerticalStereo)//判断是V方向相等的校正
      	{
      		sf = 600. / MAX(imageSize.width, imageSize.height);
      		w = cvRound(imageSize.width*sf);
      		h = cvRound(imageSize.height*sf);
      		canvas.create(h, w * 2, CV_8UC3);
      	}
      	else
      	{
      		sf = 300. / MAX(imageSize.width, imageSize.height);
      		w = cvRound(imageSize.width*sf);
      		h = cvRound(imageSize.height*sf);
      		canvas.create(h * 2, w, CV_8UC3);
      	}
      
      	for (i = 0; i < nimages; i++)
      	{
      		for (k = 0; k < 2; k++)
      		{
      			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
      			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
      			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
      			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
      			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
      			if (useCalibrated)
      			{
      				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
      					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
      				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
      			}
      		}
      
      		if (!isVerticalStereo)
      		for (j = 0; j < canvas.rows; j += 16)
      			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
      		else
      		for (j = 0; j < canvas.cols; j += 16)
      			line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
      		imshow("rectified", canvas);
      		char c = (char)waitKey();
      		if (c == 27 || c == 'q' || c == 'Q')
      			break;
      	}
      }
      
      
      static bool readStringList(const string& filename, vector<string>& l)//读取照片路径
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
      
      int main(int argc, char** argv)//主函数
      {
      	Size boardSize;
      	string imagelistfn;
      	bool showRectified;
      	cv::CommandLineParser parser(argc, argv, "{w|9|}{h|6|}{nr||}{help||}{@input|stereo_calib.xml|}");
      	if (parser.has("help"))
      		return print_help();
      	showRectified = !parser.has("nr");//command中nr为空
      	imagelistfn = parser.get<string>("@input");//如入参数文件路径数组
      	boardSize.width = parser.get<int>("w");//读入参数w
      	boardSize.height = parser.get<int>("h");//读入参数h
      	if (!parser.check())
      	{
      		parser.printErrors();
      		return 1;
      	}
      	vector<string> imagelist;
      	bool ok = readStringList(imagelistfn, imagelist);//调用读取图片路径函数
      	if (!ok || imagelist.empty())
      	{
      		cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
      		return print_help();
      	}
      
      	StereoCalib(imagelist, boardSize, false, true, showRectified);//调用标定函数
      	return 0;
      }
    