#更多内容请前往Github搜索tangjiafu或者ImageProcessing_Tang#
####特别感谢OPENCV提供的源码####      
      
      #include "opencv2/calib3d/calib3d.hpp"
      #include "opencv2/imgproc/imgproc.hpp"
      #include "opencv2/imgcodecs.hpp"
      #include "opencv2/highgui/highgui.hpp"
      #include "opencv2/core/utility.hpp"			  //调用opencv3.1图像处理库的头文件
      
      #include <stdio.h>	//C++输出输出函数的库
      
      using namespace cv;	//去除opencv库的函数的CV::
      
      static void print_help()  //控制台程序帮助函数
      {
      	printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
      	printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|sgbm3way] [--blocksize=<block_size>]\n"
      		"[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
      		"[--no-display] [-o=<disparity_image>] [-p=<point_cloud_file>]\n");
      }
      static void saveXYZ(const char* filename, const Mat& mat)  //储存XYZ三维空间坐标点，按行储存，每一行是一个坐标点，X,Y,Z    
      //按行读取，储存成为一个链表。注意matlab里面是按列读取
      {
      	const double max_z = 1.0e4;
      	FILE* fp = fopen(filename, "wt");
      	for (int y = 0; y < mat.rows; y++)
      	{
      		for (int x = 0; x < mat.cols; x++)
      		{
      			Vec3f point = mat.at<Vec3f>(y, x);	//Vec3f变量是为三维空间坐标点储存矩阵
      			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
      			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
      		}
      	}
      	fclose(fp);
      }
      
      int main(int argc, char** argv)
      {
      	std::string img1_filename = "";			  //图片文件1
      	std::string img2_filename = "";			  //图片文件2
      	std::string intrinsic_filename = "";	  //内部参数矩阵文件
      	std::string extrinsic_filename = "";	  //外部参数矩阵文件
      	std::string disparity_filename = "";	  //视差文件
      	std::string point_cloud_filename = "";	  //点云文件   //参数在这里设置
      
      	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };//匹配方法选择
      	int alg = STEREO_SGBM;	 //默认方法为SGBM
      	int SADWindowSize, numberOfDisparities;
      	bool no_display;   //布尔变量，决定是否显示
      	float scale;	  //尺度因子
      
      	Ptr<StereoBM> bm = StereoBM::create(16, 9);//创建BM匹配方法，变量名为bm
      	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);  //创建SGBM匹配方法，变量名为SGBM
      	cv::CommandLineParser parser(argc, argv,
      		"{max-disparity|64|}{blocksize|7|}{no-display||}{scale|1|}{help||}{algorithm|sgbm|}{@left|left01.jpg|}{@right|right01.jpg|}{i|intrinsics.xml|}{e|extrinsics.xml|}{o|disparity.xml|}{p|point.dat|}");
      	//	"{help h||}{algorithm||}{max-disparity|0|}{blocksize|0|}{no-display||}{scale|1|}{i||}{e||}{o||}{p||}");	//此行需要修改参数在这里设置
      	//此行视作为批量宏定义，｛name|value|｝是一个定义变量名|变量值|	
      	//下面若干行程序，将这些宏定义转化为实际的程序赋值和调用，其中包括一些check
      	if (parser.has("help"))
      	{
      		print_help();
      		return 0;
      	}
      	img1_filename = parser.get<std::string>("@left");
      	img2_filename = parser.get<std::string>("@right");
      	if (parser.has("algorithm"))
      	{
      		std::string _alg = parser.get<std::string>("algorithm"); //这几行程序非常关键，逻辑极其复杂，注意是?号三目运算符的嵌套，
      		alg = _alg == "bm" ? STEREO_BM :						 //
      			_alg == "sgbm" ? STEREO_SGBM :						 //
      			_alg == "hh" ? STEREO_HH :							 //
      			_alg == "var" ? STEREO_VAR :						 //
      			_alg == "sgbm3way" ? STEREO_3WAY : -1;				 //
      	}															 //注意，可以转换为即为C中的switch语句																			   、
      	numberOfDisparities = parser.get<int>("max-disparity");
      	SADWindowSize = parser.get<int>("blocksize");
      	scale = parser.get<float>("scale");
      	no_display = parser.has("no-display");
      	if (parser.has("i"))
      		intrinsic_filename = parser.get<std::string>("i");
      	if (parser.has("e"))
      		extrinsic_filename = parser.get<std::string>("e");
      	if (parser.has("o"))
      		disparity_filename = parser.get<std::string>("o");
      	if (parser.has("p"))
      		point_cloud_filename = parser.get<std::string>("p");
      	if (!parser.check())
      	{
      		parser.printErrors();
      		return 1;
      	}
      	if (alg < 0)				//以上为CommandLineParser类的具体调用，赋值，下面为check,仔细核对是否为空或者满足程序逻辑要求，保证不会出现程序中断
      	{
      		printf("Command-line parameter error: Unknown stereo algorithm\n\n");
      		print_help();
      		return -1;
      	}
      	if (numberOfDisparities < 1 || numberOfDisparities % 16 != 0)
      	{
      		printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
      		print_help();
      		return -1;
      	}
      	if (scale < 0)
      	{
      		printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
      		return -1;
      	}
      	if (SADWindowSize < 1 || SADWindowSize % 2 != 1)						   //保证SAD窗口边长为奇数
      	{
      		printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
      		return -1;
      	}
      	if (img1_filename.empty() || img2_filename.empty())						   //保证图片1，2文件都存在
      	{
      		printf("Command-line parameter error: both left and right images must be specified\n");
      		return -1;
      	}
      	if ((!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()))		  //保证内部外部参数矩阵文件存在
      	{
      		printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
      		return -1;
      	}
      
      	if (extrinsic_filename.empty() && !point_cloud_filename.empty())   //保证外部参数和点云矩阵的外部参数文件路径存在
      	{
      		printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
      		return -1;
      	}
      
      	int color_mode = alg == STEREO_BM ? 0 : -1;		//imread(filepath,flag)   flag>0, 该函数返回3通道图像，如果磁盘上的图像文件是单通道的灰度图像，则会被强制转为3通道；
      	//                         flag = 0, 该函数返回单通道图像，如果磁盘的图像文件是多通道的则会被强制转为单通道；
      	//                         flag<0, 则函数不对图像进行通道转换
      	Mat img1 = imread(img1_filename, color_mode);
      	Mat img2 = imread(img2_filename, color_mode);
      
      	if (img1.empty())	 //check 图片1是否存在
      	{
      		printf("Command-line parameter error: could not load the first input image file\n");
      		return -1;
      	}
      	if (img2.empty())	 //check  图片2是否存在  
      	{
      		printf("Command-line parameter error: could not load the second input image file\n");
      		return -1;
      	}
      
      	if (scale != 1.f)	 //scale默认设置1.0，说明该if语句默认不执行
      	{
      		Mat temp1, temp2;
      		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
      		resize(img1, temp1, Size(), scale, scale, method);
      		img1 = temp1;
      		resize(img2, temp2, Size(), scale, scale, method);
      		img2 = temp2;
      	}
      
      	Size img_size = img1.size();
      
      	Rect roi1, roi2;	//判断满足校正后的区域
      	Mat Q;
      
      	if (!intrinsic_filename.empty())   //读取内部参数.xml文件中的M1,D1,M2,D2;并将参数存储在M1,D1,M2,D2变量中
      	{
      		// reading intrinsic parameters
      		FileStorage fs(intrinsic_filename, FileStorage::READ);
      		if (!fs.isOpened())
      		{
      			printf("Failed to open file %s\n", intrinsic_filename.c_str());
      			return -1;
      		}
      
      		Mat M1, D1, M2, D2;
      		fs["M1"] >> M1;
      		fs["D1"] >> D1;
      		fs["M2"] >> M2;
      		fs["D2"] >> D2;
      
      		M1 *= scale;  //乘以尺度因子，默认为1.0
      		M2 *= scale;
      
      		fs.open(extrinsic_filename, FileStorage::READ);//读取外部参数.xml文件,并将R,T存储在R,T矩阵变量中
      		if (!fs.isOpened())
      		{
      			printf("Failed to open file %s\n", extrinsic_filename.c_str());
      			return -1;
      		}
      
      		Mat R, T, R1, P1, R2, P2;
      		fs["R"] >> R;
      		fs["T"] >> T;
      
      		stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);		  //调用校正函数，求参数R1,P1,R2,P2,Q
      
      		Mat map11, map12, map21, map22;		  //映射矩阵
      		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);//利用R1,P1,内部参数M1,D1，求映射矩阵
      		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);//利用R1,P1,内部参数M1,D1 ，求映射矩阵
      
      		Mat img1r, img2r;
      		remap(img1, img1r, map11, map12, INTER_LINEAR);//利用映射矩阵进行图片校正
      		remap(img2, img2r, map21, map22, INTER_LINEAR);//利用映射矩阵进行图片校正
      
      		img1 = img1r;
      		img2 = img2r;
      	}
      
      	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;//各种匹配方法共同的参数设置
      
      	bm->setROI1(roi1);										//下面为BM匹配方法的参数设置，这里将校正得到的有效区域传入匹配中
      	bm->setROI2(roi2);										//
      	bm->setPreFilterCap(31);								//
      	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);//
      	bm->setMinDisparity(0);									//
      	bm->setNumDisparities(numberOfDisparities);				//
      	bm->setTextureThreshold(10);							//
      	bm->setUniquenessRatio(15);								//
      	bm->setSpeckleWindowSize(100);							//
      	bm->setSpeckleRange(32);								//
      	bm->setDisp12MaxDiff(1);								//
      
      	sgbm->setPreFilterCap(63);								      //下面数行将SGBM方法的参数设置，
      	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;	  //
      	sgbm->setBlockSize(sgbmWinSize);							  //
      	//
      	int cn = img1.channels();									  //
      	//
      	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);				  //
      	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);				  //
      	sgbm->setMinDisparity(0);									  //
      	sgbm->setNumDisparities(numberOfDisparities);				  //
      	sgbm->setUniquenessRatio(10);								  //
      	sgbm->setSpeckleWindowSize(100);							  //
      	sgbm->setSpeckleRange(32);									  //
      	sgbm->setDisp12MaxDiff(1);									  //
      	if (alg == STEREO_HH)					             ///这个程序段，说明的是SGBM方法的不同分类，分别为HH,SGBM,SGBM_3WAY
      		sgbm->setMode(StereoSGBM::MODE_HH);				 ///
      	else if (alg == STEREO_SGBM)						 ///
      		sgbm->setMode(StereoSGBM::MODE_SGBM);			 ///
      	else if (alg == STEREO_3WAY)						 ///
      		sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);		 ///
      	Mat disp, disp8;
      	//Mat img1p, img2p, dispp;
      	//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
      	//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
      
      	int64 t = getTickCount();
      	if (alg == STEREO_BM)
      		bm->compute(img1, img2, disp);					                      //计算bm方法得到的视差图
      	else if (alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY)
      		sgbm->compute(img1, img2, disp);									  //计算sgbm方法得到的视差图
      	t = getTickCount() - t;
      	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());			  //打印时间，单位为ms
      
      	//disp = dispp.colRange(numberOfDisparities, img1p.cols);
      	if (alg != STEREO_VAR)
      		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
      	else
      		disp.convertTo(disp8, CV_8U);
      
      	if (!disparity_filename.empty())				 //判断视差图矩阵存储文件是否存在
      		//	FileStorage fs(disparity_filename,FileStorage::WRITE);	//FileStorage fs(intrinsic_filename, FileStorage::READ);
      		printf("Write the disparity to file \n");	  //printf提示调试时，控制台程序是否运行这一步，即在控制台可判断是否计算视差图
      	if (!point_cloud_filename.empty())				 //判断点云矩阵存储文件是否存在
      	{
      		printf("storing the point cloud...");		//printf提示调试时，控制台程序是否运行这一步
      		fflush(stdout);
      		Mat xyz;									//定义点云文件
      		reprojectImageTo3D(disp, xyz, Q, true);		//利用视差图和矩阵Q计算三维空间点
      		saveXYZ(point_cloud_filename.c_str(), xyz);	//存储三维空间点到点云.xml文件中
      		printf("\n");
      	}
      	if (!no_display)			  //该if程序段用于图片显示，显示原来图片，视差图，no_display布尔变量判断是否显示，T表示不显示 
      	{
      		namedWindow("left", 1);
      		imshow("left", img1);
      		namedWindow("right", 1);
      		imshow("right", img2);
      		namedWindow("disparity", 0);
      		imshow("disparity", disp8);
      		printf("press any key to continue...");
      		fflush(stdout);
      		waitKey(0);
      		printf("\n");
      	}
      	return 0;
      }
