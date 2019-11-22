#  更多内容请前往Github搜索tangjiafu或者ImageProcessing_Tang   #
####特别感谢slam十四讲提供的源码####
            #include <iostream>
            #include <fstream>
            #include <opencv2/core/core.hpp>
            #include <opencv2/highgui/highgui.hpp>
            #include <boost/format.hpp>
            #include <Eigen/Geometry>
            #include <pcl/point_types.h>
            #include <pcl/io/pcd_io.h>
            #include <pcl/visualization/pcl_visualizer.h>
            #include <pcl/visualization/cloud_viewer.h>
            
            using namespace std;
            int main()
            {
            	vector<cv::Mat> colorImgs, depthImgs;    //彩色图和深度图
            	vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿
            
            	ifstream fin("./pose.txt");
            	if (!fin)
            	{
            		cerr << "请在有pose.txt的目录下运行此程序" << endl;
            		return 1;
            	}
            
            	for (int i = 0; i<5; i++)
            	{
            		boost::format fmt("./%s/%d.%s"); //图像文件格式
            		colorImgs.push_back(cv::imread((fmt%"color" % (i + 1) % "png").str()));
            		depthImgs.push_back(cv::imread((fmt%"depth" % (i + 1) % "pgm").str(), -1)); // 使用-1读取原始图像
            
            		double data[7] = { 0 };
            		for (auto& d : data)
            			fin >> d;
            		Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
            		Eigen::Isometry3d T(q);
            		T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
            		poses.push_back(T);
            	}
            
            	// 计算点云并拼接
            	// 相机内参 
            	double cx = 325.5;
            	double cy = 253.5;
            	double fx = 518.0;
            	double fy = 519.0;
            	double depthScale = 1000.0;
            
            	cout << "正在将图像转换为点云..." << endl;
            
            	// 定义点云使用的格式：这里用的是XYZRGB
            	typedef pcl::PointXYZRGB PointT;
            	typedef pcl::PointCloud<PointT> PointCloud;
            
            	// 新建一个点云
            	PointCloud::Ptr pointCloud(new PointCloud);
            	for (int i = 0; i<5; i++)
            	{
            		cout << "转换图像中: " << i + 1 << endl;
            		cv::Mat color = colorImgs[i];
            		cv::Mat depth = depthImgs[i];
            		Eigen::Isometry3d T = poses[i];
            		for (int v = 0; v<color.rows; v++)
            		for (int u = 0; u<color.cols; u++)
            		{
            			unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
            			if (d == 0) continue; // 为0表示没有测量到
            			Eigen::Vector3d point;
            			point[2] = double(d) / depthScale;
            			point[0] = (u - cx)*point[2] / fx;
            			point[1] = (v - cy)*point[2] / fy;
            			Eigen::Vector3d pointWorld = T*point;
            
            			PointT p;
            			p.x = pointWorld[0];
            			p.y = pointWorld[1];
            			p.z = pointWorld[2];
            			p.b = color.data[v*color.step + u*color.channels()];
            			p.g = color.data[v*color.step + u*color.channels() + 1];
            			p.r = color.data[v*color.step + u*color.channels() + 2];
            			pointCloud->points.push_back(p);
            		}
            	}
            
            	pointCloud->is_dense = false;
            	cout << "点云共有" << pointCloud->size() << "个点." << endl;
            	pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
            
            
            
            
            
            
            	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            
            	std::string dir = "E:\\postgraduat\\Learn_class\\slam\\pointJoint\\pointJoint\\";
            	std::string filename = "map.pcd";
            
            	if (pcl::io::loadPCDFile<PointT>((dir + filename), *cloud) == -1){
            		//* load the file 
            		PCL_ERROR("Couldn't read PCD file \n");
            		return (-1);
            	}
            	printf("Loaded %d data points from PCD\n",
            		cloud->width * cloud->height);
            
            	for (size_t i = 0; i < cloud->points.size(); i += 10000)
            		printf("%8.3f %8.3f %8.3f %5d %5d %5d %5d\n",
            		cloud->points[i].x,
            		cloud->points[i].y,
            		cloud->points[i].z,
            		cloud->points[i].r,
            		cloud->points[i].g,
            		cloud->points[i].b,
            		cloud->points[i].a
            		);
            
            	pcl::visualization::PCLVisualizer viewer("Cloud viewer");
            	viewer.setCameraPosition(0, 0, -3.0, 0, -1, 0);
            	viewer.addCoordinateSystem(0.3);
            
            	viewer.addPointCloud(cloud);
            	while (!viewer.wasStopped())
            		viewer.spinOnce(100);
            	system("pause");
            
            	return 0;
            }
