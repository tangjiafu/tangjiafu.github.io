# 1.包含目录 配置： #
（1）OpenCV安装目录下的...\include目录

（2）OpenCV安装目录下的...\include\opencv目录

（3）OpenCV安装目录下的...\include\opencv2目录

（ 注：此处也可以只 添加一个...\include目录 ）

 (这里最好每个项后面都加上分号)

#  2.库目录 配置： #

 具体操作与 “包含目录”的配置类似，只是添加的路径不同而已。

  OpenCV目录下的...x64\vc12\lib  

 注：  
- 此处的x64表示电脑是64位，32位选择x86  
- vc10表示VS是2010，vc11对应VS2012，vc12对应VS2013，vc14对应VS2015   
# 链接器 配置： #
将OpenCV安装目录下的库 的名字添加进来即可。

如：opencv_world310.lib  
（注：项目的 Debug配置则添加 以d结尾的lib文件项目的 Release配置则添加 其他的lib文件 ）  
至此，所有配置工作已完成。  

# 测试代码： #

              #include <opencv2\opencv.hpp>
              
              #include <iostream>
              
              int main()
              
              {
              
              //图像显示测试
              
              cv::Mat img = cv::imread("I://Desktop//picture//1.jpg");//注意这里的//的方向，一开始写反了，怎么都出不来。
              
              cv::imshow("gril", img);
              
              cv::waitKey(27);
              
              getchar();
              
              }