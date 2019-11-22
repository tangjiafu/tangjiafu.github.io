# VS2013+PCL1.8环境配置 #

一般是需要将环境设为64位
配置包括包含目录，附加库目录，附加依赖项目录


C/C++的“常规”，主要用来设置库的头文件路径。这里设置好PCL及第三方库的头文件路径。

## 1.附加包含目录添加：注意PCL安装路径 ##

          D:\PCL 1.8.0\include\pcl-1.8    
          D:\PCL 1.8.0\3rdParty\Boost\include\boost-1_59    
          D:\PCL 1.8.0\3rdParty\Eigen\eigen3    
          D:\PCL 1.8.0\3rdParty\FLANN\include       
          D:\PCL 1.8.0\3rdParty\OpenNI2\Include   
          D:\PCL 1.8.0\3rdParty\Qhull\include   
          D:\PCL 1.8.0\3rdParty\VTK\include\vtk-7.0   
## 2.附加库目录，其在项目属性的“连接器”子项的“常规”下，主要用来设置库的lib文件路径。这里设置好设置PCL及第三方库的lib文件路径

          D:\PCL 1.8.0\lib
          D:\PCL 1.8.0\3rdParty\Boost\lib  
          D:\PCL 1.8.0\3rdParty\FLANN\lib         
          D:\PCL 1.8.0\3rdParty\OpenNI2\lib          
          D:\PCL 1.8.0\3rdParty\Qhull\lib          
          D:\PCL 1.8.0\3rdParty\VTK\lib 
## 3. 附加依赖项，其在项目属性的“连接器”子项的“输入”下，主要用来设置编译所需的lib。这里设置好设置PCL及第三方库的lib文件名。 ##
          pcl_apps_debug.lib  
          pcl_common_debug.lib  
          pcl_features_debug.lib  
          pcl_filters_debug.lib 
          pcl_io_debug.lib  
          pcl_io_ply_debug.lib    
          pcl_kdtree_debug.lib  
          pcl_keypoints_debug.lib 
          pcl_ml_debug.lib  
          pcl_octree_debug.lib  
          pcl_outofcore_debug.lib 
          pcl_people_debug.lib  
          pcl_recognition_debug.lib 
          pcl_registration_debug.lib  
          pcl_sample_consensus_debug.lib  
          pcl_search_debug.lib  
          pcl_segmentation_debug.lib  
          pcl_simulation_debug.lib  
          pcl_stereo_debug.lib  
          pcl_surface_debug.lib 
          pcl_tracking_debug.lib  
          pcl_visualization_debug.lib 
          libboost_atomic-vc120-mt-gd-1_59.lib    
          libboost_chrono-vc120-mt-gd-1_59.lib        
          libboost_container-vc120-mt-gd-1_59.lib     
          libboost_context-vc120-mt-gd-1_59.lib     
          libboost_coroutine-vc120-mt-gd-1_59.lib     
          libboost_date_time-vc120-mt-gd-1_59.lib   
          libboost_exception-vc120-mt-gd-1_59.lib       
          libboost_filesystem-vc120-mt-gd-1_59.lib    
          libboost_graph-vc120-mt-gd-1_59.lib   
          libboost_iostreams-vc120-mt-gd-1_59.lib   
          libboost_locale-vc120-mt-gd-1_59.lib    
          libboost_log-vc120-mt-gd-1_59.lib   
          libboost_log_setup-vc120-mt-gd-1_59.lib   
          libboost_math_c99-vc120-mt-gd-1_59.lib    
          libboost_math_c99f-vc120-mt-gd-1_59.lib   
          libboost_math_c99l-vc120-mt-gd-1_59.lib   
          libboost_math_tr1-vc120-mt-gd-1_59.lib    
          libboost_math_tr1f-vc120-mt-gd-1_59.lib   
          libboost_math_tr1l-vc120-mt-gd-1_59.lib   
          libboost_mpi-vc120-mt-gd-1_59.lib   
          libboost_prg_exec_monitor-vc120-mt-gd-1_59.lib    
          libboost_program_options-vc120-mt-gd-1_59.lib   
          libboost_random-vc120-mt-gd-1_59.lib    
          libboost_regex-vc120-mt-gd-1_59.lib       
          libboost_serialization-vc120-mt-gd-1_59.lib   
          libboost_signals-vc120-mt-gd-1_59.lib   
          libboost_system-vc120-mt-gd-1_59.lib    
          libboost_test_exec_monitor-vc120-mt-gd-1_59.lib     
          libboost_thread-vc120-mt-gd-1_59.lib    
          libboost_timer-vc120-mt-gd-1_59.lib   
          libboost_unit_test_framework-vc120-mt-gd-1_59.lib   
          libboost_wave-vc120-mt-gd-1_59.lib    
          libboost_wserialization-vc120-mt-gd-1_59.lib    
          flann_cpp_s-gd.lib    
          flann_s-gd.lib    
          flann-gd.lib    
          OpenNI2.lib   
          qhullstatic_r-gd.lib    
          qhull_p-gd.lib    
          qhull_r-gd.lib    
          qhullcpp-gd.lib   
          qhull-gd.lib    
          qhullstatic-gd.lib                  
          vtkalglib-7.0-gd.lib               
          vtkChartsCore-7.0-gd.lib                     
          vtkCommonColor-7.0-gd.lib              
          vtkCommonComputationalGeometry-7.0-gd.lib              
          vtkCommonCore-7.0-gd.lib              
          vtkCommonDataModel-7.0-gd.lib                 
          vtkCommonExecutionModel-7.0-gd.lib               
          vtkCommonMath-7.0-gd.lib               
          vtkCommonMisc-7.0-gd.lib       
          vtkCommonSystem-7.0-gd.lib           
          vtkCommonTransforms-7.0-gd.lib                                 
          vtkDICOMParser-7.0-gd.lib                        
          vtkDomainsChemistry-7.0-gd.lib                    
          vtkDomainsChemistryOpenGL2-7.0-gd.lib          
          vtkexoIIc-7.0-gd.lib                 
          vtkexpat-7.0-gd.lib                        
          vtkFiltersAMR-7.0-gd.lib               
          vtkFiltersCore-7.0-gd.lib                   
          vtkFiltersExtraction-7.0-gd.lib                
          vtkFiltersFlowPaths-7.0-gd.lib           
          vtkFiltersGeneral-7.0-gd.lib              
          vtkFiltersGeneric-7.0-gd.lib         
          vtkFiltersGeometry-7.0-gd.lib      
          vtkFiltersHybrid-7.0-gd.lib    
          vtkFiltersHyperTree-7.0-gd.lib     
          vtkFiltersImaging-7.0-gd.lib     
          vtkFiltersModeling-7.0-gd.lib      
          vtkFiltersParallel-7.0-gd.lib      
          vtkFiltersParallelImaging-7.0-gd.lib     
          vtkFiltersProgrammable-7.0-gd.lib      
          vtkFiltersSelection-7.0-gd.lib     
          vtkFiltersSMP-7.0-gd.lib     
          vtkFiltersSources-7.0-gd.lib     
          vtkFiltersStatistics-7.0-gd.lib      
          vtkFiltersTexture-7.0-gd.lib     
          vtkFiltersVerdict-7.0-gd.lib       
          vtkfreetype-7.0-gd.lib     
          vtkGeovisCore-7.0-gd.lib         
          vtkglew-7.0-gd.lib         
          vtkGUISupportQt-7.0-gd.lib     
          vtkGUISupportQtSQL-7.0-gd.lib            
          vtkhdf5-7.0-gd.lib     
          vtkhdf5_hl-7.0-gd.lib        
          vtkImagingColor-7.0-gd.lib     
          vtkImagingCore-7.0-gd.lib        
          vtkImagingFourier-7.0-gd.lib     
          vtkImagingGeneral-7.0-gd.lib     
          vtkImagingHybrid-7.0-gd.lib        
          vtkImagingMath-7.0-gd.lib              
          vtkImagingMorphological-7.0-gd.lib   
          vtkImagingSources-7.0-gd.lib   
          vtkImagingStatistics-7.0-gd.lib    
          vtkImagingStencil-7.0-gd.lib                 
          vtkInfovisCore-7.0-gd.lib    
          vtkInfovisLayout-7.0-gd.lib        
          vtkInteractionImage-7.0-gd.lib           
          vtkInteractionStyle-7.0-gd.lib       
          vtkInteractionWidgets-7.0-gd.lib      
          vtkIOAMR-7.0-gd.lib      
          vtkIOCore-7.0-gd.lib       
          vtkIOEnSight-7.0-gd.lib      
          vtkIOExodus-7.0-gd.lib         
          vtkIOExport-7.0-gd.lib     
          vtkIOGeometry-7.0-gd.lib       
          vtkIOImage-7.0-gd.lib      
          vtkIOImport-7.0-gd.lib     
          vtkIOInfovis-7.0-gd.lib      
          vtkIOLegacy-7.0-gd.lib     
          vtkIOLSDyna-7.0-gd.lib     
          vtkIOMINC-7.0-gd.lib     
          vtkIOMovie-7.0-gd.lib      
          vtkIONetCDF-7.0-gd.lib     
          vtkIOParallel-7.0-gd.lib     
          vtkIOParallelXML-7.0-gd.lib         
          vtkIOPLY-7.0-gd.lib     
          vtkIOSQL-7.0-gd.lib       
          vtkIOVideo-7.0-gd.lib     
          vtkIOXML-7.0-gd.lib     
          vtkIOXMLParser-7.0-gd.lib     
          vtkjpeg-7.0-gd.lib      
          vtkjsoncpp-7.0-gd.lib         
          vtklibxml2-7.0-gd.lib     
          vtkmetaio-7.0-gd.lib    
          vtkNetCDF-7.0-gd.lib      
          vtkNetCDF_cxx-7.0-gd.lib    
          vtkoggtheora-7.0-gd.lib     
          vtkParallelCore-7.0-gd.lib      
          vtkpng-7.0-gd.lib     
          vtkproj4-7.0-gd.lib      
          vtkRenderingAnnotation-7.0-gd.lib    
          vtkRenderingContext2D-7.0-gd.lib       
          vtkRenderingContextOpenGL2-7.0-gd.lib      
          vtkRenderingCore-7.0-gd.lib     
          vtkRenderingFreeType-7.0-gd.lib      
          vtkRenderingImage-7.0-gd.lib     
          vtkRenderingLabel-7.0-gd.lib     
          vtkRenderingLOD-7.0-gd.lib     
          vtkRenderingOpenGL2-7.0-gd.lib     
          vtkRenderingQt-7.0-gd.lib      
          vtkRenderingVolume-7.0-gd.lib      
          vtkRenderingVolumeOpenGL2-7.0-gd.lib     
          vtksqlite-7.0-gd.lib      
          vtksys-7.0-gd.lib      
          vtktiff-7.0-gd.lib     
          vtkverdict-7.0-gd.lib      
          vtkViewsContext2D-7.0-gd.lib       
          vtkViewsCore-7.0-gd.lib      
          vtkViewsInfovis-7.0-gd.lib     
          vtkViewsQt-7.0-gd.lib            
          vtkzlib-7.0-gd.lib           
  
# X64位的release的附加依赖项配置 #
            pcl_common_release.lib      
            pcl_features_release.lib      
            pcl_filters_release.lib     
            pcl_io_ply_release.lib      
            pcl_io_release.lib      
            pcl_kdtree_release.lib      
            pcl_keypoints_release.lib       
            pcl_ml_release.lib    
            pcl_octree_release.lib              
            pcl_outofcore_release.lib   
            pcl_people_release.lib    
            pcl_recognition_release.lib     
            pcl_registration_release.lib      
            pcl_sample_consensus_release.lib      
            pcl_search_release.lib      
            pcl_segmentation_release.lib      
            pcl_stereo_release.lib        
            pcl_surface_release.lib     
            pcl_tracking_release.lib    
            pcl_visualization_release.lib     
            libboost_atomic-vc120-mt-1_59.lib     
            libboost_chrono-vc120-mt-1_59.lib     
            libboost_container-vc120-mt-1_59.lib      
            libboost_context-vc120-mt-1_59.lib      
            libboost_coroutine-vc120-mt-1_59.lib      
            libboost_date_time-vc120-mt-1_59.lib      
            libboost_exception-vc120-mt-1_59.lib    
            libboost_filesystem-vc120-mt-1_59.lib     
            libboost_graph-vc120-mt-1_59.lib        
            libboost_iostreams-vc120-mt-1_59.lib      
            libboost_locale-vc120-mt-1_59.lib       
            libboost_log-vc120-mt-1_59.lib      
            libboost_log_setup-vc120-mt-1_59.lib      
            libboost_math_c99-vc120-mt-1_59.lib     
            libboost_math_c99f-vc120-mt-1_59.lib      
            libboost_math_c99l-vc120-mt-1_59.lib      
            libboost_math_tr1-vc120-mt-1_59.lib     
            libboost_math_tr1f-vc120-mt-1_59.lib      
            libboost_math_tr1l-vc120-mt-1_59.lib      
            libboost_mpi-vc120-mt-1_59.lib      
            libboost_prg_exec_monitor-vc120-mt-1_59.lib     
            libboost_program_options-vc120-mt-1_59.lib      
            libboost_random-vc120-mt-1_59.lib     
            libboost_regex-vc120-mt-1_59.lib      
            libboost_serialization-vc120-mt-1_59.lib    
            libboost_signals-vc120-mt-1_59.lib      
            libboost_system-vc120-mt-1_59.lib     
            libboost_test_exec_monitor-vc120-mt-1_59.lib      
            libboost_thread-vc120-mt-1_59.lib     
            libboost_timer-vc120-mt-1_59.lib      
            libboost_unit_test_framework-vc120-mt-1_59.lib    
            libboost_wave-vc120-mt-1_59.lib   
            libboost_wserialization-vc120-mt-1_59.lib     
            flann_cpp_s.lib     
            flann_s.lib     
            qhull.lib     
            qhullcpp.lib      
            qhullstatic.lib     
            qhullstatic_r.lib       
            qhull_p.lib     
            qhull_r.lib       
            vtkalglib-7.0.lib     
            vtkChartsCore-7.0.lib     
            vtkCommonColor-7.0.lib      
            vtkCommonComputationalGeometry-7.0.lib      
            vtkCommonCore-7.0.lib     
            vtkCommonDataModel-7.0.lib      
            vtkCommonExecutionModel-7.0.lib     
            vtkCommonMath-7.0.lib       
            vtkCommonMisc-7.0.lib     
            vtkCommonSystem-7.0.lib       
            vtkCommonTransforms-7.0.lib       
            vtkDICOMParser-7.0.lib      
            vtkDomainsChemistry-7.0.lib       
            vtkDomainsChemistryOpenGL2-7.0.lib    
            vtkexoIIc-7.0.lib       
            vtkexpat-7.0.lib    
            vtkFiltersAMR-7.0.lib     
            vtkFiltersCore-7.0.lib        
            vtkFiltersExtraction-7.0.lib        
            vtkFiltersFlowPaths-7.0.lib       
            vtkFiltersGeneral-7.0.lib     
            vtkFiltersGeneric-7.0.lib       
            vtkFiltersGeometry-7.0.lib      
            vtkFiltersHybrid-7.0.lib      
            vtkFiltersHyperTree-7.0.lib       
            vtkFiltersImaging-7.0.lib     
            vtkFiltersModeling-7.0.lib      
            vtkFiltersParallel-7.0.lib      
            vtkFiltersParallelImaging-7.0.lib     
            vtkFiltersProgrammable-7.0.lib      
            vtkFiltersSelection-7.0.lib       
            vtkFiltersSMP-7.0.lib     
            vtkFiltersSources-7.0.lib       
            vtkFiltersStatistics-7.0.lib        
            vtkFiltersTexture-7.0.lib       
            vtkFiltersVerdict-7.0.lib       
            vtkfreetype-7.0.lib       
            vtkGeovisCore-7.0.lib     
            vtkglew-7.0.lib       
            vtkGUISupportQt-7.0.lib       
            vtkGUISupportQtSQL-7.0.lib        
            vtkhdf5-7.0.lib       
            vtkhdf5_hl-7.0.lib        
            vtkImagingColor-7.0.lib       
            vtkImagingCore-7.0.lib        
            vtkImagingFourier-7.0.lib       
            vtkImagingGeneral-7.0.lib     
            vtkImagingHybrid-7.0.lib        
            vtkImagingMath-7.0.lib      
            vtkImagingMorphological-7.0.lib     
            vtkImagingSources-7.0.lib     
            vtkImagingStatistics-7.0.lib      
            vtkImagingStencil-7.0.lib       
            vtkInfovisCore-7.0.lib      
            vtkInfovisLayout-7.0.lib        
            vtkInteractionImage-7.0.lib     
            vtkInteractionStyle-7.0.lib       
            vtkInteractionWidgets-7.0.lib       
            vtkIOAMR-7.0.lib        
            vtkIOCore-7.0.lib       
            vtkIOEnSight-7.0.lib      
            vtkIOExodus-7.0.lib       
            vtkIOExport-7.0.lib     
            vtkIOGeometry-7.0.lib     
            vtkIOImage-7.0.lib      
            vtkIOImport-7.0.lib     
            vtkIOInfovis-7.0.lib    
            vtkIOLegacy-7.0.lib     
            vtkIOLSDyna-7.0.lib     
            vtkIOMINC-7.0.lib     
            vtkIOMovie-7.0.lib      
            vtkIONetCDF-7.0.lib     
            vtkIOParallel-7.0.lib     
            vtkIOParallelXML-7.0.lib      
            vtkIOPLY-7.0.lib      
            vtkIOSQL-7.0.lib      
            vtkIOVideo-7.0.lib      
            vtkIOXML-7.0.lib      
            vtkIOXMLParser-7.0.lib      
            vtkjpeg-7.0.lib     
            vtkjsoncpp-7.0.lib      
            vtklibxml2-7.0.lib      
            vtkmetaio-7.0.lib     
            vtkNetCDF-7.0.lib     
            vtkNetCDF_cxx-7.0.lib     
            vtkoggtheora-7.0.lib      
            vtkParallelCore-7.0.lib     
            vtkpng-7.0.lib      
            vtkproj4-7.0.lib      
            vtkRenderingAnnotation-7.0.lib      
            vtkRenderingContext2D-7.0.lib     
            vtkRenderingContextOpenGL2-7.0.lib      
            vtkRenderingCore-7.0.lib      
            vtkRenderingFreeType-7.0.lib      
            vtkRenderingImage-7.0.lib     
            vtkRenderingLabel-7.0.lib     
            vtkRenderingLOD-7.0.lib     
            vtkRenderingOpenGL2-7.0.lib     
            vtkRenderingQt-7.0.lib      
            vtkRenderingVolume-7.0.lib      
            vtkRenderingVolumeOpenGL2-7.0.lib     
            vtksqlite-7.0.lib       
            vtksys-7.0.lib      
            vtktiff-7.0.lib     
            vtkverdict-7.0.lib      
            vtkViewsContext2D-7.0.lib     
            vtkViewsCore-7.0.lib      
            vtkViewsInfovis-7.0.lib     
            vtkViewsQt-7.0.lib      
            vtkzlib-7.0.lib         

## 
# 修改系统环境变量 ## #
          ;%PCL_ROOT%\bin  
          ;%PCL_ROOT%\3rdParty\Qhull\bin  
          ;%PCL_ROOT%\3rdParty\FLANN\bin  
          ;%PCL_ROOT%\3rdParty\VTK\bin  
          ;%PCL_ROOT%\3rdParty\OpenNI2\Tools  

##
## 测试代码，由于缺少点云文件，但是如果没有出现关于PCL库的错误，即配置成功 ##
##
      #include <pcl/visualization/cloud_viewer.h>  
      #include <iostream>  
      #include <pcl/io/io.h>  
      #include <pcl/io/pcd_io.h>  
      #include <iostream>  
      #include <pcl/filters/filter.h>  
      int user_data;
      void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)    
      { 
      viewer.setBackgroundColor(1.0, 0.5, 1.0);  
      pcl::PointXYZ o; 
      o.x = 1.0; 
      o.y = 0; 
      o.z = 0; 
      viewer.addSphere(o, 0.25, "sphere", 0);  
      std::cout << "i only run once" << std::endl;     
      }  
      void  viewerPsycho(pcl::visualization::PCLVisualizer& viewer)   
      {
	   static unsigned count = 0;   
	   std::stringstream ss;    
	   ss << "Once per viewer loop: " << count++;   
	   viewer.removeShape("text", 0);   
	   viewer.addText(ss.str(), 200, 300, "text", 0);   
	//FIXME: possible race condition here:   
	   user_data++;   
      }   
      int main()    
      {  
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptrcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);   
	   pcl::io::loadPCDFile("test_cloud.pcd", *cloud);    
	   pcl::visualization::CloudViewer 
      viewer("Cloud Viewer");    
 
	//blocks until the cloud is actually rendered    
	   viewer.showCloud(cloud);   
 
	//use the following functions to get access to the underlying more advanced/powerful   
	//PCLVisualizer    
 
	//This will only get called once   
	   viewer.runOnVisualizationThreadOnce(viewerOneOff);   
 
	//This will get called once per visualization iteration    
	   viewer.runOnVisualizationThread(viewerPsycho);   
 
	   while (!viewer.wasStopped())   
	   {    
		std::cout << cloud->width << endl;    
		std::cout << cloud->height << endl;   
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho   
		//and you should guard against race conditions yourself...    
		user_data++;    
	   }    
	   return 0;    
      }  