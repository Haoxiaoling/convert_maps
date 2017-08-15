#include<iostream>
#include <stdlib.h>
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>
#include<pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/filters/project_inliers.h>
#include<pcl/ModelCoefficients.h>
 #include <pcl/visualization/cloud_viewer.h>
 #include <pcl/filters/radius_outlier_removal.h>
 #include <pcl/filters/statistical_outlier_removal.h>

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output);

int main(int argc,char** argv)
{
  if(argc != 5)
  {
    std::cerr<<"please specify command line arg '-p', '-f'"<<std::endl;
    //exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr dcloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> cloud_projected;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr dcloud_left (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud);
  std::cout<<cloud->width*cloud->height<<std::endl;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(atof(argv[2]),atof(argv[3]));
  pass.filter(*cloud_filtered);

  downsample(cloud_filtered,dcloud_filtered);

  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_left);

  downsample(cloud_left,dcloud_left);

  pcl::PointXYZ minPt, maxPt;

   //获取坐标极值
  pcl::getMinMax3D(*cloud, minPt, maxPt);

   //输出结果
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
  /////////////////////////////////////////////////////////////
  //                  Max x: 29.6804                         //
  //                  Max y: 18.354                          //
  //                  Max z: 40.9599                         //
  //                  Min x: -29.883                         //
  //                  Min y: -29.9186                        //
  //                  Min z: 18.8391                         //
  /////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void calculateCoordinates(std::list<sttl::Frame> &frames, std::list<std::list<sttl::Transform>::iterator> &wavefront){            //
//                                                                                                                                  //
//	while(!wavefront.empty()){                                                                                                      //
//		std::list<sttl::Transform>::iterator trans = wavefront.front();                                                               //
//		wavefront.pop_front();                                                                                                        //
//		std::list<sttl::Frame>::iterator frame = findFrame(frames, trans->frameName);                                                 //
//		std::list<sttl::Frame>::iterator reference = findFrame(frames, trans->referenceFrameName);                                    //
//		if(frame->visited) continue;                                                                                                  //
//		if(!reference->visited){                                                                                                      //
//			std::cerr<<"Reference not visited - should not happen!!!"<<std::endl;                                                       //
//			continue;                                                                                                                   //
//		}                                                                                                                             //
//		frame->rootTransform = reference->rootTransform * trans->transform;                                                           //
//		frame->visited = true;                                                                                                        //
//		wavefront.insert(wavefront.end(), frame->children.begin(), frame->children.end());                                            //
//	}                                                                                                                               //
//                                                                                                                                  //
//}                                                                                                                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




  /*int **map;
  int n=(maxPt.x-minPt.x)/0.1+60;
  int m=(maxPt.y-minPt.y)/0.1+60;
  map=new int*[n];
  for(int i=0;i<n;i++)
  {
    map[i]=new int[m];
  }*/

  //std::cout<<sizeof(map)<<std::endl<<n<<" "<<m<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  //*cloud=complete_cloud;
  //pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered2);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>);
  // build the filter
  outrem.setInputCloud(cloud_filtered2);
  outrem.setRadiusSearch(0.1);
  outrem.setMinNeighborsInRadius (10);
  // apply filter
  outrem.filter (*cloud_filtered3);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered2);
  proj.setModelCoefficients (coefficients);
  proj.filter (cloud_projected);
  pcl::io::savePCDFileASCII (argv[4], cloud_projected);

  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
  //viewer.setFullScreen(true);
   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> left_cloud_color_handler (dcloud_left, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (dcloud_left, left_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_cloud_color_handler (dcloud_filtered, 230, 20, 20); // Red
  viewer.addPointCloud (dcloud_filtered, filtered_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem (1.0, "cloud",0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position
  //pcl::visualization::CloudViewer viewer ("Matrix transformation example");
  //viewer.showCloud(cloud_filtered);
   while (!viewer.wasStopped ())
   {
     viewer.spinOnce ();
   }
  return(0);
}

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
  //pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> fil;
  fil.setInputCloud (input);
  fil.setLeafSize (0.1f, 0.1f, 0.1f);
  fil.filter (*output);
}
