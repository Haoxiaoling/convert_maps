#include"sttl.h"
#include<list>
#include <iostream>
#include<vector>
#include <Eigen/Geometry>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>


int CountLines(char *filename);
void readxyz2pcd(const std::string &file,const std::string &name,const std::string &floorpath);
int main(int argc,char** argv)
{
  if(argc != 5)
  {
    std::cerr<<"Usage: "<<std::endl;
    std::cerr<<"./loadtrans ".stt file path" "the .xyz file path" "the path the .pcd file you want to save from .xyz file" "the path you want to save the complete point cloud" "<<std::endl;
    exit(0);
  }

  std::list<sttl::Transform> transforms,transform1;
  sttl::loadList(transforms,argv[1]);
  std::string wfloor=argv[2];
  std::string cfloor=argv[3];
    //sttl::loadList(transform1,"2F_S10_Group2.stt");
  //transforms.splice(transforms.begin(), transform1);
  //sttl::saveList(transforms,"all.stt");
  std::cerr<<"There are "<<transforms.size()<<"transforms! "<<std::endl;

  std::list<sttl::Frame> frames;

  generateFrames(transforms,frames);
  std::cerr<<frames.size()<<std::endl;
  pcl::PointCloud<pcl::PointXYZ> complete_cloud;

  for(std::list<sttl::Frame>::iterator itr = frames.begin(); itr!= frames.end(); ++itr)
  {
    if(itr->name=="root")
      continue;
    readxyz2pcd(wfloor+itr->name+"_oct0_05.xyz",itr->name,cfloor);
  }
  //pcl::PointCloud<pcl::PointXYZ>::Ptr complete_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for(std::list<sttl::Frame>::iterator itr = frames.begin(); itr!= frames.end(); ++itr)
  {
    if(itr->name=="root")
      continue;
    Eigen::Affine3d transform=itr->rootTransform;
    ifstream fon((cfloor+itr->name+"_oct0_05.pcd").c_str());
    if(!fon)
      continue;
    //printf ("Method #1: using a Matrix4f\n");
    //std::cout << transform.matrix() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::io::loadPCDFile (cfloor+itr->name+"_oct0_05.pcd", *source_cloud);
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, transformed_cloud, transform);
    complete_cloud+=transformed_cloud;
    std::cerr<<"add the "<<itr->name<<"_oct0_05.pcd into maps"<<std::endl;
  }
  std::cerr<<"Finished adding all existing point cloud to maps"<<std::endl;

  //pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud=complete_cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (cloud_filtered);

  //pcl::io::savePCDFileASCII ("test_pcd.pcd", complete_cloud);
  pcl::io::savePCDFileASCII (argv[4], cloud_filtered);

  //pcl::io::savePCDFileASCII ("test_pcd.pcd", transformed_cloud);
  //std::list<sttl::Frame>::iterator itr = frames.begin();
  //std::cerr<<(itr+1)->name<<" and its transforms is "<<std::endl;
  /*for(std::list<sttl::Frame>::iterator itr = frames.begin(); itr!= frames.end(); ++itr)
  {
    std::cerr<<itr->name<<" and its parent is "<<std::endl;
    for(int i=0;i<4;i++)
    {
      for(int j=0;j<4;j++)
        std::cerr<<itr->rootTransform(i,j)<<" ";
      std::cerr<<std::endl;
    }
  }*/
  //std::cerr<<frames.begin()->name<<std::endl;
  //std::string path=frames.begin()->name+"_oct0_05.xyz";
  //std::cerr<<path<<std::endl;



    //frame=frame*frames.begin()->rootTransform;
    //std::cerr<<n<<" "<<frame.size()<<std::endl;
  //std::cerr<<sizeof(frames.begin()->rootTransform)<<std::endl;
}


int CountLines(const std::string &filename)
{
  std::ifstream ReadFile;
  int n=0;
  std::string tmp;
  ReadFile.open(filename.c_str(),std::ios::in);//ios::in 表示以只读的方式读取文件
  if(ReadFile.fail())//文件打开失败:返回0
  {
     return 0;
  }
  else//文件存在
  {
     while(std::getline(ReadFile,tmp))
     {
      n++;
     }
     return n;
  }
  ReadFile.close();
}

void readxyz2pcd(const std::string &file,const std::string &name,const std::string &floorpath)
{
  std::fstream fin;
	fin.open (file.c_str(), std::fstream::in);
  if(!fin.is_open())
    //std::cerr<<"file opens successfully"<<std::endl;
    return ;
  //const char* filename=file.c_str();
  //std::ifstream fin(file.c_str());
  int n=CountLines(file);
  double s;
  //std::vector< std::vector<double> > frame(n,std::vector<double>(3));
  Eigen::MatrixXd frame(n,3);
  int row=0;
  int col=0;
  int count=0;
   while( fin >> s )
   {
     count++;
     if(count>2 && count<6)
     {;
       frame(row,col++)=s;
       //std::cerr<<frame(row,col-1)<<" ";
     }
     if(count==8)
     {
       //frame(row,col)=1;
       count=0;
       col=0;
       row++;
       //std::cerr<<std::endl;
     }
   }
   std::cerr<<"this point cloud has "<<n<<" lines "<<std::endl;

   pcl::PointCloud<pcl::PointXYZ> cloud;
   cloud.width    = n;
   cloud.height   = 1;
   cloud.is_dense = false;
   cloud.points.resize (cloud.width * cloud.height);

   for(int i=0;i<frame.rows();i++)
   {
     cloud.points[i].x = frame(i,0);
     cloud.points[i].y = frame(i,1);
     cloud.points[i].z = frame(i,2);
   }
   pcl::io::savePCDFileASCII (floorpath+name+"_oct0_05.pcd", cloud);
   std::cerr<<"Finish convert .xyz to .pcd whose name is "<<name<<std::endl;
}

/*void buildthemap(const std::string &file,const pcl::PointCloud<pcl::PointXYZ> &cloud_temp)
{
  Eigen::Affine3d transform=frames.begin()->rootTransform;
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform.matrix() << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile ("2F/"+frames.begin()->name+"_oct0_05.pcd", *source_cloud);
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*source_cloud, transformed_cloud, transform);
}*/
