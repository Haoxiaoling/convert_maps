/*
      writed by Xiaoling
      time:2017.8.16
      for: generate transform tree from the .stt files.

*/
#include <iostream>
#include <string>
#include"sttl.h"
#include <list>
#include <iterator>
#include <stdint.h>
#include <dirent.h>
#include<vector>
#include <fstream>

static int scan_dir(std::string path,std::list<std::string> &filepath);
static int filterDot(const struct dirent * dir);

int main(int argc,char** argv)
{
  /*std::list<sttl::Transform> transforms;
  sttl::loadList(transforms,argv[1]);*/
    if(argc!=3)
    {
      std::cerr<<"The usage:"<<std::endl;
      std::cerr<<"./graphviz \"the path of .stt files\" \"output file path\" \"Just do huangkuan.stt?\""<<std::endl;
      exit(0);
    }
    std::list<std::string> filepath;
    scan_dir(argv[1],filepath);

    std::list<sttl::Transform> transforms;

    std::list<sttl::Frame> frames;


    for(std::list<std::string>::iterator itr=filepath.begin();itr!=filepath.end();++itr)
    {
        std::string arg=argv[3];
        if(arg=="h")
        {
          std::string a=*itr;
          int found = a.find("_huangkun.stt");
          if(found!=std::string::npos)
          {
            std::list<sttl::Transform> transtemp;
            std::cerr<<*itr<<std::endl;
            sttl::loadList(transtemp,*itr);
            transforms.splice(transforms.end(), transtemp);
          }
        }
        else
        {
          std::string a=*itr;
          int found = a.find(".stt");
          if(found!=std::string::npos)
          {

            std::list<sttl::Transform> transtemp;
            std::cerr<<*itr<<std::endl;
            sttl::loadList(transtemp,*itr);
            transforms.splice(transforms.end(), transtemp);
          }
        }
    }

    //generateFrames(transforms,frames);
    std::cerr<<transforms.size()<<std::endl;
    std::vector<std::string> tree;
    tree.push_back("digraph G {");
    tree.push_back("size = \"200,200\"");
    tree.push_back("root [shape=box,style=filled,color=green]");
    for(std::list<sttl::Transform>::iterator ii=transforms.begin();ii!=transforms.end();++ii)
    {
      tree.push_back(ii->referenceFrameName+" -> "+ii->frameName);
    }
    tree.push_back("}");

    std::ofstream output_file(argv[2]);
    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(tree.begin(), tree.end(), output_iterator);
    std::cerr<<argv[3]<<std::endl;
    /*std::list<sttl::Transform> transtest;
    sttl::loadList(transtest,argv[2]);
    std::vector<std::string> tree;
    tree.push_back("digraph G {");
    for(std::list<sttl::Transform>::iterator ii=transtest.begin();ii!=transtest.end();++ii)
    {
      tree.push_back(ii->referenceFrameName+" -> "+ii->frameName);
    }
    tree.push_back("}");

    std::ofstream output_file(argv[3]);
    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(tree.begin(), tree.end(), output_iterator);*/
}


static int scan_dir(std::string path,std::list<std::string> &filepath)
{
  struct dirent **namelist;
  int n;
  n = scandir(path.c_str(), &namelist, filterDot, alphasort);

  for(int i=0;i<n;i++)
  {
    std::string subpath=path+"/"+namelist[i]->d_name;
    if(namelist[i]->d_type==DT_DIR)
      scan_dir(subpath,filepath);
    else filepath.push_back(subpath);
  }
}

static int filterDot(const struct dirent * dir) {
    if (strcmp(dir->d_name,".") == 0 || strcmp(dir->d_name, "..") == 0) {
        // 过滤掉 "."和".."
        return 0;
    } else {
        return 1;
    }
}
