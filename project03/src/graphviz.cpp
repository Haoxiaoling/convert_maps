#include <iostream>
#include <string>
#include"sttl.h"
#include <list>
#include <stdint.h>
#include <dirent.h>
#include <fstream>

static int scan_dir(std::string path,std::list<std::string> &filepath);
static int filterDot(const struct dirent * dir);

int main(int argc,char** argv)
{
  /*std::list<sttl::Transform> transforms;
  sttl::loadList(transforms,argv[1]);*/
    std::list<std::string> filepath;
    scan_dir(argv[1],filepath);

    std::list<sttl::Transform> transforms;

    std::list<sttl::Frame> frames;

    for(std::list<std::string>::iterator itr=filepath.begin();itr!=filepath.end();++itr)
    {
      std::list<sttl::Transform> transtemp;
      std::cerr<<*itr<<std::endl;
      sttl::loadList(transtemp,*itr);
      transforms.splice(transforms.end(), transtemp);
    }

    //generateFrames(transforms,frames);
    std::cerr<<transforms.size()<<std::endl;

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
