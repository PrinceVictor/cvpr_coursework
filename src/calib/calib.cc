#include <iostream>
#include <ctime>
#include <dirent.h>
#include <opencv2/opencv.hpp>

#include <calib.h>

namespace CALIB {

void get_images(const std::string image_path,
                std::vector<std::string>& image_list,
                const int max_length){
  DIR* dir;
  dir = opendir(image_path.c_str());
  struct dirent* ptr;
  std::vector<std::string> list;
  while((ptr = readdir(dir)) != NULL){
    if(ptr->d_name[0] == '.') continue;
    else{
      list.emplace_back(ptr->d_name);
    }
  }
  closedir(dir);

  if(list.size() <= max_length){
    image_list = list;
    return;
  }
  else{
    std::srand(std::time(NULL));
    std::random_shuffle(list.begin(), list.end());
    for(int i=0; i<max_length; i++){
      image_list.emplace_back(list[i]);
    }
    return;
  }
}

}

