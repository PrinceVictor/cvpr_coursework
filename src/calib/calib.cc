// Copyright 2019 Hongbin Zhou
// License
// Author: Hongbin Zhou

#include <iostream>
#include <memory>
#include <vector>
#include <cstring>
#include <ctime>
#include <dirent.h>

#include <opencv2/opencv.hpp>

#include <calib.h>

namespace CALIB {

Calibration::Calibration(){

  std::printf("\ndefault constructor function!!!");

  chessis_board_image_path = std::make_shared<std::string>
      ("/home/victor/Darling/course/cvpr/Assignment1/Camera_A/Mode1");
  image_list = std::make_shared<std::vector<std::string>>();

  get_images();

}

Calibration::Calibration(const std::string image_path){

  std::printf("\ncall constructor function!!!"
              "\nthis constructor function gets the image path and image list");

  chessis_board_image_path = std::make_shared<std::string>(image_path);
  image_list = std::make_shared<std::vector<std::string>>();

  std::printf("\nit gets the chessis board "
              "image path:\n__ %S __", image_path.c_str());

  get_images();

}

Calibration::~Calibration(){
  std::printf("\ndefault destructor function !!!");
}


void Calibration::get_images(const int max_length){
  DIR* dir;
  dir = opendir(chessis_board_image_path->c_str());
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
    image_list = std::make_shared<std::vector<std::string>>(list);
  }
  else{
    std::srand(std::time(NULL));
    std::random_shuffle(list.begin(), list.end());
    for(int i=0; i<max_length; i++){
      image_list->emplace_back(list[i]);
    }
  }

  std::printf("\nImage List: ||");
  for(auto it : *image_list){
    std::printf("\n__ %s __", it.c_str());
  }

}

inline void Calibration::change_image_path(const std::string image_path){

  chessis_board_image_path = std::make_shared<std::string>(image_path);

}

}

