#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstring>

#include <calib.h>

using CALIB::Calibration;

int main(int argc, char **argv){

  std::string image_path = "/home/victor/Darling/course/cvpr/Assignment1";

  if(argc < 2){
    std::printf("\ninput is none !!!");
    std::printf("\nthe default Assignment1 path is: \n__ %s __", image_path.c_str());
  }
  else{
    std::printf("\nget the input !!!");
    std::printf("\nthe input Assignment1 image path is \n __ %s __", image_path.c_str());
  }

  std::string Camera_A = image_path + "/Camera_A/Mode1";
  std::string Camera_B = image_path + "/Camera_B/Mode2";

//  std::vector<std::string> imageA_list;

  std::shared_ptr<Calibration>Calib(new Calibration(Camera_A));

}
