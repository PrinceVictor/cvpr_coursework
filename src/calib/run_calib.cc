#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstring>

#include <calib.h>

int main(int argc, char **argv){

  std::printf("%d \n%s ", argc, argv[0]);

  std::string image_path = "/home/victor/Darling/course/cvpr/Assignment1";

  if(argc < 2){
    std::printf("\ninput is none !!!");
    std::printf("\nthe default chessis board image path is: \n__ %s __", image_path.c_str());
  }
  else{
    std::printf("\nget the input !!!");
    std::printf("\nthe input chessis board image path is \n __ %s __", image_path.c_str());
  }

  std::string Camera_A = image_path + "/Camera_A/Mode1";
  std::string Camera_B = image_path + "/Camera_B/Mode2";

  std::vector<std::string> imageA_list;

  CALIB::get_images(Camera_A, imageA_list, 10);

  std::printf("\nImage List: ||");
  for(auto it : imageA_list){
    std::printf("\n__ %s __", it.c_str());
  }




}
