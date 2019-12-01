#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
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


}


