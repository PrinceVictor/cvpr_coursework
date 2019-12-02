// Copyright 2019 Hongbin Zhou
// License
// Author: Hongbin Zhou

#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>

#include <log.h>
#include <calib.h>

using CALIB::Calibration;

int main(int argc, char **argv){

  std::string image_path = "/home/victor/Darling/course/cvpr/Assignment1";

  if(argc < 2){
    LOG_OUTPUT("input is none !!!");
    LOG_OUTPUT("the default Assignment1 path is: __ {} __", image_path.c_str());
  }
  else{
    LOG_OUTPUT("get the input !!!");
    LOG_OUTPUT("the input Assignment1 image path is  __ {} __", image_path.c_str());
  }

  std::string Camera_A = image_path + "/Camera_A/Mode1";
  std::string Camera_B = image_path + "/Camera_B/Mode2";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  FLAGS_log_dir = "/home/victor/Darling/course/cvpr/cvpr_coursework/cvpr_coursecode/log";

  LOG_OUTPUT("create calibration class");
//  std::vector<std::string> imageA_list;

  std::shared_ptr<Calibration>Calib(new Calibration(Camera_A));

//  Calib->get_chessis_board_corners();

  google::ShutdownGoogleLogging();

  return 0;

}
