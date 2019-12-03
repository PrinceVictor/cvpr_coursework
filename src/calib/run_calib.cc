// Copyright 2019 Hongbin Zhou
// License
// Author: Hongbin Zhou

#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <ctime>

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

  std::string Camera_A = image_path + "/Camera_A/Mode1_temp";
  std::string Camera_B = image_path + "/Camera_B/Mode2";

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  FLAGS_log_dir = "/home/victor/Darling/course/cvpr/cvpr_coursework/cvpr_coursecode/log";

  LOG_OUTPUT("create calibration class");
//  std::vector<std::string> imageA_list;

  std::shared_ptr<Calibration>Calib(new Calibration(Camera_A));

  LOG_OUTPUT("get_chessis_board_corners");

  Calib->get_chessis_board_corners();

  LOG_OUTPUT("camera_calibrate");

  Calib->camera_calibrate();

  LOG_OUTPUT("image_undistort");

  Calib->image_undistort();

  LOG_OUTPUT("show_parameters");

  Calib->show_parameters();

  LOG_OUTPUT("show_project_errors");

  Calib->get_project_error();

  LOG_OUTPUT("save_parameters");

  std::string para_save = "/home/victor/Darling/course/cvpr/cvpr_coursework/cvpr_coursecode/para";
  std::string para_name = "parameter.xml";

  Calib->save_parameters(para_save, para_name);

  google::ShutdownGoogleLogging();

  return 0;

}
