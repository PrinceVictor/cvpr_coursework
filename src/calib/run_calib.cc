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

  std::string image_path, log_path, para_save_path, para_name;

  int images_num = 10;

  if(argc < 2){

    LOG_OUTPUT("input is none !!!");

    image_path = "/home/victor/Darling/course/cvpr/Assignment1/Camera_A/Model1";
    log_path =  "/home/victor/Darling/course/cvpr/cvpr_coursework/cvpr_coursecode/log";
    para_save_path = "/home/victor/Darling/course/cvpr/cvpr_coursework/cvpr_coursecode/para";

    LOG_OUTPUT("the default Assignment1 path is: __ {} __", image_path.c_str());
  }
  else if(argc >= 2){

    LOG_OUTPUT("get the input image path!!!");

    image_path = argv[1];

    log_path = "../log";
    para_save_path = "../para";

    int start = image_path.find("Camera_");

    if(start == image_path.npos){
      para_name = "parameter.xml";
    }
    else{
      para_name = "parameter_" + image_path.substr(start, 8) + ".xml";
    }

    if(argc >= 3){
      images_num = std::stoi(argv[2]);
    }
    if(argc >= 4){
      para_name = argv[3];
    }

    LOG_OUTPUT("the input Assignment1 image path is  __ {} __", image_path.c_str());
  }

  FLAGS_log_dir = log_path;

  google::InitGoogleLogging(argv[0]);

  google::SetStderrLogging(google::INFO);

  LOG_OUTPUT("create calibration class");
//  std::vector<std::string> imageA_list;

  std::shared_ptr<Calibration>Calib(new Calibration(image_path, images_num));

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

  Calib->save_parameters(para_save_path, para_name);

  google::ShutdownGoogleLogging();

  return 0;

}
