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
#include <Eigen/Eigen>

#include <log.h>
#include <calib.h>

namespace CALIB {

Calibration::Calibration(){

  LOG_OUTPUT("default constructor function!!!");

  _chessis_board_image_path = std::make_shared<std::string>
      ("/home/victor/Darling/course/cvpr/Assignment1/Camera_A/Mode1");

}

Calibration::Calibration(const std::string& image_path){

  LOG_OUTPUT("call constructor function!!!"
              "\nthis constructor function do follow task"
              "\n1.gets the image path and image list"
              "\n2.Inilized the parameters");

  _chessis_board_image_path = std::make_shared<std::string>(image_path);

  set_board_size();

  LOG_OUTPUT("it gets the chessis board "
              "image path: __ {} __", image_path.c_str());

  get_images_list();

}

Calibration::~Calibration(){
  LOG_OUTPUT("default destructor function !!!");
}

/**
 * @brief Calibration::get_images
 * @param max_length
 *        the maximum length of the image list
 */
std::vector<std::string> Calibration::get_images_list(const int& max_length){

  DIR* dir;
  dir = opendir(_chessis_board_image_path->c_str());
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
    _image_list = std::make_shared<std::vector<std::string>>(list);
  }
  else{
    std::srand(std::time(NULL));
    std::random_shuffle(list.begin(), list.end());

    _image_list = std::make_shared<std::vector<std::string>>();
    for(int i=0; i<max_length; i++){
      _image_list->emplace_back(list[i]);
    }
  }

  LOG_OUTPUT("Image List: ||");
  for(auto it : *_image_list){
    LOG_OUTPUT("__ {} __", it.c_str());
  }

  return *_image_list;

}

/**
 * @brief Calibration::change_image_path
 * @param image_path
 *        change the chess_board image path to this
 */
inline void Calibration::change_image_path(const std::string& image_path){

  _chessis_board_image_path = std::make_shared<std::string>(image_path);

}

std::vector<std::vector<cv::Point2f>> Calibration::get_chessis_board_corners(){

  _image_corners = std::make_shared<std::vector<std::vector<cv::Point2f>>>();

  std::string head_path = *_chessis_board_image_path;

  for(int i=0; i<1; i++){
    std::string full_path = head_path + '/' + (*_image_list)[i];

    //get the gray scale image
    cv::Mat image = cv::imread(full_path, cv::IMREAD_UNCHANGED);
    std::printf("\nimage rows %d, cols %d, chanl %d, type %d",
                image.rows, image.cols, image.channels(), image.type());

    std::vector<cv::Point2f> image_corners;

    bool is_found = cv::findChessboardCorners(image,
                                              *_board_size,
                                              image_corners);

//    cv::Mat image_copy = image.clone();
//    cv::drawChessboardCorners(image_copy, *_board_size, image_corners, is_found);
//    std::printf("\n corners size: %d", image_corners.size());

    cv::Mat image_gray;
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

    cv::cornerSubPix(image_gray, image_corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                                      cv::TermCriteria::EPS,
                                      30,
                                      0.1));

    cv::drawChessboardCorners(image, *_board_size, image_corners, is_found);
    LOG_OUTPUT("corners size: %ld", image_corners.size());

    if(image_corners.size() == _board_size->area()){
      _image_corners->emplace_back(image_corners);
    }

//    cv::namedWindow("test1", cv::WINDOW_NORMAL);
//    cv::imshow("test1", image);

//    cv::namedWindow("test2", cv::WINDOW_NORMAL);
//    cv::imshow("test2", image_copy);

//    cv::waitKey(0);

    return *_image_corners;
  }
}

std::vector<cv::Point3f> Calibration::set_object_corners(const float &delat_x,
                                                         const float &delat_y){
  _object_corners.reset();
}

inline void Calibration::set_board_size(const int& point1, const int& point2){
  _board_size = std::make_shared<cv::Size>(point1, point2);
}

}

