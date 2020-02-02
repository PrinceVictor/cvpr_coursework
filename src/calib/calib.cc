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
      ("/home/victor/Darling/course/cvpr/Assignment1/Camera_A/Mode1_temp");

}

Calibration::Calibration(const std::string& image_path,
                         const int& max_length){

  LOG_OUTPUT("call constructor function!!!"
              "\nthis constructor function do follow task"
              "\n1.gets the image path and image list"
              "\n2.Inilized the parameters");

  _chessis_board_image_path = std::make_shared<std::string>(image_path);

  set_board_size();

  _must_undistort = true;

  LOG_OUTPUT("it gets the chessis board "
              "image path: __ {} __", image_path.c_str());

  get_images_list(max_length);

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
  _object_corners = std::make_shared<std::vector<std::vector<cv::Point3f>>>();

  _image_corners->resize(_image_list->size());
  _object_corners->resize(_image_list->size());

  std::vector<cv::Point3f> object_corners = set_object_corners();

  std::string head_path = *_chessis_board_image_path;

  cv::Mat image;

  LOG_OUTPUT("image list size {}", _image_list->size());

//omp_set_num_threads(4);
//#pragma omp parallel for
  for(int i=0; i< _image_list->size(); i++){

    std::string full_path = head_path + '/' + (*_image_list)[i];

    //get the gray scale image
    image = cv::imread(full_path, cv::IMREAD_UNCHANGED);
//    LOG_OUTPUT("\nimage rows {}, cols {}, chanl {}, type {}",
//                image.rows, image.cols, image.channels(), image.type());

    std::vector<cv::Point2f> image_corners;

    bool is_found = cv::findChessboardCorners(image,
                                              *_board_size,
                                              image_corners);

//    cv::Mat image_copy = image.clone();
//    cv::drawChessboardCorners(image_copy, *_board_size, image_corners, is_found);
//    std::printf("\n corners size: %d", image_corners.size());

    cv::Mat image_gray;
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

    cv::cornerSubPix(image_gray, image_corners, cv::Size(6, 6), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                                      cv::TermCriteria::EPS,
                                      50,
                                      0.05));

    cv::drawChessboardCorners(image, *_board_size, image_corners, is_found);
//    LOG_OUTPUT("corners size: {}", image_corners.size());

    _image_size = std::make_shared<cv::Size>(image.size());

    if(image_corners.size() == _board_size->area()){

      (*_image_corners)[i] = image_corners;
      (*_object_corners)[i] = object_corners;

//      for(int i=0; i<image_corners.size(); i++){
//        LOG_OUTPUT("image x:{} y:{}", image_corners[i].x, image_corners[i].y);
//        LOG_OUTPUT("object x:{} y:{}", object_corners[i].x, object_corners[i].y);
//      }

    }

//    cv::namedWindow("test1", cv::WINDOW_NORMAL);
//    cv::imshow("test1", image);

//    cv::namedWindow("test2", cv::WINDOW_NORMAL);
//    cv::imshow("test2", image_copy);

//    cv::waitKey(0);
    std::string write_name = "../result/corners/corners_" + (*_image_list)[i];
//    cv::imwrite(write_name, image);
  }

//  cv::destroyAllWindows();
  LOG_OUTPUT("image rows {}, cols {}, chanl {}, type {}",
             image.rows, image.cols, image.channels(), image.type());

  return *_image_corners;
}

double Calibration::camera_calibrate(){

  if(_image_corners->size() == 0){

    LOG_OUTPUT("NO IMAGE CONCERS ---> Computing Failed!!!");
    return 0;
  }

  _must_undistort = true;

  return cv::calibrateCamera(*_object_corners,
                             *_image_corners,
                             *_image_size,
                             _camera_K_matrix,
                             _distort_coeff,
                             _R_list,
                             _T_list);
}

std::vector<cv::Point3f> Calibration::set_object_corners(const float &delat_x,
                                                         const float &delat_y){
  std::vector<cv::Point3f> object_corners;
  for(int i=0; i < _board_size->height; i++){
    for(int j=0; j < _board_size->width; j++){

      object_corners.emplace_back(cv::Point3f(j*delat_x, i*delat_y, 0.0));

    }
  }

  return object_corners;
}

void Calibration::image_undistort(){

  if(_image_corners->size() == 0){

    LOG_OUTPUT("get none sets of image corners and object corners");
    return ;
  }
  else{
    LOG_OUTPUT("sets of image corners and object corners {}", _image_corners->size());
  }


  cv::Rect* crop_ROI = new cv::Rect;

  cv::Mat new_camera_K_matrix;

  new_camera_K_matrix = cv::getOptimalNewCameraMatrix(_camera_K_matrix,
                                                      _distort_coeff,
                                                      *_image_size,
                                                      1,
                                                      *_image_size,
                                                      crop_ROI,
                                                      true);
  cv::Size input_size;
  cv::Mat new_K;
  if(crop_ROI->width && crop_ROI->height){
    new_K = new_camera_K_matrix;
  }
  else{
    input_size = *_image_size;
  }

  cv::Mat map1, map2;
  cv::initUndistortRectifyMap(_camera_K_matrix,
                              _distort_coeff,
                              cv::Mat(),
                              new_K,
                              *_image_size,
                              CV_32FC1,
                              map1,
                              map2);

  LOG_OUTPUT("ROI {} {} {} {}", crop_ROI->height,
                                crop_ROI->width,
                                crop_ROI->x,
                                crop_ROI->y);

  std::string head_path = *_chessis_board_image_path;

  for(auto it : *_image_list){

    std::string image_path = head_path + '/' + it;
    cv::Mat image = cv::imread(image_path);

    cv::Mat undistort_image;
    cv::remap(image, undistort_image, map1, map2, cv::INTER_LINEAR);

//    cv::undistort(image,
//                  undistort_image,
//                  _camera_K_matrix,
//                  _distort_coeff,
//                  new_camera_K_matrix);

    cv::namedWindow("raw_image", cv::WINDOW_NORMAL);
    cv::imshow("raw_image", image);

    cv::namedWindow("undistorted_image", cv::WINDOW_NORMAL);

//    cv::Mat roi =undistort_image(*crop_ROI);
//    undistort_image.copyTo(roi, *crop_ROI);

//    if(crop_ROI->height && crop_ROI->width){
//      cv::imshow("undistorted_image", undistort_image(*crop_ROI));
//    }
//    else{
      cv::imshow("undistorted_image", undistort_image);
//    }
    cv::waitKey(0);

    std::string write_name = "../result/undistort/undistort_" + it;
//    cv::imwrite(write_name, undistort_image);
  }

//  cv::destroyAllWindows();

  delete crop_ROI;

  return ;

}

double Calibration::get_project_error(){

  if(_image_corners->size() == 0){

    LOG_OUTPUT("get none sets of image corners and object corners");
    return 0;
  }
  else{
    LOG_OUTPUT("sets of image corners and object corners {}", _image_corners->size());
  }

  double error_sum = 0;

  LOG_OUTPUT("object corners {}", _object_corners->size());

  for(int i=0; i<_object_corners->size(); i++){

    std::vector<cv::Point2f> projected_image_points;

    cv::projectPoints((*_object_corners)[i],
                      _R_list[i],
                      _T_list[i],
                      _camera_K_matrix,
                      _distort_coeff,
//                      cv::Mat(),
                      projected_image_points);

//    LOG_OUTPUT("--------where---------");

    std::vector<cv::Point2f> temp_image_corners = (*_image_corners)[i];

//    cv::Mat temp_image_corners_mat = cv::Mat(1, temp_image_corners.size(), CV_32FC2);
//    cv::Mat projected_image_points_mat = cv::Mat(1, projected_image_points.size(), CV_32FC2);

    double temp_error = 0;
    for (int j = 0; j < temp_image_corners.size(); j++)
    {
//      temp_image_corners_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(temp_image_corners[j].x, temp_image_corners[j].y);
//      projected_image_points_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(projected_image_points[j].x, projected_image_points[j].y);

      double l2_error = 0;
      l2_error = std::pow(temp_image_corners[j].x - projected_image_points[j].x, 2) +
          std::pow(temp_image_corners[j].y - projected_image_points[j].y, 2);

      temp_error += l2_error;

//      LOG_OUTPUT("image x:{} y:{} || object x:{} y:{}", temp_image_corners[j].x, temp_image_corners[j].y,
//                                                        projected_image_points[j].x, projected_image_points[j].y);
//      LOG_OUTPUT("pixel error {}", l2_error);
    }

//    double err = cv::norm(temp_image_corners_mat, projected_image_points_mat, cv::NORM_L2); ///temp_image_corners.size();
    temp_error = temp_error / temp_image_corners.size();
    error_sum = temp_error + error_sum;

    LOG_OUTPUT("{}'s MSE: {:.3f}, RMSE: {:.3f}", (*_image_list)[i], temp_error, std::sqrt(temp_error));
  }

  errors.total_mse = error_sum;
  errors.total_rms = std::sqrt(error_sum);
  errors.mean_mse = error_sum/_image_list->size();
  errors.mean_rms = std::sqrt(errors.mean_mse);

  LOG_OUTPUT("Total MSE: {:.3f}, Mean per image MSE: {:.3f}, Mean per image RMSE: {:.3f}",
             errors.total_mse,
             errors.mean_mse,
             errors.mean_rms);

  return error_sum;
}

void Calibration::show_parameters(){

  LOG_OUTPUT("matrix K: {}x{}", _camera_K_matrix.rows,
                                _camera_K_matrix.cols);

  LOG_OUTPUT("matrix K: {:<9.2f} {:<9.2f} {:<9.2f}", _camera_K_matrix.at<double>(0,0),
                                                     _camera_K_matrix.at<double>(0,1),
                                                     _camera_K_matrix.at<double>(0,2));
  LOG_OUTPUT("          {:<9.2f} {:<9.2f} {:<9.2f}", _camera_K_matrix.at<double>(1,0),
                                                     _camera_K_matrix.at<double>(1,1),
                                                     _camera_K_matrix.at<double>(1,2));
  LOG_OUTPUT("          {:<9.2f} {:<9.2f} {:<9.2f}", _camera_K_matrix.at<double>(2,0),
                                                     _camera_K_matrix.at<double>(2,1),
                                                     _camera_K_matrix.at<double>(2,2));

  LOG_OUTPUT("distort_coeff: {}x{}", _distort_coeff.rows,
                                     _distort_coeff.cols);

  LOG_OUTPUT("distort_coeff: k1: {:.2f}", _distort_coeff.at<double>(0,0));
  LOG_OUTPUT("               k2: {:.2f}", _distort_coeff.at<double>(0,1));
  LOG_OUTPUT("               p1: {:.2f}", _distort_coeff.at<double>(0,2));
  LOG_OUTPUT("               p2: {:.2f}", _distort_coeff.at<double>(0,3));
  LOG_OUTPUT("               p3: {:.2f}", _distort_coeff.at<double>(0,4));

}

void Calibration::save_parameters(const std::string& path,
                                  const std::string& name){

  std::string para_name = path + '/' + name;

  _parameter_write = std::make_shared<cv::FileStorage>(para_name.c_str(),
                                                       cv::FileStorage::WRITE);

  *_parameter_write << "calibrated_parameters_file" << "Write_by_Hongbin_Zhou";

  std::time_t time_now;
  time_now = std::time(&time_now);

  *_parameter_write << "Date" << std::asctime(std::localtime(&time_now)) ;

  *_parameter_write << "path" << *_chessis_board_image_path;

  *_parameter_write << "iamge_files" << "[";

  for(auto it : *_image_list){

    *_parameter_write << it;
  }

  *_parameter_write << "]";

  *_parameter_write << "K_intrinsics_matrix" << _camera_K_matrix
                    << "distorted_coeffients" << _distort_coeff;

  *_parameter_write << "calibrate_errors" << "[";

  *_parameter_write << "{:" << "total_mse" << errors.total_mse
                    << "total_rms" << errors.total_rms
                    << "mean_mse" << errors.mean_mse
                    << "mean_rms" << errors.mean_rms << "}";

  *_parameter_write << "]";

  _parameter_write->release();

  LOG_OUTPUT("write parameters in to {}", para_name.c_str());
}

inline void Calibration::set_board_size(const int& point1, const int& point2){

  _board_size = std::make_shared<cv::Size>(point1, point2);
  LOG_OUTPUT("set board size width {} height {}", _board_size->width, _board_size->height);
}

}

