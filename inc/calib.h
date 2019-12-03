// Copyright 2019 Hongbin Zhou
// License
// Author: Hongbin Zhou

#ifndef __CALIB_H__
#define __CALIB_H__

#include <iostream>
#include <vector>
#include <cstring>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#define FORMAT_STRING fmt::format
#define LOG_FUNCTION LOG(INFO)<<__FUNCTION__<<" | "
#define LOG_OUTPUT LOG_FUNCTION<<FORMAT_STRING

/**
 * @brief namespace for monocular camera calibration
 */
namespace CALIB {

/**
 * @brief The Calibration class
 */
class Calibration{

public:

  Calibration();

  Calibration(const std::string& image_path);

  ~Calibration();

  std::vector<std::string> get_images_list(const int& max_length = 10);

  void change_image_path(const std::string& image_path);

  std::vector<std::vector<cv::Point2f>> get_chessis_board_corners();

  std::vector<cv::Point3f> set_object_corners(const float& delat_x = 100.0f,
                                              const float& delat_y = 100.0f);

  void set_board_size(const int& point1 = 8,
                      const int& point2 = 6);

  double camera_calibrate();

  void image_undistort();

  void show_parameters();

  void save_parameters(const std::string& path,
                       const std::string& name);

  double get_project_error();

private:

  std::shared_ptr<std::string> _chessis_board_image_path;

  std::shared_ptr<std::vector<std::string>> _image_list;

  std::shared_ptr<cv::Size> _board_size;

  std::shared_ptr<cv::Size> _image_size;

  std::shared_ptr<std::vector<std::vector<cv::Point2f>>> _image_corners;

  std::shared_ptr<std::vector<std::vector<cv::Point3f>>> _object_corners;

  std::shared_ptr<cv::FileStorage> _parameter_write;

  cv::Mat _camera_K_matrix;

  cv::Mat _distort_coeff;

  std::vector<cv::Mat> _R_list, _T_list;

  bool _must_undistort;

};


}



#endif
