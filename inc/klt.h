// Copyright 202 Hongbin Zhou
// License
// Author: Hongbin Zhou

#ifndef __KLT_H__
#define __KLT_H__

#include <iostream>
#include <vector>
#include <cstring>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

namespace klttracking {

class KLT{

public:

  KLT();

  KLT(const std::string& video_path);

  ~KLT();

  void klt_pipline();

  void get_frist_frame_feature();

private:

  void klt_tracking();

  void recreate_track_points();

  void get_corner_points(std::vector<cv::Point2f>& corner_points);

  void update_frame();

  bool get_one_frame();

  std::shared_ptr<cv::Mat> current_frame_;
  std::shared_ptr<cv::Mat> current_gray_;

  std::shared_ptr<cv::Mat> previous_frame_;
  std::shared_ptr<cv::Mat> previous_gray_;

  std::shared_ptr<std::vector<cv::Point2f>> tracked_points_;
  std::shared_ptr<std::vector<cv::Point2f>> start_points_;

  std::shared_ptr<cv::VideoCapture> capture_stream_;


};

}
#endif

