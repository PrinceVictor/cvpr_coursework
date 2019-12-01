// Copyright 2019 Hongbin Zhou
// License
// Author: Hongbin Zhou

#ifndef __CALIB_H__
#define __CALIB_H__

#include <iostream>
#include <vector>
#include <cstring>
#include <memory>

namespace CALIB {

class Calibration{

public:

  Calibration();

  Calibration(const std::string image_path);

  ~Calibration();

  void get_images(const int max_length = 10);

  void change_image_path(const std::string image_path);

private:

  std::shared_ptr<std::string> chessis_board_image_path;

  std::shared_ptr<std::vector<std::string>> image_list;

  std::string a;

};


}



#endif
