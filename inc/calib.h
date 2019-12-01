#ifndef __CALIB_H__
#define __CALIB_H__

#include <iostream>

namespace CALIB {

void get_images(const std::string image_path,
                std::vector<std::string>& image_list,
                const int max_length);

}



#endif
