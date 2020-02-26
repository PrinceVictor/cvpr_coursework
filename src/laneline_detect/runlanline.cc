#include "log.h"
#include "laneline.h"

using namespace laneline;

int main(int argc, char **argv){

  LaneDetector lanedetector;  // 创建类对象
  lanedetector.doDetection("../lane_image/");
}
