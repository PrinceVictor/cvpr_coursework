#include "klt.h"
#include "log.h"

using klttracking::KLT;

int main(int argc, char **argv)
{

  std::string video_path = "../video/100208AA_out.mp4";

  KLT* klt = new KLT(video_path);

  klt->get_frist_frame_feature();

  klt->klt_pipline();

  return 0;
}

