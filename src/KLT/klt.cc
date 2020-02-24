#include "log.h"
#include "klt.h"

void draw_points(cv::Mat& frame,
                 std::vector<cv::Point2f>& feature_points,
                 cv::Scalar& color){

  for(int i=0 ; i<feature_points.size() ; i++){
    cv::circle(frame,
               feature_points[i],
               2,
               cv::Scalar(0, 0, 255),
               2,
               8);
  }
}

void draw_lines(cv::Mat& frame,
                std::vector<cv::Point2f>& start_points,
                std::vector<cv::Point2f>& predict_points){

  for (int i = 0; i < predict_points.size(); i++)
  {
    cv::line(frame, start_points[i], predict_points[i], cv::Scalar(0, 255, 0), 2, 8);
    cv::circle(frame, start_points[i], 2, cv::Scalar(0, 0, 255), 2, 8);
  }

}

void draw_arrows(cv::Mat& frame,
                std::vector<cv::Point2f>& start_points,
                std::vector<cv::Point2f>& end_points){

  for (int i = 0; i < end_points.size(); i++)
  {

    const double PI = 3.1415926;
    double alpha = 15, len =10;

    cv::Point2f arrow, pStart = start_points[i], pEnd = end_points[i];

    double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
    line(frame, pStart, pEnd, cv::Scalar(255, 0, 0), 2, 8);
    arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
    arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
    line(frame, pEnd, arrow, cv::Scalar(255, 0, 0), 2, 8);
    arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
    arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
    cv::line(frame, pEnd, arrow, cv::Scalar(255, 0, 0), 2, 8);
  }

}

namespace klttracking {

KLT::KLT(){

  //default open camera
  capture_stream_ = std::make_shared<cv::VideoCapture>(0);

  if(!capture_stream_->isOpened()){

    LOG_OUTPUT("open default camera failed!!");
    return;
  }

  LOG_OUTPUT("success open default camera");

}

KLT::KLT(const std::string &video_path){

  capture_stream_ = std::make_shared<cv::VideoCapture>(video_path);

  if(!capture_stream_->isOpened()){

    LOG_OUTPUT("open video failed!!!"
               "\n --------  from path : {}", video_path.c_str());
    return ;
  }

  LOG_OUTPUT("open video success!!!"
             "\n --------  from path : {}", video_path.c_str());

}

KLT::~KLT(){

  capture_stream_->release();
}

bool KLT::get_one_frame(){

  capture_stream_->read(*current_frame_);

  cv::cvtColor(*current_frame_, *current_gray_, cv::COLOR_BGR2GRAY);

  if(current_frame_->empty()){
    return false;
  }
  else{
    return true;
  }

}

void KLT::get_corner_points(std::vector<cv::Point2f>& corner_points){

  double maxCorners = 5000;
  double qualityLevel = 0.01;
  double minDistance = 10; // 小于这个就属于同一个特征点
  double blockSize = 3;

  cv::goodFeaturesToTrack(*current_gray_,
                          corner_points,
                          maxCorners,
                          qualityLevel,
                          minDistance,
                          cv::Mat(),
                          blockSize,
                          false, 0.04);

  LOG_OUTPUT("Get the {} corner points", corner_points.size());

}

void KLT::update_frame(){

  current_frame_->copyTo(*previous_frame_);
  current_gray_->copyTo(*previous_gray_);

}


void KLT::get_frist_frame_feature(){

  current_frame_ = std::make_shared<cv::Mat>();
  current_gray_ = std::make_shared<cv::Mat>();

  previous_frame_ = std::make_shared<cv::Mat>();
  previous_gray_ = std::make_shared<cv::Mat>();

  tracked_points_ = std::make_shared<std::vector<cv::Point2f>>();
  start_points_ = std::make_shared<std::vector<cv::Point2f>>();

  get_one_frame();

  get_corner_points(*tracked_points_);

  LOG_OUTPUT("track points {}", tracked_points_->size());

  *start_points_ = *tracked_points_;

  update_frame();
}

void KLT::recreate_track_points(){


    std::vector<cv::Point2f> temp_points;

    get_corner_points(temp_points);

    tracked_points_->insert(tracked_points_->end(),
                            temp_points.begin(),
                            temp_points.end());

    start_points_->insert(start_points_->end(),
                          temp_points.begin(),
                          temp_points.end());

    LOG_OUTPUT("tracked points less than 80, re creat corner points!");

}

void KLT::klt_tracking(){

  std::vector<cv::Point2f> predict_points;

  std::vector<uchar> status;
  std::vector<float> errors;

  cv::calcOpticalFlowPyrLK(*previous_gray_,
                           *current_gray_,
                           *tracked_points_,
                           predict_points,
                           status,
                           errors);

  int k = 0;

  for( int i=0; i<predict_points.size(); i++){

    double dist = std::abs((*tracked_points_)[i].x - predict_points[i].x) +
        std::abs((*tracked_points_)[i].y - predict_points[i].y);

    if (dist > 2 && status[i])
    {
      (*start_points_)[k] = (*start_points_)[i];
      (*tracked_points_)[k] = (*tracked_points_)[i];
      predict_points[k++] = predict_points[i];
    }

  }

  start_points_->resize(k);
  predict_points.resize(k);

  draw_lines(*current_frame_, *start_points_, *tracked_points_);
  draw_arrows(*current_frame_, *tracked_points_, predict_points);

  std::swap(*tracked_points_, predict_points);

}

void KLT::klt_pipline(){

//  cv::Size frame_size = cv::Size(current_frame_->cols, current_frame_->rows);
//  cv::VideoWriter write;
//  write.open("/home/victor/cvpr_course/code/cvpr_coursework/video/result.mp4",
//             cv::VideoWriter::fourcc('m','j','p','g'), 25, frame_size);

  while(get_one_frame()){

    if(tracked_points_->size() < 80){

      recreate_track_points();
      LOG_OUTPUT("track points {}", tracked_points_->size());
    }

    klt_tracking();

    //  cv::namedWindow("klt_tracking", cv::WINDOW_NORMAL);
    cv::imshow("klt_tracking", *current_frame_);
//    write.write(*current_frame_);

    update_frame();


    if (27 == cv::waitKey(30)){

//      write.release();
      break;
    }
  }

}


}

