#include "../3rdparty/ORB_SLAM2_modified/include/ORBextractor.h"

#include <iostream>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(){
  std::string file_name = "//home//zjht//ws//learn-slam//learn_it//images.jpg";
  cv::Mat image = cv::imread(file_name);
  if (! image.data){
    std::cout<<"Could not open or find image.\n";
    return -1;
  }
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
  imshow("image", image);
  cv::waitKey(0);


  int features = 1000;
  float scale = 1.2;
  int level = 8;
  int ini = 20;
  int min = 7;
  ORB_SLAM2::ORBextractor extractor(features, scale, level, ini, min);

  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  std::vector<cv::KeyPoint> kps;
  cv::Mat desps;
  (extractor)( gray, cv::Mat(), kps, desps);

  for (auto it = kps.begin(); it != kps.end(); ++it){
    int i = 0;
    cv::circle(gray, (*it).pt, 1, cv::Scalar( 110, 220, 0 ),  2, 8);
  }
  imshow("image", gray);
  cv::waitKey(0);
  return 0;
}
