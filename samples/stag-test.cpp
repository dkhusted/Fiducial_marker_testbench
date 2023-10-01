#include "../models/stag/src/Stag.h"
#include "opencv2/opencv.hpp"

int main() {
  cv::Mat image = cv::imread("./images/STag-img/1.png", cv::IMREAD_GRAYSCALE);

  cv::imshow("img", image);

  Stag stag(15, 7, true);
  int marker_Cnt = stag.detectMarkers(image);
  printf("Marker ID: %d\n", stag.markers[0].id);
  
  // cv::drawContours(image, stag.getContours(), -1, cv::Scalar(255, 0, 0), 2);
  // stag.logResults("./app/STag_results/");
  cv::waitKey(0);
  return 0;
}