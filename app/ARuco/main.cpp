#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

/* bool read_cmd_line(std::string &calib, */ 
/* 									std::string &image) */


int main(int argc, char *argv[])
{
  cv::Mat markerImage;

  cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  cv::aruco::generateImageMarker(dict, 23, 200, markerImage, 1);

  cv::imwrite("marker23.png", markerImage);

  return EXIT_SUCCESS;
}
