#include <iostream>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <filesystem>


namespace fs = std::filesystem;

/* small bin to generate 10 markers from 6x6*/

constexpr int a3Width300DPI = 3508;

int main(int argc, char *argv[])
{

  if(argc < 2){
    std::cout << "Please provide folder" << std::endl;
    std::cout << "Usage ./bin/aruco <folder>" << std::endl;
    return EXIT_FAILURE;
  }
  std::string out(argv[1]);

	assert(fs::is_directory(out));
  fs::path outp{out};

  cv::Mat markerImage;
  cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  for(int i = 0; i < 10; ++i){
    fs::path cim = outp / (std::to_string(i) + ".png");
    cv::aruco::generateImageMarker(dict, i, a3Width300DPI, markerImage);
    if(!fs::exists(cim)){
      cv::imwrite(cim.string(), markerImage);
    }
    else{
      std::cout << cim << " already exists!\n";
    }
  }

  return EXIT_SUCCESS;
}
