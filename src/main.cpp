#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>

#include "utils.hpp"

namespace fs = std::filesystem;
namespace po = boost::program_options;
const cv::Size CHESSBOARD_SIZE = cv::Size(6, 9);
constexpr float SQUARE_DIMENSIONS = 2.635e-2f; //meters


int main(int argc, char *argv[]){

	try{
		std::string path, out, conf;
		po::options_description opt("CameraCalibration options");

		opt.add_options()
			("path,p", po::value<std::string>(&path), "path to images")
			("conf,c", po::value<std::string>(&conf), "configuration file")
			("out,o", po::value<std::string>(&out), "out camera parameters in .yml")
			("help,h", "produce help message")
			;

		po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(opt).run(), vm);
		po::notify(vm);

		if(vm.count("help")){
			std::cout << opt << std::endl;
			return 1;
		}

		if(path.empty()){
			std::cout << "No image path given!\n";
			return 1;
		}
		else{

			YAML::Node cameraParams = createCameraCalibrationSchema();
			/* todo : functionalize*/
			/* cv::Mat image, canvass; */

			cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			cv::Mat distortionParams = cv::Mat::zeros(14, 1, CV_64F);
			cv::Mat stdDevDistortionParams, stdDeviationExtrinsics, viewError;
	    std::vector<cv::Mat> rVectors, tVectors;

			std::vector<std::vector<cv::Point2f>> allCrnrs;

			/* cv::namedWindow("Calibration Images", cv::WINDOW_NORMAL); */
			std::vector<cv::Point2f> foundPoints;
			std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints;


			/* todo check this! */
			cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001);

			bool found = false;
			int i = 0, added = 0;


			for(const auto& en : fs::directory_iterator(path)){

				cv::Mat image = cv::imread(en.path(), cv::IMREAD_GRAYSCALE);
				std::vector<cv::Point3f> worldCoords;

				if(i == 0){
					cameraParams["Camera.widthPix"] = image.size[1]/2;
					cameraParams["Camera.heightPix"] = image.size[0]/2;
				}

				cv::resize(image, image, cv::Size(image.size[1]/2, 
						image.size[0]/2), cv::INTER_AREA);


				found = findChessboardCorners(image, CHESSBOARD_SIZE, foundPoints,
						cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

				if(found){

					cv::cornerSubPix(image, foundPoints, cv::Size(11, 11), cv::Size(-1, -1), criteria);

					allCrnrs.push_back(foundPoints);
					createKnownBoardDim(CHESSBOARD_SIZE, SQUARE_DIMENSIONS, worldCoords);
					worldSpaceCornerPoints.push_back(worldCoords);
					added++;

					std::cout << "size wp " << added 
						<< " " << worldSpaceCornerPoints[added - 1].size() << std::endl;
					std::cout << "size pb " << added 
						<< " " << allCrnrs[added - 1].size() << std::endl;

				}
				else{
					std::cout << "Unable to find points in image : " << en.path().filename() << std::endl;
				}

				std::cout << i++ << std::endl;
			}

			/* worldSpaceCornerPoints.resize(allCrnrs.size(), worldSpaceCornerPoints[0]); */

			std::cout << "size wp " << worldSpaceCornerPoints.size() << std::endl;
			std::cout << "size pb " << allCrnrs.size() << std::endl;

			std::cout << "size wp " << worldSpaceCornerPoints.at(0).size() << std::endl;
			std::cout << "size pb " << allCrnrs.at(0).size() << std::endl;

			std::cout << "Starting calibration!" << std::endl;

			int flags = cv::CALIB_RATIONAL_MODEL | 
				cv::CALIB_THIN_PRISM_MODEL | cv::CALIB_TILTED_MODEL;

			calibrateCamera(worldSpaceCornerPoints, allCrnrs, 
					CHESSBOARD_SIZE, cameraMatrix, distortionParams, rVectors, tVectors,
					stdDevDistortionParams, stdDeviationExtrinsics, viewError, flags
					);

			cameraParams["Camera.fx"] = cameraMatrix.at<double>(0,0);
			cameraParams["Camera.fy"] = cameraMatrix.at<double>(1,1);
			cameraParams["Camera.cx"] = cameraMatrix.at<double>(0,2);
			cameraParams["Camera.cy"] = cameraMatrix.at<double>(1,2);

			cameraParams["Camera.k1"] = distortionParams.at<double>(0,0);
			cameraParams["Camera.k2"] = distortionParams.at<double>(1,0);
			cameraParams["Camera.p1"] = distortionParams.at<double>(2,0);
			cameraParams["Camera.p2"] = distortionParams.at<double>(3,0);
			cameraParams["Camera.k3"] = distortionParams.at<double>(4,0);
			cameraParams["Camera.k4"] = distortionParams.at<double>(5,0);
			cameraParams["Camera.k5"] = distortionParams.at<double>(6,0);
			cameraParams["Camera.k6"] = distortionParams.at<double>(7,0);
			cameraParams["Camera.s1"] = distortionParams.at<double>(8,0);
			cameraParams["Camera.s2"] = distortionParams.at<double>(9,0);
			cameraParams["Camera.s3"] = distortionParams.at<double>(10,0);
			cameraParams["Camera.s4"] = distortionParams.at<double>(11,0);
			cameraParams["Camera.taox"] = distortionParams.at<double>(12,0);
			cameraParams["Camera.taoy"] = distortionParams.at<double>(13,0);

			std::cout << "Calibration finished" << std::endl;



			if(vm.count("out")){
				std::ofstream fout(out);
				fout << cameraParams;
				fout.close();
			}
		}
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << "\n";
		return 1;
	}    
	return 0;
}
