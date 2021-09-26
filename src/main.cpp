#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
/* #include <opencv2/aruco.hpp> */
#include <opencv2/calib3d.hpp>


#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>


namespace fs = std::filesystem;

constexpr float SQUARE_DIMENSIONS = 2.635e-2f; //meters
/* constexpr float ARUCO_DIMENSIONS = */ 
const cv::Size CHESSBOARD_SIZE = cv::Size(6, 9);

/* std::string imDirr = "imgs/" */

void createKnownBoardDim(cv::Size brdSize, float sqrEdgeLength, std::vector<cv::Point3f> &corners)
{
	for(int i = 0; i < brdSize.height; i++)
		for(int j = 0; j < brdSize.width; j++)
			corners.push_back(cv::Point3f(j * sqrEdgeLength, i * sqrEdgeLength, 0.0));
}

void getChessboardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f>> &allCrnrs, bool showResult = false)
{
	std::vector<cv::Point2f> pointBuf;
	for(std::vector<cv::Mat>::iterator iter = images.begin(); 
			iter != images.end(); iter++){
		bool found = findChessboardCorners(*iter, cv::Size(9, 6), pointBuf, 
				cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

		if(found){
				allCrnrs.push_back(pointBuf);
		}
		if(showResult){
			drawChessboardCorners(*iter, cv::Size(9, 6), pointBuf, found);
			cv::imshow("Looking for Corners", *iter);
			cv::waitKey(0);
		}
	}
}

void cameraCalibration(std::vector<cv::Mat> calibrationImages, cv::Size boardSize, float squaredLength, cv::Mat& cameraMatrix)
{
	std::vector<std::vector<cv::Point2f>> checkerboardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);
	std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);

	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
	

}

int main(int argc, char *argv[]){
	cv::Mat image, canvass;

	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distCoefficents;

	std::vector<cv::Mat> savedImages;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

	std::string imDirr(argv[1]);
	std::cout << imDirr << std::endl;

	cv::namedWindow("Calibration Images", cv::WINDOW_NORMAL);
	std::vector<cv::Vec2f> foundPoints;
	bool found = false;

	int i = 0;
	for(const auto& en : fs::directory_iterator(imDirr)){
		std::cout << i++ << std::endl;
		if(en.path().extension() == ".JPG"){
			/* image = cv::imread(en.path(), cv::IMREAD_GRAYSCALE); */
			image = cv::imread(en.path());
			cv::resize(image, image, cv::Size(image.size[1] / 4, image.size[0] / 4), cv::INTER_AREA);

			found = findChessboardCorners(image, CHESSBOARD_SIZE, foundPoints,
					cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

			if(found){
				image.copyTo(canvass);
				drawChessboardCorners(canvass, CHESSBOARD_SIZE, foundPoints, found);
				cv::imshow("Calibration Images", canvass);
			}
			else{
				image.copyTo(canvass);
				cv::imshow("Calibration Images", canvass);
			}
		}
		else{
			continue;
		}

		char exitStat = cv::waitKey(200);

	}

  return 1;
}
