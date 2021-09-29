#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "utils.hpp"
#include <iostream>

void createKnownBoardDim(cv::Size brdSize, float sqrEdgeLength, std::vector<cv::Point3f> &corners)
{
	for(int i = 0; i < brdSize.height; i++){
		for(int j = 0; j < brdSize.width; j++){
			corners.push_back(cv::Point3f(j * sqrEdgeLength, i * sqrEdgeLength, 0.0));
		}
	}
}

/* void getChessboardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f>> &allCrnrs, bool showResult = false) */
/* { */
/* 	std::vector<cv::Point2f> pointBuf; */
/* 	for(std::vector<cv::Mat>::iterator iter = images.begin(); */ 
/* 			iter != images.end(); iter++){ */
/* 		bool found = findChessboardCorners(*iter, cv::Size(9, 6), pointBuf, */ 
/* 				cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE); */

/* 		if(found){ */
/* 				allCrnrs.push_back(pointBuf); */
/* 		} */
/* 		if(showResult){ */
/* 			drawChessboardCorners(*iter, cv::Size(9, 6), pointBuf, found); */
/* 			cv::imshow("Looking for Corners", *iter); */
/* 			cv::waitKey(0); */
/* 		} */
/* 	} */
/* } */

/* void cameraCalibration(std::vector<cv::Mat> calibrationImages, cv::Size boardSize, float squaredLength, cv::Mat& cameraMatrix, cv::Mat &distortionParams, cv::Mat &stdDevDistortionParams, cv::Mat &stdDeviationExtrinsics, cv::Mat &viewError) */
/* { */
/* 	std::vector<std::vector<cv::Point2f>> checkerboardImageSpacePoints; */
/* 	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false); */
/* 	std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1); */

/* 	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]); */

/* 	std::vector<cv::Mat> rVectors, tVectors; */
/* 	distortionParams = cv::Mat::zeros(14, 1, CV_64F); */

/* 	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, */ 
/* 			boardSize, cameraMatrix, ditortionParams, rVectors, tVecotrs, */
/* 			stdDevDistortionParams, stdDeviationExtrinsics, viewError */
/* 			); */

/* } */
