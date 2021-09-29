#ifndef UTILS_HPP_NPVQH3EA
#define UTILS_HPP_NPVQH3EA

void createKnownBoardDim(cv::Size brdSize, 
		float sqrEdgeLength, std::vector<cv::Point3f> &corners);

/* void getChessboardCorners(std::vector<cv::Mat> images, */ 
/* 		std::vector<std::vector<cv::Point2f>> &allCrnrs, bool showResult); */

/* void cameraCalibration(std::vector<cv::Mat> calibrationImages, cv::Size boardSize, */ 
/* 		float squaredLength, cv::Mat& cameraMatrix, cv::Mat &distortionParams, */
/* 		cv::Mat &stdDevDistortionParams, cv::Mat &stdDeviationExtrinsics, cv::Mat &viewError); */

#endif /* end of include guard: UTILS_HPP_NPVQH3EA */
