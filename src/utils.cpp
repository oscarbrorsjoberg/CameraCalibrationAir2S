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

