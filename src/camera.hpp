#ifndef CAMERACONTAINER_HPP_BZC17YU2
#define CAMERACONTAINER_HPP_BZC17YU2

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

using vecp2f = std::vector<cv::Point2f>;
using vecp3f = std::vector<cv::Point3f>;

enum {
	C_CIRCLES = 0,
	C_CHESS = 1,
	C_SB_CHESS = 2
} PointType;

enum {
	REGULAR = 0,
	RO = 1
} CalibType;

class CalibrationConfig{
	public:
		CalibrationConfig() = delete;
		CalibrationConfig(YAML::Node config);

		CalibrationConfig(const CalibrationConfig &other) = delete;
		CalibrationConfig &operator=(const CalibrationConfig &other) = delete;
		CalibrationConfig(CalibrationConfig &&other) = delete;
		CalibrationConfig  &operator=(CalibrationConfig &&other) = delete;

		~CalibrationConfig() = default;

		std::function<bool(const cv::Mat &im, vec2pf &foundPoints)> findPoints;

		int flags(){return operationFlags;}
		int fixedPoint(){return fp;}
		CalibType calibType(){return ct;}
		PointType pointType(){return pt;}
		float dim(){return dimension;}

		// handle this?
		cv::Size patternSize(){return patternSize;}

	private:

		int operationFlags;
		int pointFlags;
		PointType pt;
		CalibType ct;
		int fp;
		cv::Size patternSize;

		float dimension; // meters in object of interest (cricles, chessboards, and other)
		cv::TermCriteria criteria;

};


class Camera {

	public:
		const cv::Mat& getIntrinsics() const 
		{return intrinsics;} 
		const cv::Mat& getDistortionParams() const
		{return distortionParams;}

		bool isPrismaModel() const {return thinPrismaModel;}
		bool isRationalModel() const {return rationalModel;}
		bool isTilted() const {return tiltedModel;}
		bool isCalibrated() const {return calibrated;}

		double aspectRatio() const {return aspectRatio;}
		double focalLength() const {return focalLength;}

		double sensorWidth() const {return sensorWidth;}
		double sensorHeight() const {return sensorHeight;}

		double pixWidth() const {return pixWidht;}
		double pixHeight() const {return pixHeight;}

		void setPixWidth(double sizeX){pixWidht = sizeX;}
		void setPixHeight(double sizeY){pixHeight = sizeY;}

		cv::Point2f getPP() const {return principalPoint;}

		Camera(const std::string &name) noexcept;
		Camera(YAML::Node inpt);

		Camera(Camera &&other) = default;
		Camera &operator=(Camera &&other) = default;
		Camera(const Camera &other) = default;
		Camera &operator=(const Camera &other) = default;

		~Camera() = default;


		bool write(const std::string &output);

		double calibrateCamera(const std::vector<vecp3f> &worldPoints,
				const std::vector<vecp2f> &imagePoints,
				CalibrationConfig calibConf);

		cv::Mat undistortImage(const cv::Mat &input) const;
		vecp2f undistortPoints(const vecp2f &input) const;

	private:

		cv::Mat intrinsics;
		cv::Mat distortionParams;

		// TODO : Document these
		
		struct {
			cv::Mat stdDevIntrinsics; 
			cv::Mat stdDeviationExtrinsics; 
			cv::Mat viewError;
			std::vector<cv::Mat> rVectors;
			std::vector<cv::Mat>tVectors;
		} CalibrationStat;


		bool thinPrismaModel;
		bool rationalModel;
		bool tiltedModel;

		std::string name;

		bool calibrated;

		double aspectRatio;
		double focalLength;

		double sensorWidth;
		double sensorHeight;

		double pixWidht;
		double pixHeight;

		cv::Point2d principalPoint;
};

#endif /* end of include guard: CAMERACONTAINER_HPP_BZC17YU2 */
