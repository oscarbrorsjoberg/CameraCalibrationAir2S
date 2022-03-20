#ifndef CAMERACONTAINER_HPP_BZC17YU2
#define CAMERACONTAINER_HPP_BZC17YU2

#include <vector>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

using vecp2f = std::vector<cv::Point2f>;
using vecp3f = std::vector<cv::Point3f>;

typedef enum {
	C_CIRCLES = 0,
	C_CHESS = 1,
	C_SB_CHESS = 2
} PointType;

typedef enum {
	REGULAR = 0,
	RO = 1
} CalibType;

class CalibrationConfig{
	public:
		CalibrationConfig() = delete;

		CalibrationConfig(const YAML::Node &config);

		CalibrationConfig(const CalibrationConfig &other) = delete;
		CalibrationConfig &operator=(const CalibrationConfig &other) = delete;
		CalibrationConfig(CalibrationConfig &&other) = delete;
		CalibrationConfig  &operator=(CalibrationConfig &&other) = delete;

		~CalibrationConfig() = default;

		std::function<bool(const cv::Mat &im, vecp2f &foundPoints)> findPoints;


		int oflags() const {return operationFlags;}
		int pflags() const {return pointFlags;}

		int fixedPoint() const {return fp;}
		CalibType calibType() const {return ct;}
		PointType pointType() const {return pt;}
		float dim() const {return dimension;}

		cv::TermCriteria criteria() const {return crit;}

		// handle this?
		cv::Size patternSize(){return ps;}

	private:

		int operationFlags;
		int pointFlags;
		PointType pt;
		CalibType ct;
		int fp;
		cv::Size ps;

		float dimension; // meters in object of interest (cricles, chessboards, and other)
		cv::TermCriteria crit;

};


class Camera {

	public:
		const cv::Mat& getIntrinsics() const 
		{return intrinsics;} 
		const cv::Mat& getDistortionParams() const
		{return distortionParams;}

		bool isPrismaModel() const {return thinPrismaModel_;}
		bool isRationalModel() const {return rationalModel_;}
		bool isTilted() const {return tiltedModel_;}
		bool isCalibrated() const {return calibrated_;}
		double aspectRatio() const {return aspectRatio_;}
		double focalLength() const {return focalLength_;}
		double sensorWidth() const {return sensorWidth_;}
		double sensorHeight() const {return sensorHeight_;}
		double pixWidth() const {return pixWidth_;}
		double pixHeight() const {return pixHeight_;}
		void setPixWidth(double sizeX){pixWidth_ = sizeX;}
		void setPixHeight(double sizeY){pixHeight_ = sizeY;}

		cv::Point2f getPP() const {return principalPoint_;}

		Camera(const std::string &name);
		/* Camera(): */
		/* { */
		/* } */

		Camera(YAML::Node inpt);

		Camera(Camera &&other) = default;
		Camera &operator=(Camera &&other) = default;
		Camera(const Camera &other) = default;
		Camera &operator=(const Camera &other) = default;

		~Camera() = default;


		bool write(const std::string &output);

		double calibrate(const std::vector<vecp3f> &worldPoints,
				const std::vector<vecp2f> &imagePoints,
				const CalibrationConfig &calibConf);

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


		bool thinPrismaModel_;
		bool rationalModel_;
		bool tiltedModel_;
		std::string name_;
		bool calibrated_;
		double aspectRatio_;
		double focalLength_;
		double sensorWidth_;
		double sensorHeight_;
		double pixWidth_;
		double pixHeight_;
		cv::Point2d principalPoint_;
};

#endif /* end of include guard: CAMERACONTAINER_HPP_BZC17YU2 */
