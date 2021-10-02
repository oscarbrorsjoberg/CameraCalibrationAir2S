#ifndef CAMERACONTAINER_HPP_BZC17YU2
#define CAMERACONTAINER_HPP_BZC17YU2

class CalibrationConfig{
	public:
		CalibrationConfig();
		~CalibrationConfig();

	private:
		int operationFlags;

};

class CameraContainer{

	public:
		const cv::Mat& getIntrinsics() const 
		{return intrinsics;} 
		const cv::Mat& getDistortionParams() const
		{return distortionParams;}

		bool isPrismaModel(){return thinPrismaModel;}
		bool isRationalModel(){return rationalModel;}
		bool isTilted(){return tiltedModel;}

		double getAspectRatio(){return aspectRatio;}
		double getFocalLength(){return focalLength;}

		cv::Point2f getPP(){return principalPoint;}

		CameraContainer();

	CameraContainer(YAML::Node inpt);

	void dumpCameraContainer();
	void calibrateCamera(const std::vector<std::vector<cv::Point3f>> &worldPoints,
										   const std::vector<std::vector<cv::Point2f>> &imagePoints,
											 CalibrationConfig calibConf);

	cv::Mat undistortImage(const cv::Mat &input) const;

	std::vector<cv::Point2f> 
		undistortPoints(const std::vector<cv::Point2f> &input) const;

	private:
		cv::Mat intrinsics;
		cv::Mat distortionParams;

		bool thinPrismaModel;
		bool rationalModel;
		bool tiltedModel;
		double aspectRatio;
		double focalLength;
		cv::Point2d principalPoint;

};

YAML::Node createCameraCalibrationSchema();


#endif /* end of include guard: CAMERACONTAINER_HPP_BZC17YU2 */
