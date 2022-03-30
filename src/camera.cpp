#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ctime>
#include <chrono>
#include <vector>
#include <array>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>


#include "camera.hpp"

using chsys = std::chrono::system_clock;

constexpr int NUMBR_TRANSFORM_STDDEV = 6;

const std::string transExDesc[] = {"R0", "R1", "R2", "T0", "T1", "T2"};
const std::string distDesc[] = {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "k4", 
				"k5", "k6", "s1", "s2", "s3", "s4", "taox", "taoy"};


static std::map<std::string, int> calibrationFlags_m;
static std::map<std::string, int> pointFlagsChess_m;
static std::map<std::string, int> pointFlagsCircle_m;


static const std::array< 
								std::tuple<std::string, int>, 15
								> posCalibrationFlags {
// cameraMatrix contains valid initial values of fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image center ( imageSize is used), and focal distances are computed in a least-squares fashion. Note, that if intrinsic parameters are known, there is no need to use this function just to estimate extrinsic parameters. Use solvePnP instead.
std::make_tuple("cv::CALIB_USE_INTRINSIC_GUESS", cv::CALIB_USE_INTRINSIC_GUESS),

// The principal point is not changed during the global optimization. It stays at the center or at a different location specified when CALIB_USE_INTRINSIC_GUESS is set too.
std::make_tuple("cv::CALIB_FIX_PRINCIPAL_POINT", cv::CALIB_FIX_PRINCIPAL_POINT),

// The functions consider only fy as a free parameter. The ratio fx/fy stays the same as in the input cameraMatrix . When CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are ignored, only their ratio is computed and used further.
std::make_tuple("cv::CALIB_FIX_ASPECT_RATIO", cv::CALIB_FIX_ASPECT_RATIO),
// Tangential distortion coefficients (p1,p2) are set to zeros and stay zero.
std::make_tuple("cv::CALIB_ZERO_TANGENT_DIST", cv::CALIB_ZERO_TANGENT_DIST),

// The corresponding radial distortion coefficient is not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
std::make_tuple("cv::CALIB_FIX_K1", cv::CALIB_FIX_K1),
std::make_tuple("cv::CALIB_FIX_K2", cv::CALIB_FIX_K2),
std::make_tuple("cv::CALIB_FIX_K3", cv::CALIB_FIX_K3),
std::make_tuple("cv::CALIB_FIX_K4", cv::CALIB_FIX_K4),
std::make_tuple("cv::CALIB_FIX_K5", cv::CALIB_FIX_K5),
std::make_tuple("cv::CALIB_FIX_K6", cv::CALIB_FIX_K6),
// Coefficients k4, k5, and k6 are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the rational model and return 8 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients.
std::make_tuple("cv::CALIB_RATIONAL_MODEL", cv::CALIB_RATIONAL_MODEL),

//Coefficients s1, s2, s3 and s4 are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the thin prism model and return 12 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients.

std::make_tuple("cv::CALIB_THIN_PRISM_MODEL", cv::CALIB_THIN_PRISM_MODEL),

//The thin prism distortion coefficients are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
	std::make_tuple("cv::CALIB_FIX_S1_S2_S3_S4", cv::CALIB_FIX_S1_S2_S3_S4),
//Coefficients tauX and tauY are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the tilted sensor model and return 14 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients.
std::make_tuple("cv::CALIB_TILTED_MODEL", cv::CALIB_TILTED_MODEL),
//The coefficients of the tilted sensor model are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
std::make_tuple("cv::CALIB_FIX_TAUX_TAUY", cv::CALIB_FIX_TAUX_TAUY)

};

static const std::array<std::tuple<std::string, int>, 8> posPointFlagsChess{
	// Use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness).
	std::make_tuple("cv::CALIB_CB_ADAPTIVE_THRESH", cv::CALIB_CB_ADAPTIVE_THRESH),

// Use additional criteria (like contour area, perimeter, square-like shape) to filter out false quads extracted at the contour retrieval stage.
	std::make_tuple("cv::CALIB_CB_FILTER_QUADS", cv::CALIB_CB_FILTER_QUADS),

// Run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed.
std::make_tuple("cv::CALIB_CB_FAST_CHECK", cv::CALIB_CB_FAST_CHECK),

// Normalize the image gamma with equalizeHist before detection.
std::make_tuple("cv::CALIB_CB_NORMALIZE_IMAGE", cv::CALIB_CB_NORMALIZE_IMAGE),

// Run an exhaustive search to improve detection rate.
std::make_tuple("cv::CALIB_CB_EXHAUSTIVE", cv::CALIB_CB_EXHAUSTIVE),

// Up sample input image to improve sub-pixel accuracy due to aliasing effects.
std::make_tuple("cv::CALIB_CB_ACCURACY", cv::CALIB_CB_ACCURACY),

// The detected pattern is allowed to be larger than patternSize (see description).
std::make_tuple("cv::CALIB_CB_LARGER", cv::CALIB_CB_LARGER),

// The detected pattern must have a marker (see description). This should be used if an accurate camera calibration is required.
std::make_tuple("cv::CALIB_CB_MARKER", cv::CALIB_CB_MARKER),

};

static const std::array<std::tuple<std::string, int>, 3> posPointFlagsCircle{
	// uses symmetric pattern of circles.
	std::make_tuple("cv::CALIB_CB_SYMMETRIC_GRID", cv::CALIB_CB_SYMMETRIC_GRID),

	// uses asymmetric pattern of circles.
	std::make_tuple("cv::CALIB_CB_ASYMMETRIC_GRID", cv::CALIB_CB_ASYMMETRIC_GRID),

	// uses a special algorithm for grid detection. 
	// It is more robust to perspective distortions but much 
	// more sensitive to background clutter.
	std::make_tuple("cv::CALIB_CB_CLUSTERING", cv::CALIB_CB_CLUSTERING)
};



static void initFlagsMaps()
{

	for(const auto &[f, m] : posPointFlagsChess){
		pointFlagsChess_m[f] = m;
	}

	for(const auto &[f, m] : posPointFlagsCircle){
		pointFlagsCircle_m[f] = m;
	}

	for(const auto &[f, m] : posCalibrationFlags){
		calibrationFlags_m[f] = m;
	}

}


CalibrationConfig::CalibrationConfig(const YAML::Node &config):
	operationFlags(0),
	pointFlags(0),
	crit(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001)

{

	initFlagsMaps();

	std::string calibType = config["CalibrationType"].as<std::string>();
	std::string pointType = config["PointType"].as<std::string>();

	// does this work or does it have to be dynamic?
	std::array<int, 2> size = config["PatternSize"].as<std::array<int, 2>>();
	this->ps = cv::Size(size[0], size[1]);


	this->dimension = config["PatternDimensions"].as<float>();

	assert((!calibType.empty() && (calibType == "REGULAR" || calibType == "RO")
				, "Calib Type is required!"));
	
	assert((!pointType.empty() && 
			(pointType == "CIRCLE" || pointType == "CHESS" 
			 || pointType == "SB_CHESS"), "Point Type is required!"));


	for(const auto &fl : config["CalibrationFlags"].as<std::vector<std::string>>())
		this->operationFlags |= calibrationFlags_m[fl];

	for(const auto &fl : config["PointFlags"].as<std::vector<std::string>>())
		this->pointFlags |= 
          pointType == "CIRCLE" ? pointFlagsCircle_m[fl] : pointFlagsChess_m[fl];


	if(pointType == "CIRCLE"){

		using F = bool(*)(
				cv::InputArray,
				cv::Size,
				cv::OutputArray,
				int,
				const cv::Ptr<cv::FeatureDetector>&
				);

		findPoints = std::bind(static_cast<F>(cv::findCirclesGrid), 
				std::placeholders::_1, 
				this->ps, 
				std::placeholders::_2, 
				this->pointFlags,
				cv::SimpleBlobDetector::create()
				);

		pt = PointType::C_CIRCLES;
	}
	else if(pointType == "CHESS"){

		findPoints = std::bind(cv::findChessboardCorners, 
				std::placeholders::_1, 
				this->ps, 
				std::placeholders::_2, 
				this->pointFlags
				);

		pt = PointType::C_CHESS;
	}
	else if(pointType == "SB_CHESS"){

		using G = bool(*)(
				cv::InputArray,
				cv::Size,
				cv::OutputArray,
				int
				);

		findPoints = std::bind(static_cast<G>(cv::findChessboardCornersSB),
				std::placeholders::_1, 
				this->ps, 
				std::placeholders::_2, 
				this->pointFlags
				);

		pt = PointType::C_SB_CHESS;

	}
	else{
		throw std::runtime_error(pointType + " is not a valid point type!\n");
	}

	if(calibType == "REGULAR"){
		ct = CalibType::REGULAR;
	}
	else if(calibType == "RO"){
		ct = CalibType::RO;
	}
	else{
		throw std::runtime_error(calibType + " is not a valid calibration type!\n");
	}


}


static std::string loadCameraSchema(const std::string &name)
{
	std::string cmra = "Camera.name: " + name + "\n";

	cmra +=	"Camera.fx: 0.0\n";
	cmra += "Camera.fy: 0.0\n";
	cmra += "Camera.cx: 0.0\n";
	cmra += "Camera.cy: 0.0\n";

	/* distortion parameters*/
	cmra += "Camera.k1: 0.0\n";
	cmra += "Camera.k2: 0.0\n";
	cmra += "Camera.p1: 0.0\n";
	cmra += "Camera.p2: 0.0\n";
	cmra += "Camera.k3: 0.0\n";
	cmra += "Camera.k4: 0.0\n";
	cmra += "Camera.k5: 0.0\n";
	cmra += "Camera.k6: 0.0\n";
	cmra += "Camera.s1: 0.0\n";
	cmra += "Camera.s2: 0.0\n";
	cmra += "Camera.s3: 0.0\n";
	cmra += "Camera.s4: 0.0\n";
	cmra += "Camera.taox: 0.0\n";
	cmra += "Camera.taoy: 0.0\n";

	/* image sizes */
	cmra += "Camera.widthPix: 0.0\n";
	cmra += "Camera.heightPix: 0.0\n";

	cmra += "FileInformation.DateOfCreation: 0.0\n";
	return cmra;
}


bool Camera::write(const std::string &output)
{
	std::ofstream fout(output);

	YAML::Node temp = YAML::Load(loadCameraSchema(name_));


	chsys::time_point p = chsys::now();
	time_t t = chsys::to_time_t(p);

	char str[26];
	ctime_r(&t, str);


	temp["Camera.fx"] = this->intrinsics.at<double>(0,0);
	temp["Camera.fy"] = this->intrinsics.at<double>(1,1);
	temp["Camera.cx"] = this->intrinsics.at<double>(0,2);
	temp["Camera.cy"] = this->intrinsics.at<double>(1,2);
	temp["Camera.k1"] = this->distortionParams.at<double>(0,0);
	temp["Camera.k2"] = this->distortionParams.at<double>(1,0);
	temp["Camera.p1"] = this->distortionParams.at<double>(2,0);
	temp["Camera.p2"] = this->distortionParams.at<double>(3,0);
	temp["Camera.k3"] = this->distortionParams.at<double>(4,0);
	temp["Camera.k4"] = this->distortionParams.at<double>(5,0);
	temp["Camera.k5"] = this->distortionParams.at<double>(6,0);
	temp["Camera.k6"] = this->distortionParams.at<double>(7,0);
	temp["Camera.s1"] = this->distortionParams.at<double>(8,0);
	temp["Camera.s2"] = this->distortionParams.at<double>(9,0);
	temp["Camera.s3"] = this->distortionParams.at<double>(10,0);
	temp["Camera.s4"] = this->distortionParams.at<double>(11,0);
	temp["Camera.taox"] = this->distortionParams.at<double>(12,0);
	temp["Camera.taoy"] = this->distortionParams.at<double>(13,0);

	temp["FileInformation.DateOfCreation"] = str;

	if(fout.is_open() && fout.good()){
		fout << temp;
	}
	else{
		return false;
	}

	fout.close();

	return true;
}

bool Camera::dump_stats(const std::string &output)
{
	std::ofstream log(output);


	if(log.is_open() && log.good()){
		for(int i = 0; i < CalibrationStat.numberSamples; i++){

			cv::Mat rot = CalibrationStat.rVectors.at(i);
			cv::Mat tran = CalibrationStat.tVectors.at(i);
			double partRms = CalibrationStat.viewError.at<double>(i, 0);

			log << "image " << i << ": "<< "Rotation: " << rot.at<double>(0,0) << " " <<
				rot.at<double>(1,0) << " "<< rot.at<double>(2,0) <<
				" Translation: " << tran.at<double>(0,0) << " " <<
				tran.at<double>(1,0) << " " << tran.at<double>(2,1) << " viewError(rms): " 
				<< partRms << std::endl;

			log << "Trans stats: [";

			for(int i = 0; i < NUMBR_TRANSFORM_STDDEV; i++){
				log << transExDesc[i] << ": " 
					<< CalibrationStat.stdDeviationExtrinsics.at<double>(NUMBR_TRANSFORM_STDDEV + i, 0) << " ";
			}
			log << "]" << std::endl;
		}

		log << "== Standard Deviation intrinsics parameters:" << std::endl;
		for(int i = 0; i < CalibrationStat.stdDevIntrinsics.size[0]; i++){
			log << distDesc[i] << ": " << CalibrationStat.stdDevIntrinsics.at<double>(i,0) << std::endl;
		}
		log.close();
	}
	else{
		return false;
	}
	return true;
}

Camera::Camera(const std::string &camName): 
	intrinsics{cv::Mat::eye(3, 3, CV_64F)},
	distortionParams{cv::Mat::zeros(14, 1, CV_64F)},
	name_(camName)
{
}

void Camera::print(){
	if(this->calibrated_){


		cv::calibrationMatrixValues(
				this->intrinsics,
				cv::Size(this->pixWidth_, this->pixHeight_),
				this->sensorWidth_,
				this->sensorHeight_,
				this->fovx_,
				this->fovy_,
				this->focalLength_,
				this->principalPoint_,
				this->aspectRatio_
				);

		std::cout << "Camera " << this->name_ << std::endl;
		std::cout << "fovh: " << this->fovx_ << std::endl;
		std::cout << "fovv: " << this->fovy_ << std::endl;
		std::cout << "focal length: " << this->focalLength_ << std::endl;
		std::cout << "aspect ratio: " << this->aspectRatio_ << std::endl;
		std::cout << "principal point x " << this->principalPoint_.x << std::endl;
		std::cout << "principal point y " << this->principalPoint_.y << std::endl;
	}
	else{
		std::cout << "Uncalibrated camera!" << std::endl;
	}
}


void Camera::projectPoints(const std::vector<vecp3f> &worldPoints, 
														std::vector<vecp2f> &projectedPoints)
{

	cv::projectPoints(worldPoints, 
			this->CalibrationStat.rVectors, 
			this->CalibrationStat.tVectors,
			this->intrinsics,
			this->distortionParams,
			projectedPoints,
			cv::noArray(),
			0
			);
}


double Camera::calibrate(const std::vector<vecp3f> &worldPoints,
								const std::vector<vecp2f> &imagePoints,
								const CalibrationConfig &calibConf)
{

	this->calibrated_ = true;
	this->CalibrationStat.numberSamples = imagePoints.size();

	// TODO: this can be two different functions!
	switch(calibConf.calibType()){
		case CalibType::REGULAR:
			return cv::calibrateCamera(
					worldPoints, 
					imagePoints, 
					cv::Size(this->pixWidth_, this->pixHeight_), 
					this->intrinsics, 
					this->distortionParams, 
					this->CalibrationStat.rVectors, 
					this->CalibrationStat.tVectors,
					this->CalibrationStat.stdDevIntrinsics, 
					this->CalibrationStat.stdDeviationExtrinsics, 
					this->CalibrationStat.viewError, 
					calibConf.oflags(),
					calibConf.criteria()
					);
			break;
		case CalibType::RO:
			return cv::calibrateCameraRO(
					worldPoints, 
					imagePoints, 
					cv::Size(this->pixWidth_, this->pixHeight_), 
					calibConf.fixedPoint(),
					this->intrinsics, 
					this->distortionParams, 
					this->CalibrationStat.rVectors, 
					this->CalibrationStat.tVectors,
					cv::noArray(), // could try to use this later
					/* this->CalibrationStat.newObjPoints, */
					this->CalibrationStat.stdDevIntrinsics, 
					this->CalibrationStat.stdDeviationExtrinsics, 
					cv::noArray(), // could try to use this later
					this->CalibrationStat.viewError, 
					calibConf.oflags(),
					calibConf.criteria()
					);
			break;
		default:
			throw std::runtime_error("Unknown type");
			break;
	}
}
