#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <vector>
#include <array>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>


#include "Camera.hpp"

using  chsys = chrono::system_clock ;


static std::map<std::string, int> calibrationFlags_m;
static std::map<std::string, int> pointFlags_m;

static const std::array<std::tuple<std::string, int>, 16> posCalibrationFlags =
{
// cameraMatrix contains valid initial values of fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image center ( imageSize is used), and focal distances are computed in a least-squares fashion. Note, that if intrinsic parameters are known, there is no need to use this function just to estimate extrinsic parameters. Use solvePnP instead.
{"cv::CALIB_USE_INTRINSIC_GUESS", cv::CALIB_USE_INTRINSIC_GUESS},

// The principal point is not changed during the global optimization. It stays at the center or at a different location specified when CALIB_USE_INTRINSIC_GUESS is set too.
{"cv::CALIB_FIX_PRINCIPAL_POINT", cv::CALIB_FIX_PRINCIPAL_POINT},

// The functions consider only fy as a free parameter. The ratio fx/fy stays the same as in the input cameraMatrix . When CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are ignored, only their ratio is computed and used further.
{"cv::CALIB_FIX_ASPECT_RATIO", cv::CALIB_FIX_ASPECT_RATIO},
// Tangential distortion coefficients (p1,p2) are set to zeros and stay zero.
{"cv::CALIB_ZERO_TANGENT_DIST", cv::CALIB_ZERO_TANGENT_DIST},

// The corresponding radial distortion coefficient is not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
{"cv::CALIB_FIX_K1", cv::CALIB_FIX_K1},
{"cv::CALIB_FIX_K2", cv::CALIB_FIX_K2}, 
{"cv::CALIB_FIX_K3", cv::CALIB_FIX_K3}, 
{"cv::CALIB_FIX_K4", cv::CALIB_FIX_K4}, 
{"cv::CALIB_FIX_K5", cv::CALIB_FIX_K5}, 
{"cv::CALIB_FIX_K6", cv::CALIB_FIX_K6}, 
// Coefficients k4, k5, and k6 are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the rational model and return 8 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients.
{"cv::CALIB_RATIONAL_MODEL", cv::CALIB_RATIONAL_MODEL},

//Coefficients s1, s2, s3 and s4 are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the thin prism model and return 12 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients.

{"cv::CALIB_THIN_PRISM_MODEL", cv::CALIB_THIN_PRISM_MODEL},

//The thin prism distortion coefficients are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
	{"cv::CALIB_FIX_S1_S2_S3_S4", cv::CALIB_FIX_S1_S2_S3_S4},
//Coefficients tauX and tauY are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the tilted sensor model and return 14 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients.
{"cv::CALIB_TILTED_MODEL", cv::CALIB_TILTED_MODEL},
//The coefficients of the tilted sensor model are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
{"cv::CALIB_FIX_TAUX_TAUY", cv::CALIB_FIX_TAUX_TAUY}
};

static const std::array<std::tuple<std::string, int>, 11> posPointFlags =
{
	// Use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness).
	{"cv::CALIB_CB_ADAPTIVE_THRESH", cv::CALIB_CB_ADAPTIVE_THRESH},

// Use additional criteria (like contour area, perimeter, square-like shape) to filter out false quads extracted at the contour retrieval stage.
	{"cv::CALIB_CB_FILTER_QUADS", cv::CALIB_CB_FILTER_QUADS},

// Run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed.
{"cv::CALIB_CB_FAST_CHECK", cv::CALIB_CB_FAST_CHECK},

// Normalize the image gamma with equalizeHist before detection.
{"cv::CALIB_CB_NORMALIZE_IMAGE", cv::CALIB_CB_NORMALIZE_IMAGE},

// Run an exhaustive search to improve detection rate.
{"cv::CALIB_CB_EXHAUSTIVE", cv::CALIB_CB_EXHAUSTIVE},

// Up sample input image to improve sub-pixel accuracy due to aliasing effects.
{"cv::CALIB_CB_ACCURACY", cv::CALIB_CB_ACCURACY},

// The detected pattern is allowed to be larger than patternSize (see description).
{"cv::CALIB_CB_LARGER", cv::CALIB_CB_LARGER},

// The detected pattern must have a marker (see description). This should be used if an accurate camera calibration is required.
{"cv::CALIB_CB_MARKER", cv::CALIB_CB_MARKER},

	// uses symmetric pattern of circles.
{"cv::CALIB_CB_SYMMETRIC_GRID", cv::CALIB_CB_SYMMETRIC_GRID}, 

// uses asymmetric pattern of circles.
{"cv::CALIB_CB_ASYMMETRIC_GRID", cv::CALIB_CB_ASYMMETRIC_GRID},

	// uses a special algorithm for grid detection. 
	// It is more robust to perspective distortions but much 
	// more sensitive to background clutter.
{"cv::CALIB_CB_CLUSTERING", cv::CALIB_CB_CLUSTERING}
};



static void initFlagsMaps()
{

	for(const auto &[f, s] : posPointFlags){
		pointFlags_m[f] = m;
	}

	for(const auto &[f, s] : posCalibrationFlags){
		calibrationFlags_m[f] = m;
	}

}



CalibrationConfig::CalibrationConfig(YAML::Node &config):
	criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001)

{

	initFlagMaps();

	for( const auto &fl : calibConfig["CalibrationFlags"].as<std::vector<std::string>>)
		operationFlags |= calibrationFlags_m[fl];

	// try if not in
	for( const auto &fl : calibConfig["PointFlags"].as<std::vector<std::string>>;)
		pointFlags |= pointFlags_m[fl];

	std::string calibType = calibConfig["CalibrationType"].as<std::string>();
	std::string pointType = calibConfig["PointType"].as<std::string>();

	// does this work or does it have to be dynamic?
	int size[2] = calibConfig["PatternSize"].as<int[2]>();
	this->patternSize(size[0], size[1]);

	assert((!calibType.empty() && (calibType == "REGULAR" || calibType == "RO")
				, "Calib Type is required!");
	assert((!pointType.empty() && 
			(pointType == "CIRCLE" || pointType == "CHESS" 
			 || pointType == "SB_CHESS"), "Point Type is required!");

	if(pointType == "CIRCLE"){
		findPoints = std::bind(cv::findCirclesGrid,
														_1, this->patternSize, _2, this->pointFlags);
		pt = PointType::C_CIRCLES;
	}
	else if(pointType == "CHESS"){
		findPoints = std::bind(cv::findChessBoardCorners,
														_1, this->patternSize, _2, this->pointFlags);
		pt = PointType::C_CHESS;
	}
	else if(pointType == "SB_CHESS"){
		findPoints = std::bind(cv::findChessBoardCornersSB,
														_1, this->patternSize, _2, this->pointFlags, cv::noArray());
		pt = PointType::C_SB_CHESS;
		// TODO: check no Array

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


bool write(const std::string &output)
{
	std::ofstream fout(output);

	YAML::Node temp = YAML::Load(loadCameraSchema(name));


	chsys::time_point p = chsys::now();
	time_t t = chsys::to_time_t(p);
	char str[26];
	ctime_s(str, sizeof(str), &t);


	cameraParams["Camera.fx"] = this->cameraMatrix.at<double>(0,0);
	cameraParams["Camera.fy"] = this->cameraMatrix.at<double>(1,1);
	cameraParams["Camera.cx"] = this->cameraMatrix.at<double>(0,2);
	cameraParams["Camera.cy"] = this->cameraMatrix.at<double>(1,2);
	cameraParams["Camera.k1"] = this->distortionParams.at<double>(0,0);
	cameraParams["Camera.k2"] = this->distortionParams.at<double>(1,0);
	cameraParams["Camera.p1"] = this->distortionParams.at<double>(2,0);
	cameraParams["Camera.p2"] = this->distortionParams.at<double>(3,0);
	cameraParams["Camera.k3"] = this->distortionParams.at<double>(4,0);
	cameraParams["Camera.k4"] = this->distortionParams.at<double>(5,0);
	cameraParams["Camera.k5"] = this->distortionParams.at<double>(6,0);
	cameraParams["Camera.k6"] = this->distortionParams.at<double>(7,0);
	cameraParams["Camera.s1"] = this->distortionParams.at<double>(8,0);
	cameraParams["Camera.s2"] = this->distortionParams.at<double>(9,0);
	cameraParams["Camera.s3"] = this->distortionParams.at<double>(10,0);
	cameraParams["Camera.s4"] = this->distortionParams.at<double>(11,0);
	cameraParams["Camera.taox"] = this->distortionParams.at<double>(12,0);
	cameraParams["Camera.taoy"] = this->distortionParams.at<double>(13,0);

	cameraParams["FileInformation.DateOfCreation"] = str;

	if(fout.is_open() && fout.good()){
		fout << temp;
	}
	fout.close();
}

Camera::Camera(const std::string &camName):
	name(camName),
	cameraMatrix{cv::Mat::eye(3, 3, CV_64F)},
	distortionParams(cv::Mat::zeros(14, 1, CV_64F))
{

}


double CameraContainer::
calibrateCamera(const std::vector<vecp3f> &worldPoints,
								const std::vector<vecp2f> &imagePoints,
								const CalibrationConfig calibConf)
{

	calibrated = true;

	// TODO: this can be two different functions!
	switch(calibConf.calibType()){
		case CalibType::REGULAR:
			return cv::calibrateCamera(
					worldPoints, 
					imagePoints, 
					cv::Size(this->pixWidth, this->pixHeight), 
					this->cameraMatrix, 
					this->distortionParams, 
					this->CalibrationStat.rVectors, 
					this->CalibrationStat.tVectors,
					this->CalibrationStat.stdDevIntrinsics, 
					this->CalibrationStat.stdDeviationExtrinsics, 
					this->CalibrationStat.viewError, 
					calibConfig.flags(),
					calibConfig.criteria()
					);
			break;
		case CalibType::RO:
			return cv::calibrateCameraRO(
					worldPoints, 
					imagePoints, 
					cv::Size(this->pixWidth, this->pixHeight), 
					calibConfig.fixedPoint(),
					this->cameraMatrix, 
					this->distortionParams, 
					this->CalibrationStat.rVectors, 
					this->CalibrationStat.tVectors,
					cv::noArray(), // could try to use this later
					/* this->CalibrationStat.newObjPoints, */
					this->CalibrationStat.stdDevIntrinsics, 
					this->CalibrationStat.stdDeviationExtrinsics, 
					this->CalibrationStat.viewError, 
					calibConfig.flags(),
					calibConfig.criteria()
					);
			break;
		default:
			throw std::runtime_error("Unknown ");
			break;
	}

}
