#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <sstream>

#include <opencv2/calib3d.hpp>

#include "camera.hpp"

std::string calibFlagsNone = "CalibrationFlags: []\n";
std::string pointFlagsNone = "PointFlags: []\n";
std::string regSize = "PatternSize: [6, 9]\n";
std::string geoDim = "PatternDimensions: 0.02635\n";
std::string pType = "PointType: CHESS\n";
std::string cType = "CalibrationType: REGULAR\n"; 

TEST(calibrationConfig, regular){


	YAML::Node test = YAML::Load(
			calibFlagsNone +
			pointFlagsNone +
			regSize +
			geoDim  +
			pType +
			cType
			);

	CalibrationConfig cc(test);

	EXPECT_FLOAT_EQ(cc.dim(), 2.635e-2f);

	EXPECT_FALSE(cc.oflags());
	EXPECT_FALSE(cc.pflags());
	EXPECT_TRUE(cc.pointType() == PointType::C_CHESS);
	EXPECT_TRUE(cc.calibType() == CalibType::REGULAR);

}

TEST(calibrationConfig, operationFlags){

	/* YAML::Node test = YAML::Load(input1); */
	YAML::Node test = YAML::Load(
			"CalibrationFlags: ["
			"cv::CALIB_USE_INTRINSIC_GUESS," 
			"cv::CALIB_FIX_PRINCIPAL_POINT," 
			"cv::CALIB_FIX_ASPECT_RATIO,"
			"cv::CALIB_ZERO_TANGENT_DIST,"
			"cv::CALIB_FIX_K2,"
			"cv::CALIB_FIX_K3,"
			"cv::CALIB_FIX_K4,"
			"cv::CALIB_FIX_K5,"
			"cv::CALIB_FIX_K6,"
			"cv::CALIB_RATIONAL_MODEL,"
			"cv::CALIB_THIN_PRISM_MODEL,"
			"cv::CALIB_FIX_S1_S2_S3_S4,"
			"cv::CALIB_TILTED_MODEL,"
			"cv::CALIB_FIX_TAUX_TAUY"
			"]\n" +
			pointFlagsNone +
			regSize +
			geoDim  +
			pType +
			cType
			);

	CalibrationConfig cc(test);

	EXPECT_TRUE(cc.oflags() & cv::CALIB_USE_INTRINSIC_GUESS);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_PRINCIPAL_POINT);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_ASPECT_RATIO);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_ZERO_TANGENT_DIST);
	EXPECT_FALSE(cc.oflags() & cv::CALIB_FIX_K1);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_K2);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_K3);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_K4);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_K5);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_K6);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_RATIONAL_MODEL);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_THIN_PRISM_MODEL);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_S1_S2_S3_S4);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_TILTED_MODEL);
	EXPECT_TRUE(cc.oflags() & cv::CALIB_FIX_TAUX_TAUY);

}

TEST(CalibrationConfig, pointFlagsChess){
	YAML::Node test = YAML::Load(
			calibFlagsNone +
			"PointFlags: ["
			"cv::CALIB_CB_ADAPTIVE_THRESH,"
			"cv::CALIB_CB_FILTER_QUADS,"
			"cv::CALIB_CB_FAST_CHECK,"
			"cv::CALIB_CB_NORMALIZE_IMAGE,"
			"cv::CALIB_CB_EXHAUSTIVE,"
			"cv::CALIB_CB_ACCURACY,"
			"cv::CALIB_CB_LARGER,"
			"cv::CALIB_CB_MARKER,"
			"]\n" +
			regSize +
			geoDim  +
			pType +
			cType
			);

	CalibrationConfig cc(test);

	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_ADAPTIVE_THRESH);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_FILTER_QUADS);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_FAST_CHECK);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_NORMALIZE_IMAGE);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_EXHAUSTIVE);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_ACCURACY);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_LARGER);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_MARKER);

}

TEST(CalibrationConfig, pointFlagsCircle){
	YAML::Node test = YAML::Load(
			calibFlagsNone +
			"PointFlags: ["
			"cv::CALIB_CB_SYMMETRIC_GRID,"
			"cv::CALIB_CB_ASYMMETRIC_GRID,"
			"cv::CALIB_CB_CLUSTERING"
			"]\n" +
			regSize +
			geoDim +
			"PointType: CIRCLE\n" +
			cType
			);

	CalibrationConfig cc(test);

	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_SYMMETRIC_GRID);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_ASYMMETRIC_GRID);
	EXPECT_TRUE(cc.pflags() & cv::CALIB_CB_CLUSTERING);

}


TEST(CalibrationConfig, pointFlagsCircle2){
	YAML::Node test = YAML::Load(
			calibFlagsNone +
			"PointFlags: ["
			"]\n" +
			regSize +
			geoDim +
			"PointType: CIRCLE\n" +
			cType
			);

	CalibrationConfig cc(test);

	EXPECT_FALSE(cc.pflags() & cv::CALIB_CB_SYMMETRIC_GRID);
	EXPECT_FALSE(cc.pflags() & cv::CALIB_CB_ASYMMETRIC_GRID);
	EXPECT_FALSE(cc.pflags() & cv::CALIB_CB_CLUSTERING);

}
int main(int argc, char *argv[]){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
