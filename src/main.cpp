#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>

#include "utils.hpp"
#include "camera.hpp"

namespace fs = std::filesystem;
namespace po = boost::program_options;

// TODO: put in config file!
//

/* const cv::Size CHESSBOARD_SIZE = cv::Size(6, 9); */
/* constexpr float SQUARE_DIMENSIONS = 2.635e-2f; //meters */

/* constexpr int NUMBR_TRANSFORM_STDDEV = 6; */

const std::string distDesc[] = {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "k4", 
				"k5", "k6", "s1", "s2", "s3", "s4", "taox", "taoy"};
const std::string transExDesc[] = {"R0", "R1", "R2", "T0", "T1", "T2"};


bool read_cmd_line(int argc, char *argv[],
		std::string &impath, std::string &conf, std::string &out)
{
	po::options_description opt("CameraCalibration options");



	opt.add_options()
		("help,h", "produce help message")
		("path,p", po::value<std::string>(&impath)->required(), "path to images")
		("conf,c", po::value<std::string>(&conf)->required(), "configuration file")
		("out,o", po::value<std::string>(&out)->required(), "out camera parameters in .yml")
		;

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(opt).run(), vm);

	if(vm.count("help")){
		std::cout << opt << std::endl;
		return false;
	}

	po::notify(vm);
	return true;
}


int main(int argc, char *argv[])
{

	try{
		std::string impath, out, conf; 

		if(!read_cmd_line(argc, argv, impath, conf, out)){
			return 0;
		}

		assert(fs::path(out).extension() == ".yml");
		assert(fs::path(conf).extension() == ".yml");
		assert(fs::is_directory(impath));

		// out
		/* YAML::Node cameraParams = createCameraCalibrationSchema(); */

		YAML::Node ymlConf = YAML::LoadFile(conf);

		Camera cam("test");	
		CalibrationConfig calibConf(ymlConf);

		/* todo : functionalize*/

		std::vector<std::vector<cv::Point2f>> allCrnrs;
		std::vector<cv::Point2f> foundPoints;
		std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints;

		/* todo check this! */
		cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001);
		bool found = false;
		int im_idx = 0, added = 0;

		std::vector<std::tuple<bool, std::string>> imageInformation;

		// collect points in image 
		for(const auto& en : fs::directory_iterator(impath)){

			cv::Mat image = cv::imread(en.path(), cv::IMREAD_GRAYSCALE);
			std::vector<cv::Point3f> worldCoords;

			std::string imInfo = "File " + std::to_string(im_idx) + ": " + en.path().u8string() + " ";
			if(im_idx == 0){
				cam.setPixWidth(image.size[1]/2);
				cam.setPixHeight(image.size[0]/2);

			}

			imInfo += "size x: " + std::to_string(image.size[1]/2) 
				+ " size y: " + std::to_string(image.size[0]/2) + " ";
			cv::resize(image, image, cv::Size(image.size[1]/2, 
						image.size[0]/2), cv::INTER_AREA);

			found = calibConf.findPoints(image, foundPoints);

			/* found = findChessboardCorners(image, calibConf.patternSize(), foundPoints, */
			/* 		cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE); */

			if(found){

				cv::Scalar scales = 
					estimateChessboardSharpness(image, calibConf.patternSize(), foundPoints); 

				std::cout << en.path() << std::endl;
				std::cout << scales << std::endl;

				cv::cornerSubPix(image, foundPoints, cv::Size(11, 11), cv::Size(-1, -1), criteria);
				allCrnrs.push_back(foundPoints);

				createKnownBoardDim(calibConf.patternSize(),
														calibConf.dim(), 
														worldCoords);

				worldSpaceCornerPoints.push_back(worldCoords);

				added++;

				imInfo += "world points: " + 
					std::to_string(worldSpaceCornerPoints[added - 1].size()) + " "; 

				imInfo += "image points: " + 
					std::to_string(allCrnrs[added - 1].size()) + " "; 

			}
			else{
				imInfo += "image points: 0 world points: 0";
			}
			std::tuple<bool, std::string> collectRes = std::make_tuple(found, imInfo);

			imageInformation.push_back(collectRes);
			std::cout << im_idx++ << std::endl; // TODO : make a progress bar
		}


		std::cout << "Starting calibration!" << std::endl;

		double rms = cam.calibrate(worldSpaceCornerPoints,
															allCrnrs, 
															calibConf);

		std::cout << "Calibration finished" << std::endl;

		// TODO : move to a stats file from calibration
		/*
		std::ofstream log(log_path);
		int foundIdx = 0;
		for(auto &t : imageInformation){
			if(std::get<0>(t)){

				cv::Mat rot = rVectors.at(foundIdx);
				cv::Mat tran = tVectors.at(foundIdx);
				double partRms = viewError.at<double>(i, 0);
				log << std::get<1>(t) << "Rotation: " << rot.at<double>(0,0) << " " <<
					rot.at<double>(1,0) << " "<< rot.at<double>(2,0) <<
					" Translation: " << tran.at<double>(0,0) << " " <<
					tran.at<double>(1,0) << " " << tran.at<double>(2,1) << " viewError(rms): " 
					<< partRms << std::endl;

				log << "Trans stats: [";
				for(int i = 0; i < NUMBR_TRANSFORM_STDDEV; i++){
					log << transExDesc[i] << ": " 
						<< stdDeviationExtrinsics.at<double>(NUMBR_TRANSFORM_STDDEV*foundIdx + i, 0) << " ";
				}
				log << "]" << std::endl;

				foundIdx++;

			}
			else{
				log << std::get<1>(t) << std::endl;
			}
		}
		*/

		std::cout << "=== Calibration result ===" << std::endl;
		std::cout << "== RMS:" << rms << std::endl;

		cam.write(out);

		// TODO : move to a stats file from calibration
		/* log << "== Standard Deviation intrinsics parameters:" << std::endl; */
		/* for(int i = 0; i < stdDevIntrinsics.size[0]; i++){ */
		/* 	log << distDesc[i] << ": " << stdDevIntrinsics.at<double>(i,0) << std::endl; */
		/* } */
		/* log.close(); */

	}
	catch(std::exception const & e) {
		std::cerr << e.what() << std::endl;
	}

	return 0;

}
