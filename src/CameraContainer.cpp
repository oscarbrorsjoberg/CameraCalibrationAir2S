#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

#include "CameraContainer.hpp"


YAML::Node createCameraCalibrationSchema()
{
	std::string cmra = "Camera.name: DJI AIR2S\n";
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

	return YAML::Load(cmra);

}
