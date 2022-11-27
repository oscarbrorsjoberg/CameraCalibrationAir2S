# Camera Calibration toolbox wrapping OpenCV
A small wrapper for testing out the different camera calibration methods in OpenCV.

## Dependencies/Requirments

1. C++17/gcc 9.4
2. OpenCV VERSION 4.5.2
3. Boost VERSION 1.74.0
4. yaml-cpp VERSION 0.7.0
5. GTest 1.10.0

## Building

Using standard cmake procedure:

```
mkdir build
cd build
cmake <path/to/CameraCalbration>
make
```

## How to use

The toolbox contains two separate binaries:
CameraCalibration and CameraUndistort.


# Camera Sensor Notes

## Sensor Sizes

[image](./educ/450px-Sensorformate_KB.svg.png) 
[sizes](https://designreviews.com/standards/digital-camera-sensor-size-chart/)

# Some babbeling
Full Frame Sensor = 35mm film 
36x24mm
> ASP-C
> 4/3rds
> Nikon
> 2/3rds
1/1.7" 


### CameraUndistort
Explain some stuff here!

## what-the-camera-calibration?


K [R | t]

1. [Zhang](https://www.cvl.isy.liu.se/education/undergraduate/tsbb09/lasmaterial/zhang-report.pdf)
	Zhangs report give some crucial insight into how the camera matrix can 
	be resectioned in intrinsic and extrinisc, and how to solve the different intrinsic and extrinsic camera parameters 
	given the calibration points. Some of the equations are covered more thoroughly in these 
	[slides](https://www.cvl.isy.liu.se/education/undergraduate/tsbb09/forelasningsslides/CameraCalibration2.pdf)

2. [Practical advice](https://calib.io/blogs/knowledge-base/calibration-best-practices)
	Some practical advice regarding set up and collecting data points.

3. [CircularVsChecker](https://www.researchgate.net/post/Which-pattern-circle-pattern-or-checkerboard-pattern-should-be-used-for-automotive-camera-calibration-fisheye-wide-webcam)

## CameraCalibration

Camera calibration can be done with different configurations, this is modulated through the camera calibration configuration file. Please see example: <>.

Here the different parameters that can be tweaked in a calibration step will be explained.
Different calibration flags will also be explained.

CalibrationFlags: 
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

PointFlags: 

  [cv::CALIB_CB_FAST_CHECK]
PatternSize: [6, 9]
PatternDimensions: 0.02635
PointType: 
    CHESS_SB
    CHESS
    CIRCLE
CalibrationType: REGULAR

## Distortions

### Radial distortions

### Tangential distortions

## How to collect points

### Circles (Circles grid)

Article?

### Chessboard (Chessboard regular)
Regular Zhang

### Chessboard (Chessboard SB)
http://bmvc2018.org/contents/papers/0508.pdf

### What about the subpixels?

Many of the functions handles subpixel precision in different ways.

Calculate after?
Use CameraCalibrateRO?
https://elib.dlr.de/71888/1/strobl_2011iccv.pdf


## FAQ


1. What is the rational model? cv::CALIB\_RATIONAL\_MODEL

Coefficients k4, k5, and k6 are enabled. To provide the backward compatibility, this extra flag should be explicitly specified to make the calibration function use the rational model and return 8 coefficients. If the flag is not set, the function computes and returns only 5 distortion coefficients.

2. What is the thin prisma model? CALIB\_THIN\_PRISM\_MODEL

The thin prism distortion coefficients are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.

2. What is the tilted model? CALIB\_TILTED\_MODEL

The coefficients of the tilted sensor model are not changed during the optimization. If CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.

3. What is the absolute conic?
Maths: K^(-T)K^(-1)
