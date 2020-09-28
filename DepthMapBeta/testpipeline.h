#pragma once

#include "stereocameracalib.h"
#include "disparitymap.h"

void arhitectureTest()
{
	// -- Calibration of cameras -----------------------------------------------
	int numberOfImages = 25;
	vector<vector<Point3f>> objectPoints = createObjectPointsForMultipleImg(numberOfImages, 6, 8, 0.025);
	vector<vector<Point2f>> imgPointsLeft, imgPointsRight;
	Size patternSize{ 8, 6 }, imageSize{ 1280, 720 };
	// image points
	createImagePoints(imgPointsLeft, imgPointsRight, numberOfImages, patternSize);

	Mat cameraMatLeft, cameraMatRight, distCoeffsLeft, distCoeffsRight,
		rvecsleft, rvecsright, tvecsleft, tvecsright,
		stdDeviationsIntrinsicsLeft, stdDeviationsIntrinsicsRight,
		stdDeviationsExtrinsicsLeft, stdDeviationsExtrinsicsRight,
		perViewErrorsLeft, perViewErrorsRight;
	// Individual calibration
	calibrateCamera(objectPoints, imgPointsLeft, imageSize, cameraMatLeft, distCoeffsLeft,
		rvecsleft, tvecsleft);
	cout << "Camera matrix left:\n" << cameraMatLeft << endl;
	cout << "Distance coeffs left:\n" << distCoeffsLeft << endl;
	calibrateCamera(objectPoints, imgPointsRight, imageSize, cameraMatRight, distCoeffsRight,
		rvecsright, tvecsright);
	cout << "Camera matrix right:\n" << cameraMatRight << endl;
	cout << "Distance coeffs right:\n" << distCoeffsRight << endl;
	// Stereo calibration
	Mat R, T, E, F;
	stereoCalibrate(objectPoints, imgPointsLeft, imgPointsRight, cameraMatLeft, distCoeffsLeft,
		cameraMatRight, distCoeffsRight, imageSize, R, T, E, F, CALIB_FIX_INTRINSIC);

	// --Rectification
	Mat R1, R2, P1, P2, Q;
	stereoRectify(cameraMatLeft, distCoeffsLeft, cameraMatRight, distCoeffsRight,
		imageSize, R, T, R1, R2, P1, P2, Q);

	// -- Remap preparation
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(cameraMatLeft, distCoeffsLeft, R1, P1, imageSize, CV_16SC2, map11, map12);
	initUndistortRectifyMap(cameraMatRight, distCoeffsRight, R2, P2, imageSize, CV_16SC2, map21, map22);

	// -- Disparity map ----------------------------------------------------------------
	DisparityMapMaker maker(32, 15);
	int targetX = 100, targetY = 300;
	Mat image, left, right, leftr, rightr, disparitymap, depthmap;
	
	image = imread("Image1.png");

	// Image preparation
	image(Rect{ 0, 0, image.cols / 2, image.rows }).copyTo(left);
	image(Rect{ image.cols / 2, 0, image.cols / 2, image.rows }).copyTo(right);
	cvtColor(left, left, COLOR_BGR2GRAY);
	cvtColor(right, right, COLOR_BGR2GRAY);

	remap(left, leftr, map11, map12, INTER_LINEAR);
	remap(right, rightr, map21, map22, INTER_LINEAR);

	// -- Depth map ----------------------------------------------------------
	maker.compute(leftr, rightr);
	reprojectImageTo3D(maker.getDisparityMap(), depthmap, Q);
	// output
	double distancex = depthmap.at<Point3f>(targetX, targetY).x;
	double distancey = depthmap.at<Point3f>(targetX, targetY).y;
	double distancez = depthmap.at<Point3f>(targetX, targetY).z;
	addTarget(left, targetX, targetY);
	putText(left, "Distance:   " + to_string(distancex)+" "+to_string(distancey)+" "+to_string(distancez), 
		Point(100, 100), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1);

	addTarget(leftr, targetX, targetY);
	putText(leftr, "Distance:   " + to_string(distancex) + " " + to_string(distancey) + " " + to_string(distancez),
		Point(100, 100), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1);

	imshow("result", left);
	imshow("resultr", leftr);
	while (true)
	{
		if (waitKey(33) >= 0) break;
	}
}