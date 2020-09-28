#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;
/* adds target to an image */
void addTarget(Mat& img, int x, int y);

/* Creates Mat3f which contains calibration pattern points */
Mat createObjectPoints(int n, int m, double distance);

/* Creates Object Points set for multiple images*/
vector<vector<Point3f>> createObjectPointsForMultipleImg(int numberOfImages, int n, int m, float distance);

/* Returns left and right subimages from strereoimage */
void cutStereoForTwoParts(const Mat& stereoImg, Mat& leftImg, Mat& rightImg);

/* Create Image Points for left and right images of stereo image.
	@param imgPointsLeft resulting image point for left image
	@param imgPointsRight resulting image point for right image
	@param numbernumberOfImages number of stereo images wich stored in Images dirrectory
			and named from one: "number.png"
	@param patterSize pattern size of chessboard
*/
void createImagePoints(vector<vector<Point2f>>& imgPointsLeft, vector<vector<Point2f>>& imgPointRight,
	int numberOfImages, Size patternSize);

/* Visual testing funcion of createImagePoints */
void testImagePoints();

/* Computs 4x4 perspective transformation matrix */
void calibrateStereoCameraForQ(Mat& Q, int numberOfImages = 25);