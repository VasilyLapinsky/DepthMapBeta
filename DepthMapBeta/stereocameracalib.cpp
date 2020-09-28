#include "stereocameracalib.h"


void addTarget(Mat& img, int x, int y)
{
	for (int i = x - 50; i < x + 50; i++) {
		img.at<Vec3b>(y, i)[0] = 255;
		img.at<Vec3b>(y, i)[1] = 255;
		img.at<Vec3b>(y, i)[2] = 255;
	}
	for (int j = y - 50; j < y + 50; j++) {
		img.at<Vec3b>(j, x)[0] = 255;
		img.at<Vec3b>(j, x)[1] = 255;
		img.at<Vec3b>(j, x)[2] = 255;
	}
}

/* Creates Mat3f which contains calibration pattern points */
Mat createObjectPoints(int n, int m, float distance)
{
	Mat result;
	for (int j = 0; j < m; ++j) {
		for (int i = 0; i < n; ++i) {
			result.push_back(Point3f{ i * distance, j * distance, 0 });
		}
	}
	return result;
}

vector<vector<Point3f>> createObjectPointsForMultipleImg(int numberOfImages, int n, int m, float distance)
{
	vector<vector<Point3f>> result;
	vector<Point3f> objPoints = createObjectPoints(n, m, distance);
	for (int i = 0; i < numberOfImages; ++i)
		result.push_back(objPoints);
	return result;
}

/* Returns left and right subimages from strereoimage */
void cutStereoForTwoParts(const Mat& stereoImg, Mat& leftImg, Mat& rightImg)
{
	stereoImg(Rect{ 0, 0, stereoImg.cols / 2, stereoImg.rows }).copyTo(leftImg);
	stereoImg(Rect{ stereoImg.cols / 2, 0, stereoImg.cols / 2, stereoImg.rows }).copyTo(rightImg);
}

/* Create Image Points for left and right images of stereo image.
	@param imgPointsLeft resulting image point for left image
	@param imgPointsRight resulting image point for right image
	@param numbernumberOfImages number of stereo images wich stored in Images dirrectory
			and named from one: "number.png"
	@param patterSize pattern size of chessboard
*/
void createImagePoints(vector<vector<Point2f>>& imgPointsLeft, vector<vector<Point2f>>& imgPointRight, 
		int numberOfImages, Size patternSize) 
{
	Mat left, right, image;
	for (int i = 1; i <= numberOfImages; ++i)
	{
		vector<Point2f> cornersleft, cornersright;
		image = imread("D:/ZED/" + to_string(i) + ".png");
		cutStereoForTwoParts(image, left, right);
		bool isCornersFindfLeft = findChessboardCorners(left, patternSize, cornersleft, CALIB_CB_ADAPTIVE_THRESH);
		bool isCornersFindfRight = findChessboardCorners(right, patternSize, cornersright, CALIB_CB_ADAPTIVE_THRESH);
		if (isCornersFindfLeft && isCornersFindfRight) {
			imgPointsLeft.push_back(cornersleft);
			imgPointRight.push_back(cornersright);
		}
	}
}

void testImagePoints()
{
	int numberOfImages = 25;
	Mat imgPointsLeft, imgPointRight;
	Size patternSize{ 8, 6 };
	Mat left, right, image;
	for (int i = 1; i <= numberOfImages; ++i)
	{
		Mat cornersleft, cornersright;
		image = imread("D:/ZED/" + to_string(i) + ".png");
		cutStereoForTwoParts(image, left, right);
		bool isCornersFindfLeft = findChessboardCorners(left, patternSize, cornersleft, CALIB_CB_ADAPTIVE_THRESH);
		bool isCornersFindfRight = findChessboardCorners(right, patternSize, cornersright, CALIB_CB_ADAPTIVE_THRESH);
		if (isCornersFindfLeft && isCornersFindfRight) {
			imgPointsLeft.push_back(cornersleft);
			imgPointRight.push_back(cornersright);
			drawChessboardCorners(left, patternSize, cornersleft, true);
			imwrite("D:/Test/left "+to_string(i)+".png", left);
			drawChessboardCorners(right, patternSize, cornersright, true);
			imwrite("D:/Test/right " + to_string(i) + ".png", right);
		}
	}
}


void calibrateStereoCameraForQ(Mat & Q, int numberOfImages)
{
	// -- Calibration of cameras
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
	Mat R1, R2, P1, P2;
	stereoRectify(cameraMatLeft, distCoeffsLeft, cameraMatRight, distCoeffsRight,
		imageSize, R, T, R1, R2, P1, P2, Q);
}


