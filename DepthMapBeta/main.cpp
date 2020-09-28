#include <opencv2/highgui.hpp>

#include "stereocameracalib.h"
#include "disparitymap.h"
#include "testpipeline.h"

void readQ(string filename, Mat& Q)
{
	Q = Q.zeros(4, 4, CV_32F);
	ifstream in(filename);
	char ch;
	in >> ch; // read [
	for (int i = 0; i < 4; ++i)
	{
		double temp;
		for (int j = 0; j < 4; ++j)
		{
			in >> temp >> ch;// read number and ','
			Q.at<float>(i, j) = temp;
		}
	}
}

void addLines(Mat & img)
{
	for (int i = 0; i < img.rows; i++)
		for (int j = 0; j < img.cols; j++)
			if ((i % 20 == 10 && j % 2 == 1) ||
				(j % 50 == 25 && i % 2 == 1))
			{
				img.at<Vec3b>(i, j)[0] = 255;
				img.at<Vec3b>(i, j)[1] = 255;
				img.at<Vec3b>(i, j)[2] = 255;
			}
}

void videoCameraTest()
{
	Mat Q;
	readQ("Q25.txt", Q);
	VideoCapture cap(1);
	if (!cap.isOpened()) {
		cout << "Error opening video stream or file" << endl;
		return;
	}
	StereoImagePreprocessor preProcessor(COLOR_BGR2GRAY, 1, 1, INTER_LINEAR_EXACT);
	DisparityMapMaker maker(32, 15);
	int targetX = 325, targetY = 150;
	Mat image, depthMap, leftimage;
	for (;;) {
		cap >> image;
		preProcessor.process(image);
		preProcessor.getleft().copyTo(leftimage);
		maker.compute(preProcessor.getleft(), preProcessor.getright());
		reprojectImageTo3D(maker.getDisparityMap(), depthMap, Q);
		double distance = depthMap.at<Point3f>(targetX, targetY).z;
		addTarget(leftimage, targetX, targetY);
		putText(leftimage, "Distance:   "+to_string(distance), Point(100, 100),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1);
		imshow("result", leftimage);
		if (waitKey(33) >= 0) break;
	}
}

void imageTest()
{
	int xTarget = 500, yTarget = 250;
	Mat image = imread("Image1.png");
	Mat left, right;
	cutStereoForTwoParts(image, left, right);
	Mat depthMap;

	Mat Q;
	readQ("Q10.txt", Q);

	StereoImagePreprocessor preProcessor(COLOR_BGR2GRAY, 1, 1, INTER_LINEAR_EXACT);
	DisparityMapMaker maker(32, 15);

	preProcessor.process(image);
	maker.compute(preProcessor.getleft(), preProcessor.getright());
	reprojectImageTo3D(maker.getDisparityMap(), depthMap, Q);

	double distance = depthMap.at<Point3f>(xTarget, yTarget).z;

	addTarget(right, xTarget, yTarget);
	putText(right, "Distnce: " + to_string(distance), Point(100, 100),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1);
	imshow("result", right);

	while (true) {
		if (waitKey(33) >= 0) break;
	}
}


int main()
{
	videoCameraTest();
	return 0;
}