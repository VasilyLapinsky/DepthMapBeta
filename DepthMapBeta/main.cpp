

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

using namespace std;
using namespace cv;
using namespace ximgproc;



void makeDisparityMap(Mat image, Mat& disparity_map)
{
	// Cut main stereo image into two parts
	Mat left, right;
	image(Rect{ 0, 0, image.cols / 2, image.rows }).copyTo(left);
	image(Rect{ image.cols / 2, 0, image.cols / 2, image.rows }).copyTo(right);

	// Resize images
	Mat left_for_matcher, right_for_matcher;
	resize(left, left_for_matcher, Size(), 0.5, 0.5, INTER_LINEAR_EXACT);
	resize(right, right_for_matcher, Size(), 0.5, 0.5, INTER_LINEAR_EXACT);

	// Set max disparity
	int min_disp = 0, max_disp = 16;
	if (max_disp % 16 != 0)
		max_disp += 16 - (max_disp % 16);
	// Create filters
	Ptr<StereoSGBM> left_matcher = StereoSGBM::create(min_disp,   //mindisp
		max_disp, //numdisp
		7,   //BlockSize
		0,   //P1
		0,   //P2
		1,   //dispdiffmax
		0,   //prefiltercap
		0,   //uniqueness
		0,   //speckle window size
		1,   //speckle range
		cv::StereoSGBM::MODE_SGBM);
	Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

	// Change image color set
	cvtColor(left_for_matcher, left_for_matcher, COLOR_BGR2GRAY);
	cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

	// Make matrix of disparities
	Mat left_disparity, norm_left_disparity, right_disparity, norm_right_disparity;
	left_matcher->compute(left_for_matcher, right_for_matcher, left_disparity);
	right_matcher->compute(left_for_matcher, right_for_matcher, right_disparity);

	// Make wls filter
	Mat filtered_image;
	wls_filter->filter(left_disparity, left_for_matcher, filtered_image, right_disparity);

	// Convert disparity map into grayscale image
	getDisparityVis(filtered_image, disparity_map, 16);
}

void videoCameraTest() {
	namedWindow("Input", cv::WINDOW_AUTOSIZE);
	namedWindow("Map", cv::WINDOW_AUTOSIZE);
	VideoCapture cap(1);
	//cap.open(string(argv[1]));

	if (!cap.isOpened()) {
		cout << "Error opening video stream or file" << endl;
		return;
	}

	Mat image, disparity_map;
	for (;;) {
		cap >> image;
		if (image.empty()) break;
		imshow("Input", image);

		makeDisparityMap(image, disparity_map);

		imshow("Map", disparity_map);
		if (waitKey(33) >= 0) break;
	}
}

void videoTest()
{
	VideoCapture cap("Video1.svo");
	if (!cap.isOpened()) {
		cout << "Error opening video stream or file" << endl;
		return;
	}
	Mat image, disparity_map;
	for (;;) {
		cap >> image;
		if (image.empty()) break;
		imshow("Input", image);

		makeDisparityMap(image, disparity_map);

		imshow("Map", disparity_map);
		if (waitKey(33) >= 0) break;
	}
}
void imageTest()
{
	Mat image = imread("Image1.png");
	if (image.empty()) {
		cout << "Error opening image" << endl;
		return;
	}
	// Cut input image for better visualization
	Mat left, right;
	image(Rect{ 0, 0, image.cols / 2, image.rows }).copyTo(left);
	image(Rect{ image.cols / 2, 0, image.cols / 2, image.rows }).copyTo(right);
	imshow("left", left);
	imshow("right", right);
	// 
	Mat disparity_map;
	makeDisparityMap(image, disparity_map);
	imshow("Map", disparity_map);
	for (;;) {
		if (waitKey(33) >= 0) break;
	}
}


int main(int argc, char** argv)
{

	imageTest();
}

