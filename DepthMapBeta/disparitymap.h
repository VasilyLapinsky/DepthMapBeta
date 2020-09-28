#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;
using namespace ximgproc;

class StereoImagePreprocessor
{
private:
	Mat left, right;
	double fx, fy;
	int cvtcode, interpolation, dstCn;
public:
	/*
	@param cvtcode color space conversion code (see #ColorConversionCodes).
	@param fx scale factor along the horizontal axis; when it equals 0, it is computed as
			\f[\texttt{(double)dsize.width/src.cols}\f]
	@param fy scale factor along the vertical axis; when it equals 0, it is computed as
			\f[\texttt{(double)dsize.height/src.rows}\f]
	@param interpolation interpolation method, see #InterpolationFlags OpenCV
	@param dstCn number of channels in the destination image; if the parameter is 0, the number of the
			channels is derived automatically from input image and code.
	*/
	StereoImagePreprocessor(int cvtcode, double fx = 0, double fy = 0,
		int interpolation = INTER_LINEAR, int dstCn = 0);
	/*
	Preprocess an image for DisparityMapMaker
	Result will be stored in the object
	@param image input image which will be preprocessed
	*/
	void process(const Mat& image);
	/* Gives left image after proccessing*/
	Mat getleft();
	/* Gives right image after proccessing*/
	Mat getright();
};

class DisparityMapMaker
{
private:
	Mat disparitymap;
	int numDisparities, blockSize;
	Ptr<StereoMatcher> left_matcher;
	Ptr<StereoMatcher> right_matcher;
	Ptr<DisparityWLSFilter> wls_filter;
public:
	/*
	@param numDisparities the disparity search range. For each pixel algorithm 
		will find the best disparity from 0 (default minimum disparity) to numDisparities. 
		The search range can then be shifted by changing the minimum disparity.
	@param blockSizehe linear size of the blocks compared by the algorithm. 
		The size should be odd (as the block is centered at the current pixel). 
	*/
	DisparityMapMaker(int numDisparities = 0, int blockSize = 21);
	/*
	Computer a disparite map for leand and right images. The result stored in the obj.
	*/
	void compute(const Mat& left, const Mat& right);
	/* Get stored disparity map*/
	Mat getDisparityMap();
};

class DisparityMapWriter
{
private:
	String wname;
	int rtype, colormap;
public:
	/*
	@param winname Name of the window
	@param rtype desired output matrix type or, rather, the depth since the number of channels are the
		same as the input has; if rtype is negative, the output matrix will have the same type as the input.
	@param colormap The colormap to apply, see #ColormapTypes
	*/
	DisparityMapWriter(const String& winname, int rtype, int colormap);
	/* Show disparity map in the window with custom preparations*/
	void show(const Mat& disparitymap);
};