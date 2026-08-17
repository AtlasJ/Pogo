#include "ImageRegistration.h"
//#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "mtrx.h"
#include "Logger.h"

using namespace cv;

struct ImageRegistration::Impl {
	//variables
	MIL_ID _ECChomographyMatrix = M_NULL;
	cv::Mat _ORBhomographyMatrix;
	AlignmentType _imageAlignmentType = AlignmentType::NONE;

	double offset_x = 0.0, offset_y = 0.0;

	//functions
	cv::Mat MatroxtoCV_SingleChannel(MIL_ID & imgPtr);
	cv::Mat MatroxToCV(MIL_ID & imgPtr);
	void CVtoMatrox(cv::Mat input, MIL_ID & output);
	MIL_ID convertImage(MIL_ID & srcColour, int imageChannel);
	void transformPoint(cv::Mat & homography, cv::Point2f & point);
	void transformPoint(MIL_ID homography, cv::Point2f & point);
};

ImageRegistration::ImageRegistration() : p(std::make_unique<Impl>())
{
	MbufAlloc2d(M_DEFAULT, 3, 3, 32 + M_FLOAT, M_ARRAY, &p->_ECChomographyMatrix);
}

ImageRegistration::~ImageRegistration()
{
	if (p->_ECChomographyMatrix) MbufFree(p->_ECChomographyMatrix);
}

bool ImageRegistration::ECCImageAlignment(MIL_ID & srcImageColour, MIL_ID & templateImageColour, int imgChannel)
{
	MIL_ID templateImage = mtrx::to_mono(templateImageColour);//p->convertImage(templateImageColour, imgChannel);
	MIL_ID srcImage = mtrx::to_mono(srcImageColour);//p->convertImage(srcImageColour, imgChannel);
	cv::Mat im1 = p->MatroxtoCV_SingleChannel(templateImage);
	cv::Mat im2 = p->MatroxtoCV_SingleChannel(srcImage);

	if (im1.empty() || im2.empty())
	{
		MbufFree(templateImage);
		MbufFree(srcImage);
		p->_imageAlignmentType = AlignmentType::NONE;
		return false;
	}

	cv::Mat resizedIm1, resizedImg2;
	//resize Image to speed up
	cv::resize(im1, resizedIm1, cv::Size(), 0.2, 0.2);
	cv::resize(im2, resizedImg2, cv::Size(), 0.2, 0.2);

	// Define the motion model
	const int warp_mode = cv::MOTION_EUCLIDEAN;
	//const int warp_mode = cv::MOTION_TRANSLATION;
	//const int warp_mode = cv::MOTION_AFFINE;

	// Set a 2x3 or 3x3 warp matrix depending on the motion model.
	Mat warp_matrix;

	// Initialize the matrix to identity
	if (warp_mode == MOTION_HOMOGRAPHY)
		warp_matrix = Mat::eye(3, 3, CV_32F);
	else
		warp_matrix = Mat::eye(2, 3, CV_32F);

	// Specify the number of iterations.
	int number_of_iterations = 100;

	// Specify the threshold of the increment
	// in the correlation coefficient between two iterations
	double termination_eps = 0.001;

	// Define termination criteria
	TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);

	// Run the ECC algorithm. The results are stored in warp_matrix.
	try
	{
		findTransformECC(
			resizedIm1,
			resizedImg2,
			warp_matrix,
			warp_mode,
			criteria
		);
	}
	catch (Exception e)
	{
		MbufFree(templateImage);
		MbufFree(srcImage);
		p->_imageAlignmentType = AlignmentType::NONE;
		return false;
	}
	
	warp_matrix.at<float>(0, 2) = warp_matrix.at<float>(0, 2) * 5;
	warp_matrix.at<float>(1, 2) = warp_matrix.at<float>(1, 2) * 5;

	MIL_FLOAT *host;
	MIL_ID pitch = M_NULL;
	MbufInquire(p->_ECChomographyMatrix, M_HOST_ADDRESS, &host);
	MbufInquire(p->_ECChomographyMatrix, M_PITCH, &pitch);

	for (int y = 0; y < 3; y++)
	{
		for (int x = 0; x < 3; x++)
		{
			if (x == 0 & y == 2) {
				host[x + (y * pitch)] = 0;
				//cout << "matrix: " << host[x + (y * pitch)] << endl;
			}
			else if (x == 1 & y == 2) {
				host[x + (y * pitch)] = 0;
				//cout << "matrix: " << host[x + (y * pitch)] << endl;
			}
			else if (x == 2 & y == 2) {
				host[x + (y * pitch)] = 1;
				//cout << "matrix: " << host[x + (y * pitch)] << endl;
			}
			else
			{
				host[x + (y * pitch)] = warp_matrix.at<float>(y, x);
				//cout << "matrix: " << host[x + (y * pitch)] << endl;
			}
		}
	}

	MbufFree(templateImage);
	MbufFree(srcImage);
	p->_imageAlignmentType = AlignmentType::ECC;
	ct::logger::debug("Hello");
	return true;
}

bool ImageRegistration::ORBImageAlignment(MIL_ID & srcImageColour, MIL_ID & templateImageColour, int imgChannel, int distanceFilter, bool fast)
{
	//MIL_ID srcImage = p->convertImage(srcImageColour, imgChannel);
	//MIL_ID templateImage = p->convertImage(templateImageColour, imgChannel);

	//cv::Mat inputImg = p->MatroxtoCV_SingleChannel(srcImage);
	//cv::Mat templateImg = p->MatroxtoCV_SingleChannel(templateImage);

	//auto orb = cv::cuda::ORB::create(10000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20, false);

	////================================================================ initialize input Img ================================================================
	//auto inputKp = std::vector<cv::KeyPoint>();
	//auto inputImg_gpu = cv::cuda::GpuMat();
	//inputImg_gpu.upload(inputImg);
	//orb->detect(inputImg_gpu, inputKp);

	//auto inputDesc_gpu = cv::cuda::GpuMat();
	//orb->compute(inputImg_gpu, inputKp, inputDesc_gpu);

	//auto inputDesc = cv::Mat();
	//inputDesc_gpu.download(inputDesc);

	////debugging
	//auto inputImgKeypoint = cv::Mat();
	//if (false) cv::drawKeypoints(inputImg, inputKp, inputImgKeypoint, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
	//if (false) cv::imwrite("view4InputKeyPoints.jpg", inputImgKeypoint);
	////debugging

	////===================================================================== initialize template Img ================================================================
	//auto templateKp = std::vector<cv::KeyPoint>();
	//auto templateImg_gpu = cv::cuda::GpuMat();
	//templateImg_gpu.upload(templateImg);
	//orb->detect(templateImg_gpu, templateKp);

	//auto templateDesc_gpu = cv::cuda::GpuMat();
	//orb->compute(templateImg_gpu, templateKp, templateDesc_gpu);

	//auto templateDesc = cv::Mat();
	//templateDesc_gpu.download(templateDesc);

	////debugging
	//auto templateImgKeypoint = cv::Mat();
	//if (false) cv::drawKeypoints(templateImg, templateKp, templateImgKeypoint, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
	//if (false) cv::imwrite("view4TemplateKeyPoints.jpg", templateImgKeypoint);
	////debugging

	//std::vector< std::vector<cv::DMatch> > knn_matches;
	//std::vector<cv::DMatch> good_matches;

	//if (fast)
	//{
	//	//===================================================================== Brute Force Matching ================================================================
	//	auto matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
	//	matcher->knnMatch(inputDesc_gpu, templateDesc_gpu, knn_matches, 2, cv::cuda::GpuMat(), true);
	//	const float ratio_thresh = 0.85f; //-- Filter matches using the Lowe's ratio test	
	//									  //-- Filter matches using the Lowe's ratio test
	//	for (size_t i = 0; i < knn_matches.size(); i++)
	//	{
	//		if (knn_matches[i].size() >= 2)
	//		{
	//			if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
	//			{
	//				good_matches.push_back(knn_matches[i][0]);
	//			}
	//		}
	//	}
	//}
	//else
	//{
	//	//===================================================================== FLANN Matching ================================================================
	//	cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
	//	const float ratio_thresh = 0.85f; //-- Filter matches using the Lowe's ratio test	
	//	matcher.knnMatch(inputDesc, templateDesc, knn_matches, 2);
	//	for (size_t i = 0; i < knn_matches.size(); i++)
	//	{
	//		if (knn_matches[i].size() >= 2)
	//		{
	//			if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
	//			{
	//				good_matches.push_back(knn_matches[i][0]);
	//			}
	//		}

	//	}

	//}

	////debugging
	//if (false)
	//{
	//	//-- Draw matches
	//	cv::Mat img_matches;
	//	cv::drawMatches(inputImg, inputKp, templateImg, templateKp, good_matches, img_matches);
	//	//-- Show detected matches
	//	cv::imwrite("Good Matches.jpg", img_matches);
	//}
	////debugging

	//// Extract location of good matches
	//std::vector<cv::Point2f> points1, points2;
	//std::vector<cv::KeyPoint> keyPoints1, keyPoints2;
	//std::vector<cv::DMatch> new_Matches;
	//int count = 0;
	//for (size_t i = 0; i < good_matches.size(); i++)
	//{
	//	float distance = sqrt(powf(inputKp[good_matches[i].queryIdx].pt.x - templateKp[good_matches[i].trainIdx].pt.x, 2) +
	//		powf(inputKp[good_matches[i].queryIdx].pt.y - templateKp[good_matches[i].trainIdx].pt.y, 2));
	//	if (distanceFilter < 200) distanceFilter = 200;
	//	if (distance < distanceFilter)
	//	{
	//		keyPoints1.push_back(inputKp[good_matches[i].queryIdx]);
	//		keyPoints2.push_back(templateKp[good_matches[i].trainIdx]);
	//		points1.push_back(inputKp[good_matches[i].queryIdx].pt);
	//		points2.push_back(templateKp[good_matches[i].trainIdx].pt);
	//		cv::DMatch match;
	//		match = good_matches[i];
	//		match.queryIdx = count;
	//		match.trainIdx = count;
	//		new_Matches.push_back(match);
	//		count++;
	//	}
	//}
	//if (false)
	//{
	//	cv::Mat Newimg_matches;
	//	cv::drawMatches(inputImg, keyPoints1, templateImg, keyPoints2, new_Matches, Newimg_matches);
	//	cv::imwrite("New Matches.jpg", Newimg_matches);
	//}

	//if (points1.size() < 4 || points2.size() < 4)
	//{
	//	p->_imageAlignmentType = AlignmentType::NONE;
	//	if (srcImage) MbufFree(srcImage);
	//	if (templateImage) MbufFree(templateImage);
	//	return false;
	//}
	//// Find homography
	//p->_ORBhomographyMatrix = cv::Mat(3, 3, CV_32FC1);
	//p->_ORBhomographyMatrix = cv::findHomography(points1, points2, cv::RHO);
	////std::cout << "Estimated homography : \n" << h << std::endl;

	//if (p->_ORBhomographyMatrix.size().width != 3 || p->_ORBhomographyMatrix.size().height != 3)
	//{
	//	p->_imageAlignmentType = AlignmentType::NONE;
	//	if (srcImage) MbufFree(srcImage);
	//	if (templateImage) MbufFree(templateImage);
	//	return false;
	//}

	//p->_imageAlignmentType = AlignmentType::ORB;
	//if (srcImage) MbufFree(srcImage);
	//if (templateImage) MbufFree(templateImage);
	return true;
}

MIL_ID ImageRegistration::warpImage(MIL_ID & inputImage, MIL_ID & templateImage)
{
	if (p->_imageAlignmentType == AlignmentType::NONE) return M_NULL;
	else if (p->_imageAlignmentType == AlignmentType::ECC)
	{
		if (!p->_ECChomographyMatrix) return M_NULL;

		int width = MbufInquire(templateImage, M_SIZE_X, M_NULL);
		int height = MbufInquire(templateImage, M_SIZE_Y, M_NULL);
		int templateSizeBand = MbufInquire(templateImage, M_SIZE_BAND, M_NULL);
		int inputSizeBand = MbufInquire(inputImage, M_SIZE_BAND, M_NULL);
		if (templateSizeBand != inputSizeBand) return M_NULL;

		MIL_ID Output = MbufAllocColor(M_DEFAULT, inputSizeBand, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MbufCopy(templateImage, Output);

		MimWarp(inputImage, Output, p->_ECChomographyMatrix, M_NULL, M_WARP_POLYNOMIAL, M_BILINEAR + M_OVERSCAN_DISABLE);
		cv::Point2f point = cv::Point2f(0,0);
		p->transformPoint(p->_ECChomographyMatrix, point);
		return Output;
	}
	else if (p->_imageAlignmentType == AlignmentType::ORB)
	{
		int width = MbufInquire(templateImage, M_SIZE_X, M_NULL);
		int height = MbufInquire(templateImage, M_SIZE_Y, M_NULL);
		int templateSizeBand = MbufInquire(templateImage, M_SIZE_BAND, M_NULL);
		int inputSizeBand = MbufInquire(inputImage, M_SIZE_BAND, M_NULL);
		if (templateSizeBand != inputSizeBand) return M_NULL;
		MIL_ID Output = MbufAllocColor(M_DEFAULT, inputSizeBand, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

		cv::Mat inputColourImg = p->MatroxToCV(inputImage);
		cv::Mat imgColourReg = p->MatroxToCV(templateImage);

		warpPerspective(inputColourImg, imgColourReg, p->_ORBhomographyMatrix, inputColourImg.size(), cv::INTER_CUBIC, cv::BORDER_TRANSPARENT);
		p->CVtoMatrox(imgColourReg, Output);
		cv::Point2f point = cv::Point2f(0, 0);
		p->transformPoint(p->_ORBhomographyMatrix, point);
		return Output;
	}
	return M_NULL;
}

bool ImageRegistration::getOffset(double & offset_x, double & offset_y)
{
	offset_x = p->offset_x;
	offset_y = p->offset_y;
	return true;
}

MIL_ID ImageRegistration::Impl::convertImage(MIL_ID & srcColour, int imageChannel)
{	
	int width = MbufInquire(srcColour, M_SIZE_X, M_NULL);
	int height = MbufInquire(srcColour, M_SIZE_Y, M_NULL);
	MIL_ID dest = MbufAllocColor(M_DEFAULT, 1, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

	MIL_ID avg = MbufAllocColor(M_DEFAULT, 1, width, height, 32 + M_FLOAT, M_IMAGE + M_PROC, M_NULL);
	MIL_ID red = MbufAllocColor(M_DEFAULT, 1, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MIL_ID blue = MbufAllocColor(M_DEFAULT, 1, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MIL_ID green = MbufAllocColor(M_DEFAULT, 1, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MbufCopyColor(srcColour, red, M_RED);
	MbufCopyColor(srcColour, blue, M_BLUE);
	MbufCopyColor(srcColour, green, M_GREEN);

	if (imageChannel == (int)ImageType::GRAYSCALE) {
		MimArith(red, green, avg, M_ADD + M_SATURATION);
		MimArith(avg, blue, avg, M_ADD + M_SATURATION);
		MimArith(avg, 3, avg, M_DIV_CONST);
		MbufCopy(avg, dest);
	}
	else if (imageChannel == (int)ImageType::BLUE1GREEN2)
	{
		MimArith(green, green, avg, M_ADD + M_SATURATION);
		MimArith(avg, blue, avg, M_ADD + M_SATURATION);
		MimArith(avg, 3, avg, M_DIV_CONST);
		MbufCopy(avg, dest);
	}
	else if (imageChannel == (int)ImageType::BLUE2GREEN1)
	{
		MimArith(green, blue, avg, M_ADD + M_SATURATION);
		MimArith(avg, blue, avg, M_ADD + M_SATURATION);
		MimArith(avg, 3, avg, M_DIV_CONST);
		MbufCopy(avg, dest);
	}
	else if (imageChannel == (int)ImageType::BLUE)
	{
		MbufCopy(blue, dest);
	}
	else if (imageChannel == (int)ImageType::RED)
	{
		MbufCopy(red, dest);
	}
	else if (imageChannel == (int)ImageType::RED_MINUS_BLUE)
	{
		MimArith(red, blue, dest, M_SUB + M_SATURATION);
	}

	MbufFree(avg);
	MbufFree(red);
	MbufFree(blue);
	MbufFree(green);
	return dest;
}

cv::Mat ImageRegistration::Impl::MatroxtoCV_SingleChannel(MIL_ID & imgPtr)
{
	if (imgPtr == M_NULL) return cv::Mat();
	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MbufInquire(imgPtr, M_SIZE_X, &SizeX);
	MbufInquire(imgPtr, M_SIZE_Y, &SizeY);

	MIL_UINT8 *host;
	MIL_ID pitch = M_NULL;
	MbufInquire(imgPtr, M_HOST_ADDRESS, &host);
	MbufInquire(imgPtr, M_PITCH, &pitch);

	Mat Image(SizeY, SizeX, CV_8UC1);

	int intensity;

	for (int y = 0; y < SizeY; y++)
	{
		for (int x = 0; x < SizeX; x++)
		{
			intensity = host[x + (y * pitch)];
			Image.at<uchar>(Point(x, y)) = intensity;
		}
	}
	return  Image;
}

cv::Mat ImageRegistration::Impl::MatroxToCV(MIL_ID & imgPtr) {
	if (imgPtr == M_NULL) return cv::Mat();

	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MIL_INT SizeC = 0;
	MbufInquire(imgPtr, M_SIZE_X, &SizeX);
	MbufInquire(imgPtr, M_SIZE_Y, &SizeY);
	MbufInquire(imgPtr, M_SIZE_BAND, &SizeC);

	//std::cout << "SIZE C: " << (int)SizeC << std::endl;

	MIL_UINT8 *host;
	MIL_ID pitch = M_NULL;
	MbufInquire(imgPtr, M_HOST_ADDRESS, &host);
	MbufInquire(imgPtr, M_PITCH, &pitch);

	MIL_ID mRed = M_NULL, mGreen = M_NULL, mBlue = M_NULL;
	MIL_UINT8 *hostRed, *hostGreen, *hostBlue;
	MIL_ID pitR = M_NULL, pitG = M_NULL, pitB = M_NULL;

	mRed = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, M_NULL);
	mGreen = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, M_NULL);
	mBlue = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, M_NULL);

	MbufCopyColor(imgPtr, mRed, M_RED);
	MbufCopyColor(imgPtr, mGreen, M_GREEN);
	MbufCopyColor(imgPtr, mBlue, M_BLUE);

	MbufInquire(mRed, M_HOST_ADDRESS, &hostRed);
	MbufInquire(mRed, M_PITCH, &pitR);

	MbufInquire(mGreen, M_HOST_ADDRESS, &hostGreen);
	MbufInquire(mGreen, M_PITCH, &pitG);

	MbufInquire(mBlue, M_HOST_ADDRESS, &hostBlue);
	MbufInquire(mBlue, M_PITCH, &pitB);

	int type_ = ((int)SizeC == 3) ? CV_8UC3 : CV_8UC1;
	cv::Mat_<cv::Vec3b> Image(SizeY, SizeX, type_);
	//MbufSave(MIL_TEXT("data/debug/OCR/matrox.tiff"), imgPtr);

	int intensity_red;
	int intensity_green;
	int intensity_blue;

	//std::cout << "Image Size: (" << Image.rows << "," << Image.cols << "," << Image.channels() << ") h x w x c" << std::endl;

	for (int y = 0; y < SizeY; y++)
	{
		for (int x = 0; x < SizeX; x++)
		{
			intensity_red = hostRed[x + (y * pitR)];
			intensity_green = hostGreen[x + (y * pitG)];
			intensity_blue = hostBlue[x + (y * pitB)];

			//std::cout << SizeX << " : "  << x << " | " << SizeY << " : " << y;
			Image(y, x)[0] = intensity_blue;
			//std::cout << " | 1";

			Image(y, x)[1] = intensity_green;
			//std::cout << "2";

			Image(y, x)[2] = intensity_red;
			//std::cout << "3";
			//std::cout << " | Pass" << std::endl;
		}
	}
	cv::Mat img = Image;
	//cv::imwrite("data/debug/OCR/cv.png", img);

	if (mRed) MbufFree(mRed);
	if (mGreen) MbufFree(mGreen);
	if (mBlue) MbufFree(mBlue);
	return img;
}

void ImageRegistration::Impl::CVtoMatrox(cv::Mat input, MIL_ID & output)
{
	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MIL_INT BandSize = 0;
	MbufInquire(output, M_SIZE_X, &SizeX);
	MbufInquire(output, M_SIZE_Y, &SizeY);
	MbufInquire(output, M_SIZE_BAND, &BandSize);

	if (BandSize != 3) return;
	if (SizeX != input.size().width || SizeY != input.size().height) return;

	MIL_ID mRed = M_NULL, mGreen = M_NULL, mBlue = M_NULL;
	MIL_UINT8 *hostRed, *hostGreen, *hostBlue;
	MIL_ID pitR = M_NULL, pitG = M_NULL, pitB = M_NULL;

	mRed = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, M_NULL);
	mGreen = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, M_NULL);
	mBlue = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, M_NULL);

	MbufCopyColor(output, mRed, M_RED);
	MbufCopyColor(output, mGreen, M_GREEN);
	MbufCopyColor(output, mBlue, M_BLUE);

	MbufInquire(mRed, M_HOST_ADDRESS, &hostRed);
	MbufInquire(mRed, M_PITCH, &pitR);

	MbufInquire(mGreen, M_HOST_ADDRESS, &hostGreen);
	MbufInquire(mGreen, M_PITCH, &pitG);

	MbufInquire(mBlue, M_HOST_ADDRESS, &hostBlue);
	MbufInquire(mBlue, M_PITCH, &pitB);

	int intensity_red;
	int intensity_green;
	int intensity_blue;

	//std::cout << "Image Size: (" << Image.rows << "," << Image.cols << "," << Image.channels() << ") h x w x c" << std::endl;

	for (int y = 0; y < SizeY; y++)
	{
		for (int x = 0; x < SizeX; x++)
		{
			cv::Vec3b colour = input.at<cv::Vec3b>(cv::Point(x, y));
			intensity_blue = colour.val[2];
			intensity_green = colour.val[1];
			intensity_red = colour.val[0];

			hostRed[x + (y * pitR)] = intensity_blue;
			hostGreen[x + (y * pitG)] = intensity_green;
			hostBlue[x + (y * pitB)] = intensity_red;
		}
	}
	MbufCopyColor2d(mRed, output, 0, 0, 0, M_RED, 0, 0, SizeX, SizeY);
	MbufCopyColor2d(mGreen, output, 0, 0, 0, M_GREEN, 0, 0, SizeX, SizeY);
	MbufCopyColor2d(mBlue, output, 0, 0, 0, M_BLUE, 0, 0, SizeX, SizeY);

	if (mRed) MbufFree(mRed);
	if (mGreen) MbufFree(mGreen);
	if (mBlue) MbufFree(mBlue);
}

void ImageRegistration::Impl::transformPoint(cv::Mat & homography, cv::Point2f & point)
{

	cv::Point2f newPoint;

	cv::Mat inverse = homography.inv();

	std::vector<cv::Point2f> dstPoints, srcPoints;
	dstPoints.push_back(cv::Point2f(point.x, point.y));
	cv::perspectiveTransform(dstPoints, srcPoints, inverse);

	if (srcPoints.size() > 0)
	{
		newPoint.x = srcPoints[0].x;
		newPoint.y = srcPoints[0].y;
	}
	else
	{
		newPoint.x = point.x;
		newPoint.y = point.y;
	}

	offset_x = newPoint.x;
	offset_y = newPoint.y;
}

void ImageRegistration::Impl::transformPoint(MIL_ID transMat, cv::Point2f & point)
{
	MIL_FLOAT *host;
	MIL_ID pitch = M_NULL;
	MbufInquire(transMat, M_HOST_ADDRESS, &host);
	MbufInquire(transMat, M_PITCH, &pitch);

	cv::Point2f newPoint;
	newPoint.x = host[0 + (0 * pitch)] * point.x + host[1 + (0 * pitch)] * point.x + host[2 + (0 * pitch)];
	newPoint.y = host[0 + (1 * pitch)] * point.y + host[1 + (1 * pitch)] * point.y + host[2 + (1 * pitch)];

	offset_x = newPoint.x;
	offset_y = newPoint.y;
}