#pragma once
//#include <opencv2/opencv.hpp> //NOTE: reorder this to the first include will solve ambiguos symbol error
#include <chrono>
#include "mil.h"
#include <QImage>
#include <QPixmap>
#include <QFileInfo>
#include <QSet>
#include "Eigen/Core"
#include "EM_Core.h"
#include "EM_Vector.h"
#include "QView.h"
#include "ScopedTimeLogger.h"
#include "TimeLogger.h"
#include <opencv2/opencv.hpp>
#include <unordered_set>
#include <vips/vips8>
#include "libvips/VImage8.h"

//#define M_PI 3.14159

namespace util {

	//ID
	QString getRowColID(int row, int col);
	bool getRowColFromID(const QString& id, int& row, int& col);

	int getOpticsSize(const QHash<QString, OpticsInfo3D>& optics);

	bool is_equal(const double& value1, const double& value2, double allowable_error = 0.001);

	//conversion
	double px_to_mm(const double& px, const double& scale);
	double px_to_um(const double& px, const double& scale);
	double mm_to_px(const double& mm, const double& scale);
	double um_to_px(const double& um, const double& scale);

	double scale(double n, double inMin, double inMax, double outMin, double outMax);

	std::string num_to_str(double value, int integer_length, int decimal_length, bool no_negative = false);
	double str_to_num(std::string value, int integer_length, int decimal_length);

	class Timer {
	private:
		std::chrono::time_point<std::chrono::system_clock, std::chrono::system_clock::duration> _start;

	public:
		Timer() { start(); }
		void start() { _start = std::chrono::system_clock::now(); }
		double current() const {
			auto d = std::chrono::system_clock::now() - _start;
			return std::chrono::duration<double, std::milli>(d).count();
		}
		double stop() const { return current(); }

	};

	class CornerFinder {
	public:
		struct Point {
			std::string id = "";
			double x = 0.0;
			double y = 0.0;
		};

		Point topleft();
		Point topright();
		Point btmright();
		Point btmleft();
		void add(Point point);
		void clear();

	private:
		std::vector<Point> points;
	};

	//histogramMatching
	class ImagePreprocess {
	public:
		ImagePreprocess(QString refImageFilePath = QString(), QString maskImageFilePath = QString());  // imageType: 0 QImage, 1 MIL
		~ImagePreprocess();

		// median kernel is times two of imageJ
		void Diff_of_medianFilter(QImage& qImg, QImage & output,  int brightThreshold = 30, int darkThreshold = 20, int imgType = 1, int median1 = 3, int median2 = 9, int darkMedian1 = 9, int darkMedian2 = 15, bool findDifference = false, int diffThreshold = 40);
		void Diff_of_medianFilter(MIL_ID & milImg, MIL_ID &output, int brightThreshold = 30, int darkThreshold = 20, int imgType = 1, int median1 = 3, int median2 = 9, int darkMedian1 = 9, int darkMedian2 = 15, bool findDifference = false, int diffThreshold = 40);
		void performHistMatching(QImage & qImg, QImage & output);
		void performHistMatching(MIL_ID & milImg, MIL_ID &output);
		void HighlightDarkDefects(QImage & qImg, QImage & output, int darkThreshold = 40, int differenceThreshold = 40, int brightDifferenceThreshold = 60);
		void HighlightDarkDefects(MIL_ID & milImg, MIL_ID & output, int darkThreshold = 40, int darkDifferenceThreshold = 40, int brightDifferenceThreshold = 40);
		void medianFilter(QImage& qImg, QImage & output, int imgType = 0, int median = 9);
		void medianFilter(MIL_ID & milImg, MIL_ID & output, int imgType = 0, int median = 9);
		int width();
		int height();

	private:
		MIL_ID _milRefImg = M_NULL;
		MIL_ID _milMaskImg = M_NULL;
		cv::Mat _refImage;
		cv::Mat _maskImage;
		int _refWidth = 0;
		int _refHeight = 0;
	};

	QString type2str(int type);
	void QImagetoUnsignedChar(const QImage& img, unsigned char* red, unsigned char* green, unsigned char* blue);
	void QImagetoUnsignedChar(const QImage& img, unsigned char** buf);
	void miltoUnsignedChar(MIL_ID & img, unsigned char** red, unsigned char** green, unsigned char** blue);
	void miltoAllocatedUnsignedChar(MIL_ID& img, unsigned char* red, unsigned char* green, unsigned char* blue);
	void miltoAllocatedUnsignedChar(MIL_ID& img, unsigned char* buf);
	void miltoUnsignedChar(MIL_ID & img, unsigned char** buf);
	void miltoUnsignedShort(MIL_ID& img, unsigned short* buf);
	void miltoAllocatedUnsignedShort(MIL_ID& img, unsigned short* buf);
	void cv_to_Mil(const cv::Mat cvImg, MIL_ID & milImg);
	void Mil_to_cv(const MIL_ID & milImg, cv::Mat & cvImg);
	void qImg_to_Mil(const QImage & qImg, MIL_ID & milImg);
	void Mil_to_qImg(const MIL_ID & milImg, QImage & qImg);
	void Mil_to_cv(const MIL_ID & milImg, cv::Mat & cvImg);
	cv::Mat QImageToCvMat(const QImage& inImage, bool inCloneImageData = true);
	void free_ptr(unsigned char* p);
	void free_ptr(unsigned short* p);
	//void to_CV(QImage& input, cv::Mat& output);
	//void to_qimg(cv::Mat& input, QImage& output);

	vips::VImage to_vimage(MIL_ID mBuf, void** data); //duration: 2ms, sample: 1024x1024
	MIL_ID to_milID(vips::VImage vimg); //duration: 3ms, sample: 1024x1024
	
	void drawCross(int cx, int cy, int size, QImage& qimg);
	QString combineID(QString viewID, QString opticID);
	QPixmap rotatePixmap(const QPixmap& img, double degree);
	QImage rotateQImage(const QImage& img, double degree);
	void rotateImage(unsigned char* image, int width, int height, double rotationAngle);
	MIL_ID convertBayerToRGB(unsigned char* image, int width, int height, MIL_INT64 type);
	void formQRGB(QImage& img, const unsigned char* pRedBuf, const unsigned char* pGreenBuf, const unsigned char* pBlueBuf, const unsigned char* pAlphaBuf);
	unsigned char* generateAlphaImage(int width, int height, double rotationAngle); //remember to delete the alpha image after usage
	void UnsignedChar_to_Mil(int width, int height, unsigned char* red, unsigned char* green, unsigned char* blue, MIL_ID & destination);
	void UnsignedChar_to_Mil(int width, int height, unsigned char* mono, MIL_ID & destination);
	void UnsignedShort_to_Mil(int width, int height, unsigned short* red, unsigned short* green, unsigned short* blue, MIL_ID & destination);
	void UnsignedShort_to_Mil(int width, int height, unsigned short* mono, MIL_ID & destination);

	QString addSuffix(const QString& filePath, const QString& suffix);
	QString changeExtension(const QString& filePath, const QString& newExtension);

	void offsetBufferValue(unsigned char* buf, int w, int h, int channel, int value);

	bool isPNG(const QString imgPath);
	bool isBMP(const QString imgPath);

	QString convert_to_BMP_ext(const QString imgPath);

	bool isRAMOver(double percentage);

	void saveCroppedMilImg(MIL_ID & src, QRect rect, QString & filePath);
	QImage MatroxtoQImage(MIL_ID & src);
	QVector<double> getNumsFromString(QString);

	void createFolder(const QString& path);
	void copyTo(const QString& sourcePath, const QString& destinationPath);
	void renameFileWithTimestamp(const QString& path);
	void copyRecipe(const QString& sourceDir, const QString& destinationDir, const QStringList& excludeList = {}, bool addBBA = false);

	//DBSCAN
	enum class DBType { CORE, BORDER, NOISE, UNASSIGNED, CLUSTERED };

	struct DBRect {
		QRectF rect; //user only need to input this
		DBType type; 
		QSet<QString> neighbours;
		int clusterIndex = 0;

		friend bool operator==(const DBRect& lhs, const DBRect& rhs) {
			return lhs.rect == rhs.rect;
		}
	};

	double distance(const DBRect& db1, const DBRect& db2);
	std::vector<std::vector<QString>> dbscan(QHash<QString, DBRect>& dbrects, double eps, int minPts);

	// Template function to calculate the median of a sorted vector
	template<typename T>
	T calculateMedian(std::vector<T>& vec) {
		size_t size = vec.size();
		if (size % 2 == 0) {
			return (vec[size / 2 - 1] + vec[size / 2]) / 2.0;
		}
		else {
			return vec[size / 2];
		}
	}

	// Template function to remove outliers from a vector using the IQR method
	template<typename T>
	void removeOutliers(std::vector<T>& vec) {
		if (vec.size() < 4) {
			// Not enough data to calculate quartiles
			return;
		}

		// Sort the vector
		std::sort(vec.begin(), vec.end());

		// Calculate Q1 (first quartile)
		std::vector<T> lowerHalf(vec.begin(), vec.begin() + vec.size() / 2);
		T Q1 = calculateMedian(lowerHalf);

		// Calculate Q3 (third quartile)
		std::vector<T> upperHalf(vec.size() % 2 == 0 ? vec.begin() + vec.size() / 2 : vec.begin() + vec.size() / 2 + 1, vec.end());
		T Q3 = calculateMedian(upperHalf);

		// Calculate IQR
		T IQR = Q3 - Q1;

		// Determine the bounds for outliers
		T lowerBound = Q1 - 1.5 * IQR;
		T upperBound = Q3 + 1.5 * IQR;

		// Remove outliers
		vec.erase(std::remove_if(vec.begin(), vec.end(), [lowerBound, upperBound](T x) {
			return x < lowerBound || x > upperBound;
		}), vec.end());
	}

	class ComboScrollBlocker : public QObject {
		bool eventFilter(QObject* obj, QEvent* event) override {
			if (event->type() == QEvent::Wheel) {
				return true; // block wheel scroll
			}
			return QObject::eventFilter(obj, event);
		}
	};
}

namespace algo {
	template <typename T>
	void swap(T* a, T* b) {
		T t = *a;
		*a = *b;
		*b = t;
	}

	template <typename T>
	void bubbleSort(std::vector<T> arr, bool ascending) {
		int i, j;
		bool swapped;

		int n = arr.size();

		for (i = 0; i < n - 1; i++)
		{
			swapped = false;
			for (j = 0; j < n - i - 1; j++)
			{
				if (ascending)
				{
					if (arr[j] > arr[j + 1])
					{
						swap(&arr[j], &arr[j + 1]);
						swapped = true;
					}
				}
				else
				{
					if (arr[j] < arr[j + 1])
					{
						swap(&arr[j], &arr[j + 1]);
						swapped = true;
					}
				}
			}

			//If no two elements were swapped by inner loop, then break 
			if (swapped == false) break;
		}
	}

	template <typename T>
	void bubbleSort(const std::vector<T>& arr, std::vector<int>& index, bool ascending) {
		int i, j;
		bool swapped;
		std::vector<T> temp;

		for (int k = 0; k < arr.size(); k++)
		{
			index.emplace_back(k);
			temp.emplace_back(arr[k]);
		}

		for (i = 0; i < arr.size() - 1; i++)
		{
			swapped = false;
			for (j = 0; j < arr.size() - i - 1; j++)
			{
				if (ascending)
				{
					if (temp[j] > temp[j + 1])
					{
						swap(&temp[j], &temp[j + 1]);
						swap(&index[j], &index[j + 1]);
						swapped = true;
					}
				}
				else
				{
					if (temp[j] < temp[j + 1])
					{
						swap(&temp[j], &temp[j + 1]);
						swap(&index[j], &index[j + 1]);
						swapped = true;
					}
				}
			}

			//If no two elements were swapped by inner loop, then break 
			if (swapped == false) break;
		}
	}


	//Circles
	enum CircleType {
		LARGEST_RADIUS, SMALLEST_RADIUS
	};

	struct Circle {
		double radius;
		double x, y, cx, cy;
	};

	/*bool find_circle(Circle& circle, const cv::Mat& img, double min_radius, double max_radius, CircleType type);
	void find_circles(std::vector<Circle>& circles, const cv::Mat& img, double min_radius, double max_radius);*/
}


//Temp
namespace em {

	enum class RotationSequence {
		RXRYRZ, RXRZRY, RYRXRZ, RYRZRX, RZRXRY, RZRYRX
	};

	M3d get_rotation_matrix(double rx, double ry, double rz, RotationSequence rs);
}