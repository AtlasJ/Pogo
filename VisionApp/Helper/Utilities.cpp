#include "Utilities.h"
#include "mtrx.h"
#include <QDebug>
#include <QPainter>
#include <QRegularExpression>
#include "Logger.h"
#include <cmath>
#include <QDateTime>
#include <QDir>

using namespace util;

QString util::getRowColID(int row, int col)
{
	return QString("R%1C%2").arg(row).arg(col);
}

bool util::getRowColFromID(const QString& id, int& row, int& col)
{
	if (id.contains("R") && id.contains("C")) {
		QRegularExpression regex(R"(R(\d+)C(\d+))");
		QRegularExpressionMatch match = regex.match(id);

		if (match.hasMatch()) {
			// Extract the row and column numbers
			row = match.captured(1).toInt(); // First captured group
			col = match.captured(2).toInt(); // Second captured group
		}
	}

	return false;
}

int util::getOpticsSize(const QHash<QString, OpticsInfo3D>& optics)
{
	int numOptics = 0;
	for (auto o : optics) {
		numOptics++;
		if (o.exposureMode == ct::s_parallel) {
			numOptics++; //parallel will have extra one optics
		}
	}

	return numOptics;
}

bool util::is_equal(const double & value1, const double & value2, double allowable_error)
{
	if (abs(value1 - value2) > allowable_error) {
		return false;
	}
	return true;
}

double util::px_to_mm(const double & px, const double & scale)
{
	return px * scale / 1000;
}

double util::px_to_um(const double & px, const double & scale)
{
	return px * scale;
}

double util::mm_to_px(const double & mm, const double & scale)
{
	return mm / scale * 1000;
}

double util::um_to_px(const double & um, const double & scale)
{
	return um / scale;
}

double util::scale(double n, double inMin, double inMax, double outMin, double outMax)
{
	return (n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

std::string util::num_to_str(double value, int integer_length, int decimal_length, bool no_negative)
{
	if (no_negative) {
		value = abs(value);
	}

	if (value < 0) integer_length++;

	int ideal_length = integer_length + decimal_length;
	int multiplier = std::pow(10, decimal_length);
	int output = value * multiplier;
	std::string output_s = std::to_string(output);

	int length = output_s.length();
	while (length != ideal_length) {
		if (length < ideal_length) {
			output_s.insert(0, "0"); //insert at 1st pos char '0'
		}
		else {
			output_s.erase(0, 1); //remove (1st char, 1 char) 
		}

		length = output_s.length();
	}

	QString s = output_s.c_str();
	int hyphenIndex = s.indexOf('-');
    s = s.mid(hyphenIndex);

	return s.toStdString();
}

double util::str_to_num(std::string value, int integer_length, int decimal_length)
{
	int ideal_length = integer_length + decimal_length;
	int multiplier = std::pow(10, decimal_length);

	while (value.length() > ideal_length) {
		value.erase(0, 1); //remove (1st char, 1 char) 
	}

	return std::stod(value) / multiplier;
}

QString util::type2str(int type)
{
	QString r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

void util::QImagetoUnsignedChar(const QImage & img, unsigned char * red, unsigned char * green, unsigned char * blue)
{
	int width = img.width();
	int height = img.height();

	// Allocate memory for monoChannel images
	red = new unsigned char[width * height];
	green = new unsigned char[width * height];
	blue = new unsigned char[width * height];

	// Copy red, green, and blue channels to monoChannel images
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			QRgb pixel = img.pixel(x, y);
			(red)[y * width + x] = qRed(pixel);
			(green)[y * width + x] = qGreen(pixel);
			(blue)[y * width + x] = qBlue(pixel);
		}
	}

}

void util::QImagetoUnsignedChar(const QImage & img, unsigned char ** buf)
{
	int width = img.width();
	int height = img.height();

	// Allocate memory for monoChannel images
	*buf = new unsigned char[width * height];

	// Copy red, green, and blue channels to monoChannel images
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			QRgb pixel = img.pixel(x, y);
			(*buf)[y * width + x] = qGray(pixel);
		}
	}
}

void util::miltoUnsignedChar(MIL_ID & img, unsigned char ** red, unsigned char ** green, unsigned char ** blue)
{
	MIL_INT width, height;
	MbufInquire(img, M_SIZE_X, &width);
	MbufInquire(img, M_SIZE_Y, &height);

	// Allocate memory for monoChannel images
	*red = new unsigned char[width * height];
	*green = new unsigned char[width * height];
	*blue = new unsigned char[width * height];

	//referenceImg
	MIL_UINT8* RedSrcPtr, *GreenSrcPtr, *BlueSrcPtr;
	MIL_ID PitchPtr;
	MbufInquire(img, M_HOST_ADDRESS_BAND + 0, &RedSrcPtr);
	MbufInquire(img, M_HOST_ADDRESS_BAND + 1, &GreenSrcPtr);
	MbufInquire(img, M_HOST_ADDRESS_BAND + 2, &BlueSrcPtr);
	MbufInquire(img, M_PITCH, &PitchPtr);

	// Copy red, green, and blue channels to monoChannel images
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {

			int R = RedSrcPtr[x + (y * PitchPtr)];
			int G = GreenSrcPtr[x + (y * PitchPtr)];
			int B = BlueSrcPtr[x + (y * PitchPtr)];
			(*red)[y * width + x] = R;
			(*green)[y * width + x] = G;
			(*blue)[y * width + x] = B;
		}
	}
}

void util::miltoAllocatedUnsignedChar(MIL_ID& img, unsigned char* red, unsigned char* green, unsigned char* blue)
{
	MbufGetColor(img, M_SINGLE_BAND, M_RED, red);
	MbufGetColor(img, M_SINGLE_BAND, M_GREEN, green);
	MbufGetColor(img, M_SINGLE_BAND, M_BLUE, blue);
}

void util::miltoAllocatedUnsignedChar(MIL_ID& img, unsigned char* buf)
{
	MbufGet(img, buf);
}

void util::miltoUnsignedChar(MIL_ID & img, unsigned char ** buf)
{
	MIL_INT width, height, bandSize;
	MbufInquire(img, M_SIZE_X, &width);
	MbufInquire(img, M_SIZE_Y, &height);
	MbufInquire(img, M_SIZE_BAND, &bandSize);

	if (bandSize == 1)
	{
		// Allocate memory for monoChannel images
		*buf = new unsigned char[width * height];

		MIL_UINT8* SrcPtr;
		MIL_ID PitchPtr;
		MbufInquire(img, M_HOST_ADDRESS, &SrcPtr);
		MbufInquire(img, M_PITCH, &PitchPtr);

		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				(*buf)[y * width + x] = SrcPtr[x + (y * PitchPtr)];
			}
		}
	}
	else if (bandSize == 3)
	{

	}
}

void util::miltoUnsignedShort(MIL_ID& img, unsigned short* buf)
{
	auto width = mtrx::get_width(img);
	auto height = mtrx::get_height(img);
	buf = new unsigned short[width * height];
	MbufGet(img, buf);
}

void util::miltoAllocatedUnsignedShort(MIL_ID& img, unsigned short* buf)
{
	MbufGet(img, buf);
}

void util::cv_to_Mil(const cv::Mat cvImg, MIL_ID & milImg)
{
	ct::logger::trace("Start - cv to mil");
	if (milImg)
	{
		MbufFree(milImg);
		milImg = M_NULL;
	}

	if (type2str(cvImg.type()) == "16UC1")
	{
		milImg = MbufAlloc2d(M_DEFAULT, cvImg.cols, cvImg.rows, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MbufPut(milImg, cvImg.data);
	}
	else if(cvImg.channels() == 3)
	{
		milImg = MbufAllocColor(M_DEFAULT, 3, cvImg.cols, cvImg.rows, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MbufPutColor(milImg, M_PACKED + M_BGR24, M_ALL_BANDS, cvImg.data);
	}
	else if (cvImg.channels() == 1)
	{
		milImg = MbufAlloc2d(M_DEFAULT, cvImg.cols, cvImg.rows, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MbufPut(milImg, cvImg.data);
	}
	ct::logger::trace("End - cv to mil");
}

void util::Mil_to_cv(const MIL_ID & milImg, cv::Mat & cvImg)
{
	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MIL_INT BandSize = 0;
	MIL_INT type = mtrx::get_type(milImg);
	MbufInquire(milImg, M_SIZE_X, &SizeX);
	MbufInquire(milImg, M_SIZE_Y, &SizeY);
	MbufInquire(milImg, M_SIZE_BAND, &BandSize);

	if (type == 16)
	{
		cvImg = cv::Mat(SizeY, SizeX, CV_16UC1);
		MbufGet(milImg, cvImg.data);
	}
	else if (BandSize == 1)
	{
		cvImg = cv::Mat(SizeY, SizeX, CV_8UC1);
		MbufGet(milImg, cvImg.data);
	}
	else if (BandSize == 3)
	{
		cvImg = cv::Mat(SizeY, SizeX, CV_8UC3);
		MbufGetColor(milImg, M_PACKED + M_BGR24, M_ALL_BANDS, cvImg.data);
	}
}

cv::Mat util::QImageToCvMat(const QImage& inImage, bool inCloneImageData)
{
	switch (inImage.format())
	{
		// 8-bit, 4 channel
	case QImage::Format_ARGB32:
	case QImage::Format_ARGB32_Premultiplied:
	{
		cv::Mat mat(inImage.height(), inImage.width(),
			CV_8UC4,
			const_cast<uchar*>(inImage.bits()),
			static_cast<size_t>(inImage.bytesPerLine()));

		return (inCloneImageData ? mat.clone() : mat);
	}

	// 8-bit, 3 channel
	case QImage::Format_RGB32:
	{
		cv::Mat mat(inImage.height(), inImage.width(),
			CV_8UC4,
			const_cast<uchar*>(inImage.bits()),
			static_cast<size_t>(inImage.bytesPerLine()));

		cv::Mat matNoAlpha;
		cv::cvtColor(mat, matNoAlpha, cv::COLOR_BGRA2BGR); // Drop the alpha channel

		return (inCloneImageData ? matNoAlpha.clone() : matNoAlpha);
	}

	case QImage::Format_RGB888:
	{
		QImage swapped = inImage.rgbSwapped();
		return cv::Mat(swapped.height(), swapped.width(),
			CV_8UC3,
			const_cast<uchar*>(swapped.bits()),
			static_cast<size_t>(swapped.bytesPerLine())).clone();
	}

	// 8-bit, 1 channel
	case QImage::Format_Indexed8:
	{
		cv::Mat mat(inImage.height(), inImage.width(),
			CV_8UC1,
			const_cast<uchar*>(inImage.bits()),
			static_cast<size_t>(inImage.bytesPerLine()));

		return (inCloneImageData ? mat.clone() : mat);
	}

	default:
		qWarning() << "QImage format not handled in switch:" << inImage.format();
		break;
	}

	return cv::Mat();
}

void util::free_ptr(unsigned char* p)
{
	if (p != nullptr) {
		delete[] p;
		p = nullptr;
	}
}

void util::free_ptr(unsigned short* p)
{
	if (p != nullptr) {
		delete[] p;
		p = nullptr;
	}
}

void util::qImg_to_Mil(const QImage & qImg, MIL_ID & milImg)
{
	ct::logger::trace("Start - qimage to mil");
	if (qImg.format() == QImage::Format_RGB32)
	{
		milImg = MbufAllocColor(M_DEFAULT, 3, qImg.width(), qImg.height(), 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MbufPutColor(milImg, M_PACKED + M_BGR32, M_ALL_BANDS, qImg.bits());

	}
	else if (qImg.format() == QImage::Format_Grayscale8)
	{
		milImg = MbufAlloc2d(M_DEFAULT, qImg.width(), qImg.height(), 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MbufPut(milImg, qImg.bits());
	}
	ct::logger::trace("End - qimage to mil");
}

void util::Mil_to_qImg(const MIL_ID & milImg, QImage & qImg)
{
	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MIL_INT BandSize = 0;
	MbufInquire(milImg, M_SIZE_X, &SizeX);
	MbufInquire(milImg, M_SIZE_Y, &SizeY);
	MbufInquire(milImg, M_SIZE_BAND, &BandSize);

	if (BandSize == 3)
	{
		qImg = QImage(SizeX, SizeY, QImage::Format_RGB32);
		MbufGetColor(milImg, M_PACKED + M_BGR32, M_ALL_BANDS, qImg.bits());	
	}
	else if (BandSize == 1)
	{
		qImg = QImage(SizeX, SizeY, QImage::Format_Grayscale8);
		MbufGet(milImg, qImg.bits());
	}
}

vips::VImage util::to_vimage(MIL_ID mBuf, void** data)
{
	auto w = mtrx::get_width(mBuf);
	auto h = mtrx::get_height(mBuf);
	auto band = mtrx::get_band(mBuf);

	size_t dataSize = w * h * band;
	*data = malloc(dataSize);

	if (band == 1) MbufGet(mBuf, *data);
	else if (band == 3) MbufGetColor(mBuf, M_PACKED + M_RGB24, M_ALL_BANDS, *data);

	vips::VImage img = vips::VImage::new_from_memory(*data, dataSize, w, h, band, VIPS_FORMAT_UCHAR);

	return img;
}

MIL_ID util::to_milID(vips::VImage img)
{
	int w = img.width();
	int h = img.height();
	int bands = img.bands();  // 1 = grayscale, 3 = RGB, etc.
	VipsBandFormat format = img.format();  // e.g. VIPS_FORMAT_UCHAR

	size_t imgSize = w * h * bands;
	void* data = img.write_to_memory(&imgSize);

	MIL_INT mil_type;
	if (format == VIPS_FORMAT_UCHAR)  mil_type = 8 + M_UNSIGNED;
	else if (format == VIPS_FORMAT_USHORT) mil_type = 16 + M_UNSIGNED;
	else if (format == VIPS_FORMAT_FLOAT)  mil_type = 32 + M_FLOAT;
	else 
		throw std::runtime_error("Unsupported VImage format for MIL");


	MIL_ID mBuf;
	if (bands == 1) {
		MbufAlloc2d(M_DEFAULT, w, h, mil_type, M_IMAGE + M_PROC, &mBuf);
		MbufPut(mBuf, data);
	}
	else {
		MbufAllocColor(M_DEFAULT, bands, w, h, mil_type, M_IMAGE + M_PROC, &mBuf);
		MbufPutColor(mBuf, M_PACKED + M_RGB24, M_ALL_BANDS, data);
	}

	g_free(data); 

	return mBuf;
}

void util::drawCross(int cx, int cy, int size, QImage& qimg)
{
	QPainter painter(&qimg);
	QPen pen(Qt::red);
	pen.setWidth(30);
	painter.setPen(Qt::red);

	// Draw a horizontal line (cross)
	painter.drawLine(cx - size, cy, cx + size, cy);

	// Draw a vertical line (cross)
	painter.drawLine(cx, cy - size, cx, cy + size);

	// End the painting
	painter.end();
}

QString util::combineID(QString viewID, QString opticID)
{
	return viewID + "_" + opticID;
}

QPixmap util::rotatePixmap(const QPixmap & img, double degree)
{
	QTransform transform;
	transform.rotate(degree);
	return img.transformed(transform, Qt::SmoothTransformation);

	/*QPixmap rotatedPixmap(img.size());
	rotatedPixmap.fill(Qt::transparent);

	QPainter painterRotate(&rotatedPixmap);
	painterRotate.setRenderHint(QPainter::Antialiasing);
	painterRotate.translate(img.width() / 2, img.height() / 2);
	painterRotate.rotate(degree);
	painterRotate.translate(-img.width() / 2, -img.height() / 2);
	painterRotate.drawPixmap(0, 0, rotatedPixmap);
	painterRotate.end();

	return rotatedPixmap;*/
}

QImage util::rotateQImage(const QImage & img, double degree)
{
	QTransform transform;
	transform.rotate(degree);
	return img.transformed(transform, Qt::SmoothTransformation);

	/*QPixmap qpixmapImg = QPixmap::fromImage(img); 

	QPixmap rotatedPixmap(img.size());
	rotatedPixmap.fill(Qt::transparent); 
	QPainter painterRotate(&rotatedPixmap);
	painterRotate.setRenderHint(QPainter::Antialiasing);
	painterRotate.translate(img.width() / 2, img.height() / 2);
	painterRotate.rotate(degree);
	painterRotate.translate(-img.width() / 2, -img.height() / 2);
	painterRotate.drawPixmap(0, 0, qpixmapImg);
	painterRotate.end();

	return rotatedPixmap.toImage();*/
}

void util::rotateImage(unsigned char* image, int width, int height, double rotationAngle)
{
	MIL_ID milImage = MbufAlloc2d(M_DEFAULT, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	// Copy the image data to MIL image buffer
	MbufPut2d(milImage, 0, 0, width, height, image);

	// Rotate the image using MIL
	MimRotate(milImage, milImage, rotationAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);

	MbufGet2d(milImage, 0, 0, width, height, image);

	// Free MIL objects
	MbufFree(milImage);
}

MIL_ID util::convertBayerToRGB(unsigned char* image, int width, int height, MIL_INT64 type)
{
	MIL_ID mBayer = MbufAlloc2d(M_DEFAULT, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MIL_ID mResult = MbufAllocColor(M_DEFAULT, 3, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

	MbufPut2d(mBayer, 0, 0, width, height, image);
	MbufBayer(mBayer, mResult, M_DEFAULT, type + M_AVERAGE_2X2);
	//MbufSaveA("mtest.jpg", mResult);

	mtrx::free_buffer(mBayer);
	
	return mResult;
}

void util::formQRGB(QImage& img, const unsigned char* pRedBuf, const unsigned char* pGreenBuf, const unsigned char* pBlueBuf, const unsigned char* pAlphaBuf)
{
	QRgb* pBuf;
	int offset = 0;

	if (img.format() != QImage::Format_RGB32) {
		img = QImage(img.width(), img.height(), QImage::Format_RGB32);
	}

	if (!pAlphaBuf)
	{
		for (int i = 0; i < img.height(); ++i)
		{
			pBuf = reinterpret_cast<QRgb*>(img.scanLine(i));
			offset = i * img.width();

			for (int j = 0; j < img.width(); ++j)
			{
				pBuf[j] = qRgb(pRedBuf[offset + j], pGreenBuf[offset + j], pBlueBuf[offset + j]);
			}
		}
	}
	else
	{
		for (int i = 0; i < img.height(); ++i)
		{
			pBuf = reinterpret_cast<QRgb*>(img.scanLine(i));
			offset = i * img.width();

			for (int j = 0; j < img.width(); ++j)
			{
				pBuf[j] = qRgba(pRedBuf[offset + j], pGreenBuf[offset + j], pBlueBuf[offset + j], pAlphaBuf[offset + j]);
			}
		}
	}
}

unsigned char * util::generateAlphaImage(int width, int height, double rotationAngle)
{
	unsigned char* alpha;
	alpha = new unsigned char[width * height];
	for (int i = 0; i < width * height; ++i) {
		alpha[i] = 255;
	}

	MIL_ID milImage = MbufAlloc2d(M_DEFAULT, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	// Copy the image data to MIL image buffer
	MbufPut2d(milImage, 0, 0, width, height, alpha);

	// Rotate the image using MIL
	MimRotate(milImage, milImage, rotationAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);

	//MimBinarize(milImage, milImage, M_FIXED + M_GREATER_OR_EQUAL, 255, M_NULL);

	MbufGet2d(milImage, 0, 0, width, height, alpha);

	// Free MIL objects
	MbufFree(milImage);

	return alpha;
}

void util::UnsignedChar_to_Mil(int width, int height, unsigned char * red, unsigned char * green, unsigned char * blue, MIL_ID & destination)
{
	ct::logger::trace("Start - Unsign char to mil color");
	destination = MbufAllocColor(M_DEFAULT, 3, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

	MbufPutColor(destination, M_SINGLE_BAND, M_RED, red);
	MbufPutColor(destination, M_SINGLE_BAND, M_GREEN, green);
	MbufPutColor(destination, M_SINGLE_BAND, M_BLUE, blue);
	ct::logger::trace("End - Unsign char to mil color");
}

void util::UnsignedChar_to_Mil(int width, int height, unsigned char * mono, MIL_ID & destination)
{
	destination = MbufAlloc2d(M_DEFAULT, width, height, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MbufPut(destination, mono);
}

void util::UnsignedShort_to_Mil(int width, int height, unsigned short * red, unsigned short * green, unsigned short * blue, MIL_ID & destination)
{
	ct::logger::trace("Start - Unsign short to mil color");
	destination = MbufAllocColor(M_DEFAULT, 3, width, height, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

	MbufPutColor(destination, M_SINGLE_BAND, M_RED, red);
	MbufPutColor(destination, M_SINGLE_BAND, M_GREEN, green);
	MbufPutColor(destination, M_SINGLE_BAND, M_BLUE, blue);
	ct::logger::trace("End - Unsign short to mil color");
}

void util::UnsignedShort_to_Mil(int width, int height, unsigned short * mono, MIL_ID & destination)
{
	ct::logger::trace("Start - Unsign short to mil mono");
	destination = MbufAlloc2d(M_DEFAULT, width, height, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

	MbufPut(destination, mono);
	ct::logger::trace("End - Unsign short to mil mono");
}

QString util::addSuffix(const QString& filePath, const QString& suffix)
{
	QString base = filePath;
	int dotIndex = base.lastIndexOf('.');
	if (dotIndex != -1) base.insert(dotIndex, suffix);
	return base;
}

QString util::changeExtension(const QString& filePath, const QString& newExtension)
{
	QFileInfo fi(filePath);
	return fi.path() + "/" + fi.completeBaseName() + "." + newExtension;
}

void util::offsetBufferValue(unsigned char* buf, int w, int h, int channel, int value)
{
	int size = w * h * channel; 
	for (int i = 0; i < size; i++) {
		auto new_value = buf[i] + value;
		if (new_value < 0) new_value = 0;
		if (new_value > 255) new_value = 255;
		buf[i] = new_value;
	}
}

bool util::isPNG(const QString imgPath)
{
	QStringList nameList = imgPath.split(".");
	if (nameList.size() > 1)
	{
		if (nameList.last() == "png") return true;
	}

	return false;
}

bool util::isBMP(const QString imgPath)
{
	QStringList nameList = imgPath.split(".");
	if (nameList.size() > 1)
	{
		if (nameList.last() == "bmp") return true;
	}

	return false;
}

QString util::convert_to_BMP_ext(const QString imgPath)
{
	QString newImgPath;
	QStringList nameList = imgPath.split(".");
	if (nameList.size() > 1)
	{
		if (nameList.last() != "bmp")
		{
			newImgPath = nameList[0] + ".bmp";
			if (!QFileInfo::exists(newImgPath)) newImgPath = imgPath;
		}
	}
	return newImgPath;
}

bool util::isRAMOver(double percentage)
{
	MEMORYSTATUSEX memStatus;
	memStatus.dwLength = sizeof(memStatus);
	if (GlobalMemoryStatusEx(&memStatus)) {
		DWORDLONG total = memStatus.ullTotalPhys;
		DWORDLONG used = total - memStatus.ullAvailPhys;
		double usagePercent = (double)used / total * 100.0;

		return usagePercent >= percentage;
	}
	else {
		ct::logger::error("Failed to get memory status.");
		return false;
	}
}

void util::saveCroppedMilImg(MIL_ID & src, QRect rect, QString & filePath)
{
	MIL_ID croppedSrc = MbufChildColor2d(src, M_ALL_BANDS, rect.x(), rect.y(), rect.width(), rect.height(), M_NULL);
	//MbufExportA(filePath.toStdString().c_str(), M_JPEG_LOSSLESS, croppedSrc);
	MbufSaveA(filePath.toStdString().c_str(), croppedSrc);
	MbufFree(croppedSrc);
}
QVector<double> util::getNumsFromString(QString str)
{
	// Create a regular expression pattern to match numbers (including decimals)
	//cheat sheet: https://www.rexegg.com/regex-quickstart.html
	QRegularExpression re("\\d+\\.?\\d*");

	// Get an iterator over the matches in the string
	QRegularExpressionMatchIterator matchIterator = re.globalMatch(str);

	// Loop through the matches and extract the numbers
	QVector<double> numbers;
	while (matchIterator.hasNext()) {
		QRegularExpressionMatch match = matchIterator.next();
		auto number = match.captured().toDouble();
		numbers.append(number);
	}
	return numbers;
}

void util::createFolder(const QString & path)
{
	QDir destinationDirObj(path);
	if (!destinationDirObj.exists()) {
		if (!destinationDirObj.mkpath(".")) {
			ct::logger::warn("Failed to create folder: %s", path.toStdString().c_str());
			return;
		}
	}
}

void util::copyTo(const QString & sourcePath, const QString & destinationPath)
{
	// Open the source file
	QFile sourceFile(sourcePath);
	if (!sourceFile.open(QIODevice::ReadOnly)) {
		ct::logger::warn("Failed to copy file: %s", sourcePath.toStdString().c_str());
		return;
	}

	// Open the destination file
	QFile destinationFile(destinationPath);
	if (!destinationFile.open(QIODevice::WriteOnly)) {
		ct::logger::warn("Failed to paste file: %s", destinationPath.toStdString().c_str());
		return;
	}

	// Copy the contents of the source file to the destination file
	destinationFile.write(sourceFile.readAll());

	// Close the files
	sourceFile.close();
	destinationFile.close();
}

void util::copyRecipe(const QString& sourceDir, const QString& destinationDir, const QStringList& excludeList, bool addBBA)
{
	QDir src(sourceDir);
	if (!src.exists()) {
		ct::logger::warn("Source directory does not exist: %s", qPrintable(sourceDir));
		return;
	}

	if (!QDir(destinationDir).exists()) {
		if (!QDir().mkpath(destinationDir)) {
			ct::logger::warn("Failed to create destination directory: %s", qPrintable(destinationDir));
			return;
		}
	}

	// Base folder name "A" -> optionally "A_BBA"
	const QString baseName = QFileInfo(sourceDir).fileName();
	QString targetName = baseName;

	if (addBBA) {
		// avoid double suffix if already present (case-insensitive)
		if (!baseName.endsWith("_BBA", Qt::CaseInsensitive))
			targetName += "_BBA";
	}

	const QString dstRoot = QDir(destinationDir).filePath(targetName);

	// Guard: prevent copying into a subdir of the source
	const QString srcAbs = QDir(sourceDir).absolutePath();
	const QString dstAbs = QDir(dstRoot).absolutePath();
	if (dstAbs.startsWith(srcAbs + QDir::separator())) {
		ct::logger::warn("Destination is inside source. Aborting to avoid recursion.");
		return;
	}

	if (!QDir(dstRoot).exists()) {
		if (!QDir().mkpath(dstRoot)) {
			ct::logger::warn("Failed to create base folder at destination: %s", qPrintable(dstRoot));
			return;
		}
	}

	// Deep copy that ALWAYS overwrites same-name subfolders and files
	std::function<void(const QString&, const QString&)> copyDeep;
	copyDeep = [&](const QString& s, const QString& d) {
		if (!QDir(d).exists() && !QDir().mkpath(d)) {
			ct::logger::warn("Failed to create subdirectory: %s", qPrintable(d));
			return;
		}

		const QFileInfoList items = QDir(s).entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries);
		for (const QFileInfo& it : items) {
			const QString name = it.fileName();
			const QString childS = it.absoluteFilePath();
			const QString childD = d + QDir::separator() + name;

			if (it.isDir()) {
				if (QDir(childD).exists() && !QDir(childD).removeRecursively()) {
					ct::logger::warn("Failed to remove existing folder: %s", qPrintable(childD));
					continue;
				}
				if (!QDir().mkpath(childD)) {
					ct::logger::warn("Failed to create folder: %s", qPrintable(childD));
					continue;
				}
				copyDeep(childS, childD);
			}
			else {
				if (QFile::exists(childD) && !QFile::remove(childD)) {
					ct::logger::warn("Failed to remove existing file: %s", qPrintable(childD));
					continue;
				}
				util::copyTo(childS, childD);
			}
		}
		};

	// FIRST SURFACE (direct children inside source base folder):
	const QFileInfoList top = src.entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries);
	for (const QFileInfo& entry : top) {
		const QString name = entry.fileName();
		const QString sPath = entry.absoluteFilePath();
		const QString dPath = dstRoot + QDir::separator() + name;

		if (entry.isDir()) {
			if (excludeList.contains(name, Qt::CaseInsensitive)) {
				ct::logger::info("Excluded top-level folder: %s", qPrintable(name));
				continue;
			}
			if (QDir(dPath).exists() && !QDir(dPath).removeRecursively()) {
				ct::logger::warn("Failed to remove existing top-level folder: %s", qPrintable(dPath));
				continue;
			}
			if (!QDir().mkpath(dPath)) {
				ct::logger::warn("Failed to create top-level folder: %s", qPrintable(dPath));
				continue;
			}
			copyDeep(sPath, dPath);
		}
		else {
			if (QFile::exists(dPath) && !QFile::remove(dPath)) {
				ct::logger::warn("Failed to remove existing top-level file: %s", qPrintable(dPath));
				continue;
			}
			util::copyTo(sPath, dPath);
		}
	}
}

void util::renameFileWithTimestamp(const QString & path)
{
	// Open the file to get its information
	QFile file(path);
	QFileInfo fileInfo(file);

	// Generate a timestamp for renaming the file
	QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss");

	// Create the new file name with timestamp
	QString newFileName = fileInfo.baseName() + "_" + timestamp + "." + fileInfo.suffix();

	// Create the new file path
	QString newFilePath = fileInfo.dir().filePath(newFileName);

	// Rename the file
	if (!file.rename(newFilePath)) {
		ct::logger::warn("Failed to rename file: %s -> %s", path.toStdString().c_str(), newFilePath.toStdString().c_str());
	}
}

QImage util::MatroxtoQImage(MIL_ID & Img)
{
	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MIL_INT BandSize = 0;
	MbufInquire(Img, M_SIZE_X, &SizeX);
	MbufInquire(Img, M_SIZE_Y, &SizeY);
	MbufInquire(Img, M_SIZE_BAND, &BandSize);

	if (BandSize == 1)
	{
		QRgb pixel;
		QImage qImg(SizeX, SizeY, QImage::Format_RGB32);

		MIL_UINT8 *host;
		MIL_ID pitch = M_NULL;
		MbufInquire(Img, M_HOST_ADDRESS, &host);
		MbufInquire(Img, M_PITCH, &pitch);

		for (int x = 0; x < SizeX; x++)
		{
			for (int y = 0; y < SizeY; y++)
			{
				int gray = host[x + (y * pitch)];
				pixel = qRgb(gray, gray, gray);
				qImg.setPixel(x, y, pixel);
			}
		}

		return qImg;
	}
	else if (BandSize == 3)
	{
		MIL_ID MilRed, MilBlue, MilGreen;

		MIL_UINT8* mSrcPtrRed;
		MIL_ID mPitchPtrRed;

		MIL_UINT8* mSrcPtrGreen;
		MIL_ID mPitchPtrGreen;

		MIL_UINT8* mSrcPtrBlue;
		MIL_ID mPitchPtrBlue;

		MIL_INT *PixelValueRed, *PixelValueGreen, *PixelValueBlue;

		MIL_INT SizeX = 0;
		MIL_INT SizeY = 0;
		MbufInquire(Img, M_SIZE_X, &SizeX);
		MbufInquire(Img, M_SIZE_Y, &SizeY);

		MbufAllocColor(M_DEFAULT, 1, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, &MilRed);
		MbufAllocColor(M_DEFAULT, 1, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, &MilGreen);
		MbufAllocColor(M_DEFAULT, 1, SizeX, SizeY, 8, M_IMAGE + M_PROC + M_DISP, &MilBlue);

		MbufInquire(MilRed, M_HOST_ADDRESS, &mSrcPtrRed);
		MbufInquire(MilRed, M_PITCH, &mPitchPtrRed);

		MbufInquire(MilGreen, M_HOST_ADDRESS, &mSrcPtrGreen);
		MbufInquire(MilGreen, M_PITCH, &mPitchPtrGreen);

		MbufInquire(MilBlue, M_HOST_ADDRESS, &mSrcPtrBlue);
		MbufInquire(MilBlue, M_PITCH, &mPitchPtrBlue);

		MbufCopyColor(Img, MilRed, M_RED);
		MbufCopyColor(Img, MilGreen, M_GREEN);
		MbufCopyColor(Img, MilBlue, M_BLUE);


		QRgb pixel;
		QImage qImg(SizeX, SizeY, QImage::Format_RGB32);

		MIL_UINT8 *host;
		MIL_ID pitch = M_NULL;
		MbufInquire(Img, M_HOST_ADDRESS, &host);
		MbufInquire(Img, M_PITCH, &pitch);

		int red, blue, green;
		for (int x = 0; x < SizeX; x++)
		{
			for (int y = 0; y < SizeY; y++)
			{
				red = mSrcPtrRed[x + (y * mPitchPtrRed)];
				blue = mSrcPtrBlue[x + (y * mPitchPtrBlue)];
				green = mSrcPtrGreen[x + (y * mPitchPtrGreen)];
				pixel = qRgb(red, green, blue);
				qImg.setPixel(x, y, pixel);
			}
		}

		MbufFree(MilRed);
		MbufFree(MilGreen);
		MbufFree(MilBlue);
		return qImg;
	}

	return QImage();
}
	
//Temp
em::M3d em::get_rotation_matrix(double rx, double ry, double rz, em::RotationSequence rs)
{
	auto A = em::to_radian(rx);
	auto B = em::to_radian(ry);
	auto C = em::to_radian(rz);

	Eigen::Matrix3d RX;
	RX << 1, 0, 0,
		0, cos(A), -sin(A),
		0, sin(A), cos(A);

	Eigen::Matrix3d RY;
	RY << cos(B), 0, sin(B),
		0, 1, 0,
		-sin(B), 0, cos(B);

	Eigen::Matrix3d RZ;
	RZ << cos(C), -sin(C), 0,
		sin(C), cos(C), 0,
		0, 0, 1;

	Eigen::Matrix3d rotationMatrix;

	switch (rs) {
	case em::RotationSequence::RXRYRZ:
		rotationMatrix = RZ * RY * RX;
		break;
	case em::RotationSequence::RXRZRY:
		rotationMatrix = RY * RZ * RX;
		break;
	case em::RotationSequence::RYRXRZ:
		rotationMatrix = RZ * RX * RY;
		break;
	case em::RotationSequence::RYRZRX:
		rotationMatrix = RX * RZ * RY;
		break;
	case em::RotationSequence::RZRXRY:
		rotationMatrix = RY * RX * RZ;
		break;
	case em::RotationSequence::RZRYRX:
		rotationMatrix = RX * RY * RZ;
		break;
	}

	return rotationMatrix;
}

util::CornerFinder::Point util::CornerFinder::topleft()
{
	return Point();
}

util::CornerFinder::Point util::CornerFinder::topright()
{
	return Point();
}

util::CornerFinder::Point util::CornerFinder::btmright()
{
	return Point();
}

util::CornerFinder::Point util::CornerFinder::btmleft()
{
	return Point();
}

void util::CornerFinder::add(Point point)
{
}

void util::CornerFinder::clear()
{
}

//bool algo::find_circle(Circle & circle, const cv::Mat& img, double min_radius, double max_radius, CircleType type)
//{
//	if (img.empty()) return false;
//
//	// Convert the image to grayscale
//	cv::Mat grayImage;
//
//	if (img.channels() == 3) {
//		cv::cvtColor(img, grayImage, cv::COLOR_BGR2GRAY);
//	}
//	else {
//		grayImage = img;
//	}
//
//	// Apply Gaussian blur to reduce noise and improve circle detection
//	cv::GaussianBlur(grayImage, grayImage, cv::Size(9, 9), 2, 2);
//
//	// Detect circles using the Hough Circle Transform
//	std::vector<cv::Vec3f> circles;
//	cv::HoughCircles(grayImage, circles, cv::HOUGH_GRADIENT, 1, grayImage.rows / 8, 10, 80, min_radius, max_radius);
//	
//	bool found = false;
//	int smallest_index = 0, largest_index = 0;
//	float smallest_radius = max_radius + 1;
//	float largest_radius = min_radius - 1;
//
//	int index = 0;
//	for (auto& c : circles) {
//
//		auto& r = c[2];
//
//		if (r > largest_radius) {
//			largest_radius = r;
//			largest_index = index;
//		}
//
//		if (r < smallest_radius) {
//			smallest_radius = r;
//			smallest_index = index;
//		}
//		
//		found = true;
//		index++;
//	}
//
//	if (found) {
//		if (type == CircleType::LARGEST_RADIUS) index = largest_index;
//		else index = smallest_index;
//
//		circle.cx = circles[index][0];
//		circle.cy = circles[index][1];
//		circle.radius = circles[index][2];
//		circle.x = circle.cx - circle.radius;
//		circle.y = circle.cy - circle.radius;
//
//		// Draw detected circles on the original image
//		cv::Point center(cvRound(circle.cx), cvRound(circle.cy));
//		int radius = cvRound(circle.radius);
//		cv::circle(img, center, radius, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
//
//		// Display the result
//		cv::namedWindow("Image", cv::WINDOW_NORMAL);
//		cv::imshow("Image", img);
//		cv::resizeWindow("Image", img.cols, img.rows);
//	}
//
//	return found;
//}
//
//void algo::find_circles(std::vector<Circle>& circles, const cv::Mat& img, double min_radius, double max_radius)
//{
//	if (img.empty()) return;
//
//	// Convert the image to grayscale
//	cv::Mat grayImage;
//
//	if (img.channels() == 3) {
//		cv::cvtColor(img, grayImage, cv::COLOR_BGR2GRAY);
//	}
//	else {
//		grayImage = img;
//	}
//
//	// Apply Gaussian blur to reduce noise and improve circle detection
//	cv::GaussianBlur(grayImage, grayImage, cv::Size(9, 9), 2, 2);
//
//	// Detect circles using the Hough Circle Transform
//	std::vector<cv::Vec3f> cs;
//	cv::HoughCircles(grayImage, cs, cv::HOUGH_GRADIENT, 1, grayImage.rows / 8, 100, 60, min_radius, max_radius);
//
//	Circle circle;
//	for (auto& c : cs) {
//		circle.cx = c[0];
//		circle.cy = c[1];
//		circle.radius = c[2];
//		circle.x = circle.cx - circle.radius;
//		circle.y = circle.cy - circle.radius;
//		circles.emplace_back(circle);
//	}
//}

util::ImagePreprocess::ImagePreprocess(QString refImageFilePath, QString maskImageFilePath)
{
	//load ref image

	if (QFileInfo::exists(refImageFilePath))
	{
		_refImage = cv::imread(refImageFilePath.toStdString());
		cv_to_Mil(_refImage, _milRefImg);
		
		MIL_INT SizeX = 0;
		MIL_INT SizeY = 0;
		MbufInquire(_milRefImg, M_SIZE_X, &SizeX);
		MbufInquire(_milRefImg, M_SIZE_Y, &SizeY);
		_refWidth = SizeX;
		_refHeight = SizeY;
	}
	else
	{
		qDebug() << refImageFilePath << "not found!!!";
		_refImage = cv::Mat();
		if (_milRefImg) MbufFree(_milRefImg);
		_milRefImg = M_NULL;
	}
	

	//load mask image
	//_maskImage = cv::imread(maskImageFilePath.toStdString());
	//if (!_maskImage.empty())
	//{
	//	cv::cvtColor(_maskImage, _maskImage, cv::COLOR_BGR2GRAY);
	//	cv_to_Mil(_maskImage, _milMaskImg);
	//}
}

util::ImagePreprocess::~ImagePreprocess()
{
	if (_milRefImg)
	{
		MbufFree(_milRefImg);
		_milRefImg = M_NULL;
	}

	if (_milMaskImg)
	{
		MbufFree(_milMaskImg);
		_milMaskImg = M_NULL;
	}
}

void util::ImagePreprocess::Diff_of_medianFilter(QImage & qImg, QImage & output, int brightThreshold, int darkThreshold, int imgType, int median1, int median2, int darkMedian1, int darkMedian2, bool findDifference, int diffThreshold)
{
	if (_milRefImg == M_NULL) return;

	QImage paddedImg = qImg.scaled(_refWidth, _refHeight);
	paddedImg.fill(Qt::black);

	QPainter painter(&paddedImg);
	painter.drawImage(0, 0, qImg);


	MIL_ID milImg = M_NULL;
	qImg_to_Mil(paddedImg, milImg);

	Diff_of_medianFilter(milImg, milImg, brightThreshold, darkThreshold, imgType, median1, median2, darkMedian1, darkMedian2, findDifference, diffThreshold);

	Mil_to_qImg(milImg, output);
	if (milImg) MbufFree(milImg);

	if (false)
	{
		if (qImg.format() == QImage::Format_RGB32)
		{
			cv::Mat input_image(qImg.height(), qImg.width(), CV_8UC4, qImg.bits(), qImg.bytesPerLine());
			std::vector<cv::Mat> channels;
			cv::split(input_image, channels);

			// Extract the green channel (index 1)
			cv::Mat greenChannel = channels[1];

			//perform difference of median filter
			cv::Mat medianBlur3, medianBlur9, edge, inverseEdge;
			cv::medianBlur(greenChannel, medianBlur3, median1);
			cv::medianBlur(greenChannel, medianBlur9, median2);
			
			cv::subtract(medianBlur3, medianBlur9, edge);
			cv::threshold(edge, edge, 20, 255, cv::THRESH_BINARY);

			//get Edges from Input Img
			cv::cvtColor(input_image, input_image, cv::COLOR_BGRA2RGB); // Convert BGRA to BGR
			cv::Mat output_image = cv::Mat::zeros(qImg.height(), qImg.width(), CV_8UC3);
			input_image.copyTo(output_image, edge);

			//get background from ref img
			cv::bitwise_not(edge, inverseEdge);
			cv::Mat ref_image;
			cv::cvtColor(_refImage, ref_image, cv::COLOR_BGRA2RGB); // Convert BGRA to BGR
			ref_image.copyTo(output_image, inverseEdge);

			// Convert back to QImage
			QImage outputImg(output_image.data, output_image.cols, output_image.rows, static_cast<int>(output_image.step), QImage::Format_RGB888);
			//QImage greenImg(greenChannel.data, greenChannel.cols, greenChannel.rows, static_cast<int>(greenChannel.step), QImage::Format_Grayscale8);
			output = outputImg.copy();
		}
	}

}

void util::ImagePreprocess::Diff_of_medianFilter(MIL_ID & milImg, MIL_ID & output, int brightThreshold, int darkThreshold, int imgType, int median1, int median2, int darkMedian1, int darkMedian2, bool findDifference, int diffThreshold) //0 red, 1 green, 2 blue
{
	if (_milRefImg == M_NULL)
	{
		qDebug() << "milRefImg = null";
		return;
	}
	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MIL_INT BandSize = 0;
	MbufInquire(milImg, M_SIZE_X, &SizeX);
	MbufInquire(milImg, M_SIZE_Y, &SizeY);
	MbufInquire(milImg, M_SIZE_BAND, &BandSize);

	MIL_ID milChannel = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MIL_ID refMilChannel = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	if (BandSize == 3)
	{
		/*	MbufSaveA("DOMOriInputImage.jpg", milImg);
			MbufSaveA("DOMOriRefImage.jpg", _milRefImg);*/
			//extract red, green or blue channel to process
		
		if (imgType == 0)
		{
			MbufCopyColor(milImg, milChannel, M_RED);
			MbufCopyColor(_milRefImg, refMilChannel, M_RED);
		}
		else if (imgType == 1)
		{
			MbufCopyColor(milImg, milChannel, M_GREEN);
			MbufCopyColor(_milRefImg, refMilChannel, M_GREEN);
		}
		else if (imgType == 2)
		{
			MbufCopyColor(milImg, milChannel, M_BLUE);
			MbufCopyColor(_milRefImg, refMilChannel, M_BLUE);
		}
		else
		{
			MbufFree(milChannel);
			MbufFree(refMilChannel);
			return;
		}
	}
	else if (BandSize == 1)
	{ 
		MbufCopy(milImg, milChannel);
		MbufCopy(_milRefImg, refMilChannel);
	}

	//perform difference of median filter
	MIL_ID mKernel3 = MbufAlloc2d(M_DEFAULT, median1, median1, 32 + M_UNSIGNED, M_STRUCT_ELEMENT, M_NULL);
	MIL_ID medianFilter3 = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimRank(milChannel, medianFilter3, mKernel3, M_MEDIAN, M_GRAYSCALE);
	//MbufSaveA("Diff_of_medianFilter3.jpg", medianFilter3);

	MIL_ID mKernel9 = MbufAlloc2d(M_DEFAULT, median2, median2, 32 + M_UNSIGNED, M_STRUCT_ELEMENT, M_NULL);
	MIL_ID medianFilter9 = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimRank(milChannel, medianFilter9, mKernel9, M_MEDIAN, M_GRAYSCALE);
	//MimDilate(medianFilter9, medianFilter9, 2, M_GRAYSCALE);
	//MbufSaveA("Diff_of_medianFilter9.jpg", medianFilter9);

	//bright DOM
	MIL_ID DoMBright = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimArith(medianFilter3, medianFilter9, DoMBright, M_SUB + M_SATURATION);

	MimBinarize(DoMBright, DoMBright, M_FIXED + M_GREATER_OR_EQUAL, brightThreshold, M_NULL);

	// dark DOM
	//perform difference of median filter
	MIL_ID mKernel_dark1 = MbufAlloc2d(M_DEFAULT, darkMedian1, darkMedian1, 32 + M_UNSIGNED, M_STRUCT_ELEMENT, M_NULL);
	MIL_ID medianFilterDark1 = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimRank(milChannel, medianFilterDark1, mKernel_dark1, M_MEDIAN, M_GRAYSCALE);
	//MbufSaveA("Diff_of_medianFilterDark1.jpg", medianFilterDark1);

	MIL_ID mKernel_dark2 = MbufAlloc2d(M_DEFAULT, darkMedian2, darkMedian2, 32 + M_UNSIGNED, M_STRUCT_ELEMENT, M_NULL);
	MIL_ID medianFilterDark2 = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimRank(milChannel, medianFilterDark2, mKernel_dark2, M_MEDIAN, M_GRAYSCALE);
	//MbufSaveA("Diff_of_medianFilter9.jpg", medianFilter9);

	MIL_ID DoMDark = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimArith(medianFilterDark2, medianFilterDark1, DoMDark, M_SUB + M_SATURATION);

	MimBinarize(DoMDark, DoMDark, M_FIXED + M_GREATER_OR_EQUAL, darkThreshold, M_NULL);
	
	MIL_ID DoM = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MIL_ID DoM_inverse = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MbufClear(DoM_inverse, 255);
	MimArith(DoMBright, DoMDark, DoM, M_ADD + M_SATURATION);
	MimArith(DoM_inverse, DoM, DoM_inverse, M_SUB + M_SATURATION);
	/*MbufSaveA("DOM_inverse.jpg", DoM_inverse);
	MbufSaveA("DoM.jpg", DoM);*/

	MbufFree(DoMBright);
	MbufFree(DoMDark);

	// get difference between input and ref image
	if (findDifference)
	{
		MIL_ID difference_input_ref = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MIL_ID refMedianFilter3 = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MimRank(refMilChannel, refMedianFilter3, mKernel3, M_MEDIAN, M_GRAYSCALE);
		MimArith(milChannel, refMilChannel, difference_input_ref, M_SUB + M_SATURATION);
		MimBinarize(difference_input_ref, difference_input_ref, M_FIXED + M_GREATER_OR_EQUAL, diffThreshold, M_NULL);

		MimArith(DoM, difference_input_ref, DoM, M_ADD + M_SATURATION);
		MimArith(DoM_inverse, difference_input_ref, DoM_inverse, M_SUB + M_SATURATION);

		MbufFree(difference_input_ref);
		MbufFree(refMedianFilter3);
	}

	MimClip(DoM, DoM, M_GREATER, 0, M_NULL, 1, M_NULL);
	MimArith(milImg, DoM, milImg, M_MULT + M_SATURATION);
	//MbufSaveA("DoMColour.jpg", milImg);

	MIL_ID cloneRefImg = MbufClone(_milRefImg, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, &cloneRefImg);
	MimClip(DoM_inverse, DoM_inverse, M_GREATER, 0, M_NULL, 1, M_NULL);
	MimArith(_milRefImg, DoM_inverse, cloneRefImg, M_MULT + M_SATURATION);
	//MbufSaveA("DoMRef.jpg", cloneRefImg);

	MimArith(milImg, cloneRefImg, milImg, M_ADD + M_SATURATION);
	//MbufSaveA("DoMFinal.jpg", milImg);
	
	MbufFree(milChannel);
	MbufFree(refMilChannel);
	MbufFree(mKernel3);
	MbufFree(medianFilter3);
	MbufFree(mKernel9);
	MbufFree(medianFilter9);
	MbufFree(mKernel_dark1);
	MbufFree(medianFilterDark1);
	MbufFree(mKernel_dark2);
	MbufFree(medianFilterDark2);
	MbufFree(DoM);
	MbufFree(DoM_inverse);
	MbufFree(cloneRefImg);
	
}

void util::ImagePreprocess::performHistMatching(QImage & qImg, QImage & output)
{
	Timer time;

	//check if all band size are the same
	//check if all image size are the same
	if (qImg.format() == QImage::Format_RGB32)
	{
		cv::Mat input_image(qImg.height(), qImg.width(), CV_8UC4, qImg.bits(), qImg.bytesPerLine());

		/*cv::imwrite("testSaveInputImg.jpg", input_image);
		cv::imwrite("testSaveRefImg.jpg", _refImage);*/

		cv::Mat input_bgr[3];   //destination array
		split(input_image, input_bgr);//split source

		cv::Mat ref_bgr[3];   //destination array
		split(_refImage, ref_bgr);//split source

		cv::Mat results(input_image.cols, input_image.rows, CV_32FC3);
		cv::Mat result_bgr[3];
		split(results, result_bgr);

		for (int i = 0; i < 3; i++)
		{
			cv::MatND ref_hist, input_hist;
			int dims = 1;
			float hranges[] = { 0,255 };
			const float *ranges[] = { hranges };
			int size = 256;
			int channels = 0;

			//Calculate the histogram of template image and target image
			if (_maskImage.empty())
			{
				cv::calcHist(&input_bgr[i], 1, &channels, cv::Mat(), input_hist, dims, &size, ranges);
				cv::calcHist(&ref_bgr[i], 1, &channels, cv::Mat(), ref_hist, dims, &size, ranges);
			}
			else
			{
				cv::calcHist(&input_bgr[i], 1, &channels, _maskImage, input_hist, dims, &size, ranges);
				cv::calcHist(&ref_bgr[i], 1, &channels, _maskImage, ref_hist, dims, &size, ranges);
			}
			// Get the cumulative histogram of the template image and the target image
			float src_cdf[256] = { 0 };
			float dst_cdf[256] = { 0 };
			for (int i = 0; i < 256; i++) {
				if (i == 0) {
					src_cdf[i] = input_hist.at<float>(i);
					dst_cdf[i] = ref_hist.at<float>(i);
				}
				else {
					src_cdf[i] = src_cdf[i - 1] + input_hist.at<float>(i);
					dst_cdf[i] = dst_cdf[i - 1] + ref_hist.at<float>(i);
				}
			}

			/// Prescribed processing of the target image
			// // Calculate the difference in cumulative probability
			float diff_cdf[256][256];
			for (int i = 0; i < 256; i++) {
				for (int j = 0; j < 256; j++) {
					diff_cdf[i][j] = fabs(src_cdf[i] - dst_cdf[j]);
				}
			}

			//2. Build gray level mapping table
			cv::Mat lut(1, 256, CV_8U);
			for (int i = 0; i < 256; i++) {
				// Find the mapped gray level with the source  gray level of i and the normalized gray level with the smallest difference in cumulative probability of i
				float min = diff_cdf[i][0];
				int index = 0;
				for (int j = 0; j < 256; j++) {
					if (min > diff_cdf[i][j]) {
						min = diff_cdf[i][j];
						index = j;
					}
				}
				lut.at<uchar>(i) = static_cast<uchar>(index);
			}
			// Use the lookup table to get the equalized image	
			cv::LUT(input_bgr[i], lut, result_bgr[i]);
		}

		cv::merge(result_bgr, 3, input_image);
		//cv::imwrite("testSaveOutputImg.jpg", input_image);

		// Create a QImage using the cv::Mat data
		cv::Mat output_image(input_image.rows, input_image.cols, CV_8UC3);
		cv::cvtColor(input_image, output_image, cv::COLOR_BGRA2RGB); // Convert BGRA to BGR
		//cv::medianBlur(output_image, output_image, 3);
		QImage outputImg(output_image.data, output_image.cols, output_image.rows, static_cast<int>(output_image.step), QImage::Format_RGB888);
		output = outputImg.copy();
		//qDebug() << "histMatchingDuration:" << time.current();	
	}	

}

void util::ImagePreprocess::performHistMatching(MIL_ID & milImg, MIL_ID & output)
{
}

void util::ImagePreprocess::HighlightDarkDefects(QImage & qImg, QImage & output, int darkThreshold, int darkDifferenceThreshold, int brightDifferenceThreshold)
{
	QImage paddedImg = qImg.scaled(_refWidth, _refHeight);
	paddedImg.fill(Qt::black);

	QPainter painter(&paddedImg);
	painter.drawImage(0, 0, qImg);

	MIL_ID milImg = M_NULL;
	qImg_to_Mil(paddedImg, milImg);

	HighlightDarkDefects(milImg, milImg, darkThreshold, darkDifferenceThreshold, brightDifferenceThreshold);

	Mil_to_qImg(milImg, output);
	if (milImg) MbufFree(milImg);
}

void util::ImagePreprocess::HighlightDarkDefects(MIL_ID & milImg, MIL_ID & output, int darkThreshold, int darkDifferenceThreshold, int brightDifferenceThreshold)
{
	if (_milRefImg == M_NULL) return;
	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MIL_INT BandSize = 0;
	MbufInquire(milImg, M_SIZE_X, &SizeX);
	MbufInquire(milImg, M_SIZE_Y, &SizeY);
	MbufInquire(milImg, M_SIZE_BAND, &BandSize);

	if (BandSize == 3)
	{
		//inputImg
		MIL_UINT8* mRedSrcPtr, *mGreenSrcPtr, *mBlueSrcPtr;
		MIL_ID mPitchPtr;
		MbufInquire(milImg, M_HOST_ADDRESS_BAND + 0, &mRedSrcPtr);
		MbufInquire(milImg, M_HOST_ADDRESS_BAND + 1, &mGreenSrcPtr);
		MbufInquire(milImg, M_HOST_ADDRESS_BAND + 2, &mBlueSrcPtr);
		MbufInquire(milImg, M_PITCH, &mPitchPtr);

		//referenceImg
		MIL_UINT8* rRedSrcPtr, *rGreenSrcPtr, *rBlueSrcPtr;
		MIL_ID rPitchPtr;
		MbufInquire(_milRefImg, M_HOST_ADDRESS_BAND + 0, &rRedSrcPtr);
		MbufInquire(_milRefImg, M_HOST_ADDRESS_BAND + 1, &rGreenSrcPtr);
		MbufInquire(_milRefImg, M_HOST_ADDRESS_BAND + 2, &rBlueSrcPtr);
		MbufInquire(_milRefImg, M_PITCH, &rPitchPtr);

		//outputImg
		MIL_UINT8* oRedSrcPtr, *oGreenSrcPtr, *oBlueSrcPtr;
		MIL_ID oPitchPtr;
		MbufInquire(output, M_HOST_ADDRESS_BAND + 0, &oRedSrcPtr);
		MbufInquire(output, M_HOST_ADDRESS_BAND + 1, &oGreenSrcPtr);
		MbufInquire(output, M_HOST_ADDRESS_BAND + 2, &oBlueSrcPtr);
		MbufInquire(output, M_PITCH, &oPitchPtr);

		for (int x = 0; x < SizeX; x++)
		{
			for (int y = 0; y < SizeY; y++)
			{
				int mR = mRedSrcPtr[x + (y * mPitchPtr)];
				int mG = mGreenSrcPtr[x + (y * mPitchPtr)];
				int mB = mBlueSrcPtr[x + (y * mPitchPtr)];

				int darkCount = 0;
				if (mR <= darkThreshold) darkCount++;
				if (mG <= darkThreshold) darkCount++;
				if (mB <= darkThreshold) darkCount++;

				int rR = rRedSrcPtr[x + (y * rPitchPtr)];
				int rG = rGreenSrcPtr[x + (y * rPitchPtr)];
				int rB = rBlueSrcPtr[x + (y * rPitchPtr)];

				int mGray = (mR + mG + mB) / 3;
				int rGray = (rR + rG + rB) / 3;
				int difference = mGray - rGray;

				// if R,G,B have at least two are less than threshold then the pixel will be used for next action
				if (darkCount >= 2)
				{
					// subtract the pixel of input and ref img, if the difference grayscale value is negative and greater than threshold invert the pixel of three channels
					if (difference < 0 && abs(difference) > darkDifferenceThreshold)
					{
						oRedSrcPtr[x + (y * oPitchPtr)] = 255 - mR;
						oGreenSrcPtr[x + (y * oPitchPtr)] = 255 - mG;
						oBlueSrcPtr[x + (y * oPitchPtr)] = 255 - mB;
					}
					
				}
				else
				{
					if (abs(difference) > brightDifferenceThreshold)
					{
						oRedSrcPtr[x + (y * oPitchPtr)] = mR;
						oGreenSrcPtr[x + (y * oPitchPtr)] = mG;
						oBlueSrcPtr[x + (y * oPitchPtr)] = mB;
					}
					else
					{
						oRedSrcPtr[x + (y * oPitchPtr)] = rR;
						oGreenSrcPtr[x + (y * oPitchPtr)] = rG;
						oBlueSrcPtr[x + (y * oPitchPtr)] = rB;
					}
				}
			}
		}
	}
}

void util::ImagePreprocess::medianFilter(QImage & qImg, QImage & output, int imgType, int median)
{
	QImage paddedImg = qImg.scaled(_refWidth, _refHeight);
	paddedImg.fill(Qt::black);

	QPainter painter(&paddedImg);
	painter.drawImage(0, 0, qImg);

	MIL_ID milImg = M_NULL;
	qImg_to_Mil(paddedImg, milImg);

	medianFilter(milImg, milImg, imgType, median);

	Mil_to_qImg(milImg, output);
	if (milImg) MbufFree(milImg);
}

void util::ImagePreprocess::medianFilter(MIL_ID & milImg, MIL_ID & output, int imgType, int median)
{
	if (_milRefImg == M_NULL) return;
	MIL_INT SizeX = 0;
	MIL_INT SizeY = 0;
	MIL_INT BandSize = 0;
	MbufInquire(milImg, M_SIZE_X, &SizeX);
	MbufInquire(milImg, M_SIZE_Y, &SizeY);
	MbufInquire(milImg, M_SIZE_BAND, &BandSize);

	MIL_ID milChannel = MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	if (BandSize == 3)
	{
		if (imgType == 0)
		{
			MbufCopyColor(milImg, milChannel, M_RED);
		}
		else if (imgType == 1)
		{
			MbufCopyColor(milImg, milChannel, M_GREEN);
		}
		else if (imgType == 2)
		{
			MbufCopyColor(milImg, milChannel, M_BLUE);
		}
		else
		{
			MbufFree(milChannel);
			return;
		}
	}
	else if (BandSize == 1)
	{
		MbufCopy(milImg, milChannel);
	}

	//perform difference of median filter
	MIL_ID mKernel = MbufAlloc2d(M_DEFAULT, median, median, 32 + M_UNSIGNED, M_STRUCT_ELEMENT, M_NULL);
	MIL_ID medianFiltered= MbufAlloc2d(M_DEFAULT, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MimRank(milChannel, medianFiltered, mKernel, M_MEDIAN, M_GRAYSCALE);
	
	MbufCopyColor2d(medianFiltered, milImg, M_ALL_BANDS, 0, 0, M_ALL_BANDS, 0, 0, SizeX, SizeY);

	MbufFree(milChannel);
	MbufFree(mKernel);
	MbufFree(medianFiltered);
}

int util::ImagePreprocess::width()
{
	return _refWidth;
}

int util::ImagePreprocess::height()
{
	return _refHeight;
}


//DBSCAN
double util::distance(const DBRect& db1, const DBRect& db2) {
	double dx = (db1.rect.x() + db1.rect.width() / 2) - (db2.rect.x() + db2.rect.width() / 2);
	double dy = (db1.rect.y() + db1.rect.height() / 2) - (db2.rect.y() + db2.rect.height() / 2);
	return std::sqrt(dx * dx + dy * dy);
}

void inline expandCluster(std::vector<QString>& clusters, util::DBRect & db, QHash<QString, util::DBRect>& dbrects)
{
	for (auto& neighbourID : db.neighbours) {

		auto& neighbourDB = dbrects[neighbourID];

		if (neighbourDB.type == DBType::CORE) {
			clusters.push_back(neighbourID);
			neighbourDB.type = DBType::CLUSTERED;
			neighbourDB.clusterIndex = clusters.size() - 1;
			expandCluster(clusters, neighbourDB, dbrects);
		}
	}
}

std::vector<std::vector<QString>> util::dbscan(QHash<QString, util::DBRect>& dbrects, double eps, int minPts) {

	/*
	DBSCAN: Density-based spatial clustering of applications with noise
	1. Loop through all points, assign core points, border points and noise
	1.1. Core points: Points that have more or equal amount of min neighbouring points
	1.2. Border points: Points that have neighbouring points but less than minimum
	1.3. Noise: Points that do not have neighbouring points
	2. Assign cluster to a random core point and expand cluster to neighbouring core points
	3. Assign border points to closest cluster
	*/

	//ensure data is clean
	for (auto& db : dbrects) {
		db.type = DBType::UNASSIGNED;
		db.neighbours.clear();
		db.clusterIndex = 0;
	}

	for (auto& db : dbrects) {

		if (db.type != DBType::UNASSIGNED) continue;

		//find neighbours
		for (const auto& otherID : dbrects.keys()) {

			const auto& otherdb = dbrects.value(otherID);

			if (distance(db, otherdb) <= eps) {
				db.neighbours.insert(otherID);
			}
		}

		if (db.neighbours.size() >= minPts) db.type = DBType::CORE;
		else if (db.neighbours.size()) db.type = DBType::BORDER;
		else db.type = DBType::NOISE;
	}

	//cluster
	std::vector<std::vector<QString>> clusters;
	for (const auto& id : dbrects.keys()) {

		auto& db = dbrects[id];

		if (db.type == DBType::CORE || db.type == DBType::NOISE) {
			clusters.emplace_back(); 
			clusters.back().push_back(id);

			db.type = DBType::CLUSTERED;
			db.clusterIndex = clusters.size() - 1;

			expandCluster(clusters.back(), db, dbrects);
		}
	}

	//assign border points to closest cluster
	for (const auto& id : dbrects.keys()) {

		auto& db = dbrects[id];

		if (db.type == DBType::BORDER) {

			//closest neighbour
			QString closestID;
			double closestDistance = eps;

			for (auto& neighbourID : db.neighbours) {

				auto& neighbourDB = dbrects[neighbourID];
				
				auto currentDistance = distance(db, neighbourDB);

				if (currentDistance < closestDistance) {
					closestID = neighbourID;
					closestDistance = currentDistance;
				}
			}

			//assign to closest neighbour's cluster
			auto idx = dbrects[closestID].clusterIndex;
			if (idx < clusters.size()) {
				clusters[idx].push_back(id);
			}
		}
	}

	return clusters;
}
