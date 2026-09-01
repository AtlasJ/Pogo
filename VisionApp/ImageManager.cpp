#include "ImageManager.h"
#include "Logger.h"
#include "OpticsInfo.h"
#include <opencv2/stitching.hpp>
#include <QPainter>
#include "Utilities.h"
#include "mtrx.h"
#include "CAMManager.h"
#include "MessageQue.h"
#include "focus_stacking.h"
#include "QOSTool.h"
#include "ProfilerManager.h"
#include "ScaleManager.h"
#include "SystemData.h"

#include "ImageSavingThread.h"

#include <vips/vips8>
#include "libvips/VImage8.h"

#include "Timer.h"

/*
* need to handle multiple light
* image stitch and mechanical stitch
* Do one image and apply same transformation to other images
* Not supported for multi optics yet
* - load stitch data
* QVector<FrameInfo>: Array of same view dif optics
*/


struct MilEntry { MIL_ID id = M_NULL; QString path; };
static QHash<QString, MilEntry> s_milCache;

extern TMessageQue<FrameInfo> g_imageQueue;

void ImageManager::run()
{
	ct::logger::info("[QThread] Imagemanager started");

	m_zstackThread = std::thread([this]() {
		this->process_stack_queues_thread();
	});

	m_stitchThread = std::thread([this]() {
		this->process_stitch_thread();
	});

	while (true) {

		if (m_release) {
			if (m_zstackThread.joinable()) m_zstackThread.join();
			if (m_stitchThread.joinable()) m_stitchThread.join();
			for (auto it = s_milCache.begin(); it != s_milCache.end(); ++it) {
				if (it->id != M_NULL) {
					MbufFree(it->id);
					it->id = M_NULL;
				}
			}
			s_milCache.clear();
			reset();
			return;
		}
		//os_tool::doNothing(1);

		//ct::logger::info("[IM] Waiting for image...");

		while (g_imageQueue.size() > 0) {

			FrameInfo info;
			if (!g_imageQueue.get(info)) continue;

			if (info.type == ct::s_height_snapshot) continue;

			ct::logger::info("[IM] Received image: %s", getInfoID(info).toStdString().c_str());

			auto s = QString("[IM] Total image preprocessing: %1").arg(getInfoID(info));
			ScopedTimeLogger Stimer(s.toStdString().c_str());

			emit imageReceived(info);

			preprocess_info(info);
		}
	}
}

void ImageManager::release()
{
	m_release = true;
	// Wake the zstack/stitch workers parked on their condition variables —
	// their wait predicates slept through shutdown, so run()'s join() stalled
	// and the destructor burned the full thread-wait timeout on every exit.
	m_zstackCV.notify_all();
	m_stitchCV.notify_all();
}

void ImageManager::reset()
{
	ct::logger::trace("Reset image manager");

	std::scoped_lock lock(m_infosMapMutex, m_zstackDequeMutex, m_zstackMapMutex);

	m_infosMap.clear();
	m_zstackMap.clear();
	m_stitchMap.clear();
	m_zstackDeque.clear();
	m_combineMap.clear();

	m_zstackMap.reserve(1000);

	update_stitchData();
	update_opticsData();
	update_zCount();

	ct::logger::trace("Done Reset image manager");
}

void ImageManager::attach(Fiducial* fiducialAlgo)
{
	m_fiducialAlgo = fiducialAlgo;
}

void ImageManager::attach(QHash<QString, QView>* views, QHash<QString, OpticsInfo>* optics2D)
{
	m_views = views;
	m_optics = optics2D;
}

void ImageManager::attach(QHash<QString, QLineScan>* linescans, QHash<QString, OpticsInfo3D>* optics3D)
{
	m_linescans = linescans;
	m_optics3D = optics3D;
}

void ImageManager::update_stitchData()
{
	ct::logger::trace("Update stitch map");

	if (m_views != nullptr) {
		if (m_views->size()) {
			//view
			for (auto& s : *m_views) {
				if (!s.map_to_sview.isEmpty()) {
					ct::logger::trace("S: %s", s.map_to_sview.toStdString().c_str());
					m_stitchMap[s.map_to_sview].imageStatus.insert(s.id, false);
				}
			}
		}
	}


	//Reason to append optics in hash as key is because the images does not come in groups. 
	//Scanning takes time, therefore, its better to send the first optics in for stitching and proceed to wait for 2nd optics to come in after scan is done.
	if (m_linescans != nullptr) {
		if (m_linescans->size()) {
			//linescan
			for (auto& s : *m_linescans) {
				if (!s.map_to_slinescan.isEmpty()) {
					ct::logger::trace("S: %s", s.map_to_slinescan.toStdString().c_str());
					
					for (auto& o : *m_optics3D) {
						auto opticID = o.id;

						if (o.exposureMode == ct::s_parallel) {
							opticID = o.id + "E1";
						}

						auto hid = s.map_to_slinescan + "_HeightMap_" + opticID;
						m_stitchMap[hid].imageStatus.insert(s.id, false);

						auto iid = s.map_to_slinescan + "_IMap";
						m_stitchMap[iid].imageStatus.insert(s.id, false);

						if (o.exposureMode == ct::s_parallel) {
							opticID = o.id + "E2";

							auto hid = s.map_to_slinescan + "_HeightMap_" + opticID;
							m_stitchMap[hid].imageStatus.insert(s.id, false);

							auto iid = s.map_to_slinescan + "_IMap";
							m_stitchMap[iid].imageStatus.insert(s.id, false);
						}
					}
				}
			}
		}
	}

	//line
	/*for (auto& s : m_stitchMap.keys()) {
		for (auto is : m_stitchMap[s].imageStatus.keys()) {
			ct::logger::trace("Stitch map: %s -> %s", s.toStdString().c_str(), is.toStdString().c_str());
		}
	}*/
}

void ImageManager::update_opticsData()
{
	//For future expansion
	//Mainly when certain views only use specific lighting
}

void ImageManager::update_zCount()
{
	if (m_views != nullptr) {
		for (auto& v : *m_views) {
			if (v.zstack.generate_2D_stack || v.zstack.generate_3D_stack) {
				
				for (auto& opticID : v.opticIDs) {

					auto id = util::combineID(v.id, opticID);
					int count = 0;

					if (v.zstack.acq_type == ct::s_preset) {
						count = v.zstack.preset_iteration;
					}
					else if (v.zstack.acq_type == ct::s_encoder) {
						count = v.zstack.encoder_range_um / v.zstack.step_um;
					}

					m_zCount[id] = count;
				}
			}
		}
	}
}

void ImageManager::queueStackImage(QString id) {
	{
		std::lock_guard<std::mutex> lock(m_zstackDequeMutex);
		m_zstackDeque.push_back(id);
	}
	//if does not wait for offset push front, always prioritize the last stack
	ct::logger::info("[Zstack] Queue stack image: %s", id.toStdString().c_str());
	m_zstackCV.notify_one();
}

void ImageManager::process_stack_queues_thread()
{
	while (!m_release) {
		QString id;

		{
			std::unique_lock<std::mutex> lock(m_zstackDequeMutex);
			m_zstackCV.wait(lock, [this]() { return m_release || !m_zstackDeque.empty(); });
			if (m_release) return;

			id = m_zstackDeque.front();

			ct::logger::info("Waiting for zstack data: %s", id.toStdString().c_str());

			while (!m_release) {
				{
					std::unique_lock<std::mutex> lock(m_zstackMapMutex);
					if (m_zstackMap.contains(id)) {
						break;
					}
				}
				os_tool::goSleep(500);
				//wait for zstack data until receive
			}
			if (m_release) return;
			ct::logger::info("Received zstack data: %s", id.toStdString().c_str());

			m_zstackDeque.pop_front();
		}

		stackImage(id); 
	}
}

void ImageManager::stackImage(QString id)
{
	QVector<FrameInfo> zstacks;

	{
		std::lock_guard<std::mutex> lock(m_zstackMapMutex);
		if (!m_zstackMap.contains(id)) {
			ct::logger::error("Invalid zstack ID: %s", id.toStdString().c_str());
			return;
		}

		zstacks = m_zstackMap[id];
	}

	ct::logger::info("[Zstack] Start: %s", id.toStdString().c_str());

	int index = 0;

	MEMORYSTATUSEX memStatus;
	memStatus.dwLength = sizeof(memStatus);
	GlobalMemoryStatusEx(&memStatus);
	size_t availableMemory = memStatus.ullAvailPhys; // Available memory in bytes

	ct::logger::info("[Zstack] Get memory status");

	FrameInfo stackedFrame;

	std::vector<cv::Mat> input_images;
	cv::cuda::GpuMat output_image;

	for (auto& frame : zstacks) {
		cv::Mat cvImg;
		util::Mil_to_cv(frame.pImage->id(), cvImg);
		input_images.push_back(cvImg);

		if (index == 0) {
			stackedFrame = frame;
		}

		index++;
	}

	ct::logger::info("[Zstack] Input images: %d", zstacks.size());

	focus_stacking_wrapper(input_images, output_image, availableMemory);
	
	ct::logger::info("[Zstack] Perform focus stacking");

	cv::Mat output;
	output_image.download(output);

	ct::logger::info("[Zstack] Download from GPU");

	cv::imwrite("zstack.bmp", output);
	ct::logger::info("[Zstack] Save CV image");

	MIL_ID mOutput;
	MbufImportA("zstack.bmp", M_BMP, M_RESTORE + M_NO_COMPRESS, M_DEFAULT_HOST, &mOutput);
	ct::logger::info("[Zstack] Load to MIL");

	auto sharedOutput = mtrx::MbufPoolManager::instance().attach(mOutput, mtrx::PoolDestructorType::FREE_BUFFER);

	{
		std::lock_guard<std::mutex> lock(m_zstackMapMutex);
		m_zstackMap.remove(id);
	}

	stackedFrame.pImage = sharedOutput;
	stackedFrame.postTask.stackImage = false;
	stackedFrame.channel = mtrx::get_band(mOutput);
	stackedFrame.width = mtrx::get_width(mOutput);
	stackedFrame.height = mtrx::get_height(mOutput);
	stackedFrame.bufferSize = stackedFrame.width * stackedFrame.height * stackedFrame.channel;
	rotate_image(stackedFrame.pImage->id(), stackedFrame.pImage->id(), stackedFrame.postTask.rotationalAngle);
	preprocess_done(stackedFrame);
}

void ImageManager::resume()
{
	m_condition.wakeOne();
}

void ImageManager::wait()
{
	QMutexLocker locker(&m_mutex);
	m_condition.wait(&m_mutex);
}

void ImageManager::preprocess_info(FrameInfo& info)
{
	auto s = QString("[IM] Preprocess info: %1_%2").arg(info.viewID).arg(info.opticID);
	ScopedTimeLogger Stimer(s.toStdString().c_str());
	/*
	1. Determine the format of the data
	2. Do task: Rotate, Conversion, Combine RGB
	3. Check if after image is preprocessed, is there a need for stitching or zstack
	*/

	auto cid = util::combineID(info.viewID, info.opticID);
	ct::logger::debug("[IM] Preprocessing image: %s", cid.toStdString().c_str());

	FrameInfo preprocessedInfo;
	preprocessedInfo = info;

	double rotationAgle = info.postTask.rotationalAngle;
	bool formRGB = info.postTask.combineRGB;

	ct::logger::debug("[IM] Pixel Type: %ld, %s", info.pixelFormat, info.type.toStdString().c_str());
	ct::logger::debug("[IM] Form RGB: %d", formRGB);
	
	auto isMono = (info.pixelFormat == ICAM_pixelFormat::Mono8 && 
		info.type == ct::s_mono ||
		info.pixelFormat == ICAM_pixelFormat::Mono8 && info.type == ct::s_color || 
		info.pixelFormat == ICAM_pixelFormat::Mono12) && !formRGB;


	if (isMono) {
		ct::logger::debug("[IM] Preprocess 1: Mono");
		//Reuse the same buffer since nothing changed
		
		process_sizing(info);

		preprocessedInfo.pImage = info.pImage;
		if (!info.postTask.stackImage) rotate_image(info.pImage->id(), preprocessedInfo.pImage->id(), rotationAgle);

		process_rgbOffset(preprocessedInfo);

		preprocess_done(preprocessedInfo);

	}
	else if (info.pixelFormat == ICAM_pixelFormat::BayerGB8 && info.type == ct::s_color) {
		ct::logger::debug("[IM] Preprocess 2.1: BayerGB8");
		//Receive: BayerGB, Target: RGB8

		{
			ScopedTimeLogger Stimer("[IM] BayerGB to RGB");
			auto mbuf = mtrx::bayer_to_rgb(info.pImage->id(), M_BAYER_GB);
			preprocessedInfo.pImage = mtrx::MbufPoolManager::instance().attach(mbuf, mtrx::PoolDestructorType::FREE_BUFFER);
		}

		process_sizing(preprocessedInfo);

		if (!info.postTask.stackImage) rotate_image(preprocessedInfo.pImage->id(), preprocessedInfo.pImage->id(), rotationAgle);

		process_rgbOffset(preprocessedInfo);

		preprocess_done(preprocessedInfo);

	}
	else if (info.pixelFormat == ICAM_pixelFormat::BayerRG8 && info.type == ct::s_color) {
		ct::logger::debug("[IM] Preprocess 2.2: BayerRG8");
		//Receive: BayerRG, Target: RGB8

		{
			ScopedTimeLogger Stimer("[IM] BayerRG to RGB");
			auto mbuf = mtrx::bayer_to_rgb(info.pImage->id(), M_BAYER_RG);
			preprocessedInfo.pImage = mtrx::MbufPoolManager::instance().attach(mbuf, mtrx::PoolDestructorType::FREE_BUFFER);
		}
		process_sizing(preprocessedInfo);

		if (!info.postTask.stackImage) rotate_image(preprocessedInfo.pImage->id(), preprocessedInfo.pImage->id(), rotationAgle);

		process_rgbOffset(preprocessedInfo);

		ct::logger::debug("[IM] X1: %s", util::combineID(preprocessedInfo.viewID, preprocessedInfo.opticID).toStdString().c_str());
		preprocess_done(preprocessedInfo);

	}
	else if (info.pixelFormat == ICAM_pixelFormat::RGB8 && info.type == ct::s_color) {
		ct::logger::debug("[IM] Preprocess 3: RGB8");
		process_sizing(info);
		preprocessedInfo.pImage = info.pImage;
		if (!info.postTask.stackImage) rotate_image(preprocessedInfo.pImage->id(), preprocessedInfo.pImage->id(), rotationAgle);
		process_rgbOffset(preprocessedInfo);
		preprocess_done(preprocessedInfo);

	}
	else if (info.pixelFormat == ICAM_pixelFormat::Mono8 && info.type == ct::s_color && formRGB) {
		ct::logger::debug("[IM] Preprocess 4: Form RGB from Mono");
		process_sizing(info);
		if (!info.postTask.stackImage) rotate_image(info.pImage->id(), info.pImage->id(), rotationAgle);
		process_combination(info);

	}
	else if (info.type == ct::s_height_map) {
		ct::logger::debug("[IM] Preprocess 5: Heightmap");

		MIL_ID rotatedHM = M_NULL, rotatedIM = M_NULL;
		rotate_heightMap(info.pHeightMap->id(), rotatedHM, rotationAgle, info.postTask.scanReversed);
		preprocessedInfo.pHeightMap = mtrx::MbufPoolManager::instance().attach(rotatedHM, mtrx::PoolDestructorType::FREE_BUFFER);

		if (info.pImage) {
			rotate_heightMap(info.pImage->id(), rotatedIM, rotationAgle, info.postTask.scanReversed);
			preprocessedInfo.pImage = mtrx::MbufPoolManager::instance().attach(rotatedIM, mtrx::PoolDestructorType::FREE_BUFFER);
		}

		preprocessedInfo.type = info.type;
		preprocessedInfo.width = mtrx::get_width(preprocessedInfo.pHeightMap->id());
		preprocessedInfo.height = mtrx::get_height(preprocessedInfo.pHeightMap->id());

		preprocess_done(preprocessedInfo);
	}
	else {
		ct::logger::error("[IM] Image pixel type not supported: %s_%s"
			, preprocessedInfo.viewID.toStdString().c_str()
			, preprocessedInfo.opticID.toStdString().c_str());
	}
}

void ImageManager::process_sizing(FrameInfo& info)
{
	ct::logger::debug("in process_sizing");
	if (m_views->contains(info.viewID)) {
		auto v = (*m_views)[info.viewID];
		//add flatfield
		if (v.preprocess.flatfield) {
		//1. load images
			ct::logger::info("In flatfield");
			const QString cfgPath = QDir(Common::Directory::getRecipeCurrentPath()).filePath("greycard.json");
			const QString key = "greycard";
			MilEntry& entry = s_milCache[key];

			// 2) If not loaded yet, read JSON and load into cache
			if (entry.id == M_NULL) {
				QFile f(cfgPath);
				if (!f.exists() || !f.open(QIODevice::ReadOnly | QIODevice::Text)) {
					ct::logger::error("[IM] greycard.json missing or cannot open: %s",
						cfgPath.toStdString().c_str());
					return;
				}
				const auto doc = QJsonDocument::fromJson(f.readAll());
				f.close();
				if (!doc.isObject()) {
					ct::logger::error("[IM] greycard.json invalid JSON.");
					return;
				}
				const QString savedPath = doc.object().value("greycardPath").toString().trimmed();
				if (savedPath.isEmpty() || !QFileInfo::exists(savedPath)) {
					ct::logger::error("[IM] grey card path empty or missing: %s",
						savedPath.toStdString().c_str());
					return;
				}

				// If path changed and we somehow had an old buffer (shouldn't for first run), free it.
				if (!entry.path.isEmpty() && entry.path != savedPath && entry.id != M_NULL) {
					mtrx::free_buffer(entry.id);
				}

				entry.path = savedPath;
				entry.id = MbufRestoreA(savedPath.toUtf8().constData(), M_DEFAULT_HOST, M_NULL);
				if (entry.id == M_NULL) {
					ct::logger::error("[IM] Failed to load grey card image: %s",
						savedPath.toStdString().c_str());
					return;
				}
			}

			// 3) Use the cached MIL_ID
			MIL_ID originalGreyCard = entry.id;
			if (originalGreyCard == M_NULL) {
				std::cerr << "Error: Could not load the grey card image." << std::endl;
				return;
			}

			// This MIL_ID will hold the final grayscale image to be processed.
			MIL_ID greyCardMono = M_NULL;
			bool freeGreyCardMono = false;

			// Inquire the number of bands to check if it's a color image
			MIL_INT numBands = 0;
			MbufInquire(originalGreyCard, M_SIZE_BAND, &numBands);

			if (numBands == 3)
			{
				std::cout << "Image is color. Converting to grayscale." << std::endl;

				// Inquire dimensions to create a new buffer
				MIL_INT sizeX, sizeY;
				MbufInquire(originalGreyCard, M_SIZE_X, &sizeX);
				MbufInquire(originalGreyCard, M_SIZE_Y, &sizeY);

				// Allocate a new 8-bit monochrome buffer for the grayscale image
				MbufAlloc2d(M_DEFAULT_HOST, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &greyCardMono);

				// Convert the 3-band color image to a 1-band grayscale image
				MimConvert(originalGreyCard, greyCardMono, M_RGB_TO_L);
		
				// Free the original color image as it's no longer needed
				//MbufFree(originalGreyCard);
				freeGreyCardMono = true;
			}
			else
			{
				std::cout << "Image is already grayscale. No conversion needed." << std::endl;
				// The original image is already grayscale, so we'll just use it directly
				greyCardMono = originalGreyCard;
			}


			MIL_INT sizeX, sizeY, type;
			MbufInquire(info.pImage->id(), M_SIZE_X, &sizeX);
			MbufInquire(info.pImage->id(), M_SIZE_Y, &sizeY);
			MbufInquire(info.pImage->id(), M_TYPE, &type);

			double mean, stdDev;

			MIL_ID MilStatContext = MimAlloc(M_DEFAULT_HOST, M_STATISTICS_CONTEXT, M_DEFAULT, M_NULL);
			MIL_ID MilStatResult = MimAllocResult(M_DEFAULT_HOST, M_DEFAULT, M_STATISTICS_RESULT, &MilStatResult);

			MimControl(MilStatContext, M_STAT_MEAN, M_ENABLE);
			MimControl(MilStatContext, M_STAT_STANDARD_DEVIATION, M_ENABLE);

			MimStatCalculate(MilStatContext, greyCardMono, MilStatResult, M_DEFAULT);
			MimGetResult(MilStatResult, M_STAT_MEAN + M_TYPE_MIL_DOUBLE, &mean);
			MimGetResult(MilStatResult, M_STAT_STANDARD_DEVIATION + M_TYPE_MIL_DOUBLE, &stdDev);

			double targetValue = mean + stdDev;
			// 2. Form correction map
			// Allocate a floating-point buffer for the correction map
			MIL_ID correctionMap = MbufAlloc2d(M_DEFAULT_HOST, sizeX, sizeY, 32 + M_FLOAT, M_IMAGE + M_PROC, M_NULL);

			// Create the correction map by dividing the target value by the grey card image.
			// This creates a map of factors to equalize the image intensity.
			MimArith(targetValue, greyCardMono, correctionMap, M_CONST_DIV);


			// 3. Apply the correction map
			// Allocate a buffer for the corrected image
			MIL_ID correctedImg = MbufAllocColor(M_DEFAULT_HOST, 3, sizeX, sizeY, type, M_IMAGE + M_PROC, M_NULL);
			Timer time;
			// Multiply the input image with the correction map to apply the correction.
			// M_SATURATION ensures that pixel values that overflow or underflow are clipped to the valid range. [1]
			MimArith(info.pImage->id(), correctionMap, correctedImg, M_MULT | M_SATURATION);

			info.pImage = mtrx::MbufPoolManager::instance().attach(correctedImg);

			ct::logger::info("flat Field time: %.2fms", time.duration());
			time.reset_timer();
			// Save the corrected image
			//MbufSaveA(outputPath.c_str(), correctedImg);

			// 4. Free all allocated MIL resources
			if (MilStatResult) MimFree(MilStatResult);
			if (MilStatContext) MimFree(MilStatContext);
			if (correctionMap) MbufFree(correctionMap);
			if (correctedImg) MbufFree(correctedImg);
			//MbufFree(img);
			if (freeGreyCardMono && greyCardMono) {
				MbufFree(greyCardMono);               
			}
		}

		if (v.preprocess.crop) {
			ScopedTimeLogger stl("[IM] Crop image");
			auto& c = v.preprocess.cropRect;
			auto mCrop = mtrx::crop(info.pImage->id(), c.x(), c.y(), c.width(), c.height());
			info.pImage = mtrx::MbufPoolManager::instance().attach(mCrop);

			MIL_INT band = MbufInquire(info.pImage->id(), M_SIZE_BAND, M_NULL);
			info.width = c.width();
			info.height = c.height();
			info.bufferSize = band * info.width, info.height;
		}

		if (v.preprocess.resize) {
			ScopedTimeLogger stl("[IM] Resize image");
			auto& s = v.preprocess.resizeRect;
			info.width = s.width();
			info.height = s.height();

			MIL_INT band = MbufInquire(info.pImage->id(), M_SIZE_BAND, M_NULL);
			MIL_INT type = MbufInquire(info.pImage->id(), M_TYPE, M_NULL);
			MIL_INT attribute = MbufInquire(info.pImage->id(), M_EXTENDED_ATTRIBUTE, M_NULL);

			info.bufferSize = band * info.width, info.height;

			MIL_ID mBufDst = M_NULL;
			MbufAllocColor(M_DEFAULT_HOST, band, info.width, info.height, type, attribute, &mBufDst);
			MimResize(info.pImage->id(), mBufDst, M_FILL_DESTINATION, M_FILL_DESTINATION, M_INTERPOLATE);
			info.pImage = mtrx::MbufPoolManager::instance().attach(mBufDst);
		}
	}
}

void ImageManager::rotate_image_discrete(FrameInfo& info)
{
	const int angle = SystemData::instance()._camImageRotation;
	if (angle == 0) return;
	if (!info.pImage) return;
	if (info.type == ct::s_height_map) return; //camera images only

	ScopedTimeLogger stl("[IM] Rotate camera image " + std::to_string(angle));

	MIL_ID src = info.pImage->id();
	MIL_INT band = MbufInquire(src, M_SIZE_BAND, M_NULL);
	MIL_INT type = MbufInquire(src, M_TYPE, M_NULL);
	MIL_INT w = MbufInquire(src, M_SIZE_X, M_NULL);
	MIL_INT h = MbufInquire(src, M_SIZE_Y, M_NULL);
	MIL_INT attribute = MbufInquire(src, M_EXTENDED_ATTRIBUTE, M_NULL);

	//90/270 swap the buffer dimensions
	MIL_INT dw = (angle == 180) ? w : h;
	MIL_INT dh = (angle == 180) ? h : w;

	MIL_ID mDst = M_NULL;
	MbufAllocColor(M_DEFAULT_HOST, band, dw, dh, type, attribute, &mDst);
	MimRotate(src, mDst, angle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NEAREST_NEIGHBOR + M_OVERSCAN_CLEAR);

	info.pImage = mtrx::MbufPoolManager::instance().attach(mDst);
	info.width = (int)dw;
	info.height = (int)dh;
	info.bufferSize = (int)(band * dw * dh);
}

void ImageManager::preprocess_done(FrameInfo& info)
{
	auto id = util::combineID(info.viewID, info.opticID);

	ct::logger::debug("[IM] Done preprocessing");
	auto s = QString("[IM] Preprocess done: %1").arg(id);
	ScopedTimeLogger Stimer(s.toStdString().c_str());

	//camera alignment rotation option (90/180/270), applied before the image leaves preprocessing
	rotate_image_discrete(info);

	ct::logger::debug("[IM] Emit image preprocessed");
	emit imagePreprocessed(info);


	//Handle non recipe image
	if (!(*m_views).contains(info.viewID) && !(*m_linescans).contains(info.viewID)) {
		ct::logger::info("[IM] None recipe image received, no action required to pass to ImageReady");
		QVector<FrameInfo> infos;
		infos.push_back(info);
		emit imageReady(infos);
		return;
	}

	if (info.postTask.stackImage) {
		if (!m_zstackMap.contains(id)) m_zstackMap[id].reserve(1000);
		m_zstackMap[id].push_back(info);
		return;
	}

	{
		std::lock_guard<std::mutex> lock(m_infosMapMutex);
		m_infosMap[getInfoID(info)].push_back(info);
	}
	
	//check if all optics have been acquired
	if (!all_optics_acquired(info)) return;

	process_final_image(info);
	ct::logger::debug("[IM] Process final image");
}

void ImageManager::process_combination(FrameInfo& info)
{
	FrameInfo preprocessedInfo;
	preprocessedInfo = info;

	auto cid = util::combineID(info.viewID, info.opticID);

	if (!m_combineMap.contains(cid)) m_combineMap.insert(cid, QVector<FrameInfo>());
	m_combineMap[cid].push_back(info);

	auto& datas = m_combineMap[cid];
	ct::logger::info("Push into combine map for RGB: %s -> %d", cid.toStdString().c_str(), datas.size());

	int w = info.width;
	int h = info.height;


	if (datas.size() == 3) {
		ScopedTimeLogger Stimer("[IM] Form RGB image");

		//check if all 3 data are valid
		MIL_ID ptrR = M_NULL, ptrG = M_NULL, ptrB = M_NULL;

		for (auto& data : datas) {
			if (data.postTask.bandType == BandType::M) {
				ct::logger::error("[IM] Unable to process combination. Invalid band type: M");
				return;
			}
			else if (data.postTask.bandType == BandType::R) ptrR = data.pImage->id();
			else if (data.postTask.bandType == BandType::G) ptrG = data.pImage->id();
			else if (data.postTask.bandType == BandType::B) ptrB = data.pImage->id();
		}

		if (ptrR != M_NULL && ptrG != M_NULL && ptrB != M_NULL) {
			MIL_ID mRGB = M_NULL;
			mtrx::form_RGB(ptrR, ptrG, ptrB, mRGB);
			preprocessedInfo.pImage = mtrx::MbufPoolManager::instance().attach(mRGB);

			datas.clear();
			m_combineMap.remove(cid);

			process_rgbOffset(preprocessedInfo);
			preprocess_done(preprocessedInfo);
		}
		else {
			ct::logger::error("[IM] Unable to process combination. Insufficient Band Type to proceed: %d, %d, %d", ptrR, ptrG, ptrB);
		}
	}
}

void ImageManager::process_rgbOffset(FrameInfo& info)
{
	const auto& offset = info.postTask.rgbOffset;

	if (mtrx::is_color(info.pImage->id())) {
		if (offset.R != 0) mtrx::offset_intensity(info.pImage->id(), M_RED, offset.R);
		if (offset.G != 0) mtrx::offset_intensity(info.pImage->id(), M_GREEN, offset.G);
		if (offset.B != 0) mtrx::offset_intensity(info.pImage->id(), M_BLUE, offset.B);
	}
	else {
		if (offset.M != 0) mtrx::offset_intensity(info.pImage->id(), M_ALL_BANDS, offset.M);
	}
}

void ImageManager::process_final_image(const FrameInfo& info)
{
	QVector<FrameInfo> infos;

	{
		std::lock_guard<std::mutex> lock(m_infosMapMutex);
		infos = m_infosMap[getInfoID(info)];
	}

	auto sID = infos[0].stitchID;
	auto type = infos[0].type;
	auto w = infos[0].width;
	auto h = infos[0].height;

	bool isView = true;
	auto vID = infos[0].viewID;
	if (infos[0].type == ct::s_height_map) {
		isView = false;
	}

	if (sID.isEmpty()) {
		ct::logger::info("[IM] Image received, pass out: %s", vID.toStdString().c_str());

		emit imageReady(infos);

		{
			std::lock_guard<std::mutex> lock(m_infosMapMutex);
			m_infosMap.remove(getInfoID(info));
		}
	}
	else {

		if (!isView) {
			sID = sID + "_HeightMap_" + infos[0].opticID;
		}

		ct::logger::debug("[IM] Received image: %s", vID.toStdString().c_str());
		ct::logger::debug("[IM] Stitch ID: %s", sID.toStdString().c_str());
		if (m_stitchMap.contains(sID)) {

			if (m_stitchMap[sID].imageStatus.contains(vID)) {
				m_stitchMap[sID].imageStatus[vID] = true;

				if (readyToStitch(m_stitchMap[sID].imageStatus)) {
					ct::logger::info("[IM] Ready to stitch: %s", sID.toStdString().c_str());
					{
						std::lock_guard<std::mutex> lock(m_stitchDequeMutex);
						m_stitchDeque.push_back(sID);
					}
					m_stitchCV.notify_one();
				}
			}
			else {
				ct::logger::error("[IM] Image Manager stitch data not tally: %s", vID.toStdString().c_str());
			}
		}
	}
}

void ImageManager::rotate_image(FrameInfo& info)
{
	if (abs(info.postTask.rotationalAngle) > 0.001) {
		ScopedTimeLogger Stimer("Rotate image: " + std::to_string(info.postTask.rotationalAngle));
		MimRotate(info.pImage->id(), info.pImage->id(), info.postTask.rotationalAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
	}
}

void ImageManager::rotate_image(MIL_ID mSrc, MIL_ID mDst, double rotateAngle)
{
	if (abs(rotateAngle) > 0.001) {
		ScopedTimeLogger Stimer("Rotate image: " + std::to_string(rotateAngle));
		MimRotate(mSrc, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
	}
}

void ImageManager::printOptic3D()
{
	qDebug() << "Printing optic 3D";
	if (!m_optics3D->isEmpty())
	{
		for (auto& o : *m_optics3D)
		{
			qDebug() << "Name: " << o.name;
			qDebug() << "Divider: " << o.divider;
		}
	}
}

void ImageManager::rotate_heightMap(MIL_ID mSrc, MIL_ID& mDst, double rotateAngle, bool scanReversed)
{
	if (mSrc == M_NULL) return;

	/*
	* A scan taught in the negative direction along the scan axis delivers its profiles in
	* reverse order, so the raw frame arrives mirrored along that axis. Normalise it HERE,
	* before anything else, and every branch below then behaves exactly as it does for a
	* forward scan. Flipping at the end instead would not be equivalent: a flip and a rotation
	* conjugate rather than commute, so the fine skew rotation would come out with the wrong
	* sense, and the invertX/invertY flags would apply to the wrong edges.
	*
	* One vertical flip covers every backend and both scan axes, because rows are profiles in
	* the raw frame regardless of API - each branch below asserts exactly that by computing its
	* scan extent from the buffer height. Copied rather than flipped in place: mSrc belongs to
	* the caller and returns to the buffer pool.
	*/
	MIL_ID mFlipped = M_NULL;
	if (scanReversed) {
		mFlipped = MbufAlloc2d(M_DEFAULT, mtrx::get_width(mSrc), mtrx::get_height(mSrc),
			mtrx::get_type(mSrc) + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		if (mFlipped != M_NULL) {
			MimFlip(mSrc, mFlipped, M_FLIP_VERTICAL, M_DEFAULT);
			mSrc = mFlipped;
		}
		else {
			ct::logger::error("[ImageManager] Reverse scan: could not allocate the flip buffer. "
				"The height map will come out mirrored along the scan axis.");
		}
	}
	mtrx::BufferCollector bcFlipped(mFlipped); //frees on return; safe with M_NULL

	bool onTranslate = true;

	const QString& api = ProfilerManager::instance().getAPI();
	const bool& invertX = ProfilerManager::instance().getInvertX();
	const bool& invertY = ProfilerManager::instance().getInvertY();

	auto scale = ScaleManager::instance().um_per_px();
	double extraPx= ScaleManager::instance().mm_to_px(SystemData::instance().m_extraMoveFor3DLaser);

	//X-axis scans need the raw frame remapped 90 degrees so the scan direction lies along image X;
	//Y-axis scans keep the raw orientation (profiles already stack along image Y)
	const bool scanAlongY = SystemData::instance().isLineScanAxisY();

	if (api == "Gocator") {
		ct::logger::info("[ImageManager] rotate_heightmap---Gocator");
		auto w = mtrx::get_width(mSrc);
		auto h = mtrx::get_height(mSrc);

		MIL_INT sw = w * 7 / scale;
		MIL_INT sh = h * 7 / scale;

		auto type = mtrx::get_type(mSrc);

		if (scanAlongY) {
			mDst = MbufAlloc2d(M_DEFAULT, sw, sh, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MimResize(mSrc, mDst, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);
			MimRotate(mDst, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
		}
		else {
			auto mRotate = MbufAlloc2d(M_DEFAULT, h, w, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			mDst = MbufAlloc2d(M_DEFAULT, sh, sw, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MimRotate(mSrc, mRotate, 90, w / 2, h / 2, h / 2, w / 2, M_BICUBIC + M_OVERSCAN_ENABLE);
			MimRotate(mRotate, mRotate, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
			MimResize(mRotate, mDst, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);
			mtrx::BufferCollector bc(mRotate);
		}
	}
	else if (api == "SmartRay") {

		int divider = 1;
		if (!m_optics3D->isEmpty())
		{
			for (auto& o : *m_optics3D)
			{
				divider = o.divider;
				break;
			}
		}

		ct::logger::info("[ImageManager] rotate_heightmap---SmartRay");
		auto w = mtrx::get_width(mSrc);
		auto h = mtrx::get_height(mSrc);

		MIL_INT sw = w * 6.3 / scale;
		MIL_INT sh = h * 4 * divider / scale;
		//MIL_INT sh = h * 4 / scale;

		auto type = mtrx::get_type(mSrc);

		if (scanAlongY) {
			mDst = MbufAlloc2d(M_DEFAULT, sw, sh, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MimResize(mSrc, mDst, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);

			if (onTranslate && extraPx != 0.0) {
				MimTranslate(mDst, mDst, 0.0, -extraPx, M_BICUBIC + M_OVERSCAN_CLEAR); //extra move is along the scan (Y) axis
			}

			//laser line lies along image X for Y-axis scans; mirror it like the vertical flip does for X-axis scans
			MimFlip(mDst, mDst, M_FLIP_HORIZONTAL, M_DEFAULT);

			MimRotate(mDst, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
		}
		else {
			auto mResize = MbufAlloc2d(M_DEFAULT, sw, sh, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MimResize(mSrc, mResize, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);

			if (onTranslate && extraPx != 0.0) {
				MimTranslate(mResize, mResize, -extraPx, 0.0, M_BICUBIC + M_OVERSCAN_CLEAR);
			}

			mDst = MbufAlloc2d(M_DEFAULT, sh, sw, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MimRotate(mResize, mDst, 90, sw / 2, sh / 2, sh / 2, sw / 2, M_BICUBIC + M_OVERSCAN_ENABLE);
			MimFlip(mDst, mDst, M_FLIP_VERTICAL, M_DEFAULT);

			//double centerHeight = (h - 580 - 507) / 2 + 507;
			MimRotate(mDst, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);

			mtrx::BufferCollector bc(mResize);
		}
	}
	//else if (api == "SSZN") {

	//	int divider = 1;
	//	if (!m_optics3D->isEmpty())
	//	{
	//		for (auto& o : *m_optics3D)
	//		{
	//			divider = o.divider;
	//			break;
	//		}
	//	}

	//	ct::logger::info("[ImageManager] rotate_heightmap---SSZN");
	//	auto w = mtrx::get_width(mSrc);
	//	auto h = mtrx::get_height(mSrc);


	//	MIL_INT sw = w * 3.5 / scale;
	//	MIL_INT sh = h * 4 * divider / scale;
	//	//Hardcoded scale need be dynamic in the future
	//	auto type = mtrx::get_type(mSrc);
	//	auto mResize = MbufAlloc2d(M_DEFAULT, sw, sh, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	//	MimResize(mSrc, mResize, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);
	//	mtrx::BufferCollector bc(mResize);

	//	if (onTranslate && extraPx != 0.0) {
	//		MimTranslate(mResize, mResize, -extraPx, 0.0, M_BICUBIC + M_OVERSCAN_CLEAR);
	//	}

	//	mDst = MbufAlloc2d(M_DEFAULT, sh, sw, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	//	MimRotate(mResize, mDst, 90, sw / 2, sh / 2, sh / 2, sw / 2, M_BICUBIC + M_OVERSCAN_ENABLE);
	//	MimRotate(mDst, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
	//	
	//	if (invertX)
	//	{
	//		MimFlip(mDst, mDst, M_FLIP_HORIZONTAL, M_DEFAULT);
	//	}
	//	if (invertY)
	//	{
	//		MimFlip(mDst, mDst, M_FLIP_VERTICAL, M_DEFAULT);
	//	}
	//}

	else if (api == "SSZN") {

		int divider = 1;
		if (!m_optics3D->isEmpty())
		{
			for (auto& o : *m_optics3D)
			{
				divider = o.divider;
				break;
			}
		}

		ct::logger::info("[ImageManager] rotate_heightmap---SSZN with Padding Compensation");
		auto w = mtrx::get_width(mSrc);
		auto h = mtrx::get_height(mSrc);

		MIL_INT sw = w * 3.5 / scale;
		MIL_INT sh_content = h * 4 * divider / scale;


		double paddingRatio = 0.0042 / 0.004;
		MIL_INT sh_total = (MIL_INT)(sh_content * paddingRatio);

		auto type = mtrx::get_type(mSrc);

		if (scanAlongY) {
			mDst = MbufAlloc2d(M_DEFAULT, sw, sh_total, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MbufClear(mDst, 0.0);
			MIL_ID mResizeChild = M_NULL;
			MbufChild2d(mDst, 0, 0, sw, sh_content, &mResizeChild);
			MimResize(mSrc, mResizeChild, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);
			MbufFree(mResizeChild);

			if (onTranslate && extraPx != 0.0) {
				MimTranslate(mDst, mDst, 0.0, -extraPx, M_BICUBIC + M_OVERSCAN_CLEAR); //extra move is along the scan (Y) axis
			}

			MimRotate(mDst, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);

			if (invertX)
			{
				MimFlip(mDst, mDst, M_FLIP_HORIZONTAL, M_DEFAULT);
			}
			if (invertY)
			{
				MimFlip(mDst, mDst, M_FLIP_VERTICAL, M_DEFAULT);
			}
		}
		else {
			auto mResize = MbufAlloc2d(M_DEFAULT, sw, sh_total, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MbufClear(mResize, 0.0);
			MIL_ID mResizeChild = M_NULL;
			MbufChild2d(mResize, 0, 0, sw, sh_content, &mResizeChild);
			MimResize(mSrc, mResizeChild, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);
			MbufFree(mResizeChild);

			mtrx::BufferCollector bc(mResize);

			if (onTranslate && extraPx != 0.0) {
				MimTranslate(mResize, mResize, -extraPx, 0.0, M_BICUBIC + M_OVERSCAN_CLEAR);
			}

			mDst = MbufAlloc2d(M_DEFAULT, sh_total, sw, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

			MimRotate(mResize, mDst, 90, sw / 2, sh_total / 2, sh_total / 2, sw / 2, M_BICUBIC + M_OVERSCAN_ENABLE);
			MimRotate(mDst, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);

			if (invertX)
			{
				MimFlip(mDst, mDst, M_FLIP_HORIZONTAL, M_DEFAULT);
			}
			if (invertY)
			{
				MimFlip(mDst, mDst, M_FLIP_VERTICAL, M_DEFAULT);
			}
		}
	}


	else if (api == "KeyenceLJ") {

		int divider = 1;
		if (!m_optics3D->isEmpty())
		{
			for (auto& o : *m_optics3D)
			{
				divider = o.divider;
				break;
			}
		}

		/*
		* This divider is now only used by the offline fallback below - the live path takes
		* the sub-sampling count from the driver, which applied it. Still clamped, because a
		* 0 reaching the height maths gives sh == 0, MbufAlloc2d returns M_NULL, and the null
		* buffer propagates as "BufferPool: created a new pool: 0_0_0_0" -> "Trying to attach
		* a NULL ID", killing the ImageManager thread mid-scan with no useful error.
		*
		* The asymmetry that hid that: Profiler_Keyence::setDivider() clamps the very same
		* value with std::max(1, divider) and logs "setDivider OK (divider = 1)", so the
		* driver looked healthy while the image maths still saw 0.
		*/
		if (divider < 1)
		{
			ct::logger::warn("[ImageManager] optics3D divider is %d - clamping to 1. "
				"Fix the recipe; a 0 here used to take the app down.", divider);
			divider = 1;
		}

		ct::logger::info("[ImageManager] rotate_heightmap---KeyenceLJ");
		auto w = mtrx::get_width(mSrc);
		auto h = mtrx::get_height(mSrc);

		/*
		* X pitch is still a constant; Y pitch is not, and no longer lives here at all - see
		* the block below. Compare Profiler_SSZN, where 3.5 and 4 are the same two quantities
		* and both are still hardcoded.
		*
		*   KEYENCE_X_PITCH_UM - from the LJ-X8060 data sheet: "profile data interval 5 um,
		*                        profile data count 3200 points". Those two multiply out to
		*                        the 16 mm FAR-side X range the same sheet quotes, and to the
		*                        laser_fov_mm in VisionApp_CRUD.cpp / VisionApp_JSON.cpp.
		*                        Profiler_Keyence logs the controller's own figure on every
		*                        scan - "MEASURED LASER FOV = <mm> mm (<N> points @ <P> um)".
		*                        If <P> is not 5.0, the profile data interval has been changed
		*                        (the sheet says it is adjustable from 4 um up, and that doing
		*                        so also changes the X measurement range); the controller wins,
		*                        so copy <P> here and recompute laser_fov as <P> * 3200.
		*
		*   Y pitch            - gantry travel per encoder trigger. No data sheet can supply it:
		*                        it is a property of the machine, not the head, so it belongs in
		*                        config and is read from the driver below. Measured on
		*                        Codetrace-CK 2026-08-27 at 4.0 um per encoder count, from
		*                        155.999 mm of travel against 39000 counts.
		*
		* Known simplification: the sheet gives the X range as 15 mm at the NEAR limit and
		* 16 mm at FAR, so the true pitch varies by ~2% across the +/-7.3 mm Z range. A single
		* constant cannot express that, so X scale is exact at FAR and reads ~2% narrow at
		* NEAR. Below the other error sources for now; revisit if X accuracy ever dominates.
		*
		* Z pitch needs no attention here - zPitchForHead() maps 8060 to 0.8 um automatically.
		*/
		constexpr double KEYENCE_X_PITCH_UM = 5.0;    //LJ-X8060 default profile data interval

		/*
		* Y pitch comes from the DRIVER, not from a constant here. getLinePitchUm() returns
		* keyence.json's yPitchUm multiplied by the sub-sampling count setDivider() actually
		* pushed, which is exactly the quantity this line needs. That one call removes both
		* defects the old code carried:
		*
		*   - There used to be a KEYENCE_Y_PITCH_UM constant that had to be kept equal to
		*     keyence.json by hand. Nothing detected a mismatch; parts simply measured long
		*     or short along the scan axis, and setScanLength's batch count was sized from
		*     one value while the image was stretched by the other.
		*   - The divider was read from the FIRST entry in the optics hash, which is not
		*     necessarily the entry the scan used. The driver's copy is the one applied.
		*
		* The fallback only matters offline - profiler.json sets the API even when no driver
		* object exists, so this branch can run with nothing connected (replaying saved
		* images, for instance). It is the old constant, and it says so in the log.
		*/
		constexpr double KEYENCE_Y_PITCH_FALLBACK_UM = 10.0;

		double linePitchUm = ProfilerManager::instance().getLinePitchUm();
		if (linePitchUm <= 0.0) {
			linePitchUm = KEYENCE_Y_PITCH_FALLBACK_UM * divider;
			ct::logger::warn("[ImageManager] No profiler object - falling back to %.3f um per line "
				"(%.1f um x divider %d). Height maps are only to scale if that matches the machine.",
				linePitchUm, KEYENCE_Y_PITCH_FALLBACK_UM, divider);
		}

		/*
		* Output pixel pitch. Both axes are divided by the SAME number, which is what keeps the
		* aspect ratio right - that was never the question. The question is what that number is.
		*
		*   world scale (default) - ScaleManager's um_per_px, shared with the 2D camera, so the
		*       height map lines up with 2D views and with anything taught against them. On this
		*       machine that is ~27.5 um against the sensor's 5 um, so it costs a 5.5x resample.
		*
		*   native (recipe flag) - the sensor's X pitch, fixed. At the machine's 5 um across and
		*       5 um along that is 1:1 and nothing is resampled at all; a coarser line pitch is
		*       upsampled along the scan axis instead. The image no longer matches 2D views,
		*       which is why it is opt-in per recipe. See the note below for why it is pinned to
		*       the X pitch rather than derived from the two.
		*/
		double outPitchUm = scale;
		if (SystemData::instance()._heightMapNativeScale) {
			/*
			* Pinned to the X pitch, NOT derived from the line pitch - and that is the whole
			* point. Substituting the batch count, sh = profiles x linePitch / outPitch =
			* (distance / linePitch) x linePitch / outPitch = distance / outPitch: the line
			* pitch cancels, so the output size depends only on the scan distance. Change the
			* divider to buy speed and every image keeps its dimensions, which means ROIs and
			* algorithms taught against them keep working.
			*
			* Deriving it from the line pitch instead - min() or max() - would resize every
			* image the moment the divider moved, and silently invalidate everything taught.
			* max() also loses real X detail; min() is stable only while the line pitch stays
			* coarser than 5 um, and collapses at divider 1.
			*
			* The cost is that a coarse line pitch is UPSAMPLED along the scan axis: real data
			* is never discarded, but the extra rows are interpolated and carry no new
			* information. Memory is then set by the X pitch regardless of the divider.
			*/
			outPitchUm = KEYENCE_X_PITCH_UM;

			/*
			* Native resolution is roughly 32x the pixels here, and it grows with scan length: a
			* 50 mm scan is 32 Mpx, a 150 mm one is 96 Mpx, and each of those needs a second
			* buffer of the same size for the rotate. Rather than let a long scan quietly try to
			* allocate a gigabyte and fail somewhere unhelpful, back the pitch off and say so.
			*
			* This is the ONE case where the output size is not simply distance / X pitch, so it
			* breaks the size invariance above - but it only triggers past ~155 mm of scan, it
			* depends on the DISTANCE rather than on the divider, and it says so loudly. A recipe
			* whose scans are all the same length still gets identical dimensions every run.
			*/
			constexpr double MAX_OUTPUT_MPX = 100.0;
			const double mpx = (w * KEYENCE_X_PITCH_UM / outPitchUm)
				* (h * linePitchUm / outPitchUm) / 1.0e6;
			if (mpx > MAX_OUTPUT_MPX) {
				const double grow = std::sqrt(mpx / MAX_OUTPUT_MPX);
				ct::logger::warn("[ImageManager] Native 3D scale would be %.0f Mpx - coarsening the "
					"output pitch from %.2f to %.2f um to stay under %.0f Mpx",
					mpx, outPitchUm, outPitchUm * grow, MAX_OUTPUT_MPX);
				outPitchUm *= grow;
			}

			//extraPx was converted with the world scale at the top of this function, so it is in
			//the wrong units for a natively-scaled image. Redo it against the pitch in use.
			extraPx = SystemData::instance().m_extraMoveFor3DLaser * 1000.0 / outPitchUm;
		}

		MIL_INT sw = w * KEYENCE_X_PITCH_UM / outPitchUm;
		MIL_INT sh = h * linePitchUm / outPitchUm;

		ct::logger::info("[ImageManager] Keyence height map: %lld x %lld px at %.3f um/px (%s), "
			"from %lld x %lld raw at %.2f um across x %.3f um along",
			sw, sh, outPitchUm,
			SystemData::instance()._heightMapNativeScale ? "native" : "world scale",
			w, h, KEYENCE_X_PITCH_UM, linePitchUm);

		auto type = mtrx::get_type(mSrc);

		if (scanAlongY) {
			//profiles already stack along image Y, so no 90 degree remap
			mDst = MbufAlloc2d(M_DEFAULT, sw, sh, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MimResize(mSrc, mDst, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);

			if (onTranslate && extraPx != 0.0) {
				MimTranslate(mDst, mDst, 0.0, -extraPx, M_BICUBIC + M_OVERSCAN_CLEAR); //extra move is along the scan (Y) axis
			}

			MimRotate(mDst, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);
		}
		else {
			//scan direction must end up along image X, so resize then remap 90 degrees
			auto mResize = MbufAlloc2d(M_DEFAULT, sw, sh, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MimResize(mSrc, mResize, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BICUBIC + M_OVERSCAN_ENABLE);

			if (onTranslate && extraPx != 0.0) {
				MimTranslate(mResize, mResize, -extraPx, 0.0, M_BICUBIC + M_OVERSCAN_CLEAR);
			}

			mDst = MbufAlloc2d(M_DEFAULT, sh, sw, type + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
			MimRotate(mResize, mDst, 90, sw / 2, sh / 2, sh / 2, sw / 2, M_BICUBIC + M_OVERSCAN_ENABLE);
			MimRotate(mDst, mDst, rotateAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_BICUBIC + M_OVERSCAN_CLEAR);

			mtrx::BufferCollector bc(mResize);
		}

		//honour the profiler.json flags, as the SSZN branch does; Gocator/SmartRay ignore them
		if (invertX)
		{
			MimFlip(mDst, mDst, M_FLIP_HORIZONTAL, M_DEFAULT);
		}
		if (invertY)
		{
			MimFlip(mDst, mDst, M_FLIP_VERTICAL, M_DEFAULT);
		}
	}

	else {
		ct::logger::error("[Image Manager] Failed to get sensor api type", api.toStdString().c_str());
	}


}

QString ImageManager::getInfoID(const FrameInfo& info)
{
	if (m_idType == "None") {
		return info.viewID;
	}
	else if (m_idType == "Index") {
		return info.viewID + "_" + QString::number(info.index);
	}
	else if (m_idType == "RowCol") {
		return info.viewID + "_" + util::getRowColID(info.row, info.col);
	}

	return QString();
}

bool ImageManager::all_optics_acquired(const FrameInfo& info)
{
	int index;
	{
		std::lock_guard<std::mutex> lock(m_infosMapMutex);
		index = m_infosMap[getInfoID(info)].size();
	}

	if (info.type != ct::s_height_map) {

		if (!(*m_views).contains(info.viewID)) {
			ct::logger::warn("[IM] Invalid viewID found when checking for optics: %s", info.viewID.toStdString().c_str());
			return true;
		}

		int expectedOpticsNum = (*m_views)[info.viewID].opticIDs.size();
		ct::logger::debug("[IM] Optic size checking %s: %d vs %d", getInfoID(info).toStdString().c_str(), index, expectedOpticsNum);
		return (index == expectedOpticsNum);
	}
	else {
		auto numOptics = util::getOpticsSize(*m_optics3D);
		ct::logger::debug("[IM] 3D Optic check: %d <-> %d", index, numOptics);
		return (index == numOptics);
	}
	return false;
}

bool ImageManager::readyToStitch(const QHash<QString, bool>& status)
{
	for (auto key : status.keys()) {
		auto s = status[key];
		ct::logger::debug("Stitch status: %s -> %d", key.toStdString().c_str(), s);
		if (s == false) return false;
	}
	return true;
}

void ImageManager::on_the_fly_stitching(QString sID)
{
	// Create a Stitcher object
	cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::SCANS);
	cv::Mat stitched_image;
	cv::Stitcher::Status status = stitcher->stitch(m_stitchMap[sID].images, stitched_image);

	// Get the width and height of the image
	int width = stitched_image.cols;
	int height = stitched_image.rows;
	int channels = stitched_image.channels(); // Number of channels (e.g., 3 for BGR)

	// Calculate the total size of the image data in bytes
	size_t imageSize = width * height * channels;

	// Check if the stitching was successful
	if (status == cv::Stitcher::OK) {
		//cv::imwrite("test.jpg", stitched_image);
		QVector<FrameInfo> infos; //TODO: Finish for multi optics

		//Get first viewID
		auto vID = m_stitchMap[sID].imageStatus.keys()[0];

		//Get info map
		auto& iinfos = m_infosMap[vID];

		for (auto& iinfo : iinfos) {
			FrameInfo info;

			//// Allocate memory for the new unsigned char array
			//unsigned char* pBuf = new unsigned char[imageSize];

			//// Copy the data from the cv::Mat to the new unsigned char array
			//std::memcpy(pBuf, mono.data, imageSize);

			//TODO: Change this to multi optic once stitch is supported
			auto& iinfo = m_infosMap[vID];
			info.bufferSize = imageSize;
			info.width = width;
			info.height = height;
			info.pixelFormat = iinfo[0].pixelFormat;
			info.timeStamp = iinfo[0].timeStamp;
			info.cameraID = iinfo[0].cameraID;
			info.viewID = sID;
			info.stitchID = sID;
			info.type = iinfo[0].type;

			MIL_ID mbuf = M_NULL;
			util::cv_to_Mil(stitched_image, mbuf);
			info.pImage = mtrx::MbufPoolManager::instance().attach(mbuf);

			infos.push_back(info);

			break; //TODO: Remove this once multi optic is supported
		}

		ct::logger::info("[IM] Done stitching 2D: %s", sID.toStdString().c_str());
		emit imageReady(infos);

		{
			std::lock_guard<std::mutex> lock(m_infosMapMutex);
			m_infosMap.remove(vID);
		}
	}
	else if (status == cv::Stitcher::ERR_NEED_MORE_IMGS) {
		ct::logger::error("[ImageManager] Stitching failed. Need more images");
	}
	else if (status == cv::Stitcher::ERR_HOMOGRAPHY_EST_FAIL) {
		ct::logger::error("[ImageManager] Stitching failed. Homography estimation failed");
	}
	else if (status == cv::Stitcher::ERR_CAMERA_PARAMS_ADJUST_FAIL) {
		ct::logger::error("[ImageManager] Stitching failed. Camera parameters adjustment failed");
	}
	else {
		ct::logger::error("[ImageManager] Stitching failed. Unknown error");
	}
}

void ImageManager::mechanical_stitching(QString sID)
{
	ct::logger::info("[IM] Mechanical stitching: %s", sID.toStdString().c_str());
	auto sIDs = m_stitchMap[sID].imageStatus.keys(); ct::logger::trace("[IM] MS1");
	auto firstChildID = m_stitchMap[sID].imageStatus.keys()[0]; ct::logger::trace("[IM] MS2");

	QVector<FrameInfo> infos;
	{
		std::lock_guard<std::mutex> lock(m_infosMapMutex);
		infos = m_infosMap[firstChildID];
	}
	ct::logger::trace("[IM] MS3");

	if (infos.size()) {
		if (infos[0].type == ct::s_height_map) {
			ct::logger::trace("[IM] MS3d-1");
			mechanical_stitch_linescan(sID);
			ct::logger::trace("[IM] MS3d-2");
		}
		else {
			ct::logger::trace("[IM] MS2d-1");
			mechanical_blend_view(sID);
			/*if (SystemData::instance()._stitchingMethod == 1) {
				mechanical_stitch_view(sID);
			}
			else if (SystemData::instance()._stitchingMethod == 2) {
				mechanical_blend_view(sID);
			}
			else if (SystemData::instance()._stitchingMethod == 3) {
				guided_stitch_view(sID);
			}*/
			ct::logger::trace("[IM] MS2d-2");
		}
	}
}

void ImageManager::homography_stitching(QString sID)
{
}

void ImageManager::mechanical_stitch_view(QString sID)
{
	ScopedTimeLogger stl("[IM] Mechanical Stitching");

	QVector<FrameInfo> infos; 

	if (!m_views->contains(sID)) return;

	auto vIDs = m_stitchMap[sID].imageStatus.keys();
	auto firstViewID = m_stitchMap[sID].imageStatus.keys()[0];

	QHash<QString, QVector<FrameInfo>> infosMap;
	{
		std::lock_guard<std::mutex> lock(m_infosMapMutex);
		for (auto vID : vIDs) {
			infosMap.insert(vID, m_infosMap[vID]);
			m_infosMap.remove(vID);
		}
	}

	int opticSize = infosMap[firstViewID].size();

	auto sview = (*m_views)[sID];

	double xmin_mm = 999999, xmax_mm = 0;
	double ymin_mm = 999999, ymax_mm = 0;

	int view_width_px, view_height_px;

	for (auto vID : vIDs) {
		if (m_views->contains(vID)) {
			auto v = (*m_views)[vID];
			if (xmin_mm > v.world.wx) {
				xmin_mm = v.world.wx;
			}

			if (xmax_mm < v.world.wx) {
				xmax_mm = v.world.wx;
			}

			if (ymin_mm > v.world.wy) {
				ymin_mm = v.world.wy;
			}

			if (ymax_mm < v.world.wy) {
				ymax_mm = v.world.wy;
			}

			view_width_px = v.px.w;
			view_height_px = v.px.h;
		}
	}

	auto scale = ScaleManager::instance().um_per_px();
	int width_px = util::mm_to_px(xmax_mm - xmin_mm, scale) + view_width_px; //sview.px.w;
	int height_px = util::mm_to_px(ymax_mm - ymin_mm, scale) + view_height_px;//sview.px.h;

	int offset_x_px = view_width_px / 2;
	int offset_y_px = view_height_px / 2;

	ct::logger::debug("Stitch size: %d, %d\n", width_px, height_px);

	int imageSize = width_px * height_px;

	unsigned char* alphaBuf = nullptr;
	//auto alphaBuf = util::generateAlphaImage(5120, 5120, m_camAngle);

	for (int optIndex = 0; optIndex < opticSize; optIndex++) {
		QImage simage = QImage(width_px, height_px, QImage::Format_RGB32);
		simage.fill(Qt::transparent);
		QPainter painter(&simage);

		ICAM_pixelFormat pixelFormat;
		QString optID;
		QString camID;

		for (auto vID : vIDs) {

			if (!m_views->contains(vID)) continue;

			auto v = (*m_views)[vID];

			auto& iinfo = infosMap[vID][optIndex];

			if (SystemData::instance()._saveUnstitchedImages) {
				auto root = SystemData::instance()._workingPath + "Unstitched/";
				Common::Directory::createDir(root);
				QString filename = root + iinfo.viewID + "_" + iinfo.opticID + ".jpg";
				ct::logger::info("Saving unstitched image: %s", filename.toStdString().c_str());
				ImageSavingThread::instance().enqueue(filename.toStdString(), iinfo.pImage);
			}

			pixelFormat = iinfo.pixelFormat;
			optID = iinfo.opticID;
			camID = iinfo.cameraID;

			QImage qimg;
			util::Mil_to_qImg(iinfo.pImage->id(), qimg);


			qimg = qimg.convertToFormat(QImage::Format_ARGB32);

			auto scale = ScaleManager::instance().um_per_px();
			auto tx = util::mm_to_px(v.world.wx - xmin_mm, scale);// +offset_x_px;
			auto ty = util::mm_to_px(v.world.wy - ymin_mm, scale);// +offset_y_px;

			ct::logger::debug("Stitch Pos: %f, %f", tx, ty);

			int width = qimg.width();
			int height = qimg.height();

			auto camAngle = SystemData::instance()._camAngles["cam1"];
			if(alphaBuf == nullptr) alphaBuf = util::generateAlphaImage(width, height, camAngle);

			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					// Get the ARGB pixel value at (x, y)
					QRgb pixel = qimg.pixel(x, y);

					// Get the current alpha value
					int alpha = qAlpha(pixel);

					// Set the new alpha value from the unsigned char array
					alpha = alphaBuf[y * width + x];

					// Create a new pixel value with updated alpha
					pixel = qRgba(qRed(pixel), qGreen(pixel), qBlue(pixel), alpha);

					// Set the updated pixel value in the image
					qimg.setPixel(x, y, pixel);
				}
			}

			QPoint destPos = QPoint(tx, ty);
			painter.drawImage(destPos, qimg);
		}

		painter.end();

		FrameInfo info;
		info.bufferSize = imageSize;
		info.width = width_px;
		info.height = height_px;
		info.pixelFormat = pixelFormat;
		info.cameraID = camID;
		info.viewID = sID;
		info.stitchID = "";
		info.opticID = optID;
		info.type = ct::s_color;
		auto cid = util::combineID(sID, optID);

		MIL_ID mSBuf;
		util::qImg_to_Mil(simage, mSBuf);

		if (alphaBuf)
		{
			delete[] alphaBuf; alphaBuf = nullptr;
		}

		info.pImage = mtrx::MbufPoolManager::instance().attach(mSBuf);

		infos.push_back(info);
	}

	ct::logger::info("[IM] Done stitching 2D: %s", sID.toStdString().c_str());
	emit imageReady(infos);
}

void ImageManager::mechanical_blend_view(QString sID)
{
	ScopedTimeLogger stl("[IM] Mechanical Blend Stitching");

	QVector<FrameInfo> infos;

	const QString basePath = Common::Directory::CurrentImageSetPath + "\\Unstitched\\";
	const QString outputRoot = Common::Directory::CurrentImageSetPath + "\\Restitched\\";
	//Common::Directory::createDir(outputRoot);

	if (!m_views->contains(sID)) return;

	auto vIDs = m_stitchMap[sID].imageStatus.keys();
	auto firstViewID = m_stitchMap[sID].imageStatus.keys()[0];

	double camSize = 5120.0;
	double halfCamsize = camSize / 2;

	QHash<QString, QVector<FrameInfo>> infosMap;
	{
		std::lock_guard<std::mutex> lock(m_infosMapMutex);
		for (auto vID : vIDs) {
			infosMap.insert(vID, m_infosMap[vID]);
			m_infosMap.remove(vID);
		}
	}

	double xmin_mm = 999999, xmax_mm = 0;
	double ymin_mm = 999999, ymax_mm = 0;

	int numRow = 0, numCol = 0;

	int opticSize = infosMap[firstViewID].size();
	int view_width_px, view_height_px;

	for (auto& vID : vIDs) {

		if (!m_views->contains(vID)) {
			ct::logger::error("[IM] Invalid view ID: %s", vID.toStdString().c_str());
			continue;
		}

		auto& v = (*m_views)[vID];

		auto datas = vID.split("-");
		auto row = datas[1].toInt();
		auto col = datas[2].toInt();

		if (row > numRow) numRow = row;
		if (col > numCol) numCol = col;

		if (xmin_mm > v.world.wx) {
			xmin_mm = v.world.wx;
		}

		if (xmax_mm < v.world.wx) {
			xmax_mm = v.world.wx;
		}

		if (ymin_mm > v.world.wy) {
			ymin_mm = v.world.wy;
		}

		if (ymax_mm < v.world.wy) {
			ymax_mm = v.world.wy;
		}

		view_width_px = v.px.w;
		view_height_px = v.px.h;
	}

	auto scale = ScaleManager::instance().um_per_px();
	int width_px = util::mm_to_px(xmax_mm - xmin_mm, scale) + view_width_px; //sview.px.w;
	int height_px = util::mm_to_px(ymax_mm - ymin_mm, scale) + view_height_px;

	int imageSize = width_px * height_px;

	numRow++;
	numCol++;

	ct::logger::trace("[IM][Mechanical Blend Stitching] Row: %d, Col: %d", numRow, numCol);

	try {
		for (int optIndex = 0; optIndex < opticSize; optIndex++) {

			std::vector<vips::VImage> rowImages;
			std::vector<void*> rawBuffers;
			rawBuffers.reserve(1000);

			ICAM_pixelFormat pixelFormat;
			QString optID;
			QString camID;

			for (int row = 0; row < numRow; row++) {

				std::vector<vips::VImage> colImages;

				for (int col = 0; col < numCol; col++) {
					auto vID = sID + QString("-%1-%2").arg(row).arg(col);

					if (!m_views->contains(vID)) continue;

					auto& iinfo = infosMap[vID][optIndex];

					pixelFormat = iinfo.pixelFormat;
					optID = iinfo.opticID;
					camID = iinfo.cameraID;

					if (SystemData::instance()._saveUnstitchedImages) {
						auto root = SystemData::instance()._workingPath + "Unstitched/";
						Common::Directory::createDir(root);
						QString filename = root + iinfo.viewID + "_" + iinfo.opticID + ".jpg";
						ct::logger::info("Saving unstitched image: %s", filename.toStdString().c_str());
						ImageSavingThread::instance().enqueue(filename.toStdString(), iinfo);
					}

					void* data = nullptr;
					vips::VImage img = util::to_vimage(iinfo.pImage->id(), &data);
					if (data) rawBuffers.push_back(data);

					colImages.push_back(img);
				}

				// Horizontally stitch each row
				vips::VImage rowStitched = colImages[0];
				for (int col = 1; col < colImages.size(); ++col) {
					auto vID = sID + QString("-%1-%2").arg(row).arg(col);
					ct::logger::info("vID: %s", vID.toStdString().c_str());

					if (!(*m_views).contains(vID)) {
						ct::logger::error("[IM] Failed to guide stitching. Invalid view ID: %s", vID.toStdString().c_str());
						continue;
					}

					auto& v = (*m_views)[vID];

					auto scale = ScaleManager::instance().um_per_px();
					auto tx = util::mm_to_px(v.world.wx - xmin_mm, scale);
					auto ty = util::mm_to_px(v.world.wy - ymin_mm, scale);

					ct::logger::info("offset: %.2f, %.2f, %.2f", scale, tx, ty);

					auto overlapSize = (camSize * col) - tx;
					auto overlapHalfsize = overlapSize / 2.0;

					auto left_point = tx + overlapHalfsize;
					auto right_point = overlapHalfsize;

					// First stitch: img0 + img1
					rowStitched = rowStitched.merge(colImages[col], VIPS_DIRECTION_HORIZONTAL,
						-tx, 0,
						vips::VImage::option()->set("mblend", 300));

					//std::string outputPath = "horizontalStitch" + std::to_string(row) + "-" + std::to_string(col) + ".jpg";
					//rowStitched.write_to_file(outputPath.c_str());

					ct::logger::info("[IM] Done horizontal stitching: %s", vID.toStdString().c_str());
				}

				rowImages.push_back(rowStitched);
			}

			// Vertically stitch all rows
			vips::VImage finalStitched = rowImages[0];
			int firstCol = 0;
			for (int row = 1; row < rowImages.size(); ++row) {
				auto vID = sID + QString("-%1-%2").arg(row).arg(firstCol);
				ct::logger::info("vID: %s", vID.toStdString().c_str());

				if (!(*m_views).contains(vID)) {
					ct::logger::error("[IM] Failed to guide stitching. Invalid view ID: %s", vID.toStdString().c_str());
					continue;
				}

				auto& v = (*m_views)[vID];

				auto scale = ScaleManager::instance().um_per_px();
				auto tx = util::mm_to_px(v.world.wx - xmin_mm, scale);
				auto ty = util::mm_to_px(v.world.wy - ymin_mm, scale);

				auto overlapSize = (camSize * row) - ty;
				auto overlapHalfsize = overlapSize / 2.0;

				auto top_point = ty + overlapHalfsize;
				auto btm_point = overlapHalfsize;

				auto halfWidth = rowImages[row].width() / 2;

				// First stitch: img0 + img1
				finalStitched = finalStitched.merge(rowImages[row], VIPS_DIRECTION_VERTICAL,
					0, -ty,
					vips::VImage::option()->set("mblend", 300));

				//std::string outputPath = "verticalStitch" + std::to_string(row) + ".jpg";
				//finalStitched.write_to_file(outputPath.c_str());

				ct::logger::info("[IM] Done vertical stitching: %s", vID.toStdString().c_str());
			}

			/*QString outputPath = outputRoot + sID + "_" + optID + ".jpg";
			ct::logger::info("[IM] Saving stitched image: %s", outputPath.toStdString().c_str());
			finalStitched.write_to_file(outputPath.toStdString().c_str());*/

			FrameInfo info;
			info.bufferSize = imageSize;
			info.width = width_px;
			info.height = height_px;
			info.pixelFormat = pixelFormat;
			info.cameraID = camID;
			info.viewID = sID;
			info.stitchID = "";
			info.opticID = optID;
			info.type = ct::s_color;
			auto cid = util::combineID(sID, optID);

			auto mBuf = util::to_milID(finalStitched);

			auto band = mtrx::get_band(mBuf);
			auto type = mtrx::get_type(mBuf);
			auto attribute = mtrx::get_attribute(mBuf);
			info.pImage = mtrx::MPM::instance().acquire(info.width, info.height, band, type);

			mtrx::paste_to_center(mBuf, info.pImage->id());

			infos.push_back(info);

			ct::logger::info("[IM] Stitching infos size: %d", infos.size());

			mtrx::free_buffer(mBuf);
			for (void* ptr : rawBuffers) free(ptr);
		}
	}
	catch (const vips::VError& e) {
		std::cerr << "VIPS error: " << e.what() << std::endl;
		ct::logger::error("[IM] Failed to perform 2D stitching: %s", sID.toStdString().c_str());
		return;
	}

	ct::logger::info("[IM] Done stitching 2D: %s", sID.toStdString().c_str());
	ct::logger::info("[IM] Final Stitching infos size: %d", infos.size());
	emit imageReady(infos);
}

void ImageManager::mechanical_stitch_linescan(QString sID)
{
	QVector<FrameInfo> infos; 

	if (!m_stitchMap.contains(sID)) return;

	auto stitchID = sID;
	auto s = sID.split("_");
	if (s.size()) stitchID = s.at(0);

	auto vIDs = m_stitchMap[sID].imageStatus.keys();
	auto firstChildID = m_stitchMap[sID].imageStatus.keys()[0];
	QHash<QString, QVector<FrameInfo>> infosMap;
	{
		std::lock_guard<std::mutex> lock(m_infosMapMutex);
		for (auto vID : vIDs) {
			infosMap.insert(vID, m_infosMap[vID]);
			m_infosMap.remove(vID);
		}
	}

	int opticSize = infosMap[firstChildID].size();

	auto sline = (*m_linescans)[sID];

	const bool scanAlongY = SystemData::instance().isLineScanAxisY();

	double xmin_mm = 999999, xmax_mm = 0;
	double ymin_mm = 999999, ymax_mm = 0;

	//laser FOV size in px, perpendicular to the scan direction
	int view_span_px = 0;

	for (auto l : *m_linescans) {
		if (l.map_to_slinescan == stitchID) {
			if (xmin_mm > l.start_point.wx) {
				xmin_mm = l.start_point.wx;
			}

			if (xmax_mm < l.end_point.wx) {
				xmax_mm = l.end_point.wx;
			}

			if (ymin_mm > l.start_point.wy) {
				ymin_mm = l.start_point.wy;
			}

			if (ymin_mm > l.end_point.wy) {
				ymin_mm = l.end_point.wy;
			}

			if (ymax_mm < l.start_point.wy) {
				ymax_mm = l.start_point.wy;
			}

			if (ymax_mm < l.end_point.wy) {
				ymax_mm = l.end_point.wy;
			}

			if (xmin_mm > l.end_point.wx) {
				xmin_mm = l.end_point.wx;
			}

			if (xmax_mm < l.start_point.wx) {
				xmax_mm = l.start_point.wx;
			}

			view_span_px = scanAlongY ? l.px.w : l.px.h;
		}
	}

	auto scale = ScaleManager::instance().um_per_px();
	int width_px = util::mm_to_px(xmax_mm - xmin_mm, scale); //sview.px.w;
	int height_px = util::mm_to_px(ymax_mm - ymin_mm, scale); // sview.px.h;
	if (scanAlongY) width_px += view_span_px;
	else height_px += view_span_px;

	ct::logger::debug("XY (min, max): %f, %f, %f, %f\n", xmin_mm, ymin_mm, xmax_mm, ymax_mm);
	ct::logger::debug("Stitch size: %d, %d\n", width_px, height_px);

	int imageSize = width_px * height_px;

	for (int optIndex = 0; optIndex < opticSize; optIndex++) {
		//Create a blank grayscale OpenCV image
		auto simage8 = MbufAlloc2d(M_DEFAULT_HOST, width_px, height_px, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		auto simage16 = MbufAlloc2d(M_DEFAULT_HOST, width_px, height_px, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

		MIL_UINT8* shost8;
		MIL_ID spitch8 = M_NULL;
		MbufInquire(simage8, M_HOST_ADDRESS, &shost8);
		MbufInquire(simage8, M_PITCH, &spitch8);

		MIL_UINT16* shost16;
		MIL_ID spitch16 = M_NULL;
		MbufInquire(simage16, M_HOST_ADDRESS, &shost16);
		MbufInquire(simage16, M_PITCH, &spitch16);
		
		ICAM_pixelFormat pixelFormat;
		QString optID;
		QString camID;
		QString linescanID;
		bool hasIMAP = false;

		for (auto vID : vIDs) {
			if (!m_linescans->contains(vID)) continue;

			auto v = (*m_linescans)[vID];
			auto& iinfo = infosMap[vID][optIndex];
			linescanID = iinfo.viewID;
			pixelFormat = iinfo.pixelFormat;
			optID = iinfo.opticID;
			camID = iinfo.cameraID;

			// Calculate the transformed positions
			auto w = mtrx::get_width(iinfo.pHeightMap->id());
			auto h = mtrx::get_height(iinfo.pHeightMap->id());

			auto start_x = (int)util::mm_to_px(v.start_point.wx - xmin_mm, scale);
			auto start_y = (int)util::mm_to_px(v.start_point.wy - ymin_mm, scale);
			auto end_x = start_x + w;
			auto end_y = start_y + h;

			ct::logger::debug("Start: %d, %d - End: %d, %d", start_x, start_y, end_x, end_y);

			if (iinfo.pImage != M_NULL) {
				hasIMAP = true;

				MIL_UINT8* host8;
				MIL_ID pitch8 = M_NULL;
				MbufInquire(iinfo.pImage->id(), M_HOST_ADDRESS, &host8);
				MbufInquire(iinfo.pImage->id(), M_PITCH, &pitch8);

				MIL_UINT16* host16;
				MIL_ID pitch16 = M_NULL;
				MbufInquire(iinfo.pHeightMap->id(), M_HOST_ADDRESS, &host16);
				MbufInquire(iinfo.pHeightMap->id(), M_PITCH, &pitch16);

				for (int x = start_x; x < end_x; x++) {
					for (int y = start_y; y < end_y; y++) {

						if (x < 0 || y < 0 || x > width_px || y > height_px) continue;

						auto sx = x - start_x;
						auto sy = y - start_y;

						if (sx < 0 || sy < 0 || sx > w || sy > h) continue;

						if (host8[sx + (sy * pitch8)] != 0) shost8[x + y * spitch8] = host8[sx + (sy * pitch8)];
						if (host16[sx + (sy * pitch16)] != 0) shost16[x + y * spitch16] = host16[sx + (sy * pitch16)];
					}
				}

			}
			else {
				MIL_UINT16* host16;
				MIL_ID pitch16 = M_NULL;
				MbufInquire(iinfo.pHeightMap->id(), M_HOST_ADDRESS, &host16);
				MbufInquire(iinfo.pHeightMap->id(), M_PITCH, &pitch16);

				for (int x = start_x; x < end_x; x++) {
					for (int y = start_y; y < end_y; y++) {

						if (x < 0 || y < 0 || x > width_px || y > height_px) continue;

						auto sx = x - start_x;
						auto sy = y - start_y;

						if (sx < 0 || sy < 0 || sx > w || sy > h) continue;

						if (host16[sx + (sy * pitch16)] != 0) shost16[x + y * spitch16] = host16[sx + (sy * pitch16)];
					}
				}
			}
		}

		FrameInfo info;
		info.bufferSize = imageSize;
		info.width = width_px;
		info.height = height_px;
		info.pixelFormat = pixelFormat;
		info.cameraID = camID;
		info.viewID = stitchID;
		info.stitchID = sID;
		info.opticID = optID;
		info.type = ct::s_height_map;
		info.pImage = mtrx::MPM::instance().attach(simage8);
		info.pHeightMap = mtrx::MPM::instance().attach(simage16);
		infos.push_back(info);
	}

	ct::logger::info("[IM] Done stitching 3D: %s", sID.toStdString().c_str());
	emit imageReady(infos);
}

void ImageManager::guided_stitch_view(QString sID)
{
	ScopedTimeLogger stl("[IM] Guided Stitching");

	QVector<FrameInfo> infos;

	const QString basePath = Common::Directory::CurrentImageSetPath + "\\Unstitched\\";
	const QString outputRoot = Common::Directory::CurrentImageSetPath + "\\Restitched\\";
	//Common::Directory::createDir(outputRoot);

	if (!m_views->contains(sID)) return;

	auto vIDs = m_stitchMap[sID].imageStatus.keys();
	auto firstViewID = m_stitchMap[sID].imageStatus.keys()[0];

	double camSize = 5120.0;
	double halfCamsize = camSize / 2;

	QHash<QString, QVector<FrameInfo>> infosMap;
	{
		std::lock_guard<std::mutex> lock(m_infosMapMutex);
		for (auto vID : vIDs) {
			infosMap.insert(vID, m_infosMap[vID]);
			m_infosMap.remove(vID);
		}
	}

	double xmin_mm = 999999, xmax_mm = 0;
	double ymin_mm = 999999, ymax_mm = 0;

	int numRow = 0, numCol = 0;

	int opticSize = infosMap[firstViewID].size();
	int view_width_px, view_height_px;

	for (auto& vID : vIDs) {

		if (!m_views->contains(vID)) {
			ct::logger::error("[IM] Invalid view ID: %s", vID.toStdString().c_str());
			continue;
		}

		auto& v = (*m_views)[vID];

		auto datas = vID.split("-");
		auto row = datas[1].toInt();
		auto col = datas[2].toInt();

		if (row > numRow) numRow = row;
		if (col > numCol) numCol = col;

		if (xmin_mm > v.world.wx) {
			xmin_mm = v.world.wx;
		}

		if (xmax_mm < v.world.wx) {
			xmax_mm = v.world.wx;
		}

		if (ymin_mm > v.world.wy) {
			ymin_mm = v.world.wy;
		}

		if (ymax_mm < v.world.wy) {
			ymax_mm = v.world.wy;
		}

		view_width_px = v.px.w;
		view_height_px = v.px.h;
	}

	auto scale = ScaleManager::instance().um_per_px();
	int width_px = util::mm_to_px(xmax_mm - xmin_mm, scale) + view_width_px; //sview.px.w;
	int height_px = util::mm_to_px(ymax_mm - ymin_mm, scale) + view_height_px;

	int imageSize = width_px * height_px;

	numRow++;
	numCol++;

	ct::logger::trace("[IM][Guided Stitch] Row: %d, Col: %d", numRow, numCol);

	try {
		for (int optIndex = 0; optIndex < opticSize; optIndex++) {

			std::vector<vips::VImage> rowImages;
			std::vector<void*> rawBuffers;
			rawBuffers.reserve(1000);

			ICAM_pixelFormat pixelFormat;
			QString optID;
			QString camID;

			for (int row = 0; row < numRow; row++) {

				std::vector<vips::VImage> colImages;
		
				for (int col = 0; col < numCol; col++) {
					auto vID = sID + QString("-%1-%2").arg(row).arg(col);

					if (!m_views->contains(vID)) continue;

					auto& iinfo = infosMap[vID][optIndex];

					pixelFormat = iinfo.pixelFormat;
					optID = iinfo.opticID;
					camID = iinfo.cameraID;

					if (SystemData::instance()._saveUnstitchedImages) {
						auto root = SystemData::instance()._workingPath + "Unstitched/";
						Common::Directory::createDir(root);
						QString filename = root + iinfo.viewID + "_" + iinfo.opticID + ".jpg";
						ct::logger::info("Saving unstitched image: %s", filename.toStdString().c_str());

						ImageSavingThread::instance().enqueue(filename.toStdString(), iinfo.pImage);
					}

					void* data = nullptr;
					vips::VImage img = util::to_vimage(iinfo.pImage->id(), &data);
					if (data) rawBuffers.push_back(data);  
				
					colImages.push_back(img);
				}

				// Horizontally stitch each row
				vips::VImage rowStitched = colImages[0];
				for (int col = 1; col < colImages.size(); ++col) {
					auto vID = sID + QString("-%1-%2").arg(row).arg(col);
					ct::logger::info("vID: %s", vID.toStdString().c_str());

					if (!(*m_views).contains(vID)) {
						ct::logger::error("[IM] Failed to guide stitching. Invalid view ID: %s", vID.toStdString().c_str());
						continue;
					}

					auto& v = (*m_views)[vID];

					auto tx = util::mm_to_px(v.world.wx - xmin_mm, 1.67);

					auto overlapSize = (camSize * col) - tx;
					auto overlapHalfsize = overlapSize / 2.0;

					auto left_point = tx + overlapHalfsize;
					auto right_point = overlapHalfsize;

					ct::logger::debug("tx: %.2f", tx);
					ct::logger::debug("overlapSize: %.2f", overlapSize);
					ct::logger::debug("overlapHalfsize: %.2f", overlapHalfsize);
					ct::logger::debug("left_point: %.2f", left_point);
					ct::logger::debug("right_point: %.2f", right_point);

					// First stitch: img0 + img1
					rowStitched = rowStitched.mosaic(colImages[col], VIPS_DIRECTION_HORIZONTAL,
						left_point, halfCamsize, // right center of img0
						right_point, halfCamsize, // left center of img1
						vips::VImage::option()->set("hwindow", 10)->set("harea", 30)->set("mblend", 300));

					//std::string outputPath = "horizontalStitch" + std::to_string(row) + "-" + std::to_string(col) + ".jpg";
					//rowStitched.write_to_file(outputPath.c_str());

					ct::logger::info("[IM] Done horizontal stitching: %s", vID.toStdString().c_str());
				}

				rowImages.push_back(rowStitched);
			}

			// Vertically stitch all rows
			vips::VImage finalStitched = rowImages[0];
			int firstCol = 0;
			for (int row = 1; row < rowImages.size(); ++row) {
				auto vID = sID + QString("-%1-%2").arg(row).arg(firstCol);
				ct::logger::info("vID: %s", vID.toStdString().c_str());

				if (!(*m_views).contains(vID)) {
					ct::logger::error("[IM] Failed to guide stitching. Invalid view ID: %s", vID.toStdString().c_str());
					continue;
				}

				auto& v = (*m_views)[vID];

				auto ty = util::mm_to_px(v.world.wy - ymin_mm, 1.67);

				auto overlapSize = (camSize * row) - ty;
				auto overlapHalfsize = overlapSize / 2.0;

				auto top_point = ty + overlapHalfsize;
				auto btm_point = overlapHalfsize;

				auto halfWidth = rowImages[row].width() / 2;

				ct::logger::debug("ty: %.2f", ty);
				ct::logger::debug("overlapSize: %.2f", overlapSize);
				ct::logger::debug("overlapHalfsize: %.2f", overlapHalfsize);
				ct::logger::debug("top_point: %.2f", top_point);
				ct::logger::debug("btm_point: %.2f", btm_point);

				// First stitch: img0 + img1
				finalStitched = finalStitched.mosaic(rowImages[row], VIPS_DIRECTION_VERTICAL,
					halfWidth, top_point, // top center of img0
					halfWidth, btm_point, // btm center of img1
					vips::VImage::option()->set("hwindow", 10)->set("harea", 50)->set("mblend", 300));

				ct::logger::info("[IM] Done vertical stitching: %s", vID.toStdString().c_str());
			}

			FrameInfo info;
			info.bufferSize = imageSize;
			info.width = width_px;
			info.height = height_px;
			info.pixelFormat = pixelFormat;
			info.cameraID = camID;
			info.viewID = sID;
			info.stitchID = "";
			info.opticID = optID;
			info.type = ct::s_color;
			auto cid = util::combineID(sID, optID);

			auto mBuf = util::to_milID(finalStitched);

			auto band = mtrx::get_band(mBuf);
			auto type = mtrx::get_type(mBuf);
			auto attribute = mtrx::get_attribute(mBuf);

			info.pImage = mtrx::MPM::instance().acquire(info.width, info.height, band, type);

			mtrx::paste_to_center(mBuf, info.pImage->id());

			infos.push_back(info);

			mtrx::free_buffer(mBuf);
			for (void* ptr : rawBuffers) free(ptr);
		}
	}
	catch (const vips::VError& e) {
		std::cerr << "VIPS error: " << e.what() << std::endl;
		ct::logger::error("[IM] Failed to perform 2D stitching: %s", sID.toStdString().c_str());
		return;
	}

	ct::logger::info("[IM] Done stitching 2D: %s", sID.toStdString().c_str());
	emit imageReady(infos);
}

void ImageManager::guided_stitch_linescan(QString sID)
{
}

void ImageManager::process_stitch_thread()
{
	while (!m_release) {
		QString id;

		{
			std::unique_lock<std::mutex> lock(m_stitchDequeMutex);
			m_stitchCV.wait(lock, [this]() { return m_release || !m_stitchDeque.empty(); });
			if (m_release) return;

			id = m_stitchDeque.front();
			m_stitchDeque.pop_front();
		}

		if (m_stitchingMethod == StitchingMethod::MECHANICAL) {
			mechanical_stitching(id);
		}
		else {
			on_the_fly_stitching(id);
		}
	}
}
