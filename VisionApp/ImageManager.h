#pragma once
#include <QCoreApplication>
#include <QThread>
#include <QVector>
#include <QMutex>
#include <QWaitCondition>
#include <QSet>
#include <QHash>
#include <QCoreApplication>
#include "FrameInfo.h"
#include <opencv2/opencv.hpp>
#include <queue>
#include <QObject>
#include "QView.h"
#include "QLineScan.h"
#include "OpticsInfo.h"
#include <QDebug>
#include "Fiducial.h"
#include "WinEvents.h"

struct StitchData {
	std::vector<cv::Mat> images;
	QHash<QString, bool> imageStatus;
};

enum class StitchingMethod {
	MECHANICAL, ON_THE_FLY, GUIDED
};

Q_DECLARE_METATYPE(FrameInfo)
Q_DECLARE_METATYPE(QVector<FrameInfo>)
class ImageManager : public QThread {
	Q_OBJECT

public:
	
	ImageManager(QObject* parent = nullptr) : QThread(parent) {
		m_appEvents.createEvent("SnapDone");
	}
	~ImageManager() {}

	void run() override;
	void release();
	void reset();
		
	void attach(Fiducial* fiducialAlgo); //remove
	void attach(QHash <QString, QView>* views, QHash<QString, OpticsInfo>* optics2D);
	void attach(QHash <QString, QLineScan>* linescans, QHash<QString, OpticsInfo3D>* optics3D);

	void printOptic3D();
public slots:
	void queueStackImage(QString id);

private:
	std::atomic<bool> m_release = false;

	QMutex m_mutex;
	QWaitCondition m_condition;
	WinEvents m_appEvents;

	std::mutex m_infosMapMutex;
	QHash<QString, QVector<FrameInfo>> m_infosMap;

	std::thread m_stitchThread;
	std::mutex m_stitchDequeMutex;
	std::mutex m_stitchMapMutex;
	std::condition_variable m_stitchCV;
	std::deque<QString> m_stitchDeque;
	QHash <QString, StitchData> m_stitchMap;
	QHash<QString, int> m_zCount;

	Fiducial* m_fiducialAlgo = nullptr;

	std::thread m_zstackThread;
	std::mutex m_zstackDequeMutex;
	std::mutex m_zstackMapMutex;
	std::condition_variable m_zstackCV;
	std::deque<QString> m_zstackDeque;
	QHash<QString, QVector<FrameInfo>> m_zstackMap;

	QHash <QString, QView>* m_views = nullptr;
	QHash <QString, QLineScan>* m_linescans = nullptr;

	QHash <QString, OpticsInfo>* m_optics = nullptr;
	QHash <QString, OpticsInfo3D>* m_optics3D = nullptr;

	QHash <QString, QVector<FrameInfo>> m_combineMap;

	StitchingMethod m_stitchingMethod = StitchingMethod::MECHANICAL;

	void resume();
	void wait();

	QString m_idType = "None";

	QString getInfoID(const FrameInfo& info);
	bool all_optics_acquired(const FrameInfo& info);

	void preprocess_info(FrameInfo& info);
	void process_sizing(FrameInfo& info);
	void process_combination(FrameInfo& info);
	void process_rgbOffset(FrameInfo& info);
	void process_final_image(const FrameInfo& info);
	void preprocess_done(FrameInfo& info);

	void rotate_image(FrameInfo& info);
	void rotate_image(MIL_ID mSrc, MIL_ID mDst, double rotateAngle);
	void rotate_image_discrete(FrameInfo& info); //camera alignment 90/180/270 option
	void rotate_heightMap(MIL_ID mSrc, MIL_ID& mDst, double rotateAngle);

	bool readyToStitch(const QHash<QString, bool>& status);

	void on_the_fly_stitching(QString);
	void mechanical_stitching(QString);
	void homography_stitching(QString);

	void mechanical_stitch_view(QString sID);
	void mechanical_blend_view(QString sID);
	void mechanical_stitch_linescan(QString sID);

	void guided_stitch_view(QString sID);
	void guided_stitch_linescan(QString sID);

	void process_stitch_thread();

	void update_stitchData();
	void update_opticsData();
	void update_zCount();

	void process_stack_queues_thread();
	void stackImage(QString id);

signals:
	void imageReceived(FrameInfo infos);
	void imagePreprocessed(FrameInfo infos);
	void imageReady(QVector<FrameInfo> infos);
};
