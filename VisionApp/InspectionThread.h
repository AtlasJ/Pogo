#pragma once
#include <QThread>
#include <QImage>
#include <QMutex>
#include <QWaitCondition>
#include <queue>
#include <atomic>
#include "FrameInfo.h"

/*
* Production inspection worker.
*
* Receives images from the acquisition flow and routes each to the matching
* algorithm using the FrameInfo linkage (type / viewID / opticID):
*  - "srx_ocr" frames (the SR-X reader capture from the non-barcode side,
*    carried as a QImage payload) -> AlgoManager OCR
*  - height map frames (ct::s_height_map with pHeightMap) -> AlgoManager 3D height
*
* Only active during production (setActive(true) at production start). When
* inactive, enqueued frames are dropped - normal acquisition just saves images.
*/
class InspectionThread : public QThread {
	Q_OBJECT

public:
	static InspectionThread& instance();

	void setActive(bool on);
	bool isActive() const { return m_active; }

	//routing uses the FrameInfo fields; the SR-X OCR capture rides along as a
	//QImage payload since it does not come from a MIL acquisition buffer
	void enqueue(const FrameInfo& info, const QImage& ocrImage = QImage());

	void release();

signals:
	void inspectionResult(QString algo, bool pass, QString detail);

protected:
	void run() override;

private:
	explicit InspectionThread(QObject* parent = nullptr);
	~InspectionThread();
	Q_DISABLE_COPY(InspectionThread)

	struct Item {
		FrameInfo info;
		QImage ocrImage;
	};

	void process(const Item& item);

	std::queue<Item> m_queue;
	QMutex m_mutex;
	QWaitCondition m_cv;
	std::atomic<bool> m_active = false;
	std::atomic<bool> m_running = true;
};
