#pragma once
#include <QThread>
#include <QImage>
#include <QHash>
#include <QMutex>
#include <QWaitCondition>
#include <queue>
#include <atomic>
#include "FrameInfo.h"
#include "AlgoSetupTypes.h"
#include "AlgoHeight3Types.h" //AlgoHeight3Output - V3 runs the production height path

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
	bool isIdle(); //queue empty and not mid-algo

	//routing uses the FrameInfo fields; the SR-X OCR capture rides along as a
	//QImage payload since it does not come from a MIL acquisition buffer
	void enqueue(const FrameInfo& info, const QImage& ocrImage = QImage());

	/*
	* Each unit's cable name from the PMF (unit ID X#Y# -> cable), set from the GUI before a run
	* is queued. A unit's 3D height runs the V3 algo named after its cable, or the default one.
	* Empty - every unit runs the default - when the run has no usable Live Mapping.
	*/
	void setUnitCables(const QHash<QString, QString>& cables);
	QString unitCable(const QString& unitID) const;

	//report an algo that will not run for a unit (e.g. OCR with no image), so
	//the status table and progress accounting still complete
	void reportSkipped(const QString& unitID, const QString& algo, const QString& reason) {
		emit inspectionResult(unitID, algo, false, reason);
	}

	void release();

signals:
	void inspectionResult(QString unitID, QString algo, bool pass, QString detail);

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
	bool waitResult(bool& doneFlag, int timeoutMs);

	std::queue<Item> m_queue;
	QMutex m_mutex;
	QWaitCondition m_cv;
	std::atomic<bool> m_active = false;
	std::atomic<bool> m_running = true;
	std::atomic<bool> m_processing = false;

	//result handoff from AlgoManager's worker (see ctor connects)
	QMutex m_resMutex;
	QWaitCondition m_resCv;
	bool m_ocrDone = false;
	bool m_height3Done = false;
	QString m_height3Algo;   //the algo this unit's run was asked for; only its result is taken
	AlgoOcrOutput m_ocrOutput;
	AlgoHeight3Output m_height3Output;

	mutable QMutex m_cableMutex;
	QHash<QString, QString> m_unitCables;
};
