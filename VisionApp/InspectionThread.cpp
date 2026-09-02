#include "InspectionThread.h"
#include "AlgoManager.h"
#include <QElapsedTimer>
#include "Logger.h"
#include "OpticsInfo.h"

InspectionThread& InspectionThread::instance()
{
	static InspectionThread inst;
	return inst;
}

InspectionThread::InspectionThread(QObject* parent)
	: QThread(parent)
{
	//result capture: process() dispatches one algo at a time and waits here,
	//so each result can be attributed to the unit that produced the image
	connect(&AlgoManager::instance(), &AlgoManager::ocrFinished, this, [this](AlgoOcrOutput output) {
		QMutexLocker lock(&m_resMutex);
		m_ocrOutput = output;
		m_ocrDone = true;
		m_resCv.wakeAll();
	}, Qt::DirectConnection);

	connect(&AlgoManager::instance(), &AlgoManager::heightFinished, this, [this](AlgoHeightOutput output) {
		QMutexLocker lock(&m_resMutex);
		m_heightOutput = output;
		m_heightDone = true;
		m_resCv.wakeAll();
	}, Qt::DirectConnection);
}

InspectionThread::~InspectionThread()
{
}

void InspectionThread::setActive(bool on)
{
	if (m_active == on) return;
	m_active = on;
	ct::logger::info("[Inspection] Production inspection %s", on ? "ACTIVATED" : "deactivated");

	if (!on) {
		//drop anything still queued from the stopped run
		QMutexLocker lock(&m_mutex);
		std::queue<Item> empty;
		std::swap(m_queue, empty);
	}
}

void InspectionThread::enqueue(const FrameInfo& info, const QImage& ocrImage)
{
	if (!m_active) return; //normal acquisition: images are only saved, not inspected

	{
		QMutexLocker lock(&m_mutex);
		m_queue.push({ info, ocrImage });
	}
	m_cv.wakeAll();
}

bool InspectionThread::isIdle()
{
	if (m_processing) return false;
	QMutexLocker lock(&m_mutex);
	return m_queue.empty();
}

void InspectionThread::release()
{
	m_running = false;
	m_active = false;
	m_cv.wakeAll();
	{
		QMutexLocker lock(&m_resMutex);
		m_resCv.wakeAll();
	}
	wait(3000);
}

void InspectionThread::run()
{
	ct::logger::info("[QThread] Inspection thread started");

	while (m_running) {
		Item item;
		{
			QMutexLocker lock(&m_mutex);
			while (m_queue.empty() && m_running) m_cv.wait(&m_mutex);
			if (!m_running) break;
			item = m_queue.front();
			m_queue.pop();
		}

		if (!m_active) continue;
		m_processing = true;
		process(item);
		m_processing = false;
	}

	ct::logger::info("[QThread] Inspection thread stopped");
}

bool InspectionThread::waitResult(bool& doneFlag, int timeoutMs)
{
	QMutexLocker lock(&m_resMutex);
	QElapsedTimer timer;
	timer.start();
	while (!doneFlag && m_running && timer.elapsed() < timeoutMs) {
		m_resCv.wait(&m_resMutex, 200);
	}
	return doneFlag;
}

void InspectionThread::process(const Item& item)
{
	const auto& info = item.info;
	const QString unitID = info.viewID.isEmpty() ? QStringLiteral("board") : info.viewID;
	constexpr int algoTimeoutMs = 60000;

	//route by the frame linkage, then wait for the result so it can be
	//attributed to this unit
	if (info.type == "srx_ocr" && !item.ocrImage.isNull()) {
		ct::logger::info("[Inspection] OCR image received (unit=%s, optic=%s, %dx%d) - running OCR",
			unitID.toStdString().c_str(), info.opticID.toStdString().c_str(),
			item.ocrImage.width(), item.ocrImage.height());

		{
			QMutexLocker lock(&m_resMutex);
			m_ocrDone = false;
		}
		AlgoManager::instance().runOcr(item.ocrImage);

		if (!waitResult(m_ocrDone, algoTimeoutMs)) {
			ct::logger::error("[Inspection] OCR timed out (unit=%s)", unitID.toStdString().c_str());
			emit inspectionResult(unitID, "OCR", false, "timeout");
			return;
		}

		AlgoOcrOutput output;
		{
			QMutexLocker lock(&m_resMutex);
			output = m_ocrOutput;
		}
		QString text = output.roi1Text;
		ct::logger::info("[Inspection] OCR result (unit=%s): ok=%d, text='%s' %s (%lldms)",
			unitID.toStdString().c_str(), output.ok ? 1 : 0, text.toStdString().c_str(),
			output.message.toStdString().c_str(), (long long)output.elapsedMs);
		emit inspectionResult(unitID, "OCR", output.ok, text.isEmpty() ? output.message : text);
	}
	else if (info.type == ct::s_height_map && info.pHeightMap) {
		ct::logger::info("[Inspection] Height map received (unit=%s, optic=%s) - running 3D height",
			unitID.toStdString().c_str(), info.opticID.toStdString().c_str());

		{
			QMutexLocker lock(&m_resMutex);
			m_heightDone = false;
		}
		AlgoManager::instance().setHeightMap(info.pHeightMap);
		AlgoManager::instance().runHeight();

		if (!waitResult(m_heightDone, algoTimeoutMs)) {
			ct::logger::error("[Inspection] 3D height timed out (unit=%s)", unitID.toStdString().c_str());
			emit inspectionResult(unitID, "3D Height", false, "timeout");
			return;
		}

		AlgoHeightOutput output;
		{
			QMutexLocker lock(&m_resMutex);
			output = m_heightOutput;
		}
		QStringList heights;
		for (const auto& r : output.roiResults) heights << QString::number(r.avgHeightUm, 'f', 1);
		ct::logger::info("[Inspection] 3D height result (unit=%s): pass=%d, heights(um)=[%s] (%lldms)",
			unitID.toStdString().c_str(), output.pass ? 1 : 0,
			heights.join(", ").toStdString().c_str(), (long long)output.elapsedMs);
		emit inspectionResult(unitID, "3D Height", output.pass, heights.join(", "));
	}
	else {
		ct::logger::warn("[Inspection] Unroutable frame ignored (type=%s, unit=%s, optic=%s)",
			info.type.toStdString().c_str(), unitID.toStdString().c_str(),
			info.opticID.toStdString().c_str());
	}
}
