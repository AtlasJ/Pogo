#include "InspectionThread.h"
#include "AlgoManager.h"
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
	//log algorithm results while production inspection is active; the Algo
	//Setup page has its own connections for interactive runs
	connect(&AlgoManager::instance(), &AlgoManager::ocrFinished, this, [this](AlgoOcrOutput output) {
		if (!m_active) return;
		QString text = output.roi1Text;
		if (!output.roi2Text.isEmpty()) text += " | " + output.roi2Text;
		ct::logger::info("[Inspection] OCR result: ok=%d, text='%s' (%lldms)",
			output.ok ? 1 : 0, text.toStdString().c_str(), (long long)output.elapsedMs);
		emit inspectionResult("OCR", output.ok, text);
	}, Qt::DirectConnection);

	connect(&AlgoManager::instance(), &AlgoManager::heightFinished, this, [this](AlgoHeightOutput output) {
		if (!m_active) return;
		QStringList heights;
		for (const auto& r : output.roiResults) heights << QString::number(r.avgHeightUm, 'f', 1);
		ct::logger::info("[Inspection] 3D height result: pass=%d, heights(um)=[%s] (%lldms)",
			output.pass ? 1 : 0, heights.join(", ").toStdString().c_str(), (long long)output.elapsedMs);
		emit inspectionResult("3D Height", output.pass, heights.join(", "));
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

void InspectionThread::release()
{
	m_running = false;
	m_active = false;
	m_cv.wakeAll();
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
		process(item);
	}

	ct::logger::info("[QThread] Inspection thread stopped");
}

void InspectionThread::process(const Item& item)
{
	const auto& info = item.info;

	//route by the frame linkage
	if (info.type == "srx_ocr" && !item.ocrImage.isNull()) {
		ct::logger::info("[Inspection] OCR image received (view=%s, optic=%s, %dx%d) - running OCR",
			info.viewID.toStdString().c_str(), info.opticID.toStdString().c_str(),
			item.ocrImage.width(), item.ocrImage.height());
		AlgoManager::instance().runOcr(item.ocrImage);
	}
	else if (info.type == ct::s_height_map && info.pHeightMap) {
		ct::logger::info("[Inspection] Height map received (view=%s, optic=%s) - running 3D height",
			info.viewID.toStdString().c_str(), info.opticID.toStdString().c_str());
		AlgoManager::instance().setHeightMap(info.pHeightMap);
		AlgoManager::instance().runHeight();
	}
	else {
		ct::logger::warn("[Inspection] Unroutable frame ignored (type=%s, view=%s, optic=%s)",
			info.type.toStdString().c_str(), info.viewID.toStdString().c_str(),
			info.opticID.toStdString().c_str());
	}
}
