#include "InspectionThread.h"
#include "AlgoManager.h"
#include <QElapsedTimer>
#include "Logger.h"
#include "OpticsInfo.h"
#include "SystemData.h" //_heightMapNativeScale, the V3 scale precondition

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

	//V3 is what the production height path runs; the V1 connection above is kept because
	//the Algo Setup page still drives that algorithm offline
	connect(&AlgoManager::instance(), &AlgoManager::height3Finished, this,
		[this](int stage, AlgoHeight3Output output) {
			//production only ever asks for a Run All, so a result for any other stage is the
			//Algo Setup page working and must not be mistaken for this unit's verdict
			if (stage != (int)AlgoH3Stage::All) return;
			QMutexLocker lock(&m_resMutex);
			m_height3Output = output;
			m_height3Done = true;
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

		/*
		* NATIVE SCALE IS A PRECONDITION, NOT A PREFERENCE.
		*
		* V3's scaling is taught in um/px (5.0 on both axes by default), and rotate_heightMap
		* only produces a 5.0 um/px map while heightMapNativeScale is ON - with it off the map
		* comes out at the world scale, roughly 27.5 um/px, so every height would be wrong by
		* about 5.5x AND STILL LOOK ENTIRELY PLAUSIBLE. The flag lives in the recipe and
		* DEFAULTS TO FALSE (SystemData.h), so a new or copied recipe starts out wrong.
		*
		* Fail the unit rather than report a confidently wrong number: a mis-scaled PASS is
		* discovered by the customer, a stopped line is discovered immediately. Same shape as
		* isSafeToScan() refusing on the wrong trigger mode.
		*/
		if (!SystemData::instance()._heightMapNativeScale) {
			ct::logger::error("[Inspection] 3D height REFUSED (unit=%s): heightMapNativeScale is off, "
				"so this height map is at the world scale and every height would mis-scale by ~5.5x. "
				"Enable native 3D scale in the recipe.", unitID.toStdString().c_str());
			emit inspectionResult(unitID, "3D Height", false,
				"native 3D scale is off - enable it in the recipe");
			return;
		}

		//straight from this frame's buffers. The intensity map is optional: the pipeline never
		//measures from it, it only makes the map recognisable to an operator reviewing the unit
		QString note;
		if (!AlgoManager::instance().height3SetSourceMaps(info.pHeightMap, info.pImage, note)) {
			ct::logger::error("[Inspection] 3D height failed (unit=%s): %s",
				unitID.toStdString().c_str(), note.toStdString().c_str());
			emit inspectionResult(unitID, "3D Height", false, note);
			return;
		}
		if (!note.isEmpty()) {
			ct::logger::warn("[Inspection] 3D height (unit=%s): %s",
				unitID.toStdString().c_str(), note.toStdString().c_str());
		}

		{
			QMutexLocker lock(&m_resMutex);
			m_height3Done = false;
		}
		//one Run All per unit: every stage re-runs, so the part is segmented fresh and the
		//taught ROIs land on THIS unit rather than on where the last one happened to sit
		AlgoManager::instance().runHeight3(AlgoH3Stage::All);

		if (!waitResult(m_height3Done, algoTimeoutMs)) {
			ct::logger::error("[Inspection] 3D height timed out (unit=%s)", unitID.toStdString().c_str());
			emit inspectionResult(unitID, "3D Height", false, "timeout");
			return;
		}

		AlgoHeight3Output output;
		{
			QMutexLocker lock(&m_resMutex);
			output = m_height3Output;
		}

		const QString summary = algoH3RunSummary(output);
		ct::logger::info("[Inspection] 3D height result (unit=%s): pass=%d, %s "
			"(segment %.2f x %.2f mm at %.3f deg, %lldms)",
			unitID.toStdString().c_str(), output.overallPass ? 1 : 0,
			summary.toStdString().c_str(),
			output.segWidthUm / 1000.0, output.segHeightUm / 1000.0, output.segAngleDeg,
			(long long)(output.totalElapsedMs > 0 ? output.totalElapsedMs : output.measure.elapsedMs));

		//every pin's height at debug level: too long for the inspection log, and the only
		//record of what was actually measured when a unit is disputed later
		if (!output.roiResults.isEmpty()) {
			QStringList heights;
			for (const auto& r : output.roiResults) {
				heights << QStringLiteral("%1%2").arg(r.valid ? QString::number(r.heightUm, 'f', 1)
					: QStringLiteral("nodata"), r.pass ? QString() : QStringLiteral("*"));
			}
			ct::logger::debug("[Inspection] 3D height pins (unit=%s, * = fail): [%s]",
				unitID.toStdString().c_str(), heights.join(", ").toStdString().c_str());
		}

		emit inspectionResult(unitID, "3D Height", output.overallPass, summary);
	}
	else {
		ct::logger::warn("[Inspection] Unroutable frame ignored (type=%s, unit=%s, optic=%s)",
			info.type.toStdString().c_str(), unitID.toStdString().c_str(),
			info.opticID.toStdString().c_str());
	}
}
