#pragma once
#include "IProfiler.h"
#include <QString>
#include "FrameInfo.h"
#include <QImage>
#include <functional>
#include <mutex>
#include "mtrx.h"
#include "Gocator\Include\GoSdk\GoSdk.h"

namespace ct {

	struct GoInfo {
		QString id = "";
		GoMode mode = GO_MODE_SURFACE;
		double resolution_x_mm = 0.0;
		double resolution_y_mm = 0.0;
		double resolution_z_mm = 0.0;
		QImage qHeightMap;
		QImage qIntensity = QImage();
		MIL_ID mHeightMap = M_NULL;
		MIL_ID mIntensity = M_NULL;
		std::vector<double> profiles;
		std::function<void()> fnc;

		void reset() {
			id = "";
			resolution_x_mm = 0.0;
			resolution_y_mm = 0.0;
			resolution_z_mm = 0.0;
			qHeightMap = QImage();
			qIntensity = QImage();
			mtrx::free_buffer(mHeightMap);
			mtrx::free_buffer(mIntensity);
			profiles.clear();
		}
	};
}

class Profiler_Gocator : public IProfiler {
public:
	Profiler_Gocator();
	~Profiler_Gocator();

	//Query
	const double getExposure() const;
	const double getYResolution()const;
	const QString& getFirmwareVersion() const;
	const QString& getSerialNumber() const;
	bool isConnected() const;
	const bool isGrabbing() const;


	//Connection
	bool enable(bool enable);
	bool connect(QString info);
	bool disconnect();


	//Acquisition
	bool start();
	bool stop();
	bool snapShot();


	//Control
	bool enableIntensityMap(bool enable);
	bool setScanLength(double mm);
	bool setGain(double gain);
	bool setDuoHeadGain(double gain, double gain2);
	bool setExposureMode(ExposureMode mode);
	bool setExposure(double us);
	bool setMultiExposure(double us, double us2);
	bool setDynamicExposure(double min_us, double max_us);
	bool setParallelExposure(double us, double us2);
	bool waitAcquisition(int ms);
	bool setMSR(bool enable);
	bool setLaserLineThreshold(double threshold);
	bool setDivider(int divider);


	//Data
	const FrameInfo& getFrame() const;
	FrameInfo& getFrame();
	void resetFrame();


	//Misc
	bool loadConfig(QString path);
	QString errorMsg();

	static kStatus kCall Callback(kPointer context, GoSensor m_sensor, GoDataSet dataset);

private:

	kStatus m_status;
	kAssembly m_api = kNULL;
	GoSystem m_system = kNULL;
	GoSensor m_sensor = kNULL;
	GoSetup m_setup = kNULL;
	GoLayout m_layout = kNULL;
	GoAccelerator m_accelerator = kNULL;
	kIpAddress m_ipAddress;
	double m_exposure = 0.0;
	bool m_enable=true;

	ct::GoInfo m_goInfo;
	FrameInfo m_frameInfo;

	bool m_softTriggered = false;
	std::mutex m_mutex;
	std::condition_variable m_conditionVariable;

	kChar m_ip_string[FILENAME_MAX];
	kChar m_filename[FILENAME_MAX];
	kChar m_path[FILENAME_MAX];
	kChar m_serialNumber[FILENAME_MAX];

	QString m_errorMsg = "";

	QString getStatus(kStatus status);
	bool setMode(GoMode mode);
	bool safeGuard() const;
	bool verifyConnection();
	
	bool reconnect();
	bool enableDataChannel(bool enable);
	QString loadedJob();
};