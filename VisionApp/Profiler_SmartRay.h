#pragma once
#include "IProfiler.h"
#include <QString>
#include "FrameInfo.h"
#include <QImage>
#include <functional>
#include <mutex>
#include <iostream>
#include <vector>
#include <sstream>
#include "mtrx.h"
#include "SmartRay\include\sr_api\sr_api.h"
#include "SmartRay\cpp_wrapper\SensorManager.h"

class Profiler_SmartRay : public IProfiler {
public:

	Profiler_SmartRay();
	~Profiler_SmartRay();



	//Query
	const double getExposure() const; //Only gets master sensor and larger exposure
	const double getYResolution() const;
	const QString& getFirmwareVersion() const; //Only gets master sensor
	const QString& getSerialNumber() const; //Only gets master sensor
	bool isConnected() const;
	const bool isGrabbing() const;
	


	//Connection
	bool enable(bool enable);
	bool connect(QString ip);
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





private:


	SensorManager sensorManager;

	SharedPtr<Sensor> master;
	SharedPtr<Sensor> slave;
	bool status;
	bool m_connectionStatus;
	double YResolution = 0.0063;
	double ZResolution = 0.00041;
	ExposureMode m_mode = SINGLE;
	double* m_exposure;
	char* m_firmware = new char[256];
	char* m_firmware2 = new char[256];
	char* m_serial = new char[256];
	bool m_enable = true;
	static double numProfilesToCapture;
	uint16_t    port = DEFAULT_PORT_NUM;
	const uint32_t timeoutSeconds = 1;
	bool m_softTriggered = false;
	std::mutex m_mutex;
	std::condition_variable m_conditionVariable;
	bool m_enableImap = false;

	int m_divider = 1;

	FrameInfo m_frameInfo;

	QString m_errorMsg = "";

	bool safeGuard() const;


	QString incrementIPAddress(const QString& ip);

	bool setDefaultAcquisitionMode();
	bool setLaserLineThreshold();
	bool loadCalibration();
	bool sendConfig();
	bool sendParam();
	bool setDataTriggerMode(DataTriggerMode aMode);
	bool m_setMSRmode=false;
	bool snapshotFlag = false;
	bool startSnapshot();
	bool waitAcquisitionSnapshot(int ms);
	bool setDataTriggerParam(int div, int delay,TriggerEdgeMode aParam);
	bool setDataTriggerSource(DataTriggerSource aSource);
	bool getDataTrigger();
	bool getTransportResolution();
	bool getZmapResolution();
};