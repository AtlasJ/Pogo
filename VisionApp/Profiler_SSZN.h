#pragma once
#include "IProfiler.h"
#include <QString>
#include "FrameInfo.h"
#include <QImage>
#include <functional>
#include <mutex>
#include "mtrx.h"
#include "SSZN\include\SR7Link.h"



#define DEVICEID 0
#define MAXHEIGHT 15000

struct BatchStore {
	// geometry/meta
	int   batchPoints = 0;   // rows
	int   xPoints = 0;   // cols
	float xPitch = 0.0f;

	// data per head (0..1)
	std::array<std::vector<int>, 2>         profile;   // size = batchPoints * xPoints
	std::array<std::vector<uint8_t>, 2>     intensity; // size = batchPoints * xPoints
	std::array<std::vector<uint32_t>, 2>    encoder;   // size = batchPoints

	bool finished = false;
};



class Profiler_SSZN : public IProfiler {
public:
	Profiler_SSZN();
	~Profiler_SSZN();

	//Query
	const double getExposure() const;
	const double getYResolution()const;
	const QString& getFirmwareVersion() const;
	const QString& getSerialNumber() const;
	bool isConnected() const;
	const bool isGrabbing() const;
	const int getImageWidth() const;
	const int getImageHeight() const;

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
	bool setTestFlag(bool flag);

	//Data
	const FrameInfo& getFrame() const;
	FrameInfo& getFrame();
	void resetFrame();


	//Misc
	bool loadConfig(QString path);
	QString errorMsg();

	static void BatchOneTimeCallBack(const void* info, const SR7IF_Data* data);
private:
	double m_exposure = 0.0;
	bool m_enable = true;
	FrameInfo m_frameInfo;
	bool m_softTriggered = false;
	std::mutex m_mutex;
	std::condition_variable m_conditionVariable;
	QString m_errorMsg = "";
	QString m_serialNumber = "";
	int m_divider = 1; 
	std::atomic<bool> m_release = false;

	bool safeGuard() const;
	int getNearestExposureIndex(int us);
	
	bool testFlag =false ;

	BatchStore _last;
	double YResolution ;
	bool m_connectionStatus;

	static Profiler_SSZN* s_instance;

	int loadOptics3DDivider(int defaultDivider = 1);
	bool loadOptics3DLaserLimits(int& lowerLimit, int& upperLimit, int defaultLower = 60, int defaultUpper = 60);
	bool loadOptics3DLightPeak(int& lightSensitivityIdx,int& peakSensitivityVal,int& peakSelectionIdx,int defLightIdx = 0,int defPeakSens = 1,int defPeakSel = 0);


	bool snapshotFlag;
	bool startSnapshot();
	bool waitAcquisitionSnapshot(int ms);

};

