#pragma once
#include <QString>
#include "FrameInfo.h"

class IProfiler {
public:

	enum ExposureMode{
		SINGLE, MULTI, DYNAMIC, PARALLEL
	};

	IProfiler() {};
	~IProfiler() {};

	//Query
	virtual const double getExposure() const = 0;
	virtual const double getYResolution() const = 0;
	virtual const QString& getFirmwareVersion() const = 0;
	virtual const QString& getSerialNumber() const = 0;

	virtual bool isConnected() const = 0;
	virtual const bool isGrabbing() const = 0;

	//Connection
	virtual bool enable(bool enable) = 0;
	virtual bool connect(QString sn) = 0;
	virtual bool disconnect() = 0;


	//Acquisition
	virtual bool start() = 0;
	virtual bool stop() = 0;
	virtual bool snapShot() = 0;


	//Control
	virtual bool enableIntensityMap(bool enable) = 0;
	virtual bool setScanLength(double mm) = 0;
	virtual bool setGain(double gain) = 0;
	virtual bool setDuoHeadGain(double gain, double gain2) = 0;
	virtual bool setDivider(int gain) = 0;
	virtual bool setExposureMode(ExposureMode mode) = 0;
	virtual bool setExposure(double us) = 0;
	virtual bool setMultiExposure(double us,double us2) = 0;
	virtual bool setDynamicExposure(double min_us, double max_us) = 0;
	virtual bool setParallelExposure(double min_us, double max_us) = 0;
	virtual bool waitAcquisition(int ms) = 0;
	virtual bool setMSR(bool enable) = 0;
	virtual bool setLaserLineThreshold(double threshold) = 0;

	//Data
	virtual const FrameInfo& getFrame() const = 0;
	virtual FrameInfo& getFrame() = 0;
	virtual void resetFrame() = 0;

	//Misc
	virtual bool loadConfig(QString path) = 0;
	virtual QString errorMsg() = 0;
};