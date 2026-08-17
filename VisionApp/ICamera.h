#pragma once
#include <QString>
#include "FrameInfo.h"

class ICamera {
public:
	ICamera() {};
	~ICamera() {};

	//Query
	virtual const int getWidth() const = 0;
	virtual const int getHeight() const = 0;
	virtual const int getChannel() const = 0;

	virtual const double getExposure() const = 0;
	virtual const double getGain() const = 0;

	virtual const QString& getName() const = 0;
	virtual const QString& getSerialNumber() const = 0;

	virtual bool isConnected() const = 0;
	virtual const bool isGrabbing() const = 0;


	//Connection
	virtual bool enable(bool enable) = 0;
	virtual bool connect(QString sn) = 0;
	virtual bool disconnect() = 0;


	//Acquisition
	virtual bool startGrab() = 0;
	virtual bool stopGrab() = 0;


	//Control
	virtual bool setExposure(double exposure) = 0;
	virtual bool setGain(double gain) = 0;
	virtual bool softTrigger() = 0;
	virtual bool waitAcquisition(int ms) = 0;
	virtual bool setTriggerOutput(QString line, QString source) = 0;

	virtual bool setDO(int DO, bool on) = 0;

	//Data
	virtual const FrameInfo& frame() const = 0;
	virtual FrameInfo& frame() = 0;
	virtual void resetFrame() = 0;

	//Misc
	virtual bool loadConfig(QString path) = 0;
	virtual QString errorMsg() = 0;
};