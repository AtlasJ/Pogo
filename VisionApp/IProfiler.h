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

	/*
	* Optional bring-up diagnostics. These are NOT pure: a backend that has nothing to add
	* contributes nothing to the report and needs no code. Only Profiler_Keyence implements
	* them today, for the Profiler Scan Test on the Test Run page.
	*/

	//Multi-line human-readable dump of whatever the backend knows about itself and what it
	//last pushed to the controller. Free-form; the caller only prints it.
	virtual QString diagnostics() const { return QString(); }

	//The controller's own trigger and encoder counters, if it has any. Returns false when
	//unsupported or unavailable, in which case the outputs are untouched.
	virtual bool getCounters(quint32& triggerCount, qint32& encoderCount) {
		Q_UNUSED(triggerCount); Q_UNUSED(encoderCount);
		return false;
	}

	//Raw dimensions of the last batch the driver received, BEFORE any resize or rotate. The
	//processed height map is stretched by the pitch constants and the divider, so its size is
	//not the profile count and must never be used as one. Returns false when unsupported.
	virtual bool getLastBatchSize(int& profiles, int& pointsPerProfile) const {
		Q_UNUSED(profiles); Q_UNUSED(pointsPerProfile);
		return false;
	}

	//Hand over the last raw batch exactly as the controller delivered it - points by
	//profiles, before ImageManager resizes by the pitch constants and rotates 90 degrees -
	//and clear the backend's own references. The caller then holds the only ones, so the
	//pool memory is freed as soon as it drops them. False when unsupported or nothing is held.
	virtual bool takeLastRawFrame(mtrx::SharedMilID& height, mtrx::SharedMilID& intensity) {
		Q_UNUSED(height); Q_UNUSED(intensity);
		return false;
	}

	//Would a scan right now produce trustworthy data? A backend says no when a setting is
	//wrong in a way that still lets the scan COMPLETE and look plausible - the dangerous
	//kind. 'reason' is filled in for the operator when this returns false.
	virtual bool isSafeToScan(QString& reason) const { Q_UNUSED(reason); return true; }
};