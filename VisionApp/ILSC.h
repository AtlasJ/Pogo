#pragma once
#include <string>
#include <vector>

enum class LSC_RC {
	PASS, FAIL, INVALID_CONNECTION, INVALID_CHANNEL,
	FAILED_TO_DISCONNECT
};

namespace lsc {
	enum class MODE {
		CONTINUOUS, TRIGGER
	};

	struct IntensityData {
		int channelIndex = 0;
		int intensity = 0;
	};

	struct SequenceData {
		int triggerSource = 1;
		int exposure_us = 5000;
		std::vector<IntensityData> intensityDatas;
	};
}

class ILSC {
public:
	ILSC();
	~ILSC();

	virtual const std::string& id() const = 0;
	virtual std::string& id() = 0;

	virtual const std::string& name() const = 0;
	virtual std::string& name() = 0;

	virtual int enable(bool enable) = 0;
	virtual int connect() = 0;
	virtual int disconnect() = 0;
	virtual int reconnect() = 0;
	virtual bool isConnected() const = 0;

	virtual void setConnectionTimeOut(int ms) = 0;
	virtual void setResponseTimeOut(int ms) = 0;

	virtual int numChannel() const = 0;
	virtual int& numChannel() = 0;

	virtual int toggle(int ch, bool on) = 0;
	virtual int setIntensity(int ch, int intensity) = 0;
	virtual int setMultiIntensity(lsc::IntensityData* idata, int size) = 0;
	virtual int getIntensity(int ch, int& intensity) = 0;
	virtual int setIP(std::string ip) = 0; //ip/sn
	virtual int setPort(int port) = 0;

	virtual int setMode(lsc::MODE mode) = 0;
	virtual int setTriggerDuration(int ch, int us) = 0;
	virtual int setTriggerSequence(const std::vector<lsc::SequenceData>& datas) = 0;
	virtual int setMaxCurrent(int ch, double dCurrent) = 0;

	virtual std::string codeString(int returnCode) = 0;
};