#pragma once
#include "ILSC.h"

#include "OPTController.h"
#include "OPTErrorCode.h"

#include <map>

class LSC_OPT : public ILSC {
private:
	std::string m_id = "";
	std::string m_name = "";
	bool m_enable = true;
	std::string m_ip = "192.168.11.20";
	int m_port = 1000;
	int m_connectionTimeout = 3000;
	int m_responseTimeout = 1000;
	int m_numChannel = 0;

	int m_lastIntensity[4] = { 150, 14, 31, 0 };

	std::map<int, int> m_timeUnitMap;

	OPTController_Handle m_handler;

public:
	LSC_OPT();
	~LSC_OPT();

	const std::string& id() const override;
	std::string& id() override;

	const std::string& name() const override;
	std::string& name() override;

	int enable(bool toggle) override;
	int connect() override;
	int disconnect() override;
	int reconnect() override;
	bool isConnected() const override;

	void setConnectionTimeOut(int ms) override;
	void setResponseTimeOut(int ms) override;

	int numChannel() const override;
	int& numChannel() override; //not using since OPT provide a way to get channels

	int toggle(int ch, bool on) override;
	int setIntensity(int ch, int intensity) override;
	int setMultiIntensity(lsc::IntensityData* idata, int size) override;
	int getIntensity(int ch, int& intensity) override;

	//
	int setIP(std::string ip) override;
	int setPort(int port) override;

	int setMode(lsc::MODE mode);
	int setTriggerDuration(int ch, int us);
	int setTriggerSequence(const std::vector<lsc::SequenceData>& datas);

	std::string codeString(int returnCode) override;
	int setMaxCurrent(int ch, double dCurrent);
};