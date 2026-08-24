#pragma once
#include "ILSC.h"

#include "MVL/CommonToolDll.h"
#include "MVL/ControllerDll.h"
#include "MVL/sControllerDll.h"
#include "MVL/ErrorCode.h"

class LSC_CST_MVL : public ILSC {
private:
	std::string m_id = "";
	std::string m_name = "";
	bool m_enable = true;
	bool m_isConnect = false;
	std::string m_ip = "192.168.1.208";
	int m_port = 1000;
	int m_connectionTimeout = 3000;
	int m_responseTimeout = 1000;
	int m_numChannel = 0;
	int m_connectionType = 0;

	int m_lastIntensity[4] = { 0, 0, 0, 0 };
	lsc::MODE m_mode = lsc::MODE::CONTINUOUS;

	ControllerHandle m_handler;

public:
	LSC_CST_MVL();
	~LSC_CST_MVL();

	const std::string& id() const override;
	std::string& id() override;

	const std::string& name() const override;
	std::string& name() override;

	int enable(bool enable) override;
	int connect() override;
	int disconnect() override;
	int reconnect() override;
	bool isConnected() const override;

	void setConnectionTimeOut(int ms) override;
	void setResponseTimeOut(int ms) override;

	int numChannel() const override;
	int& numChannel() override; // Not using, MVL provide their own method

	int toggle(int ch, bool on) override;
	int setIntensity(int ch, int intensity) override;
	int setMultiIntensity(lsc::IntensityData* idata, int size) override;
	int getIntensity(int ch, int& intensity) override;

	int setIP(std::string ip) override;
	int setPort(int port) override;

	int setMode(lsc::MODE mode);
	int setTriggerDuration(int ch, int us);
	int setTriggerSequence(const std::vector<lsc::SequenceData>& datas);

	std::string codeString(int returnCode) override;

	int setMaxCurrent(int ch, double dCurrent);
};