#pragma once
#include "ILSC.h"
#include "QClient.h"
#include <QTcpSocket>

#include "CodeConfig.h"
#include "QSocketWorker.h"

class LSC_VLP : public ILSC {
private:
	std::string m_id = "";
	std::string m_name = "";
	bool m_enable = true;
	std::string m_ip = "192.168.11.20";
	int m_port = 1000;
	QClient m_client;
	//QTcpSocket* m_socket = nullptr;
	QSocketWorker* m_socketWorker = nullptr;
	int m_connectionTimeout = 3000;
	int m_responseTimeout = 300;
	int m_numChannel = 4;

	int m_intensity[4] = { 150, 14, 31, 0 };

	std::string getCheckSum(std::string);
	std::string getChannelStr(int);
	std::string getIntensityStr(int);

public:
	LSC_VLP();
	~LSC_VLP();

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
	int& numChannel() override;

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
	int setMaxCurrent(int ch, double dCurrent);

	std::string codeString(int returnCode) override;

	//custom
	void writeToLSC(int ch, int intensity);
};