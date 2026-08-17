#pragma once
#include "ILSC.h"
#include <array>
#include <set>
#include <QString>

#include "VTLight_define.h"
#include "VTLight_ErrorCode.h"
#include "VTLight_typedef.h"
#include "VTLightController.h"
#include "VTLightControllerManager.h"

class LSC_VIE : public ILSC {
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
	std::array<double, 32> m_maxCurrent;
	std::array<bool, 32> m_hasMaxCurrent;

	CVTLightControllerManager m_manager;
	CVTLightController m_controller;
	std::vector<STLightCtrlStatus> m_lsc;

	std::vector<lsc::SequenceData> m_sequenceDatas;

	std::array<DWORD, 32> m_continuousGroupID; //can expand if future model supports more channels
	std::string getStringRetCode(VI32 ret);

	DWORD getGroupID(const std::set<int>& channels);
	int setContinuousMode();
	int setTriggerMode();
	QString maxCurrentJsonPath() const;
	void loadMaxCurrentJson();
	bool saveMaxCurrentJson() const;
	int applySavedMaxCurrent();
	bool isValidChannelIndex(int ch) const;
	double maxCurrentFor(int ch) const;
	double intensityToCurrent(int ch, int intensity) const;
	int currentToIntensity(int ch, double current) const;

	//Local func
	struct LSCSettings {
		int* settingID = nullptr;
		double* current = nullptr;
	};

public:
	LSC_VIE();
	~LSC_VIE();

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
