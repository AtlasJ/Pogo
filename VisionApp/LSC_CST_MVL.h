#pragma once
#include "ILSC.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

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

	int m_lastIntensity[4] = { 0, 0, 0, 0 }; //last nonzero intensity per channel - restores on toggle(on)
	lsc::MODE m_mode = lsc::MODE::CONTINUOUS;

	//strobe setup (see setStrobeConfig): trigger source follows the channel
	//trigger_type in lsc.json (LAN = internal trigger, IO = external trigger)
	bool m_internalTrigger = true;
	int m_strobeInternalCycle = 0;//internal trigger cycle via SetIntCycleValue, 0 = leave controller setting

	ControllerHandle m_handler;

	/*
	* The MVL controller drops an idle TCP client after a few seconds (observed ~5 s:
	* the first command after connect succeeds, everything later fails with ERROR_TX).
	* KeepAlive() every 2 s holds the session open. The mutex serializes ALL controller
	* commands - they arrive from the GUI thread, the job thread and the keepalive
	* thread, and the protocol cannot interleave requests.
	*/
	std::thread m_keepAliveThread;
	std::atomic<bool> m_keepAliveRun{ false };
	std::mutex m_ioMutex;
	void keepAliveLoop();
	void stopKeepAlive();

	/*
	* Observed on the machine: only the FIRST command after a connection succeeds -
	* the controller appears to reset the session per command (CST's own tool still
	* works, so it must reconnect too). Every command therefore runs through this
	* helper: on failure it reconnects and retries once. Assumes m_ioMutex is held.
	*/
	bool reconnectLocked();
	int runCmd(const char* what, const std::function<int()>& fn);

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

	void setStrobeConfig(bool internalTrigger, int internalCycle);
	int setMode(lsc::MODE mode);
	int setTriggerDuration(int ch, int us);
	int setTriggerSequence(const std::vector<lsc::SequenceData>& datas);

	std::string codeString(int returnCode) override;

	int setMaxCurrent(int ch, double dCurrent);
};