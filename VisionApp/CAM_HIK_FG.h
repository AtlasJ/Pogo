#pragma once
#include "ICamera.h"
#include "FrameInfo.h"
#include "MessageQue.h"
namespace mvfg{ 
#include "MVFGControl.h" 
}
#include <QString>

#include <condition_variable>

class CAM_HIK_FG : public ICamera {
private:
	bool m_enable = true;
	int m_width = 0;
	int m_height = 0;
	int m_channel = 1; //1: Mono, 3: Color
	double m_exposure = 10000.0;
	double m_gain = 1.0;
	int m_id = 0;
	QString m_name = "";
	QString m_serialNumber = "";
	bool m_softTriggered = false;

	void* m_stream = nullptr;
	void* m_handle = nullptr;
	void* m_interface = nullptr;
	bool m_connected = false;
	bool m_grabbing = false;

	QString m_errorMsg = "";

	FrameInfo m_frameInfo;
	std::mutex m_mutex;
	std::condition_variable m_conditionVariable;

	int m_index = 0;

	bool accessible() const;
	const char* getStringRetCode(int ret) const;
	bool logErrorCode(const char* msg, int ret);

	void setupStandardParams();

public:
	CAM_HIK_FG();
	~CAM_HIK_FG();

	//Query
	const int getWidth() const;
	const int getHeight() const;
	const int getChannel() const;

	const double getExposure() const;
	const double getGain() const;

	const QString& getName() const;
	const QString& getSerialNumber() const;

	bool isConnected() const;
	const bool isGrabbing() const;


	//Connection
	bool enable(bool enable);
	bool connect(QString sn);
	bool disconnect();


	//Acquisition
	bool startGrab();
	bool stopGrab();


	//Control
	bool setExposure(double exposure);
	bool setGain(double gain);
	bool softTrigger();
	bool waitAcquisition(int ms);
	bool setTriggerOutput(QString line, QString source);

	bool setDO(int DO, bool on);

	//Misc
	bool loadConfig(QString path);
	QString errorMsg();


	//Data
	const FrameInfo& frame() const;
	FrameInfo& frame();
	void resetFrame();

	//Callback
	static void __stdcall FrameCallback(mvfg::MV_FG_BUFFER_INFO* pInfo, void* pUser);
};
