#pragma once                  
 # include "ICamera.h"
#include "IMVFG/IMVFGApi.h"
#include "FrameInfo.h"
#include "MessageQue.h"       

#include <condition_variable>

class CAM_IRayple_FG : public ICamera {
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

	bool m_isConnected = false;

	IMV_FG_IF_HANDLE m_ifHandle = nullptr;
	IMV_FG_DEV_HANDLE m_devHandle = nullptr;
	QString m_errorMsg = "";

	FrameInfo m_frameInfo;
	std::mutex m_mutex;
	std::condition_variable m_conditionVariable;

	bool accessible() const;

public:
	CAM_IRayple_FG();
	~CAM_IRayple_FG();

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
	static void FrameCallback(IMV_FG_Frame* pFrame, void* pVoid);
};
