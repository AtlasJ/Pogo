#pragma once
#include "ICamera.h"
namespace OPTSciCam {
	#include "OPT/SciCam.h"
}
#include "FrameInfo.h"
#include "MessageQue.h"

#include <condition_variable>

class CAM_OPT : public ICamera {
	
	enum LineSelectorType {
		Line1 = 0,
		Line2 = 1,
		Line3 = 2,

	};
	static const QMap<QString, LineSelectorType> lineSelectorMap;

	enum LineSourceType {
		UserOutput = 0,
		FrameTriggerWait = 1,
		ExposureActive = 2,
		Timer1Active = 18,

	};
	static const QMap<QString, LineSourceType> lineSourceMap;


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
	bool m_isStartGrabbing = false;

	bool m_isConnected = false;

	void* m_handle = nullptr;
	QString m_errorMsg = "";

	FrameInfo m_frameInfo;
	std::mutex m_mutex;
	std::condition_variable m_conditionVariable;

	bool accessible() const;

public:
	CAM_OPT();
	~CAM_OPT();

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
	 static void PayloadCallback(void* payload, void* pVoid);
};
