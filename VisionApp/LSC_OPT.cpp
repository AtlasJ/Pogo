#include "LSC_OPT.h"
#include "Logger.h"
#include "ScopedTimeLogger.h"

LSC_OPT::LSC_OPT()
{
}

LSC_OPT::~LSC_OPT()
{
}

const std::string & LSC_OPT::id() const
{
	return m_id;
}

std::string & LSC_OPT::id()
{
	return m_id;
}

const std::string & LSC_OPT::name() const
{
	return m_name;
}

std::string & LSC_OPT::name()
{
	return m_name;
}

int LSC_OPT::enable(bool toggle)
{
	m_enable = toggle;
	return (int)LSC_RC::PASS;
}

int LSC_OPT::connect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	char* serialNumber = new char[m_ip.length() + 1];
	std::strcpy(serialNumber, m_ip.c_str());

	ct::logger::info("IP for opt:%s", serialNumber);

	auto ret = OPTController_CreateEthernetConnectionBySN(serialNumber, &m_handler);
	delete[] serialNumber;
	ct::logger::info("IP for opt:%s", serialNumber);
	for (int i = 0; i < numChannel(); i++) {
		m_timeUnitMap.insert(std::pair<int,int>(0, -1));

		ct::logger::info("numchan:%d", i);
	}

	if (OPT_SUCCEED != ret) return (int)LSC_RC::INVALID_CONNECTION;
	
	return (int)LSC_RC::PASS;
}

int LSC_OPT::disconnect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	auto ret = OPTController_DestroyEthernetConnection(m_handler);
	if (OPT_SUCCEED != ret) return (int)LSC_RC::FAILED_TO_DISCONNECT;

	m_handler = NULL;
	return (int)LSC_RC::PASS;
}

int LSC_OPT::reconnect()
{
	return connect();
}

bool LSC_OPT::isConnected() const
{
	if (!m_enable) return true;

	if (OPTController_IsConnect(m_handler) != OPT_SUCCEED) return false;
	return true;
}

void LSC_OPT::setConnectionTimeOut(int ms)
{
	m_connectionTimeout = ms;
}

void LSC_OPT::setResponseTimeOut(int ms)
{
	m_responseTimeout = ms;
}

int LSC_OPT::numChannel() const
{
	if (!m_enable) return 0;

	int numChannel = 0;
	OPTController_GetControllerChannels(m_handler, &numChannel); 
	return numChannel;
}

int & LSC_OPT::numChannel()
{
	return m_numChannel;
}

int LSC_OPT::toggle(int ch, bool on)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	ch++;
	if (on) {
		if (OPTController_TurnOnChannel(m_handler, ch) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
	}
	else {
		if (OPTController_TurnOffChannel(m_handler, ch) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
	}

	return (int)LSC_RC::PASS;
}

int LSC_OPT::setIntensity(int ch, int intensity)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	ch++;
	if (OPTController_SetIntensity(m_handler, ch, intensity) != OPT_SUCCEED) return (int)LSC_RC::FAIL;

	return (int)LSC_RC::PASS;
}

int LSC_OPT::setMultiIntensity(lsc::IntensityData* idata, int size)
{
	int status = (int)LSC_RC::PASS;
	if (!m_enable) return status;

	IntensityItem* items = new IntensityItem[size];
	for (int i = 0; i < size; i++) {
		items[i].channelIndex = idata[i].channelIndex + 1;
		items[i].intensity = idata[i].intensity;
	}

	auto ret = OPTController_SetMultiIntensity(m_handler, items, 4);
	if (ret != OPT_SUCCEED) status = (int)LSC_RC::FAIL;

	delete[] items;

	return status;
}

int LSC_OPT::getIntensity(int ch, int & intensity)
{
	if (!m_enable) return (int)LSC_RC::PASS;
	ch++;
	if (OPTController_ReadIntensity(m_handler, ch, &intensity) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
	return (int)LSC_RC::PASS;
}

int LSC_OPT::setIP(std::string ip)
{
	m_ip = ip;
	return (int)LSC_RC::PASS;
}

int LSC_OPT::setPort(int port)
{
	m_port = port;
	return (int)LSC_RC::PASS;
}

int LSC_OPT::setMode(lsc::MODE mode)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	int channelSegment = 1;
	int modeInt = 0;

	if (mode == lsc::MODE::TRIGGER) {
		modeInt = 1;
		OPTController_SetTriggerMode(m_handler, channelSegment, 2);
	}
	else {
		OPTController_SetTriggerMode(m_handler, channelSegment, 1);
	}

	if (OPTController_SetWorkMode(m_handler, modeInt) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
	return (int)LSC_RC::PASS;
}

int LSC_OPT::setTriggerDuration(int ch, int us)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	ch++;

	if (0 <= us && us < 1000) {
		if (OPTController_SetTimeUnit(m_handler, ch, 0) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
		if (OPTController_SetTriggerWidth(m_handler, ch, us) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
	}
	else if (1000 <= us && us < 10000) {
		if (OPTController_SetTimeUnit(m_handler, ch, 1) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
		if (OPTController_SetTriggerWidth(m_handler, ch, us / 10) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
	}
	else if (10000 <= us && us < 1000000) {
		if (OPTController_SetTimeUnit(m_handler, ch, 2) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
		if (OPTController_SetTriggerWidth(m_handler, ch, us / 1000) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
	}
	else if (1000000 <= us && us < 100000000) {
		if (OPTController_SetTimeUnit(m_handler, ch, 3) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
		if (OPTController_SetTriggerWidth(m_handler, ch, us / 100000) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
	}

	return (int)LSC_RC::PASS;
}

int LSC_OPT::setTriggerSequence(const std::vector<lsc::SequenceData>& datas)
{
	ct::logger::info("Datas size: %d", datas.size());

	//TODO: Only support channel 1-4 currently
	//1: 1-4
	//2: 5-8 
	//...
	int channelSegment = 1;

	int numSeq = datas.size();
	int* triggerSources = new int[numSeq];
	int* pulseWidth = new int[numSeq * 4];
	int* intensity = new int[numSeq * 4];

	for (int i = 0; i < numSeq; i++) triggerSources[i] = 0;
	for (int i = 0; i < numSeq * 4; i++) {
		pulseWidth[i] = 0;
		intensity[i] = 0;
	}

	for (int i = 0; i < numSeq; i++) {
		
		triggerSources[i] = datas[i].triggerSource;

		for (int j = 0; j < datas[i].intensityDatas.size(); j++) {
			auto ch = datas[i].intensityDatas[j].channelIndex;

			ScopedTimeLogger stl("Set TIME UNIT");

			int index = ch + (i * 4);
			intensity[index] = datas[i].intensityDatas[j].intensity;

			int us = datas[i].exposure_us;

			if (0 <= us && us < 1000) {
				if (m_timeUnitMap[ch] != 0) {
					if (OPTController_SetTimeUnit(m_handler, ch, 0) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
					m_timeUnitMap[ch] = 0;
				}
				pulseWidth[index] = us;
			}
			else if (1000 <= us && us < 10000) {
				if (m_timeUnitMap[ch] != 1) {
					if (OPTController_SetTimeUnit(m_handler, ch, 1) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
					m_timeUnitMap[ch] = 1;
				}
				pulseWidth[index] = us / 10;
			}
			else if (10000 <= us && us < 1000000) {
				if (m_timeUnitMap[ch] != 2) {
					if (OPTController_SetTimeUnit(m_handler, ch, 2) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
					m_timeUnitMap[ch] = 2;
				}
				pulseWidth[index] = us / 1000;
			}
			else if (1000000 <= us && us < 100000000) {
				if (m_timeUnitMap[ch] != 3) {
					if (OPTController_SetTimeUnit(m_handler, ch, 3) != OPT_SUCCEED) return (int)LSC_RC::FAIL;
					m_timeUnitMap[ch] = 3;
				}
				pulseWidth[index] = us / 100000;
			}
		}
	}

	auto code = OPTController_SetSeqTable(m_handler, channelSegment, numSeq, triggerSources, intensity, pulseWidth);

	if (code != OPT_SUCCEED) {
		ct::logger::error("Failed to set trigger sequence: %ld", code);
		for (int i = 0; i < numSeq; i++) {
			ct::logger::info("Trigger source: %d", triggerSources[i]);

			int j = i * 4;

			ct::logger::info("Intensity: %d, %d, %d, %d", intensity[j], intensity[j + 1], intensity[j + 2], intensity[j + 3]);
			ct::logger::info("Pulse Width: %d, %d, %d, %d", pulseWidth[j], pulseWidth[j + 1], pulseWidth[j + 2], pulseWidth[j + 3]);
		}

		return (int)LSC_RC::FAIL;
	}
	ct::logger::info("Set Trigger Sequence Success!");

	/*int* triggerSourcesRet = new int[numSeq];
	int* pulseWidthRet = new int[numSeq * 4];
	int* intensityRet = new int[numSeq * 4];
	OPTController_ReadSeqTable(m_handler, channelSegment, &numSeq, triggerSourcesRet, intensityRet, pulseWidthRet);

	for (int i = 0; i < numSeq; i++) {
		ct::logger::info("Trigger source: %d", triggerSourcesRet[i]);

		int j = i * 4;

		ct::logger::info("Intensity: %d, %d, %d, %d", intensityRet[j], intensityRet[j + 1], intensityRet[j + 2], intensityRet[j + 3]);
		ct::logger::info("Pulse Width: %d, %d, %d, %d", pulseWidthRet[j], pulseWidthRet[j + 1], pulseWidthRet[j + 2], pulseWidthRet[j + 3]);

	}
	
	delete[] triggerSourcesRet;
	delete[] intensityRet;
	delete[] pulseWidthRet;
	*/

	delete[] triggerSources;
	delete[] intensity;
	delete[] pulseWidth;

	return (int)LSC_RC::PASS;
}

std::string LSC_OPT::codeString(int returnCode)
{
	return std::string();
}

int LSC_OPT::setMaxCurrent(int ch, double dCurrent)
{
	return (int)LSC_RC::PASS;
}
