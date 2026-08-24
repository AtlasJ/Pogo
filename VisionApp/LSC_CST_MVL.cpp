#include "LSC_CST_MVL.h"
#include "Logger.h"

LSC_CST_MVL::LSC_CST_MVL()
{
}

LSC_CST_MVL::~LSC_CST_MVL()
{
}

const std::string & LSC_CST_MVL::id() const
{
	return m_id;
}

std::string & LSC_CST_MVL::id()
{
	return m_id;
}

const std::string & LSC_CST_MVL::name() const
{
	return m_name;
}

std::string & LSC_CST_MVL::name()
{
	return m_name;
}

int LSC_CST_MVL::enable(bool enable)
{
	m_enable = true;
	return (int)LSC_RC::PASS;
}

int LSC_CST_MVL::connect()
{
	if (!m_enable) return (int)LSC_RC::PASS;
	ct::logger::info("Connecting to MVL LSC: %s...", m_ip.c_str());

	int ret = ERROR_CONNECT;

	if (m_ip == "") {
		ret = CreateSerialPort(m_port, &m_handler);
		m_connectionType = 1;
	}
	else {
		ret = ConnectIP(&m_ip[0], m_connectionTimeout / 1000, &m_handler);
		m_connectionType = 0;
	}

	m_isConnect = false;

	if (ret == SUCCESS) {
		m_isConnect = true;
		ct::logger::info("Connected to MVL LSC: %s", m_ip.c_str());
		return (int)LSC_RC::PASS;
	}

	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::disconnect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	auto ret = DestroyIpConnection(m_handler);

	if (ret == SUCCESS) {
		m_isConnect = false;
		return (int)LSC_RC::PASS;
	}

	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::reconnect()
{
	return connect();
}

bool LSC_CST_MVL::isConnected() const
{
	return m_isConnect;
}

void LSC_CST_MVL::setConnectionTimeOut(int ms)
{
	m_connectionTimeout = ms;
}

void LSC_CST_MVL::setResponseTimeOut(int ms)
{
	m_responseTimeout = ms;
}

int LSC_CST_MVL::numChannel() const
{
	int num;
	GetChannelNumberSummary_s(m_connectionType, &num, m_handler);
	return num;
}

int & LSC_CST_MVL::numChannel()
{
	return m_numChannel;
}

int LSC_CST_MVL::toggle(int ch, bool on)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	auto ret = SetON_OFF_s(m_connectionType, ch+1, (int)on, m_handler);

	if (ret == SUCCESS) return (int)LSC_RC::PASS;
	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::setIntensity(int ch, int intensity)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	//strobe mode uses its own intensity register
	auto ret = (m_mode == lsc::MODE::TRIGGER)
		? SetStrobeValue(m_connectionType, ch+1, intensity, m_handler)
		: SetDigitalValue(m_connectionType, ch+1, intensity, m_handler);

	if (ret == SUCCESS) return (int)LSC_RC::PASS;
	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::setMultiIntensity(lsc::IntensityData* idata, int size)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	int ret = SUCCESS;

	if (m_mode == lsc::MODE::TRIGGER) {
		std::vector<MulStbValItem> items(size);
		for (int i = 0; i < size; i++) {
			items[i].channelIndex = idata[i].channelIndex + 1;
			items[i].StrobeValue = idata[i].intensity;
		}
		ret = SetMulStrobeValue(m_connectionType, items.data(), size, m_handler);
	}
	else {
		std::vector<MulDigValItem> items(size);
		for (int i = 0; i < size; i++) {
			items[i].channelIndex = idata[i].channelIndex + 1;
			items[i].DigitalValue = idata[i].intensity;
		}
		ret = SetMulDigitalValue(m_connectionType, items.data(), size, m_handler);
	}

	if (ret == SUCCESS) return (int)LSC_RC::PASS;
	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::getIntensity(int ch, int & intensity)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	auto ret = (m_mode == lsc::MODE::TRIGGER)
		? GetStrobeValue(m_connectionType, &intensity, ch+1, m_handler)
		: GetDigitalValue(m_connectionType, &intensity, ch+1, m_handler);

	if (ret == SUCCESS) return (int)LSC_RC::PASS;
	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::setIP(std::string ip)
{
	m_ip = ip;
	return (int)LSC_RC::PASS;
}

int LSC_CST_MVL::setPort(int port)
{
	m_port = port;
	return (int)LSC_RC::PASS;
}

int LSC_CST_MVL::setMode(lsc::MODE mode)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	//0 = continuous output, 1 = external trigger (strobe)
	int triMode = (mode == lsc::MODE::TRIGGER) ? 1 : 0;
	auto ret = SetLightTriMode(m_connectionType, triMode, m_handler);

	if (ret == SUCCESS) {
		m_mode = mode;
		ct::logger::info("[LSC_CST_MVL] Mode set to %s", triMode ? "STROBE" : "CONTINUOUS");
		return (int)LSC_RC::PASS;
	}

	ct::logger::error("[LSC_CST_MVL] Failed to set light trigger mode (%d)", ret);
	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::setTriggerDuration(int ch, int us)
{
	return (int)LSC_RC::PASS;
}

int LSC_CST_MVL::setTriggerSequence(const std::vector<lsc::SequenceData>& datas)
{
	return (int)LSC_RC::PASS;
}

std::string LSC_CST_MVL::codeString(int returnCode)
{
	return std::string();
}

int LSC_CST_MVL::setMaxCurrent(int ch, double dCurrent)
{
	return (int)LSC_RC::PASS;
}