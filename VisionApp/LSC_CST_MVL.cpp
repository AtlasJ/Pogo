#include "LSC_CST_MVL.h"
#include "Logger.h"

LSC_CST_MVL::LSC_CST_MVL()
{
}

LSC_CST_MVL::~LSC_CST_MVL()
{
	stopKeepAlive();
}

//hold the controller's TCP session open - it drops idle clients (ERROR_TX follows)
void LSC_CST_MVL::keepAliveLoop()
{
	int lastRet = SUCCESS;
	while (m_keepAliveRun) {
		//2 s period, sliced so shutdown stays responsive
		for (int i = 0; i < 20 && m_keepAliveRun; i++)
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if (!m_keepAliveRun) break;

		std::lock_guard<std::mutex> lock(m_ioMutex);
		if (!m_isConnect) continue;

		const int ret = KeepAlive(m_connectionType, m_handler);
		if (ret != SUCCESS && lastRet == SUCCESS)
			ct::logger::warn("[LSC_CST_MVL] KeepAlive failed (%d) - controller connection lost?", ret);
		else if (ret == SUCCESS && lastRet != SUCCESS)
			ct::logger::info("[LSC_CST_MVL] KeepAlive recovered");
		lastRet = ret;
	}
}

void LSC_CST_MVL::stopKeepAlive()
{
	m_keepAliveRun = false;
	if (m_keepAliveThread.joinable()) m_keepAliveThread.join();
}

bool LSC_CST_MVL::reconnectLocked()
{
	DestroyIpConnection(m_handler);
	m_isConnect = false;

	const int ret = (m_ip == "")
		? CreateSerialPort(m_port, &m_handler)
		: ConnectIP(&m_ip[0], m_connectionTimeout / 1000, &m_handler);

	if (ret == SUCCESS) {
		m_isConnect = true;
		ct::logger::info("[LSC_CST_MVL] Reconnected to %s", m_ip.c_str());
		return true;
	}

	ct::logger::error("[LSC_CST_MVL] Reconnect FAILED (%d)", ret);
	return false;
}

int LSC_CST_MVL::runCmd(const char* what, const std::function<int()>& fn)
{
	int ret = fn();
	if (ret == SUCCESS) return ret;

	char msg[512] = { 0 };
	GetErrMsg(ret, msg);
	ct::logger::warn("[LSC_CST_MVL] %s failed (%d: %s) - reconnecting to retry", what, ret, msg);

	if (!reconnectLocked()) return ret;

	ret = fn();
	ct::logger::info("[LSC_CST_MVL] %s retry after reconnect: %s (%d)",
		what, ret == SUCCESS ? "SUCCESS" : "FAIL", ret);
	return ret;
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

		{
			char model[128] = { 0 };
			if (GetControllerModel_s(m_connectionType, model, m_handler) == SUCCESS)
				ct::logger::info("[LSC_CST_MVL] Controller model: %s", model);
			else
				ct::logger::warn("[LSC_CST_MVL] GetControllerModel_s failed - model unknown");
		}

		//the controller drops idle clients: keep the session alive
		stopKeepAlive();
		m_keepAliveRun = true;
		m_keepAliveThread = std::thread(&LSC_CST_MVL::keepAliveLoop, this);
		ct::logger::info("[LSC_CST_MVL] KeepAlive thread started (2 s period)");
		return (int)LSC_RC::PASS;
	}

	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::disconnect()
{
	if (!m_enable) return (int)LSC_RC::PASS;

	stopKeepAlive();

	std::lock_guard<std::mutex> lock(m_ioMutex);
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

	/*
	* This controller has no on/off command (SetON_OFF/SetLightState get no reply).
	* Emulated with the intensity register instead: OFF writes 0, ON restores the
	* last intensity that was set on the channel.
	*/
	if (ch < 0 || ch >= 4) return (int)LSC_RC::FAIL;

	/*
	* Strobe + EXTERNAL trigger: the light only emits while a Y110 pulse runs - between
	* pulses it is already dark, so on/off has nothing to do. Writing 0 here also RACED
	* the next snap: the intensity was rewritten ~7 ms before the trigger edge and the
	* flash sometimes fired before the controller latched it (randomly darker images).
	* Strobe + INTERNAL trigger free-runs, so there the 0/restore emulation MUST stay -
	* it is the only way to stop the flashing.
	*/
	if (m_mode == lsc::MODE::TRIGGER && !m_internalTrigger) {
		ct::logger::trace("[LSC_CST_MVL] toggle %s ch %d skipped - external strobe only fires on trigger",
			on ? "ON" : "OFF", ch + 1);
		return (int)LSC_RC::PASS;
	}

	const int target = on ? m_lastIntensity[ch] : 0;
	if (on && target <= 0) {
		ct::logger::trace("[LSC_CST_MVL] toggle ON ch %d skipped - no cached intensity yet", ch + 1);
		return (int)LSC_RC::PASS;
	}

	std::lock_guard<std::mutex> lock(m_ioMutex);

	const bool strobe = (m_mode == lsc::MODE::TRIGGER);
	auto ret = runCmd(strobe ? "SetStrobeValue(toggle)" : "SetDigitalValue(toggle)", [&]() {
		return strobe
			? SetStrobeValue(m_connectionType, ch + 1, target, m_handler)
			: SetDigitalValue(m_connectionType, ch + 1, target, m_handler);
	});

	ct::logger::debug("[LSC_CST_MVL] toggle ch=%d %s -> intensity %d (%s register) ret=%d",
		ch + 1, on ? "ON" : "OFF", target, strobe ? "STROBE" : "CONTINUOUS", ret);

	if (ret == SUCCESS) return (int)LSC_RC::PASS;
	ct::logger::error("[LSC_CST_MVL] Failed to toggle ch %d (%d)", ch + 1, ret);
	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::setIntensity(int ch, int intensity)
{
	if (!m_enable) {
		ct::logger::warn("[LSC_CST_MVL][strobe] setIntensity ch %d SKIPPED - controller disabled", ch + 1);
		return (int)LSC_RC::PASS;
	}

	std::lock_guard<std::mutex> lock(m_ioMutex);

	//strobe mode uses its own intensity register
	const bool strobe = (m_mode == lsc::MODE::TRIGGER);
	auto ret = runCmd(strobe ? "SetStrobeValue" : "SetDigitalValue", [&]() {
		return strobe
			? SetStrobeValue(m_connectionType, ch + 1, intensity, m_handler)
			: SetDigitalValue(m_connectionType, ch + 1, intensity, m_handler);
	});

	ct::logger::debug("[LSC_CST_MVL] %s ch=%d val=%d ret=%d",
		strobe ? "SetStrobeValue" : "SetDigitalValue", ch + 1, intensity, ret);

	if (ret == SUCCESS) {
		if (ch >= 0 && ch < 4 && intensity > 0) m_lastIntensity[ch] = intensity; //toggle(on) restore value
		return (int)LSC_RC::PASS;
	}
	ct::logger::error("[LSC_CST_MVL] Failed to set %s intensity ch %d (%d)",
		strobe ? "strobe" : "digital", ch + 1, ret);
	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::setMultiIntensity(lsc::IntensityData* idata, int size)
{
	if (!m_enable) {
		ct::logger::warn("[LSC_CST_MVL][strobe] setMultiIntensity SKIPPED - controller disabled");
		return (int)LSC_RC::PASS;
	}

	std::lock_guard<std::mutex> lock(m_ioMutex);

	int ret = SUCCESS;
	const bool strobe = (m_mode == lsc::MODE::TRIGGER);

	{
		std::string vals;
		for (int i = 0; i < size; i++) {
			if (!vals.empty()) vals += ", ";
			vals += "ch" + std::to_string(idata[i].channelIndex + 1) + "=" + std::to_string(idata[i].intensity);
		}
		ct::logger::info("[LSC_CST_MVL][strobe] %s: %s",
			strobe ? "SetMulStrobeValue" : "SetMulDigitalValue", vals.c_str());
	}

	if (strobe) {
		std::vector<MulStbValItem> items(size);
		for (int i = 0; i < size; i++) {
			items[i].channelIndex = idata[i].channelIndex + 1;
			items[i].StrobeValue = idata[i].intensity;
		}
		ret = runCmd("SetMulStrobeValue", [&]() { return SetMulStrobeValue(m_connectionType, items.data(), size, m_handler); });
	}
	else {
		std::vector<MulDigValItem> items(size);
		for (int i = 0; i < size; i++) {
			items[i].channelIndex = idata[i].channelIndex + 1;
			items[i].DigitalValue = idata[i].intensity;
		}
		ret = runCmd("SetMulDigitalValue", [&]() { return SetMulDigitalValue(m_connectionType, items.data(), size, m_handler); });
	}

	ct::logger::info("[LSC_CST_MVL][strobe] %s returned %d (%s)",
		strobe ? "SetMulStrobeValue" : "SetMulDigitalValue", ret,
		ret == SUCCESS ? "SUCCESS" : "FAIL");

	if (ret == SUCCESS) {
		for (int i = 0; i < size; i++) {
			const int ch = idata[i].channelIndex;
			if (ch >= 0 && ch < 4 && idata[i].intensity > 0) m_lastIntensity[ch] = idata[i].intensity;
		}
		return (int)LSC_RC::PASS;
	}
	return (int)LSC_RC::FAIL;
}

int LSC_CST_MVL::getIntensity(int ch, int & intensity)
{
	if (!m_enable) return (int)LSC_RC::PASS;

	std::lock_guard<std::mutex> lock(m_ioMutex);

	auto ret = runCmd("GetIntensity", [&]() {
		return (m_mode == lsc::MODE::TRIGGER)
			? GetStrobeValue(m_connectionType, &intensity, ch + 1, m_handler)
			: GetDigitalValue(m_connectionType, &intensity, ch + 1, m_handler);
	});

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

//SetLightTriMode values. Observed on this machine's strobe-only controller:
//value 0 is ACCEPTED (and the box then fires on its external trigger input),
//value 2 is REJECTED (no reply). So 0 is treated as external/default and 1 is
//the remaining candidate for internal - unverified, test with trigger_type=LAN
//and a strobe_internal_cycle in lsc.json.
static constexpr int TRI_MODE_CONTINUOUS = 0;
static constexpr int TRI_MODE_EXTERNAL = 0;
static constexpr int TRI_MODE_INTERNAL = 1;

void LSC_CST_MVL::setStrobeConfig(bool internalTrigger, int internalCycle)
{
	m_internalTrigger = internalTrigger;
	m_strobeInternalCycle = internalCycle;
	ct::logger::info("[LSC_CST_MVL] Strobe config: trigger=%s, internal_cycle=%d",
		internalTrigger ? "INTERNAL" : "EXTERNAL", internalCycle);
}

int LSC_CST_MVL::setMode(lsc::MODE mode)
{
	if (!m_enable) {
		ct::logger::warn("[LSC_CST_MVL][strobe] setMode SKIPPED - controller disabled (mode stays %s)",
			m_mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS");
		return (int)LSC_RC::PASS;
	}

	ct::logger::info("[LSC_CST_MVL][strobe] setMode: %s (internalTrigger=%d, internalCycle=%d)",
		mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS",
		m_internalTrigger ? 1 : 0, m_strobeInternalCycle);

	std::lock_guard<std::mutex> lock(m_ioMutex);

	//trigger source follows the channel trigger_type: LAN = internal, IO = external
	int triMode = TRI_MODE_CONTINUOUS;
	if (mode == lsc::MODE::TRIGGER) triMode = m_internalTrigger ? TRI_MODE_INTERNAL : TRI_MODE_EXTERNAL;

	auto ret = runCmd("SetLightTriMode", [&]() { return SetLightTriMode(m_connectionType, triMode, m_handler); });

	if (ret != SUCCESS) {
		//strobe-only controllers ignore the trigger-mode command entirely (no reply,
		//200 ms timeout). The mode is still tracked so intensity goes to the strobe
		//register and the pulse width is applied - which IS how this model works.
		ct::logger::warn("[LSC_CST_MVL] SetLightTriMode %d refused (%d) - strobe-only controller assumed, "
			"tracking mode %s anyway", triMode, ret,
			mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS");
		m_mode = mode;
		return (int)LSC_RC::PASS;
	}

	m_mode = mode;
	ct::logger::info("[LSC_CST_MVL] Mode set to %s (tri_mode=%d)",
		mode == lsc::MODE::TRIGGER ? "STROBE" : "CONTINUOUS", triMode);

	//internal trigger cycle (strobe period when self-triggered)
	if (mode == lsc::MODE::TRIGGER && m_internalTrigger && m_strobeInternalCycle > 0) {
		ret = runCmd("SetIntCycleValue", [&]() { return SetIntCycleValue(m_connectionType, m_strobeInternalCycle, m_handler); });
		if (ret != SUCCESS) ct::logger::error("[LSC_CST_MVL] Failed to set internal cycle %d (%d)", m_strobeInternalCycle, ret);
		else ct::logger::info("[LSC_CST_MVL] Internal cycle set to %d", m_strobeInternalCycle);
	}


	return (int)LSC_RC::PASS;
}

int LSC_CST_MVL::setTriggerDuration(int ch, int us)
{
	if (!m_enable) {
		ct::logger::warn("[LSC_CST_MVL][strobe] setTriggerDuration ch %d SKIPPED - controller disabled", ch + 1);
		return (int)LSC_RC::PASS;
	}

	std::lock_guard<std::mutex> lock(m_ioMutex);
	auto ret = runCmd("SetPulseUnit", [&]() { return SetPulseUnit(m_connectionType, ch + 1, us, m_handler); });
	ct::logger::info("[LSC_CST_MVL][strobe] SetPulseUnit ch=%d us=%d ret=%d (%s)",
		ch + 1, us, ret, ret == SUCCESS ? "SUCCESS" : "FAIL");

	if (ret == SUCCESS) return (int)LSC_RC::PASS;
	ct::logger::error("[LSC_CST_MVL] Failed to set pulse width on ch %d (%d)", ch + 1, ret);
	return (int)LSC_RC::FAIL;
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