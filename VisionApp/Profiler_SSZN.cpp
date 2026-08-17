#pragma once
#include "Profiler_SSZN.h"
#include "Logger.h"
#include "mtrx.h"
#include "MessageQue.h"
#include "CommonDir.h"
#include "QCommonStruct.h"
#include <QFileDialog>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include "MbufPoolManager.h"

extern TMessageQue<FrameInfo> g_imageQueue;
Profiler_SSZN* Profiler_SSZN::s_instance = nullptr;

void Profiler_SSZN::BatchOneTimeCallBack(const void* info, const SR7IF_Data* data)
{
    const SR7IF_STR_CALLBACK_INFO* conInfo = (const SR7IF_STR_CALLBACK_INFO*)info;

    if (!s_instance || s_instance->m_release.load()) return; //actually this is not necessary if callback is disconnected

    ct::logger::info("[Profiler_SSZN] Callback triggered");

    if (conInfo->returnStatus != SR7IF_OK)
    {
        if (conInfo->returnStatus == -100) {
            ct::logger::warn(
                "[Profiler_SSZN] Callback warning, Return Value = %d",
                conInfo->returnStatus
            );
        }
        else {
            ct::logger::error(
                "[Profiler_SSZN] Callback error, Return Value = %d",
                conInfo->returnStatus
            );
        }
        return;
    }

    const int rows = conInfo->BatchPoints;
    const int cols = conInfo->xPoints;
    if (rows <= 0 || cols <= 0) {
        ct::logger::warn(
            "[Profiler_SSZN] Empty batch (rows=%d, cols=%d) — skipping",
            rows, cols
        );
        return;
    }

    ct::logger::info("[Profiler_SSZN] Batch Info:");
    ct::logger::info("  Sensor Heads: %d", conInfo->HeadNumber);
    ct::logger::info("  Batch Lines Retrieved: %d", rows);
    ct::logger::info("  X-axis Data Points: %d", cols);
    ct::logger::info("  X-axis Pitch: %.5f", conInfo->xPixth);

    const int head = 0;
    const int* profPtr = SR7IF_GetBatchProfilePoint(data, head);
    const unsigned char* grayPtr = SR7IF_GetBatchIntensityPoint(data, head);
    const unsigned int* encPtr = SR7IF_GetBatchEncoderPoint(data, head);

    if (!profPtr) { ct::logger::error("[Profiler_SSZN] profile pointer is null");  return; }
    if (!grayPtr) { ct::logger::error("[Profiler_SSZN] intensity pointer is null"); return; }
    if (!encPtr) { ct::logger::warn("[Profiler_SSZN] encoder pointer is null (continuing without it)"); }

    if (!s_instance->testFlag)
    {
        FrameInfo& frame = s_instance->m_frameInfo;
        frame.profiles.clear();

        frame.type = ct::s_height_map;
        frame.width = cols;
        frame.height = rows;
        frame.bufferSize = cols * rows;
        frame.channel = 1;

        // --- Allocate target MIL buffers ---
        frame.pHeightMap = mtrx::MPM::instance().acquire(cols, rows, 1, 16 + M_UNSIGNED);
        frame.pImage = mtrx::MPM::instance().acquire(cols, rows, 1, 8 + M_UNSIGNED);

        if (!frame.pHeightMap || !frame.pImage) {
            ct::logger::error("[Profiler_SSZN] shared buffers not allocated");
            return;
        }

        MIL_ID hId = frame.pHeightMap->id();
        MIL_ID gId = frame.pImage->id();

        MIL_UINT16* hPtr = M_NULL; MIL_INT pitchBytes16 = 0;
        MIL_UINT8* gPtr = M_NULL; MIL_INT pitchBytes8 = 0;

        MbufInquire(hId, M_HOST_ADDRESS, &hPtr);
        MbufInquire(hId, M_PITCH, &pitchBytes16);

        MbufInquire(gId, M_HOST_ADDRESS, &gPtr);
        MbufInquire(gId, M_PITCH, &pitchBytes8);



        const MIL_INT pitch16 = pitchBytes16;
        const MIL_INT pitch8 = pitchBytes8;


        constexpr int    INVALID_SENTINEL = -100000000;
        constexpr double MAX_U16 = 65535.0;


        constexpr double MIN_VAL = -400000.0;
        constexpr double MAX_VAL = 400000.0;
        constexpr double RANGE = (MAX_VAL - MIN_VAL);

        auto fixedToU16 = [](int v32) -> MIL_UINT16
            {
                constexpr double MIN_VAL_L = -400000.0;
                constexpr double MAX_VAL_L = 400000.0;
                constexpr double RANGE_L = (MAX_VAL_L - MIN_VAL_L);
                constexpr double MAX_U16_L = 65535.0;

                double v = static_cast<double>(v32);

                // clamp to fixed range
                if (v < MIN_VAL_L) v = MIN_VAL_L;
                if (v > MAX_VAL_L) v = MAX_VAL_L;

                // map [MIN_VAL, MAX_VAL] -> [0, 1]
                double norm = (v - MIN_VAL_L) / RANGE_L;

                if (norm < 0.0) norm = 0.0;
                if (norm > 1.0) norm = 1.0;

                double s = norm * MAX_U16_L;
                return static_cast<MIL_UINT16>(s + 0.5);
            };


        if (true) //normalisation
        {
            for (int r = 0; r < rows; ++r) {
                MIL_UINT16* hLine = hPtr + r * pitch16;
                MIL_UINT8* gLine = gPtr + r * pitch8;
                const int* pRow = profPtr + (r * cols);
                const unsigned char* gRow = grayPtr + (r * cols);

                for (int c = 0; c < cols; ++c) {
                    int v32 = pRow[c];

                    if (v32 <= INVALID_SENTINEL) {
                        hLine[c] = 0;   // invalid -> black
                    }
                    else {
                        hLine[c] = fixedToU16(v32);
                    }

                    gLine[c] = gRow[c]; // 8-bit intensity copy
                }
            }
        }

        if (false) //raw output method
        {
            for (int r = 0; r < rows; ++r) {
                MIL_UINT16* hLine = hPtr + r * pitch16;
                MIL_UINT8* gLine = gPtr + r * pitch8;
                const int* pRow = profPtr + (r * cols);
                const unsigned char* gRow = grayPtr + (r * cols);

                for (int c = 0; c < cols; ++c) {
                    int v32 = pRow[c];

                    if (v32 <= INVALID_SENTINEL) {
                        hLine[c] = 0;   // invalid -> black
                    }
                    {
                        double height_um = static_cast<double>(v32) / 100.0;
                        double gray = height_um + 32768.0;

                        // clamp into 16-bit range
                        if (gray < 0.0)       gray = 0.0;
                        else if (gray > 65535.0) gray = 65535.0;

                        hLine[c] = gray; // round to nearest
                    }

                    gLine[c] = gRow[c]; // 8-bit intensity copy
                }
            }
        }

        //MbufSaveA("TestA.tiff", frame.pHeightMap->id());
        //MbufSaveA("TestA.jpg", frame.pImage-> id());

        g_imageQueue.push_back(frame);

        frame = FrameInfo();

        ct::logger::info("[Profiler_SSZN] Enqueued frame (%dx%d)", cols, rows);
        s_instance->m_softTriggered = false;
        s_instance->m_conditionVariable.notify_one();
    }
    else
    {

        MIL_ID tmpRaw32 = M_NULL;
        MbufAlloc2d(M_DEFAULT, cols, rows, 32 + M_SIGNED, M_IMAGE + M_PROC, &tmpRaw32);
        if (tmpRaw32 != M_NULL)
        {
            // profPtr is rows*cols contiguous int32 data
            MbufPut(tmpRaw32, profPtr);
            //MbufSaveA("SSZN_TestRaw32.tiff", tmpRaw32);
            ct::logger::info("[Profiler_SSZN] Saved raw 32-bit profile to SSZN_TestRaw32.tiff");
            MbufFree(tmpRaw32);
        }
        else
        {
            ct::logger::error("[Profiler_SSZN] Failed to allocate 32-bit raw buffer");
        }

        // --- 2) Build 16-bit height + 8-bit intensity in vectors ---
        std::vector<MIL_UINT16> height16(rows * cols);
        std::vector<MIL_UINT8>  gray8(rows * cols);

        constexpr int INVALID_SENTINEL = -100000000;

        for (int r = 0; r < rows; ++r) {
            const int* pRow = profPtr + (r * cols);
            const unsigned char* gRow = grayPtr + (r * cols);

            for (int c = 0; c < cols; ++c) {
                int v32 = pRow[c];
                MIL_UINT16 out = 0;

                if (v32 > INVALID_SENTINEL) {
                    double height_um = static_cast<double>(v32) * 1000.0;
                    double gray = 32768.0 + height_um;

                    // clamp into 16-bit range
                    if (gray < 0.0)          gray = 0.0;
                    else if (gray > 65535.0) gray = 65535.0;

                    out = static_cast<MIL_UINT16>(gray + 0.5);
                }

                const int idx = r * cols + c;
                height16[idx] = out;
                gray8[idx] = gRow[c];
            }
        }

        // --- 3) Allocate temp MIL buffers for 16-bit and 8-bit images ---
        MIL_ID tmpHeight = M_NULL;
        MIL_ID tmpGray = M_NULL;

        MbufAlloc2d(M_DEFAULT, cols, rows, 16 + M_UNSIGNED, M_IMAGE + M_PROC, &tmpHeight);
        MbufAlloc2d(M_DEFAULT, cols, rows, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &tmpGray);

        if (tmpHeight != M_NULL && tmpGray != M_NULL) {
            MbufPut(tmpHeight, height16.data());
            MbufPut(tmpGray, gray8.data());

            //MbufSaveA("SSZN_TestHeight.tiff", tmpHeight);
            //MbufSaveA("SSZN_TestIntensity.tiff", tmpGray);

            ct::logger::info("[Profiler_SSZN] Saved test images: SSZN_TestHeight.tiff, SSZN_TestIntensity.tiff");
        }
        else {
            ct::logger::error("[Profiler_SSZN] Failed to allocate test MIL buffers (tmpHeight/tmpGray is null)");
        }

        if (tmpHeight != M_NULL) MbufFree(tmpHeight);
        if (tmpGray != M_NULL) MbufFree(tmpGray);
        s_instance->m_softTriggered = false;
        s_instance->m_conditionVariable.notify_one();
    }
}

Profiler_SSZN::Profiler_SSZN()
{
    Profiler_SSZN::s_instance = this;
}

Profiler_SSZN::~Profiler_SSZN()
{
    ct::logger::info("[Profiler_SSZN] Closed successfully");
}

static int comboTextToIndex(const QString& text, const QStringList& opts, int defIdx)
{
    QString s = text.trimmed();

    // exact match
    int idx = opts.indexOf(s);
    if (idx >= 0) return idx;

    // case-insensitive match
    for (int i = 0; i < opts.size(); ++i) {
        if (opts[i].compare(s, Qt::CaseInsensitive) == 0)
            return i;
    }

    // if someone saved "0"/"1"/"2"... as string, accept it
    bool ok = false;
    int asInt = s.toInt(&ok);
    if (ok) return qBound(0, asInt, opts.size() - 1);

    return defIdx;
}

static int comboTextToIntClamped(const QString& text, int defVal, int minV, int maxV)
{
    bool ok = false;
    int v = text.trimmed().toInt(&ok);
    if (!ok) return defVal;
    return qBound(minV, v, maxV);
}


int Profiler_SSZN::getNearestExposureIndex(int us)
{
    static const int exposureTable[] = {
        10, 15, 30, 60, 120, 240, 480, 960, 1920, 2400, 4900, 9800
    };
    constexpr int size = sizeof(exposureTable) / sizeof(exposureTable[0]);

    if (us <= exposureTable[0]) return 0;
    if (us >= exposureTable[size - 1]) return size - 1;

    int bestIdx = 0;
    int bestDiff = std::abs(us - exposureTable[0]);

    for (int i = 1; i < size; ++i)
    {
        int diff = std::abs(us - exposureTable[i]);
        if (diff < bestDiff)
        {
            bestDiff = diff;
            bestIdx = i;
        }
    }
    return bestIdx;
}



//Query
const double Profiler_SSZN::getExposure() const
{
    return m_exposure;
}


const double Profiler_SSZN::getYResolution() const
{
    ct::logger::error("[Profiler_SSZN] Y Resolution not available on SSZN");
    return -1;
}

const QString& Profiler_SSZN::getFirmwareVersion() const
{
    if (!safeGuard()) return "SENSOR_NOT_CONNECTED";

    auto version = SR7IF_GetVersion();
    return QString("%1").arg(version);
}

const QString& Profiler_SSZN::getSerialNumber() const
{
    return m_serialNumber;
}

bool Profiler_SSZN::isConnected() const
{
    if (!safeGuard()) return false;
    if (!m_connectionStatus)
    {
        ct::logger::info("[Profiler_SSZN] Sensor not connected");
    }
    else
    {
        ct::logger::info("[Profiler_SSZN] Sensor is connected");
    }
    return (m_connectionStatus);
}

const bool  Profiler_SSZN::isGrabbing() const
{
    return false;
}

const int  Profiler_SSZN::getImageWidth() const
{
    ct::logger::error("[Profiler_SSZN] getImageWidth not available on SSZN");
    return -1;
}

const int  Profiler_SSZN::getImageHeight() const
{
    ct::logger::error("[Profiler_SSZN] getImageHeight not available on SSZN");
    return -1;
}

bool Profiler_SSZN::enable(bool enable)
{
    m_enable = true;
    return true;
}

bool Profiler_SSZN::connect(QString ip)
{
    const QStringList parts = ip.trimmed().split('.');
    if (parts.size() != 4) {
        ct::logger::error("[Profiler_SSZN] Invalid IP format: %s",
            ip.toUtf8().constData());
        m_connectionStatus = false;
        return false;
    }

    SR7IF_ETHERNET_CONFIG cfg{};
    for (int i = 0; i < 4; ++i) {
        bool ok = false;
        int val = parts[i].toInt(&ok);
        if (!ok || val < 0 || val > 255) {
            ct::logger::error("[Profiler_SSZN] Invalid IP octet: %s (index %d)",
                parts[i].toUtf8().constData(), i);
            m_connectionStatus = false;
            return false;
        }
        cfg.abyIpAddress[i] = static_cast<unsigned char>(val);
    }

    // Open control connection
    const int ret = SR7IF_EthernetOpen(DEVICEID, &cfg);
    if (ret != 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_EthernetOpen failed, ret = %d", ret);
        m_connectionStatus = false;
        return false;
    }

    ct::logger::info("[Profiler_SSZN] Sensor IP: %s", ip.toUtf8().constData());
    m_connectionStatus = true;
    return true;


}

bool Profiler_SSZN::disconnect() {
    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SSZN] Disconnecting Sensor");

    //SR7IF_StopMeasure(DEVICEID);

    //if (SR7IF_SetBatchOneTimeDataHandler(DEVICEID, nullptr) == 0) ct::logger::error("[Profiler_SSZN] Failed to disconnect from callback");

    const int ret = SR7IF_CommClose(DEVICEID);
    if (ret < 0) 
    {
        ct::logger::error("[Profiler_SSZN] Failed to Disconnect Sensor");
        return false;
    }

    m_release = true;

    return true;
}

bool Profiler_SSZN::start() {

    if (!safeGuard())
    {
        ct::logger::error("[Profiler_SSZN] Cannot start: safeGuard() failed.");
        return false;
    }
    m_softTriggered = true;
    ct::logger::info("[Profiler_SSZN] Starting batch acquisition with callback...");
    int target[4] = { 0 };
    int samplingRate;
    int exposure;
    int divider;
    int lightSensitivity;
    int laserUpperLimmit;
    int laserLowerLimmit;
    int lightControl;

    SR7IF_GetSetting(DEVICEID, 0x10, 0x00, 0x02, 0, &samplingRate, 4);
    SR7IF_GetSetting(DEVICEID, 0x10, 0x01, 0x06, target, &exposure, 1);
    SR7IF_GetSetting(DEVICEID, 0x10, 0x00, 0x09, 0, &divider, 2);
    SR7IF_GetSetting(DEVICEID, 0x10, 0x01, 0x05, target, &lightSensitivity, 1);
    SR7IF_GetSetting(DEVICEID, 0x10, 0x01, 0x0B, target, &lightControl, 1);
    SR7IF_GetSetting(DEVICEID, 0x10, 0x01, 0x0C, target, &laserUpperLimmit, 1);
    SR7IF_GetSetting(DEVICEID, 0x10, 0x01, 0x0D, target, &laserLowerLimmit, 1);


    ct::logger::info("[Profiler_SSZN] samplingRate        = %d", samplingRate);
    ct::logger::info("[Profiler_SSZN] exposure            = %d", exposure);
    ct::logger::info("[Profiler_SSZN] divider             = %d", divider);
    ct::logger::info("[Profiler_SSZN] lightSensitivity    = %d", lightSensitivity);
    ct::logger::info("[Profiler_SSZN] lightControl        = %d", lightControl);
    ct::logger::info("[Profiler_SSZN] laserUpperLimmit    = %d", laserUpperLimmit);
    ct::logger::info("[Profiler_SSZN] laserLowerLimmit    = %d", laserLowerLimmit);




    _last.finished = false;
    _last.batchPoints = 0;
    _last.xPoints = 0;
    _last.xPitch = 0.0;

    int ret12 = SR7IF_StartMeasureWithCallback(DEVICEID, 0);
    if (ret12 < 0)
    {
        ct::logger::error("[Profiler_SSZN] SR7IF_StartMeasureWithCallback failed, ret = %d", ret12);
        return false;
    }

    return true;


}

bool Profiler_SSZN::stop()
{
    if (!safeGuard()) return false;
    ct::logger::info("[Profiler_SSZN] Stopping batch acquisition...");

    m_conditionVariable.notify_one();

     //1) Stop batch processing
    int ret = SR7IF_StopMeasure(DEVICEID);
    if (ret < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_StopMeasure failed, ret = %d", ret);
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_StopMeasure success, ret = %d", ret);
    }

    //m_softTriggered = false;

    ct::logger::info("[Profiler_SSZN] Stop completed");
    return true;
}


bool Profiler_SSZN::enableIntensityMap(bool enable)
{

    ct::logger::info("[Profiler_SSZN] enableIntensityMap OK (enable = %d, intensity = %d)");
    return true;
}

bool Profiler_SSZN::setScanLength(double mm)
{
    double reso = 0.004* m_divider;
    if (!safeGuard()) return false;
    double realmm = mm / reso;
    int realmmInt = realmm;
    ct::logger::info("[Profiler_SSZN] Set scan length (mm): %.2f -> %d, %.5f,%d", mm, realmmInt, reso, m_divider);

    int ret = SR7IF_SetSetting( DEVICEID, 1, 0x10, 0x00, 0x0a,0,&realmmInt, 2);
    if (ret<0)
    {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting failed, ret = %d", ret);
        return false;
    }
    else
    {
        ct::logger::info("[Profiler_SSZN] Sensor has set profile length (px): %d", realmmInt);
    }

    return true;
} 

bool Profiler_SSZN::setGain(double gain) 
{
    ct::logger::error("[Profiler_SSZN] setGain not available on SSZN");
    return -1;
}

bool Profiler_SSZN::setDuoHeadGain(double gain, double gain2)
{
    ct::logger::error("[Profiler_SSZN] setDuoHeadGain not available on SSZN");
    return -1;
}

bool Profiler_SSZN::setExposureMode(ExposureMode mode)
{
    ct::logger::error("[Profiler_SSZN] setExposureMode not available on SSZN");
    return -1;
}

bool Profiler_SSZN::setExposure(double us)
{
    ct::logger::info("[Profiler_SSZN] Running set exposure");
    if (!safeGuard()) return false;
    int exposure = getNearestExposureIndex(us);
    int target[4] = { 0 };
    target[0] = 0;
    //int exposure = us;

    unsigned char data = static_cast<unsigned char>(exposure);
    int dataSize = sizeof(data);
    ct::logger::info("[Profiler_SSZN] Exposure : %d", exposure);
    const int ret = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x06, target, &exposure,1);
    if (ret < 0)
    {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting failed, ret = %d", ret);
    }
    else
    {
        ct::logger::info("[Profiler_SSZN] Sensor has set exposure: %d", exposure);
    }

    int laserBrightnessUpper = 90; // 1~99
    int laserBrightnessLower = 90; // 1~99
    loadOptics3DLaserLimits(laserBrightnessLower, laserBrightnessUpper);

    laserBrightnessUpper = std::max(1, std::min(99, laserBrightnessUpper));
    laserBrightnessLower = std::max(1, std::min(99, laserBrightnessLower));

    const int retUpper = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0C, target, &laserBrightnessUpper, 1);
    if (retUpper < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(LaserBrightnessUpper) failed, ret = %d", retUpper);
    }
    ct::logger::info("[Profiler_SSZN] LaserBrightnessUpper set: %d", laserBrightnessUpper);

    const int retLower = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0D, target, &laserBrightnessLower, 1);
    if (retLower < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(LaserBrightnessLower) failed, ret = %d", retLower);
    }
    ct::logger::info("[Profiler_SSZN] LaserBrightnessLower set: %d", laserBrightnessLower);
    int lightSensitivity = 0; // 0..3 (combo index)
    int peakSensitivity = 1; // 1..5 (value)
    int peakSelection = 0; // 0..3 (combo index)

    loadOptics3DLightPeak(lightSensitivity, peakSensitivity, peakSelection);

    const int retLS = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x05, target, &lightSensitivity, 1);
    if (retLS < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(LightSensitivity) failed, ret = %d", retLS);
    }
    ct::logger::info("[Profiler_SSZN] LightSensitivity set: %d", lightSensitivity);

    const int retPS = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0F, target, &peakSensitivity, 1);
    if (retPS < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(PeakSensitivity) failed, ret = %d", retPS);
    }
    ct::logger::info("[Profiler_SSZN] PeakSensitivity set: %d", peakSensitivity);

    const int retSel = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x11, target, &peakSelection, 1);
    if (retSel < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(PeakSelection) failed, ret = %d", retSel);
    }
    ct::logger::info("[Profiler_SSZN] PeakSelection set: %d", peakSelection);
    ct::logger::info("[Profiler_SSZN] Finish running set exposure");
    return true;
}

bool Profiler_SSZN::setMultiExposure(double us, double us2)
{
    ct::logger::error("[Profiler_SSZN] setMultiExposure not available on SSZN");
    return -1;
}

bool Profiler_SSZN::setDynamicExposure(double min_us, double max_us)
{
    ct::logger::error("[Profiler_SSZN] setDynamicExposure not available on SSZN");
    return -1;
}

bool Profiler_SSZN::setParallelExposure(double us, double us2)
{
    ct::logger::error("[Profiler_SSZN] setParallelExposure not available on SSZN");
    return -1;
}

bool Profiler_SSZN::waitAcquisition(int ms)
{
    if (!m_softTriggered)
    {
        ct::logger::error("[Profiler_SSZN] m_softTriggered m_softtriggerd return");
        return true;
    }


    std::unique_lock<std::mutex> lock(m_mutex);

    // Wait until the callback notifies, or until we time out
    if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) ==std::cv_status::no_timeout)
    {
        ct::logger::info("[Profiler_SSZN] done waitAcquisitiion");
        return true;
    }

    m_softTriggered = false;
    return false;
}

bool Profiler_SSZN::setMSR(bool enable)
{
    ct::logger::error("[Profiler_SSZN] setMSR not available on SSZN");
    return -1;
}

bool Profiler_SSZN::setLaserLineThreshold(double threshold)
{
    int target[4] = { 0 };

    int laserBrightnessUpper = 90;
    int laserBrightnessLower = 90;

    // Load from optics (same as in loadConfig)
    loadOptics3DLaserLimits(laserBrightnessLower, laserBrightnessUpper);

    // Apply to device
    const int retUpper = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0C, target, &laserBrightnessUpper, 1);
    const int retLower = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0D, target, &laserBrightnessLower, 1);

    if (retUpper < 0 || retLower < 0) {
        ct::logger::error("[Profiler_SSZN] setLaserLineThreshold failed (retUpper = %d, retLower = %d, upper = %d, lower = %d)", retUpper, retLower, laserBrightnessUpper, laserBrightnessLower);
        return false;
    }

    ct::logger::info("[Profiler_SSZN] setLaserLineThreshold OK (upper = %d, lower = %d)", laserBrightnessUpper, laserBrightnessLower);
    return true;
}

bool Profiler_SSZN::setDivider(int divider)
{
    m_divider = divider;
    const int ret = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x09, 0, &m_divider, 2);

    if (ret < 0) {
        ct::logger::error("[Profiler_SSZN] setDivider failed, ret = %d (divider = %d)", ret, m_divider);
        return false;
    }

    ct::logger::info("[Profiler_SSZN] setDivider OK (divider = %d, ret = %d)", m_divider, ret);
    return true;
}

//Data
const FrameInfo& Profiler_SSZN::getFrame() const
{
    return m_frameInfo;
}

FrameInfo& Profiler_SSZN::getFrame()
{
    return m_frameInfo;
}


void Profiler_SSZN::resetFrame()
{
    m_frameInfo = FrameInfo();
}


bool Profiler_SSZN::loadConfig(QString path)
{
    ct::logger::info("[Profiler_SSZN] loadConfig");


    if (!safeGuard()) return false;
    m_divider = loadOptics3DDivider(1);

    int triggerMode = 2; //0 continuous, 1 external, 2 encoder
    int batchSwitching = 1;
    int samplingFreq = 1500;
    int intensity = 0;
    int encodertype = 1; //0: single ended, 1: differential
    int encoderMode = 1; //0: 1 phase 1 inc, 1: 2 phase 1 inc, 2: 2 phase 2 inc, 3: 2 phase 4 inc
    int triggerInterval = m_divider;
    int lightSensitivity = 0; //0: High precision, 1: High dynamic range1, 2: High dynamic range 2, 3: High dynamic range 3, 4: High Dynamic Range 4, 5: Custom dynamic range
    int lightControl = 1; //0: Auto, 1: Manual
    int laserBrightnessUpper = 90; //1~99
    int laserBrightnessLower = 90; //1~99
    int target[4] = { 0 };
    int peakSensitivity = 1;
    int peakSelection = 0;
    loadOptics3DLaserLimits(laserBrightnessLower, laserBrightnessUpper);
    loadOptics3DLightPeak(lightSensitivity, peakSensitivity, peakSelection, lightSensitivity,peakSensitivity,peakSelection);

    const int ret0 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x01, 0, &triggerMode, 1);
    const int ret1 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x03, 0, &batchSwitching, 1);
    const int ret2 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x02, 0, &samplingFreq, 1);
    const int ret3 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x21, 0, &intensity, 1);
    const int ret5 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x0b, 0, &encodertype, 1);
    const int ret4 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x07, 0, &encoderMode, 1);
    const int ret6 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x09, 0, &triggerInterval, 2);
    const int ret7 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x05, target, &lightSensitivity, 1);
    const int ret8 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0B, target, &lightControl, 1);
    const int ret9 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0C, target, &laserBrightnessUpper, 1);
    const int ret10 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0D, target, &laserBrightnessLower, 1);
    const int ret12 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x0F, target, &peakSensitivity, 1);
    const int ret13 = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x01, 0x11, target, &peakSelection,1);

    bool ok = true;

    if (ret0 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(triggerMode) failed, ret = %d", ret0);
        ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(triggerMode) OK (ret = %d)", ret0);
    }

    if (ret1 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(batchSwitching) failed, ret = %d", ret1);
        ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(batchSwitching) OK (ret = %d)", ret1);
    }

    if (ret2 < 0) {
        //ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(samplingFreq) failed, ret = %d", ret2);
        //ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(samplingFreq) OK (ret = %d)", ret2);
    }

    if (ret3 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(intensity) failed, ret = %d", ret3);
        ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(intensity) OK (ret = %d)", ret3);
    }

    if (ret4 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(encodertype) failed, ret = %d", ret4);
        ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(encodertype) OK (ret = %d)", ret4);
    }

    if (ret5 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(encoderMode) failed, ret = %d", ret5);
        ok = false;
    } else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(encoderMode) OK (ret = %d)", ret5);
    }

    if (ret6 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(triggerInterval) failed, ret = %d", ret6);
        ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(triggerInterval) OK (ret = %d)", ret6);
    }

    if (ret7 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(Light Sensitivity) failed, ret = %d", ret7);
        ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(Light Sensitivity) OK (ret = %d)", ret7);
    }

    if (ret9 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(LaserBrightnessUpper) failed, ret = %d", ret9);
        ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(LaserBrightnessUpper) OK (ret = %d)", ret9);
    }

    if (ret10 < 0) {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetSetting(LaserBrightnessLower) failed, ret = %d", ret10);
        ok = false;
    }
    else {
        ct::logger::info("[Profiler_SSZN] SR7IF_SetSetting(LaserBrightnessLower) OK (ret = %d)", ret10);
    }

    int ret11 = SR7IF_SetBatchOneTimeDataHandler(DEVICEID, BatchOneTimeCallBack);
    if (ret11 < 0)
    {
        ct::logger::error("[Profiler_SSZN] SR7IF_SetBatchOneTimeDataHandler failed, ret = %d", ret11);
        // ok = false;
    }

    // 5) Start measurement with callback



    if (!ok) {
        // At least one setting failed
        return false;
    }
    ct::logger::info("[Profiler_SSZN] Finish running set exposure");
    return true;
}

bool Profiler_SSZN::safeGuard() const
{
    if (!m_enable) {
        ct::logger::warn("Profiler not enabled");
        return false;
    }

    return true;
}

QString Profiler_SSZN::errorMsg()
{
    return m_errorMsg;
}

bool Profiler_SSZN::setTestFlag(bool flag)
{
    testFlag = flag;
    ct::logger::info("[Profiler] Test Flag set to %d", flag);
    return true;
}

int Profiler_SSZN::loadOptics3DDivider(int defaultDivider)
{
    int divider = defaultDivider;

    QString recipePath = Common::Directory::getRecipeCurrentPath();
    QString opticsPath = recipePath + "/optics.json";

    QFile file(opticsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        ct::logger::error(
            "[Profiler_SSZN] Failed to open optics.json at '{}'",
            opticsPath.toStdString()
        );
        return divider;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        ct::logger::error(
            "[Profiler_SSZN] Failed to parse optics.json: {}",
            parseError.errorString().toStdString()
        );
        return divider;
    }

    QJsonObject root = doc.object();
    QJsonArray optics3D = root.value("optics3D").toArray();

    if (optics3D.isEmpty() || !optics3D.first().isObject()) {
        ct::logger::warn(
            "[Profiler_SSZN] optics3D is empty or invalid in optics.json, using default divider = {}",
            divider
        );
        return divider;
    }

    QJsonObject first = optics3D.first().toObject();
    divider = first.value("divider").toInt(defaultDivider);

    ct::logger::info(
        "[Profiler_SSZN] Using optics3D[0].divider = {}",
        divider
    );
    return divider;
}

bool Profiler_SSZN::loadOptics3DLaserLimits(int& lowerLimit, int& upperLimit, int defaultLower, int defaultUpper)
{
    lowerLimit = defaultLower;
    upperLimit = defaultUpper;

    QString recipePath = Common::Directory::getRecipeCurrentPath();
    QString opticsPath = recipePath + "/optics.json";

    QFile file(opticsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        ct::logger::error("[Profiler_SSZN] loadOptics3DLaserLimits: failed to open optics.json");
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        ct::logger::error("[Profiler_SSZN] loadOptics3DLaserLimits: failed to parse optics.json");
        return false;
    }

    QJsonObject root = doc.object();
    QJsonArray optics3D = root.value("optics3D").toArray();

    if (optics3D.isEmpty() || !optics3D.first().isObject()) {
        ct::logger::warn("[Profiler_SSZN] loadOptics3DLaserLimits: optics3D empty or invalid, using default limits");
        return false;
    }

    QJsonObject first = optics3D.first().toObject();

    lowerLimit = first.value("lowerLaserLimit").toInt(defaultLower);
    upperLimit = first.value("upperLaserLimit").toInt(defaultUpper);

    ct::logger::info("[Profiler_SSZN] loadOptics3DLaserLimits: lower=%d upper=%d", lowerLimit, upperLimit);

    return true;
}

bool Profiler_SSZN::snapShot()
{
    //ct::logger::info("[Profiler_SSZN] Starting Snapshot");

    //// 1. Set Flag
    //snapshotFlag = true;

    //// 2. Define Local Variables
    //int triggerMode = 0;   // 0: Continuous (Free Run)
    //int batchPoints = 50;  // Small batch size
    //int target[4] = { 0 };


    //int ret = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x01, target, &triggerMode, 1);
    //if (ret < 0) {
    //    ct::logger::error("[Profiler_SSZN] Snapshot Config Failed: TriggerMode");
    //    return false;
    //}

    //ret = SR7IF_SetSetting(DEVICEID, 0x01, 0x10, 0x00, 0x0A, target, &batchPoints, 2);
    //if (ret < 0) {
    //    ct::logger::error("[Profiler_SSZN] Snapshot Config Failed: BatchPoints");
    //    return false;
    //}

    //m_softTriggered = true;

    //ret = SR7IF_StartMeasureWithCallback(DEVICEID, 0);
    //if (ret < 0) {
    //    ct::logger::error("[Profiler_SSZN] Snapshot StartMeasureWithCallback Failed (ret = %d)", ret);
    //    m_softTriggered = false;
    //    return false;
    //}

    //if (!waitAcquisitionSnapshot(2000)) {
    //    stop();
    //    return false;
    //}
    //stop();

    //ct::logger::info("[Profiler_SSZN] Snapshot Completed");
    return true;

}

bool Profiler_SSZN::waitAcquisitionSnapshot(int ms)
{
    //if (!m_softTriggered) {
    //    ct::logger::warn("[Profiler_SSZN] waitAcquisitionSnapshot called but softTriggered is false");
    //    return true;
    //}

    //std::unique_lock<std::mutex> lock(m_mutex);

    //// Wait for the callback to notify us (just like normal waitAcquisition)
    //if (m_conditionVariable.wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::no_timeout)
    //{
    //    ct::logger::info("[Profiler_SSZN] Snapshot data received via Callback");

    //    // Note: The BatchOneTimeCallBack has already populated m_frameInfo
    //    // and set m_softTriggered = false.
    //    return true;
    //}

    //ct::logger::error("[Profiler_SSZN] Snapshot Timed Out");
    //m_softTriggered = false;
    return false;
}

bool Profiler_SSZN::loadOptics3DLightPeak(int& lightSensitivityIdx,
    int& peakSensitivityVal,
    int& peakSelectionIdx,
    int defLightIdx,
    int defPeakSens,
    int defPeakSel)
{
    // must match your UI texts
    const QStringList kLight = { "High Precision", "HDR 1", "HDR 2", "HDR 3" };
    const QStringList kPeakSel = { "Standard", "Near", "Far", "Invalid" };

    lightSensitivityIdx = defLightIdx;
    peakSensitivityVal = defPeakSens;
    peakSelectionIdx = defPeakSel;

    QString recipePath = Common::Directory::getRecipeCurrentPath();
    QString opticsPath = recipePath + "/optics.json";

    QFile file(opticsPath);
    if (!file.open(QIODevice::ReadOnly)) {
        ct::logger::warn("[Profiler_SSZN] loadOptics3DLightPeak: cannot open optics.json, using defaults");
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(data, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        ct::logger::warn("[Profiler_SSZN] loadOptics3DLightPeak: parse failed, using defaults");
        return false;
    }

    QJsonObject root = doc.object();
    QJsonArray optics3D = root.value("optics3D").toArray();
    if (optics3D.isEmpty() || !optics3D.first().isObject()) {
        ct::logger::warn("[Profiler_SSZN] loadOptics3DLightPeak: optics3D empty/invalid, using defaults");
        return false;
    }

    QJsonObject o = optics3D.first().toObject();

    const QString lightStr = o.value("lightSensitivity").toString("High Precision");
    const QString peakSensStr = o.value("peakSensitivity").toString("5");
    const QString peakSelStr = o.value("peakSelection").toString("Standard");

    lightSensitivityIdx = comboTextToIndex(lightStr, kLight, defLightIdx);          // 0..3
    peakSelectionIdx = comboTextToIndex(peakSelStr, kPeakSel, defPeakSel);       // 0..3
    peakSensitivityVal = comboTextToIntClamped(peakSensStr, defPeakSens, 1, 5);    // 1..5

    ct::logger::info("[Profiler_SSZN] optics3D strings -> ints: light=%d peakSens=%d peakSel=%d",
        lightSensitivityIdx, peakSensitivityVal, peakSelectionIdx);

    return true;
}