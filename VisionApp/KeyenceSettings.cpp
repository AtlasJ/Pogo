#include "KeyenceSettings.h"

#include <QHash>
#include <QMutex>
#include <QMutexLocker>
#include <QRegularExpression>
#include <QStringList>
#include <cmath>

namespace {

    using ljx::Field;
    using ljx::Level;
    using ljx::Option;

    //Fluent builder, so each table row reads as one line of what the manual says about it.
    struct B {
        Field f;

        B(const char* key, const char* group, const char* name, Level level)
        {
            f.key = QLatin1String(key);
            f.group = QLatin1String(group);
            f.name = QLatin1String(name);
            f.level = level;
        }
        B(const QString& key, const char* group, const QString& name, Level level)
        {
            f.key = key;
            f.group = QLatin1String(group);
            f.name = name;
            f.level = level;
        }

        B& tsv(const char* path, int nth = 0, int column = 0)
        {
            f.tsv = QLatin1String(path);
            f.tsvNth = nth;
            f.tsvColumn = column;
            return *this;
        }
        //A program setting: type 10h + program number.
        B& prog(int category, int item, int blockSize = 4, int offset = 0, int size = 1)
        {
            f.common = false;
            f.category = static_cast<unsigned char>(category);
            f.item = static_cast<unsigned char>(item);
            f.blockSize = blockSize;
            f.offset = offset;
            f.size = size;
            return *this;
        }
        //A common measurement setting: type 02h, category 0.
        B& comm(int item, int blockSize = 4, int offset = 0, int size = 1)
        {
            prog(0, item, blockSize, offset, size);
            f.common = true;
            return *this;
        }
        B& target(int t) { f.target1 = static_cast<unsigned char>(t); return *this; }
        B& opts(const QVector<Option>& o)
        {
            f.options = o;
            f.min = INT_MAX;
            f.max = INT_MIN;
            for (const Option& x : o) {
                f.min = qMin(f.min, x.code);
                f.max = qMax(f.max, x.code);
            }
            return *this;
        }
        B& range(int min, int max, double scale = 1.0, const char* unit = "")
        {
            f.min = min;
            f.max = max;
            f.scale = scale;
            f.unit = QLatin1String(unit);
            return *this;
        }
        B& sgn() { f.isSigned = true; return *this; }
        B& req(int code) { f.required = code; return *this; }
        B& fallback(int code) { f.fallback = code; return *this; }
        B& legacy() { f.legacy = true; return *this; }
        B& optic() { f.perOptic = true; return *this; }
        B& note(const char* n) { f.note = QLatin1String(n); return *this; }
        operator Field() const { return f; }
    };

    const QVector<Option> kOffOn = { { 0, "OFF" }, { 1, "ON" } };

    //p.70-71. Navigator writes "1kHz"; the parse lower-cases both sides.
    const QVector<Option> kSampling = {
        { 0, "10Hz" }, { 1, "20Hz" }, { 2, "50Hz" }, { 3, "100Hz" }, { 4, "200Hz" }, { 5, "500Hz" },
        { 6, "1kHz" }, { 11, "1.5kHz" }, { 7, "2kHz" }, { 12, "2.5kHz" }, { 13, "3kHz" }, { 14, "3.5kHz" },
        { 8, "4kHz" }, { 15, "4.5kHz" }, { 16, "5kHz" }, { 17, "6kHz" }, { 18, "7kHz" }, { 9, "8kHz" },
        { 20, "10kHz" }, { 19, "12kHz" }, { 21, "14kHz" }, { 10, "16kHz" } };

    //p.74. Same table as Profiler_Keyence's kExposureUs; Navigator writes the microseconds.
    const QVector<Option> kExposure = {
        { 0, "15us" }, { 1, "30us" }, { 2, "60us" }, { 10, "80us" }, { 3, "120us" }, { 11, "160us" },
        { 12, "210us" }, { 4, "240us" }, { 13, "320us" }, { 14, "380us" }, { 5, "480us" }, { 15, "640us" },
        { 6, "960us" }, { 7, "1700us|1.7ms" }, { 8, "4800us|4.8ms" }, { 9, "9600us|9.6ms" } };

    //p.73. Only the rows the manual gives the SAME code for both LJ-X head groups; the LJ-X8060 is
    //in one of them but the single-column rows do not say which. A read-back of any other code shows
    //as "code N" instead of a guess.
    const QVector<Option> kZRange = {
        { 0, "FULL" }, { 6, "1/1.5" }, { 1, "1/2" }, { 7, "1/2.5" }, { 8, "1/3" }, { 9, "1/3.5" },
        { 2, "1/4" }, { 10, "1/4.5" }, { 11, "1/5" }, { 12, "1/6" }, { 13, "1/7" }, { 3, "1/8" },
        { 15, "1/10" }, { 16, "1/12" }, { 4, "1/16" } };

    const char* const kMaskNames[6] = {
        "Upper mask 1", "Upper mask 2", "Upper mask 3", "Lower mask 1", "Lower mask 2", "Lower mask 3" };
    const char* const kMaskKeys[6] = { "maskUpper1", "maskUpper2", "maskUpper3", "maskLower1", "maskLower2", "maskLower3" };

    //0.01 um units, shown in mm.
    constexpr double kMm = 0.00001;

    QVector<Field> buildFields()
    {
        QVector<Field> v;

        //--- Common measurement settings (type 02h), p.69-70
        v << B("headModel", "Common", "Head", Level::Info).tsv("Head details|Head sel")
                .note("Read from the head at Connect");
        v << B("luminanceOutput", "Common", "Luminance output", Level::Pogo).legacy().comm(0x0B)
                .opts({ { 0, "OFF|Height data only" }, { 1, "ON|Height + luminance data" } })
                .note("Pogo turns it on for every scan");
        v << B("memoryAllocation", "Common", "Memory assignment", Level::Check)
                .tsv("Memory setting|Memory assignment").comm(0x01)
                .opts({ { 0, "Double buffer" }, { 1, "Single buffer|All areas" } }).req(1)
                .note("Pogo's batch limit (42,000 points) assumes Single buffer");
        v << B("fullMemoryOperation", "Common", "Operation at FULL memory", Level::Machine)
                .tsv("Memory setting|Operation at FULL memory").comm(0x02)
                .opts({ { 0, "Overwrite" }, { 1, "Stop" } });
        v << B("trgMinInputTime", "Common", "Minimum input time: TRG", Level::Machine)
                .tsv("Minimum input time|TRG").comm(0x06)
                .opts({ { 0, "7us" }, { 1, "10us" }, { 2, "20us" }, { 3, "50us" }, { 4, "100us" },
                        { 5, "200us" }, { 6, "500us" }, { 7, "1ms" } });
        v << B("encoderMinInputTime", "Common", "Minimum input time: ENCODER", Level::Pogo).legacy().comm(0x07)
                .opts({ { 0, "120ns" }, { 1, "150ns" }, { 2, "250ns" }, { 3, "500ns" }, { 4, "1us" },
                        { 5, "2us" }, { 6, "5us" }, { 7, "10us" }, { 8, "20us" } })
                .note("Profiler Hardware: Encoder Min Input Time");
        v << B("controlMinInputTime", "Common", "Minimum input time: Control terminal", Level::Machine)
                .tsv("Minimum input time|Control terminal").comm(0x08)
                .opts({ { 0, "250us" }, { 1, "1ms" } });
        v << B("displayUnit", "Common", "Display unit", Level::Info).tsv("Others|Display unit")
                .note("Navigator's display only - not a controller setting");

        //--- Trigger settings (category 00h), p.70-72
        v << B("triggerMode", "Trigger", "Trigger mode", Level::Pogo).legacy().prog(0x00, 0x01)
                .opts({ { 0, "Continuous trigger" }, { 1, "External trigger" }, { 2, "Encoder trigger" } })
                .note("Always Encoder - keyence.json triggerMode");
        v << B("samplingCycle", "Trigger", "Sampling frequency", Level::Machine).legacy()
                .tsv("Trigger setting|Sample Freq (Trigg. Freq.)").prog(0x00, 0x02).opts(kSampling)
                .note("Caps scan speed: frequency x Y pitch x divider. Not set = the controller keeps its own");
        v << B("batchMeasurement", "Trigger", "Batch measurement", Level::Pogo).legacy().prog(0x00, 0x03)
                .opts(kOffOn).note("Always ON - the batch callback needs it");
        v << B("batchCount", "Trigger", "Batch point", Level::Pogo).legacy().prog(0x00, 0x0A, 4, 0, 2)
                .range(50, 60000).note("Set from the scan length on every scan");
        v << B("interTriggerPitch", "Trigger", "Pitch between triggers", Level::Check)
                .tsv("Trigger setting|Pitch between triggers", 0).prog(0x00, 0x04)
                .opts({ { 0, "Do not set|OFF" }, { 1, "Set|ON" } }).req(0)
                .note("Pogo's Y pitch assumes one profile per encoder trigger (after skipping)");
        v << B("interTriggerPitchValue", "Trigger", "Pitch between triggers: value", Level::Info)
                .tsv("Trigger setting|Pitch between triggers", 1).prog(0x00, 0x05, 4, 0, 4)
                .range(1, 500000, 0.0001, "mm");
        v << B("encoderInputMode", "Trigger", "Encoder input mode", Level::Pogo).legacy().prog(0x00, 0x07)
                .opts({ { 0, "1-phase 1 time|1-phase 1 TM (no dir.)" }, { 1, "2-phase 1 time" },
                        { 2, "2-phase 2 times" }, { 3, "2-phase 4 times" } })
                .note("Profiler Hardware: Encoder Input Mode");
        v << B("subSampling", "Trigger", "Skipping", Level::Pogo).legacy().optic().prog(0x00, 0x08)
                .opts(kOffOn).note("From the optic's Divider - 1 = OFF");
        v << B("subSampleCount", "Trigger", "Points of skipping", Level::Pogo).legacy().optic()
                .prog(0x00, 0x09, 4, 0, 2).range(2, 1000).note("= the optic's Divider");
        v << B("multiControllerSync", "Trigger", "Several controller sync", Level::Check)
                .tsv("Multiple Controller Sync|Several controller Sync function").prog(0x00, 0x0B)
                .opts({ { 0, "No use|OFF" }, { 1, "Master|Synchronization master" }, { 2, "Slave|Synchronization slave" } })
                .req(0).note("Pogo drives one controller on its own");
        v << B("mutualInterference", "Trigger", "Prevention of mutual interference", Level::Check)
                .tsv("Prevention of mutual interference|Prevention of mutual interference").prog(0x00, 0x06, 4, 0, 1)
                .opts({ { 0, "OFF" }, { 1, "AB-ON" }, { 2, "ABC-ON" } }).req(0);
        v << B("slaveEmissionGroup", "Trigger", "Slave emission group", Level::Info)
                .tsv("Prevention of mutual interference|Slave Emission Group").prog(0x00, 0x06, 4, 1, 1)
                .opts({ { 0, "Group A|A" }, { 1, "Group B|B" }, { 2, "Group C|C" } });

        //--- Imaging settings (category 01h), p.72-76
        //The manual numbers these two 1-4 instead of 0-3; a 4-byte item has no byte 4, so the
        //value is the first byte like every other item.
        v << B("advancedSettings", "Imaging", "Measurement range: advanced settings", Level::Check)
                .tsv("Measurement range|Advanced settings").prog(0x01, 0x00).opts(kOffOn).req(0)
                .note("Pogo's X pitch (5.0 um) and FOV (16 mm) assume the full 3200-point profile");
        v << B("xRange", "Imaging", "Measurement range X", Level::Check)
                .tsv("X direction|Measurement range").prog(0x01, 0x02, 4, 0, 1)
                .opts({ { 0, "FULL" }, { 1, "3/4" }, { 2, "1/2" }, { 3, "1/4" }, { 4, "23/32" },
                        { 5, "19/32" }, { 6, "18/32" }, { 7, "12/32" } }).req(0)
                .note("Pogo's X pitch (5.0 um) and FOV (16 mm) assume FULL");
        v << B("xCenter", "Imaging", "Measurement range X: center", Level::Info)
                .tsv("Center of measurement range|X direction").prog(0x01, 0x02, 4, 2, 2).range(0, 1000, 0.1, "%");
        v << B("binningX", "Imaging", "Binning (X)", Level::Check)
                .tsv("X direction|Binning (X)").prog(0x01, 0x04).opts(kOffOn).req(0)
                .note("Binning halves the point count - Pogo's X pitch assumes OFF");
        v << B("zRange", "Imaging", "Measurement range Z", Level::Optic)
                .tsv("Z direction|Measurement range").prog(0x01, 0x03, 8, 0, 1).opts(kZRange)
                .note("A smaller range allows a faster sampling frequency");
        v << B("zCenter", "Imaging", "Measurement range Z: center", Level::Optic)
                .tsv("Center of measurement range|Z direction").prog(0x01, 0x03, 8, 2, 2).range(0, 1000, 0.1, "%");
        v << B("dynamicRange", "Imaging", "CMOS Dynamic Range", Level::Pogo).legacy().optic().prog(0x01, 0x05)
                .range(1, 9).note("3D Optics: Dynamic Range");
        v << B("exposureTime", "Imaging", "Exposure time", Level::Pogo).legacy().optic().prog(0x01, 0x06)
                .opts(kExposure).note("3D Optics: Exposure");
        v << B("exposureMode", "Imaging", "Exposure mode", Level::Pogo).legacy().optic().prog(0x01, 0x07)
                .opts({ { 0, "Standard" }, { 1, "Multi emission (synthesis)" }, { 2, "Multi emission (optimized light intensity)" } })
                .note("3D Optics: Exposure Mode - only Standard on this head");
        v << B("emissionsOptimized", "Imaging", "Emission times (optimized)", Level::Info).prog(0x01, 0x08)
                .opts({ { 0, "2" }, { 1, "4" } }).note("Multi emission only - Pogo uses Standard");
        v << B("emissionsSynthesis", "Imaging", "Emission times (synthesis)", Level::Info).prog(0x01, 0x09)
                .opts({ { 0, "3" }, { 1, "5" } }).note("Multi emission only - Pogo uses Standard");
        v << B("lightControlMode", "Imaging", "Control light intensity: mode", Level::Pogo).legacy().prog(0x01, 0x0B)
                .opts({ { 0, "AUTO" }, { 1, "MANUAL" } }).note("Profiler Hardware: Laser Intensity Control");
        v << B("lightUpperLimit", "Imaging", "Control light intensity: upper limit", Level::Pogo).legacy().optic()
                .prog(0x01, 0x0C).range(1, 99).note("3D Optics: UpperLimit");
        v << B("lightLowerLimit", "Imaging", "Control light intensity: lower limit", Level::Pogo).legacy().optic()
                .prog(0x01, 0x0D).range(1, 99).note("3D Optics: LowerLimit");
        v << B("lightTargetStart", "Imaging", "Control light intensity: target area left", Level::Optic)
                .tsv("Target Area|Left").prog(0x01, 0x0E, 4, 0, 2).range(0, 1000, 0.1, "%");
        v << B("lightTargetEnd", "Imaging", "Control light intensity: target area right", Level::Optic)
                .tsv("Target Area|Right").prog(0x01, 0x0E, 4, 2, 2).range(0, 1000, 0.1, "%");
        v << B("peakSensitivity", "Imaging", "Peak detection sensitivity", Level::Pogo).legacy().optic()
                .prog(0x01, 0x0F).range(1, 5).note("3D Optics: Peak Sensitivity");
        v << B("invalidInterpolation", "Imaging", "Interpolated points of invalid data", Level::Machine).legacy()
                .prog(0x01, 0x10).range(0, 255).fallback(0)
                .note("keyence.json invalidInterpolation - sent at Connect, 0 when the file does not set it");
        v << B("peakSelection", "Imaging", "Multiple peak: detection mode", Level::Pogo).legacy().optic()
                .prog(0x01, 0x11)
                .opts({ { 0, "Standard|Standard (maximum peak)" }, { 1, "NEAR" }, { 2, "FAR" },
                        { 3, "Make invalid data" }, { 4, "Make invalid data (4)" }, { 5, "Make invalid data (5)" } })
                .note("3D Optics: Peak Selection");
        v << B("peakWidthFilter", "Imaging", "Multiple peak: Blurred Light Filter", Level::Optic)
                .tsv("Multiple peak processing|Blurred Light Filter").prog(0x01, 0x12, 4, 0, 1).opts(kOffOn)
                .note("The SDK calls it the peak width filter");
        v << B("peakWidthStrength", "Imaging", "Multiple peak: Blurred Light Filter intensity", Level::Optic)
                .tsv("Blurred Light Filter|(Intensity)").prog(0x01, 0x12, 4, 1, 1).range(1, 5);
        v << B("straySuppression", "Imaging", "Multiple peak: Irregular Reflection Removal Filter", Level::Optic)
                .tsv("Multiple peak processing|Irregular Reflection Removal Filter").prog(0x01, 0x14, 4, 0, 1).opts(kOffOn)
                .note("The SDK calls it stray light suppression");
        v << B("straySuppressionStrength", "Imaging", "Multiple peak: Irregular Reflection Removal intensity", Level::Optic)
                .tsv("Irregular Reflection Removal Filter|(Intensity)").prog(0x01, 0x14, 4, 1, 1)
                .opts({ { 1, "Low|1|Weak" }, { 2, "Middle|Mid|Medium|2" }, { 3, "High|3|Strong" } });

        //Image masks: one item per mask (target 1 = mask number), p.75
        for (int m = 0; m < 6; ++m) {
            const QString k = QLatin1String(kMaskKeys[m]);
            const QString n = QLatin1String(kMaskNames[m]);
            v << B(k + QStringLiteral("Shape"), "Image masks", n + QStringLiteral(": shape"), Level::Optic)
                    .tsv("Image mask|Shape", 0, m).prog(0x01, 0x0A, 16, 0, 1).target(m)
                    .opts({ { 0, "No setting|Disabled|OFF" }, { 1, "Rectangle" }, { 2, "Triangle" } });
            static const char* const coord[6] = { "X1", "Z1", "X2", "Z2", "X3", "Z3" };
            for (int c = 0; c < 6; ++c) {
                v << B(k + QLatin1String(coord[c]), "Image masks", n + QStringLiteral(": ") + QLatin1String(coord[c]), Level::Optic)
                        .prog(0x01, 0x0A, 16, 4 + 2 * c, 2).target(m).range(0, 1000, 0.1, "%");
            }
        }

        //--- Enhanced imaging (category 03h), p.78
        v << B("translucent", "Imaging", "Peak process: translucent peak processing", Level::Optic)
                .tsv("Peak process|Translucent peak processing").prog(0x03, 0x01, 24, 0, 1).opts(kOffOn);
        v << B("translucentEdge", "Imaging", "Translucent: edge detection method", Level::Optic)
                .prog(0x03, 0x01, 24, 1, 1).opts({ { 0, "Maximum change" }, { 1, "NEAR" } });
        v << B("translucentThreshold", "Imaging", "Translucent: edge detection threshold", Level::Optic)
                .prog(0x03, 0x01, 24, 2, 2).range(0, 1000, 0.1, "%");

        //--- Profile settings (category 02h), p.76-78
        v << B("xSubSampling", "Profile", "Skipping (X-axis)", Level::Check)
                .tsv("Skipping (X-axis)").prog(0x02, 0x02).opts({ { 0, "OFF" }, { 1, "1/2" }, { 2, "1/4" } }).req(0)
                .note("Changes the point count - Pogo's X pitch assumes OFF");
        v << B("deadZoneInterpolation", "Profile", "Dead zone interpolation", Level::Optic)
                .tsv("Filter process|Dead Zone Interpolation").prog(0x02, 0x05)
                .opts({ { 0, "No interpolation|OFF" }, { 1, "Linear interpolation|Linear" }, { 2, "Vertical interpolation|Vertical" } });
        v << B("medianX", "Profile", "Filter (X-axis): median", Level::Optic)
                .tsv("Filter process (X-axis)|Median").prog(0x02, 0x0A)
                .opts({ { 0, "OFF" }, { 1, "3" }, { 2, "5" }, { 3, "7" }, { 4, "9" }, { 5, "17" }, { 6, "25" }, { 7, "33" } });
        v << B("smoothingCount", "Profile", "Filter (X-axis): smoothing count", Level::Optic)
                .tsv("Smoothing|No. of smoothing").prog(0x02, 0x0B, 4, 0, 1)
                .opts({ { 0, "1" }, { 1, "2" }, { 2, "4" }, { 3, "8" }, { 4, "16" }, { 5, "32" }, { 6, "64" } });
        v << B("smoothingPreserveShape", "Profile", "Filter (X-axis): smoothing preserve shape", Level::Optic)
                .tsv("Smoothing|Preserve Shape").prog(0x02, 0x0B, 4, 1, 1).opts(kOffOn);
        v << B("medianTime", "Profile", "Filter (time axis): median", Level::Optic)
                .tsv("Filter process (time axis)|Median").prog(0x02, 0x0C)
                .opts({ { 0, "OFF" }, { 1, "3" }, { 2, "5" }, { 3, "7" }, { 4, "9" } });
        v << B("averaging", "Profile", "Filter (time axis): averaging", Level::Optic)
                .tsv("Filter process (time axis)|Averaging").prog(0x02, 0x0D)
                .opts({ { 0, "1" }, { 1, "2" }, { 2, "4" }, { 3, "8" }, { 4, "16" }, { 5, "32" },
                        { 6, "64" }, { 7, "128" }, { 8, "256" } });
        v << B("invalidProcessCount", "Profile", "Invalid data process: processing times", Level::Optic)
                .tsv("Invalid data process|Processing times").prog(0x02, 0x0E, 4, 0, 1).range(0, 255);
        v << B("invalidResumeCount", "Profile", "Invalid data process: resume times", Level::Optic)
                .tsv("Invalid data process|Resume times").prog(0x02, 0x0E, 4, 1, 1).range(0, 255);

        //Mounting - the head's installation, so machine level
        v << B("reverseX", "Mounting", "Profile adjustment: invert X", Level::Machine)
                .tsv("X direction|Invert").prog(0x02, 0x06).opts(kOffOn)
                .note("profiler.json InvertX flips the image in software as well - do not use both");
        v << B("reverseZ", "Mounting", "Profile adjustment: invert Z", Level::Machine)
                .tsv("Z direction|Invert").prog(0x02, 0x07).opts(kOffOn);
        v << B("shiftX", "Mounting", "Profile adjustment: shift X", Level::Machine)
                .tsv("X direction|Shift").prog(0x02, 0x08, 4, 0, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("shiftZ", "Mounting", "Profile adjustment: shift Z", Level::Machine)
                .tsv("Z direction|Shift").prog(0x02, 0x09, 4, 0, 4).sgn().range(-100000000, 100000000, kMm, "mm")
                .note("Moves every height - the 3D datum sees it");

        //Tilt / height correction, 28-byte items, p.78
        v << B("tiltCorrection", "Mounting", "Tilt correction", Level::Optic)
                .tsv("Installation correction|Tilt correction").prog(0x02, 0x0F, 28, 0, 1).opts(kOffOn);
        v << B("tiltAreaCount", "Mounting", "Tilt correction: areas", Level::Optic)
                .prog(0x02, 0x0F, 28, 4, 1).opts({ { 0, "Area 1" }, { 1, "Areas 1 and 2" } });
        v << B("tiltArea1Start", "Mounting", "Tilt correction: area 1 left", Level::Optic)
                .tsv("Tilt calculation area1|Left").prog(0x02, 0x0F, 28, 8, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("tiltArea1End", "Mounting", "Tilt correction: area 1 right", Level::Optic)
                .tsv("Tilt calculation area1|Right").prog(0x02, 0x0F, 28, 12, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("tiltArea2Start", "Mounting", "Tilt correction: area 2 left", Level::Optic)
                .tsv("Tilt calculation area2|Left").prog(0x02, 0x0F, 28, 16, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("tiltArea2End", "Mounting", "Tilt correction: area 2 right", Level::Optic)
                .tsv("Tilt calculation area2|Right").prog(0x02, 0x0F, 28, 20, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("tiltPostAngle", "Mounting", "Tilt correction: corrected angle", Level::Optic)
                .tsv("Tilt correction|Corrected angle").prog(0x02, 0x0F, 28, 24, 2).sgn().range(-4500, 4500, 0.01, "deg");
        v << B("tiltAngle", "Mounting", "Tilt correction: calculated angle", Level::Optic)
                .tsv("Tilt correction|Calculated cor. angle").prog(0x02, 0x0F, 28, 26, 2).sgn().range(-4500, 4500, 0.01, "deg");

        v << B("heightCorrection", "Mounting", "Height correction", Level::Optic)
                .tsv("Installation correction|Height correction").prog(0x02, 0x10, 28, 0, 1).opts(kOffOn);
        v << B("heightArea1Start", "Mounting", "Height correction: area 1 left", Level::Optic)
                .tsv("Height calculation area1|Left").prog(0x02, 0x10, 28, 4, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("heightArea1End", "Mounting", "Height correction: area 1 right", Level::Optic)
                .tsv("Height calculation area1|Right").prog(0x02, 0x10, 28, 8, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("heightArea2Start", "Mounting", "Height correction: area 2 left", Level::Optic)
                .tsv("Height calculation area2|Left").prog(0x02, 0x10, 28, 12, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("heightArea2End", "Mounting", "Height correction: area 2 right", Level::Optic)
                .tsv("Height calculation area2|Right").prog(0x02, 0x10, 28, 16, 4).sgn().range(-100000000, 100000000, kMm, "mm");
        v << B("heightPost", "Mounting", "Height correction: corrected height", Level::Optic)
                .tsv("Height correction|Corrected height").prog(0x02, 0x10, 28, 20, 4).range(0, 99999900, kMm, "mm");
        v << B("heightSpan", "Mounting", "Height correction: calculated span", Level::Optic)
                .tsv("Height correction|Calculated cor. Span").prog(0x02, 0x10, 28, 24, 4).range(0, 131072, 1.0 / 65536.0, "");

        //--- Image mask position offset correction (category 04h), p.79. Needs a baseline profile
        //--- that only Navigator can teach, so it stays Navigator's.
        v << B("maskCorrection", "Image masks", "Image mask position correction", Level::Info)
                .tsv("Image Mask Pos. Correction|Correction type").prog(0x04, 0x01, 32, 0, 1)
                .opts({ { 0, "OFF" }, { 1, "Z correction" }, { 2, "X correction" }, { 3, "X, Z parallel correction" },
                        { 4, "X->Z correction" }, { 5, "Z->X correction" }, { 6, "Feature point correction" } })
                .note("Needs a baseline profile taught in Navigator - left to Navigator");

        return v;
    }

    QString norm(QString s)
    {
        s.replace(QChar(0x03BC), QLatin1Char('u'));     //Greek mu, as Navigator writes it
        s.replace(QChar(0x00B5), QLatin1Char('u'));     //micro sign
        s.remove(QLatin1Char(' '));
        if (s.startsWith(QLatin1Char('\''))) s.remove(0, 1);   //Navigator's text marker: "'1/2"
        return s.trimmed().toLower();
    }

    QMutex g_storeMutex;
    QHash<QString, QJsonObject> g_store;

} // namespace

namespace ljx {

    const QVector<Field>& fields()
    {
        static const QVector<Field> v = buildFields();
        return v;
    }

    const Field* field(const QString& key)
    {
        for (const Field& f : fields())
            if (f.key == key) return &f;
        return nullptr;
    }

    QString display(const Field& f, int code)
    {
        if (code == kNotSet) return QString();
        if (!f.options.isEmpty()) {
            for (const Option& o : f.options)
                if (o.code == code) return QString::fromLatin1(o.label).section(QLatin1Char('|'), 0, 0);
            return QStringLiteral("code %1").arg(code);
        }
        if (f.scale == 1.0) return QString::number(code) + f.unit;

        int decimals = 0;
        for (double s = f.scale; s < 0.999999 && decimals < 5; s *= 10.0) ++decimals;
        return QString::number(code * f.scale, 'f', decimals) + f.unit;
    }

    bool valid(const Field& f, int code)
    {
        if (!f.options.isEmpty()) {
            for (const Option& o : f.options)
                if (o.code == code) return true;
            return false;
        }
        return code >= f.min && code <= f.max;
    }

    bool parse(const Field& f, const QString& text, int* code)
    {
        const QString t = norm(text);
        if (t.isEmpty() || t == QLatin1String("---")) return false;

        if (!f.options.isEmpty()) {
            for (const Option& o : f.options) {
                for (const QString& alias : QString::fromLatin1(o.label).split(QLatin1Char('|'))) {
                    if (norm(alias) == t) {
                        *code = o.code;
                        return true;
                    }
                }
            }
            return false;
        }

        //A number, optionally followed by its unit ("50.0%", "0.000mm", "1.5deg").
        static const QRegularExpression re(QStringLiteral("^([-+]?[0-9]*\\.?[0-9]+)([a-z%]*)$"));
        const QRegularExpressionMatch m = re.match(t);
        if (!m.hasMatch()) return false;

        //Range-checked as a double first: a huge number in a hand-edited file must not overflow the
        //conversion to int.
        const double d = m.captured(1).toDouble() / f.scale;
        if (!(d >= f.min - 0.5 && d <= f.max + 0.5)) return false;
        const int c = static_cast<int>(std::lround(d));
        if (c < f.min || c > f.max) return false;
        *code = c;
        return true;
    }

    void writeField(QByteArray& block, const Field& f, int code)
    {
        if (f.offset < 0 || f.offset + f.size > block.size()) return;
        const quint32 u = static_cast<quint32>(code);
        for (int i = 0; i < f.size; ++i)
            block[f.offset + i] = static_cast<char>((u >> (8 * i)) & 0xFF);
    }

    int readField(const QByteArray& block, const Field& f)
    {
        if (f.offset < 0 || f.offset + f.size > block.size()) return kNotSet;
        quint32 u = 0;
        for (int i = 0; i < f.size; ++i)
            u |= quint32(static_cast<unsigned char>(block[f.offset + i])) << (8 * i);
        if (f.isSigned && f.size < 4) {
            const quint32 sign = 1u << (8 * f.size - 1);
            if (u & sign) u |= ~((sign << 1) - 1);
        }
        return static_cast<int>(u);
    }

    QJsonObject opticSettings(const QString& opticId)
    {
        QMutexLocker lock(&g_storeMutex);
        return g_store.value(opticId);
    }

    void setOpticSettings(const QString& opticId, const QJsonObject& values)
    {
        QMutexLocker lock(&g_storeMutex);
        if (values.isEmpty()) g_store.remove(opticId);
        else g_store.insert(opticId, values);
    }

    void removeOpticSettings(const QString& opticId)
    {
        QMutexLocker lock(&g_storeMutex);
        g_store.remove(opticId);
    }

    void clearOpticSettings()
    {
        QMutexLocker lock(&g_storeMutex);
        g_store.clear();
    }

} // namespace ljx
