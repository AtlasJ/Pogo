#pragma once

#include <QImage>
#include <QString>
#include <QStringList>

/*
* A "side view" of one raw LJ-X batch: every measured point placed by its height, seen once from
* across the laser line and once along the scan, drawn against the head's measuring range and the Z
* window the controller was set to. Made for the Production Scan Check, so the Z margin of a real
* part can be seen at a glance before a narrower Z range (and the faster sampling it allows) is
* trusted.
*
* Pure Qt (QImage + QPainter on an image), so it runs on JobThread and can be tried offline on a
* saved raw_heightmap.tiff.
*/
namespace sideview {

    struct Input {
        const quint16* data = nullptr;  //raw batch in grey levels; 0 = no measurement
        int width = 0;                  //points per profile (across the laser line)
        int height = 0;                 //profiles (along the scan)
        int strideWords = 0;            //quint16s from one row to the next
        double zPitchUm = 0.0;          //um per grey level, about grey 32768
        double xPitchUm = 0.0;          //across the line
        double yPitchUm = 0.0;          //along the scan, per profile
        double headRangeMm = 0.0;       //the head's full scale, centred on 0; 0 = unknown

        bool haveWindow = false;        //Z window known (Z range + centre)
        double windowLoMm = 0.0, windowHiMm = 0.0, centreMm = 0.0;
        QString windowLabel;            //e.g. "Z range 1/2, centre 50.0% (read from the controller)"

        QStringList titleLines;
    };

    struct Result {
        bool ok = false;
        QImage image;
        quint64 valid = 0, total = 0;
        double minMm = 0.0, maxMm = 0.0;         //extremes of the valid points
        double lowMm = 0.0, highMm = 0.0;        //0.1st and 99.9th percentile
        quint64 nearLowEdge = 0, nearHighEdge = 0;   //valid points within 0.1 mm of a window edge
    };

    Result render(const Input& in);

} // namespace sideview
