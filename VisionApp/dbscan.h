#ifndef DBSCAN_H
#define DBSCAN_H

#include <QVector>
#include <QPointF>
#include "VisionApp.h"
#include <QImage>
#include <QColor>

class DBSCAN {
public:
    static constexpr int NOISE = -1;
    static constexpr int UNVISITED = 0;

    // Run the clustering
    static void fit(const QVector<QPointF>& pts,
        double eps,
        int minPts,
        QVector<int>& labels);

    // Render into an image, auto-scaling + margin to fit all points
    // margin = number of pixels of padding around the data
    static QImage renderScaled(const QVector<QPointF>& pts,
        const QVector<int>& labels,
        double pxPerUnit,
        int margin = 20);

    static QVector<QPointF> extractLargestCluster(const QVector<QPointF>& pts, const QVector<int>& labels);

private:
    static QVector<int> regionQuery(const QVector<QPointF>& pts,
        int idx,
        double eps);

    static void expandCluster(const QVector<QPointF>& pts,
        QVector<int>& labels,
        int idx,
        QVector<int>& seedSet,
        int clusterId,
        double eps,
        int minPts);
};
#endif 