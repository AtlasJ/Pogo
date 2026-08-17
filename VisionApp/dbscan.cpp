#include "dbscan.h"
#include <QPainter>
#include <QMap>
#include <QSet>
#include <algorithm>
#include <limits>

void DBSCAN::fit(const QVector<QPointF>& pts,
    double eps,
    int minPts,
    QVector<int>& labels)
{
    int n = pts.size();
    labels.fill(UNVISITED, n);
    int clusterId = 1;
    for (int i = 0; i < n; ++i) {
        if (labels[i] != UNVISITED) continue;
        auto seeds = regionQuery(pts, i, eps);
        if (seeds.size() < minPts) {
            labels[i] = NOISE;
        }
        else {
            expandCluster(pts, labels, i, seeds, clusterId, eps, minPts);
            ++clusterId;
        }
    }
}

QVector<int> DBSCAN::regionQuery(const QVector<QPointF>& pts,
    int idx,
    double eps)
{
    QVector<int> neighbors;
    double eps2 = eps * eps;
    const auto& p = pts[idx];
    for (int j = 0; j < pts.size(); ++j) {
        QPointF d = pts[j] - p;
        if (d.x() * d.x() + d.y() * d.y() <= eps2)
            neighbors.append(j);
    }
    return neighbors;
}

void DBSCAN::expandCluster(const QVector<QPointF>& pts,
    QVector<int>& labels,
    int idx,
    QVector<int>& seedSet,
    int clusterId,
    double eps,
    int minPts)
{
    labels[idx] = clusterId;
    for (int i = 0; i < seedSet.size(); ++i) {
        int j = seedSet[i];
        if (labels[j] == NOISE)
            labels[j] = clusterId;
        if (labels[j] == UNVISITED) {
            labels[j] = clusterId;
            auto more = regionQuery(pts, j, eps);
            if (more.size() >= minPts)
                seedSet += more;
        }
    }
}

// ——— auto-scaling rendering ———
QImage DBSCAN::renderScaled(const QVector<QPointF>& pts,
    const QVector<int>& labels,
    double pxPerUnit,
    int margin)
{
    // 1) compute data bounds
    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    for (const auto& p : pts) {
        minX = std::min(minX, p.x());
        maxX = std::max(maxX, p.x());
        minY = std::min(minY, p.y());
        maxY = std::max(maxY, p.y());
    }

    // 2) compute spans in raw units
    double spanX = (maxX - minX) > 0 ? (maxX - minX) : 1.0;
    double spanY = (maxY - minY) > 0 ? (maxY - minY) : 1.0;

    // 3) image size in pixels
    int spanXp = static_cast<int>(std::ceil(spanX * pxPerUnit));
    int spanYp = static_cast<int>(std::ceil(spanY * pxPerUnit));
    int imgW = spanXp + 2 * margin;
    int imgH = spanYp + 2 * margin;

    // 4) prepare canvas
    QImage img(imgW, imgH, QImage::Format_RGB32);
    img.fill(Qt::white);
    QPainter painter(&img);
    painter.setRenderHint(QPainter::Antialiasing);

    // 5) assign each cluster a distinct HSV hue
    QSet<int> ids;
    for (int c : labels) if (c != NOISE) ids.insert(c);
    QVector<int> cvec = ids.values().toVector();
    std::sort(cvec.begin(), cvec.end());
    QMap<int, QColor> colorFor;
    int K = cvec.size();
    for (int i = 0; i < K; ++i) {
        int hue = int((360.0 * i) / K);
        colorFor[cvec[i]] = QColor::fromHsv(hue, 255, 200);
    }

    // 6) draw each point at scale = pxPerUnit
    for (int i = 0; i < pts.size(); ++i) {
        int cid = labels[i];
        QColor col = (cid == NOISE ? Qt::gray : colorFor[cid]);
        painter.setBrush(col);
        painter.setPen(Qt::black);

        double x_pix = (pts[i].x() - minX) * pxPerUnit + margin;
        double y_pix = imgH - ((pts[i].y() - minY) * pxPerUnit + margin);

        painter.drawEllipse(QPointF(x_pix, y_pix), 3, 3);
    }

    return img;
}


QVector<QPointF> DBSCAN::extractLargestCluster(const QVector<QPointF>& pts,
    const QVector<int>& labels)
{
    QVector<QPointF> result;
    int n = pts.size();
    if (n == 0 || labels.size() != n)
        return result;

    QMap<int, int> counts;
    for (int id : labels) {
        if (id > 0)
            counts[id]++;
    }
    if (counts.isEmpty())
        return result;


    int bestId = counts.begin().key();
    int bestCount = counts.begin().value();
    for (auto it = counts.constBegin(); it != counts.constEnd(); ++it) {
        if (it.value() > bestCount) {
            bestCount = it.value();
            bestId = it.key();
        }
    }


    result.reserve(bestCount);
    for (int i = 0; i < n; ++i) {
        if (labels[i] == bestId)
            result.append(pts[i]);
    }

    return result;
}

