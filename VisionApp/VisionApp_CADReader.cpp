#include "VisionApp.h"
#include "ScaleManager.h"
#include "QDragBox.h"


bool VisionApp::loadFiducialsAndPads(const QString& txtPath,QVector<QPointF>& fidList,QVector<QPointF>& padList)
{
    QFile file(txtPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Cannot open" << txtPath << ":" << file.errorString();
        return false;
    }
    QTextStream in(&file);
    in.setCodec("UTF-8");

    bool inShape = false;
    bool inPad = false;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();

        // Section markers
        if (line.startsWith('@')) {
            QString tag = line.mid(1).toUpper();
            inShape = (tag == "SHAPE");
            inPad = (tag == "PAD");
            continue;
        }
        if (line.isEmpty())
            continue;

        QStringList parts = line.simplified().split(' ');
        if (inShape) {
            // Expect: index type width height ...
            bool okIdx = false;
            int idx = parts[0].toInt(&okIdx);
            if (!okIdx || parts.size() < 4)
                continue;
            // Only rows 2-4
            if (idx >= 2 && idx <= 4) {
                bool okW, okH;
                double w = parts[2].toDouble(&okW);
                double h = parts[3].toDouble(&okH);
                if (okW && okH)
                    fidList.append(QPointF(w, h));
            }
        }
        else if (inPad) {
            // Lines beginning with "PAD"; x,y at n-4,n-3
            QString token = parts[0].toUpper();
            if (token == "PAD1" && parts.size() >= 5) {
                int n = parts.size();
                bool okX, okY;
                double x = parts[n - 4].toDouble(&okX);
                double y = parts[n - 3].toDouble(&okY);
                if (okX && okY)
                    padList.append(QPointF(x, y));
            }
        }
    }
    file.close();

    qDebug() << "Loaded" << fidList.size() << "fiducials and"
        << padList.size() << "pad points from" << txtPath;
    return true;
}


bool VisionApp::exportSingleFiducialAndPad(const QString& txtPath, const QString& jsonPath)
{
    QFile file(txtPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Cannot open" << txtPath << ":" << file.errorString();
        return false;
    }
    QTextStream in(&file);
    in.setCodec("UTF-8");

    bool inShape = false;
    bool inPad = false;

    // JSON arrays we'll build up
    QJsonArray fidArray;
    QJsonArray padArray;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();

        // Section switches
        if (line.startsWith('@')) {
            QString tag = line.mid(1).toUpper();
            inShape = (tag == "SHAPE");
            inPad = (tag == "PAD");
            continue;
        }
        if (line.isEmpty())
            continue;

        QStringList parts = line.simplified().split(' ');
        if (inShape) {
            // Expect at least: index, type, w, h, …  
            bool okIdx = false;
            int idx = parts[0].toInt(&okIdx);
            if (!okIdx)
                continue;

            // Only rows 2,3,4
            if (idx >= 2 && idx <= 4 && parts.size() >= 4) {
                bool okW, okH;
                double w = parts[2].toDouble(&okW);
                double h = parts[3].toDouble(&okH);
                if (okW && okH) {
                    QJsonObject fidObj;
                    fidObj["index"] = idx;
                    fidObj["width"] = w;
                    fidObj["height"] = h;
                    fidArray.append(fidObj);
                }
            }
        }
        else if (inPad) {
            // PAD rows can have many columns, but x,y are at n-4,n-3
            if (parts[0].toUpper().startsWith("PAD") && parts.size() >= 4) {
                int n = parts.size();
                bool okX, okY;
                double x = parts[n - 4].toDouble(&okX);
                double y = parts[n - 3].toDouble(&okY);
                if (okX && okY) {
                    QJsonObject padObj;
                    padObj["label"] = parts[0];     // e.g. "PAD1"
                    padObj["x"] = x;
                    padObj["y"] = y;
                    padArray.append(padObj);
                }
            }
        }
    }
    file.close();

    // Build root JSON
    QJsonObject root;
    root["Fiducials"] = fidArray;
    root["Pads"] = padArray;

    // Write out
    QFile out(jsonPath);
    if (!out.open(QIODevice::WriteOnly)) {
        qWarning() << "Cannot open" << jsonPath << ":" << out.errorString();
        return false;
    }
    QJsonDocument doc(root);
    out.write(doc.toJson(QJsonDocument::Indented));
    out.close();

    qDebug() << "Exported" << fidArray.size() << "fiducials and"
        << padArray.size() << "pads to" << jsonPath;
    return true;
}

void VisionApp::overlayPadCircles(
    QDragBox* box,
    QVector<QGraphicsEllipseItem*>& circleItemsVec,
    const QVector<QPointF>& padData,
    const QRectF& originalDataBoundingRect, // worldClusterRect
    const QRectF& targetContentAreaInBox, // e.g. (margin, margin, box_w - 2*margin, box_h - 2*margin)
    qreal circleRadius,
    const QColor& circleColor)
{
    if (!box || padData.isEmpty() || originalDataBoundingRect.width() <= 0 || originalDataBoundingRect.height() <= 0 ||
        targetContentAreaInBox.width() <= 0 || targetContentAreaInBox.height() <= 0) {
        qWarning() << "overlayPadCircles_AspectRatio: Invalid input";
        // Clear existing circles if any, even on error, to ensure clean state
        for (QGraphicsEllipseItem* item : circleItemsVec) { if (item) { if (item->scene()) item->scene()->removeItem(item); delete item; } }
        circleItemsVec.clear();
        return;
    }

    // Clear existing circles
    for (QGraphicsEllipseItem* item : circleItemsVec) { if (item) { if (item->scene()) item->scene()->removeItem(item); delete item; } }
    circleItemsVec.clear();

    // Calculate the scale factor to fit originalDataBoundingRect into targetContentAreaInBox, preserving aspect ratio
    qreal scaleToFitX = targetContentAreaInBox.width() / originalDataBoundingRect.width();
    qreal scaleToFitY = targetContentAreaInBox.height() / originalDataBoundingRect.height();
    qreal actualContentScale = qMin(scaleToFitX, scaleToFitY);

    if (actualContentScale <= 0) {
        qWarning() << "Calculated actualContentScale is zero or negative.";
        return;
    }

    // Dimensions of the content after scaling with 'actualContentScale'
    qreal finalScaledContentWidth = originalDataBoundingRect.width() * actualContentScale;
    qreal finalScaledContentHeight = originalDataBoundingRect.height() * actualContentScale;

    // Calculate top-left offset to center this finalScaledContent within targetContentAreaInBox
    qreal offsetX = targetContentAreaInBox.left() + (targetContentAreaInBox.width() - finalScaledContentWidth) / 2.0;
    qreal offsetY = targetContentAreaInBox.top() + (targetContentAreaInBox.height() - finalScaledContentHeight) / 2.0;

    for (const auto& pad : padData) {
        // 1. Translate original pad coordinate so originalDataBoundingRect.topLeft() is at (0,0)
        double relX = pad.x() - originalDataBoundingRect.left();
        double relY = pad.y() - originalDataBoundingRect.top();

        // 2. Scale these relative coordinates
        double scaledX = relX * actualContentScale;
        double scaledY = relY * actualContentScale;

        // 3. Add the final offset to position it in the box
        double finalX_in_box = offsetX + scaledX;
        double finalY_in_box = offsetY + (finalScaledContentHeight - scaledY);

        qreal circleDiameter = circleRadius * 2.0;
        QGraphicsEllipseItem* circle = new QGraphicsEllipseItem(
            finalX_in_box - circleRadius,
            finalY_in_box - circleRadius,
            circleDiameter,
            circleDiameter,
            box);

        circle->setBrush(QBrush(circleColor));
        circle->setPen(Qt::NoPen);
        circleItemsVec.append(circle);
    }
}