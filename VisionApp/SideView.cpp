#include "SideView.h"

#include <QColor>
#include <QFont>
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

    const QColor kBackground(27, 31, 39);
    const QColor kPanel(14, 17, 22);
    const QColor kText(230, 232, 236);
    const QColor kDim(150, 156, 168);
    const QColor kHeadRange(150, 156, 168);
    const QColor kWindow(255, 167, 38);
    const QColor kCentre(77, 208, 225);
    const QColor kData(102, 187, 106);

    //Dark blue -> blue -> cyan -> yellow -> white. Fed a log scale, so a thin surface and a dense
    //one are both visible in the same picture.
    QRgb ramp(double t)
    {
        static const QColor stops[] = { QColor(25, 40, 90), QColor(30, 110, 200), QColor(40, 200, 220),
                                        QColor(250, 220, 60), QColor(255, 255, 255) };
        t = std::clamp(t, 0.0, 1.0) * 4.0;
        const int i = std::min(3, static_cast<int>(t));
        const double f = t - i;
        const QColor& a = stops[i];
        const QColor& b = stops[i + 1];
        return qRgb(static_cast<int>(a.red() + (b.red() - a.red()) * f),
                    static_cast<int>(a.green() + (b.green() - a.green()) * f),
                    static_cast<int>(a.blue() + (b.blue() - a.blue()) * f));
    }

    //1, 2 or 5 times a power of ten, giving roughly the asked number of ticks.
    double tickStep(double span, int targetTicks)
    {
        if (span <= 0.0 || targetTicks <= 0) return 1.0;
        const double raw = span / targetTicks;
        const double mag = std::pow(10.0, std::floor(std::log10(raw)));
        for (double m : { 1.0, 2.0, 5.0 })
            if (raw <= m * mag) return m * mag;
        return 10.0 * mag;
    }

    QString mm(double v, int decimals = 2)
    {
        return (v >= 0.0 ? QStringLiteral("+") : QString()) + QString::number(v, 'f', decimals) + QStringLiteral(" mm");
    }

    struct Label {
        double y;
        QString text;
        QColor color;
    };

    //Labels down the right-hand side of a panel, nudged apart so none overlap; a short leader line
    //joins a nudged label back to the height it belongs to.
    void drawLabels(QPainter& p, const QRect& panel, std::vector<Label> labels)
    {
        std::sort(labels.begin(), labels.end(), [](const Label& a, const Label& b) { return a.y < b.y; });
        const double gap = 17.0;
        std::vector<double> placed(labels.size());
        double last = -1e9;
        for (size_t i = 0; i < labels.size(); ++i) {
            placed[i] = std::max(labels[i].y, last + gap);
            last = placed[i];
        }
        //If the column ran off the bottom, pull it back up as a block.
        const double overflow = labels.empty() ? 0.0 : placed.back() - (panel.bottom() + 40);
        if (overflow > 0)
            for (double& y : placed) y -= overflow;

        for (size_t i = 0; i < labels.size(); ++i) {
            const int x0 = panel.right() + 4;
            p.setPen(QPen(labels[i].color, 1));
            p.drawLine(QPointF(x0, labels[i].y), QPointF(x0 + 10, placed[i]));
            p.drawText(QPointF(x0 + 14, placed[i] + 5), labels[i].text);
        }
    }

} // namespace

namespace sideview {

Result render(const Input& in)
{
    Result r;
    if (!in.data || in.width <= 0 || in.height <= 0 || in.strideWords < in.width || in.zPitchUm <= 0.0)
        return r;

    auto rowPtr = [&](int y) { return in.data + qint64(y) * in.strideWords; };
    auto toMm = [&](double grey) { return (grey - 32768.0) * in.zPitchUm / 1000.0; };

    //--- 1. One histogram of grey levels gives the percentiles, extremes and edge counts exactly.
    std::vector<quint64> hist(65536, 0);
    for (int y = 0; y < in.height; ++y) {
        const quint16* row = rowPtr(y);
        for (int x = 0; x < in.width; ++x) ++hist[row[x]];
    }
    r.total = quint64(in.width) * quint64(in.height);
    r.valid = r.total - hist[0];

    if (r.valid > 0) {
        int gMin = 1, gMax = 65535;
        while (gMin < 65535 && hist[gMin] == 0) ++gMin;
        while (gMax > 1 && hist[gMax] == 0) --gMax;

        auto pct = [&](double frac) {
            const quint64 target = std::max<quint64>(1, quint64(double(r.valid) * frac));
            quint64 run = 0;
            for (int g = 1; g < 65536; ++g) {
                run += hist[g];
                if (run >= target) return g;
            }
            return 65535;
        };

        r.minMm = toMm(gMin);
        r.maxMm = toMm(gMax);
        r.lowMm = toMm(pct(0.001));
        r.highMm = toMm(pct(0.999));

        if (in.haveWindow) {
            for (int g = 1; g < 65536; ++g) {
                if (!hist[g]) continue;
                const double v = toMm(g);
                if (std::abs(v - in.windowLoMm) <= 0.1) r.nearLowEdge += hist[g];
                if (std::abs(v - in.windowHiMm) <= 0.1) r.nearHighEdge += hist[g];
            }
        }
    }

    //--- 2. Vertical extent: the whole head range, the window and every point, with a little air.
    const double half = in.headRangeMm > 0.0 ? in.headRangeMm / 2.0
        : std::max(std::abs(r.minMm), std::abs(r.maxMm));
    double zTop = half, zBot = -half;
    if (r.valid > 0) { zTop = std::max(zTop, r.maxMm); zBot = std::min(zBot, r.minMm); }
    if (in.haveWindow) { zTop = std::max(zTop, in.windowHiMm); zBot = std::min(zBot, in.windowLoMm); }
    if (zTop - zBot < 1.0) { zTop += 0.5; zBot -= 0.5; }
    const double pad = std::max(0.3, (zTop - zBot) * 0.04);
    zTop += pad;
    zBot -= pad;

    //--- 3. Layout: two panels sharing one height axis.
    const int W = 1800, H = 1290;
    const int left = 120, right = W - 330;
    const QRect panels[2] = { QRect(left, 150, right - left, 420), QRect(left, 700, right - left, 420) };
    const int ph = panels[0].height();
    const int pw = panels[0].width();

    QImage img(W, H, QImage::Format_RGB32);
    img.fill(kBackground);

    //Grey level -> pixel row, once, so the loops below do no floating point per point.
    std::vector<int> greyRow(65536, -1);
    for (int g = 1; g < 65536; ++g) {
        const int row = static_cast<int>(std::floor((zTop - toMm(g)) / (zTop - zBot) * ph));
        greyRow[g] = (row >= 0 && row < ph) ? row : -1;
    }
    std::vector<int> colPx(in.width);
    for (int x = 0; x < in.width; ++x) colPx[x] = static_cast<int>(qint64(x) * pw / in.width);

    //--- 4. Density: how many points land at each (position, height), on a log colour scale.
    for (int k = 0; k < 2; ++k) {
        const QRect& rc = panels[k];
        std::vector<quint32> counts(size_t(pw) * ph, 0);
        for (int y = 0; y < in.height; ++y) {
            const quint16* row = rowPtr(y);
            const int along = static_cast<int>(qint64(y) * pw / in.height);
            for (int x = 0; x < in.width; ++x) {
                const quint16 g = row[x];
                if (!g) continue;
                const int py = greyRow[g];
                if (py < 0) continue;
                ++counts[size_t(py) * pw + (k == 0 ? colPx[x] : along)];
            }
        }

        quint32 maxc = 1;
        for (quint32 c : counts) maxc = std::max(maxc, c);
        const double lmax = std::log1p(double(maxc));

        for (int py = 0; py < ph; ++py) {
            QRgb* line = reinterpret_cast<QRgb*>(img.scanLine(rc.top() + py));
            for (int px = 0; px < pw; ++px) {
                const quint32 c = counts[size_t(py) * pw + px];
                line[rc.left() + px] = c ? ramp(std::log1p(double(c)) / lmax) : kPanel.rgb();
            }
        }
    }

    //--- 5. Overlays and text.
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    //Pixel sizes, not points: point sizes follow the screen's DPI, and the same report must read the
    //same on the IPC (100 %) and a laptop at 150 %.
    QFont titleFont(QStringLiteral("Segoe UI"));
    titleFont.setPixelSize(22);
    titleFont.setBold(true);
    QFont font(QStringLiteral("Segoe UI"));
    font.setPixelSize(15);
    QFont small(QStringLiteral("Segoe UI"));
    small.setPixelSize(13);

    p.setPen(kText);
    p.setFont(titleFont);
    int ty = 34;
    for (int i = 0; i < in.titleLines.size(); ++i) {
        p.drawText(QPointF(left, ty), in.titleLines[i]);
        if (i == 0) p.setFont(font);
        ty += (i == 0) ? 26 : 20;
    }

    auto yOf = [&](const QRect& rc, double v) { return rc.top() + (zTop - v) / (zTop - zBot) * rc.height(); };
    const double yStep = tickStep(zTop - zBot, 10);
    const int yDec = yStep < 1.0 ? 1 : 0;

    const double acrossMm = in.width * in.xPitchUm / 1000.0;
    const double alongMm = in.height * in.yPitchUm / 1000.0;

    for (int k = 0; k < 2; ++k) {
        const QRect& rc = panels[k];

        //Panel title
        p.setFont(font);
        p.setPen(kText);
        p.drawText(QPointF(rc.left(), rc.top() - 10), k == 0
            ? QStringLiteral("Across the laser line - every profile overlaid (as Navigator's profile view)")
            : QStringLiteral("Along the scan - every point of each profile overlaid"));

        //Height grid and labels
        p.setFont(small);
        for (double v = std::ceil(zBot / yStep) * yStep; v <= zTop + 1e-9; v += yStep) {
            const double y = yOf(rc, v);
            p.setPen(QPen(QColor(255, 255, 255, 18), 1));
            p.drawLine(QPointF(rc.left(), y), QPointF(rc.right(), y));
            p.setPen(kDim);
            p.drawLine(QPointF(rc.left() - 5, y), QPointF(rc.left(), y));
            p.drawText(QRectF(rc.left() - 70, y - 11, 62, 22), Qt::AlignRight | Qt::AlignVCenter,
                QString::number(std::abs(v) < 1e-9 ? 0.0 : v, 'f', yDec));
        }

        //Position axis
        const double span = (k == 0) ? acrossMm : alongMm;
        if (span > 0.0) {
            const double xStep = tickStep(span, 8);
            for (double v = 0.0; v <= span + 1e-9; v += xStep) {
                const double x = rc.left() + v / span * rc.width();
                p.setPen(kDim);
                p.drawLine(QPointF(x, rc.bottom()), QPointF(x, rc.bottom() + 5));
                p.drawText(QRectF(x - 40, rc.bottom() + 7, 80, 20), Qt::AlignHCenter | Qt::AlignTop,
                    QString::number(v, 'f', xStep < 1.0 ? 1 : 0));
            }
            p.drawText(QRectF(rc.left(), rc.bottom() + 28, rc.width(), 20), Qt::AlignHCenter | Qt::AlignTop,
                k == 0 ? QStringLiteral("mm across the laser line (%1 mm field of view)").arg(acrossMm, 0, 'f', 1)
                       : QStringLiteral("mm along the scan (%1 profiles, %2 mm)").arg(in.height).arg(alongMm, 0, 'f', 1));
        }

        //Height axis title
        p.save();
        p.translate(rc.left() - 88, rc.center().y());
        p.rotate(-90);
        p.setPen(kDim);
        p.drawText(QRectF(-rc.height() / 2.0, -12, rc.height(), 24), Qt::AlignCenter,
            QStringLiteral("height, mm (0 = reference distance)"));
        p.restore();

        std::vector<Label> labels;

        //The window: what lies outside it can never be measured, so shade it.
        if (in.haveWindow) {
            const double yHi = yOf(rc, in.windowHiMm);
            const double yLo = yOf(rc, in.windowLoMm);
            const QColor shade(kWindow.red(), kWindow.green(), kWindow.blue(), 34);
            if (yHi > rc.top()) p.fillRect(QRectF(rc.left(), rc.top(), rc.width(), yHi - rc.top()), shade);
            if (yLo < rc.bottom()) p.fillRect(QRectF(rc.left(), yLo, rc.width(), rc.bottom() - yLo), shade);

            p.setPen(QPen(kWindow, 2, Qt::DashLine));
            p.drawLine(QPointF(rc.left(), yHi), QPointF(rc.right(), yHi));
            p.drawLine(QPointF(rc.left(), yLo), QPointF(rc.right(), yLo));
            labels.push_back({ yHi, QStringLiteral("window top ") + mm(in.windowHiMm), kWindow });
            labels.push_back({ yLo, QStringLiteral("window bottom ") + mm(in.windowLoMm), kWindow });

            const double yC = yOf(rc, in.centreMm);
            p.setPen(QPen(kCentre, 2, Qt::DashDotLine));
            p.drawLine(QPointF(rc.left(), yC), QPointF(rc.right(), yC));
            labels.push_back({ yC, QStringLiteral("centre ") + mm(in.centreMm), kCentre });
        }

        //The head's own range
        if (in.headRangeMm > 0.0) {
            p.setPen(QPen(kHeadRange, 1.5));
            const double yT = yOf(rc, half), yB = yOf(rc, -half);
            p.drawLine(QPointF(rc.left(), yT), QPointF(rc.right(), yT));
            p.drawLine(QPointF(rc.left(), yB), QPointF(rc.right(), yB));
            labels.push_back({ yT, QStringLiteral("head range ") + mm(half), kHeadRange });
            labels.push_back({ yB, QStringLiteral("head range ") + mm(-half), kHeadRange });
        }

        //Where the part actually is: 99.9 % of the points lie between these two lines.
        if (r.valid > 0) {
            p.setPen(QPen(kData, 1.5, Qt::DotLine));
            const double yH = yOf(rc, r.highMm), yL = yOf(rc, r.lowMm);
            p.drawLine(QPointF(rc.left(), yH), QPointF(rc.right(), yH));
            p.drawLine(QPointF(rc.left(), yL), QPointF(rc.right(), yL));
            labels.push_back({ yH, QStringLiteral("99.9% of points below ") + mm(r.highMm), kData });
            labels.push_back({ yL, QStringLiteral("99.9% of points above ") + mm(r.lowMm), kData });
        }

        p.setFont(small);
        drawLabels(p, rc, labels);

        p.setPen(QPen(kDim, 1));
        p.setBrush(Qt::NoBrush);
        p.drawRect(rc.adjusted(0, 0, -1, -1));
    }

    //--- 6. The numbers, under the pictures, wrapped to the image width.
    double fy = panels[1].bottom() + 64;
    const double textWidth = W - left - 30;
    auto para = [&](const QString& text, const QFont& f, const QColor& c) {
        p.setFont(f);
        p.setPen(c);
        const QRectF box(left, fy, textWidth, 200);
        const QRectF used = p.boundingRect(box, Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop, text);
        p.drawText(box, Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop, text);
        fy += used.height() + 4;
    };

    const double validPct = r.total ? 100.0 * double(r.valid) / double(r.total) : 0.0;
    para(QStringLiteral("Valid points %1 of %2 (%3%).   99.9% of heights between %4 and %5   (lowest %6, highest %7).")
        .arg(r.valid).arg(r.total).arg(validPct, 0, 'f', 1)
        .arg(mm(r.lowMm)).arg(mm(r.highMm)).arg(mm(r.minMm)).arg(mm(r.maxMm)), font, kText);

    if (in.haveWindow && r.valid > 0) {
        const double below = r.lowMm - in.windowLoMm;
        const double above = in.windowHiMm - r.highMm;
        para(QStringLiteral("%1.   Margin: %2 mm under the lowest 0.1% of points, %3 mm over the highest 0.1%.   "
                            "Within 0.1 mm of an edge: %4 points at the bottom, %5 at the top.")
            .arg(in.windowLabel).arg(below, 0, 'f', 2).arg(above, 0, 'f', 2).arg(r.nearLowEdge).arg(r.nearHighEdge),
            font, (below < 0.5 || above < 0.5) ? kWindow : kText);
    }
    else if (!in.haveWindow) {
        para(QStringLiteral("Z window not drawn - the Z range and centre could not be read from the controller or the optic."), font, kDim);
    }

    para(QStringLiteral("The window is NOMINAL: a straight fraction of the head range around the centre setting. A triangulation head is not "
                        "perfectly linear, so its real edges can sit a little off - but no point is ever measured outside the real window. "
                        "A pile-up of points against a window edge, or a surface that stops dead there, means that surface is being cut off. "
                        "Colour is the number of points at that position and height, on a log scale."), small, kDim);
    p.end();

    r.image = img;
    r.ok = true;
    return r;
}

} // namespace sideview
