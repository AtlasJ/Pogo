//
//  Guided Setup (new): the recipe file guidedSetup2.json and the layout maths built on it.
//  See GuidedSetup2Types.h for what each part means and how the frames chain together.
//
#include "GuidedSetup2Types.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QSet>
#include <cmath>

namespace gs2 {

namespace {

const char* const kFileName = "guidedSetup2.json";
const int kFileVersion = 1;

double num(const QJsonObject& o, const char* key, double def) { return o.value(QLatin1String(key)).toDouble(def); }
int integer(const QJsonObject& o, const char* key, int def) { return o.value(QLatin1String(key)).toInt(def); }
bool boolean(const QJsonObject& o, const char* key, bool def) { return o.value(QLatin1String(key)).toBool(def); }
QString str(const QJsonObject& o, const char* key) { return o.value(QLatin1String(key)).toString(); }

//a taught XY is written as [x, y] and left out while it is not taught
void putXY(QJsonObject& o, const char* key, const TaughtXY& p)
{
	if (p.set) o.insert(QLatin1String(key), QJsonArray{ p.x, p.y });
}

TaughtXY getXY(const QJsonObject& o, const char* key)
{
	TaughtXY p;
	const QJsonArray a = o.value(QLatin1String(key)).toArray();
	if (a.size() == 2 && a[0].isDouble() && a[1].isDouble()) { p.set = true; p.x = a[0].toDouble(); p.y = a[1].toDouble(); }
	return p;
}

void putDia(QJsonObject& o, const char* key, const DiaRange& d)
{
	o.insert(QLatin1String(key), QJsonArray{ d.minMm, d.maxMm });
}

DiaRange getDia(const QJsonObject& o, const char* key)
{
	DiaRange d;
	const QJsonArray a = o.value(QLatin1String(key)).toArray();
	if (a.size() == 2) { d.minMm = a[0].toDouble(); d.maxMm = a[1].toDouble(); }
	return d;
}

const char* const kCornerKey[4] = { "TL", "TR", "BR", "BL" };
const char* const kInspKey[InspTypeCount] = { "2D", "barcodeOcr", "3D" };
const char* const kRefKey[5] = { "centre", "TL", "TR", "BR", "BL" };

int refFromKey(const QString& k)
{
	for (int i = 0; i < 5; i++) if (k == QLatin1String(kRefKey[i])) return i;
	return RefCentre;
}

const QString kTempPrefix = QStringLiteral("NEW-");

} // namespace

QString sectionTitle(Section s)
{
	switch (s) {
	case SecSafety1:     return QStringLiteral("Safety check 1");
	case SecFid0:        return QStringLiteral("fid0");
	case Sec3DZ:         return QStringLiteral("3D Z");
	case SecFidRect:     return QStringLiteral("fidRect");
	case SecSlotSize:    return QStringLiteral("Slot size");
	case SecSlots:       return QStringLiteral("Slots");
	case SecUnits:       return QStringLiteral("Units");
	case SecInspections: return QStringLiteral("Inspections");
	default:             return QString();
	}
}

QString inspTypeName(InspType t)
{
	switch (t) {
	case Insp2D:         return QStringLiteral("2D");
	case InspBarcodeOcr: return QStringLiteral("Barcode + OCR");
	case Insp3D:         return QStringLiteral("3D");
	default:             return QString();
	}
}

QString refName(int ref)
{
	static const char* const names[5] = { "Unit centre", "TL corner", "TR corner", "BR corner", "BL corner" };
	return (ref >= 0 && ref < 5) ? QLatin1String(names[ref]) : QString();
}

bool sectionDone(const Setup& s, Section sec, QString* why)
{
	auto fail = [why](const QString& w) { if (why) *why = w; return false; };
	switch (sec) {
	case SecSafety1:
		if (s.safety1.emptyImages.isEmpty()) return fail(QStringLiteral("no empty image saved"));
		if (s.safety1.threshold <= 0.0) return fail(QStringLiteral("no threshold"));
		return true;
	case SecFid0:
		if (!s.fid0.xy.set) return fail(QStringLiteral("search XY not taught"));
		if (!s.fid0.startSet || !s.fid0.stopSet) return fail(QStringLiteral("Start Z / Stop Z not set"));
		if (!(s.fid0.startZ < s.fid0.stopZ)) return fail(QStringLiteral("Start Z must be above (smaller than) Stop Z"));
		if (s.fid0.patternFile.isEmpty()) return fail(QStringLiteral("no pattern learnt"));
		if (!s.fid0.dia.valid()) return fail(QStringLiteral("circle diameter range not set"));
		return true;
	case Sec3DZ:
		if (!s.z3d.offset.set) return fail(QStringLiteral("flat spot not set"));
		return true;
	case SecFidRect:
		if (s.fidRect.widthMm <= 0.0 || s.fidRect.lengthMm <= 0.0) return fail(QStringLiteral("drawing width / length not typed"));
		for (int i = 0; i < 4; i++)
			if (!s.fidRect.corner[i].set) return fail(QStringLiteral("fid%1 not taught").arg(QLatin1String(kCornerKey[i])));
		if (!s.fidRect.dia.valid()) return fail(QStringLiteral("circle diameter range not set"));
		return true;
	case SecSlotSize:
		if (!s.slotSize.set || s.slotSize.widthMm <= 0.0 || s.slotSize.lengthMm <= 0.0) return fail(QStringLiteral("TL and BR not set"));
		if (!s.slotSize.dia.valid()) return fail(QStringLiteral("circle diameter range not set"));
		return true;
	case SecSlots:
		if (s.slotList.isEmpty()) return fail(QStringLiteral("no slot"));
		for (int i = 0; i < s.slotList.size(); i++)
			for (int j = i + 1; j < s.slotList.size(); j++)
				if (gs2geo::slotsTouch({ s.slotList[i].cx, s.slotList[i].cy }, { s.slotList[j].cx, s.slotList[j].cy }, s.slotSize.widthMm, s.slotSize.lengthMm))
					return fail(QStringLiteral("%1 touches %2").arg(s.slotList[i].name, s.slotList[j].name));
		return true;
	case SecUnits: {
		const auto& u = s.unitLayout;
		if (u.widthMm <= 0.0 || u.lengthMm <= 0.0) return fail(QStringLiteral("unit width / length not typed"));
		if (!gs2geo::unitsFit(u.rows, u.cols, u.widthMm, u.lengthMm, u.marginMm, s.slotSize.widthMm, s.slotSize.lengthMm))
			return fail(QStringLiteral("the units do not fit in the slot"));
		if (s.units.size() != s.slotList.size() * u.rows * u.cols) return fail(QStringLiteral("unit list out of date"));
		QSet<QString> seen;
		for (const auto& unit : s.units) {
			if (unit.name.isEmpty()) return fail(QStringLiteral("a unit has no name"));
			if (isTemporaryName(unit.name)) return fail(QStringLiteral("%1 needs a name (Name all by position)").arg(unit.name));
			if (seen.contains(unit.name)) return fail(QStringLiteral("two units are named %1").arg(unit.name));
			seen.insert(unit.name);
		}
		return true;
	}
	case SecInspections: {
		bool any = false;
		for (int t = 0; t < InspTypeCount; t++) any = any || s.insp[t].enabled;
		if (!any) return fail(QStringLiteral("no inspection switched on"));
		if (s.insp[Insp3D].enabled && s.insp[Insp3D].scanLengthMm <= 0.0) return fail(QStringLiteral("3D scan length not set"));
		return true;
	}
	default:
		return fail(QString());
	}
}

bool isComplete(const Setup& s)
{
	for (int i = 0; i < SectionCount; i++)
		if (!sectionDone(s, (Section)i)) return false;
	return true;
}

QJsonObject toJson(const Setup& s)
{
	QJsonObject root;
	root.insert(QStringLiteral("version"), kFileVersion);
	root.insert(QStringLiteral("complete"), isComplete(s));

	QJsonObject a;
	a.insert(QStringLiteral("emptyImages"), QJsonArray::fromStringList(s.safety1.emptyImages));
	a.insert(QStringLiteral("threshold"), s.safety1.threshold);
	root.insert(QStringLiteral("safety1"), a);

	QJsonObject b;
	putXY(b, "searchXY", s.fid0.xy);
	if (s.fid0.startSet) b.insert(QStringLiteral("startZ"), s.fid0.startZ);
	if (s.fid0.stopSet) b.insert(QStringLiteral("stopZ"), s.fid0.stopZ);
	b.insert(QStringLiteral("stepMm"), s.fid0.stepMm);
	b.insert(QStringLiteral("refineStages"), s.fid0.refineStages);
	b.insert(QStringLiteral("focusBoxMm"), s.fid0.focusBoxMm);
	b.insert(QStringLiteral("pattern"), s.fid0.patternFile);
	b.insert(QStringLiteral("minScore"), s.fid0.minScore);
	putDia(b, "diaMm", s.fid0.dia);
	root.insert(QStringLiteral("fid0"), b);

	QJsonObject c;
	putXY(c, "offsetFromFid0", s.z3d.offset);
	c.insert(QStringLiteral("stepMm"), s.z3d.stepMm);
	c.insert(QStringLiteral("targetMm"), s.z3d.targetMm);
	c.insert(QStringLiteral("maxSpreadMm"), s.z3d.maxSpreadMm);
	if (s.z3d.diffSet) c.insert(QStringLiteral("diffMm"), s.z3d.diffMm);
	c.insert(QStringLiteral("diffTolMm"), s.z3d.diffTolMm);
	root.insert(QStringLiteral("z3d"), c);

	QJsonObject d;
	d.insert(QStringLiteral("widthMm"), s.fidRect.widthMm);
	d.insert(QStringLiteral("lengthMm"), s.fidRect.lengthMm);
	QJsonObject corners;
	for (int i = 0; i < 4; i++) putXY(corners, kCornerKey[i], s.fidRect.corner[i]);
	d.insert(QStringLiteral("corners"), corners);
	putDia(d, "diaMm", s.fidRect.dia);
	d.insert(QStringLiteral("residualLimitMm"), s.fidRect.residualLimitMm);
	d.insert(QStringLiteral("maxAngleDeg"), s.fidRect.maxAngleDeg);
	root.insert(QStringLiteral("fidRect"), d);

	QJsonObject e;
	e.insert(QStringLiteral("set"), s.slotSize.set);
	e.insert(QStringLiteral("widthMm"), s.slotSize.widthMm);
	e.insert(QStringLiteral("lengthMm"), s.slotSize.lengthMm);
	putDia(e, "circleDiaMm", s.slotSize.dia);
	root.insert(QStringLiteral("slotSize"), e);

	QJsonArray slotArray;
	for (const auto& sl : s.slotList) {
		QJsonObject o;
		o.insert(QStringLiteral("id"), sl.id);
		o.insert(QStringLiteral("name"), sl.name);
		o.insert(QStringLiteral("centre"), QJsonArray{ sl.cx, sl.cy });
		slotArray.append(o);
	}
	root.insert(QStringLiteral("slots"), slotArray);

	QJsonObject g;
	g.insert(QStringLiteral("widthMm"), s.unitLayout.widthMm);
	g.insert(QStringLiteral("lengthMm"), s.unitLayout.lengthMm);
	g.insert(QStringLiteral("rows"), s.unitLayout.rows);
	g.insert(QStringLiteral("cols"), s.unitLayout.cols);
	g.insert(QStringLiteral("marginMm"), s.unitLayout.marginMm);
	root.insert(QStringLiteral("unitLayout"), g);

	QJsonArray units;
	for (const auto& u : s.units) {
		QJsonObject o;
		o.insert(QStringLiteral("id"), u.id);
		o.insert(QStringLiteral("slot"), u.slotId);
		o.insert(QStringLiteral("cell"), QJsonArray{ u.row + 1, u.col + 1 });   //1-based in the file
		o.insert(QStringLiteral("name"), u.name);
		units.append(o);
	}
	root.insert(QStringLiteral("units"), units);

	QJsonObject h;
	for (int t = 0; t < InspTypeCount; t++) {
		const auto& in = s.insp[t];
		QJsonObject o;
		o.insert(QStringLiteral("enabled"), in.enabled);
		o.insert(QStringLiteral("ref"), QLatin1String(kRefKey[(in.ref >= 0 && in.ref < 5) ? in.ref : 0]));
		o.insert(QStringLiteral("offset"), QJsonArray{ in.dx, in.dy });
		o.insert(QStringLiteral("zOffsetMm"), in.zOffsetMm);
		if (t == Insp3D) o.insert(QStringLiteral("scanLengthMm"), in.scanLengthMm);
		h.insert(QLatin1String(kInspKey[t]), o);
	}
	root.insert(QStringLiteral("inspections"), h);

	root.insert(QStringLiteral("scanDirection"), QString(s.scanNegative ? "-" : "+") + s.scanAxis);
	root.insert(QStringLiteral("nextSlotId"), s.nextSlotId);
	root.insert(QStringLiteral("nextUnitId"), s.nextUnitId);
	return root;
}

Setup fromJson(const QJsonObject& root)
{
	Setup s;

	const QJsonObject a = root.value(QStringLiteral("safety1")).toObject();
	for (const auto& v : a.value(QStringLiteral("emptyImages")).toArray())
		if (!v.toString().isEmpty()) s.safety1.emptyImages << v.toString();
	s.safety1.threshold = num(a, "threshold", 0.0);

	const QJsonObject b = root.value(QStringLiteral("fid0")).toObject();
	s.fid0.xy = getXY(b, "searchXY");
	s.fid0.startSet = b.contains(QStringLiteral("startZ"));
	s.fid0.startZ = num(b, "startZ", 0.0);
	s.fid0.stopSet = b.contains(QStringLiteral("stopZ"));
	s.fid0.stopZ = num(b, "stopZ", 0.0);
	s.fid0.stepMm = num(b, "stepMm", s.fid0.stepMm);
	s.fid0.refineStages = integer(b, "refineStages", s.fid0.refineStages);
	s.fid0.focusBoxMm = num(b, "focusBoxMm", s.fid0.focusBoxMm);
	s.fid0.patternFile = str(b, "pattern");
	s.fid0.minScore = num(b, "minScore", s.fid0.minScore);
	s.fid0.dia = getDia(b, "diaMm");

	const QJsonObject c = root.value(QStringLiteral("z3d")).toObject();
	s.z3d.offset = getXY(c, "offsetFromFid0");
	s.z3d.stepMm = num(c, "stepMm", s.z3d.stepMm);
	s.z3d.targetMm = num(c, "targetMm", s.z3d.targetMm);
	s.z3d.maxSpreadMm = num(c, "maxSpreadMm", s.z3d.maxSpreadMm);
	s.z3d.diffSet = c.contains(QStringLiteral("diffMm"));
	s.z3d.diffMm = num(c, "diffMm", 0.0);
	s.z3d.diffTolMm = num(c, "diffTolMm", s.z3d.diffTolMm);

	const QJsonObject d = root.value(QStringLiteral("fidRect")).toObject();
	s.fidRect.widthMm = num(d, "widthMm", 0.0);
	s.fidRect.lengthMm = num(d, "lengthMm", 0.0);
	const QJsonObject corners = d.value(QStringLiteral("corners")).toObject();
	for (int i = 0; i < 4; i++) s.fidRect.corner[i] = getXY(corners, kCornerKey[i]);
	s.fidRect.dia = getDia(d, "diaMm");
	s.fidRect.residualLimitMm = num(d, "residualLimitMm", s.fidRect.residualLimitMm);
	s.fidRect.maxAngleDeg = num(d, "maxAngleDeg", s.fidRect.maxAngleDeg);

	const QJsonObject e = root.value(QStringLiteral("slotSize")).toObject();
	s.slotSize.set = boolean(e, "set", false);
	s.slotSize.widthMm = num(e, "widthMm", 0.0);
	s.slotSize.lengthMm = num(e, "lengthMm", 0.0);
	s.slotSize.dia = getDia(e, "circleDiaMm");

	for (const auto& v : root.value(QStringLiteral("slots")).toArray()) {
		const QJsonObject o = v.toObject();
		const QJsonArray ctr = o.value(QStringLiteral("centre")).toArray();
		if (ctr.size() != 2 || str(o, "id").isEmpty()) continue;
		Slot sl;
		sl.id = str(o, "id");
		sl.name = str(o, "name");
		sl.cx = ctr[0].toDouble();
		sl.cy = ctr[1].toDouble();
		s.slotList.append(sl);
	}

	const QJsonObject g = root.value(QStringLiteral("unitLayout")).toObject();
	s.unitLayout.widthMm = num(g, "widthMm", 0.0);
	s.unitLayout.lengthMm = num(g, "lengthMm", 0.0);
	//the page allows 1..50; a hand-edited file is held to the same
	s.unitLayout.rows = std::min(50, std::max(1, integer(g, "rows", 1)));
	s.unitLayout.cols = std::min(50, std::max(1, integer(g, "cols", 1)));
	s.unitLayout.marginMm = std::max(0.0, num(g, "marginMm", 0.0));

	for (const auto& v : root.value(QStringLiteral("units")).toArray()) {
		const QJsonObject o = v.toObject();
		const QJsonArray cell = o.value(QStringLiteral("cell")).toArray();
		if (cell.size() != 2 || str(o, "id").isEmpty()) continue;
		Unit u;
		u.id = str(o, "id");
		u.slotId = str(o, "slot");
		u.row = cell[0].toInt() - 1;
		u.col = cell[1].toInt() - 1;
		u.name = str(o, "name");
		s.units.append(u);
	}

	const QJsonObject h = root.value(QStringLiteral("inspections")).toObject();
	for (int t = 0; t < InspTypeCount; t++) {
		const QJsonObject o = h.value(QLatin1String(kInspKey[t])).toObject();
		auto& in = s.insp[t];
		in.enabled = boolean(o, "enabled", false);
		in.ref = refFromKey(str(o, "ref"));
		const QJsonArray off = o.value(QStringLiteral("offset")).toArray();
		if (off.size() == 2) { in.dx = off[0].toDouble(); in.dy = off[1].toDouble(); }
		in.zOffsetMm = num(o, "zOffsetMm", 0.0);
		in.scanLengthMm = num(o, "scanLengthMm", 0.0);
	}

	const QString dir = str(root, "scanDirection");
	s.scanNegative = dir.startsWith(QLatin1Char('-'));
	s.scanAxis = dir.endsWith(QLatin1Char('Y')) ? QStringLiteral("Y") : QStringLiteral("X");
	s.nextSlotId = std::max(1, integer(root, "nextSlotId", 1));
	s.nextUnitId = std::max(1, integer(root, "nextUnitId", 1));

	//never hand out an id the file already uses, even if the counters were edited by hand
	for (const auto& sl : s.slotList) s.nextSlotId = std::max(s.nextSlotId, sl.id.mid(1).toInt() + 1);
	for (const auto& u : s.units) s.nextUnitId = std::max(s.nextUnitId, u.id.mid(1).toInt() + 1);
	return s;
}

QString filePath(const QString& recipeDir)
{
	return recipeDir + QLatin1String(kFileName);
}

LoadResult load(const QString& recipeDir, Setup& out, QString& why)
{
	out = Setup();
	QFile f(filePath(recipeDir));
	if (!f.exists()) { why = QStringLiteral("not set up yet"); return LoadResult::Missing; }
	if (!f.open(QIODevice::ReadOnly)) { why = QStringLiteral("cannot open %1: %2").arg(f.fileName(), f.errorString()); return LoadResult::Unreadable; }
	QJsonParseError err;
	const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
	if (err.error != QJsonParseError::NoError || !doc.isObject()) {
		why = QStringLiteral("%1 is not valid JSON (%2)").arg(f.fileName(), err.errorString());
		return LoadResult::Unreadable;
	}
	out = fromJson(doc.object());
	why.clear();
	return LoadResult::Loaded;
}

bool save(const QString& recipeDir, const Setup& s, QString& why)
{
	const QString path = filePath(recipeDir);

	//an existing file we could not read is kept aside rather than overwritten unseen
	if (QFile::exists(path)) {
		QFile old(path);
		bool readable = false;
		if (old.open(QIODevice::ReadOnly)) {
			QJsonParseError err;
			readable = QJsonDocument::fromJson(old.readAll(), &err).isObject() && err.error == QJsonParseError::NoError;
			old.close();
		}
		if (!readable) {
			const QString kept = path + QStringLiteral(".unreadable-") + QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss"));
			if (!QFile::copy(path, kept)) { why = QStringLiteral("could not keep the unreadable file aside as %1").arg(kept); return false; }
		}
	}

	//all or nothing: a half-written file would place every unit wrongly
	QSaveFile file(path);
	if (!file.open(QIODevice::WriteOnly)) { why = QStringLiteral("cannot write %1: %2").arg(path, file.errorString()); return false; }
	file.write(QJsonDocument(toJson(s)).toJson(QJsonDocument::Indented));
	if (!file.commit()) { why = QStringLiteral("cannot write %1: %2").arg(path, file.errorString()); return false; }
	why.clear();
	return true;
}

gs2geo::RectFit taughtFit(const Setup& s)
{
	gs2geo::RectFit fit;
	if (!s.fid0.xy.set) return fit;
	gs2geo::Pt found[4];
	for (int i = 0; i < 4; i++) {
		if (!s.fidRect.corner[i].set) return fit;
		found[i] = { s.fid0.xy.x + s.fidRect.corner[i].x, s.fid0.xy.y + s.fidRect.corner[i].y };
	}
	return gs2geo::fitRect(found, s.fidRect.widthMm, s.fidRect.lengthMm);
}

gs2geo::Pt fid0Taught(const Setup& s)
{
	return { s.fid0.xy.x, s.fid0.xy.y };
}

void slotCornersWorld(const Setup& s, const gs2geo::Pose& pose, const Slot& slot, gs2geo::Pt out[4])
{
	gs2geo::rectCorners({ slot.cx, slot.cy }, s.slotSize.widthMm, s.slotSize.lengthMm, out);
	for (int i = 0; i < 4; i++) out[i] = gs2geo::toWorld(pose, out[i]);
}

gs2geo::Pt unitCentreLocal(const Setup& s, const Slot& slot, int row, int col)
{
	const auto& u = s.unitLayout;
	const gs2geo::Pt c = gs2geo::unitCentre(row, col, u.rows, u.cols, u.widthMm, u.lengthMm, u.marginMm);
	return { slot.cx + c.x, slot.cy + c.y };
}

void unitCornersWorld(const Setup& s, const gs2geo::Pose& pose, const Slot& slot, int row, int col, gs2geo::Pt out[4])
{
	gs2geo::rectCorners(unitCentreLocal(s, slot, row, col), s.unitLayout.widthMm, s.unitLayout.lengthMm, out);
	for (int i = 0; i < 4; i++) out[i] = gs2geo::toWorld(pose, out[i]);
}

gs2geo::Pt inspectionWorld(const Setup& s, const gs2geo::Pose& pose, const Slot& slot, int row, int col, InspType t)
{
	const auto& in = s.insp[t];
	const gs2geo::Pt c = unitCentreLocal(s, slot, row, col);
	const gs2geo::Pt r = gs2geo::unitRefPoint(in.ref, s.unitLayout.widthMm, s.unitLayout.lengthMm);
	//the offset is stored in the trolley frame, like everything below the fidRect
	return gs2geo::toWorld(pose, { c.x + r.x + in.dx, c.y + r.y + in.dy });
}

const Slot* findSlot(const Setup& s, const QString& id)
{
	for (const auto& sl : s.slotList) if (sl.id == id) return &sl;
	return nullptr;
}

QStringList gridNames(const Setup& s)
{
	std::vector<double> xs, ys;
	for (const auto& u : s.units) {
		const Slot* sl = findSlot(s, u.slotId);
		const gs2geo::Pt c = sl ? unitCentreLocal(s, *sl, u.row, u.col) : gs2geo::Pt{};
		xs.push_back(c.x);
		ys.push_back(c.y);
	}
	//units in one column sit within a unit width of each other; half of it splits columns safely
	const double tolX = std::max(1.0, s.unitLayout.widthMm / 2);
	const double tolY = std::max(1.0, s.unitLayout.lengthMm / 2);
	const auto col = gs2geo::bandIndex(xs, tolX);
	const auto row = gs2geo::bandIndex(ys, tolY);
	QStringList names;
	for (size_t i = 0; i < xs.size(); i++) names << QStringLiteral("X%1Y%2").arg(col[i] + 1).arg(row[i] + 1);
	return names;
}

void syncUnits(Setup& s)
{
	QHash<QString, Unit> existing;
	for (const auto& u : s.units) existing.insert(QStringLiteral("%1/%2/%3").arg(u.slotId).arg(u.row).arg(u.col), u);

	QVector<Unit> next;
	for (const auto& sl : s.slotList)
		for (int r = 0; r < s.unitLayout.rows; r++)
			for (int c = 0; c < s.unitLayout.cols; c++) {
				const QString key = QStringLiteral("%1/%2/%3").arg(sl.id).arg(r).arg(c);
				if (existing.contains(key)) { next.append(existing.value(key)); continue; }
				Unit u;
				u.id = QStringLiteral("u%1").arg(s.nextUnitId++);
				u.slotId = sl.id;
				u.row = r;
				u.col = c;
				next.append(u);
			}
	s.units = next;

	const QStringList grid = gridNames(s);
	QSet<QString> used;
	for (const auto& u : s.units) if (!u.name.isEmpty()) used.insert(u.name);
	for (int i = 0; i < s.units.size(); i++) {
		auto& u = s.units[i];
		if (!u.name.isEmpty()) continue;
		u.name = used.contains(grid[i]) ? kTempPrefix + u.id : grid[i];
		used.insert(u.name);
	}
}

void nameAllByPosition(Setup& s)
{
	std::vector<double> ys;
	for (const auto& sl : s.slotList) ys.push_back(sl.cy);
	const auto band = gs2geo::bandIndex(ys, std::max(1.0, s.slotSize.lengthMm / 2));
	QVector<int> order;
	for (int i = 0; i < s.slotList.size(); i++) order << i;
	std::sort(order.begin(), order.end(), [&](int a, int b) {
		if (band[a] != band[b]) return band[a] < band[b];
		return s.slotList[a].cx < s.slotList[b].cx;
	});
	for (int k = 0; k < order.size(); k++) s.slotList[order[k]].name = QStringLiteral("S%1").arg(k + 1);

	const QStringList grid = gridNames(s);
	for (int i = 0; i < s.units.size(); i++) s.units[i].name = grid[i];
}

bool isTemporaryName(const QString& name)
{
	return name.startsWith(kTempPrefix);
}

bool naturalLess(const QString& a, const QString& b)
{
	int i = 0, j = 0;
	while (i < a.size() && j < b.size()) {
		if (a[i].isDigit() && b[j].isDigit()) {
			//digit runs compare as numbers without converting them (no overflow on long runs, so the
			//ordering stays consistent for std::sort): leading zeros skipped, longer run is bigger,
			//then the first different digit decides
			int i0 = i, j0 = j;
			while (i0 < a.size() && a[i0] == QLatin1Char('0')) i0++;
			while (j0 < b.size() && b[j0] == QLatin1Char('0')) j0++;
			int i2 = i0, j2 = j0;
			while (i2 < a.size() && a[i2].isDigit()) i2++;
			while (j2 < b.size() && b[j2].isDigit()) j2++;
			if (i2 - i0 != j2 - j0) return i2 - i0 < j2 - j0;
			for (int k = 0; k < i2 - i0; k++)
				if (a[i0 + k] != b[j0 + k]) return a[i0 + k] < b[j0 + k];
			i = i2; j = j2;
		}
		else {
			const QChar ca = a[i].toUpper(), cb = b[j].toUpper();
			if (ca != cb) return ca < cb;
			i++; j++;
		}
	}
	return a.size() - i < b.size() - j;
}

QString cameraCalPath(const QString& configDir)
{
	return QDir(configDir).filePath(QStringLiteral("guidedSetup2_camera.json"));
}

QJsonObject toJson(const CameraCal& c)
{
	QJsonObject o;
	o.insert(QStringLiteral("version"), kFileVersion);
	o.insert(QStringLiteral("use"), c.useMeasured ? QStringLiteral("measured") : QStringLiteral("set"));
	if (c.measured) {
		QJsonObject m;
		m.insert(QStringLiteral("mmPerPx"), QJsonArray{ QJsonArray{ c.a[0][0], c.a[0][1] }, QJsonArray{ c.a[1][0], c.a[1][1] } });
		m.insert(QStringLiteral("umPerPxX"), c.umPerPxX);
		m.insert(QStringLiteral("umPerPxY"), c.umPerPxY);
		m.insert(QStringLiteral("angleDeg"), c.angleDeg);
		m.insert(QStringLiteral("fitPx"), c.fitPx);
		m.insert(QStringLiteral("camAngleDeg"), c.camAngleDeg);
		m.insert(QStringLiteral("imageSize"), QJsonArray{ c.imgW, c.imgH });
		m.insert(QStringLiteral("at"), QJsonArray{ c.atX, c.atY, c.atZ });
		m.insert(QStringLiteral("when"), c.when);
		o.insert(QStringLiteral("measured"), m);
	}
	return o;
}

CameraCal cameraCalFromJson(const QJsonObject& o)
{
	CameraCal c;
	const QJsonObject m = o.value(QStringLiteral("measured")).toObject();
	const QJsonArray rows = m.value(QStringLiteral("mmPerPx")).toArray();
	const QJsonArray size = m.value(QStringLiteral("imageSize")).toArray();
	if (rows.size() == 2 && rows[0].toArray().size() == 2 && rows[1].toArray().size() == 2 && size.size() == 2) {
		for (int r = 0; r < 2; r++)
			for (int k = 0; k < 2; k++) c.a[r][k] = rows[r].toArray()[k].toDouble();
		c.umPerPxX = num(m, "umPerPxX", 0.0);
		c.umPerPxY = num(m, "umPerPxY", 0.0);
		c.angleDeg = num(m, "angleDeg", 0.0);
		c.fitPx = num(m, "fitPx", 0.0);
		c.camAngleDeg = num(m, "camAngleDeg", 0.0);
		c.imgW = size[0].toInt();
		c.imgH = size[1].toInt();
		const QJsonArray at = m.value(QStringLiteral("at")).toArray();
		if (at.size() == 3) { c.atX = at[0].toDouble(); c.atY = at[1].toDouble(); c.atZ = at[2].toDouble(); }
		c.when = str(m, "when");
		//a matrix that cannot be inverted, or no image size, is no measurement
		c.measured = std::abs(c.a[0][0] * c.a[1][1] - c.a[0][1] * c.a[1][0]) > 1e-12 && c.imgW > 0 && c.imgH > 0;
	}
	c.useMeasured = c.measured && str(o, "use") == QLatin1String("measured");
	return c;
}

bool loadCameraCal(const QString& path, CameraCal& out, QString& why)
{
	out = CameraCal();
	QFile f(path);
	if (!f.exists()) { why.clear(); return true; }
	if (!f.open(QIODevice::ReadOnly)) { why = QStringLiteral("cannot open %1: %2").arg(path, f.errorString()); return false; }
	QJsonParseError err;
	const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
	if (err.error != QJsonParseError::NoError || !doc.isObject()) {
		why = QStringLiteral("%1 is not valid JSON (%2)").arg(path, err.errorString());
		return false;
	}
	out = cameraCalFromJson(doc.object());
	why.clear();
	return true;
}

bool saveCameraCal(const QString& path, const CameraCal& c, QString& why)
{
	//all or nothing: a half-written matrix would misplace everything the circle finder converts
	QSaveFile file(path);
	if (!file.open(QIODevice::WriteOnly)) { why = QStringLiteral("cannot write %1: %2").arg(path, file.errorString()); return false; }
	file.write(QJsonDocument(toJson(c)).toJson(QJsonDocument::Indented));
	if (!file.commit()) { why = QStringLiteral("cannot write %1: %2").arg(path, file.errorString()); return false; }
	why.clear();
	return true;
}

} // namespace gs2
