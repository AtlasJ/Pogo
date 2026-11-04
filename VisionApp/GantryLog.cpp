#include "GantryLog.h"
#include <QDateTime>
#include <QDir>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <share.h>
#include <string>
#define NOMINMAX
#include <windows.h>

namespace {
	std::mutex g_mutex;
	QString g_folder = QStringLiteral("log/");
	FILE* g_file = nullptr;
	QString g_fileHour;                //yyyyMMdd_HH of the open file: a new hour opens a new file, as the main log does
	thread_local std::string t_scope;  //Scope texts on this thread, " / " between them
}

namespace gantrylog {

void init(const QString& folder)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	g_folder = folder.isEmpty() ? QStringLiteral("log/") : folder;
	if (g_file) { fclose(g_file); g_file = nullptr; }
	g_fileHour.clear();
}

const char* axisName(int axis)
{
	static const char* const names[] = { "X", "Y", "Z", "A3", "A4", "A5", "A6", "A7" };
	return axis >= 0 && axis < 8 ? names[axis] : "A?";
}

void write(const char* fmt, ...)
{
	char msg[2048];
	va_list args;
	va_start(args, fmt);
	vsnprintf(msg, sizeof(msg), fmt, args);
	va_end(args);

	const QDateTime now = QDateTime::currentDateTime();
	const QString hour = now.toString(QStringLiteral("yyyyMMdd_HH"));

	std::lock_guard<std::mutex> lock(g_mutex);
	if (!g_file || hour != g_fileHour) {
		if (g_file) { fclose(g_file); g_file = nullptr; }
		QDir().mkpath(g_folder);
		const QString path = QDir(g_folder).filePath(QStringLiteral("gantry_%1.log").arg(hour));
		//shared: the file can be opened in an editor while the app writes it
		g_file = _wfsopen(reinterpret_cast<const wchar_t*>(path.utf16()), L"a", _SH_DENYNO);
		if (!g_file) return; //no log is better than a machine that stops over one
		g_fileHour = hour;
	}
	fprintf(g_file, "%s [T:%lu]%s%s%s %s\n", qPrintable(now.toString(QStringLiteral("yyyy-MM-dd hh:mm:ss.zzz"))),
		(unsigned long)GetCurrentThreadId(), t_scope.empty() ? "" : " [", t_scope.c_str(), t_scope.empty() ? "" : "]", msg);
	fflush(g_file); //a crash must not take the last moves with it
}

Scope::Scope(const QString& what)
	: m_prevLen((int)t_scope.size())
{
	if (!t_scope.empty()) t_scope += " / ";
	t_scope += what.toStdString();
}

Scope::~Scope()
{
	t_scope.resize(m_prevLen);
}

}
