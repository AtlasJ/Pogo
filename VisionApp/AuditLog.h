#pragma once
//
// AuditLog - lightweight, header-only audit trail for destructive / critical actions.
//
// Records WHO did WHAT, WHEN, and the OUTCOME to an append-only "audit.log" file
// (under Common::Directory::LocalPath) and mirrors the line to the main ct::logger.
//
// Usage:
//   AuditLog::instance().setUser(userName, accessLevel);   // once, on successful login
//   AuditLog::instance().log("RECIPE_OVERWRITE", recipeName);              // result defaults to "OK"
//   AuditLog::instance().log("RECIPE_OVERWRITE", recipeName, "FAILED");    // explicit result
//   AuditLog::instance().log("JOG", "X+0.5mm", "DENIED");                  // access-denied attempt
//
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QMutex>
#include <QMutexLocker>

#include "UserAccount.h"
#include "CommonDir.h"
#include "Logger.h"

class AuditLog
{
public:
	static AuditLog& instance()
	{
		static AuditLog inst;
		return inst;
	}

	// Call once after a successful login so subsequent actions are attributed.
	void setUser(const QString& userName, AccessLevel level)
	{
		QMutexLocker lock(&_mutex);
		_userName = userName.isEmpty() ? QStringLiteral("unknown") : userName;
		_accessLevel = levelToString(level);
	}

	// Call on logout / app close.
	void clearUser()
	{
		QMutexLocker lock(&_mutex);
		_userName = QStringLiteral("unknown");
		_accessLevel = QStringLiteral("unknown");
	}

	// Log a critical/destructive action.
	//   action  - short verb tag, e.g. "RECIPE_DELETE", "TEACH_POINT", "SERVO_TOGGLE"
	//   target  - what it acted on (recipe name, optic id, axis, file path...) - optional
	//   result  - "OK" (default), "FAILED", "DENIED", etc.
	bool log(const QString& action,
	         const QString& target = QString(),
	         const QString& result = QStringLiteral("OK"))
	{
		QMutexLocker lock(&_mutex);

		const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
		const QString line = QStringLiteral("%1 | user=%2 | level=%3 | action=%4 | target=%5 | result=%6")
			.arg(ts,
			     _userName,
			     _accessLevel,
			     action,
			     target.isEmpty() ? QStringLiteral("-") : target,
			     result);

		const QString auditPath = Common::Directory::LocalPath + QStringLiteral("audit.log");
		QFile f(auditPath);
		if (!f.open(QIODevice::Append | QIODevice::Text))
		{
			ct::logger::error("[AUDIT_WRITE_FAILED] path=%s error=%s entry=%s",
				auditPath.toStdString().c_str(),
				f.errorString().toStdString().c_str(),
				line.toStdString().c_str());
			return false;
		}

		QTextStream out(&f);
		out << line << "\n";
		out.flush();
		const bool streamOk = out.status() == QTextStream::Ok;
		const bool fileOk = f.flush();
		const QString fileError = f.errorString();
		f.close();

		if (!streamOk || !fileOk)
		{
			ct::logger::error("[AUDIT_WRITE_FAILED] path=%s error=%s entry=%s",
				auditPath.toStdString().c_str(),
				fileError.toStdString().c_str(),
				line.toStdString().c_str());
			return false;
		}

		ct::logger::info("[AUDIT] %s", line.toStdString().c_str());
		return true;
	}

private:
	AuditLog() = default;
	AuditLog(const AuditLog&) = delete;
	AuditLog& operator=(const AuditLog&) = delete;

	static QString levelToString(AccessLevel level)
	{
		switch (level)
		{
		case AccessLevel::ADMIN:    return QStringLiteral("Admin");
		case AccessLevel::ENGINEER: return QStringLiteral("Engineer");
		case AccessLevel::OPERATOR: return QStringLiteral("Operator");
		default:                    return QStringLiteral("Unknown");
		}
	}

	QString _userName = QStringLiteral("unknown");
	QString _accessLevel = QStringLiteral("unknown");
	QMutex _mutex;
};
