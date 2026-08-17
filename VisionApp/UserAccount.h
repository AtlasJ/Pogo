#pragma once
#include <QObject>


enum AccessLevel {
	ADMIN,
	ENGINEER,
	OPERATOR
};

struct AccountInfo {
	QString userID;
	QString userName;
	QString password;   // plaintext only in memory (entry/new user); never persisted
	AccessLevel accessLevel = AccessLevel::OPERATOR;
	QString salt;       // per-user random salt (persisted)
	QString pwHash;     // SHA-256 hex of salt+password (persisted instead of password)
};
