#pragma once
#include <QObject>
#include <QMutex>
#include <QWaitCondition>

class QImageGrabber : public QObject {
	Q_OBJECT

public:
	QImageGrabber();
	~QImageGrabber();

	public slots:
	void process();
	void stop();
	void resume();

signals:
	void finished();
	void error(QString err);

	void grab();

private:
	bool m_stop = false;

	QMutex m_mutex;
	QWaitCondition m_condition;
};