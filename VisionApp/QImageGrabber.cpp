#include "QImageGrabber.h"
#include "QOSTool.h"

QImageGrabber::QImageGrabber() {
}

QImageGrabber::~QImageGrabber() {

}

void QImageGrabber::stop()
{
	m_stop = true;
}

void QImageGrabber::resume()
{
	m_condition.wakeOne();
}

void QImageGrabber::process() {

	m_stop = false;

	while (true) {
		if (m_stop) break;

		emit grab();

		QMutexLocker locker(&m_mutex);
		m_condition.wait(&m_mutex);
	}

	emit finished();
}