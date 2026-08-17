#pragma once

#include <iostream>
#include <functional>
#include <queue>
#include <thread>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>

Q_DECLARE_METATYPE(std::function<void()>)
class FunctionQueue : public QThread {
	Q_OBJECT

public:
	enum Type { SEQUENTIAL, WAIT, PARALLEL, DETACH };

	FunctionQueue(QObject* parent = nullptr) : QThread(parent) {}

	template <typename Func, typename... Args>
	void enqueue(Type type, QString command, Func&& func, Args&&... args) {
		FunctionPack pack;
		pack.type = type;
		pack.command = command;
		pack.func = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);
		
		if (m_queue.empty()) m_stop = false;

		if (m_stop) m_tempQueue.emplace(pack);
		else m_queue.emplace(pack);
	}

	template <typename Func, typename... Args>
	void enqueueLastTask(Type type, QString command, Func&& func, Args&&... args) {
		m_lastTask.type = type;
		m_lastTask.command = command;
		m_lastTask.func = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);
	}

	void run() override;
	void resume(QString command);
	void stop();
	void release();
	void skip(int num); //Skip next few queue
	QString getWaitCommand();

private:
	struct FunctionPack {
		Type type;
		std::function<void()> func;
		QString command;
	};

	QMutex m_mutex;
	QWaitCondition m_condition;
	std::queue<FunctionPack> m_queue;
	std::queue<FunctionPack> m_tempQueue;
	FunctionPack m_lastTask;
	bool m_stop = false;
	bool m_release = false;
	int m_skipNum = 0;
	QString m_waitCommand = "";
	
	void clear();

signals:
	void executeUIFunc(std::function<void()> func);
};

/*
=> Design
1. Thread need to sleep when no queue is running
2. Function pointer
3. Wait signal condition
*/

/*
=> Use case
- Able to support wait signal. Wait time should be flexible

-> Barcode
1. Jog to barcode teach point
2. Snap image
3. Process barcode
If has barcodeID
4.1. Use barcodeID create production folder
Else
4.2. Obtain from
5. Save barcode image (Parallel)

Run code in UI thread
Run code in UI thread with wait feature
Run code in Alt thread
Run code in Alt thread with wait feature??
Run code in detached thread
*/

//1st Design
//class FunctionQueue : public QThread {
//	Q_OBJECT
//
//public:
//	FunctionQueue(QObject* parent = nullptr) : QThread(parent){}
//
//	template <typename Func, typename... Args>
//	void enqueue(Func&& func, Args&&... args) {
//		QMutexLocker locker(&mutex);
//		m_queue.emplace(std::bind(std::forward<Func>(func), std::forward<Args>(args)...));
//		condition.wakeOne();
//	}
//
//	void run() override {
//		while (true) {
//			QMutexLocker locker(&mutex);
//
//			if (m_queue.empty()) condition.wait(&mutex);
//
//			while (!m_queue.empty()) {
//				auto& func = m_queue.front();
//				func();
//				m_queue.pop();
//			}
//		}
//	}
//
//	bool signal(QString command) {
//
//	}
//
//private:
//	QMutex mutex;
//	QWaitCondition condition;
//	std::queue<std::function<void()>> m_queue;
//};
