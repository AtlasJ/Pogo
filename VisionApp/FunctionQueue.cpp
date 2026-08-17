#include "FunctionQueue.h"
#include "Logger.h"

void FunctionQueue::run()
{

	ct::logger::info("[QThread] Function queue started");

	while (true) {

		if (m_release) {
			clear();
			return;
		}

		while (!m_queue.empty()) {

			if (m_release) {
				clear();
				return;
			}

			if (m_stop) {
				clear();
				//To fix issue when stop() is called but the loop is not fast enough to reach here first
				//As a result, if someone calls stop(), and immediately enqueue fncpack, it will clear it out if the code reach here after the enqueue
				while (!m_tempQueue.empty()) {
					m_queue.push(std::move(m_tempQueue.front()));
					m_tempQueue.pop();
				}

				m_stop = false;

				continue;
			}

			if (m_skipNum > 0) {
				m_queue.pop();
				m_skipNum--;
				continue;
			}

			auto& pack = m_queue.front();
			auto type = pack.type;
			auto command = pack.command;

			if (type == DETACH) {
				std::thread([=]() {
					pack.func();
				}).detach();
			}
			else if (type == PARALLEL) {
				pack.func();
			}
			else {
				emit executeUIFunc(pack.func);
			}

			m_queue.pop();

			if (type == WAIT) {
				QMutexLocker locker(&m_mutex);
				m_waitCommand = command;
				ct::logger::debug("[FQueue] Waiting for: %s", command.toStdString().c_str());
				m_condition.wait(&m_mutex);
			}

			if (m_queue.empty()) {
				emit executeUIFunc(m_lastTask.func);
			}
		}
	}
}

void FunctionQueue::resume(QString command)
{
	ct::logger::debug("[FQueue] Receive: %s", command.toStdString().c_str());

	if (command == m_waitCommand) {
		m_waitCommand = "NOT_WAITING";
		ct::logger::debug("[FQueue] Resume queue after receive: %s", command.toStdString().c_str());
		m_condition.wakeOne();
	}
}

void FunctionQueue::stop()
{
	m_stop = true;
	if (m_waitCommand != "NOT_WAITING") {
		m_waitCommand = "NOT_WAITING";
		m_condition.wakeOne();
	}
}

void FunctionQueue::release()
{
	m_release = true;
	if (m_waitCommand != "NOT_WAITING") {
		m_waitCommand = "NOT_WAITING";
		m_condition.wakeOne();
	}
}

void FunctionQueue::skip(int num)
{
	m_skipNum = num;
}

QString FunctionQueue::getWaitCommand()
{
	return m_waitCommand;
}

void FunctionQueue::clear()
{
	m_queue = {};
	m_waitCommand = "NOT_WAITING";
}
