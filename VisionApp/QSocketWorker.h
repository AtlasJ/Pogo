#pragma once
#include <QtWidgets>
#include <QObject>
#include <QTcpSocket>
#include "Logger.h"

class QSocketWorker : public QObject {
    Q_OBJECT
public:
    explicit QSocketWorker(QObject* parent = nullptr) : QObject(parent) {}

public slots:
    void init() {
        socket = new QTcpSocket(this);
        QObject::connect(socket, &QTcpSocket::connected, this, &QSocketWorker::connected);
        QObject::connect(socket, &QTcpSocket::readyRead, this, &QSocketWorker::onReadyRead);
        QObject::connect(socket, &QTcpSocket::disconnected, this, &QSocketWorker::disconnected);
    }
    void connectToHost(const QString& host, quint16 port) {
        socket->connectToHost(host, port);
    }
    void writeData(const QByteArray& data) {
        if (socket && socket->state() == QAbstractSocket::ConnectedState)
            socket->write(data);
            socket->flush();
            socket->waitForBytesWritten(300);
            if (socket->waitForReadyRead(300)) { // wait for the server to respond
            	//if (intensity != -1) m_lastIntensity[ch] = real_intensity;
            }
            else {
                socket->write(data);
                socket->flush();
                socket->waitForBytesWritten(300);
                socket->waitForReadyRead(300);
            	ct::logger::warn("LSC response timeout");
            }
    }
    void disconnectFromHost() { if (socket) socket->disconnectFromHost(); }

signals:
    void connected();
    void disconnected();
    void dataReceived(QByteArray);
    void error(QString);

private slots:
    void onReadyRead() { emit dataReceived(socket->readAll()); }

private:
    QTcpSocket* socket = nullptr;
};