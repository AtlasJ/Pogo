#ifndef INACTIVITYHANDLER_H
#define INACTIVITYHANDLER_H

#include <QObject>
#include <QTimer>
#include <QEvent>

class InactivityHandler : public QObject
{
    Q_OBJECT  // Add this macro to enable Qt's meta-object system

public:
    // Constructor with timeout in seconds
    explicit InactivityHandler(int timeoutSeconds, QObject* parent = nullptr);

protected:
    // Event filter to capture user activity
    bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
    // Slot to handle inactivity timeout
    void inactiveTimeout();

private:
    QTimer* inactivityTimer;  // Timer for inactivity detection
    int timeoutSeconds;       // Timeout duration in seconds
};

#endif // INACTIVITYHANDLER_H