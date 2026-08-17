#include "InactivityHandler.h"
#include <QDebug>

InactivityHandler::InactivityHandler(int timeoutSeconds, QObject* parent)
    : QObject(parent), timeoutSeconds(timeoutSeconds)
{
    // Create and configure the timer
    inactivityTimer = new QTimer(this);
    inactivityTimer->setInterval(timeoutSeconds * 1000); // Convert seconds to milliseconds
    inactivityTimer->setSingleShot(true);
    connect(inactivityTimer, &QTimer::timeout, this, &InactivityHandler::inactiveTimeout);

    // Start the timer initially
    inactivityTimer->start();
}

// Event filter to detect user activity and reset the timer
bool InactivityHandler::eventFilter(QObject* obj, QEvent* event)
{
    // Check for any user interaction events (keyboard, mouse, etc.)
    if (event->type() == QEvent::MouseButtonPress ||
        event->type() == QEvent::KeyPress ||
        event->type() == QEvent::MouseMove) {
        // Restart the timer on user activity
        inactivityTimer->start();
    }

    // Pass the event to the base class for normal processing
    return QObject::eventFilter(obj, event);
}

// Slot that is triggered when the user is inactive for the specified duration
void InactivityHandler::inactiveTimeout()
{
    qDebug() << "No activity for" << timeoutSeconds << "seconds. Triggering timeout function.";
    // Your inactive timeout logic here
}