
#include "VisionApp.h"
#include <QtWidgets/QApplication>
#include "MessageQue.h"
#include "PostInspectionInfo.h"
#include "ResultCache.h"
#include "QFrameless.h"
#include <QSharedMemory>
#include "Timer.h"
#include "OnnxInference.h"
#include "FrameInfo.h"

#define _CRTDBG_MAP_ALLOC
#include <stdio.h>
#include <stdlib.h>
#include <crtdbg.h>

int g_viewMode;
MIL_INT g_imgType;
QString g_imgExtension;
int g_viewIndex;
bool g_forceStopInspLoop;
bool g_enableClassificationDataCollection;
QString g_bufferID;
ResultCache g_resultCache;
TMessageQue<PostInspectionInfo> g_postInspectionQueue;
TMessageQue<QVector<FrameInfo>> g_inspectionQueue;
TMessageQue<FrameInfo> g_imageQueue;
Timer g_time;
QHash<QString, QPointF> g_locatorOffsets;
QVector< Onnx::InferenceEngine*> g_ODModels;
Onnx::InferenceEngine* g_segModel;
ObjectDetectionTilingSettings g_odTilingSettings;

void Console()
{
	FreeConsole();
	AllocConsole();
	AttachConsole(GetCurrentProcessId());
	freopen("CON", "w", stdout);
	freopen("CON", "w", stderr);
	freopen("CON", "r", stdin);
}

int main(int argc, char *argv[])
{
	//timeBeginPeriod(1);
	//QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	//QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);


	////_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//QApplication* temp = new QApplication(argc, argv);
	//QRect screenGeometry = QApplication::desktop()->screenGeometry();
	//double width = screenGeometry.width();
	//
	//// assumes that the default desktop resolution is 720p (scale of 1)
	//int minWidth = 1920;
	//delete temp;

	//double scale = width / minWidth;
	//std::string scaleAsString = std::to_string(scale);
	//QByteArray scaleAsQByteArray(scaleAsString.c_str(), scaleAsString.length());
	//qputenv("QT_SCALE_FACTOR", scaleAsQByteArray);

	QApplication a(argc, argv);

	a.setAttribute(Qt::AA_EnableHighDpiScaling);
	a.setAttribute(Qt::AA_UseHighDpiPixmaps);

	QFile file(":/VisionApp/Resources/styleSheet.qss");
	file.open(QFile::ReadOnly);
	QString styleSheet = QLatin1String(file.readAll());

	a.setStyleSheet(styleSheet);
	file.close();

	Console();

	//GUID : Generated once for your application
	// you could get one GUID here: http://www.guidgenerator.com/online-guid-generator.aspx
	QSharedMemory shared("04252529-9604-42d3-ae61-cf2799e66c9a");

	if (!shared.create(512, QSharedMemory::ReadWrite))
	{
		// For a GUI application, replace this by :
		QMessageBox msgBox;
		msgBox.setText(QObject::tr("Can't start more than one instance of the application. Application is hidden in system tray icon at bottom right of desktop!!!"));
		msgBox.setIcon(QMessageBox::Critical);
		msgBox.exec();

		qWarning() << "Can't start more than one instance of the application.";

		exit(0);
	}
	else {
		qDebug() << "Application started successfully.";
	}

	VisionApp w;
	QList<QScreen*> screens = QGuiApplication::screens();
	// Check if the second screen is available
	if (screens.size() > 1) {
		QScreen *screen = screens.at(1); // Assuming the second screen is at index 1

		// Get the geometry of the second screen
		QRect screenGeometry = screen->geometry();
		w.setGeometry(screenGeometry);

		HWND hWndConsole = GetConsoleWindow();
		MoveWindow(hWndConsole, screenGeometry.x(), screenGeometry.y(), 1000, 1000, TRUE);
	}



	FrameLess f(w.window(), &w);
	w.showFullScreen();

	_CrtDumpMemoryLeaks();
	return a.exec();
}
