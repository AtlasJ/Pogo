#include "VisionApp.h"
#include "SRXManager.h"
#include "AlgoManager.h"
#include "QCommonStruct.h"
#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include "VisionAppQDragBox.h"
#include "QDragBox.h"
#include "QCrossItem.h"
#include "QRectItem.h"
#include "QLineItem.h"
#include "QEllipseItem.h"
#include "QJsonHelper.h"
#include "QJsonFile.h"
#include <QItemSelectionModel>
#include <QtCharts/QPieSeries>
#include <QtCharts/QPieSlice>
#include <chrono>
#include <thread>
#include "ErrorMsg.h"
#include <iostream>
#include "QDragBoxCommand.h"
#include "ScopedTimeLogger.h"
#include "ExtendedMenu.h"
#include "VisionAppStruct.h"
#include "mtrx.h"
#include "BoxCluster.h"
#include "EM_TSP.h"
#include "LSC_VLP.h"
#include "CommonDir.h"
#include "uidGenerator.h"
#include "ImagePathManager.h"

#include "TemplateLibraryTab.h" 
#include "DatasetPage.h"
#include "ProductionPage.h"
#include "ImageViewerTab.h"
#include "UnitConfigTab.h"
#include "Motion.h"
#include "Guided_2D3D_AlignmentTab.h"
#include "3DOpticsTab.h"

#include <csignal>
#include <signal.h>
#include <windows.h>
#include <psapi.h>

#include "CAMManager.h"
#include "SystemData.h"

#include <QMetaType>

#include "ProfilerManager.h"
#include "ScaleManager.h"

#include "MbufPoolManager.h"
#include "MotionController.h"
#include "MachineController.h"
#include "ImageSavingThread.h"
#include "AuditLog.h"

#include <QGuiApplication>
#include <QScreen>

#include <vips/vips8>

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QHeaderView>
#include <QPushButton>
#include <QInputDialog>
#include <QMessageBox>
#include <QJsonArray>
#include <QCryptographicHash>
#include <QMessageAuthenticationCode>
#include <QUuid>

// Emergency recovery admin. Works even if user.json is missing or corrupt,
// so an admin can always get in to fix accounts. Change these here if needed.
static const QString EMERGENCY_ADMIN_USER = QStringLiteral("recovery");
static const QString EMERGENCY_ADMIN_PASS = QStringLiteral("3df");

// Secret key used to sign user.json (HMAC-SHA256). Any manual edit to the file
// invalidates the signature and the file is rejected on load. Keep this private.
static const QByteArray USER_SIGNING_KEY = QByteArrayLiteral("nVsion_SMT_userStore_v1_@k3y");

// SHA-256 hex of salt+password. Passwords are never stored in the clear.
static QString hashPassword(const QString& salt, const QString& password)
{
	QByteArray data = (salt + password).toUtf8();
	return QString::fromLatin1(QCryptographicHash::hash(data, QCryptographicHash::Sha256).toHex());
}

// HMAC-SHA256 over the compact JSON of the users array (deterministic key order).
static QString computeUsersSignature(const QJsonArray& users)
{
	QByteArray payload = QJsonDocument(users).toJson(QJsonDocument::Compact);
	QMessageAuthenticationCode mac(QCryptographicHash::Sha256);
	mac.setKey(USER_SIGNING_KEY);
	mac.addData(payload);
	return QString::fromLatin1(mac.result().toHex());
}

bool compareRectanglesByY_ascending(VisionAppQDragBox * rect1, VisionAppQDragBox * rect2)
{
	return rect1->getGeometry().y() < rect2->getGeometry().y();
}

bool compareRectanglesByX_ascending(VisionAppQDragBox * rect1, VisionAppQDragBox * rect2)
{
	return rect1->getGeometry().x() < rect2->getGeometry().x();
}

bool compareRectanglesByY_descending(VisionAppQDragBox * rect1, VisionAppQDragBox * rect2)
{
	return rect1->getGeometry().y() > rect2->getGeometry().y();
}

bool compareRectanglesByX_descending(VisionAppQDragBox * rect1, VisionAppQDragBox * rect2)
{
	return rect1->getGeometry().x() > rect2->getGeometry().x();
}

bool compareArea(int b1, int b2)
{
	return (b1 > b2);
}

VisionApp::VisionApp(QWidget *parent) : QMainWindow(parent)
{
	Onnx::InferenceEngine _infer; // initialize to see if onnx working

	ui.setupUi(this);
	std::signal(SIGSEGV, &VisionApp::terminated); //link: https://stackoverflow.com/questions/343219/is-it-possible-to-use-signal-inside-a-c-class
	std::signal(SIGINT, &VisionApp::terminated);
	std::signal(SIGILL, &VisionApp::terminated);
	std::signal(SIGFPE, &VisionApp::terminated);
	std::signal(SIGTERM, &VisionApp::terminated);
	std::signal(SIGBREAK, &VisionApp::terminated);
	std::signal(SIGABRT, &VisionApp::terminated);
	std::signal(SIGABRT_COMPAT, &VisionApp::terminated);
	std::set_terminate([]()
	{
		terminated(999);
	});

	g_viewMode = (int)ViewMode::PLANE;
	g_imgExtension = ".jpg";
	g_imgType = M_JPEG_LOSSY;

	auto log_root_path = jsonHelper::getString(_systemObj, QStringLiteral("Log_Folder"), "log/");
	Common::Directory::createDir(log_root_path);
	ct::logger::init(log_root_path.toStdString());
	ct::logger::set_level(ct::logger::Level::TRACE);


	QThreadPool::globalInstance()->setMaxThreadCount(1);

	if (VIPS_INIT("test")) {
		vips_error_exit("Failed to initialize libvips");
	}

	//visionAppDragBox
	_noViewIcon = new QIcon(":/8Icon/Icon/icon8/warning.png");
	_noViewPixmap = _noViewIcon->pixmap(_noViewIcon->actualSize(QSize(64, 64)));

	//motion control
	_motionControl = new Motion();

	//guided_2D3D_AlignmentTab
	_guided_2D3D_AlignmentTab = new Guided_2D3D_AlignmentTab();

	//dragBoxIcon
	_noViewIcon = new QIcon(":/8Icon/Icon/icon8/warning.png");
	_noViewPixmap = _noViewIcon->pixmap(_noViewIcon->actualSize(QSize(64, 64)));

	//templateLibraryTab
	_templateLibraryTab = new TemplateLibraryTab();
	ui.TemplateLibraryLayout->addWidget(_templateLibraryTab);

	//datasetPage
	_datasetPage = new DatasetPage();
	ui.gridLayout_DatasetPage->addWidget(_datasetPage);

	//productionPage
	_productionPage = new ProductionPage();
	ui.gridLayout_ProductionPage->addWidget(_productionPage);

	//imageViewerTab
	_imageViewerTab = new ImageViewerTab();
	ui.gridLayout_imageViewerTab->addWidget(_imageViewerTab);

	//unitConfigTab
	_unitConfigTab = new UnitConfigTab();
	ui.gridLayout_UnitConfigTab->addWidget(_unitConfigTab);
	connect(_unitConfigTab, &UnitConfigTab::runStoredUnits, this, &VisionApp::runStoredUnits);
	connect(_unitConfigTab, &UnitConfigTab::storeSkippedUnits, this, &VisionApp::storeSkippedUnits);
	connect(_unitConfigTab, &UnitConfigTab::removeSkippedUnits, this, &VisionApp::removeSkippedUnits);

	//3DOpticsTab
	_optics3DTab = new Optics3DTab();
	ui.gridLayout_3DOpticsTab->addWidget(_optics3DTab);
	_optics3DTab->attachRecipeOptics3D(&_recipeOptics3D);

	//recipeSettingsMenu
	_recipeSettingsMenu = new ExtendedMenu(0, 300, 300);
	_recipeSettingsMenu->setParent(this);
	_systemSettingsMenu = new ExtendedMenu(2, 300, 510); //6 rows of buttons + spacing
	_systemSettingsMenu->setParent(this);
	_rightMenu = new ExtendedMenu(1, 100, 500);
	_rightMenu->setParent(this);

	// goldenRecipeDialog
	_grDialog = new GoldenRecipeDialog();

	//loggerTab
	ui.gridLayout_16->removeWidget(ui.tabWidgetOutput);
	ui.tabWidgetOutput->setParent(this);
	ui.tabWidgetOutput->setMinimumHeight(500);
	//ui.tabWidgetOutput->setMaximumWidth(this->width() / 3.0);
	ui.tabWidgetOutput->move(0, this->height());
	auto graphicEff = new QGraphicsOpacityEffect();
	graphicEff->setOpacity(0.7);
	ui.tabWidgetOutput->setGraphicsEffect(graphicEff);
	ui.tabWidgetOutput->setAutoFillBackground(true);
	ui.tabWidgetOutput->hide();

	//actionMenu
	actionMenu.addAction(tr("Draw ROI"), this, SLOT(visionObjectMode()));
	actionMenu.addAction(tr("Select"), this, SLOT(selectMode()));


	//allocate the MIL application + host system (previously done inside Algo.dll)
	_algo.init();

	readUserInfo(_userObj);
	readSystemInfo(_systemObj);

	initVariable();
	loadPortabilityInfo();

	ct::logger::info("Initializing Camera...");
	iniCamera();

	OpticsControl::instance().attach(&_portabilityInfo);

	ct::logger::info("Initializing IO...");
	iniIOCard();

	ct::logger::info("Initializing Buffer Queue...");
	iniBufferQueue(); 

	ct::logger::info("Initializing 3D sensor...");
	if (!jsonHelper::getBool(_systemObj, QStringLiteral("Disable_Profiler"), false)) {
		ProfilerManager::instance().loadConfig(QStringLiteral("C:/Advanced/Data/config/profiler.json"));
		auto connected = ProfilerManager::instance().isConnected(_profilerID);
		nvs::set_background_color(ui.toolButton_3dProfilerStatus, connected ? Qt::green : Qt::red);
	}

	ct::logger::info("Initializing External Barcode Reader...");
	if (!jsonHelper::getBool(_systemObj, QStringLiteral("Disable_BarcodeReader"), false)) {
		// Loads barcodeReader.json, connects both readers and starts the image FTP server
		SRXManager::instance().init();
	}

	ct::logger::info("Initializing read buffer...");
	readBufferInfo(_bufferInfoObj);

	ct::logger::info("Initializing Event and Shared Memory...");
	createEventAndSharedMemory(_bufferInfoObj); 

	ct::logger::info("Initializing Display Buffer...");
	assignDisplayBuffer(_bufferInfoObj); 

	//intialize Algorithm
	ct::logger::info("Initializing Algorithms...");
	_algo.createBuffer(_bufferInfoObj);

	_emapReEnableTimer.setInterval(1000);
	_emapReEnableTimerSeconds = 60;
	connect(&_emapReEnableTimer, &QTimer::timeout, this, [=]() {
		_emapReEnableTimerSeconds--;
		ui.label_emapTimer->setStyleSheet("color: red");
		ui.label_emapTimer->setText("Emap Re-enable in: " + QString::number(_emapReEnableTimerSeconds));
		if (_emapReEnableTimerSeconds == 0)
		{
			ui.checkBox_enableEmap->setChecked(true);
			_emapReEnableTimerSeconds = 60;
			_emapReEnableTimer.stop();
			ui.label_emapTimer->clear();
		}
	});
	
	ct::logger::info("Initializing UI Widgets..."); initWidget();
	ct::logger::info("Initializing Display size..."); initDisplaySize();
	ct::logger::info("Initializing Image Display widget..."); initImageDisplayWidget();
	ct::logger::info("Initializing signal slot..."); connectSignalAndSlot();
	ct::logger::info("Initializing TCPIP..."); initTCPIP();
	ct::logger::info("Initializing CT client..."); initCT_Client();
	ct::logger::info("Initializing Emap Setting..."); loadEmapSetting();
	ct::logger::info("Initializing LSC..."); initLSC(); 
	ct::logger::info("Initializing Motion..."); initMotion();

	ct::logger::info("Initializing Remaining Systems...");
	initNamingConvention(); ct::logger::info("Initialized naming convention");
	initTeachPoint(); ct::logger::info("Initialized teach point");
	initFiducial(); ct::logger::info("Initialized fiducial");
	initBarcode(); ct::logger::info("Initialized barcode");
	initBarcodeReaderPage(); ct::logger::info("Initialized barcode reader page");
	AlgoManager::instance().init();
	AlgoManager::instance().loadRecipeConfig();
	initAlgoSetupPage();
	refreshAlgoSetupPage(); ct::logger::info("Initialized algo setup page");
	initDryRunPage(); ct::logger::info("Initialized dry run page");
	initRecipeSetupZStack();
	initStitchingMethod();
	initPortability(); ct::logger::info("Initialized portability");
	loadWorldEnv(); ct::logger::info("Initialized world environment");
	initManualScalingSM(); ct::logger::info("Initialized scaling");
	initPathSM(); ct::logger::info("Initialized path");
	initConfig();  ct::logger::info("Initialized config");
	initLaserUI(); ct::logger::info("Initialized laser UI");
	initAnalysis(); ct::logger::info("Initialized analysis");
	refreshLightingTemplateComboBox(); ct::logger::info("Refresh lighting template");
	initGifIcon(); ct::logger::info("Initialized Gif");
	addObjectDetectionModels();
	


	fadeIn();

	//UI 
	// tempo hide
	ui.radioButton_useLocalEmapSetting->hide();
	ui.label_26->hide();
	ui.lineEdit_namingPrefix->hide();
	ui.label_27->hide();
	ui.lineEdit_namingPostfix->hide();
	ui.label_33->hide();
	ui.lineEdit_rowColumnRepresentation->hide();
	ui.toolButton_5->hide();
	ui.comboBox_namingMethod->setEnabled(true);
	ui.groupBox_eMap->hide();

	// naming convention widget init
	QIntValidator* validator_row = new QIntValidator(ui.lineEdit_rowStartingIndex);
	ui.lineEdit_rowStartingIndex->setValidator(validator_row);
	QIntValidator* validator_col = new QIntValidator(ui.lineEdit_colStartingIndex);
	ui.lineEdit_colStartingIndex->setValidator(validator_col);

	//validator for recipe Z height
	// RECIPE_Z_CONVEYOR_DISABLED_BEGIN
	// Recipe-based 3D Z offset UI is disabled; hide controls while old Update Z
	// behavior is active.
	//ui.lineEdit_3DZoffset->setValidator(new QDoubleValidator(this));
	ui.label_44->hide();
	ui.lineEdit_3DZoffset->hide();
	// RECIPE_Z_CONVEYOR_DISABLED_END

	//createTrayIcon
	createTrayIcon();
	initMovieIcons();

	initImageFiltering();
	initProductionUI();

	ct::logger::info("Initializing Startup State...");
	initStartupState();

	//Note: load recipe stuff after this line
	ct::logger::info("Loading Recipe...");
	openRecipe(jsonHelper::getString(_systemObj, QStringLiteral("Recent_Open_Recipe")), false);
	//load configs
	loadLaserConfig();
	initViewEditor();

	_networkPathChecker.setIpAddress(jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")));
	_networkPathCheckerTimer.setInterval(1000); // Set the interval for checking changes (in milliseconds)
	QObject::connect(&_networkPathCheckerTimer, SIGNAL(timeout()), this, SLOT(checkVerificationIp()));
	_networkPathCheckerTimer.start();

	

	iniFileRemover();
	updateSetupCheckList();
	setupGoldenRecipeTimer();
	ct::logger::info("Initialization Completed!");

	qRegisterMetaType<FrameInfo>();
	QObject::connect(&_imageManager, &ImageManager::imageReceived, this, &VisionApp::imageReceived, Qt::QueuedConnection);
	QObject::connect(&_imageManager, &ImageManager::imagePreprocessed, this, &VisionApp::imagePreprocessed, Qt::QueuedConnection);
	QObject::connect(&_imageManager, &ImageManager::imagePreprocessed, &_jobThread, &JobThread::imagePreprocessed, Qt::QueuedConnection);
	qRegisterMetaType<QVector<FrameInfo>>();
	QObject::connect(&_imageManager, &ImageManager::imageReady, this, &VisionApp::imageReady, Qt::QueuedConnection);
	QObject::connect(&_imageManager, &ImageManager::imageReady, &_jobThread, &JobThread::imageReady, Qt::QueuedConnection);
	_imageManager.attach(&_fiducial);
	_imageManager.start(QThread::HighPriority);

	connectMachineController();
	connectJobThread();
	OpticsControl::instance().toggleAllChannels(false);

	QString version = QCoreApplication::applicationVersion();
	ui.label_appVersion->setText(" v" + version);

	//testcase
	//testcase_fiducialLogic();
	//testcase_mbufpool();

	toggleWorldView();
}

void VisionApp::initRecipeSetupZStack()
{
	QSignalBlocker b1(ui.comboBox_acqType);
	QSignalBlocker b2(ui.lineEdit_step_um);
	QSignalBlocker b3(ui.lineEdit_Iteration);
	QSignalBlocker b4(ui.checkBox_Generate2dImage);

	ui.comboBox_acqType->setCurrentIndex(0);
	ui.lineEdit_step_um->setText("100");
	ui.lineEdit_Iteration->setText("1");
	ui.checkBox_Generate2dImage->setChecked(false);
}

void VisionApp::saveRecipeSetupZStack()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/fiducial_barcode_zStack.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QJsonObject json;
	json["acqType"] = ui.comboBox_acqType->currentText();
	json["step_um"] = ui.lineEdit_step_um->text().toInt();
	json["iteration"] = ui.lineEdit_Iteration->text().toInt();
	json["generate2dImage"] = ui.checkBox_Generate2dImage->isChecked();

	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly)) {
		QJsonDocument doc(json);
		file.write(doc.toJson(QJsonDocument::Indented));
		file.close();
	}

	for (auto& fidInfo : _fiducialInfos)
	{
		fidInfo.acq_type = ui.comboBox_acqType->currentText();
		fidInfo.step_um = ui.lineEdit_step_um->text().toInt();
		fidInfo.preset_iteration = ui.lineEdit_Iteration->text().toInt();
		fidInfo.generate_2D_stack = ui.checkBox_Generate2dImage->isChecked();
	}

	for (auto& barcodeInfo : _barcodeInfos)
	{
		barcodeInfo.acq_type = ui.comboBox_acqType->currentText();
		barcodeInfo.step_um = ui.lineEdit_step_um->text().toInt();
		barcodeInfo.preset_iteration = ui.lineEdit_Iteration->text().toInt();
		barcodeInfo.generate_2D_stack = ui.checkBox_Generate2dImage->isChecked();
	}
}

void VisionApp::loadRecipeSetupZStack()
{
	auto jsonPath = QStringLiteral("%1recipe/%2/fiducial_barcode_zStack.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

	QFile file(jsonPath);
	if (!file.open(QIODevice::ReadOnly))
		return;

	QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
	file.close();
	if (!doc.isObject())
		return;

	QJsonObject json = doc.object();

	ui.comboBox_acqType->blockSignals(true);
	if (json.contains("acqType"))
	{
		QString type = json["acqType"].toString();
		int index = ui.comboBox_acqType->findText(type);
		if (index != -1)
			ui.comboBox_acqType->setCurrentIndex(index);
	}
	ui.comboBox_acqType->blockSignals(false);

	ui.lineEdit_step_um->blockSignals(true);
	if (json.contains("step_um"))
		ui.lineEdit_step_um->setText(QString::number(json["step_um"].toInt()));
	ui.lineEdit_step_um->blockSignals(false);

	ui.lineEdit_Iteration->blockSignals(true);
	if (json.contains("iteration"))
		ui.lineEdit_Iteration->setText(QString::number(json["iteration"].toInt()));
	ui.lineEdit_Iteration->blockSignals(false);

	ui.checkBox_Generate2dImage->blockSignals(true);
	if (json.contains("generate2dImage"))
		ui.checkBox_Generate2dImage->setChecked(json["generate2dImage"].toBool());
	ui.checkBox_Generate2dImage->blockSignals(false);

	for (auto& fidInfo : _fiducialInfos)
	{
		fidInfo.acq_type = ui.comboBox_acqType->currentText();
		fidInfo.step_um = ui.lineEdit_step_um->text().toInt();
		fidInfo.preset_iteration = ui.lineEdit_Iteration->text().toInt();
		fidInfo.generate_2D_stack = ui.checkBox_Generate2dImage->isChecked();
	}

	for (auto& barcodeInfo : _barcodeInfos)
	{
		barcodeInfo.acq_type = ui.comboBox_acqType->currentText();
		barcodeInfo.step_um = ui.lineEdit_step_um->text().toInt();
		barcodeInfo.preset_iteration = ui.lineEdit_Iteration->text().toInt();
		barcodeInfo.generate_2D_stack = ui.checkBox_Generate2dImage->isChecked();
	}
	
}

void VisionApp::updateConnection(bool connection)
{
	qDebug() << "Connecting status: " << connection;
	ui.label_verificationLoc->setText(jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")));
	_isVerificationConnected = connection;
	_databaseThread.updateConnectionStatus(_isVerificationConnected);
	QString defectCollectorPath;
	QString recipeCollectorPath;
	QString userDataBasePath;
	QIcon icon;
	if (connection)
	{
		icon.addFile(":/8Icon/Icon/icon8/icons8-green circle-96.png");
		_dataBasePath = jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")) + "/Advanced/Data/database_3df.db";
		
		userDataBasePath = jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")) + "/Advanced/Data/userDatabase.db";
		Common::Directory::setVerificationIpAddress(jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")));

		defectCollectorPath = jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")) + "/Advanced/Data/DefectCollector/";
		recipeCollectorPath = jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")) + "/Advanced/Data/RecipeCollector/";
	}
	else
	{
		// connect to local if verification station doesnt exist
		icon.addFile(":/8Icon/Icon/icon8/icons8-red circle-96.png");
		_dataBasePath = "C:/Advanced/Data/database_3df.db";
	
		userDataBasePath = "C:/Advanced/Data/userDatabase.db";


		defectCollectorPath = "C:/Advanced/Data/DefectCollector/";
		recipeCollectorPath = "C:/Advanced/Data/RecipeCollector/";
	}

	_databaseThread.setDefectCollectorPath(defectCollectorPath);
	_databaseThread.setRecipeCollectorPath(recipeCollectorPath);


	if (!_sqliteDatabase.open(_dataBasePath))
	{
		qDebug() << "Failed to open database";
	}
	else
	{
		qDebug() << "Successfully open database";
	}
	

	// User accounts are now stored in config/user.json (see loadUserAccounts).
	Q_UNUSED(userDataBasePath);

	ui.toolButton_verificationCenterStatus->setIcon(icon);
}

void VisionApp::checkVerificationIp()
{
	_networkPathChecker.start();
}

void VisionApp::createCamAlpha()
{
	if (_camAlpha)
	{
		delete[] _camAlpha; _camAlpha = nullptr;
	}

	int camIndex = 0;

	auto w = CAMManager::instance().getWidth(_camID);
	auto h = CAMManager::instance().getHeight(_camID);

	_camAlpha = util::generateAlphaImage(w, h, SystemData::instance()._camAngles[_camID]);
}

void VisionApp::setEditMode(EditMode mode)
{
	_editMode = mode;

	switch (mode) {
	case EditMode::SELECT:
		printf("Edit: Select\n");
		_cursor.setShape(Qt::ArrowCursor);
		break;
	case EditMode::VISION_OBJECT:
		_cursor.setShape(Qt::CrossCursor);
		break;
	case EditMode::PATH_ASSIGNMENT:
		printf("Edit: Path\n");
		_cursor.setShape(Qt::UpArrowCursor);
		break;
	case EditMode::POSITION_PORTABILITY_MODE:
		_cursor.setShape(Qt::CrossCursor);
		break;
	}

	ui.graphicsViewMain->setCursor(_cursor);
	QCoreApplication::processEvents();
}

QColor VisionApp::getColor(Representation r)
{
	switch (r) {
	case Representation::ASSIGNED_VIEW:
		return QColor(0, 255, 127);
	case Representation::UNASSIGNED_VIEW:
		return QColor(150, 150, 150);
	case Representation::ASSIGNED_VO:
		return QColor(0, 191, 255);
	case Representation::UNASSIGNED_VO:
		return QColor(150, 150, 150);
	}

	return QColor();
}

bool VisionApp::loadComponentCadRois(const QString & filePath, QVector<CadRoiInfo>& cadRois)
{
	QJsonObject root;

	QString val;
	QFile file;
	QJsonDocument doc;

	bool flag = false;
	if (QFile::exists(filePath))
	{
		file.setFileName(filePath);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			flag = true;
		}
	}

	if (!root.contains("CadRois"))
	{
		flag = false;
	}

	cadRois.clear();
	auto cadRoiInfos = root["CadRois"].toArray();

	for (int i = 0; i < cadRoiInfos.size(); i++) {
		auto cadRoiInfo = cadRoiInfos[i].toObject();
		CadRoiInfo cadRoi;
		cadRoi.familyId = jsonHelper::getString(cadRoiInfo, QStringLiteral("familyId"));
		cadRoi.familyName = jsonHelper::getString(cadRoiInfo, QStringLiteral("familyName"));
		cadRoi.name = jsonHelper::getString(cadRoiInfo, QStringLiteral("name"));
		cadRoi.id = jsonHelper::getString(cadRoiInfo, QStringLiteral("id"));
		cadRoi.x = jsonHelper::getDouble(cadRoiInfo, QStringLiteral("x"));
		cadRoi.y = jsonHelper::getDouble(cadRoiInfo, QStringLiteral("y"));
		cadRoi.w = jsonHelper::getDouble(cadRoiInfo, QStringLiteral("w"));
		cadRoi.h = jsonHelper::getDouble(cadRoiInfo, QStringLiteral("h"));
	
		cadRois.append(cadRoi);
	}


	return flag;
}

void VisionApp::clearEmptyViewKey()
{
	//clear any ghost views
	for (auto it = _views.begin(); it != _views.end(); ) {
		if (it.key().isEmpty())
			it = _views.erase(it);  // erase returns the next iterator
		else
			++it;
	}
}

QStringList VisionApp::get2DLightingUsed()
{
	//templates no longer carry lighting IDs (Algo library removed)
	return QStringList();
}

QStringList VisionApp::get3DLightingUsed()
{
	//templates no longer carry lighting IDs (Algo library removed)
	return QStringList();
}

void VisionApp::loadEmapSetting()
{
	if (g_viewMode == (int)ViewMode::SINGLE) return;

	QString station = jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address"));
	if (!NetworkPathChecker::isReachable(station))
	{
		ct::logger::warn("Verification station unreachable, skip loading Emap template");
		return;
	}

	QString path = station + "/Advanced/Data/config/EmapTemplate.json";
	_emapTemplateList.clear();
	QJsonFile jsonFile;
	if (!jsonFile.load(path))
	{
		ct::logger::warn("Emap template json not exist!");
		return;
	}
	QJsonArray emapTemplateArray;
	emapTemplateArray = jsonFile.getArray("Emap_Template_List");

	for (auto emapTemplate : emapTemplateArray)
	{
		QJsonObject eObject = emapTemplate.toObject();
		EmapInfo eInfo;
		eInfo.templateName = eObject["Template_Name"].toString();

		// incoming emap
		eInfo.mode = static_cast<EmapMode>(eObject["Incoming_Mode"].toInt());
		eInfo.topInspEmap = static_cast<EmapType>(eObject["Incoming_Top_Insp"].toInt());
		eInfo.botInspEmap = static_cast<EmapType>(eObject["Incoming_Bot_Insp"].toInt());
		QJsonArray arrTxt = eObject["Incoming_Text_file_Dir"].toArray();
		QJsonArray arrCsv = eObject["Incoming_Csv_file_Dir"].toArray();
		for (auto t : arrTxt)eInfo.textFileEmapDir.append(t.toString());
		for (auto c : arrCsv)eInfo.csvEmapDir.append(c.toString());

		_emapTemplateList.insert(eInfo.templateName, eInfo);
	}

	if (_emapTemplateList.contains(_emapTemplate))_emapInfo = _emapTemplateList[_emapTemplate];
	refreshEmapSettingUi();

}

bool VisionApp::readDefectPriorityList()
{
	//QString filePath = Common::Directory::ConfigPath() + "GlobalTagName.json";
	//QString filePath = jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")) + "/Advanced/Data/config/GlobalTagName.json";
	/*QJsonObject root;

	QString val;
	QFile file;
	QJsonDocument doc;

	bool flag = false;
	if (QFile::exists(filePath))
	{
		file.setFileName(filePath);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			flag = true;
		}
	}

	if (root.contains("Tag_Name_List"))
	{
		flag = true;

		_defectPriorityList.clear();
		auto tagNameList = root["Tag_Name_List"].toArray();

		for (int i = 0; i < tagNameList.size(); i++)
		{
			for (int j = 0; j < tagNameList.size(); j++)
			{
				auto tagNameObj = tagNameList[j].toObject();
				int rank = jsonHelper::getInteger(tagNameObj, QStringLiteral("Ranking"));

				if (rank == i+1)
				{
					_defectPriorityList.append(jsonHelper::getString(tagNameObj, QStringLiteral("TagName")));
				}

			}
		}
	}

	return flag;*/


	QString station = jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address"));
	QString filePath = station + "/Advanced/Data/config/GlobalTagName.json";

	QVector<QString> defectPriorityList;
	_tagNameHash.clear();
	_defectMappingHash.clear();
	_defectPriorityList.clear();

	if (!NetworkPathChecker::isReachable(station))
	{
		ct::logger::warn("Verification station unreachable, skip loading global tag names");
		return false;
	}
	QJsonFile tagNameFile;
	if (tagNameFile.load(filePath))
	{
		auto tagNameList = tagNameFile.getArray("Tag_Name_List");

		QVector<DefectTag> defectTags;

		for (int i = 0; i < tagNameList.size(); i++)
		{
			auto t = tagNameList[i].toObject();

			DefectTag dt;
			dt.tName = t["TagName"].toString();
			dt.tCode = t["TagCode"].toString();
			dt.tColor = t["Color"].toString();
			dt.tRank = t["Ranking"].toInt();
			dt.forceFail = t["ForceFail"].toBool();

			if (dt.tRank == 0) dt.tRank = i + 1;
			dt.tCode = dt.tCode.isEmpty() ? "00000" : dt.tCode;

			_tagNameHash.insert(dt.tName, dt);
			defectTags.append(dt);
		
		}

		// Sort the defectTags by ranking (tRank)
		std::sort(defectTags.begin(), defectTags.end(), [](const DefectTag& a, const DefectTag& b) {
			return a.tRank < b.tRank;
		});

		// Populate _defectPriorityList with the sorted names
		for (const auto& dt : defectTags)
		{
			defectPriorityList.append(dt.tName);
		}

		_defectPriorityList = defectPriorityList;


		auto systemDefectList = tagNameFile.getArray("System_Defect_List");
		for (int i = 0; i < systemDefectList.size(); i++)
		{
			auto t = systemDefectList[i].toObject();

			DefectMapping dm;
			dm.systemDefect = t["SystemDefect"].toString();
			dm.defectName = t["DefectName"].toString();

			_defectMappingHash.insert(dm.systemDefect, dm.defectName);
			
		}

		return true;
	}
	else
	{
	
		return false;
	}

}

void VisionApp::initVariable()
{
	Common::Directory::setupRootDir();
	_workingMode = WorkingMode::Offline;
	_pGraphicsSceneMain = nullptr;
	_pPixmapItemMain = nullptr;
	_pRecipeItem = nullptr;
	_currentObjectID.clear();

	_isIOCardOpened = false;

	_undoStack = new QUndoStack(this);
}

void VisionApp::initWidget()
{
	_viewIcon = new QIcon(":/24x24/Icon/24x24/cil-wallpaper.png");
	_objectIcon = new QIcon(":/8Icon/Icon/icon8/icons8-ic-100.png");
	_passIcon = new QIcon(":/8Icon/Icon/icon8/checked.png");
	_failIcon = new QIcon(":/8Icon/Icon/icon8/close.png");

	ui.actionImageGrab->setEnabled(true);
	ui.treeViewRecipeExplorer->setModel(&_recipeModel);
	ui.treeViewObjectExplorer->setModel(&_objectModel);
	ui.treeViewObjectExplorer->setItemDelegate(&_objectDelegate);
	ui.treeViewResultExplorer->setModel(&_resultModel);

	ui.treeViewRecipeExplorer->setHeaderHidden(true);
	ui.treeViewResultExplorer->setHeaderHidden(true);
	ui.treeViewObjectExplorer->setHeaderHidden(false);

	_recipeSelectionModel = ui.treeViewRecipeExplorer->selectionModel();
	_resultSelectionModel = ui.treeViewResultExplorer->selectionModel();

	ui.stackedWidget->setCurrentIndex(0);

	ui.toolButton_prevImage->hide();
	ui.lineEdit_currentImageIndex->hide();
	ui.toolButton_nextImage->hide();

	ui.frame_leftMenuBar->setMinimumWidth(0);
	ui.frame_leftMenuBar->setMaximumWidth(0);
	ui.frame_top->hide();
	_blockEventFilter = true;
	ui.statusBar->hide();

	ui.frame_rightTab->setMinimumWidth(0);
	ui.frame_rightTab->setMaximumWidth(0);
	ui.frame_rightTab->hide();

	ui.frame_leftTab->setMinimumWidth(0);
	ui.frame_leftTab->setMaximumWidth(0);
	ui.frame_leftTab->hide();

	ui.toolButtonWorkingMode->setEnabled(false);
	ui.tabWidgetOutput->setCurrentIndex(0);
	ui.stackedWidgetViewSelection->setCurrentIndex(2);

	ui.labelLoginStatus->setStyleSheet("color: #f76b1c;");

	setMouseTracking(true);
	auto widgetLists = this->findChildren<QWidget*>();

	foreach(QWidget *w, widgetLists)
	{
		w->setMouseTracking(true);
	}

	// Emap Config 
	QStringList inspType = {
		"CSV 01 EMAP",
		"CSV 34 EMAP",
		"TEXT FILE EMAP"
	};
	ui.comboBox_emapTopInsp->addItems(inspType);
	if (_emapInfo.topInspEmap == EmapType::CSV01_EMAP) ui.comboBox_emapTopInsp->setCurrentIndex(ui.comboBox_emapTopInsp->findText("CSV 01 EMAP"));
	else if (_emapInfo.topInspEmap == EmapType::CSV34_EMAP)ui.comboBox_emapTopInsp->setCurrentIndex(ui.comboBox_emapTopInsp->findText("CSV 34 EMAP"));
	else if (_emapInfo.topInspEmap == EmapType::TEXT_FILE_EMAP)ui.comboBox_emapTopInsp->setCurrentIndex(ui.comboBox_emapTopInsp->findText("TEXT FILE EMAP"));
	
	ui.comboBox_emapBotInsp->addItems(inspType);
	if (_emapInfo.botInspEmap == EmapType::CSV01_EMAP) ui.comboBox_emapBotInsp->setCurrentIndex(ui.comboBox_emapBotInsp->findText("CSV 01 EMAP"));
	else if (_emapInfo.botInspEmap == EmapType::CSV34_EMAP)ui.comboBox_emapBotInsp->setCurrentIndex(ui.comboBox_emapBotInsp->findText("CSV 34 EMAP"));
	else if (_emapInfo.botInspEmap == EmapType::TEXT_FILE_EMAP)ui.comboBox_emapBotInsp->setCurrentIndex(ui.comboBox_emapBotInsp->findText("TEXT FILE EMAP"));
	
	if (_enableEmap) ui.frame_emapSetting->show();
	else ui.frame_emapSetting->hide();

	qApp->installEventFilter(this);

	showStatus(QStringLiteral("Ready"), 0);
}

void VisionApp::initImageDisplayWidget()
{
	//Main view
	ui.graphicsViewMain->setRenderHint(QPainter::Antialiasing, false);
	ui.graphicsViewMain->setDragMode(QGraphicsView::RubberBandDrag);
	ui.graphicsViewMain->setOptimizationFlags(QGraphicsView::DontSavePainterState);
	ui.graphicsViewMain->setCacheMode(QGraphicsView::CacheBackground);
	ui.graphicsViewMain->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
	ui.graphicsViewMain->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

	_sceneBound.setRect(0, 0, _imageSize.width(), _imageSize.height());
	_pGraphicsSceneMain = new QMainGraphicsScene(QRectF());
	_imageMain = QImage(_imageSize.width(), _imageSize.height(), QImage::Format_RGB32);
	_pixmapMain = QPixmap(_imageSize.width(), _imageSize.height());
	_pixmapMain.fill(QColor(50, 50, 50));
	_pGraphicsSceneMain->setSceneRect(_sceneBound);
	ui.graphicsViewMain->setScene(_pGraphicsSceneMain);
	_pPixmapItemMain = _pGraphicsSceneMain->addPixmap(_pixmapMain);
	ui.graphicsViewMain->show();

	_pGraphicsSceneMain->addItem(&_worldFOV);

	auto cam_w = CAMManager::instance().getWidth(_camID);
	auto cam_h = CAMManager::instance().getHeight(_camID);
	auto rect = QRectF(0, 0, cam_w, cam_h);
	_worldFOV.setup(rect, QColor(230, 230, 230), "FOV");
	_worldFOV.setZValue((int)UIHierarchy::DRAGGABLES);
	_worldFOV.setDragable(false);
	_worldFOV.hide();

	//FOV view
	ui.graphicsViewFOV->setRenderHint(QPainter::Antialiasing, false);
	ui.graphicsViewFOV->setDragMode(QGraphicsView::RubberBandDrag);
	ui.graphicsViewFOV->setOptimizationFlags(QGraphicsView::DontSavePainterState);
	ui.graphicsViewFOV->setCacheMode(QGraphicsView::CacheBackground);
	ui.graphicsViewFOV->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
	ui.graphicsViewFOV->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	ui.graphicsViewFOV->setMinimumSize(400, 300);

	_sceneFOV.setRect(0, 0, cam_w, cam_h);
	_pGraphicsSceneFOV = new QMainGraphicsScene(QRectF());
	_imageFOV = QImage(cam_w, cam_h, QImage::Format_RGB32);
	_pixmapFOV = QPixmap(cam_w, cam_h);
	_pixmapFOV.fill(QColor(50, 50, 50));
	_pGraphicsSceneFOV->setSceneRect(_sceneFOV);
	ui.graphicsViewFOV->setScene(_pGraphicsSceneFOV);
	_pPixmapItemFOV = _pGraphicsSceneFOV->addPixmap(_pixmapFOV);
	ui.graphicsViewFOV->fitInView(_pPixmapItemFOV, Qt::KeepAspectRatio);
	ui.graphicsViewFOV->centerOn(_pPixmapItemFOV);
	ui.graphicsViewFOV->show();


	auto setRegion = [=](const QRectF & rect, const QColor & color, const QString & name, QDragBox& db) {
		db.setup(rect, color, name);
		db.setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
		db.setDragable(true);
		db.setZValue((int)UIHierarchy::DRAGGABLES);
	};

	_pGraphicsSceneFOV->addItem(&_commonDragBox);
	auto sr = QRectF(cam_w * 0.3, cam_h * 0.3, cam_w * 0.7, cam_h * 0.7);
	setRegion(sr, QColor(0, 255, 127), "", _commonDragBox);
	_commonDragBox.hide();
}

void VisionApp::initTCPIP()
{
	//_server.run(QHostAddress::Any, jsonHelper::getInteger(_systemObj, QStringLiteral("AOI_Listening_Port")));
	_client.setServerEndPoint(QHostAddress(jsonHelper::getString(_systemObj, QStringLiteral("IM310_IP_Address"))), jsonHelper::getInteger(_systemObj, QStringLiteral("IM310_Listening_Port")));
}

void VisionApp::initStartupState()
{
	//610 UI
	//ui.toolButton_snapMono->setVisible(false);
	//ui.toolButton_snapColor->setVisible(false);
	//ui.toolButton_addColorOptic->setVisible(false);
	//ui.toolButton_highlightSameViews->setVisible(false);
	//ui.frame_opticColor->setVisible(false);
	//ui.toolButton_calibrationDropDown->setVisible(false);
	//ui.label_53->setVisible(false);
	//ui.line->setVisible(false);
	//ui.toolButton_setAwbRegion->setVisible(false);
	//ui.label_54->setVisible(false);
	//ui.lineEdit_expectedCH1->setVisible(false);
	//ui.lineEdit_expectedCH2->setVisible(false);
	//ui.lineEdit_expectedCH3->setVisible(false);
	//ui.toolButton_sampleFromRegion->setVisible(false);
	//ui.label_88->setVisible(false);
	//ui.lineEdit_obtainedIntensity->setVisible(false);
	//ui.toolButton_getIntensityFromExpectedGV->setVisible(false);
	//ui.label_85->setVisible(false);
	//ui.label_86->setVisible(false);
	//ui.toolButton_learnGoldenLightingProfile->setVisible(false);
	//ui.toolButton_learnCurrentLightingProfile->setVisible(false);

	////config ui
	//ui.checkBox_enableFiducial->setVisible(false);
	//ui.checkBox_enableRmsRecipe->setVisible(false);
	//ui.checkBox_enableMounterChecking->setVisible(false);
	//ui.checkBox_enableGoldenRecipeChecking->setVisible(false);
	//ui.groupBox_eMap->setVisible(false);
	//ui.groupBox_2->setVisible(false);

	////zstack ui
	//ui.label_146->setVisible(false);
	//ui.label_3->setVisible(false);
	//ui.comboBox_zstack_acqType->setVisible(false);
	//ui.groupBox_acqType_preset->setVisible(false);
	//ui.groupBox_acqType_encoder->setVisible(false);
	//ui.checkBox_zstack_2D->setVisible(false);
	//ui.checkBox_zstack_3D->setVisible(false);
	//ui.comboBox_zstack_acqType->setCurrentText("");
	ui.groupBox_acqType_preset->show();
	ui.groupBox_acqType_encoder->hide();
	ui.groupBox_acqType_time->hide();

	//Code any mode expected during start up here
	showCrossHair(false);
	toggleFidROISetupMode(false);
	toggleBarcodeROISetupMode(false);

	//ui.comboBox_namingMethod->setCurrentIndex(2);
	//allowOnlyIslandNamingConvention();
	ui.stackedWidget_lightingType->setCurrentIndex(0);

	updateAllChannels();
	for (const auto& id : LSCManager::instance().channels()) {
		OpticsControl::instance().toggleChannel(id, false);
	}

	//Set machine mode
	//_inspectionThread.setCountMode(InspectionThread::CountMode::INDEX);

	ui.comboBox_cameraSelection->setCurrentText(_camID);
	updateCameraTypeUI(_camID);
	updatePortabilityFeatureUI(0);


	//Hide add view function. Need to do inverse fiducial for this feature
	uidGenerator idGen;
	QString viewName = QString("view") + idGen.id().c_str();
	ui.lineEdit_viewName->setText(viewName);
	ui.lineEdit_viewName->hide();
	ui.toolButton_addView->hide();
	ui.label_157->hide();


	//block combobox scrolling for safety reasons
	ui.comboBox_barcodeType->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_circleColor->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_CSALocatorOptic->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_emapBotInsp->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_emapTemplate->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_emapTopInsp->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_FeatureLearning->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_fiducialMethod->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_warpageMethod->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_zstack_acqType->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_stitchingMethod ->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_lineScanAxis->installEventFilter(new util::ComboScrollBlocker());
	ui.comboBox_lineScanDirection->installEventFilter(new util::ComboScrollBlocker());
}

QDragBox* VisionApp::addDragBoxToScene(QMainGraphicsScene* scene, QRect rect, const QColor& color, const QString& name, const QString& id)
{
	QDragBox* p = new QDragBox();

	p->setup(rect, color, name, id);
	p->setOutterBarrier(_pGraphicsSceneFOV->sceneRect());
	p->setDragable(true);
	p->setZValue((int)UIHierarchy::DRAGGABLES);
	scene->addItem(p);

	return p;
}

void VisionApp::deleteDragBox(QMainGraphicsScene* scene, QDragBox* p)
{
	if (p) p->deleteLater();
	p = nullptr;
}

void VisionApp::initCT_Client()
{
	_ctClient = new CT::CT_Client;
	connectToServer();
}

//=========================================================== V9Link ===========================================================
static void receiveCommand(const char* command, QTcpSocket* sender, void* data)
{
	qDebug() << "ReceiveCommand";
	auto p = (VisionApp*)data;

	//qDebug() << "command:" << command;
	QString Pcommand = command;
	QStringList messageList = Pcommand.split("|");
	if (Pcommand.contains("users"))
	{
		qDebug() << "Connected to server";
	}
	else if (messageList.size() > 3)
	{
		QString fnc = messageList[2];
		if (messageList[3] != "Done")
		{
			if (fnc != "task" || fnc != "training" || fnc == "subsampling") p->sendReplyReceived(command);

			if (false) {}
			else if (fnc == "Open")
			{
				p->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
				p->show();
				Qt::WindowFlags flags = p->windowFlags();
				if (flags.testFlag(Qt::WindowStaysOnTopHint)) {
					QApplication::processEvents();
					p->setWindowFlag(Qt::WindowStaysOnTopHint, false);
					p->showExe();
				}
			}
			else if (fnc == "RunGoldenRecipeUnitCompleted")
			{
				p->goldenRecipeRunComplete();
			}
		}
	}
}

void VisionApp::connectToServer()
{
	if (!QFile::exists("CTLinkApp.exe"))
	{
		ui.toolButton_connectServer->setIcon(*(new QIcon(":/8Icon/Icon/icon8/icons8-globeDisconnected-100.png")));
		ui.toolButton_connectServer->setToolTip("disconnected from server");

		QMessageBox::warning(this, tr("Missing CTLinkApp"),
			tr("CTLinkApp.exe is missing. Please install CTLinkApp.exe!!!"));
		return;
	}

	bool connected = false;
	for (int i = 0; i < 5; i++)
	{
		_ctClient->connectCTServer("localhost", 5100, "VisionApp", receiveCommand, this);
		if (_ctClient->waitForConnection(1000))
		{
			ui.toolButton_connectServer->setIcon(*(new QIcon(":/8Icon/Icon/icon8/icons8-globeConnected-100.png")));
			ui.toolButton_connectServer->setToolTip("connected to server");
			connected = true;
			break;
		}
		else
		{
			//ui.actionconnected->setText("disconnected");
			ui.toolButton_connectServer->setIcon(*(new QIcon(":/8Icon/Icon/icon8/icons8-globeDisconnected-100.png")));
			ui.toolButton_connectServer->setToolTip("disconnected from server");
		}
		qDebug() << "retry number " << i;
	}
	if (!connected)
	{
		ShellExecute(NULL, L"open", L"CTLinkApp.exe", NULL, NULL, SW_SHOWDEFAULT);
		connectToServer();
	}

	return;
}

void VisionApp::showExe()
{
	ShowWindow(GetConsoleWindow(), SW_SHOW);
	show();
	maximizedWindow();
}

void VisionApp::hideExe()
{
	ShowWindow(GetConsoleWindow(), SW_HIDE);
	hide();
}

void VisionApp::quitExe()
{
	this->close();
}

bool VisionApp::EXE_ExistTest(LPCWSTR exeName)
{
	HWND hwnd;
	hwnd = FindWindow(NULL, exeName);
	if (hwnd != 0) {
		return true;
	}
	else {
		return false;
	}
}

void VisionApp::sendReplyReceived(QString message)
{
	QString command;
	QStringList messageList = message.split("|");
	if (messageList.size() > 2)
	{
		command = messageList[1] + "|" + messageList[0] + "|" + messageList[2] + "|Done";
	}

	if (!command.isEmpty())
	{
		_ctClient->sendCommand(command);

	}
}

void VisionApp::connectSignalAndSlot()
{
	//========================================================================= Left Menu Bar =================================================================================================
	connect(ui.toolButtonMenu, SIGNAL(clicked()), this, SLOT(toggleMenu()));
	connect(ui.toolButtonRecipeSettings, SIGNAL(clicked()), this, SLOT(showRecipeSettingsMenu()));

	//Motion Control
	connect(ui.toolButton_toggleMotionControl, &QToolButton::clicked, this, [=]() {
		/*if (ui.toolButton_toggleMotionControl->isChecked()) _motionControl->show(); 
		else _motionControl->hide();*/

		if (ui.frame_rightTab->isHidden() || !isPage(UIPage::MOTION)) showRightTab((int)UIPage::MOTION, "");

		ui.stackedWidgetViewSelection->setCurrentIndex(5);
		ui.stackedWidget->setCurrentIndex(18);
		ui.frame_leftTab->hide();

		//ui.toolButton_toggleMotionControl->setChecked(false);
	});

	//CLEANUP:
	connect(_motionControl, &Motion::jogBack, this, [=]() { if (!blockJogSignal()) return; emit jogBack(_jogDistance, _mainOptics[_camID]); });
	connect(_motionControl, &Motion::jogBottom, this, [=]() { if (!blockJogSignal()) return; emit jogDown(_jogDistance, _mainOptics[_camID]); });
	connect(_motionControl, &Motion::jogFront, this, [=]() { if (!blockJogSignal()) return; jogFront(_jogDistance, _mainOptics[_camID]); });
	connect(_motionControl, &Motion::jogLeft, this, [=]() { if (!blockJogSignal()) return; emit jogLeft(_jogDistance, _mainOptics[_camID]); });
	connect(_motionControl, &Motion::jogRight, this, [=]() { if (!blockJogSignal()) return; emit jogRight(_jogDistance, _mainOptics[_camID]); });
	connect(_motionControl, &Motion::jogTop, this, [=]() { if (!blockJogSignal()) return; emit jogUp(_jogDistance, _mainOptics[_camID]); });
	connect(_motionControl, &Motion::jogTo, this, [=](double x, double y, double z) { jogTo(x, y, z); });
	connect(_motionControl, &Motion::stepChanged, this, [=](double step) { _jogDistance = step; });
	connect(this, &VisionApp::signalEncoderChanged, this, [=](double x, double y, double z) { _motionControl->setCurrentPoint(x, y, z); });

	connect(_guided_2D3D_AlignmentTab, &Guided_2D3D_AlignmentTab::updateLaserOffset, this, [=](dat::WorldCoordinate offset) {
		//_guided_2D3D_AlignmentTab->hide();
		_laserConfig.offset.wx = offset.wx;
		_laserConfig.offset.wy = offset.wy;
		_laserConfig.offset.wz = offset.wz;

		updateLaserOffsetUI(_laserConfig.offset);
		saveLaserConfig();
		//loadLaserConfig();
		showMsg("Done Guided alignment.");
	});

	connect(_guided_2D3D_AlignmentTab, &Guided_2D3D_AlignmentTab::updatePositionPortabilityOffset, this, [=](dat::WorldCoordinate offset) {
		
		qDebug() << "[updatePositionPortabilityOffset] offset:" << offset.wx << "," << offset.wy << "," << offset.wz;
		SystemData::instance()._portability.current_info.portability_point = SystemData::instance()._portability.ref_info.portability_point + offset;
		auto pPoint = SystemData::instance()._portability.current_info.portability_point;
		qDebug() << "[updatePositionPortabilityOffset] portability_point:" << pPoint.wx << "," << pPoint.wy << "," << pPoint.wz;
		SystemData::instance()._portability.current_info.offset_point = offset;

		savePositionPortabilityInfo(PositionPortabilityType::CURRENT);
		loadCurPositionPortabilityInfo();

		openRecipe(Common::Directory::CurrentRecipe);
		showMsg("Done Guided Position Portability alignment.");
		});

	connect(_guided_2D3D_AlignmentTab, &Guided_2D3D_AlignmentTab::motionJogUp, this, [=](double step) {
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		if (!blockJogSignal()) return; emit jogUp(step, _mainOptics[_camID]);
		});

	connect(_guided_2D3D_AlignmentTab, &Guided_2D3D_AlignmentTab::motionJogDown, this, [=](double step) {
		if (notAllowToAccess(AccessLevel::OPERATOR)) return;

		if (!blockJogSignal()) return; emit jogDown(step, _mainOptics[_camID]);
		});

	
	

	connect(&_databaseThread, SIGNAL(signalDatabaseStatus(bool)), this, SLOT(slotDatabaseStatus(bool)));

	//TemplateLibraryTab
	connect(ui.actionAdd_Vision_Object_as_Default_Template, SIGNAL(triggered()), this, SLOT(setVisionObjectAsDefaultTemplate()));
	connect(_templateLibraryTab, SIGNAL(updateVisionObjectTemplate(AlgoTemplate*)), this, SLOT(updateVisionObjectTemplate(AlgoTemplate*)));
	connect(_templateLibraryTab, SIGNAL(updateVisionObjectSize(AlgoTemplate*)), this, SLOT(updateVisionObjectSize(AlgoTemplate*)));
	connect(_templateLibraryTab, SIGNAL(updateVisionObjectColor(AlgoTemplate*)), this, SLOT(updateVisionObjectColor(AlgoTemplate*)));
	connect(_templateLibraryTab, SIGNAL(deleteVisionObjectTemplate(const QString &)), this, SLOT(deleteVisionObjectTemplate(const QString &)));
	connect(_templateLibraryTab, SIGNAL(showMsg(const QString&, QMessageBox::StandardButtons)), this, SLOT(showMsg(const QString&, QMessageBox::StandardButtons)));
	connect(_templateLibraryTab, SIGNAL(generateVIDIImages(AlgoTemplate*, bool)), this, SLOT(generateVIDIImages(AlgoTemplate*, bool)));
	connect(_templateLibraryTab, SIGNAL(addVisionObjectPadding(AlgoTemplate*, int)), this, SLOT(addVisionObjectPadding(AlgoTemplate*, int)));
	connect(_templateLibraryTab, SIGNAL(editTemplateSignal()), this, SLOT(editTemplate()));
	connect(_templateLibraryTab, SIGNAL(saveTemplateReferenceImage(AlgoTemplate*)), this, SLOT(saveTemplateReferenceImage(AlgoTemplate*)));
	connect(_templateLibraryTab, SIGNAL(signalOpenGoldenRecipeDialog()), this, SLOT(openGoldenRecipeDialog()));

	//ImageViewerTab
	connect(_imageViewerTab, SIGNAL(displayCurrentView(QString, QString)), this, SLOT(displayCurrentView(QString, QString)));
	connect(ui.comboBox_ImageOptics, SIGNAL(activated(int)), this, SLOT(displayCurrentView()));
	
	//RecipeSettingsMenu
	connect(_recipeSettingsMenu, SIGNAL(recipeSettingsMenuBtnPressed(int)), this, SLOT(recipeSettingsMenuBtnPressed(int)));
	connect(_systemSettingsMenu, SIGNAL(systemSettingsMenuBtnPressed(int)), this, SLOT(systemSettingsMenuBtnPressed(int)));
	connect(ui.toolButtonSystemSettings, SIGNAL(clicked()), this, SLOT(showSystemSettingsMenu()));
	connect(ui.toolButtonOpenImage, &QToolButton::clicked, this, [&]() { promptLoadImageType(); });
	connect(ui.toolButton_userAccount, SIGNAL(clicked()), this, SLOT(loginMode()));
	connect(ui.toolButton_ImageViewer, SIGNAL(clicked()), this, SLOT(toPageLeft()));

	//WorkingPage
	connect(ui.toolButtonWorkingMode, SIGNAL(clicked()), this, SLOT(showSetupPage()));

	//ProductionPage
	connect(ui.toolButton_ProductionMode, SIGNAL(clicked()), this, SLOT(showProductionPage()));

	//DatasetPage
	connect(ui.toolButtonDatasetPage, SIGNAL(clicked()), this, SLOT(showDatasetPage()));
	connect(_datasetPage, SIGNAL(refreshDatasetView()), this, SLOT(refreshDatasetView()));
	connect(_datasetPage, SIGNAL(displayCurrentView(QString, QString, QString)), this, SLOT(displayCurrentView(QString, QString, QString)));
	connect(_datasetPage, SIGNAL(runStoredUnitsInspection(QStringList)), this, SLOT(runStoredUnitsInspection(QStringList)));
	 
	connect(ui.toolButton_prevImage, SIGNAL(clicked()), this, SLOT(showPreviousImage()));
	connect(ui.toolButton_nextImage, SIGNAL(clicked()), this, SLOT(showNextImage()));

	connect(ui.checkBox_enableFiducial, &QCheckBox::stateChanged, this, [=](int state) {
		enableFiducial((bool)state);
	});
	connect(ui.checkBox_enable3D, &QCheckBox::stateChanged, this, [=](int state) {
		_enable3D = (bool)state;
		saveRecipeConfig();
	});
	connect(ui.checkBox_disable2D, &QCheckBox::stateChanged, this, [=](int state) {
		_enable2D = state != Qt::Checked;
		saveRecipeConfig();
	});
	connect(ui.checkBox_enableVisionObjectSampling, &QCheckBox::stateChanged, this, [=](int state) {
		_enableVisionObjectSampling = (bool)state;
		saveRecipeConfig();
	});
	
	connect(ui.doubleSpinBox_passYieldPerc, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [&](double value) {
		_passYieldPerc = value;
		saveRecipeConfig();
		
	});

	connect(ui.toolButton_addView, &QToolButton::clicked, this, [&]() {
		addView();
		AuditLog::instance().log(QStringLiteral("VIEW_ADD"));
	});

	connect(ui.checkBox_saveDefectVoImg, &QCheckBox::stateChanged, this, [=](int state) {
		_saveDefectVoImg = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Save_Defect_Vo_Image", _saveDefectVoImg);
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_saveDefectRectVoImg, &QCheckBox::stateChanged, this, [=](int state) {
		_saveDefectRectVoImg = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Save_Defect_Rect_Vo_Image", _saveDefectRectVoImg);
		updateSystemInfo(_systemObj);
	});


	connect(ui.checkBox_EnableSaveInspectionImage, &QCheckBox::stateChanged, this, [=](int state) {
		enableSaveInspectionImage((bool)state);
	});

	connect(ui.checkBox_enableEmap, &QCheckBox::stateChanged, this, [=](int state) {
		_enableEmap = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Enable_Emap", _enableEmap);
		updateSystemInfo(_systemObj);

		if (_enableEmap) ui.frame_emapSetting->show();
		else ui.frame_emapSetting->hide();

		if (!_enableEmap)
		{
			//_emapReEnableTimer.start();
		}
		else
		{
			/*_emapReEnableTimerSeconds = 60;
			_emapReEnableTimer.stop();
			ui.label_emapTimer->clear();*/
		}
	});

	connect(ui.checkBox_enableRmsRecipe, &QCheckBox::stateChanged, this, [=](int state) {
		_enableRmsRecipe = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Enable_RMS_Recipe", _enableRmsRecipe);
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_enableMounterChecking, &QCheckBox::stateChanged, this, [=](int state) {
		_enableMounterChecking = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Enable_Mounter_Checking", _enableMounterChecking);
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_enableGoldenRecipeChecking, &QCheckBox::stateChanged, this, [=](int state) {
		_enableGoldenRecipeChecking = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Enable_Golden_Recipe_Checking", _enableGoldenRecipeChecking);
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_saveUnstackedImages, &QCheckBox::stateChanged, this, [=](int state) {
		SystemData::instance()._saveUnstackedImages = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Save_Unstacked_Images", (bool)SystemData::instance()._saveUnstackedImages);
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_saveUnstitchedImages, &QCheckBox::stateChanged, this, [=](int state) {
		SystemData::instance()._saveUnstitchedImages = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Save_Unstitched_Images", (bool)SystemData::instance()._saveUnstitchedImages);
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_useRecipeScaling, &QCheckBox::stateChanged, this, [=](int state) {
		SystemData::instance()._useRecipeScale = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Use_Recipe_Scale", (bool)SystemData::instance()._useRecipeScale);
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_usedAsRecipe1, &QCheckBox::stateChanged, this, [=](int state) {
		if (state == Qt::Checked) {
			QSignalBlocker blocker(ui.checkBox_usedAsRecipe2);
			ui.checkBox_usedAsRecipe2->setChecked(false);
		}
		jsonHelper::setJsonValue(_systemObj, "Used_As_Recipe1", ui.checkBox_usedAsRecipe1->isChecked());
		jsonHelper::setJsonValue(_systemObj, "Used_As_Recipe2", ui.checkBox_usedAsRecipe2->isChecked());
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_usedAsRecipe2, &QCheckBox::stateChanged, this, [=](int state) {
		if (state == Qt::Checked) {
			QSignalBlocker blocker(ui.checkBox_usedAsRecipe1);
			ui.checkBox_usedAsRecipe1->setChecked(false);
		}
		jsonHelper::setJsonValue(_systemObj, "Used_As_Recipe1", ui.checkBox_usedAsRecipe1->isChecked());
		jsonHelper::setJsonValue(_systemObj, "Used_As_Recipe2", ui.checkBox_usedAsRecipe2->isChecked());
		updateSystemInfo(_systemObj);
	});

	if (ui.checkBox_usedAsRecipe1->isChecked() && ui.checkBox_usedAsRecipe2->isChecked()) {
		QSignalBlocker blocker(ui.checkBox_usedAsRecipe2);
		ui.checkBox_usedAsRecipe2->setChecked(false);
	}

	connect(ui.checkBox_psp, &QCheckBox::stateChanged, this, [=](int state) {
		SystemData::instance()._psp = state;
		jsonHelper::setJsonValue(_systemObj, "PSP", (bool)SystemData::instance()._psp);
		updateSystemInfo(_systemObj);

		if (SystemData::instance()._psp) {
			CAMManager::instance().setTriggerOutput(_camID, "", "Line2");
		}
		else {
			CAMManager::instance().setTriggerOutput(_camID, "", "Software");
		}
	});

	connect(ui.checkBox_machineDebugMode, &QCheckBox::stateChanged, this, [=](int state) {
		SystemData::instance()._machineDebugMode = state;
		jsonHelper::setJsonValue(_systemObj, "Machine_Debug_Mode", (bool)SystemData::instance()._machineDebugMode);
		updateSystemInfo(_systemObj);

		if (SystemData::instance()._machineDebugMode) {
			MotionController::instance().enable_motion(true);
		}
		else {
			if (MachineController::instance().getMachineState() != MachineState::READY) {
				MotionController::instance().enable_motion(false);
			}
		}
	});

	connect(ui.checkBox_recipeDryRun, &QCheckBox::stateChanged, this, [=](int state) {
		_dryRun = state;
	});

	connect(ui.checkBox_bypassInspectionMode, &QCheckBox::stateChanged, this, [=](int state) {
		SystemData::instance()._bypassInspection = (bool)state;
		jsonHelper::setJsonValue(_systemObj, "Bypass_Inspection_Mode", (bool)SystemData::instance()._bypassInspection);
		updateSystemInfo(_systemObj);
	});

	connect(ui.checkBox_enableFiducialRotate, &QCheckBox::stateChanged, this, [=](int state) {
			SystemData::instance()._enableFiducialRotate = (bool)state;
			jsonHelper::setJsonValue(_systemObj, "Enable_Fiducial_Rotate", (bool)SystemData::instance()._enableFiducialRotate);
			updateSystemInfo(_systemObj);

		});

	connect(ui.checkBox_doubleFiducialChecking, &QCheckBox::stateChanged, this, [=](int state) {
			SystemData::instance()._doubleFiducialChecking = (bool)state;
			saveRecipeConfig();
		});

	connect(ui.pushButton_resetTotalBoardInspection, &QPushButton::clicked, [=]() {
		if (!passwordPromptCorrect()) return;
		SystemData::instance()._BoardEntryQty = 0;
		ui.lineEdit_totalBoardInspection->setText(QString::number(SystemData::instance()._BoardEntryQty));
	});


	//Right Menu Bar
	connect(_rightMenu, &ExtendedMenu::showPropertyTab, this, [=]() { toPage(UIPage::RECIPE); });
	connect(_rightMenu, &ExtendedMenu::showTemplateLibraryTab, this, [=]() { toPage(UIPage::TEMPLATE_LIB); });
	connect(_rightMenu, SIGNAL(showLogTab()), this, SLOT(showLogTab()));
	connect(_rightMenu, &ExtendedMenu::showRecipeSetupTab, this, [=]() { toPage(UIPage::RECIPE_SETUP); });
	connect(_rightMenu, &ExtendedMenu::showVisionObjectTab, this, [=]() { toPage(UIPage::ROI_EDITOR); });
	connect(_rightMenu, &ExtendedMenu::showPathTab, this, [=]() { 		toPage(UIPage::PATH); });
	connect(_rightMenu, &ExtendedMenu::showNamingConvention, this, [=]() { toPage(UIPage::NAMING_CONVENTION); });
	connect(_rightMenu, &ExtendedMenu::showUnitConfigTab, this, [=]() {toPage(UIPage::UNIT_CONFIG);});

	//Top Menu Bar
	connect(ui.toolButton_minimize, &QToolButton::clicked, this, [=]() {showMinimized(); });
	connect(ui.toolButton_maximize_restore, SIGNAL(clicked()), this, SLOT(maximize_restoreWindow()));
	connect(ui.toolButton_close, SIGNAL(clicked()), this, SLOT(closeWindow()));
	connect(ui.toolButton_exit, SIGNAL(clicked()), this, SLOT(closeWindow()));

	connect(ui.toolButton_runOffline, &QToolButton::clicked, this, [&]() {_datasetIndexIds.clear(); resetLoopFlags(); promptInspSelection(); });
	connect(ui.toolButton_stopRun, &QToolButton::clicked, this, [&]() { userClickStopRun(); });
	connect(ui.toolButton_testRun, &QToolButton::clicked, this, [&]() { testRun(); });
	connect(ui.toolButtonSnap, &QToolButton::clicked, this, [&]() { 
		stopLiveView();
		ct::logger::info("Camera: %s", _camID.toStdString().c_str());

		CAMManager::instance().resetFrame(_camID);
		
		if (!ui.toolButton_toggleDualView->isChecked() && !ui.toolButton_toggleFovView->isChecked()) {
			toggleFOVView();
		}

		emit snapImage(_mainOptics[_camID], "main", "");
	});

	ui.toolButtonLiveMode->setChecked(false);
	ui.toolButton_circleCrosshair->setChecked(false);
	connect(ui.toolButtonLiveMode, &QToolButton::clicked, this, [&]() { toggleLiveView(); });
	connect(ui.toolButton_circleCrosshair, &QToolButton::clicked, this, [&]() { toggleCircleCrosshair(); });
	connect(ui.toolButtonSaveImage, SIGNAL(clicked()), this, SLOT(saveImage()));

	connect(ui.toolButton_saveLighting, &QToolButton::clicked, this, [&]() { saveRecipeOptics(); AuditLog::instance().log(QStringLiteral("OPTICS_SAVE")); });
	connect(ui.toolButton_runAutoCalibration, &QToolButton::clicked, this, [&]() { openAutoCalibrationRecipe(); });
	connect(ui.toolButton_updateSnapDelay, &QToolButton::clicked, this, [&]() {
		auto ms = ui.lineEdit_snapDelay_ms->text().toInt();
		SystemData::instance()._snapDelay_ms = ms;
		_systemObj.insert(QStringLiteral("Snap_Delay(ms)"), ms);
		updateSystemInfo(_systemObj);
	});
	
	// Lighting Template
	connect(ui.toolButton_importOptic, &QToolButton::clicked, this, [&]() { importOptic(); AuditLog::instance().log(QStringLiteral("OPTIC_IMPORT")); });

	connect(ui.toolButton_ligtingTemplateSave, SIGNAL(clicked()), this, SLOT(saveAsLightingTemplate()));
	connect(ui.toolButton_ligtingTemplateImport, SIGNAL(clicked()), this, SLOT(loadLightingTemplate()));
	connect(ui.toolButton_ligtingTemplateDelete, SIGNAL(clicked()), this, SLOT(deleteLightingTemplate()));
	connect(ui.toolButton_ligtingTemplateUpdate, SIGNAL(clicked()), this, SLOT(updateLightingTemplate()));

	ui.treeViewRecipeExplorer->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(ui.graphicsViewMain, SIGNAL(mouseMove(QPoint)), this, SLOT(mouseMove(QPoint)));
	connect(ui.graphicsViewMain, SIGNAL(mousePress(QPointF, bool)), this, SLOT(graphicsViewMousePress(QPointF, bool)));
	connect(ui.graphicsViewMain, SIGNAL(mouseReleased(QPointF, bool)), this, SLOT(graphicsViewMouseReleased(QPointF, bool)));
	connect(ui.graphicsViewFOV, SIGNAL(mousePress(QPointF, bool)), this, SLOT(graphicsViewMousePress(QPointF, bool)));
	connect(ui.graphicsViewMain, SIGNAL(rubberBandChanged(QRect, QPointF, QPointF)), this, SLOT(processROIOption(QRect, QPointF, QPointF)));
	connect(ui.graphicsViewMain, SIGNAL(mouseDoubleClick(bool)), this, SLOT(setdragMode(bool)));
	connect(ui.graphicsViewMain, SIGNAL(rightMouseBtnPressed(QPoint)), this, SLOT(setRightMousePressed(QPoint)));
	connect(ui.graphicsViewMain, SIGNAL(wheelEventStart()), this, SLOT(wheelEventStart()));
	connect(ui.graphicsViewMain, SIGNAL(wheelEventEnd()), this, SLOT(wheelEventEnd()));


	connect(ui.treeViewRecipeExplorer, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onCustomContextMenu(const QPoint &)));
	connect(ui.actionAddObject, SIGNAL(triggered()), this, SLOT(addObject()));
	connect(ui.actionSelectAll, SIGNAL(triggered()), this, SLOT(selectAll()));
	connect(ui.actionAddObjectFromView, SIGNAL(triggered()), this, SLOT(addObjectFromView()));
	//connect(ui.actionRemoveObject, SIGNAL(triggered()), this, SLOT(removeObject()));
	connect(ui.treeViewObjectExplorer, SIGNAL(clicked(QModelIndex)), this, SLOT(treeViewObjectExplorerClicked(QModelIndex)));
	connect(ui.actionEditTemplate, SIGNAL(triggered()), this, SLOT(editTemplate()));
	connect(ui.actionDuplicateVisionObjects, &QAction::triggered, this, [=]() { toPage(UIPage::ROI_EDITOR); });
	connect(ui.action_saveReferenceVOImage, &QAction::triggered, this, [=]() { saveTemplateReferenceImage(nullptr); });
	connect(ui.toolButtonSelectMode, &QToolButton::clicked, this, [&]() {
		selectMode();
	});
	connect(ui.toolButtonDrawVisionObjMode, &QToolButton::clicked, this, [&]() {
		visionObjectMode();
	});

	connect(ui.lineEditUserName, SIGNAL(returnPressed()), this, SLOT(verifyLogin()));
	connect(ui.lineEditPassword, SIGNAL(returnPressed()), this, SLOT(verifyLogin()));

	// emap config
	connect(ui.radioButton_emapAuto, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_emapInfo.mode = EmapMode::AUTO;

			if (!_isUseEmapTemplate) {
				jsonHelper::setJsonValue(_systemObj, "Emap_Mode", _emapInfo.mode);
				updateSystemInfo(_systemObj);
			}
		
		
		}
		
	});
	connect(ui.radioButton_csv01Emap, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_emapInfo.mode = EmapMode::CSV01;
			
			if (!_isUseEmapTemplate)
			{
				jsonHelper::setJsonValue(_systemObj, "Emap_Mode", _emapInfo.mode);
				updateSystemInfo(_systemObj);
			}
			
		}	
	});
	connect(ui.radioButton_csv34Emap, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_emapInfo.mode = EmapMode::CSV34;
			
			if (!_isUseEmapTemplate)
			{
				jsonHelper::setJsonValue(_systemObj, "Emap_Mode", _emapInfo.mode);
				updateSystemInfo(_systemObj);
			}

		}
	});
	connect(ui.radioButton_textFileEmap, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_emapInfo.mode = EmapMode::TEXT_FILE;
		
			if (!_isUseEmapTemplate)
			{
				jsonHelper::setJsonValue(_systemObj, "Emap_Mode", _emapInfo.mode);
				updateSystemInfo(_systemObj);
			}
			
		}
	
	});
	QObject::connect(ui.comboBox_emapTopInsp, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int index) {
		if (ui.comboBox_emapTopInsp->currentText() == "CSV 01 EMAP") _emapInfo.topInspEmap = EmapType::CSV01_EMAP;
		else if (ui.comboBox_emapTopInsp->currentText() == "CSV 34 EMAP")_emapInfo.topInspEmap = EmapType::CSV34_EMAP;
		else if (ui.comboBox_emapTopInsp->currentText() == "TEXT FILE EMAP")_emapInfo.topInspEmap = EmapType::TEXT_FILE_EMAP;
		
		if (!_isUseEmapTemplate)
		{
			jsonHelper::setJsonValue(_systemObj, "Emap_Top_Insp", _emapInfo.topInspEmap);
			updateSystemInfo(_systemObj);
		}
	});
	QObject::connect(ui.comboBox_emapBotInsp, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int index) {
		if (ui.comboBox_emapBotInsp->currentText() == "CSV 01 EMAP") _emapInfo.botInspEmap = EmapType::CSV01_EMAP;
		else if (ui.comboBox_emapBotInsp->currentText() == "CSV 34 EMAP")_emapInfo.botInspEmap = EmapType::CSV34_EMAP;
		else if (ui.comboBox_emapBotInsp->currentText() == "TEXT FILE EMAP")_emapInfo.botInspEmap = EmapType::TEXT_FILE_EMAP;
	
		if (!_isUseEmapTemplate)
		{
			jsonHelper::setJsonValue(_systemObj, "Emap_Bot_Insp", _emapInfo.botInspEmap);
			updateSystemInfo(_systemObj);
		}
	});
	connect(ui.toolButton_browseCsvEmap, &QToolButton::clicked, this, [&]() {

		QFileDialog dialog;
		dialog.setFileMode(QFileDialog::Directory);
		dialog.setOption(QFileDialog::ShowDirsOnly);
		QString csvPathDir = QFileDialog::getExistingDirectory(this, tr("Choose Directory"), Common::Directory::getRecipeCurrentPath());
		if (!csvPathDir.isEmpty())
		{
			_emapInfo.csvEmapDir.append(csvPathDir);

			ui.listWidget_csvEmapDir->clear();
			ui.listWidget_csvEmapDir->addItems(_emapInfo.csvEmapDir);

			QJsonArray arr;
			for (auto d : _emapInfo.csvEmapDir) arr.append(d);
			
			if (!_isUseEmapTemplate)
			{
				jsonHelper::setJsonValue(_systemObj, "Emap_Csv_Dir", arr);
				updateSystemInfo(_systemObj);
			}
		}
	});

	connect(ui.toolButton_browseTextFileEmap, &QToolButton::clicked, this, [&]() {
		QString oriCadPath;
		QFileDialog dialog;
		dialog.setFileMode(QFileDialog::Directory);
		dialog.setOption(QFileDialog::ShowDirsOnly);
		QString textFileDir = QFileDialog::getExistingDirectory(this, tr("Choose Directory"), Common::Directory::getRecipeCurrentPath());
		if (!textFileDir.isEmpty())
		{
			// --- 
			_emapInfo.textFileEmapDir.append(textFileDir);

			ui.listWidget_txtEmapDir->clear();
			ui.listWidget_txtEmapDir->addItems(_emapInfo.textFileEmapDir);

			QJsonArray arr;
			for (auto d : _emapInfo.textFileEmapDir) arr.append(d);
			
			if (!_isUseEmapTemplate)
			{
				jsonHelper::setJsonValue(_systemObj, "Emap_Text_File_Dir", arr);
				updateSystemInfo(_systemObj);
			}
		}
		
	});
	connect(ui.toolButton_deleteCsvEmapPath, &QToolButton::clicked, this, [&]() {
		QListWidgetItem* item = ui.listWidget_csvEmapDir->currentItem();
		if (item == nullptr) return;
		QString selectedDir = item->text();

		_emapInfo.csvEmapDir.removeAt(_emapInfo.csvEmapDir.indexOf(selectedDir));

		ui.listWidget_csvEmapDir->clear();
		ui.listWidget_csvEmapDir->addItems(_emapInfo.csvEmapDir);

		QJsonArray arr;
		for (auto d : _emapInfo.csvEmapDir) arr.append(d);
		
		if (!_isUseEmapTemplate)
		{
			jsonHelper::setJsonValue(_systemObj, "Emap_Csv_Dir", arr);
			updateSystemInfo(_systemObj);
		}
	});

	connect(ui.toolButton_deleteTxtEmapPath, &QToolButton::clicked, this, [&]() {
		QListWidgetItem* item = ui.listWidget_txtEmapDir->currentItem();
		if (item == nullptr) return;
		QString selectedDir = item->text();

		_emapInfo.textFileEmapDir.removeAt(_emapInfo.textFileEmapDir.indexOf(selectedDir));

		ui.listWidget_txtEmapDir->clear();
		ui.listWidget_txtEmapDir->addItems(_emapInfo.textFileEmapDir);

		QJsonArray arr;
		for (auto d : _emapInfo.textFileEmapDir) arr.append(d);
	
		if (!_isUseEmapTemplate)
		{
			jsonHelper::setJsonValue(_systemObj, "Emap_Text_File_Dir", arr);
			updateSystemInfo(_systemObj);
		}
	});
	//BareBoardAnalysis
	connect(ui.toolButton_runBBA, &QToolButton::clicked, this, [&]() {
		_enable2D = true;
		_enable3D = true;
		const QString src = QDir::cleanPath(Common::Directory::getRecipeCurrentPath());
		const QString dst = QDir::cleanPath(Common::Directory::RecipePath());
		_inbbaInspection = true;
		this->copyAndPatchRecipe(src, dst);
		openRecipe("", true);

		inspect2D3D();
		setupProductionDir();
		//run();
		});
	
	connect(ui.comboBox_stitchingMethod, SIGNAL(currentIndexChanged(int)), this, SLOT(saveStitchingMethod()));

	connect(ui.comboBox_lineScanAxis, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
		SystemData::instance()._lineScanAxis = index;
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("LINESCAN_AXIS_CHANGED"));
		showMsg(tr("Line scan axis changed. Please reassign line scans for the new axis to take effect."));
	});

	connect(ui.comboBox_lineScanDirection, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
		SystemData::instance()._lineScanDirection = index;
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("LINESCAN_DIRECTION_CHANGED"));
		//No re-teach needed, unlike the axis: the line scans keep their taught span, the gantry
		//just traverses it the other way and the height map is flipped back to match.
		showMsg(tr("Scan direction changed. Line scans keep their taught positions - only the "
			"direction of travel changes, from the next scan onwards."));
	});

	connect(ui.checkBox_heightMapNativeScale, &QCheckBox::toggled, this, [=](bool on) {
		SystemData::instance()._heightMapNativeScale = on;
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("HEIGHTMAP_SCALE_CHANGED"));
		showMsg(on
			? tr("Height maps will now be built at the sensor's own resolution. They no longer "
				"match 2D views, so any 3D ROI taught at the old scale must be re-taught, and "
				"the images are considerably larger.")
			: tr("Height maps will be built at the shared world scale again, matching 2D views."));
	});

	connect(ui.comboBox_camRotation, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
		SystemData::instance()._camImageRotation = index * 90;
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("CAM_ROTATION"), QString::number(index * 90));
	});

	connect(ui.checkBox_homeOnStartup, &QCheckBox::toggled, this, [=](bool checked) {
		SystemData::instance()._homeOnStartup = checked;
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("HOME_ON_STARTUP"), checked ? QStringLiteral("ON") : QStringLiteral("OFF"));
	});

	connect(ui.checkBox_lscStrobeMode, &QCheckBox::toggled, this, [=](bool checked) {
		SystemData::instance()._lscStrobeMode = checked;
		saveRecipeConfig();
		AuditLog::instance().log(QStringLiteral("LSC_STROBE_MODE"), checked ? QStringLiteral("ON") : QStringLiteral("OFF"));

		auto ret = LSCManager::instance().setMode(checked ? lsc::MODE::TRIGGER : lsc::MODE::CONTINUOUS);
		if (ret != (int)LSC_RC::PASS) showMsg(tr("Failed to switch light controller mode."));
	});

	//treeViewRecipeExplorer_select
	connect(_recipeSelectionModel, SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)),
		this, SLOT(recipeSelectionChangedSlot(const QItemSelection &, const QItemSelection &)));

	//treeViewResultExplorer_select
	connect(_resultSelectionModel, SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)),
		this, SLOT(resultSelectionChangedSlot(const QItemSelection &, const QItemSelection &)));

	connect(&_objectModel, SIGNAL(itemChanged(QStandardItem *)), this, SLOT(objectModelItemChanged(QStandardItem *)));

	qRegisterMetaType<QHash<QString, ct::UnitResultInfo>>("QHash<QString, ct::UnitResultInfo>");

	//qRegisterMetaType<QVector<ct::DefectResult>>("QVector<ct::DefectResult>");
	connect(&_networkPathChecker, SIGNAL(updateConnection(bool)), this, SLOT(updateConnection(bool)));




	//list
	connect(ui.listWidget_unassignedVisionObject, &QListWidget::itemClicked, [=](QListWidgetItem *item) {
		//clear selection
		for (auto roi : _dragROI) {
			roi->setSelected(false);
		}

		//select based on list
		for (int i = 0; i < ui.listWidget_unassignedVisionObject->count(); ++i) {
			auto it = ui.listWidget_unassignedVisionObject->item(i);
			if (it->isSelected()) {
				auto key = it->text();
				auto id = it->whatsThis();
				if (_visionObject.contains(id)) {
					_visionObject[id].pDragBox->setSelected(true);
				}
			}
		}
	});

	connect(ui.toolButton_showCrossHair, &QToolButton::toggled, [=](bool state) { showCrossHair(state); });
	connect(ui.toolButton_showView, &QToolButton::toggled, [=](bool state) { showView(state); });
	connect(ui.toolButton_showVisionObject, &QToolButton::toggled, [=](bool state) { showVisionObject(state); });
	connect(ui.toolButton_showPath, &QToolButton::toggled, [=](bool state) { showPath(state); });
	connect(ui.toolButton_showLineScan, &QToolButton::toggled, [=](bool state) { showLineScans(state); });

	connect(ui.toolButton_toggleDualView, &QToolButton::toggled, [=](bool state) { toggleDualView(); });
	connect(ui.toolButton_toggleWorldView, &QToolButton::toggled, [=](bool state) { ui.stackedWidgetViewSelection->setCurrentIndex(0); toggleWorldView(); });
	connect(ui.toolButton_toggleFovView, &QToolButton::toggled, [=](bool state) { toggleFOVView(); });

	connect(ui.tb_assignView, &QToolButton::clicked, [=]() { assignViews(ui.lineEdit_viewPadding->text().toDouble(), ui.lineEdit_viewOverlapPercentage->text().toInt()); AuditLog::instance().log(QStringLiteral("VIEWS_ASSIGN")); }); //button that appears when new vo is added
	connect(ui.toolButton_assignView, &QToolButton::clicked, [=]() { assignViews(ui.lineEdit_viewPaddingMM->text().toDouble(), ui.lineEdit_viewOverlapPercentage->text().toInt()); AuditLog::instance().log(QStringLiteral("VIEWS_ASSIGN")); }); //fixed button at assignment tab
	connect(ui.toolButton_assignLineScan, &QToolButton::clicked, [=]() { assignLineScans(); AuditLog::instance().log(QStringLiteral("LINESCANS_ASSIGN")); });
	connect(ui.toolButton_collect2DView, &QToolButton::clicked, this, [=]() {
		ui.toolButton_collect2DView->setEnabled(false);  // Prevent further clicks
		_loop = 0;
		collect2DView(getViewCollectionPath()); 
		QTimer::singleShot(1000, this, [=]() { ui.toolButton_collect2DView->setEnabled(true); });
	});
	connect(ui.toolButton_collect3DView, &QToolButton::clicked, this, [=]() {
		ui.toolButton_collect3DView->setEnabled(false);  // Prevent further clicks
		_loop = 0;
		collect3DView(getViewCollectionPath()); 
		QTimer::singleShot(1000, this, [=]() { ui.toolButton_collect3DView->setEnabled(true); });
	});
	connect(ui.toolButton_collect2D3DView, &QToolButton::clicked, this, [=]() {
		ui.toolButton_collect2D3DView->setEnabled(false);  // Prevent further clicks
		_loop = 0;
		collect2D3DView(getViewCollectionPath()); 
		QTimer::singleShot(1000, this, [=]() { ui.toolButton_collect2D3DView->setEnabled(true); });
	});

	//View
	connect(ui.toolButton_setFrontLeft, &QToolButton::clicked, [=]() {
		double wx, wy, wz;
		getCurrentPoint(wx, wy, wz);
		_plane.corner_points[(int)Corner::FRONTLEFT].wx = wx;
		_plane.corner_points[(int)Corner::FRONTLEFT].wy = wy;
		_plane.corner_points[(int)Corner::FRONTLEFT].wz = wz;
		
		teachTopleft(_plane.corner_points[(int)Corner::FRONTLEFT]);
		savePlane();
		AuditLog::instance().log(QStringLiteral("PLANE_TEACH_CORNER"), QStringLiteral("FrontLeft"));
	});
	connect(ui.toolButton_setBackRight, &QToolButton::clicked, [=]() {
		double wx, wy, wz;
		getCurrentPoint(wx, wy, wz);
		_plane.corner_points[(int)Corner::BACKRIGHT].wx = wx;
		_plane.corner_points[(int)Corner::BACKRIGHT].wy = wy;
		_plane.corner_points[(int)Corner::BACKRIGHT].wz = wz;
		
		teachBtmright(_plane.corner_points[(int)Corner::BACKRIGHT]);
		savePlane();
		AuditLog::instance().log(QStringLiteral("PLANE_TEACH_CORNER"), QStringLiteral("BackRight"));
	});
	connect(ui.toolButton_jogToTopLeft, &QToolButton::clicked, [=]() {
		auto wx = _plane.corner_points[(int)Corner::FRONTLEFT].wx;
		auto wy = _plane.corner_points[(int)Corner::FRONTLEFT].wy;
		auto wz = _plane.corner_points[(int)Corner::FRONTLEFT].wz;
		emit jogSnap(wx, wy, wz, _mainOptics[_camID]);
	});
	connect(ui.toolButton_jogToBottomRight, &QToolButton::clicked, [=]() {
		auto wx = _plane.corner_points[(int)Corner::BACKRIGHT].wx;
		auto wy = _plane.corner_points[(int)Corner::BACKRIGHT].wy;
		auto wz = _plane.corner_points[(int)Corner::BACKRIGHT].wz;
		emit jogSnap(wx, wy, wz, _mainOptics[_camID]);
	});

	//── setup region mode: Plane (corner teach) vs Pitch (unit grid for the barcode flow) ──
	{
		auto applySetupRegionMode = [=]() {
			const bool pitch = SystemData::instance()._setupRegionPitchMode;
			ui.widget_pitchSetup->setVisible(pitch);

			//plane-teach controls hide in pitch mode
			for (QWidget* w : QVector<QWidget*>{
				ui.toolButton_setFrontLeft, ui.toolButton_setBackRight,
				ui.label_frontLeft, ui.label_backRight,
				ui.toolButton_jogToTopLeft, ui.toolButton_jogToBottomRight,
				ui.toolButton_collectPlaneImages }) {
				w->setVisible(!pitch);
			}
		};
		_applySetupRegionMode = applySetupRegionMode;
		applySetupRegionMode();

		connect(ui.comboBox_setupRegionMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
			SystemData::instance()._setupRegionPitchMode = (index == 1);
			applySetupRegionMode();
			saveRecipeConfig();
			AuditLog::instance().log(QStringLiteral("SETUP_REGION_MODE"), index == 1 ? QStringLiteral("PITCH") : QStringLiteral("PLANE"));
		});

		auto refreshPitchLabels = [=]() {
			auto& sd = SystemData::instance();
			ui.label_pitchP1->setText(sd._pitchP1Set
				? QString("P1: %1, %2, %3").arg(sd._pitchP1x.load(), 0, 'f', 3).arg(sd._pitchP1y.load(), 0, 'f', 3).arg(sd._pitchP1z.load(), 0, 'f', 3)
				: QStringLiteral("P1: not set"));
		};
		_refreshPitchLabels = refreshPitchLabels;

		connect(ui.toolButton_setPitchP1, &QToolButton::clicked, this, [=]() {
			double wx, wy, wz;
			getCurrentPoint(wx, wy, wz);
			auto& sd = SystemData::instance();
			sd._pitchP1x = wx;
			sd._pitchP1y = wy;
			sd._pitchP1z = wz;
			sd._pitchP1Set = true;
			refreshPitchLabels();
			saveRecipeConfig();
			AuditLog::instance().log(QStringLiteral("PITCH_SET_P1"));
		});

		connect(ui.toolButton_setPitchP2, &QToolButton::clicked, this, [=]() {
			auto& sd = SystemData::instance();
			if (!sd._pitchP1Set) {
				showMsg("Set point 1 (top left) first.");
				return;
			}

			double wx, wy, wz;
			getCurrentPoint(wx, wy, wz);

			//point 2 is the diagonally adjacent unit toward bottom right: pitch is signed p2 - p1
			sd._pitchX = wx - sd._pitchP1x;
			sd._pitchY = wy - sd._pitchP1y;
			ui.label_pitchP2->setText(QString("P2: %1, %2, %3").arg(wx, 0, 'f', 3).arg(wy, 0, 'f', 3).arg(wz, 0, 'f', 3));
			{
				QSignalBlocker b1(ui.lineEdit_pitchX);
				QSignalBlocker b2(ui.lineEdit_pitchY);
				ui.lineEdit_pitchX->setText(QString::number(sd._pitchX.load(), 'f', 3));
				ui.lineEdit_pitchY->setText(QString::number(sd._pitchY.load(), 'f', 3));
			}
			saveRecipeConfig();
			AuditLog::instance().log(QStringLiteral("PITCH_SET_P2"));
		});

		connect(ui.lineEdit_pitchX, &QLineEdit::editingFinished, this, [=]() {
			SystemData::instance()._pitchX = ui.lineEdit_pitchX->text().toDouble();
			saveRecipeConfig();
		});
		connect(ui.lineEdit_pitchY, &QLineEdit::editingFinished, this, [=]() {
			SystemData::instance()._pitchY = ui.lineEdit_pitchY->text().toDouble();
			saveRecipeConfig();
		});
		connect(ui.lineEdit_unitsX, &QLineEdit::editingFinished, this, [=]() {
			int v = std::max(1, ui.lineEdit_unitsX->text().toInt());
			ui.lineEdit_unitsX->setText(QString::number(v));
			SystemData::instance()._unitsX = v;
			saveRecipeConfig();
		});
		connect(ui.lineEdit_unitsY, &QLineEdit::editingFinished, this, [=]() {
			int v = std::max(1, ui.lineEdit_unitsY->text().toInt());
			ui.lineEdit_unitsY->setText(QString::number(v));
			SystemData::instance()._unitsY = v;
			saveRecipeConfig();
		});

		connect(ui.checkBox_pitchBarcode, &QCheckBox::toggled, this, [=](bool checked) {
			SystemData::instance()._pitchEnableBarcode = checked;
			saveRecipeConfig();
		});
		connect(ui.checkBox_pitch3D, &QCheckBox::toggled, this, [=](bool checked) {
			SystemData::instance()._pitchEnable3D = checked;
			saveRecipeConfig();
		});
		connect(ui.lineEdit_pitchScanLen, &QLineEdit::editingFinished, this, [=]() {
			double v = ui.lineEdit_pitchScanLen->text().toDouble();
			if (v <= 0) { v = 10.0; ui.lineEdit_pitchScanLen->setText("10"); }
			SystemData::instance()._pitchScanLen_mm = v;
			saveRecipeConfig();
		});
	}

	connect(ui.toolButton_generateSetupRegion, &QToolButton::clicked, [=]() { 
		generatePlane(_plane);
		stitchPlaneImage(_plane);
	});
	connect(ui.toolButton_collectPlaneImages, &QToolButton::clicked, [=]() {
		ui.toolButton_collectPlaneImages->setEnabled(false);
		generatePlane(_plane);
		collectPlaneViews(_plane);

		updateSetupCheckList();
		QTimer::singleShot(1000, this, [=]() { ui.toolButton_collectPlaneImages->setEnabled(true); });
	});

	connect(ui.toolButton_updateViewZ, &QToolButton::clicked, [=]() {

#if 0
		// RECIPE_Z_CONVEYOR_DISABLED_BEGIN
		// Recipe-based 3D Z offset is disabled. Re-enable this block to restore
		// offset-specific Update Z / Only Offset Z behavior.
		QMessageBox msgBox;
		msgBox.setWindowTitle("Update Z Settings");
		msgBox.setText("How would you like to update the Z coordinates?");

		QPushButton* btnUpdateView = msgBox.addButton("Update View Z", QMessageBox::ActionRole);
		QPushButton* btnOnlyOffset = msgBox.addButton("Only Offset Z", QMessageBox::ActionRole);
		msgBox.addButton(QMessageBox::Cancel);

		msgBox.exec();

		if (msgBox.clickedButton() == btnUpdateView) {
			auto new_z = SystemData::instance().currentCoordinate().wz;
			ui.lineEdit_viewZ->setText(QString::number(new_z));

			double z_offset = ui.lineEdit_3DZoffset->text().toDouble();
			m_currentZOffset = z_offset;

			_plane.corner_points[(int)Corner::FRONTLEFT].wz = new_z;
			_plane.corner_points[(int)Corner::BACKRIGHT].wz = new_z;

			for (auto& v : _plane.views) {
				v.world.wz = new_z;
			}

			for (auto& v : _views) {
				v.world.wz = new_z;
			}

			auto line_scan_z = new_z + z_offset;

			for (auto& scan : _lineScans) {
				scan.start_point.wz = line_scan_z;
				scan.end_point.wz = line_scan_z;
			}

			savePlane();
			saveFiducial();
			saveBarcode();
			saveRecipe();
			saveLineScans();

		}
		else if (msgBox.clickedButton() == btnOnlyOffset) {

			double original_z = ui.lineEdit_viewZ->text().toDouble();
			double z_offset = ui.lineEdit_3DZoffset->text().toDouble();

			m_currentZOffset = z_offset;

			auto line_scan_z = original_z + z_offset;

			for (auto& scan : _lineScans) {
				scan.start_point.wz = line_scan_z;
				scan.end_point.wz = line_scan_z;
			}

			savePlane();
			saveFiducial();
			saveBarcode();
			saveRecipe();
			saveLineScans();
		}
		// RECIPE_Z_CONVEYOR_DISABLED_END
#endif

		auto new_z = SystemData::instance().currentCoordinate().wz;
		ui.lineEdit_viewZ->setText(QString::number(new_z));

		_plane.corner_points[(int)Corner::FRONTLEFT].wz = new_z;
		_plane.corner_points[(int)Corner::BACKRIGHT].wz = new_z;

		for (auto& v : _plane.views) {
			v.world.wz = new_z;
		}

		for (auto& v : _views) {
			v.world.wz = new_z;
		}

		for (auto& scan : _lineScans) {
			scan.start_point.wz = new_z;
			scan.end_point.wz = new_z;
		}

		savePlane();
		saveFiducial();
		saveBarcode();
		saveRecipe();
		saveLineScans();
		AuditLog::instance().log(QStringLiteral("VIEW_Z_UPDATE"), QStringLiteral("z=%1").arg(new_z));
		});


	//ROI
	connect(ui.tb_duplicateUp, &QToolButton::clicked, [=]() { duplicateROI(Direction::UP); });
	connect(ui.tb_duplicateDown, &QToolButton::clicked, [=]() { duplicateROI(Direction::DOWN); });
	connect(ui.tb_duplicateLeft, &QToolButton::clicked, [=]() { duplicateROI(Direction::LEFT); });
	connect(ui.tb_duplicateRight, &QToolButton::clicked, [=]() { duplicateROI(Direction::RIGHT); });

	QDoubleValidator validator_score(0.0, 100.0, 2);
	QDoubleValidator validator_angle(0.0, 360.0, 2);
	ui.lineEdit_similarityScore->setValidator(&validator_score);
	ui.lineEdit_angleStep->setValidator(&validator_angle);

	QDoubleValidator validator_exposure(1000, 100000, 2);
	QDoubleValidator validator_gain(1, 10, 2);
	ui.lineEdit_exposure->setValidator(&validator_exposure);
	ui.lineEdit_gain->setValidator(&validator_gain);

	connect(ui.lineEdit_duplicatePitchMM, &QLineEdit::textEdited, this, [=](const QString &text) {
		auto pitch_mm = text.toDouble();
		auto pitch_px = util::mm_to_px(pitch_mm, (ScaleManager::instance().horizontal_um_per_px() + ScaleManager::instance().vertical_um_per_px()) / 2);
		ui.lineEdit_duplicatePitchPX->setText(QString::number(pitch_px));
	});
	connect(ui.lineEdit_duplicatePitchPX, &QLineEdit::textEdited, this, [=](const QString &text) {
		auto pitch_px = text.toDouble();
		auto pitch_mm = util::px_to_mm(pitch_px, (ScaleManager::instance().horizontal_um_per_px() + ScaleManager::instance().vertical_um_per_px()) / 2);
		ui.lineEdit_duplicatePitchMM->setText(QString::number(pitch_mm));
	});
	connect(ui.tb_duplicateFromSelectedVisionObject, &QToolButton::clicked, [=]() {
		
		bool hasSelected = false;
		for (int i = 0; i < _dragROI.count(); i++)
		{
			if (_dragROI.at(i)->isSelected() == true)
			{
				hasSelected = true;
				break;
			}
		}
		if (hasSelected)
		{
			if (ui.checkBox_partialSearchVo->isChecked())
			{
				duplicateFromSelectedVisionObject_cropped();
			}
			else
			{
				duplicateFromSelectedVisionObject();
			}

		}
		else
		{
			QMessageBox::warning(this, tr("No Unit Selected!"),
				"Please select Units for duplicating");
		}
		
	});

	connect(ui.toolButton_searchVo, &QToolButton::clicked, [=]() { searchVo(); });

	//Path 
	connect(ui.checkBox_uniformlyDistancedViews, &QCheckBox::stateChanged, [=](int state) {
		if (state == Qt::Checked) {
			ui.label_prioritizeDirection->show();
			ui.comboBox_prioritizeDirection->show();
		}
		else {
			ui.label_prioritizeDirection->hide();
			ui.comboBox_prioritizeDirection->hide();
		}
	});

	connect(ui.tb_setStartPoint, &QToolButton::clicked, [=]() { _currentSetPoint = "start"; _pathSM.start(); });
	connect(ui.tb_setEndPoint, &QToolButton::clicked, [=]() { _currentSetPoint = "end"; _pathSM.start(); });
	connect(ui.tb_generatePath, &QToolButton::clicked, [=]() { generatePath(); });
	connect(ui.tb_savePath, &QToolButton::clicked, [=]() { savePathInfo(); });
	connect(ui.tb_includeView, &QToolButton::clicked, [=]() { setViewSelectionCheckState(Qt::Checked); });
	connect(ui.tb_excludeView, &QToolButton::clicked, [=]() { setViewSelectionCheckState(Qt::Unchecked); });
	connect(ui.listWidget_viewSelection, &QListWidget::itemChanged, [=](QListWidgetItem* item) {});
	connect(ui.listWidget_paths->model(), &QAbstractItemModel::rowsMoved, [=]() { redrawPath(); });
	connect(ui.toolButton_selectAll, &QToolButton::clicked, [=]() { selectAllPath(); });
	connect(ui.toolButton_unselectAll, &QToolButton::clicked, [=]() { unselectAllPath(); });

	//Scaling
	ui.groupBox_measurementScaling->hide();
	connect(ui.tb_autoScale, &QToolButton::clicked, [=]() {
		double step_mm = ui.lineEdit_scalingStep->text().toDouble();
		//autoScaling(step_mm, _worldEnv.horizontal_scale, _worldEnv.vertical_scale);
		//saveWorldEnv();
		//showMsg(QStringLiteral("Scaling: %1, %2\n").arg(_worldEnv.horizontal_scale).arg(_worldEnv.vertical_scale));
	});
	connect(ui.tb_manualScale, &QToolButton::clicked, [=]() { _manualScalingSM.start(); });
	connect(ui.toolButton_performScaling, &QToolButton::clicked, [=]() { performScaling(); });
	connect(ui.toolButton_performCamAlignment, &QToolButton::clicked, [=]() { cameraAlignment(); });

	//AIModels
	connect(ui.toolButton_refreshODModelList, &QToolButton::clicked, [=]() { unloadODModels(); addObjectDetectionModels(); });
	connect(ui.comboBox_loadODModels, SIGNAL(currentIndexChanged(int)), this, SLOT(load_unload_ODModels()));
	connect(ui.checkBox_odEnableFastMode, &QCheckBox::stateChanged, this, [this](int state) {
		if (state == Qt::CheckState::Checked)
		{
			_od_enableTensortRt = true;

		}
		else
		{
			_od_enableTensortRt = false;

		}
		saveODModelListJson();
		});

	connect(ui.checkBox_segEnableFastMode, &QCheckBox::stateChanged, this, [this](int state) {
		if (state == Qt::CheckState::Checked)
		{
			_seg_enableTensortRt = true;

		}
		else
		{
			_seg_enableTensortRt = false;

		}
		saveODModelListJson();
		});

	connect(ui.spinBox_odTilingSize, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
		// Example logic using spin box value
		g_odTilingSettings.tilingSize = value;
		saveODModelListJson();
		});

	connect(ui.doubleSpinBox_odTilingPaddingPerc, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
		g_odTilingSettings.tilingPaddingPerc = value;
		saveODModelListJson();
		});

	connect(ui.doubleSpinBox_odTilingIou, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
		g_odTilingSettings.tilingIou = value;
		saveODModelListJson();
		});

	connect(ui.checkBox_loadSegmentationModel, &QCheckBox::stateChanged, this, [=](int state) {
		if (state == Qt::Checked) {
			ui.frame_segmentationModel->setEnabled(true);
		
		}
		else {
			ui.radioButton_segTiny->setChecked(false);
			ui.radioButton_segSmall->setChecked(false);
			ui.radioButton_segMedium->setChecked(false);
			ui.radioButton_segLarge->setChecked(false);

			ui.frame_segmentationModel->setEnabled(false);
		}
		saveODModelListJson();
	});

	connect(ui.lineEdit_segmentationScore, &QLineEdit::editingFinished, this, [=]() {
		qDebug() << "Finished editing segmentation Score:" << ui.lineEdit_segmentationScore->text();
		saveODModelListJson();
		});

	connect(ui.radioButton_segTiny, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_curSegmentationModel = ui.radioButton_segTiny->text();
			loadSegmentationModel();	
		}	
		saveODModelListJson();
	});
	connect(ui.radioButton_segSmall, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_curSegmentationModel = ui.radioButton_segSmall->text();
			loadSegmentationModel();
		}
		saveODModelListJson();
		});
	connect(ui.radioButton_segMedium, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_curSegmentationModel = ui.radioButton_segMedium->text();
			loadSegmentationModel();
		}
		saveODModelListJson();
		});
	connect(ui.radioButton_segLarge, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_curSegmentationModel = ui.radioButton_segLarge->text();
			loadSegmentationModel();
		}
		saveODModelListJson();
		});
	//Loop
	connect(ui.toolButton_toggleOfflineRun, &QToolButton::toggled, [=](bool state) { toggleOfflineRun(); });
	connect(ui.toolButton_toggleOnlineRun, &QToolButton::toggled, [=](bool state) { toggleOnlineRun(); });
	toggleOnlineRun();

	//CTLink
	connect(ui.toolButton_connectServer, SIGNAL(clicked()), this, SLOT(connectToServer()));
	//CTLink


	// Result viewer station setting
	connect(ui.radioButton_rvLocalPC, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			
			QString localPath = "C:/";
			jsonHelper::setJsonValue(_systemObj, "Verification_Station_Ip_Address", localPath);
			updateSystemInfo(_systemObj);
			_networkPathChecker.setIpAddress(jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")));

			ui.label_89->hide();
			ui.lineEdit_rvIpAddress->hide();
			ui.toolButton_rvConnect->hide();
		}
	});
	connect(ui.radioButton_rvOfflineStation, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {

			QString stationIP = "//" + ui.lineEdit_rvIpAddress->text();
			jsonHelper::setJsonValue(_systemObj, "Verification_Station_Ip_Address", stationIP);
			updateSystemInfo(_systemObj);
			_networkPathChecker.setIpAddress(jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")));

			ui.label_89->show();
			ui.lineEdit_rvIpAddress->show();
			ui.toolButton_rvConnect->show();
		}
	});
	connect(ui.toolButton_rvConnect, &QToolButton::clicked, [=]() { 
		qDebug() << "Clicked connect";
		QString stationIP = "//" + ui.lineEdit_rvIpAddress->text();
		

		jsonHelper::setJsonValue(_systemObj, "Verification_Station_Ip_Address", stationIP);
		updateSystemInfo(_systemObj);
		_networkPathChecker.setIpAddress(jsonHelper::getString(_systemObj, QStringLiteral("Verification_Station_Ip_Address")));
	});

	connect(ui.pushButton_reloadEmapTemplate, &QPushButton::clicked, [=]() {
		loadEmapSetting();
		refreshEmapSettingUi();
	});

	connect(ui.radioButton_useLocalEmapSetting, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_isUseEmapTemplate = false;

			ui.label_emapTemplate->hide();
			ui.comboBox_emapTemplate->hide();
			ui.frame_emapSetting->setEnabled(true);
			_emapInfo = _emapLocalSetting;
			refreshEmapSettingUi();
			jsonHelper::setJsonValue(_systemObj, "Enable_Emap_Template", _isUseEmapTemplate);
			updateSystemInfo(_systemObj);
		}
		
	});
	connect(ui.radioButton_useEmapTemplate, &QRadioButton::toggled, this, [=](bool checked) {
		if (checked) {
			_isUseEmapTemplate = true;

			ui.comboBox_emapTemplate->setCurrentIndex(ui.comboBox_emapTemplate->findText(_emapTemplate));

			ui.label_emapTemplate->show();
			ui.comboBox_emapTemplate->show();
			ui.frame_emapSetting->setEnabled(false);
			if (_emapTemplateList.contains(ui.comboBox_emapTemplate->currentText()))
			{
				_emapInfo = _emapTemplateList[ui.comboBox_emapTemplate->currentText()];
			}
			refreshEmapSettingUi();
			jsonHelper::setJsonValue(_systemObj, "Enable_Emap_Template", _isUseEmapTemplate);
			updateSystemInfo(_systemObj);
		}
	});

	QObject::connect(ui.comboBox_emapTemplate, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int index) {
	
		if (!_isUseEmapTemplate) return;
		if (_emapTemplateList.contains(ui.comboBox_emapTemplate->currentText()))
		{
			_emapInfo = _emapTemplateList[ui.comboBox_emapTemplate->currentText()];
			_emapTemplate = _emapInfo.templateName;
			
			jsonHelper::setJsonValue(_systemObj, "Emap_Template_Name", _emapTemplate);
			updateSystemInfo(_systemObj);
		}
		
		refreshEmapSettingUi();

	});


	connect(_grDialog, &GoldenRecipeDialog::signalRequestGoldenRecipeResult, this, [this]() {
		/*bool exeEXist = true;
		LPCWSTR a = L"Algo Editor";
		if (!EXE_ExistTest(a))
		{
			
		}*/

		
		if (!_dragROI.isEmpty())
		{
			_dragROI[0]->setSelected(true);
			editTemplate();
		}
		QString message = "VisionApp|AlgoEditor|RunGoldenRecipeUnit|" + Common::Directory::CurrentRecipe;

		

		_ctClient->sendCommand(message);

		
	});
	connect(_grDialog, SIGNAL(signalRunGoldenRecipeComplete(bool, QString)), this, SLOT(slotRunGoldenRecipeComplete(bool, QString)));

	QObject::connect(ui.comboBox_cameraSelection, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int index) {
		_camID = ui.comboBox_cameraSelection->currentText();
		updateCameraTypeUI(_camID);
	});

	QObject::connect(ui.stackedWidgetViewSelection, &QStackedWidget::currentChanged, this, [=](int index) {
	});

	QObject::connect(ui.comboBox_acqType, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int index) {
		saveRecipeSetupZStack();
		});

	connect(ui.checkBox_Generate2dImage, &QCheckBox::stateChanged, this, [=](int state) {
		qDebug() << "State Changed checkBox_Generate2dImage:" << ui.checkBox_Generate2dImage->isChecked();
		saveRecipeSetupZStack();
		});

	connect(ui.lineEdit_step_um, &QLineEdit::editingFinished, this, [=]() {
		qDebug() << "Finished editing lineEdit_step_um:" << ui.lineEdit_step_um->text();
		saveRecipeSetupZStack();
		});

	connect(ui.lineEdit_Iteration, &QLineEdit::editingFinished, this, [=]() {
		qDebug() << "Finished editing lineEdit_Iteration:" << ui.lineEdit_Iteration->text();
		saveRecipeSetupZStack();
		});

	connectShortcuts();
}

void VisionApp::refreshEmapSettingUi()
{
	qDebug() << "Refresh Emap Setting UI";
	if (_isUseEmapTemplate)
	{
		ui.label_emapDescription->setStyleSheet("color: red");
		ui.label_emapDescription->setText("CHANGE TEMPLATE SETTING AT RESULT VIEWER!");
	}
	else
	{
		ui.label_emapDescription->setStyleSheet("color: white");
		ui.label_emapDescription->setText("LOCAL EMAP SETTING");
	}
	// Emap Config 
	QStringList inspType = {
		"CSV 01 EMAP",
		"CSV 34 EMAP",
		"TEXT FILE EMAP"
	};
	

	//incoming emap
	ui.listWidget_csvEmapDir->clear();
	ui.listWidget_txtEmapDir->clear();
	if (_emapInfo.mode == EmapMode::AUTO)ui.radioButton_emapAuto->setChecked(true);
	else if (_emapInfo.mode == EmapMode::CSV01)ui.radioButton_csv01Emap->setChecked(true);
	else if (_emapInfo.mode == EmapMode::CSV34)ui.radioButton_csv34Emap->setChecked(true);
	else if (_emapInfo.mode == EmapMode::TEXT_FILE)	ui.radioButton_textFileEmap->setChecked(true);

	ui.comboBox_emapTopInsp->blockSignals(true);
	ui.comboBox_emapBotInsp->blockSignals(true);
	ui.comboBox_emapTopInsp->clear();
	ui.comboBox_emapBotInsp->clear();
	ui.comboBox_emapTopInsp->addItems(inspType);
	ui.comboBox_emapBotInsp->addItems(inspType);
	ui.comboBox_emapTopInsp->blockSignals(false);
	ui.comboBox_emapBotInsp->blockSignals(false);

	if (_emapInfo.topInspEmap == EmapType::CSV01_EMAP) ui.comboBox_emapTopInsp->setCurrentIndex(ui.comboBox_emapTopInsp->findText("CSV 01 EMAP"));
	else if (_emapInfo.topInspEmap == EmapType::CSV34_EMAP) ui.comboBox_emapTopInsp->setCurrentIndex(ui.comboBox_emapTopInsp->findText("CSV 34 EMAP"));
	else if (_emapInfo.topInspEmap == EmapType::TEXT_FILE_EMAP)ui.comboBox_emapTopInsp->setCurrentIndex(ui.comboBox_emapTopInsp->findText("TEXT FILE EMAP"));

	if (_emapInfo.botInspEmap == EmapType::CSV01_EMAP) ui.comboBox_emapBotInsp->setCurrentIndex(ui.comboBox_emapBotInsp->findText("CSV 01 EMAP"));
	else if (_emapInfo.botInspEmap == EmapType::CSV34_EMAP) ui.comboBox_emapBotInsp->setCurrentIndex(ui.comboBox_emapBotInsp->findText("CSV 34 EMAP"));
	else if (_emapInfo.botInspEmap == EmapType::TEXT_FILE_EMAP)ui.comboBox_emapBotInsp->setCurrentIndex(ui.comboBox_emapBotInsp->findText("TEXT FILE EMAP"));
	ui.listWidget_csvEmapDir->addItems(_emapInfo.csvEmapDir);
	ui.listWidget_txtEmapDir->addItems(_emapInfo.textFileEmapDir);

	// template
	ui.comboBox_emapTemplate->blockSignals(true);
	ui.comboBox_emapTemplate->clear();
	for (auto a : _emapTemplateList)ui.comboBox_emapTemplate->addItem(a.templateName);
	ui.comboBox_emapTemplate->setCurrentIndex(ui.comboBox_emapTemplate->findText(_emapInfo.templateName));
	ui.comboBox_emapTemplate->blockSignals(false);

	if (_isUseEmapTemplate) ui.radioButton_useEmapTemplate->setChecked(true);
	else  ui.radioButton_useLocalEmapSetting->setChecked(true);
	
	
}


void VisionApp::executeUIFunc(std::function<void()> func) {
	func();
}

void VisionApp::imageReceived(FrameInfo info)
{
}

void VisionApp::imagePreprocessed(FrameInfo info)
{
	auto cid = util::combineID(info.viewID, info.opticID);
	ct::logger::trace("[ImagePreprocessed] ID: %s", cid.toStdString().c_str());

	if (info.type == ct::s_height_map) return;

	_imageFOV = mtrx::to_qimg(info.pImage->id());

	displayFOV(_imageFOV);

	if (_processType == ProcessType::LIVE_INSPECT) {
		mtrx::Circle circle;

		int resizeDivisor = 5;
		int size = 100;
		auto mResized = mtrx::resize(info.pImage->id(), info.width / resizeDivisor, info.height / resizeDivisor);

		clearAllRenderMaps();

		ct::logger::info("start");
		if (!mtrx::find_circle(circle, mResized, 400, 430, mtrx::CircleType::HIGHEST_SCORE, mtrx::FOREGROUND_ANY)) {
			drawText(_pGraphicsSceneFOV, "text", "No Circle Found", QPointF(info.width/2, info.height/2), Qt::red, size);
		}

		ct::logger::info("find circle: %f, %f - %f", circle.cx, circle.cy, circle.radius);
		drawCross(_pGraphicsSceneFOV, "cross", QRectF(circle.cx * resizeDivisor, circle.cy * resizeDivisor, circle.radius*2 * resizeDivisor, circle.radius *2 * resizeDivisor), Qt::yellow);
		drawEllipse(_pGraphicsSceneFOV, "circle", circle.cx * resizeDivisor, circle.cy * resizeDivisor, circle.radius *2* resizeDivisor, circle.radius *2* resizeDivisor, Qt::yellow);

		mtrx::free_buffer(mResized);
		ct::logger::info("draw");
	}

	if (_grabber) _grabber->resume();
}

void VisionApp::imageReady(QVector<FrameInfo> infos)
{
	if (infos.size() == 0) {
		ct::logger::error("[ImageReady] Empty frames being pass in");
		return;
	}

	for (const auto& info : infos) {
		QString cid = util::combineID(info.viewID, info.opticID);
		ct::logger::info("[ImageReady] Receive ready image: %s", cid.toStdString().c_str());

		//keep the latest heightmap available for the Algo Setup page ("Use Last Scan")
		if (info.type == ct::s_height_map && info.pHeightMap) {
			AlgoManager::instance().setHeightMap(info.pHeightMap);

			//production: archive the scan beside the fiducial/reader images as
			//<X#Y#>_height.tiff + <X#Y#>_intensity.jpg (worker thread, non-blocking)
			if (_processType == ProcessType::PRODUCTION && SystemData::instance()._saveInspImages) {
				QString saveName = info.viewID;
				if (saveName.startsWith("unit_")) {
					auto parts = saveName.mid(5).split('_');
					if (parts.size() == 2) saveName = QString("X%1Y%2").arg(parts[0], parts[1]);
				}

				const QString root = Common::Directory::getProductionImageSetPath();
				ImageSaveInfo task;
				task.heightBuf = info.pHeightMap;
				task.heightPath = (root + saveName + "_height.tiff").toStdString();
				if (info.pImage) {
					task.imgBuf = info.pImage;
					task.imgPath = (root + saveName + "_intensity.jpg").toStdString();
				}
				ImageSavingThread::instance().enqueue(task);
				ct::logger::info("[ImageReady] Saving 3D scan: %s_height.tiff", (root + saveName).toStdString().c_str());
			}

			//production: hand the scan to the inspection thread for the 3D algo
			InspectionThread::instance().enqueue(info);
		}

		{
			if (!_views.contains(info.viewID) && !_lineScans.contains(info.viewID)) return;
		
		}
	}

	if (_processType == ProcessType::PRODUCTION) {
		ct::logger::info("[ImageReady] In image production mode");
	}
	else if (_processType == ProcessType::IMAGE_COLLECTION) {
		ct::logger::info("[ImageReady] In image collection mode");

		bool savedImap = false;

		for (auto& info : infos) {
			ct::logger::debug("[ImageReady] Type received: %s", info.type.toStdString().c_str());
			ImageSavingThread::instance().enqueue(SystemData::instance()._workingPath.toStdString(), info);
		}
	}
}

void VisionApp::createEventAndSharedMemory(const QJsonObject& bufferInfoObj)
{
	QString name;
	QJsonObject bufferObj;
	int imageWidth;
	int imageHeight;

	QJsonArray bufferList = jsonHelper::getArray(bufferInfoObj, QStringLiteral("Buffer"));

	for (int i = 0; i < bufferList.count(); i++)
	{
		bufferObj = bufferList[i].toObject();
		name = jsonHelper::getString(bufferObj, QStringLiteral("Buffer_Name"));
		imageWidth = jsonHelper::getInteger(bufferObj, QStringLiteral("Buffer_Width"));
		imageHeight = jsonHelper::getInteger(bufferObj, QStringLiteral("Buffer_Height"));

		if (_appSharedMem.createMemory(name.toStdString(), imageWidth * imageHeight) == false)
		{
			logMsg(QStringLiteral("Shared memory %1 fail to initialize").arg(name));
		}
	}

	//_appEvents.createEvent(std::string("LineTrigger"));
	_appEvents.createEvent(std::string("SoftTrigger"));
	_appEvents.createEvent(std::string("SoftTriggerProductionInspection"));
	_appEvents.createEvent(std::string("VidiNodeInit"));
	_appEvents.createEvent(std::string("VidiNodeOpenWorkspace"));
	_appEvents.createEvent(std::string("AIWireBondInspectionRunImage"));
	_appEvents.createEvent(std::string("VidiNodeRunImage"));
	_appEvents.createEvent(std::string("VidiNodeDeInit"));
	_appEvents.createEvent(std::string("VidiNodeReturn"));
	_appEvents.createEvent(std::string("ManualRunReturn"));
	_appEvents.createEvent(std::string("ImageAcquisitionCompleted"));
	_appEvents.createEvent(std::string("OfflineHybridInspection"));
	_appEvents.createEvent(std::string("GenerateVidiWorkspaceInfo"));
	_appEvents.createEvent(std::string("SnapDone"));

	//BYPASS:VIDI
	_appSharedMem.createMemory(std::string("ErrorInfo"), sizeof(ErrorInfo));
	_appSharedMem.createMemory(std::string("OpenWorkspaceInfo"), sizeof(OpenWorkspaceInfo));
	_appSharedMem.createMemory(std::string("InspectionInfo"), sizeof(InspectionInfo));

	_pErrorInfo = reinterpret_cast<ErrorInfo*>(_appSharedMem.getMemory(std::string("ErrorInfo")));
	_pOpenWorkspaceInfo = reinterpret_cast<OpenWorkspaceInfo*>(_appSharedMem.getMemory(std::string("OpenWorkspaceInfo")));
	_pInspectionInfo = reinterpret_cast<InspectionInfo*>(_appSharedMem.getMemory(std::string("InspectionInfo")));
	//BYPASS:VIDI
}

void VisionApp::iniCamera()
{
	if (jsonHelper::getBool(_systemObj, QStringLiteral("Disable_Camera"), true)) {
		showMsg("WARNING! Camera is disabled! Enable it in system.json");
		ct::logger::warn("Camera is disabled!");
		return;
	}

	CAMManager::instance().loadConfig(QStringLiteral("C:/Advanced/Data/config/camera.json"));

	for (auto key : CAMManager::instance().keys()) {
		ui.comboBox_cameraSelection->addItem(key);
	}

	auto connected = CAMManager::instance().isConnected(_camID);
	nvs::set_background_color(ui.toolButton_cameraStatus, connected ? Qt::green : Qt::red);
}

void VisionApp::iniIOCard()
{
	if (jsonHelper::getBool(_systemObj, QStringLiteral("Disable_IO_Card"), true)) return;

	if (!_ioCard.openIOCard(jsonHelper::getString(_systemObj, QStringLiteral("IO_Card_Device_Desc")), QStringLiteral("C:/Advanced/Data/config/%1.xml").arg(jsonHelper::getString(_systemObj, QStringLiteral("IO_Card_Profile_name")))))
	{
		showMsg(_ioCard.getLastError());
	}
	else
	{
		_isIOCardOpened = true;
		_ioCard.resetPort(0);
		_ioCard.resetPort(1);

		setVisionIO(true);

		LSCManager::instance().attachIO(&_ioCard);
	}
}

void VisionApp::initConfig()
{
}

void VisionApp::iniBufferQueue()
{

	int width = CAMManager::instance().getWidth(_camID);
	int height = CAMManager::instance().getHeight(_camID);
	int channel = CAMManager::instance().getChannel(_camID);
	
	if (width == 0 || height == 0 || channel == 0) {
		ct::logger::error("Invalid camera dimension, failed to create buffer queue.");
		return;
	}

	ct::logger::trace("[iniBufferQueue] Camera size: %d, %d", width, height);
	ct::logger::trace("[iniBufferQueue] Camera channel: %d", channel);

	/*if (!g_bufferQue.createBuffer(30, width, height, channel))
	{
		ct::logger::error("Failed to create buffer queue.");
		return;
	}*/

	ct::logger::info("[iniBufferQueue] Allocated buffer");
}

void VisionApp::initQImageKeys()
{
	_imageWorld.setText("type", "world");
	//_imagePlane.setText("type", "plane");
	_imageMain.setText("type", "fov");
}

void VisionApp::loadConfig()
{
	//lighting IO
	auto j_lightings = jsonHelper::getArray(_systemObj, QStringLiteral("Lightings"));
	_lightingIOs.clear();

	for (int i = 0; i < j_lightings.size(); i++) {

		IOInfo io;
		auto j_obj = j_lightings[i].toObject();

		io.name = j_obj["Name"].toString().toStdString();
		io.port = j_obj["Port"].toInt();
		io.bit = j_obj["Bit"].toInt();
		_lightingIOs.push_back(io);
	}

	auto j_cameras = jsonHelper::getArray(_systemObj, QStringLiteral("Cameras"));
	_cameraIOs.clear();

	for (int i = 0; i < j_cameras.size(); i++) {

		IOInfo io;
		auto j_obj = j_cameras[i].toObject();

		io.name = j_obj["Name"].toString().toStdString();
		io.port = j_obj["Port"].toInt();
		io.bit = j_obj["Bit"].toInt();
		_cameraIOs.push_back(io);
	}
}

std::string VisionApp::charsToStr(char* pCharArray)
{
	std::string str;
	str.clear();

	for (int i = 0;; i++)
	{
		if (pCharArray[i] != '~')
		{
			str = str + pCharArray[i];
		}
		else
		{
			break;
		}
	}

	return str;
}

void VisionApp::strToChars(char* pCharArr, int arrSize, std::string src)
{
	std::string str = src + "~";
	strcpy_s(pCharArr, arrSize, str.c_str());
}

void VisionApp::displayImage(const QImage& img)
{
	qDebug() << "Displaying world iamge....";
	//shift cam view back to tab
	clearImageView();

	_pixmapMain = QPixmap::fromImage(img);


	//_pGraphicsSceneMain = new QMainGraphicsScene(QRectF());
	_sceneBound.setRect(0, 0, _pixmapMain.width(), _pixmapMain.height());
	_pGraphicsSceneMain->setSceneRect(_sceneBound);
	ui.graphicsViewMain->setScene(_pGraphicsSceneMain);

	_pPixmapItemMain = _pGraphicsSceneMain->addPixmap(_pixmapMain);
	_pPixmapItemMain->setZValue((int)UIHierarchy::IMAGE);
	ui.graphicsViewMain->setDragMode(QGraphicsView::RubberBandDrag);
	//ui.graphicsViewMain->fitInView(_pPixmapItemMain, Qt::KeepAspectRatio);
	//ui.graphicsViewMain->centerOn(_pPixmapItemMain);

	QGuiApplication::processEvents();
	qDebug() << "Displaying world image done....";
}

void VisionApp::displayFOV(const QImage & img)
{
	clearDrawingFromFOV();
	_fidLocatedRegion.hide();
	_barcodeLocatedRegion.hide();
	SystemData::instance()._portability.located_region.hide();

	_pixmapFOV = QPixmap::fromImage(img);
	_pPixmapItemFOV->setPixmap(_pixmapFOV);

	_sceneFOV.setRect(0, 0, _pixmapFOV.width(), _pixmapFOV.height());
	_pGraphicsSceneFOV->setSceneRect(_sceneFOV);
	ui.graphicsViewFOV->setScene(_pGraphicsSceneFOV);

	_pPixmapItemFOV->setZValue((int)UIHierarchy::IMAGE);
	ui.graphicsViewFOV->setDragMode(QGraphicsView::RubberBandDrag);
	ui.graphicsViewFOV->show();

	if (_crossHairX) {
		_pGraphicsSceneFOV->removeItem(_crossHairX);
		delete _crossHairX;
		_crossHairX = nullptr;
	}
	_crossHairX = new QLineItem();
	auto lineX = QLineF(0, _pixmapFOV.height() / 2, _pixmapFOV.width(), _pixmapFOV.height() / 2);
	_pGraphicsSceneFOV->addItem(_crossHairX);
	_crossHairX->setup(QRectF(lineX.p1(), lineX.p2()), QColor(0, 255, 127));


	if (_crossHairY) {
		_pGraphicsSceneFOV->removeItem(_crossHairY);
		delete _crossHairY;
		_crossHairY = nullptr;
	}
	_crossHairY = new QLineItem();
	auto lineY = QLineF(_pixmapFOV.width() / 2, 0, _pixmapFOV.width() / 2, _pixmapFOV.height());
	_pGraphicsSceneFOV->addItem(_crossHairY);
	_crossHairY->setup(QRectF(lineY.p1(), lineY.p2()), QColor(0, 255, 127));

	if (!ui.toolButton_showCrossHair->isChecked()) showCrossHair(false);

	QGuiApplication::processEvents();
}

void VisionApp::clearDrawingFromFOV()
{
	clearAllRenderMaps();
}

void VisionApp::drawFOVInWorld(double cx, double cy)
{
	auto w = ScaleManager::instance().fov_to_world(CAMManager::instance().getWidth(_camID));
	auto h = ScaleManager::instance().fov_to_world(CAMManager::instance().getHeight(_camID));

	_worldFOV.setGeometry(QRectF(cx - w / 2, cy - h / 2, w, h));
	_worldFOV.show();
}

void VisionApp::displayPlane()
{
	QImage planeImg(Common::Directory::getRecipeImagesPath() + "plane.jpg");
	displayImage(planeImg);
}

void VisionApp::displayWorld()
{
	QImage planeImg(Common::Directory::getRecipeImagesPath() + "plane.jpg");

	_imageWorld.fill(Qt::black);

	if (!planeImg.isNull())
	{
		QPainter wpainter(&_imageWorld);

		double h_scale = ScaleManager::instance().horizontal_um_per_px();
		double v_scale = ScaleManager::instance().vertical_um_per_px();

		//Get FOV
		double h_cam_mm = util::px_to_mm(CAMManager::instance().getWidth(_camID), h_scale);
		double v_cam_mm = util::px_to_mm(CAMManager::instance().getHeight(_camID), v_scale);

		//get corner as point
		auto topleft_x_mm = _plane.corner_points[(int)Corner::FRONTLEFT].wx - h_cam_mm / 2;
		auto topleft_y_mm = _plane.corner_points[(int)Corner::FRONTLEFT].wy - v_cam_mm / 2;

		QPointF wpx = ScaleManager::instance().to_world_px(QPointF(topleft_x_mm, topleft_y_mm));
		wpainter.drawImage(wpx, planeImg);
	}

	displayImage(_imageWorld);
}

void VisionApp::clearImageView()
{
	if (_pGraphicsSceneMain != nullptr)
	{
		//clearAllDrawings();

		if (_pPixmapItemMain != nullptr)
		{
			_pGraphicsSceneMain->removeItem(_pPixmapItemMain);
			delete _pPixmapItemMain;
			_pPixmapItemMain = nullptr;
		}

		//delete _pGraphicsSceneMain;
		//_pGraphicsSceneMain = nullptr;
	}
}

void VisionApp::clearBufferQueue()
{
	for (auto m : _mIntensityMaps) { //SEETHIS:
		mtrx::free_buffer(m);
	}
	_mIntensityMaps.clear();

	ct::logger::debug("Cleared intensity map buffer");

	updateAllChannels();
	for (const auto& id : LSCManager::instance().channels()) {
		LSCManager::instance().toggle(id, false);
	}

	ct::logger::debug("Cleared dynamic buffer");
}

void VisionApp::drawResult(const bool& result, const QByteArray& jsonData)
{
	QStringList ocrRoiList;
	QStringList blueResultList;

	QJsonDocument doc = QJsonDocument::fromJson(jsonData);
	QJsonObject root = doc.object();
	QJsonObject sampleObj = root[QStringLiteral("sample")].toObject();

	QJsonObject imageObj = jsonHelper::getObject(sampleObj, QStringLiteral("image"));
	QJsonArray markingArr = jsonHelper::getArray(imageObj, QStringLiteral("marking"));
	for (int i = 0; i < markingArr.count(); i++)
	{
		QJsonObject markingArrObj = markingArr[i].toObject();
		QString toolType = jsonHelper::getString(markingArrObj, QStringLiteral("tool_type"));

		QJsonArray viewArr = jsonHelper::getArray(markingArrObj, QStringLiteral("view"));
		if (viewArr != QJsonArray())
		{
			for (int j = 0; j < viewArr.count(); j++)
			{
				QJsonObject viewArrObj = viewArr[j].toObject();
				QString pose = jsonHelper::getString(viewArrObj, QStringLiteral("pose"));
				QJsonObject toolTypeObj = jsonHelper::getObject(viewArrObj, toolType);

				if (toolType == QStringLiteral("blue_read"))
				{
					// feat
					QJsonArray featArr = jsonHelper::getArray(toolTypeObj, QStringLiteral("feat"));
					if (featArr != QJsonArray())
					{
						for (int l = 0; l < featArr.count(); l++)
						{
							QJsonObject featArrObj = featArr[l].toObject();
							QString ID = jsonHelper::getString(featArrObj, QStringLiteral("id"));
							QString loc = jsonHelper::getString(featArrObj, QStringLiteral("loc"));
							QString size = jsonHelper::getString(featArrObj, QStringLiteral("size"));
							QString angle = jsonHelper::getString(featArrObj, QStringLiteral("angle"));

							ocrRoiList.append(QStringLiteral("%1;%2;%3;%4;%5").arg(ID).arg(loc).arg(size).arg(angle).arg(pose));
						}
					}


					QJsonArray matchArr = jsonHelper::getArray(toolTypeObj, QStringLiteral("match"));
					if (matchArr != QJsonArray())
					{
						for (int l = 0; l < matchArr.count(); l++)
						{
							QJsonObject matchArrObj = matchArr[l].toObject();
							blueResultList.prepend(jsonHelper::getString(matchArrObj, QStringLiteral("prettified_string")));
						}
					}
					else
					{
						// might need to handle if nothing matched
					}
				}
			}
		}
	}


	for (auto x : ocrRoiList)
	{
		QString ocrRoi = x;
		QStringList info = ocrRoi.split(";");
		QString id = info.at(0);
		double centerX = info.at(1).split(",").at(0).toDouble();
		double centerY = info.at(1).split(",").at(1).toDouble();
		double width = info.at(2).split("x").at(0).toDouble();
		double height = info.at(2).split("x").at(1).toDouble();
		double angle = info.at(3).toDouble();
		double refX = info.at(4).split(",").at(4).toDouble();
		double refY = info.at(4).split(",").at(5).toDouble();
		QRectF ocrRect(0, 0, width, height);

		ocrRect.moveCenter(QPointF(refX + centerX, refY + centerY));
		ocrRect.adjust(-1, -1, 1, 1);
		drawRect(ocrRect, angle, QColor(230, 185, 5));
	}
}

bool VisionApp::createStreamMapping(const QString& recipeName, const QString& workspaceName)
{
	bool flag;

	QJsonDocument jDoc;

	_streamMappingObj.insert(QStringLiteral("Workspace"), workspaceName);
	jDoc.setObject(_streamMappingObj);

	QString streamMappingFilePath = Common::Directory::getRecipeCurrentPath() + "streamMapping.json";
	flag = saveJson(streamMappingFilePath, jDoc);

	return flag;
}

void VisionApp::getStreamMapping(const QString& recipeName, QString& workspaceName)
{
	workspaceName.clear();

	QJsonFile streamMappingFile;
	if (streamMappingFile.load(QStringLiteral("C:/Advanced/Data/Recipe/%1/streamMapping.json").arg(recipeName)))
	{
		workspaceName = streamMappingFile.getString(QStringLiteral("Workspace"));
	}
}

bool VisionApp::openWorkspace(const QString& workspaceName)
{
	QString workspacePath = QStringLiteral("C:/Advanced/Data/Workspace/%1").arg(workspaceName);
	if (!QFile::exists(workspacePath) || workspaceName.isEmpty())
	{
		closeVidiWorkSpace();
		return false;
	}

	bool VidiNodeFlag = true;
	////BYPASS:VIDI
	this->setCursor(Qt::WaitCursor);
	QCoreApplication::processEvents();

	// init vidi
	qDebug() << "initVidi";
	SetEvent(_appEvents.getEvent(std::string("VidiNodeInit")));
	WaitForSingleObject(_appEvents.getEvent(std::string("VidiNodeReturn")), INFINITE);
	if (_pErrorInfo->_isRunFail)
	{
		this->setCursor(Qt::ArrowCursor);
		showMsg(QStringLiteral("Vidi initialise failed, Error: %1").arg(QString::fromUtf8(charsToStr(_pErrorInfo->_error).c_str())));
		VidiNodeFlag = false;
	}

	// open workspace
	strToChars(_pOpenWorkspaceInfo->_workspaceName, 1024, workspaceName.toStdString());
	strToChars(_pOpenWorkspaceInfo->_workspacePath, 1024, workspacePath.toStdString());
	qDebug() << "setEventVidiOpenWorkSpace";
	SetEvent(_appEvents.getEvent(std::string("VidiNodeOpenWorkspace")));
	qDebug() << "setEventVidiOpenWorkSpace- end";
	WaitForSingleObject(_appEvents.getEvent(std::string("VidiNodeReturn")), INFINITE);
	if (_pErrorInfo->_isRunFail)
	{
		showMsg(QStringLiteral("Open workspace failed, Error: %1").arg(QString::fromUtf8(charsToStr(_pErrorInfo->_error).c_str())));
		VidiNodeFlag = false;
	}

	this->setCursor(Qt::ArrowCursor);
	////BYPASS:VIDI
	return VidiNodeFlag;

	return true;
}

void VisionApp::setVisionIO(bool state)
{
	_ioCard.writeBit(0, 4, state);
}

bool VisionApp::loadJson(QString path, QJsonObject & root)
{
	QString val;
	QFile file;
	QJsonDocument doc;

	if (QFile::exists(path))
	{
		file.setFileName(path);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());
			root = doc.object();

			return true;
		}
	}

	return false;
}

bool VisionApp::loadJson(QString path, QJsonDocument & doc)
{
	QString val;
	QFile file;

	if (QFile::exists(path))
	{
		file.setFileName(path);
		if (file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			val = file.readAll();
			file.close();

			doc = QJsonDocument::fromJson(val.toUtf8());

			return true;
		}
	}

	return false;
}

void VisionApp::recipeSelectionChangedSlot(const QItemSelection & /*newSelection*/, const QItemSelection & /*oldSelection*/)
{
	qDebug() << "recipeSelectionChangedSlot";
	for (int i = 0; i < _dragROI.count(); i++)
	{
		_dragROI.at(i)->setSelected(false);
	}

	//get the text of the selected item
	QModelIndex index = ui.treeViewRecipeExplorer->selectionModel()->currentIndex();
	QStandardItemModel* model = qobject_cast<QStandardItemModel*>(ui.treeViewRecipeExplorer->model());
	//get full path
	//QString path = _recipeModel.filePath(index)

	if (index.parent() != QModelIndex() && model)
	{
		showActiveObject(index);

		QStandardItem* item = model->itemFromIndex(index);
		QString id = item->whatsThis();
		qDebug() << "id:" << id;
		//QString key = index.data(Qt::DisplayRole).toString();
		if (_visionObject.contains(id) == true)
		{
			_visionObject[id].pDragBox->setSelected(true);
		}

		/*for (auto defRect : _defectRectShape)
		{
			auto defID = defRect->data(0).toString();
			auto indexID = defRect->data(1).toString();
			auto viewID = defRect->data(2).toString();
			auto opticID = defRect->data(3).toString();

			if (defID == id)
			{
				ui.lineEdit_currentImageIndex->setText(indexID);
				displayCurrentView(viewID, opticID);
			}
		}*/
	}
}

void VisionApp::resultSelectionChangedSlot(const QItemSelection &, const QItemSelection &)
{
	for (int i = 0; i < _dragROI.count(); i++)
	{
		_dragROI.at(i)->setSelected(false);
	}

	//get the text of the selected item
	QModelIndex index = ui.treeViewResultExplorer->selectionModel()->currentIndex();
	QStandardItemModel* model = qobject_cast<QStandardItemModel*>(ui.treeViewRecipeExplorer->model());

	if (index.parent() != QModelIndex() && model)
	{
		QStandardItem* item = model->itemFromIndex(index);
		QString id = item->whatsThis();
		//QString key = index.data(Qt::DisplayRole).toString();
		if (_visionObject.contains(id) == true)
		{
			_visionObject[id].pDragBox->setSelected(true);
		}
	}
	else
	{
		QStandardItem* pParent = _resultModel.itemFromIndex(index);

		for (int r = 0;; r++)
		{
			QStandardItem* pChild = pParent->child(r);

			if (pChild != nullptr)
			{
				//QString key = pChild->data(Qt::DisplayRole).toString();
				QString id = pChild->whatsThis();
				if (_visionObject.contains(id) == true)
				{
					_visionObject[id].pDragBox->setSelected(true);
				}
			}
			else
			{
				break;
			}
		}
	}
}

void VisionApp::treeViewRecipeExplorerClicked(QModelIndex index)
{
	qDebug() << "treeViewRecipeExplorerClicked:" << index;

	if (index.parent() != QModelIndex())
	{
		showActiveObject(index);

		for (int i = 0; i < _dragROI.count(); i++)
		{
			_dragROI.at(i)->setSelected(false);
		}

		QStandardItemModel* model = qobject_cast<QStandardItemModel*>(ui.treeViewRecipeExplorer->model());

		if (model)
		{
			QStandardItem* item = model->itemFromIndex(index);
			QString id = item->whatsThis();
			//QString key = index.data(Qt::DisplayRole).toString();
			if (_visionObject.contains(id) == true)
			{
				_visionObject[id].pDragBox->setSelected(true);
			}
		}

	}
}

void VisionApp::treeViewObjectExplorerClicked(QModelIndex index)
{
	ui.treeViewObjectExplorer->edit(index);
}

void VisionApp::treeViewRecipeExplorerEntered(QModelIndex index)
{
	showMsg(QString("Item Entered : %1").arg(index.data(Qt::DisplayRole).toString()));
}

void VisionApp::objectModelItemChanged(QStandardItem * item)
{
	QHash<QString, QVariant> objectID = item->data(Qt::UserRole + StandardItemUserRole::OBJECT_SRC).toHash();
	QString fieldEdit = objectID.value(QStringLiteral("FieldName")).toString();
	//QString objectEdit = objectID.value(QStringLiteral("ObjectName")).toString();
	QString objectEdit = objectID.value(QStringLiteral("ObjectID")).toString();

	if (fieldEdit == QStringLiteral("Skip"))
	{
		_visionObject[objectEdit].skip = item->checkState() == Qt::Checked ? true : false;
	}
	else if (fieldEdit == QStringLiteral("ForcedSkip"))
	{
		_visionObject[objectEdit].forcedSkip = item->checkState() == Qt::Checked ? true : false;
	}
	else if (fieldEdit == QStringLiteral("Subtype"))
	{
		//_visionObject[objectEdit].subType = item->data(Qt::EditRole).toString(); //old
	}
	else if (fieldEdit == QStringLiteral("Algorithm"))
	{
		//_visionObject[objectEdit].algoName = item->data(Qt::EditRole).toString(); //old
	}
	else if (fieldEdit == QStringLiteral("Angle"))
	{
		_visionObject[objectEdit].angle = item->data(Qt::EditRole).toDouble();
	}
	else if (fieldEdit == QStringLiteral("View"))
	{
		_visionObject[objectEdit].viewID = item->data(Qt::EditRole).toString();
	}
	else if (fieldEdit == QStringLiteral("Camera"))
	{
		_visionObject[objectEdit].camera = item->data(Qt::EditRole).toString();
	}
}

void VisionApp::modelResetSlot()
{
	showMsg(QStringLiteral("Model Reset"));
}

void VisionApp::rowsRemovedSlot(const QModelIndex & parent, int first, int last)
{
	showMsg(QStringLiteral("first %1, last %2").arg(first).arg(last));
}

void VisionApp::addObject() //old
{
	QModelIndex index = ui.treeViewRecipeExplorer->currentIndex();
	QStandardItem *parent = _recipeModel.itemFromIndex(index);

	if (index.isValid() == true)
	{
		QString defaultKey = QStringLiteral("object_");
		QString newKey = dragROINameGenerator(defaultKey);

		QStandardItem *pObjectItem = new QStandardItem(newKey);

		pObjectItem->setEditable(false);
		parent->appendRow(pObjectItem);

		QVisionObject newObject;
		newObject.objectName = newKey;

		//newObject.pDragBox = drawDragBox(QRectF(0, 0, 300, 300), QColor(0, 255, 127), newKey);
		uidGenerator uidGen;

		newObject.pDragBox = drawVisionAppDragBox(QRectF(0, 0, 300, 300), QColor(0, 255, 127), newKey);
		newObject.objectID = "object" + QString(uidGen.id().c_str());
		newObject.pDragBox->moveTo(_sceneBound.center());

		_visionObject.insert(newObject.objectID, newObject);

		ui.treeViewRecipeExplorer->expandAll();
		showStatus(QStringLiteral("Object added"));
	}
}

void VisionApp::addObjectFromView()
{
	addVisionObject(QRectF(_startDragPos, _endDragPos));
	refreshDragBoxSequence();
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
}

QString VisionApp::addVisionObject(QRectF rect, bool setSelected)
{
	qDebug() << "original rectWidth:" << rect.width() << " rectHeight:" << rect.height();
	// validate vision Object rect
	// check if outside sceneBound(world)
	if (!_pGraphicsSceneMain->sceneRect().contains(rect))
	{
		if (_pGraphicsSceneMain->sceneRect().intersects(rect))
		{
			
			QRectF intersectedRect = _pGraphicsSceneMain->sceneRect().intersected(rect);
			_startDragPos = intersectedRect.topLeft();
			_endDragPos = intersectedRect.bottomRight();

			rect.setTopLeft(_startDragPos);
			rect.setBottomRight(_endDragPos);
			rect.setWidth(rect.width() - 1);
			rect.setHeight(rect.height() - 1);
		}
		else
		{
			
			return "";
		}
	}

	// check if bigger than view rect
	/*int viewWidth = _imageSize.rwidth() * _worldScale;
	int viewHeight = _imageSize.rheight() * _worldScale;*/

	auto worldScale = ScaleManager::instance().world_scale();

	int viewWidth = CAMManager::instance().getWidth(_camID) * worldScale;
	int viewHeight = CAMManager::instance().getHeight(_camID) * worldScale;
	/*if (rect.width() > viewWidth || rect.height() > viewHeight)
	{
		qDebug() << "rectWidth:" << rect.width() <<" rectHeight:" << rect.height() << " viewWidth:" << viewWidth << "viewHeight:" << viewHeight;
		QMessageBox::warning(this, "Oversize Vision Object", "Vision Object cannot be greater than View size! <br> Vision Object will be resized!");
		if (rect.width() > viewWidth)
		{
			_endDragPos.setX(_endDragPos.x() - (rect.width() - viewWidth) - 5);
		}
		if (rect.height() > viewHeight)
		{

			_endDragPos.setY(_endDragPos.y() - (rect.height() - viewHeight) - 5);
		}
		rect.setTopLeft(_startDragPos);
		rect.setBottomRight(_endDragPos);
	}*/

	QString defaultKey = QStringLiteral("object_");
	QString newKey = dragROINameGenerator(defaultKey);

	uidGenerator uidGen;
	QVisionObject newObject;
	newObject.objectName = newKey;
	newObject.objectID = "object" + QString(uidGen.id().c_str());
	newObject.rect = ScaleManager::instance().world_to_fov(rect);

	newObject.pDragBox = drawVisionAppDragBox(QRectF(rect), Qt::white, newKey);
	newObject.pDragBox->setID(newObject.objectID);
	newObject.pDragBox->type((int)DragBoxType::VISIONOBJECT);
	newObject.pDragBox->setSelected(setSelected);

	_visionObject.insert(newObject.objectID, newObject);

	if (includeVisionObject_into_View(newObject.pDragBox))
	{
		newObject.pDragBox->update();
	}

	if (includeVisionObject_into_HeightMap(newObject.pDragBox)) {
		newObject.pDragBox->update();
	}
	showStatus(QStringLiteral("Object added from view"));

	processEvents();

	return newObject.objectID;
}

void VisionApp::duplicateRecipe()
{
	if (!Common::Directory::CurrentRecipe.isEmpty())
	{
		bool ok;
		bool flag = true;
		QString recipeName = QInputDialog::getText(this, tr("Duplicate Recipe"), tr("Recipe Name:"), QLineEdit::Normal, "", &ok, Qt::CoverWindow);

		if (ok && !recipeName.isEmpty())
		{
			QDir dir(Common::Directory::LocalPath + QStringLiteral("recipe//"));

			if (dir.exists(recipeName) == true)
			{
				QMessageBox::StandardButton reply = QMessageBox::question(this, "Duplicate Recipe", "Overwrite Recipe?", QMessageBox::Yes | QMessageBox::No);

				if (reply == QMessageBox::Yes)
				{
					AuditLog::instance().log(QStringLiteral("RECIPE_OVERWRITE_DUPLICATE"), recipeName);
					dir.cd(Common::Directory::LocalPath + QStringLiteral("recipe//%1").arg(recipeName));
					dir.removeRecursively();
				}
				else
				{
					flag = false;
				}
			}

			if (flag == true)
			{
				QDir dir;
				QString srcPath;
				QString dstPath;
				QStringList files;
				QStringList subtypes;
				QStringList algorithms;

				dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//"));
				dir.mkdir(recipeName);

				srcPath = Common::Directory::LocalPath + QStringLiteral("recipe//%1//%1.json").arg(Common::Directory::CurrentRecipe);
				dstPath = Common::Directory::LocalPath + QStringLiteral("recipe//%1//%1.json").arg(recipeName);
				QFile::copy(srcPath, dstPath);

				dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//%1").arg(Common::Directory::CurrentRecipe));
				QStringList fileList = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);
				for (int i = 0; i < fileList.count(); ++i)
				{
					QFileInfo info(fileList.at(i));

					if (info.suffix().contains(QStringLiteral("a"), Qt::CaseInsensitive) == true)
					{
						srcPath = Common::Directory::LocalPath + QStringLiteral("recipe//%1//%2").arg(Common::Directory::CurrentRecipe).arg(fileList.at(i));
						dstPath = Common::Directory::LocalPath + QStringLiteral("recipe//%1//%2").arg(recipeName).arg(fileList.at(i));
						QFile::copy(srcPath, dstPath);
						break;
					}
				}

				dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//%1").arg(recipeName));
				dir.mkdir(QStringLiteral("algorithm"));
				dir.mkdir(QStringLiteral("image"));
				dir.mkdir(QStringLiteral("cache"));

				dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//%1//algorithm//").arg(Common::Directory::CurrentRecipe));
				algorithms = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

				for (int i = 0; i < algorithms.count(); i++)
				{
					dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//%1//algorithm//").arg(recipeName));
					dir.mkdir(algorithms.at(i));

					dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//%1//algorithm//%2//").arg(Common::Directory::CurrentRecipe).arg(algorithms.at(i)));
					subtypes = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

					for (int j = 0; j < subtypes.count(); j++)
					{
						dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//%1//algorithm//%2//").arg(recipeName).arg(algorithms.at(i)));
						dir.mkdir(subtypes.at(j));

						dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//%1//algorithm//%2//%3//").arg(Common::Directory::CurrentRecipe).arg(algorithms.at(i)).arg(subtypes.at(j)));
						files = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

						for (int k = 0; k < files.count(); k++)
						{
							srcPath = Common::Directory::LocalPath + QStringLiteral("recipe//%1//algorithm//%2//%3//%4").arg(Common::Directory::CurrentRecipe).arg(algorithms.at(i)).arg(subtypes.at(j)).arg(files.at(k));
							dstPath = Common::Directory::LocalPath + QStringLiteral("recipe//%1//algorithm//%2//%3//%4").arg(recipeName).arg(algorithms.at(i)).arg(subtypes.at(j)).arg(files.at(k));
							QFile::copy(srcPath, dstPath);
						}
					}
				}

				showStatus(QStringLiteral("Recipe saved"));

				openRecipe(recipeName);
			}
		}
	}
	else
	{
		showMsg(QStringLiteral("Open a recipe to continue"));
	}
}

void VisionApp::archiveRecipe()
{
	if (!Common::Directory::CurrentRecipe.isEmpty())
	{
		QDir dir;
		bool ok;
		bool flag = true;

		dir.setPath(QStringLiteral("%1recipe//%2//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));
		if (dir.exists(QStringLiteral("archive")) == false)
		{
			dir.mkdir(QStringLiteral("archive"));
		}

		QString archiveName = QInputDialog::getText(this, tr("Archive Recipe"), tr("Archive Name:"), QLineEdit::Normal, "", &ok, Qt::CoverWindow);

		if (ok && !archiveName.isEmpty())
		{
			dir.setPath(QStringLiteral("%1recipe//%2//archive").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));
			if (dir.exists(archiveName) == true)
			{
				QMessageBox::StandardButton reply = QMessageBox::question(this, "Archive Recipe", "Overwrite Archive?", QMessageBox::Yes | QMessageBox::No);

				if (reply == QMessageBox::Yes)
				{
					AuditLog::instance().log(QStringLiteral("RECIPE_ARCHIVE_OVERWRITE"), QStringLiteral("%1/%2").arg(Common::Directory::CurrentRecipe, archiveName));
					clearDir(QStringLiteral("%1recipe//%2//archive//%3").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName));
				}
				else
				{
					flag = false;
				}
			}
			else
			{
				dir.setPath(QStringLiteral("%1recipe//%2//archive//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));
				dir.mkdir(archiveName);
			}

			if (flag == true)
			{
				QString srcPath;
				QString dstPath;
				QStringList files;
				QStringList subtypes;
				QStringList algorithms;

				srcPath = QStringLiteral("%1recipe//%2//%2.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
				dstPath = QStringLiteral("%1recipe//%3//archive//%2//%3.json").arg(Common::Directory::LocalPath).arg(archiveName).arg(Common::Directory::CurrentRecipe);
				QFile::copy(srcPath, dstPath);

				dir.setPath(QStringLiteral("%1recipe//%2//archive//%3").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName));
				dir.mkdir(QStringLiteral("algorithm"));

				dir.setPath(QStringLiteral("%1recipe//%2//algorithm//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));
				algorithms = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

				for (int i = 0; i < algorithms.count(); i++)
				{
					dir.setPath(QStringLiteral("%1recipe//%2//archive//%3//algorithm//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName));
					dir.mkdir(algorithms.at(i));

					dir.setPath(QStringLiteral("%1recipe//%2//algorithm//%3//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(algorithms.at(i)));
					subtypes = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

					for (int j = 0; j < subtypes.count(); j++)
					{
						dir.setPath(QStringLiteral("%1recipe//%2//archive//%3//algorithm//%4//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName).arg(algorithms.at(i)));
						dir.mkdir(subtypes.at(j));

						dir.setPath(Common::Directory::LocalPath + QStringLiteral("recipe//%1//algorithm//%2//%3//").arg(Common::Directory::CurrentRecipe).arg(algorithms.at(i)).arg(subtypes.at(j)));
						files = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

						for (int k = 0; k < files.count(); k++)
						{
							srcPath = QStringLiteral("%1recipe//%2//algorithm//%3//%4//%5").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(algorithms.at(i)).arg(subtypes.at(j)).arg(files.at(k));
							dstPath = QStringLiteral("%1recipe//%2//archive//%3//algorithm//%4//%5//%6").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName).arg(algorithms.at(i)).arg(subtypes.at(j)).arg(files.at(k));
							QFile::copy(srcPath, dstPath);
						}
					}
				}

				showStatus(QStringLiteral("Archive done"));
			}
		}
	}
	else
	{
		showMsg(QStringLiteral("Open a recipe to continue"));
	}
}

void VisionApp::restoreRecipe()
{
	if (!Common::Directory::CurrentRecipe.isEmpty())
	{
		QDir dir;
		bool ok = true;
		QString archiveName;

		dir.setPath(QStringLiteral("%1recipe//%2//archive//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));
		QStringList archives = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

		if (archives.count() > 0)
			archiveName = QInputDialog::getItem(this, tr("Open Recipe"), tr("Recipe:"), archives, 0, false, &ok, Qt::CoverWindow);

		if (ok)
		{
			QString srcPath;
			QString dstPath;
			QStringList files;
			QStringList subtypes;
			QStringList algorithms;

			AuditLog::instance().log(QStringLiteral("RECIPE_RESTORE"), QStringLiteral("%1 <- %2").arg(Common::Directory::CurrentRecipe, archiveName));
			QFile::remove(QStringLiteral("%1recipe//%2//%2.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));
			clearDir(QStringLiteral("%1recipe//%2//algorithm//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));


			srcPath = QStringLiteral("%1recipe//%3//archive//%2//%3.json").arg(Common::Directory::LocalPath).arg(archiveName).arg(Common::Directory::CurrentRecipe);
			dstPath = QStringLiteral("%1recipe//%2//%2.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
			QFile::copy(srcPath, dstPath);

			dir.setPath(QStringLiteral("%1recipe//%2//archive//%3//algorithm//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName));
			algorithms = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

			for (int i = 0; i < algorithms.count(); i++)
			{
				dir.setPath(QStringLiteral("%1recipe//%2//algorithm//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));
				dir.mkdir(algorithms.at(i));

				dir.setPath(QStringLiteral("%1recipe//%2//archive//%3//algorithm//%4//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName).arg(algorithms.at(i)));
				subtypes = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

				for (int j = 0; j < subtypes.count(); j++)
				{
					dir.setPath(QStringLiteral("%1recipe//%2//algorithm//%3//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(algorithms.at(i)));
					dir.mkdir(subtypes.at(j));

					dir.setPath(QStringLiteral("%1recipe//%2//archive//%3//algorithm//%4//%5").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName).arg(algorithms.at(i)).arg(subtypes.at(j)));
					files = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);

					for (int k = 0; k < files.count(); k++)
					{
						srcPath = QStringLiteral("%1recipe//%2//archive//%3//algorithm//%4//%5//%6").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(archiveName).arg(algorithms.at(i)).arg(subtypes.at(j)).arg(files.at(k));
						dstPath = QStringLiteral("%1recipe//%2//algorithm//%3//%4//%5").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(algorithms.at(i)).arg(subtypes.at(j)).arg(files.at(k));
						QFile::copy(srcPath, dstPath);
					}
				}
			}

			openRecipe(Common::Directory::CurrentRecipe);

			showStatus(QStringLiteral("Restore done"));
		}
	}
	else
	{
		showMsg(QStringLiteral("Open a recipe to continue"));
	}
}

void VisionApp::recipeChanged()
{
	//Prompt user at UI recipe has changed and need save
	//TODO: Make sure data really changed. In case user undo changes, need verify if there's really a change
}

void VisionApp::clearDir(const QString& path)
{
	QDir dir(path);

	dir.setFilter(QDir::NoDotAndDotDot | QDir::Files);
	for (auto x : dir.entryList())
		dir.remove(x);

	dir.setFilter(QDir::NoDotAndDotDot | QDir::Dirs);
	for (auto x : dir.entryList())
	{
		QDir subDir(dir.absoluteFilePath(x));
		subDir.removeRecursively();
	}
}

void VisionApp::editTemplate()
{
	//templates now link to the in-app algos; edit them on the Algo Setup page
	auto tmpl = _templateLibraryTab->currentAlgoTemplate();
	if (tmpl) {
		QSignalBlocker sb(ui.comboBox_algoType);
		ui.comboBox_algoType->setCurrentIndex((int)tmpl->algo());
		ui.stackedWidget_algoParams->setCurrentIndex((int)tmpl->algo());
		refreshAlgoLocatorUI();
	}
	toPage(UIPage::ALGO_SETUP);
	updateAlgoRoiVisibility();
}

void VisionApp::findVisionObject()
{

	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->isSelected())
		{
			auto rect = _dragROI[i]->getGeometry();
			if (rect.width() != 0 && rect.height() != 0)
			{
				ui.graphicsViewMain->fitInView(_pPixmapItemMain, Qt::KeepAspectRatio);
				ui.graphicsViewMain->centerOn(_pPixmapItemMain);
				ui.graphicsViewMain->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
				ui.graphicsViewMain->show();



				qreal sx = 1;
				qreal sy = 1;
				qreal ratioX = rect.width() / _pGraphicsSceneMain->width();
				qreal ratioY = rect.height() / _pGraphicsSceneMain->height();

				if (ratioX < 0.40) {
					sx = _pGraphicsSceneMain->width() / ((rect.width() / 2) * 5);

					if (ratioY < 0.40) {
						sy = _pGraphicsSceneMain->height() / ((rect.height() / 2) * 5);
					}
					else if ((0.40 >= ratioY) && (ratioY < 1.0)) {
						sx = 1; sy = 1;
					}
				}
				else {
					sx = 1; sy = 1;
				}

				if (sx < sy) { sy = sx; }
				if (sy < sx) { sx = sy; }
				if (sx > 30) { sx = 30; }
				if (sy > 30) { sy = 30; }
				if (sx <= 0) { sx = 1; }
				if (sy <= 0) { sy = 1; }

				ui.graphicsViewMain->scale(sx, sy);
				QPointF centerPt(rect.center());
				ui.graphicsViewMain->centerOn(centerPt);
			}
			break;
		}
	}

}

void VisionApp::promptInspSelection()
{
	QStringList list;
	list << "Current Path" << "Sample Images" << "Production";
	auto folderType = promptComboBox(list, "Offline Inspection", "Select folder type");

	_inspQueue = {};

	if (folderType == "Current Path") {
		_inspQueue.push(Common::Directory::CurrentImageSetPath);
		runQueuedInsp();
		return;
	}
	
	QString loadPath;
	if (folderType == "Production") {
		loadPath = Common::Directory::ProductionPath();
	}
	else {
		loadPath = Common::Directory::getRecipeSampleImagePath();
	}

	for (auto s : promptFolderSelection(loadPath, this)) {

		//if (!s.contains(Common::Directory::CurrentRecipe)) continue;

		if (s.contains("Data/Production")) {
			s += "/Images/";
		}
		else if (s.contains("SampleImages")) {
			s += "/";
		}

		qDebug() << s;

		_inspQueue.push(s);
	}
	runQueuedInsp();
}

void VisionApp::runQueuedInsp()
{
	if (_inspQueue.empty()) return;
	
	ct::logger::info("Run queued insp");
	Common::Directory::CurrentImageSetPath = _inspQueue.front();
	_inspQueue.pop();
	ct::logger::info("Run queued insp pop");
	run();
}

void VisionApp::run()
{
	resetStopRunFlags();

	runOffline();
}

void VisionApp::testRun()
{
	ui.textEdit_loopStatus->clear();

	auto runType = ui.comboBox_runType->currentText();

	/*
	* Exact match, and deliberately BEFORE the contains() chain below. That chain dispatches
	* on substrings - "2D", "3D", "Full", then "Acquisition" / "Inspection" - so a run type
	* matching none of them falls through every branch and the Run button silently does
	* nothing. Keep "3D" out of this item's text for the same reason.
	*/
	if (runType == QStringLiteral("Profiler Scan Test")) {
		runProfilerScanTest();
		return;
	}

	if (runType == QStringLiteral("Production Scan Check")) {
		runProductionScanTest();
		return;
	}

	bool online = ui.toolButton_toggleOnlineRun->isChecked();
	auto run1stFOVOnly = ui.checkBox_runOneFOVonly->isChecked();
	bool disable2DInspection = ui.checkBox_disable2D->isChecked();

	SystemData::instance()._offlineRun = !online;

	if (ui.checkBox_runLooping->isChecked()) {
		_loop = ui.lineEdit_numOfLoops->text().toInt();
	}
	else {
		_loop = 0;
	}
	_testRunLoopingNoUnload = online && ui.checkBox_runLooping->isChecked() && runType.contains("Inspection");

	_jobThread.enableRun1stFOVOnly(run1stFOVOnly);

	if (runType.contains("2D")) {
		_enable2D = true;
		_enable3D = false;
	}
	else if (runType.contains("3D")) {
		_enable2D = false;
		_enable3D = true;
	}
	else if (runType.contains("Full")) {
		_enable2D = true;
		_enable3D = true;
	}

	if (disable2DInspection && (runType.contains("Inspection") || runType == "Full Stationary")) {
		_enable2D = false;
	}


	if (runType.contains("Acquisition")) {
		recordMemory(QString("Start #%1").arg(_loop));

		startAcquisition();
	}
	else if (runType.contains("Inspection")) {
		if (online) {
			startProduction();
		}
		else {
			run();
		}
	}
	else if (runType == "Full Stationary") {
		if (online) {
			recordMemory(QString("Start #%1").arg(_loop));
			startProductionS();
		}
		else {
		}
	}
}

void VisionApp::stopRun(bool clearInspQueue)
{
	//_progressDialog = nullptr;
	for (int i = 0; i < 3; i++) { //X, Y, Z
		MotionController::instance().stop_move(_motionID, i);
	}

	InspectionThread::instance().setActive(false);

	progressBarRelease();

	setCameraAngle(_prevCamAlignedAngle);

	_brightnessOverrides.clear();

	_jobThread.stopRun();
	emit signalStopSRX();

	_stopRun = true;


	ProfilerManager::instance().stop(_profilerID);

	g_forceStopInspLoop = true;

	if(clearInspQueue) _inspQueue = {};


	ct::logger::info("Force stop success");
}

void VisionApp::userClickStopRun()
{
	_loop = 0;
	_testRunLoopingNoUnload = false;
	_autoCalPending = false; // operator aborted: do not generate the auto-cal report
	stopRun();
	vs_stopElapseTimer();

}

void VisionApp::resetStopRunFlags()
{
	_stopRun = false;

	g_forceStopInspLoop = false;
}

void VisionApp::runOffline()
{
	SystemData::instance()._offlineRun = true;
	SystemData::instance().StartInspectionTimer = QDateTime::currentDateTime();

	uidGenerator uidGen;
	_currentProductionID = uidGen.id().c_str();

	//g_prevEmaVolumeByGroup.clear();
	//Set machine mode
	_processType = ProcessType::PRODUCTION;

	_fid_image.clear();
	_fid_image.resize(_fiducialInfos.size());
	_inspMode = false;
	_inspectionThreadBusy = true;
	int lightingCount = 0;
	auto optics = _recipeOptics.constBegin();
	while (optics != _recipeOptics.constEnd())
	{
		if (optics.value().type == ct::s_color && CAMManager::instance().getChannel(_camID) == 1) lightingCount = lightingCount + 3;
		else lightingCount++;
		optics++;
	}

	if (_enable3D) lightingCount++;
	int viewCount = _views.count();
	if (ui.checkBox_runOneFOVonly->isChecked()) viewCount = 1;

	clearAllDrawings();
	QPointF wpx = ScaleManager::instance().to_world_px(QPointF(_plane.corner_points[(int)Corner::FRONTLEFT].wx, _plane.corner_points[(int)Corner::FRONTLEFT].wy));
	g_time.reset_timer();
	//run Offline Test

	InspStatus inspStatus;
	inspStatus.productionMode = false;
	inspStatus.inspectionStartTime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
	_inspStatus = inspStatus;
	readDefectPriorityList();
	if (!loadProductionInfoJson())
	{
		if (_enableBarcode)
		{
			emit readBarcode(0, false);
			if (true)
			{
				std::string id;
				uint64_t microseconds_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
				id = std::to_string(microseconds_since_epoch);
				SystemData::instance()._currentBarcode = QString("No_Barcode" + QString::fromStdString(id)).toStdString();
			}
		}
	}
	setupProductionDir();
	

	if (!_enableEmap)
	{
		QMessageBox messageBox;
		messageBox.setWindowTitle("EMAP DISABLED!");
		messageBox.setText("<font color=\"red\"><b>!!INCOMING EMAP DISABLED WARNING!!</b></font><br>Yes to continue process<br>No to stop the lot inspection");
		messageBox.setIcon(QMessageBox::Warning);

		// Set a bigger font size for the message text
		QFont font = messageBox.font();
		font.setPointSize(24); // Adjust the font size as needed
		messageBox.setFont(font);

		// Adjust the size of the message box
		messageBox.setMinimumWidth(1600); // Adjust the width as needed
		messageBox.setMinimumHeight(1600); // Adjust the height as needed
		messageBox.setWindowFlag(Qt::WindowStaysOnTopHint);

		// Add Yes and No buttons
		messageBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);

		// Execute and get the user's response
		int result = messageBox.exec();

		if (result == QMessageBox::Yes) {
			// Handle Yes response
		}
		else if (result == QMessageBox::No) {
			// Handle No response
		}
	}

	//readEmap();
	
	// reset skip vo 
	for (auto& vo : _visionObject)
	{
		vo.skip = false;
		vo.voBarcode = "";
		vo.voBarcodeName = "";
		vo.localBarcode = "";
	}
	if (_enableVisionObjectSampling)
	{
		visionObjectSampling(); // !!will crash if thre is no unit inside existing view!!
	}
	

	//setup InspectionThread
	_templateLibraryTab->loadAlgoTemplateListMask();
	_templateLibraryTab->reloadAlgoTemplateListMetaData();
	_pInspectionInfo->_isCollectImage = jsonHelper::getBool(_systemObj, QStringLiteral("Save_Inspection_Image"));
	SystemData::instance()._saveInspImages = _saveInspImg; //worker-thread mirror (reader image saving)

	qDebug() << "total Num of Views:" << getNumOfViewToProcess(_datasetIndexIds);
	qDebug() << "total Num of LineScans:" << getNumOfLineScanToProcess(_datasetIndexIds);


	if (ui.checkBox_runOneFOVonly->isChecked()) {
		g_viewIndex = 1;
		if (_enable2D && _enable3D) g_viewIndex = 2;
	}
	else {
		g_viewIndex = getNumOfViewToProcess(_datasetIndexIds);
		if (_enable2D && _enable3D) g_viewIndex = getNumOfViewToProcess(_datasetIndexIds) + getNumOfLineScanToProcess(_datasetIndexIds);
		else if(_enable3D) g_viewIndex = getNumOfLineScanToProcess(_datasetIndexIds);
	}

	if (_enableClassificationDataCollection)
	{
		QMessageBox::StandardButton reply = QMessageBox::question(this, "Allow Data Collection", "Do you want to collect data for classification?", QMessageBox::Yes | QMessageBox::No);
		if (reply == QMessageBox::Yes) g_enableClassificationDataCollection = true;
	}
	else g_enableClassificationDataCollection = false;
	//reset forceStopInspectionLoopFlag
	g_forceStopInspLoop = false;

	//start InspectionThread
	//insert Images

	int progressBarValue = getNumOfViewToProcess(_datasetIndexIds) * 2;
	if (_enable2D && _enable3D) progressBarValue = getNumOfViewToProcess(_datasetIndexIds) * 2 + getNumOfLineScanToProcess() * 2;
	else if (_enable3D) progressBarValue = getNumOfLineScanToProcess(_datasetIndexIds)*2;
	
	int viewMultiplier = 2;
	if (_enable2D && _enable3D) viewMultiplier = 4;
	
	if (ui.checkBox_runOneFOVonly->isChecked()) {
		progressBarSetup("Running Offline Inspection...", 1 * viewMultiplier + 5, true);
	}
	else {
		progressBarSetup("Running Offline Inspection...", progressBarValue + 5, true);
	}

	_productionPage->startElapseTime();

	ct::logger::info("Done offline inspection init");
	
	auto lineScansCopy = _lineScans;
	auto recipeOptics3DCopy = _recipeOptics3D;
	std::thread([lineScansCopy, recipeOptics3DCopy,this]() {
		this->loadImagesForOfflineInspection(lineScansCopy, recipeOptics3DCopy);
	}).detach();

	//copyAllFilesToProduction(Common::Directory::CurrentImageSetPath, Common::Directory::getProductionImageSetPath());
}

void VisionApp::load_2dOffline_image()
{
	QHash<QString, QView>::const_iterator view = _views.constBegin();
	while (view != _views.constEnd())
		{
			ct::logger::info("start load View");
			if (_stopRun) {
				ct::logger::info("load offline view returned!!!");
				return;
			}

			if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
				ct::logger::warn("[Offline] Memory overload, waiting for more memory to proceed 3D acquisition...");

				while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
					if (_stopRun) break;
					os_tool::goSleep(1000);
				}
				ct::logger::warn("[Offline] Sufficient memory, proceed 3D acquisition.");
			}

			if (view.value().type == ct::s_child_view) {
				view++;
				continue;
			}
			ct::logger::info("done load View");

			QVector<FrameInfo> IInfos;
			auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), view.value(), _mainOptics[_camID], _recipeOptics);

			QHash<QString, QString> imgPaths;
			imgPaths = ipf.getAllOpticPaths();

			QHash<QString, QString>::const_iterator imgPath = imgPaths.constBegin();

			while (imgPath != imgPaths.constEnd())
			{
				QString iPath = util::convert_to_BMP_ext(imgPath.value());
				ct::logger::trace("[Offline] Load 2D: %s", iPath.toStdString().c_str());
				MIL_INT bandSize = 3;

				FrameInfo IInfo;
				mtrx::SharedMilID img = nullptr;
				if (QFileInfo::exists(iPath))
				{
					MIL_INT sizeX = 0, sizeY = 0;
					MIL_INT type = 0;
					MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_BAND, &bandSize);
					MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_X, &sizeX);
					MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_Y, &sizeY);
					MbufDiskInquireA(iPath.toStdString().c_str(), M_TYPE, &type);

					img = mtrx::MbufPoolManager::instance().acquire(sizeX, sizeY, bandSize, type);

					MIL_INT imgType = M_JPEG_LOSSY;
					if (util::isPNG(iPath)) imgType = M_PNG;
					if (util::isBMP(iPath)) imgType = M_BMP;

					MbufLoadA(iPath.toStdString().c_str(), img->id());
					copyFileToFolder(iPath, Common::Directory::getProductionImageSetPath());
				}
				else
				{
					img = mtrx::MbufPoolManager::instance().acquire(5120, 5120, 3, 8 + M_UNSIGNED);
					MbufClear(img->id(), M_BLACK);
					ct::logger::error("Path not found: %s", iPath.toStdString().c_str());
				}

				MIL_INT width, height;
				MbufInquire(img->id(), M_SIZE_X, &width);
				MbufInquire(img->id(), M_SIZE_Y, &height);
				size_t  imgSize = width * height;

				QString optType = ct::s_color;
				if (bandSize == 1) optType = ct::s_mono;

				uidGenerator uidGen;
				IInfo.width = (int)width;
				IInfo.height = (int)height;
				IInfo.bufferSize = (int)imgSize;
				IInfo.timeStamp = QString(uidGen.id().c_str()).toInt();
				IInfo.cameraID = _camID;
				IInfo.viewID = view.value().id;
				IInfo.opticID = imgPath.key();
				IInfo.type = optType;
				IInfo.pImage = img;
				IInfos.push_back(IInfo);

				imgPath++;
			}


			_progressValue++;
			if (_progressDialog)_progressDialog->setValue(_progressValue);

			if (ui.checkBox_runOneFOVonly->isChecked()) break;

			if (_stopRun) {
				ct::logger::info("load offline view returned!!!");
				return;
			}
			++view;
		}
}

void VisionApp::load_2dOffline_image_parallel()
{
	QVector<QString> viewIDs = _views.keys().toVector();
#pragma omp parallel for
	for (int i = 0; i < viewIDs.size(); ++i)
	{
		const QString& viewKey = viewIDs[i];
		const auto& currentView = _views[viewKey];
		if (currentView.type == ct::s_child_view)
			continue;

		if (_stopRun) continue;

		
		if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
			ct::logger::warn("[Offline] Memory overload, waiting...");
			while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
				if (_stopRun) break;
				os_tool::goSleep(1000);
			}
		}

		
		QVector<FrameInfo> threadInfos;
		auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), currentView, _mainOptics[_camID], _recipeOptics);
		QHash<QString, QString> imgPaths = ipf.getAllOpticPaths();
		QVector<QString> opticKeys = imgPaths.keys().toVector();

		
		for (int j = 0; j < opticKeys.size(); ++j)
		{
			const QString& opticID = opticKeys[j];
			QString iPath = util::convert_to_BMP_ext(imgPaths[opticID]);
			MIL_INT bandSize = 3;
			mtrx::SharedMilID img = M_NULL;
			FrameInfo IInfo;

			if (QFileInfo::exists(iPath)) {
				MIL_INT sizeX = 0, sizeY = 0;
				MIL_INT type = 0;
				MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_BAND, &bandSize);
				MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_X, &sizeX);
				MbufDiskInquireA(iPath.toStdString().c_str(), M_SIZE_Y, &sizeY);
				MbufDiskInquireA(iPath.toStdString().c_str(), M_TYPE, &type);

				img = mtrx::MbufPoolManager::instance().acquire(sizeX, sizeY, bandSize, type);

				MIL_INT imgType = M_JPEG_LOSSY;
				if (util::isPNG(iPath)) imgType = M_PNG;
				if (util::isBMP(iPath)) imgType = M_BMP;

				MbufLoadA(iPath.toStdString().c_str(), img->id());
				copyFileToFolder(iPath, Common::Directory::getProductionImageSetPath());
			}
			else {
				img = mtrx::MbufPoolManager::instance().acquire(5120, 5120, 3, 8 + M_UNSIGNED);
				MbufClear(img->id(), M_BLACK);
				ct::logger::error("Path not found: %s", iPath.toStdString().c_str());
			}

			MIL_INT width = 0, height = 0;
			MbufInquire(img->id(), M_SIZE_X, &width);
			MbufInquire(img->id(), M_SIZE_Y, &height);

			QString optType = (bandSize == 1) ? ct::s_mono : ct::s_color;
			uidGenerator uidGen;

			IInfo.width = (int)width;
			IInfo.height = (int)height;
			IInfo.bufferSize = (int)(width * height);
			IInfo.timeStamp = QString(uidGen.id().c_str()).toInt();
			IInfo.cameraID = _camID;
			IInfo.viewID = currentView.id;
			IInfo.opticID = opticID;
			IInfo.type = optType;
			IInfo.pImage = img;

			// Protect global map write
#pragma omp critical
			threadInfos.push_back(IInfo);
		}

		// Push into inspection queue safely
#pragma omp critical
		{
			_progressValue++;

			if (_progressDialog) {
				QMetaObject::invokeMethod(_progressDialog, "setValue", Qt::QueuedConnection,
					Q_ARG(int, _progressValue));
			}
		}
	}
}

void VisionApp::load_3dOffline_image()
{
	std::map<QString, QString> scanSequence;
	for (const auto& l : _lineScans) {
		if (l.type == ct::s_child_linescan) continue;
		if (l.id == "") continue;

		scanSequence.insert({ QString("%1_%2").arg(l.px.cx).arg(l.id), l.id });
	}

	//check for failed load and reduce g_viewIndex
	for (const auto& seq : scanSequence) {

		auto l = _lineScans[seq.second];

		QVector<QString> optics3D;

		for (auto& o : _recipeOptics3D) {
			if (o.exposureMode == ct::s_parallel) {
				optics3D.push_back(o.id + "E1");
				optics3D.push_back(o.id + "E2");
			}
			else {
				optics3D.push_back(o.id);
			}
		}

		for (auto& opticID : optics3D) {

			QString hid = l.id + "_HeightMap_" + opticID;
			QString iid = l.id + "_IMap";

			QString path_hm = Common::Directory::CurrentImageSetPath + "/" + hid + ".tiff";
			QString ipath = Common::Directory::CurrentImageSetPath + "/" + iid + ".jpg";

			ct::logger::trace("[Offline] Load IMap: %s", ipath.toStdString().c_str());
			ct::logger::trace("[Offline] Load 3D: %s", path_hm.toStdString().c_str());
			ct::logger::info("[Offline] IMap ID: %s", iid.toStdString().c_str());

			if (!QFile::exists(ipath) || !QFile::exists(path_hm)) {
				ct::logger::error("[Offline] Failed to load intensity map: %s", ipath.toStdString().c_str());
				g_viewIndex--;
				ct::logger::info("g_viewIndex after failed load 3d: %d", g_viewIndex);
				continue;
			}
		}
	}

	//actual Load
	for (const auto& seq : scanSequence) {

		auto l = _lineScans[seq.second];

		QVector<FrameInfo> IInfos;

		QVector<QString> optics3D;

		for (auto& o : _recipeOptics3D) {
			if (o.exposureMode == ct::s_parallel) {
				optics3D.push_back(o.id + "E1");
				optics3D.push_back(o.id + "E2");
			}
			else {
				optics3D.push_back(o.id);
			}
		}

		for (auto& opticID : optics3D) {

			if (_stopRun) {
				ct::logger::info("load offline linescan returned!!!");
				return;
			}

			if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
				ct::logger::warn("[Offline] Memory overload, waiting for more memory to proceed 3D acquisition...");

				while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
					if (_stopRun) break;
					os_tool::goSleep(1000);
				}
				ct::logger::warn("[Offline] Sufficient memory, proceed 3D acquisition.");
			}

			QString hid = l.id + "_HeightMap_" + opticID;
			QString iid = l.id + "_IMap";

			QString path_hm = Common::Directory::CurrentImageSetPath + "/" + hid + ".tiff";
			QString ipath = Common::Directory::CurrentImageSetPath + "/" + iid + ".jpg";

			ct::logger::trace("[Offline] Load IMap: %s", ipath.toStdString().c_str());
			ct::logger::trace("[Offline] Load 3D: %s", path_hm.toStdString().c_str());
			ct::logger::info("[Offline] IMap ID: %s", iid.toStdString().c_str());

			if (!QFile::exists(ipath) || !QFile::exists(path_hm)) {
				ct::logger::error("[Offline] Failed to load intensity map: %s", ipath.toStdString().c_str());
				g_viewIndex--;
				ct::logger::info("g_viewIndex after failed load 3d: %d", g_viewIndex);
				continue;
			}

			ct::logger::info("[Offline] loading heightmap");
			auto mBuf = MbufRestoreA(path_hm.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
			auto mImap = MbufRestoreA(ipath.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
			auto sharedMbuf = mtrx::MbufPoolManager::instance().attach(mBuf, mtrx::PoolDestructorType::FREE_BUFFER);
			auto sharedImap = mtrx::MbufPoolManager::instance().attach(mImap, mtrx::PoolDestructorType::FREE_BUFFER);
			ct::logger::info("[Offline] loading heightmap - end");

			copyFileToFolder(path_hm, Common::Directory::getProductionImageSetPath());
			copyFileToFolder(ipath, Common::Directory::getProductionImageSetPath());

			if (mBuf) {
				FrameInfo IInfo;

				auto w = mtrx::get_width(mBuf);
				auto h = mtrx::get_height(mBuf);
				uidGenerator uidGen;
				IInfo.width = w;
				IInfo.height = h;
				IInfo.bufferSize = w * h;
				IInfo.timeStamp = QString(uidGen.id().c_str()).toInt();
				IInfo.cameraID = _camID;
				IInfo.viewID = l.id;
				IInfo.type = ct::s_height_map;
				IInfo.opticID = opticID;
				IInfo.pHeightMap = sharedMbuf;
				IInfo.pImage = sharedImap;

				ct::logger::info("[Offline] Append heightmap: %s", l.name.toStdString().c_str());
				IInfos.push_back(IInfo);

			}
		}

		if (IInfos.size() > 0)
		{
			_progressValue++;
			if (_progressDialog)_progressDialog->setValue(_progressValue);
		}		

		if (ui.checkBox_runOneFOVonly->isChecked()) break;
	}
}

void VisionApp::load_3dOffline_image_parallel(QHash <QString, QLineScan> linescan, QHash<QString, OpticsInfo3D> optic3D)
{
	std::map<QString, QString> scanSequence;
	for (const auto& l : linescan) {
		if (l.type == ct::s_child_linescan) continue;
		if (l.id == "") continue;

		scanSequence.insert({ QString("%1_%2").arg(l.px.cx).arg(l.id), l.id });
	}

	std::vector<QString> scanKeys;
	for (const auto& kv : scanSequence) scanKeys.push_back(kv.second);

	QVector<QString> optics3D;
	for (const auto& o : optic3D) {
		if (o.exposureMode == ct::s_parallel) {
			optics3D.push_back(o.id + "E1");
			optics3D.push_back(o.id + "E2");
		}
		else {
			optics3D.push_back(o.id);
		}
	}

	//check for failed load and reduce g_viewIndex
	for (int i = 0; i < scanKeys.size(); ++i)
	{
		const QString& scanID = scanKeys[i];
		const auto it = _lineScans.constFind(scanID);
		if (it == linescan.cend()) { /* log & continue */ }
		const auto l = it.value();   // safe copy
		QVector<FrameInfo> threadInfos;

		for (const auto& opticID : optics3D)
		{
			QString hid = l.id + "_HeightMap_" + opticID;
			QString iid = l.id + "_IMap";
			QString path_hm = Common::Directory::CurrentImageSetPath + "/" + hid + ".tiff";
			QString ipath = Common::Directory::CurrentImageSetPath + "/" + iid + ".jpg";

			if (!QFile::exists(ipath) || !QFile::exists(path_hm)) {
				ct::logger::error("[Offline] Failed to load intensity map: %s", ipath.toStdString().c_str());
				g_viewIndex--;
				ct::logger::info("g_viewIndex after failed load 3d: %d", g_viewIndex);
				continue;
			}
		}
	}

	//actual Load
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < scanKeys.size(); ++i)
	{
		const QString& scanID = scanKeys[i];
		const auto it = _lineScans.constFind(scanID);
		if (it == linescan.cend()) { /* log & continue */ }
		const auto l = it.value();   // safe copy
		QVector<FrameInfo> threadInfos;

		for (const auto& opticID : optics3D)
		{
			if (_stopRun) continue;

			if (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
				ct::logger::warn("[Offline] Memory overload, waiting...");

				while (util::isRAMOver(SystemData::instance()._maxAllowRamUsage)) {
					if (_stopRun) break;
					os_tool::goSleep(1000);
				}

				ct::logger::warn("[Offline] Resumed 3D acquisition.");
			}

			QString hid = l.id + "_HeightMap_" + opticID;
			QString iid = l.id + "_IMap";
			QString path_hm = Common::Directory::CurrentImageSetPath + "/" + hid + ".tiff";
			QString ipath = Common::Directory::CurrentImageSetPath + "/" + iid + ".jpg";

			if (!QFile::exists(ipath) || !QFile::exists(path_hm)) {
				ct::logger::error("[Offline] Failed to load intensity map: %s", ipath.toStdString().c_str());
				/*g_viewIndex--;
				ct::logger::info("g_viewIndex after failed load 3d: %d", g_viewIndex);*/
				continue;
			}

			MIL_ID mBuf = MbufRestoreA(path_hm.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
			MIL_ID mImap = MbufRestoreA(ipath.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);
			auto sharedMbuf = mtrx::MbufPoolManager::instance().attach(mBuf, mtrx::PoolDestructorType::FREE_BUFFER);
			auto sharedImap = mtrx::MbufPoolManager::instance().attach(mImap, mtrx::PoolDestructorType::FREE_BUFFER);

			copyFileToFolder(path_hm, Common::Directory::getProductionImageSetPath());
			copyFileToFolder(ipath, Common::Directory::getProductionImageSetPath());

			if (mBuf) {
				FrameInfo IInfo;
				auto w = mtrx::get_width(mBuf);
				auto h = mtrx::get_height(mBuf);
				uidGenerator uidGen;
				IInfo.width = w;
				IInfo.height = h;
				IInfo.bufferSize = w * h;
				IInfo.timeStamp = QString(uidGen.id().c_str()).toInt();
				IInfo.cameraID = _camID;
				IInfo.viewID = l.id;
				IInfo.type = ct::s_height_map;
				IInfo.opticID = opticID;
				IInfo.pHeightMap = sharedMbuf;
				IInfo.pImage = sharedImap;
				threadInfos.push_back(IInfo);
			}
		}

		// 🔒 Push to inspection queue safely
#pragma omp critical(queue)
		{
			if (threadInfos.size() > 0)
			{
					_progressValue++;

				if (_progressDialog) {
					QMetaObject::invokeMethod(_progressDialog, "setValue", Qt::QueuedConnection,
						Q_ARG(int, _progressValue));
				}
			}			
		}

		if (ui.checkBox_runOneFOVonly->isChecked()) continue;
	}
}

void VisionApp::copyAllFilesToProduction(QString sourcePath, QString destPath)
{
	QDir srcDir(sourcePath);
	if (!srcDir.exists()) {
		qDebug() << "Source path does not exist:" << sourcePath;
		return;
	}

	QDir().mkpath(destPath);  // Ensure destination exists

	QDirIterator it(sourcePath, QDir::Files | QDir::NoSymLinks, QDirIterator::Subdirectories);
	while (it.hasNext()) {
		QString srcFilePath = it.next();
		QString relativePath = srcDir.relativeFilePath(srcFilePath);
		QString dstFilePath = destPath + "/" + relativePath;

		QDir dstDir = QFileInfo(dstFilePath).dir();
		if (!dstDir.exists())
			QDir().mkpath(dstDir.absolutePath());

		if (!QFile::copy(srcFilePath, dstFilePath)) {
			qDebug() << "Failed to copy:" << srcFilePath << "→" << dstFilePath;
		}
		else {
			qDebug() << "Copied:" << srcFilePath << "→" << dstFilePath;
		}
	}
}

bool VisionApp::copyFileToFolder(const QString& sourceFilePath, const QString& destFolderPath, bool overwrite)
{
	return true;
	QFileInfo srcInfo(sourceFilePath);
	if (!srcInfo.exists() || !srcInfo.isFile()) {
	
		return false;
	}

	QDir destDir(destFolderPath);
	if (!destDir.exists()) {
		QDir().mkpath(destFolderPath);
	}

	QString destFilePath = destDir.filePath(srcInfo.fileName());

	if (overwrite && QFile::exists(destFilePath)) {
		QFile::remove(destFilePath);
	}

	bool success = QFile::copy(sourceFilePath, destFilePath);
	if (!success) {
		
	}
	else {
		
	}
	return success;
}

void VisionApp::loadImagesForOfflineInspection(QHash <QString, QLineScan> linescan, QHash<QString, OpticsInfo3D> optic3D)
{
	if (_enable2D) {
		if (SystemData::instance()._enable_multi_thread) load_2dOffline_image_parallel();
		else load_2dOffline_image();
	}

	//not yet add support for singleView
	//X3D: Add for heightmap
	if (_enable3D) {
		if (SystemData::instance()._enable_multi_thread) load_3dOffline_image_parallel(linescan, optic3D);
		else load_3dOffline_image();
	}
}

void VisionApp::GenerateVidiWorkspaceInfo()
{
	//BYPASS VIDI
	SetEvent(_appEvents.getEvent(std::string("GenerateVidiWorkspaceInfo")));

	WaitForSingleObject(_appEvents.getEvent(std::string("VidiNodeReturn")), INFINITE);
	//BYPASS VIDI
}

void VisionApp::openVidiWorkSpace()
{
	//BYPASS VIDI
	bool ok;
	QDir dir(Common::Directory::WorkspacePath());
	QStringList filters;
	filters << "*.vrws";
	QStringList workspaces = dir.entryList(filters, QDir::NoDotAndDotDot | QDir::AllEntries);
	workspaces.append("None");
	QString workspaceName = QInputDialog::getItem(this, tr("Open Workspace"), tr("Workspace:"), workspaces, 0, false, &ok, Qt::CoverWindow);

	if (ok && !Common::Directory::CurrentRecipe.isEmpty() && !workspaceName.isEmpty() && workspaceName != "None")
	{
		createStreamMapping(Common::Directory::CurrentRecipe, workspaceName);
		_hasWorkspace = openWorkspace(workspaceName);
		GenerateVidiWorkspaceInfo();
	}
	else if (workspaceName == "None")
	{
		createStreamMapping(Common::Directory::CurrentRecipe, "");
		closeVidiWorkSpace();
	}
	//BYPASS VIDI
}

void VisionApp::closeVidiWorkSpace()
{
	//BYPASS VIDI
	//BYPASS VIDI
}

void VisionApp::resetResult()
{
	_resultModel.clear();

	QStringList header;
	header.append(QStringLiteral(""));
	_resultModel.setHorizontalHeaderLabels(header);

	QStandardItem *parentItem = _resultModel.invisibleRootItem();

	_pUninspectItem = new QStandardItem(QStringLiteral("Uninspect"));
	_pUninspectItem->setEditable(false);
	parentItem->appendRow(_pUninspectItem);

	_pSkipItem = new QStandardItem(QStringLiteral("Skip"));
	_pSkipItem->setEditable(false);
	parentItem->appendRow(_pSkipItem);

	_pPassItem = new QStandardItem(QStringLiteral("Pass"));
	_pPassItem->setEditable(false);
	parentItem->appendRow(_pPassItem);

	_pFailItem = new QStandardItem(QStringLiteral("Fail"));
	_pFailItem->setEditable(false);
	parentItem->appendRow(_pFailItem);
}

bool VisionApp::createResultFolder(QString& path)
{
	bool flag = false;
	QDir dir;
	QString folderName = QDateTime::currentDateTime().toString().replace(":", "-");

	path.clear();

	dir.cd(QStringLiteral("%1recipe//%2//result//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe));
	if (dir.mkdir(folderName) == true)
	{
		path = QStringLiteral("%1recipe//%2//result//%3//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe).arg(folderName);
		flag = true;
	}

	return flag;
}

void VisionApp::closeApp()
{
	//saveRecipe();
	this->close();
}

void VisionApp::showImageView()
{
	ui.stackedWidgetViewSelection->setCurrentIndex(0);
	showStatus(QStringLiteral("Show image view"));
}

void VisionApp::showChartView()
{
	ui.stackedWidgetViewSelection->setCurrentIndex(1);
	showStatus(QStringLiteral("Show chart view"));
}

int VisionApp::numBoxSelected()
{
	int count = 0;
	for (int i = 0; i < _dragROI.count(); i++)
	{
		if (_dragROI.at(i)->isSelected() == true)
		{
			count++;
		}
	}

	return count;
}

QVector<VisionAppQDragBox*> VisionApp::getSelectedVisionObject()
{
	QVector<VisionAppQDragBox*> selected;

	for (int i = 0; i < _dragROI.count(); i++)
	{
		if (_dragROI.at(i)->isSelected() == true)
		{
			selected.append(_dragROI.at(i));
		}
	}
	return selected;
}

QVector<QString> VisionApp::getSelectedViewIDs()
{
	QVector<QString> selected;

	if (g_viewMode == (int)ViewMode::SINGLE) {
		auto id = ui.label_curViewName->whatsThis();
		selected.append(id);
		return selected;
	}

	for (const auto& view : _views) {
		ct::logger::debug("View: %s, %x", view.id.toStdString().c_str(), view.pDragBox);
		if (view.pDragBox == nullptr) continue;

		if (view.pDragBox->isSelected()) {
			selected.append(view.id);
		}
	}

	return selected;
}

void VisionApp::selectAll()
{
}

void VisionApp::initGifIcon()
{
	qDebug() << "initGifIcon";
	_movieMainUi = new QMovie(":/VisionApp/Icon/icons8-robot.gif");
	connect(_movieMainUi, &QMovie::frameChanged, this, [=](int frame) {
		ui.toolButtonWorkingMode->setIcon(QIcon(_movieMainUi->currentPixmap()));
	});

	// if movie doesn't loop forever, force it to.
	if (_movieMainUi->loopCount() != -1) connect(_movieMainUi, SIGNAL(finished()), _movieMainUi, SLOT(start()));

	if (_movieMainUi->state() == QMovie::NotRunning)
	{
		_movieMainUi->start();
	}

	_movieProductionMode = new QMovie(":/VisionApp/Icon/icon8/production-ezgif.com-gif-maker.gif");
	connect(_movieProductionMode, &QMovie::frameChanged, this, [=](int frame) {
		ui.toolButton_ProductionMode->setIcon(QIcon(_movieProductionMode->currentPixmap()));
		});

	// if movie doesn't loop forever, force it to.
	if (_movieProductionMode->loopCount() != -1) connect(_movieProductionMode, SIGNAL(finished()), _movieProductionMode, SLOT(start()));

	if (_movieProductionMode->state() == QMovie::NotRunning)
	{
		_movieProductionMode->start();
	}



	ui.toolButton_synthiaBrain->hide();
	//_movieBrain = new QMovie(":/VisionApp/Icon/ai.gif");
	//connect(_movieBrain, &QMovie::frameChanged, this, [=](int frame) {
	//	ui.toolButton_synthiaBrain->setIcon(QIcon(_movieBrain->currentPixmap()));
	//});

	//// if movie doesn't loop forever, force it to.
	//if (_movieBrain->loopCount() != -1) connect(_movieBrain, SIGNAL(finished()), _movieBrain, SLOT(start()));

	//if (_movieBrain->state() == QMovie::NotRunning)
	//{
	//	_movieBrain->start();
	//}


	// if want stop
	/*if (_movieOnlineGlobe->state() == QMovie::Running)
	{
		_movieOnlineGlobe->stop();
	}*/
	
}

void VisionApp::loginMode()
{
	if (!_curUserAccInfo.userName.isEmpty())
		AuditLog::instance().log(QStringLiteral("LOGOUT"), _curUserAccInfo.userName);
	AuditLog::instance().clearUser();
	_curUserAccInfo = AccountInfo();

	initWidget(); // make login page full size
	ui.lineEditPassword->clear();
	ui.labelLoginStatus->clear();
	ui.stackedWidgetViewSelection->setCurrentIndex(2);
}

void VisionApp::setUserEnvironment(AccessLevel accessLevel)
{
	//temp
	ui.comboBox_cameraSelection->hide();
	ui.comboBox_ImageOptics->hide();
	ui.label_yield->hide();
	ui.label_UPH->hide();
	ui.lineEdit_yield->hide();
	ui.lineEdit_UPH->hide();
	ui.label_status->hide();
	ui.toolButtonDatasetPage->hide();

	bool mainUIVisibility = true;
	if (accessLevel == AccessLevel::ADMIN)
	{
		mainUIVisibility = true;
	}
	else if (accessLevel == AccessLevel::ENGINEER)
	{
		mainUIVisibility = true;
	}
	else if (accessLevel == AccessLevel::OPERATOR)
	{
		mainUIVisibility = false;
	}

	// initialize
	clearAllDrawings();
	showInfo();
	_blockEventFilter = !mainUIVisibility;


	ui.toolButtonSelectMode->setVisible(mainUIVisibility);
	ui.toolButtonDrawVisionObjMode->setVisible(mainUIVisibility);

	//handle adaptive resolution
	if (g_viewMode == (int)ViewMode::PLANE) {
		ui.frame_leftTab->hide();
		ui.toolButton_ImageViewer->hide();
	}
	else {
		ui.frame_leftTab->setVisible(mainUIVisibility);
	}

	ui.frame_rightTab->setVisible(mainUIVisibility);
	ui.toolButtonRecipeSettings->setVisible(mainUIVisibility);
	ui.toolButtonSystemSettings->setVisible(mainUIVisibility);
	ui.toolButtonOpenImage->setVisible(mainUIVisibility);
	ui.toolButtonRecipeSettings->setVisible(mainUIVisibility);

	// top tool bar
	ui.toolButton_toggleMotionControl->setVisible(mainUIVisibility);
	ui.toolButton_stopRun->setVisible(mainUIVisibility);
	ui.toolButton_runOffline->setVisible(mainUIVisibility);
	ui.toolButtonLiveMode->setVisible(mainUIVisibility);
	ui.toolButtonSnap->setVisible(mainUIVisibility);
	ui.toolButtonSaveImage->setVisible(mainUIVisibility);
	ui.toolButtonSelectMode->setVisible(mainUIVisibility);
	ui.toolButtonDrawVisionObjMode->setVisible(mainUIVisibility);
	ui.toolButton_showCrossHair->setVisible(mainUIVisibility);
	ui.toolButton_showView->setVisible(mainUIVisibility);
	ui.toolButton_showVisionObject->setVisible(mainUIVisibility);
	ui.toolButton_showLineScan->setVisible(mainUIVisibility);
	ui.toolButton_showPath->setVisible(mainUIVisibility);

	ui.toolButton_toggleDualView->setVisible(mainUIVisibility);
	ui.toolButton_toggleWorldView->setVisible(mainUIVisibility);
	ui.toolButton_toggleFovView->setVisible(mainUIVisibility);

	if (!mainUIVisibility) toggleFOVView();

	ui.toolButtonWorkingMode->setVisible(mainUIVisibility);
	
	// AccessLevel::OPERATOR
	if (!mainUIVisibility) {
		ui.toolButton_ProductionMode->animateClick();
		ui.frame_hardwareStatus->setVisible(false);
	}
	setUIVisibilityMotionControl(accessLevel);


	qDebug() << "SetuserEnvironment End";

	setUIVisibility();
}

void VisionApp::setUIVisibility()
{
	//temp
	ui.listWidget_viewSelection->hide();
	ui.tb_includeView->hide();
	ui.tb_excludeView->hide();
	//ui.label_status->hide();

	//moved to system level
	ui.label_40->hide();
	ui.lineEdit_scalingStep->hide();
	ui.toolButton_cameraAlignment->hide();


	//ui.toolButton_teachPoint->hide();
	//ui.toolButton_jogToTeachPoint->hide();
	//ui.toolButton_setBackRight->hide();
	//ui.toolButton_setFrontLeft->hide();
	//ui.toolButton_setOpticAsMain->hide();
	ui.toolButton_generateSetupRegion->hide();

	//exposure and gain: per-optic camera settings, editable on the optics page

	//hide:result viewer
	//ui.label_78->hide();
	//ui.toolButton_verificationCenterStatus->hide();
	//ui.label_verificationLoc->hide();

	//hide: world view
	//ui.toolButton_toggleDualView->hide();
	//ui.toolButton_toggleMotionControl->hide();

	//hide: Rect toggles
	//ui.toolButton_showView->hide();
	//ui.toolButton_showLineScan->hide();
	//ui.toolButton_showPath->hide();

	ui.toolButton_circleCrosshair->hide();

	//hide: Learn color reference
	ui.toolButton_learnColorSegmentReference->hide();
	ui.toolButton_7->hide();
}

void VisionApp::setUIVisibilityMotionControl(AccessLevel accessLevel)
{
	bool mainUIVisibility = (accessLevel == AccessLevel::ADMIN); // AccessLevel::ENGINEER, AccessLevel::OPERATOR restricted

	//tower lights, buzzer and z brake release rows are admin only
	const std::vector<int> restrictedDOs = { 3, 4, 5, 6, 8 };
	for (auto i : restrictedDOs) {
		auto wire = findChild<QLabel*>(QString("label_EMXA_DO%1_w").arg(i));
		if (wire) wire->setVisible(mainUIVisibility);

		auto addr = findChild<QLabel*>(QString("label_EMXA_DO%1_a").arg(i));
		if (addr) addr->setVisible(mainUIVisibility);

		auto desc = findChild<QLabel*>(QString("label_EMXA_DO%1_d").arg(i));
		if (desc) desc->setVisible(mainUIVisibility);

		auto button = findChild<QToolButton*>(QString("toolButton_EMXA_DO%1").arg(i));
		if (button) button->setVisible(mainUIVisibility);
	}

	ui.toolButton_servo_x->setEnabled(mainUIVisibility);
	ui.toolButton_servo_y->setEnabled(mainUIVisibility);
	ui.toolButton_servo_z->setEnabled(mainUIVisibility);


	ui.lineEdit_y_acceleration->setReadOnly(!mainUIVisibility);
	ui.lineEdit_y_velocity->setReadOnly(!mainUIVisibility);
	ui.toolButton_updateVelocity_y->setEnabled(mainUIVisibility);

	ui.lineEdit_z_acceleration->setReadOnly(!mainUIVisibility);
	ui.lineEdit_z_velocity->setReadOnly(!mainUIVisibility);
	ui.toolButton_updateVelocity_z->setEnabled(mainUIVisibility);

	ui.toolButton_updateVelocity_all->setEnabled(mainUIVisibility);


	ui.lineEdit_x_acceleration->setReadOnly(!mainUIVisibility);
	ui.lineEdit_x_velocity->setReadOnly(!mainUIVisibility);
	ui.lineEdit_x_velocity3d->setReadOnly(!mainUIVisibility);
	ui.toolButton_updateVelocity_x->setEnabled(mainUIVisibility);

}

void VisionApp::workingMode()
{
	_workingMode = _workingMode == WorkingMode::Offline ? WorkingMode::Online : WorkingMode::Offline;

	showInfo();

	if (_workingMode == WorkingMode::Online)
	{
		if (!Common::Directory::CurrentRecipe.isEmpty())
		{
			//disconnect(ui.graphicsViewMain, SIGNAL(rubberBandChanged(QRect, QPointF, QPointF)), this, SLOT(processROIOption(QRect, QPointF, QPointF)));

			for (auto x : _dragROI)
			{
				x->setVisible(false);
			}

			clearAllDrawings();
			//ui.menuBar->setEnabled(false);
			//ui.groupBox_recipe->hide();
			//ui.groupBox_output->show();
			ui.toolButtonSelectMode->hide();
			ui.toolButtonDrawVisionObjMode->hide();
			_blockEventFilter = true;
			if (!ui.frame_rightTab->isHidden()) showRightTab(ui.stackedWidget->currentIndex(), "");
			ui.toolButtonRecipeSettings->hide();
			ui.toolButtonSystemSettings->hide();
			ui.toolButtonOpenImage->hide();
			ui.stackedWidgetViewSelection->setCurrentIndex(0);
			ui.toolButtonWorkingMode->setIcon(QIcon(":/images/Icon/icons-online.png"));
			statusBar()->setStyleSheet("background: rgb(0, 255, 127); color: black");
			showStatus(QStringLiteral("Online mode"));
		}
		else
		{
			showMsg(QStringLiteral("Open a recipe to continue"));
			_workingMode = WorkingMode::Offline;
		}
	}
	else
	{
		//connect(ui.graphicsViewMain, SIGNAL(rubberBandChanged(QRect, QPointF, QPointF)), this, SLOT(processROIOption(QRect, QPointF, QPointF)));

		for (auto x : _dragROI)
		{
			x->setVisible(true);
		}

		ui.lineEditPassword->setText("");
		ui.labelLoginStatus->setText("");
		ui.tabWidgetOutput->setCurrentIndex(0);
		ui.stackedWidgetViewSelection->setCurrentIndex(2);
		ui.toolButtonWorkingMode->setIcon(QIcon(":/images/Icon/icons-offline.png"));
		statusBar()->setStyleSheet("background: rgb(247, 107, 28); color: white");
		showStatus(QStringLiteral("Offline mode"));
	}
}

void VisionApp::triggerCamera()
{
	CAMManager::instance().frame(_camID)->opticID = _mainOptics[_camID].id;
	CAMManager::instance().frame(_camID)->type = _mainOptics[_camID].type;
	CAMManager::instance().frame(_camID)->postTask.combineRGB = false;
	CAMManager::instance().frame(_camID)->postTask.rotationalAngle = SystemData::instance()._camAngles[_camID];
	CAMManager::instance().softTrigger(_camID);
}

void VisionApp::toggleLiveView()
{
	if (_grabber == nullptr) {
		startLiveView();
	}
	else {
		stopLiveView();
	}
}

void VisionApp::startLiveView()
{
	if (_grabber != nullptr) {
		printf("Live view already running\n");
		return;
	}

	toggleFOVView();

	auto bandType = BandType::M;
	if (CAMManager::instance().getChannel(_camID) == 1 && _mainOptics[_camID].type == ct::s_color) {
		auto bandType = BandType::B; //Mono cam but RGB light special abit. Only can choose one band though.
		//User have to self open the lights they want
	}

	OpticsControl::instance().setBand(_camID, _mainOptics[_camID], bandType);

	_grabber = new QImageGrabber();
	connect(_grabber, SIGNAL(grab()), this, SLOT(triggerCamera()));
	startThread(_grabber);

	connect(_grabber, &QObject::destroyed, this, [&]() { _grabber = nullptr; });

	ui.comboBox_cameraSelection->setEnabled(false);
	ct::logger::info("Start live view");

	QSignalBlocker block(ui.toolButtonLiveMode);
	ui.toolButtonLiveMode->setChecked(true);
	processEvents();
}

void VisionApp::stopLiveView()
{
	_processType = ProcessType::NONE;

	if (_grabber == nullptr) {
		ct::logger::info("Live view already stopped");
		return;
	}

	if (_grabber) {
		_grabber->stop();
		OpticsControl::instance().toggleAllChannels(false);
		ui.comboBox_cameraSelection->setEnabled(true);
		ct::logger::info("Stop live view");
	}

	QSignalBlocker block(ui.toolButtonLiveMode);
	ui.toolButtonLiveMode->setChecked(false);
	processEvents();
}

void VisionApp::toggleCircleCrosshair()
{
	QSignalBlocker sb(ui.toolButton_circleCrosshair);

	if (!ui.toolButton_circleCrosshair->isChecked()) {
		_processType = ProcessType::NONE;
		ui.toolButton_circleCrosshair->setChecked(false);
		stopLiveView();
	}
	else {
		_processType = ProcessType::LIVE_INSPECT;
		ui.toolButton_circleCrosshair->setChecked(true);
		startLiveView();
	}
}

bool VisionApp::verifyLogin()
{
	QString nameEnter = ui.lineEditUserName->text();
	QString passwordEnter = ui.lineEditPassword->text();


	AccountInfo a;
	bool accountExist = false;

	// Emergency recovery admin (independent of user.json, always available)
	if (nameEnter == EMERGENCY_ADMIN_USER && passwordEnter == EMERGENCY_ADMIN_PASS)
	{
		a.userName = nameEnter;
		a.password = passwordEnter;
		a.accessLevel = AccessLevel::ADMIN;
		accountExist = true;
	}
	else
	{
		// Look the user up in user.json (compare salted password hashes)
		for (const AccountInfo& acc : loadUserAccounts())
		{
			if (acc.userName == nameEnter && hashPassword(acc.salt, passwordEnter) == acc.pwHash)
			{
				a = acc;
				accountExist = true;
				break;
			}
		}
	}

	if (accountExist)
	{
		_curUserAccInfo = a;

		// Attribute all subsequent audited actions to this user, and record the login.
		AuditLog::instance().setUser(_curUserAccInfo.userName, _curUserAccInfo.accessLevel);
		AuditLog::instance().log(QStringLiteral("LOGIN"),
			nameEnter == EMERGENCY_ADMIN_USER ? QStringLiteral("recovery-admin") : QString());

		ui.lineEditPassword->setText("");


		ui.toolButtonSelectMode->show();
		ui.toolButtonDrawVisionObjMode->show();
		ui.frame_top->show();
		ui.statusBar->show();

		ui.toolButtonRecipeSettings->show();
		ui.toolButtonSystemSettings->show();
		ui.toolButtonOpenImage->show();
		if (ui.frame_leftMenuBar->width() == 0) toggleMenu();
		toPage(UIPage::RECIPE);
		ui.toolButton_ImageViewer->animateClick();
		_blockEventFilter = false;

		ui.toolButtonWorkingMode->setEnabled(true);
		ui.tabWidgetOutput->setCurrentIndex(0);
		ui.stackedWidgetViewSelection->setCurrentIndex(0);

		ui.graphicsViewMain->fitInView(_pPixmapItemMain, Qt::KeepAspectRatio);
		ui.graphicsViewMain->centerOn(_pPixmapItemMain);
		ui.graphicsViewMain->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
		ui.graphicsViewMain->show();

		showStatus(QStringLiteral("%1 login").arg(_curUserAccInfo.userName));
		qDebug() << "Access Level: " << _curUserAccInfo.accessLevel;

		QString accessLevelString;
		if (_curUserAccInfo.accessLevel == AccessLevel::ADMIN) accessLevelString = "Admin";
		else if (_curUserAccInfo.accessLevel == AccessLevel::ENGINEER) accessLevelString = "Engineer";
		else if (_curUserAccInfo.accessLevel == AccessLevel::OPERATOR) accessLevelString = "Operator";
		QString userInfoMessage = "Welcome back: <b>" + _curUserAccInfo.userName +"</b>!   Access Level: <b>" + accessLevelString + "</b>";
		//ui.label_status->setText(userInfoMessage);

		setUserEnvironment(_curUserAccInfo.accessLevel);

		initMachine();

		QString userNameInfo = "User Name: " + _curUserAccInfo.userName;
		QString userAccessLevelInfo = "    Access Level:" + accessLevelString;
		ui.label_username->setText(userNameInfo);
		ui.label_accessLevel->setText(userAccessLevelInfo);
	}
	else
	{
		AuditLog::instance().log(QStringLiteral("LOGIN"), nameEnter, QStringLiteral("FAILED"));
		ui.labelLoginStatus->setText(QStringLiteral("Wrong user/ password"));
	}

	return true;
}

void VisionApp::initMachine()
{
	if (!SystemData::instance()._homeOnStartup) {
		ct::logger::info("[Machine] Home on startup disabled, skipping auto home");
		return;
	}

	//NOTE: driver power is not software controlled on this machine, no need to turn on before homing
	emit homeAll();
}

void VisionApp::escapeKeyPressed()
{
}

void VisionApp::createPieChart()
{
	QPieSeries *series = new QPieSeries();
	series->append("Jane", 1);
	series->append("Advisys", 2);
	series->append("Andy", 3);
	series->append("Barbara", 4);
	series->append("Axel", 5);

	QPieSlice *slice = series->slices().at(1);
	slice->setExploded();
	slice->setLabelVisible();
	slice->setPen(QPen(Qt::darkGreen, 2));
	slice->setBrush(QColor(0, 255, 127));

	QChart *chart = new QChart();
	chart->addSeries(series);
	chart->legend()->hide();

	ui.chartViewMain->setChart(chart);
	ui.chartViewMain->chart()->setTheme(QChart::ChartThemeDark);
	ui.chartViewMain->setRenderHint(QPainter::Antialiasing);
}

QString VisionApp::createTemplateImagesDirectory()
{
	QString path = QStringLiteral("%1recipe/%2/template_Images").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);
	CreateDirectoryA(path.toStdString().c_str(), NULL);

	return path;
}

void VisionApp::setupProductionDir()
{
	//production folder: <recipe>[@]<uid>
	_productionID = Common::Directory::CurrentRecipe + "[@]" + _currentProductionID;
	Common::Directory::setupProductionDir(_productionID);

	qDebug() << "[@@@@] _productionID: " << _productionID;
	qDebug() << "[@@@@] ProductionPath: " << Common::Directory::ProductionPath();
	qDebug() << "[@@@@] getProductionResultPath: " << Common::Directory::getProductionResultPath();


	SystemData::instance()._workingPath = Common::Directory::getProductionImageSetPath();
	_jobThread.setRootPath(Common::Directory::getProductionImageSetPath());

	saveProductionBarcodeInfo();
	saveProductionInfoJson();
}

void VisionApp::saveProductionBarcodeInfo()
{
	auto jsonPath = Common::Directory::getProductionResultPath() + "BarcodeInfo.json";

	QJsonObject j_root;
	j_root.insert(QStringLiteral("1"), ui.lineEdit_barcodeID->text());
	j_root.insert(QStringLiteral("2"), ui.lineEdit_barcodeID2->text());

	auto ret = saveJson(jsonPath, QJsonDocument(j_root));
	if (ret) showStatus(QStringLiteral("Successfully saved barcodeID!"));
	else showStatus(QStringLiteral("Failed to save barcodeID!"));
}

void VisionApp::saveProductionInfoJson()
{
	auto jsonPath = Common::Directory::getProductionImageSetPath() + "info.json";

	QJsonObject j_root;
	j_root.insert(QStringLiteral("barcode_id"), SystemData::instance()._currentBarcode.c_str());
	j_root.insert(QStringLiteral("fiducial_angle"), _fiducial.getAngle());
	j_root.insert(QStringLiteral("fiducial_offset_x"), _fiducial.getOffset().x());
	j_root.insert(QStringLiteral("fiducial_offset_y"), _fiducial.getOffset().y());

	QJsonArray fidArray;
	
	for (auto fDetail: _inspStatus.fiducialHash)
	{
		int index = fDetail.index;
		bool status = fDetail.isPass;
		QRectF fRect = fDetail.fiducialRect;
		double score = fDetail.fiducialScore;
		QString type = fDetail.type;

		QJsonObject fObject;
		fObject.insert("Index", index);
		fObject.insert("Status", status);
		fObject.insert("Score", score);
		fObject.insert("X", fRect.x());
		fObject.insert("Y", fRect.y());
		fObject.insert("W", fRect.width());
		fObject.insert("H", fRect.height());
		fObject.insert("Message", fDetail.fiducialStatus);
		fObject.insert("Type", fDetail.type);
		fidArray.append(fObject);
	}
	j_root.insert(QStringLiteral("fiducial_status"), fidArray);

	auto ret = saveJson(jsonPath, QJsonDocument(j_root));

	if (ret) showStatus(QStringLiteral("Successfully saved production info!"));
	else showStatus(QStringLiteral("Failed to save production info!"));
}

bool VisionApp::loadProductionInfoJson()
{
	qDebug() << "loadProductionInfoJson";
	QJsonObject root;
	QString infoJsonPath = Common::Directory::CurrentImageSetPath + "info.json";
	qDebug() << "production info path:" << infoJsonPath;
	if (loadJson(infoJsonPath, root)) {

		if (!root.contains("barcode_id"))
		{
			return false;
		}
		SystemData::instance()._currentBarcode = jsonHelper::getString(root, QStringLiteral("barcode_id")).toStdString();
		
	//	QHash<int, bool> fidStatusHash;
	//	QStringList fidStatusList;
		QJsonArray fidArr = root.value("fiducial_status").toArray();
		for (const QJsonValue& f : fidArr)
		{
			
			QJsonObject fObj = f.toObject();

			int index = fObj.value("Index").toInt();
			bool status = fObj.value("Status").toBool();
			double score = fObj.value("Score").toDouble();
			double x = fObj.value("X").toDouble();
			double y = fObj.value("Y").toDouble();
			double w = fObj.value("W").toDouble();
			double h = fObj.value("H").toDouble();
			QString message = fObj.value("Message").toString();
			QString type = fObj.value("Type").toString();

			InspStatus::FiducialDetail fDetail;
			fDetail.index = index;
			fDetail.fiducialRect = QRectF(x,y,w,h);
			fDetail.fiducialScore = score;
			fDetail.fiducialStatus = message;
			fDetail.isPass = status;
			fDetail.type = type;

		/*	fidStatusHash.insert(index, status);
			if(status)fidStatusList.append(QString::number(index) + "_true");
			else fidStatusList.append(QString::number(index) + "_false");*/

			_inspStatus.fiducialHash.insert(index, fDetail);
			
		}
		/*_inspStatus.fiducialVector = fidStatusHash;
		_inspStatus.fiducialStatus = fidStatusList;*/

		qDebug() << "_inspStatus.fiducialStatus.size: " << _inspStatus.fiducialHash.size();
		return true;
	}
	else return false;
}

void VisionApp::saveFiducialResultJson(QString rootPath)
{
	qDebug() << "saveFiducialInfoJson";
	auto jsonPath = rootPath + "/FiducialInfo.json";

	QJsonObject j_root;
	j_root.insert(QStringLiteral("fiducial_angle"), _fiducial.getAngle());
	j_root.insert(QStringLiteral("fiducial_offset_x"), _fiducial.getOffset().x());
	j_root.insert(QStringLiteral("fiducial_offset_y"), _fiducial.getOffset().y());


	auto ret = saveJson(jsonPath, QJsonDocument(j_root));

	if (ret) showStatus(QStringLiteral("Successfully saved fiducial result!"));
	else showStatus(QStringLiteral("Failed to save fiducial result!"));
}

void VisionApp::updateMsgBoxBorder()
{
	// Create a painter path to draw the rounded rectangle
	QPainterPath roundedPath;
	roundedPath.addRoundedRect(QRectF(0, 0, _msg.width(), _msg.height()), 12, 12);

	// Set the painter path as the shape of the window
	_msg.setMask(roundedPath.toFillPolygon().toPolygon());

	_msg.update();

	update();
}

void VisionApp::toggleImageView()
{
	if (ui.graphicsViewMain->transform().m11() == 1.0)
	{
		ui.graphicsViewMain->fitInView(_pPixmapItemMain, Qt::KeepAspectRatio);
		ui.graphicsViewMain->centerOn(_pPixmapItemMain);
		ui.graphicsViewMain->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
		ui.graphicsViewMain->show();
	}
	else
	{
		ui.graphicsViewMain->resetMatrix();
		ui.graphicsViewMain->scale(1, 1);
	}
}

void VisionApp::toggleDrawingAndRois()
{
	if (_workingMode == WorkingMode::Online)
		return;

	bool flag = false;
	for (auto x : _renderedShape)
	{
		if (x->isVisible())
		{
			x->setVisible(false);
		}
		else
		{
			flag = true;
			x->setVisible(true);
		}
	}

	for (auto x : _dragROI)
		x->setVisible(!flag);
}

VisionApp::~VisionApp()
{
	ct::logger::info("~Visionapp");

	MachineController::instance().notifyEvent(MachineEvent::SOFTWARE_OFF);

	//stop live read (LOFF to both readers) and camera live view before teardown
	SRXManager::instance().setLiveRead(SRXManager::SRX1, false);
	SRXManager::instance().setLiveRead(SRXManager::SRX2, false);
	stopLiveView();
	SRXManager::instance().release(); //worker drains the queued LOFFs, then exits

	resetFilterInfo();

	//release hardware
	ProfilerManager::instance().stop(_profilerID);
	ProfilerManager::instance().disconnect(_profilerID);
	setVisionIO(false);
	for (const auto& id : LSCManager::instance().channels()) {
		LSCManager::instance().toggle(id, false);
	}
	LSCManager::instance().disconnect();
	CAMManager::instance().stopGrab(_camID);
	CAMManager::instance().disconnect(_camID);
	if (_motionTimer) _motionTimer->stop();
	_ioCard.closeIOCard();



	//clean up UI
	if (_pGraphicsSceneMain != nullptr) {
		clearVisionObject();
		clearAllDrawings();
		if (_pPixmapItemMain != nullptr)
		{
			_pGraphicsSceneMain->removeItem(_pPixmapItemMain);
		}
	}
	_templateLibraryTab->releaseAlgoTemplates();
	if (_camAlpha) delete[] _camAlpha; _camAlpha = nullptr;
	


	//Release qthreads
	_imageManager.release();
	_jobThread.release();
	InspectionThread::instance().release();

	/*
	* AlgoManager keeps the last scanned height map alive on purpose, so the Algo Setup page
	* can offer "Use Last Scan". It is a singleton and nothing else ever drops that reference,
	* so after any 3D scan exactly one MIL buffer outlives everything above - which is all it
	* takes for MappFreeDefault() below to report
	*   "MsysFree(): System still has buffer(s) associated with it"
	* and then
	*   "MappFree(): Application still has system(s) associated with it. Application was not freed."
	*
	* It has to be released HERE, before release_pools(): MbufPool::release() only frees the ids
	* in m_buffers, and alloc()/attach() add ids there only for POOL_IDLE. An attached
	* FREE_BUFFER buffer - which is exactly what the rotated height map is - is invisible to it,
	* so the pool teardown cannot clean up after this reference no matter when it runs.
	*/
	AlgoManager::instance().setHeightMap(nullptr);

	mtrx::MPM::instance().release_pools();
	_databaseThread.terminate(); 
	_networkPathChecker.terminate(); 

	//Release algo
	udpateRecipeVersion(Common::Directory::getRecipeCurrentPath());
	vips_shutdown();
	_algo.release(); //buffers + MIL application/host system

	//engage the Z brake (brake release OFF) before the motion controller
	//disconnects, so the Z axis never drops after servo power is lost
	MachineController::instance().turnOnBrake();

	MachineController::instance().release();
	MotionController::instance().release(_motionID);

	ct::logger::info("Software safely closed...");

	ct::logger::release();
}

bool VisionApp::eventFilter(QObject * obj, QEvent * event)
{

	if (event->type() == QEvent::MouseButtonPress) {
		QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
		// Get the mouse cursor position
		QPoint pos = mouseEvent->globalPos();

		auto x = _recipeSettingsMenu->pos().x();
		auto y = _recipeSettingsMenu->pos().y();
		auto x1 = _recipeSettingsMenu->pos().x() + _recipeSettingsMenu->width();
		auto y1 = _recipeSettingsMenu->pos().y() + _recipeSettingsMenu->height();

		if (!(pos.x() > x && pos.x() < x1 && pos.y() > y && pos.y() < y1))
		{
			toggleRecipeSettingsMenu(false);
		}

		auto sx = _recipeSettingsMenu->pos().x();
		auto sy = _recipeSettingsMenu->pos().y();
		auto sx1 = _recipeSettingsMenu->pos().x() + _recipeSettingsMenu->width();
		auto sy1 = _recipeSettingsMenu->pos().y() + _recipeSettingsMenu->height();

		if (!(pos.x() > sx && pos.x() < sx1 && pos.y() > sy && pos.y() < sy1))
		{
			toggleSystemSettingsMenu(false);
		}

		_worldFOV.hide();
	}

	if (event->type() == QEvent::MouseMove) {
		if (!_blockEventFilter)
		{
			QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
			// Get the mouse cursor position
			QPoint pos = mouseEvent->globalPos();
			QPoint referencePos = QPoint(geometry().x(), geometry().y());
			pos = pos - referencePos;
			rightMenuMouseMoveEvent(pos, false);

			if (_editMode == EditMode::RULER && mouseEvent->buttons() == Qt::LeftButton) {
				if (_ruler) {
					_pGraphicsSceneMain->removeItem(_ruler);
					delete _ruler;
					_ruler = nullptr;
				}
				_ruler = drawLine(QLineF(_lastMousePressPos, _scenePos));
			}
		}
	}

	if (event->type() == QEvent::MouseButtonRelease) {
		if (!_blockEventFilter)
		{

			processUtilityInfo();

			if (_editMode == EditMode::RULER) {
				_editMode = EditMode::SELECT;
				_cursor.setShape(Qt::ArrowCursor);
				ui.graphicsViewMain->setCursor(_cursor);
				ui.graphicsViewMain->setDragMode(QGraphicsView::RubberBandDrag);

				//verify 
				MIL_ID MilMarker = MmeasAllocMarker(M_DEFAULT, M_STRIPE, M_DEFAULT, M_NULL);
				MIL_ID MilImage = mtrx::to_milID(_imageWorld);
				auto mMono = mtrx::to_mono(MilImage);

				double w = _ruler->width(), h = _ruler->height();
				double cx = _ruler->x() + w / 2, cy = _ruler->y() + h / 2;

				ct::logger::trace("Center: %f, %f", cx, cy);
				ct::logger::trace("Size: %f, %f", w, h);

				MmeasSetMarker(MilMarker, M_POLARITY, M_ANY, M_DEFAULT);
				MmeasSetMarker(MilMarker, M_BOX_CENTER, cx, cy);
				MmeasSetMarker(MilMarker, M_BOX_SIZE, w, h);
				MmeasSetMarker(MilMarker, M_BOX_ANGLE, 0.0, M_NULL);
				MmeasSetMarker(MilMarker, M_SUB_REGIONS_NUMBER, M_DEFAULT, M_NULL);
				MmeasSetMarker(MilMarker, M_NUMBER, M_DEFAULT, M_NULL);
				MmeasSetMarker(MilMarker, M_EDGEVALUE_MIN, M_DEFAULT, M_NULL);

				// Remove strength score function.
				MmeasSetScore(MilMarker, M_STRENGTH_SCORE, 0, 0, M_MAX_POSSIBLE_VALUE, M_MAX_POSSIBLE_VALUE, M_DEFAULT, M_DEFAULT, M_DEFAULT);

				// Set the width score function to find the widest stripe.
				MmeasSetScore(MilMarker, M_STRIPE_WIDTH_SCORE, 0, M_MAX_POSSIBLE_VALUE, M_MAX_POSSIBLE_VALUE, M_MAX_POSSIBLE_VALUE, M_DEFAULT, M_DEFAULT, M_DEFAULT);

				// Measure and print results.
				MmeasFindMarker(M_DEFAULT, mMono, MilMarker, M_DEFAULT);

				MIL_DOUBLE num;
				MmeasGetResult(MilMarker, M_NUMBER, &num, M_NULL);

				MIL_DOUBLE* angle = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge1_x = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge1_y = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge2_x = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge2_y = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge_width = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge_str = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge_contrast = new MIL_DOUBLE[num];
				MIL_DOUBLE* length = new MIL_DOUBLE[num];
				MIL_DOUBLE* score = new MIL_DOUBLE[num];
				MIL_DOUBLE* stripe_width = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge_start_x = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge_start_y = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge_end_x = new MIL_DOUBLE[num];
				MIL_DOUBLE* edge_end_y = new MIL_DOUBLE[num];

				MmeasGetResult(MilMarker, M_ANGLE, angle, M_NULL);
				MmeasGetResult(MilMarker, M_POSITION + M_EDGE_FIRST, edge1_x, edge1_y);
				MmeasGetResult(MilMarker, M_POSITION + M_EDGE_SECOND, edge2_x, edge2_y);
				MmeasGetResult(MilMarker, M_EDGE_WIDTH, edge_width, M_NULL);
				MmeasGetResult(MilMarker, M_EDGE_STRENGTH, edge_str, M_NULL);
				MmeasGetResult(MilMarker, M_EDGE_CONTRAST, edge_contrast, M_NULL);
				MmeasGetResult(MilMarker, M_EDGE_START, edge_start_x, edge_start_y);
				MmeasGetResult(MilMarker, M_EDGE_END, edge_end_x, edge_end_y);
				MmeasGetResult(MilMarker, M_LENGTH, length, M_NULL);
				MmeasGetResult(MilMarker, M_SCORE, score, M_NULL);
				MmeasGetResult(MilMarker, M_STRIPE_WIDTH, stripe_width, M_NULL);

				for (int i = 0; i < num; i++) {
					ct::logger::trace("Angle: %f", angle[i]);
					ct::logger::trace("Edge Width: %f", edge_width[i]);
					ct::logger::trace("Strength: %f", edge_str[i]);
					ct::logger::trace("Contrast: %f", edge_contrast[i]);
					ct::logger::trace("Length: %f", length[i]);
					ct::logger::trace("Score: %f", score[i]);
					ct::logger::trace("Stripe Width: %f", stripe_width[i]);
					ct::logger::trace("Edge1: %f, %f", edge1_x[i], edge1_y[i]);
					ct::logger::trace("Edge2: %f, %f", edge2_x[i], edge2_y[i]);
					ct::logger::trace("Start Edge: %f, %f", edge_start_x[i], edge_start_y[i]);
					ct::logger::trace("End Edge: %f, %f", edge_end_x[i], edge_end_y[i]);
				}

				delete[] angle;
				delete[] edge1_x;
				delete[] edge1_y;
				delete[] edge2_x;
				delete[] edge2_y;
				mtrx::free_buffer(mMono);
				mtrx::free_buffer(MilImage);
			}
			else if (_editMode == EditMode::NAVIGATE_TO) {
				_editMode = EditMode::SELECT;
				_cursor.setShape(Qt::ArrowCursor);
				ui.graphicsViewMain->setCursor(_cursor);

				QPoint pos = _scenePos;
				auto wmm = ScaleManager::instance().to_world_mm(pos);
				emit jogSnap(wmm.x(), wmm.y(), SystemData::instance().currentCoordinate().wz, _mainOptics[_camID]);

				saveQImageWithCrossSection(_imageFOV, "test/N_camera.jpg");
			}
		}

		QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
		// Get the mouse cursor position
		QPoint pos = _scenePos;

		if (_pointJog) {
			_pointJog = false;

			auto point = ScaleManager::instance().to_world_mm(QPointF(_scenePos.x(), _scenePos.y()));
			auto shifted = _fiducial.getShiftedPoint(em::V2d(point.x(), point.y()));
			jogTo(shifted.x(), shifted.y(), _recipeZ);
		}
	}

	if (event->type() == QEvent::KeyPress) {
		QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
		if (keyEvent->key() == Qt::Key_Control) {

			if (_editMode == EditMode::PATH_ASSIGNMENT) {
				addViewToPath(_lastSelectedViewHovered);
			}

		}
	}

	// Call the base class implementation to handle other events
	return QMainWindow::eventFilter(obj, event);
}

void VisionApp::rightMenuMouseMoveEvent(QPoint pos, bool blockEventFilter)
{
	if (notAllowToAccess(AccessLevel::OPERATOR)) {
		_blockEventFilter = true;
		return;
	}
	if (_isAutoMode) return;
	
	auto width = this->width();
	auto x = width - 105;
	auto y = ui.frame_topRightTools->y() + 40;

	// set right Menu to right side if rightMenu position is at 0,0
	if (_rightMenu->isHidden())
	{
		_rightMenu->move(x + _rightMenu->minimumWidth(), y);
	}

	// ANIMATION
	QPropertyAnimation *animation = new QPropertyAnimation(_rightMenu, "geometry");
	animation->setDuration(300);
	QRect startValue = _rightMenu->geometry();
	animation->setStartValue(startValue);
	animation->setEasingCurve(QEasingCurve::InOutQuad);


	if (pos.x() > (width - 30) && pos.x() < (width - 5) && pos.y() > (ui.frame_top->pos().y() + ui.frame_top->height()) && _rightMenu->isHidden())
	{
		_blockEventFilter = true;
		_rightMenu->show();
		_rightMenu->activateWindow();
		_rightMenu->raise();
		_rightMenu->setParent(this);
		_rightMenu->setHeight(ui.stackedWidgetViewSelection->height() - 20);
		animation->setEndValue(QRect(x, y, _rightMenu->minimumWidth(), _rightMenu->minimumHeight()));
		animation->start();
		//_rightMenu->move(x, y);
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			_blockEventFilter = false;
		});
	}
	else if (pos.x() < (width - _rightMenu->width()) && !_rightMenu->isHidden())
	{
		_blockEventFilter = true;
		animation->setEndValue(QRect(x + _rightMenu->minimumWidth(), y, _rightMenu->minimumWidth(), _rightMenu->minimumHeight()));
		animation->start();
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			_rightMenu->hide();
			_blockEventFilter = blockEventFilter;
		});
	}
}

void VisionApp::moveWidgetAnimation(QWidget * widget, QRect EndValue, bool enable)
{
	// ANIMATION
	QPropertyAnimation *animation = new QPropertyAnimation(widget, "geometry");
	animation->setDuration(300);
	QRect startValue = widget->geometry();
	animation->setStartValue(startValue);
	animation->setEasingCurve(QEasingCurve::InOutQuad);

	if (enable)
	{
		widget->show();
		widget->activateWindow();
		widget->raise();
		animation->setEndValue(EndValue);
		animation->start();
	}
	else
	{
		animation->setEndValue(EndValue);
		animation->start();
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			widget->hide();
		});
	}
}

void VisionApp::resizeWidgetAnimation(QWidget * widget, int minValue, int maxValue, bool enable, int rightTabIndex, QStackedWidget* stackWidget)
{
	int widthExtended = 0;

	//GET WIDTH
	int maxWidth = maxValue;
	auto width = widget->width();
	int maxExtend = maxWidth;
	int standard = minValue;

	bool show = false;
	// SET MAX WIDTH
	//used less than max width logic to fix bug of sometimes right tab cannot be shown, this is because when the right tab is hidden sometimes before it can be resized to zero,
	//the next show trigger is open, this causes the width to not be zero but 2 or other values
	//therefore less than max width logic is added to prevent treating a size of 2 as a shown width
	//qDebug() << "widget Width:" << width;
	//if (width <= minValue + 10)

	if (width < maxValue - 10)
	{
		widthExtended = maxExtend;
		show = true;
	}
	else
	{
		widthExtended = standard;
	}

	if (show) widget->show();

	// ANIMATION
	QPropertyAnimation *animation = new QPropertyAnimation(widget, "minimumWidth");

	animation->setDuration(300);
	animation->setStartValue(width);
	animation->setEndValue(widthExtended);
	animation->setEasingCurve(QEasingCurve::InOutQuad);
	animation->start();

	connect(animation, &QPropertyAnimation::finished, this, [=]() {
		if (!show)
		{
			widget->hide();
			for (int i = 0; i < _dragROI.size(); i++)
			{
				_dragROI[i]->setFrozen(false);
			}
			if (enable)
			{
				for (int i = 0; i < _dragROI.size(); i++)
				{
					_dragROI[i]->setFrozen(true);
				}
				stackWidget->setCurrentIndex(rightTabIndex);
				resizeWidgetAnimation(widget, minValue, maxValue, false, rightTabIndex, stackWidget);
			}
		}
		else
		{
			for (int i = 0; i < _dragROI.size(); i++)
			{
				_dragROI[i]->setFrozen(false);
			}
		}
	});
}

void VisionApp::fadeWidgetAnimation(QWidget * widget, bool enable)
{
	if (enable)
	{
		QGraphicsOpacityEffect *eff = new QGraphicsOpacityEffect();
		eff->setOpacity(0);
		widget->setGraphicsEffect(eff);
		widget->show();
		QPropertyAnimation *a = new QPropertyAnimation(eff, "opacity");
		a->setDuration(500);
		a->setStartValue(0);
		a->setEndValue(1);
		a->setEasingCurve(QEasingCurve::InBack);
		a->start(QPropertyAnimation::DeleteWhenStopped);
		connect(a, &QPropertyAnimation::finished, this, [=]() {
			widget->setGraphicsEffect(nullptr);
		});
	}
	else
	{
		//fade out
		QGraphicsOpacityEffect *eff = new QGraphicsOpacityEffect(this);
		widget->setGraphicsEffect(eff);
		QPropertyAnimation *a = new QPropertyAnimation(eff, "opacity");
		connect(a, &QPropertyAnimation::finished, this, [=]() {
			widget->hide();
			widget->setGraphicsEffect(nullptr);

		});
		a->setDuration(500);

		a->setStartValue(1);
		a->setEndValue(0);
		a->setEasingCurve(QEasingCurve::OutBack);
		a->start(QPropertyAnimation::DeleteWhenStopped);

	}

}

void VisionApp::testcase_fiducialLogic()
{
	ct::logger::debug("<=======================Test Fiducial=======================>");
	_fiducial.setLearntFid(em::V2d(4, 2), em::V2d(4, 0));
	_fiducial.setShiftedFid(em::V2d(0, 4), em::V2d(0, 2));

	_fiducial.compute();
	auto t1 = _fiducial.getShiftedPoint(em::V2d(6, 2));
	auto t2 = _fiducial.getShiftedPoint(em::V2d(5, 1));
	ct::logger::debug("Expected (2,4) ---> Get (%f, %f)", t1.x(), t1.y());
	ct::logger::debug("Expected (1,3) ---> Get (%f, %f)", t2.x(), t2.y());


	_fiducial.setShiftedFid(em::V2d(3, 6), em::V2d(5, 6));

	_fiducial.compute();
	auto r0 = _fiducial.getShiftedPoint(em::V2d(4, 2));
	auto r1 = _fiducial.getShiftedPoint(em::V2d(6, 2));
	auto r2 = _fiducial.getShiftedPoint(em::V2d(5, 1));
	ct::logger::debug("Expected (3,6) ---> Get (%f, %f)", r0.x(), r0.y());
	ct::logger::debug("Expected (3,8) ---> Get (%f, %f)", r1.x(), r1.y());
	ct::logger::debug("Expected (4,7) ---> Get (%f, %f)", r2.x(), r2.y());
}

void VisionApp::testcase_mbufpool()
{
	showMsg("Test case: Buffer pool being carry out. May affect normal operation");

	//mtrx::MbufPoolManager::instance().create_pool(3000, 3000, 1, 8 + M_UNSIGNED, 5, mtrx::PoolDestructorType::POOL_IDLE);
	//mtrx::MbufPoolManager::instance().create_pool(3000, 3000, 1, 8 + M_UNSIGNED, 5, mtrx::PoolDestructorType::POOL_IDLE);
	//mtrx::MbufPoolManager::instance().create_pool(4000, 4000, 1, 8 + M_UNSIGNED, 5, mtrx::PoolDestructorType::POOL_IDLE);
	//mtrx::MbufPoolManager::instance().create_pool(3000, 3000, 1, 16 + M_UNSIGNED, 5, mtrx::PoolDestructorType::POOL_IDLE);

	std::vector<FrameInfo> imageDatas;

	//used up all available buffers
	ct::logger::info("[Test] Created 15 buffers");
	for (int i = 0; i < 5; i++) {
		FrameInfo data, data2, data3;

		data.pImage = mtrx::MbufPoolManager::instance().acquire(3000, 3000, 1, 8 + M_UNSIGNED);
		data2.pImage = mtrx::MbufPoolManager::instance().acquire(4000, 4000, 1, 8 + M_UNSIGNED);
		data3.pImage = mtrx::MbufPoolManager::instance().acquire(3000, 3000, 1, 16 + M_UNSIGNED);

		imageDatas.push_back(data);
		imageDatas.push_back(data2);
		imageDatas.push_back(data3);
	}

	//request buffer when no idle buffers
	ct::logger::info("[Test] Simulate requesting buffer with no idles");
	FrameInfo data4;
	data4.pImage = mtrx::MbufPoolManager::instance().acquire(3000, 3000, 1, 8 + M_UNSIGNED);

	//request buffer when no available format
	ct::logger::info("[Test] Simulate requesting unavailable format");
	FrameInfo data5;
	data5.pImage = mtrx::MbufPoolManager::instance().acquire(2000, 2000, 1, 8 + M_UNSIGNED);

	//simulate passing a copy to another entity, by right only two ptr will be free
	auto data9 = imageDatas.back();

	//release last 3 pool
	ct::logger::info("[Test] Simulate releaseing 3 buffers to idle");
	imageDatas.erase(imageDatas.end() - 3, imageDatas.end());

	//try to access idle pool
	ct::logger::info("[Test] Simulate accessing the 3 released idle buffers");
	FrameInfo data6, data7, data8;
	data6.pImage = mtrx::MbufPoolManager::instance().acquire(3000, 3000, 1, 8 + M_UNSIGNED);
	data7.pImage = mtrx::MbufPoolManager::instance().acquire(4000, 4000, 1, 8 + M_UNSIGNED);
	data8.pImage = mtrx::MbufPoolManager::instance().acquire(3000, 3000, 1, 16 + M_UNSIGNED);

	ct::logger::info("[Test] Releasing all pools");
	mtrx::MPM::instance().release_pools();
}

void VisionApp::progressBarSetup(QString displayText, int maxValue, bool enableCancel)
{
	_progressValue = 0;
	_progressDialog = new QProgressDialog(displayText, "Cancel", 0, 10000, this);
	_progressDialog->setAttribute(Qt::WA_DeleteOnClose);
	_progressDialog->setParent(ui.centralWidget);
	int screenIndex = QApplication::desktop()->screenNumber(this);
	QRect screenGeometry = QApplication::desktop()->screenGeometry(screenIndex);
	int xScreen = screenGeometry.x() + (screenGeometry.width() - _progressDialog->width()) / 2;
	int yScreen = screenGeometry.y() + (screenGeometry.height() - _progressDialog->height()) / 2;
	//_progressDialog->setWindowFlags(Qt::SplashScreen);
	_progressDialog->setWindowModality(Qt::ApplicationModal);
	_progressDialog->setWindowFlag(Qt::WindowContextHelpButtonHint, 0);
	//_progressDialog->setWindowFlag(Qt::FramelessWindowHint, 1);
	
	if (enableCancel) connect(_progressDialog, &QProgressDialog::canceled, this, [&]() { userClickStopRun(); });
	else _progressDialog->setWindowFlags(_progressDialog->windowFlags() & ~Qt::WindowCloseButtonHint);
	
	_progressDialog->setCancelButton(0);
	_progressDialog->setWindowFlag(Qt::WindowStaysOnTopHint);
	_progressDialog->setWindowFlag(Qt::Dialog); //able to block input
	
	QProgressBar* progressBar = _progressDialog->findChild<QProgressBar*>();
	if (progressBar) {
		QString progressBarStyleSheet = "QProgressBar"
			"{"
			"border-radius: 6px;"
			" color: black; "
			"text-align: center;"
			"}"
			"QProgressBar::chunk "
			"{"
			"background-color: #09ff00;"
			"border-radius :6px;"

			"}";


		progressBar->setStyleSheet(progressBarStyleSheet);
	}

	_progressDialog->reset();
	_progressDialog->setWindowTitle("  ");
	_progressDialog->setMinimumDuration(0);
	_progressDialog->move(xScreen, yScreen);
	
	_progressDialog->setMaximum(maxValue + 1);
	

	//incrementProgressBar();
	
	_progressDialog->open();

}

void VisionApp::incrementProgressBar()
{

	if (_progressDialog)
	{		
		_progressValue++;
	
		_progressDialog->setValue(_progressValue);
		
	}
}

void VisionApp::progressBarRelease(bool directCancel)
{

	if (_progressDialog)
	{
	
		if (!directCancel) _progressDialog->setValue(_progressDialog->maximum());
		_progressDialog->blockSignals(true);
		_progressDialog->close();
		_progressDialog->blockSignals(false);
		_progressDialog->deleteLater();
		_progressDialog = nullptr;
	}

}

void VisionApp::loadingBarSetup(QString title)
{
	_loadingDialog = new QProgressDialog(ui.centralWidget);
	_loadingDialog->setRange(0, 0); // Indeterminate
	_loadingDialog->setLabelText(title);
	_loadingDialog->setCancelButton(nullptr); // Removes the Cancel button
	_loadingDialog->setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint); // Removes Close button

	int screenIndex = QApplication::desktop()->screenNumber(this);
	QRect screenGeometry = QApplication::desktop()->screenGeometry(screenIndex);
	int xScreen = screenGeometry.x() + (screenGeometry.width() - _loadingDialog->width()) / 2;
	int yScreen = screenGeometry.y() + (screenGeometry.height() - _loadingDialog->height()) / 2;
	_loadingDialog->move(xScreen, yScreen);
	_loadingDialog->show();
}

void VisionApp::loadingBarRelease()
{
	if (_loadingDialog)
	{
		_loadingDialog->close();
		delete _loadingDialog;
		_loadingDialog = nullptr;
	}
}

void VisionApp::updateWindowMask(int borderRadius)
{
	// Create a painter path to draw the rounded rectangle
	QPainterPath roundedPath;
	roundedPath.addRoundedRect(QRectF(0, 0, width(), height()), borderRadius, borderRadius);

	// Set the painter path as the shape of the window
	setMask(roundedPath.toFillPolygon().toPolygon());
	update();
}

void VisionApp::showActiveObject(QModelIndex & index)
{
	QStandardItemModel* model = qobject_cast<QStandardItemModel*>(ui.treeViewRecipeExplorer->model());
	if (index.parent() != QModelIndex() && model)
	{
		QStringList comboOption;
		QHash<QString, QVariant> objectID;
		QHash<QString, QVariant> spinBoxParam;
		QStandardItem *key;
		QStandardItem *value;
		QStringList header;
		QList<QStandardItem*> row;

		_objectModel.clear();

		header.append(QStringLiteral("Property"));
		header.append(QStringLiteral("Value"));
		_objectModel.setHorizontalHeaderLabels(header);

		QStandardItem* item = model->itemFromIndex(index);
		QString id = item->whatsThis();

		QVisionObject object = _visionObject.value(id);
		QStandardItem *parentItem = _objectModel.invisibleRootItem();

		_currentObjectID = object.objectName;

		// ObjectName
		////////////////////////////////////////////////
		key = new QStandardItem(QStringLiteral("ObjectName"));
		value = new QStandardItem(QString("%1").arg(object.objectName));

		row.clear();
		objectID.clear();

		objectID.insert(QStringLiteral("ObjectID"), object.objectID);
		objectID.insert(QStringLiteral("ObjectName"), object.objectName);
		objectID.insert(QStringLiteral("FieldName"), QStringLiteral("ObjectName"));

		key->setEditable(false);
		value->setData(objectID, Qt::UserRole + StandardItemUserRole::OBJECT_SRC);
		value->setEditable(false);

		row << key << value;
		parentItem->appendRow(row);
		////////////////////////////////////////////////

		// Skip
		////////////////////////////////////////////////
		key = new QStandardItem(QStringLiteral("Skip"));
		value = new QStandardItem();

		row.clear();
		objectID.clear();

		objectID.insert(QStringLiteral("ObjectID"), object.objectID);
		objectID.insert(QStringLiteral("ObjectName"), object.objectName);
		objectID.insert(QStringLiteral("FieldName"), QStringLiteral("Skip"));

		key->setEditable(false);
		value->setCheckable(true);
		value->setCheckState(object.skip == true ? Qt::Checked : Qt::Unchecked);
		value->setEditable(false);
		value->setData(objectID, Qt::UserRole + StandardItemUserRole::OBJECT_SRC);
		value->setData(Editor::DEFAULT, Qt::UserRole + StandardItemUserRole::EDITOR_TYPE);

		row << key << value;
		parentItem->appendRow(row);
		////////////////////////////////////////////////

		// Template
		////////////////////////////////////////////////
		key = new QStandardItem(QStringLiteral("Template"));
		value = new QStandardItem(QString("%1").arg(object.templateName));

		row.clear();
		objectID.clear();

		objectID.insert(QStringLiteral("ObjectID"), object.objectID);
		objectID.insert(QStringLiteral("ObjectName"), object.objectName);
		objectID.insert(QStringLiteral("FieldName"), QStringLiteral("Template"));

		key->setEditable(false);
		value->setData(objectID, Qt::UserRole + StandardItemUserRole::OBJECT_SRC);
		value->setEditable(false);

		row << key << value;
		parentItem->appendRow(row);
		////////////////////////////////////////////////


		// Angle
		////////////////////////////////////////////////
		key = new QStandardItem(QStringLiteral("Angle"));
		value = new QStandardItem(QString("%1").arg(object.angle));

		row.clear();
		objectID.clear();
		spinBoxParam.clear();

		objectID.insert(QStringLiteral("ObjectID"), object.objectID);
		objectID.insert(QStringLiteral("ObjectName"), object.objectName);
		objectID.insert(QStringLiteral("FieldName"), QStringLiteral("Angle"));
		spinBoxParam.insert(QStringLiteral("Min"), 0);
		spinBoxParam.insert(QStringLiteral("Max"), 360.0);
		spinBoxParam.insert(QStringLiteral("Wrapping"), true);
		spinBoxParam.insert(QStringLiteral("Decimal"), 1);
		spinBoxParam.insert(QStringLiteral("SingleStep"), 45.0);
		spinBoxParam.insert(QStringLiteral("Suffix"), QStringLiteral(" degrees"));

		key->setEditable(false);
		value->setData(objectID, Qt::UserRole + StandardItemUserRole::OBJECT_SRC);
		value->setData(Editor::DOUBLESPINBOX, Qt::UserRole + StandardItemUserRole::EDITOR_TYPE);
		value->setData(spinBoxParam, Qt::UserRole + StandardItemUserRole::EDITOR_PARAM);

		row << key << value;
		parentItem->appendRow(row);
		////////////////////////////////////////////////

		// View
		////////////////////////////////////////////////
		key = new QStandardItem(QStringLiteral("View"));
		value = new QStandardItem(QString("%1").arg(object.viewID));

		row.clear();
		objectID.clear();
		spinBoxParam.clear();

		objectID.insert(QStringLiteral("ObjectID"), object.objectID);
		objectID.insert(QStringLiteral("ObjectName"), object.objectName);
		objectID.insert(QStringLiteral("FieldName"), QStringLiteral("View"));

		key->setEditable(false);
		value->setData(objectID, Qt::UserRole + StandardItemUserRole::OBJECT_SRC);
		value->setEditable(false);

		row << key << value;
		parentItem->appendRow(row);
		////////////////////////////////////////////////

		//// Camera
		//////////////////////////////////////////////////
		//key = new QStandardItem(QStringLiteral("Camera"));
		//value = new QStandardItem(QString("%1").arg(object.camera));

		//row.clear();
		//comboOption.clear();
		//objectID.clear();

		//objectID.insert(QStringLiteral("ObjectName"), object.objectName);
		//objectID.insert(QStringLiteral("FieldName"), QStringLiteral("Camera"));
		//comboOption << QStringLiteral("Default");

		//key->setEditable(false);
		//value->setData(objectID, Qt::UserRole + StandardItemUserRole::OBJECT_SRC);
		//value->setData(Editor::COMBOBOX, Qt::UserRole + StandardItemUserRole::EDITOR_TYPE);
		//value->setData(comboOption, Qt::UserRole + StandardItemUserRole::EDITOR_PARAM);

		//row << key << value;
		//parentItem->appendRow(row);
		//////////////////////////////////////////////////

		ui.treeViewObjectExplorer->resizeColumnToContents(0);
		ui.treeViewObjectExplorer->show();
	}
	else
	{
		_objectModel.clear();
	}
}

void VisionApp::graphicsViewMousePress(QPointF pt, bool isLeftClick)
{
	_lastMousePressPos = pt;

	if (_pRecipeItem != nullptr)
	{
		bool flag = true;
		for (int i = 0; i < _dragROI.count(); i++)
		{
			QRectF rect = _dragROI.at(i)->getGeometry();
			if (rect.contains(pt) == true)
			{
				flag = false;
				break;
			}
		}

		if (flag)
		{
			_objectModel.clear();
			ui.treeViewRecipeExplorer->setCurrentIndex(_recipeModel.indexFromItem(_pRecipeItem));
			ui.treeViewRecipeExplorer->clearSelection();
			ui.treeViewResultExplorer->clearSelection();
		}
	}
}

void VisionApp::graphicsViewMouseReleased(QPointF pt, bool isLeftClick)
{
	if (isPage(UIPage::ZSTACK)) {
		checkSelectedView();
	}
}

void VisionApp::logMsg(QString msg, bool reset, QColor color, qreal size)
{
	if (msg.isEmpty()) return;

	if (reset == true)
		ui.textEditLog->clear();

	ui.textEditLog->setTextColor(color);
	ui.textEditLog->setFontPointSize(size);
	ui.textEditLog->append(msg);
}

void VisionApp::showInfo(const QString& info)
{
	ui.textEditInfo->clear();
	ui.textEditInfo->setFontPointSize(13);
	ui.textEditInfo->setTextColor(QColor(0, 255, 127));

	ui.textEditInfo->append(QStringLiteral("Recipe: %1 ").arg(Common::Directory::CurrentRecipe));
	ui.textEditInfo->append(QStringLiteral("User:   %1 ").arg(_currentUser));
	ui.textEditInfo->append(QStringLiteral(" "));
	/*ui.textEditInfo->append(QStringLiteral("Snap Time: %1 ms").arg(_snapTime));
	ui.textEditInfo->append(QStringLiteral("Insp Time: %1 ms").arg(_inspTime));*/
	ui.textEditInfo->append(QStringLiteral(" "));

	if (!info.isEmpty())
	{
		ui.textEditInfo->setFontPointSize(9);
		ui.textEditInfo->setTextColor(QColor(200, 200, 200));
		ui.textEditInfo->append(QStringLiteral(">> %1").arg(info));
	}
}

void VisionApp::clearLogMsg()
{
	ui.textEditLog->clear();
}

void VisionApp::clearInfoMsg()
{
	ui.textEditInfo->clear();
}

void VisionApp::openSystemSetting()
{
	//QDesktopServices::openUrl(QUrl::fromLocalFile(QStringLiteral("%1config//").arg(Common::Directory::LocalPath)));
	//showMsg(QStringLiteral("Restart app to apply changes"));
	toPage(UIPage::CONFIG);
}

void VisionApp::showRecipeInExplorer()
{
	if (!Common::Directory::CurrentRecipe.isEmpty())
	{
		QDesktopServices::openUrl(QUrl::fromLocalFile(QStringLiteral("%1recipe//%2//").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe)));
	}
	else
	{
		showMsg(QStringLiteral("Open a recipe to continue"));
	}
}

void VisionApp::goSleep(int msSleep)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(msSleep));
}

void VisionApp::setMessageBoxTitleColor(QMessageBox & messageBox, QColor color)
{
	HWND hwnd = reinterpret_cast<HWND>(messageBox.effectiveWinId());
	SetWindowLongPtr(hwnd, GWL_EXSTYLE, GetWindowLongPtr(hwnd, GWL_EXSTYLE) | WS_EX_LAYERED);
	SetLayeredWindowAttributes(hwnd, 0, 255, LWA_ALPHA);
	SetLayeredWindowAttributes(hwnd, color.rgb(), 0, LWA_COLORKEY);
}

QString VisionApp::dragROINameGenerator(QString name)
{
	int index = 1;
	QString id = name + QString::number(index);
	while (dragROIExistTest(id))
	{
		index++;
		id = name + QString::number(index);
	}

	return id;
}

bool VisionApp::dragROIExistTest(QString name)
{
	bool existFlag = false;
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->getName() == name)
		{
			existFlag = true;
			break;
		}
	}
	return existFlag;
}

QString VisionApp::getNextSamplePath()
{
	QDirIterator folderIt(Common::Directory::getRecipeSampleImagePath(), QDir::Dirs | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);
	QString lastFolder;
	int sampleIndex = 0;
	while (folderIt.hasNext()) {
		lastFolder = folderIt.next();
		QFileInfo folder(lastFolder);
		lastFolder = folder.completeBaseName();
		auto s = lastFolder.split("_");
		if (s.size() == 2) {
			auto num = s.at(1).toInt();
			if (num > sampleIndex) sampleIndex = num;
		}
	}

	sampleIndex++;

	auto rootPath = QString("%1\\Sample_%2\\").arg(Common::Directory::getRecipeSampleImagePath()).arg(sampleIndex);
	Common::Directory::createDir(rootPath);
	return rootPath;
}

QString VisionApp::getViewCollectionPath()
{
	QString rootPath;

	_jobThread.enableRun1stFOVOnly(false);

	if (ui.comboBox_collectView->currentIndex() == 0) {
		rootPath = getNextSamplePath();
	}
	else if (ui.comboBox_collectView->currentIndex() == 1) {
		rootPath = Common::Directory::getRecipeSetupImagePath();
	}
	else if (ui.comboBox_collectView->currentIndex() == 2) {
		rootPath = Common::Directory::getRecipeRefImagePath();
	}

	return rootPath;
}

void VisionApp::clear2DImages(QString folderPath)
{
	// Create a QDir object for the folder
	QDir folder(folderPath);

	// Check if the folder exists
	if (!folder.exists()) {
		ct::logger::warn("Folder does not exists: %s", folderPath.toStdString().c_str());
		return;
	}

	AuditLog::instance().log(QStringLiteral("CLEAR_2D_IMAGES"), folderPath);

	// Use QDirIterator to iterate through the files in the folder
	QDirIterator it(folderPath, QDir::Files | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);

	while (it.hasNext()) {
		QString filePath = it.next();

		if (!filePath.contains("IMap") && !filePath.contains("HeightMap")) {
			QFile::remove(filePath);
		}
	}
}

void VisionApp::clear3DImages(QString folderPath)
{
	// Create a QDir object for the folder
	QDir folder(folderPath);

	// Check if the folder exists
	if (!folder.exists()) {
		ct::logger::warn("Folder does not exists: %s", folderPath.toStdString().c_str());
		return;
	}

	AuditLog::instance().log(QStringLiteral("CLEAR_3D_IMAGES"), folderPath);

	// Use QDirIterator to iterate through the files in the folder
	QDirIterator it(folderPath, QDir::Files | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);

	while (it.hasNext()) {
		QString filePath = it.next();

		if (filePath.contains("IMap") || filePath.contains("HeightMap")) {
			QFile::remove(filePath);
		}
	}
}

void VisionApp::onCustomContextMenu(const QPoint & point)
{
	QModelIndex index = ui.treeViewRecipeExplorer->indexAt(point);

	if ((index.row() != -1) && (index.parent() == QModelIndex()))
	{
		QMenu menu(this);
		menu.addAction(ui.actionAddObject);
		menu.exec(ui.treeViewRecipeExplorer->viewport()->mapToGlobal(point));
	}
	else
	{
		if (index.row() != -1)
		{
			QMenu menu(this);

			QAction* findVisionObjectAction = new QAction(tr("Find Vision Object"), this);
			connect(findVisionObjectAction, &QAction::triggered, this, &VisionApp::findVisionObject);

			menu.addAction(findVisionObjectAction);
			menu.addAction(ui.actionEditTemplate);
			menu.addAction(ui.actionDuplicateVisionObjects);

			menu.exec(ui.treeViewRecipeExplorer->viewport()->mapToGlobal(point));
		}
	}
}

//Draw Roi
//Draw VISION_OBJECT using mouse
void VisionApp::processROIOption(QRect rubberBandRect, QPointF fromScenePoint, QPointF toScenePoint)
{
	if ((fromScenePoint.isNull()) && (toScenePoint.isNull()))
	{
		if (_pRecipeItem != nullptr)
		{
			if (_editMode == EditMode::VISION_OBJECT) {
				addObjectFromView();
				ui.toolButtonSelectMode->animateClick();
			}
			else if (_editMode == EditMode::POSITION_PORTABILITY_MODE)
			{
				QRectF croppedRegion = QRectF(_startDragPos, _endDragPos);
				QImage croppedImageWorld = _imageWorld.copy(croppedRegion.toRect());

				QRectF bounds = croppedRegion.isNull() ? _pGraphicsSceneMain->itemsBoundingRect() : croppedRegion;

				QImage out(bounds.size().toSize(), QImage::Format_ARGB32);
				out.fill(Qt::transparent);

				QPainter painter(&out);
				painter.setRenderHint(QPainter::Antialiasing, true);
				painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

				_pPixmapItemMain->setVisible(false);
				_pGraphicsSceneMain->render(&painter,
					QRectF(0, 0, out.width(), out.height()),   // target in output image
					bounds);     
				//out = out.copy(croppedRegion.toRect());// source from scene

				out.save("render.jpg");
				

				auto baseOffset = SystemData::instance()._portability.current_info.portability_point -
					SystemData::instance()._portability.ref_info.portability_point;
				SystemData::instance()._portability.current_info.offset_point = baseOffset;

				_guided_2D3D_AlignmentTab->loadPositionPortabilityImages(croppedImageWorld, out);
				_guided_2D3D_AlignmentTab->setCurrentPositionPortabilityOffset(baseOffset);
				_guided_2D3D_AlignmentTab->setMode(Guided_2D3D_AlignmentTab::AlignmentMode::OFFSET_POSITION_PORTABILITY);
				_guided_2D3D_AlignmentTab->show();
				
				_pPixmapItemMain->setVisible(true);
				ct::logger::info("Position Portability Mode");
				ui.toolButtonSelectMode->animateClick();
			}
		}
	}
	else
	{
		_startDragPos = fromScenePoint;
		_endDragPos = toScenePoint;
	}
}

bool VisionApp::isPage(UIPage page) {

	if (ui.stackedWidget->currentIndex() == (int)page) return true;
	return false;
}

void VisionApp::checkRecipeFacing(QString recipeName, bool &isTop)
{
	qDebug() << "Recipe Name: " << recipeName;
	if (!recipeName.isEmpty()) {
		QChar lastAlphabet;
		for (int i = recipeName.length() - 1; i >= 0; i--) {
			if (recipeName.at(i).isLetter()) {
				lastAlphabet = recipeName.at(i);
				break;
			}
		}

		if (lastAlphabet == 'T') {
			qDebug() << "isTop: true";
			isTop = true;
			
		}
		else if (lastAlphabet == 'B') {
			qDebug() << "isTop: false";
			isTop = false;
			
		}
		else {
			qDebug() << "Last word is neither T nor B";
			
		}
	}
	else {
		qDebug() << "No words in the string";
		
	}
}

bool VisionApp::toPage(UIPage page) {
	//showAllGraphicItems(false); //view, vo, path
	toggleFiducialUI(false);

	//algo setup ROIs are page-scoped: leaving the page hides them
	if (page != UIPage::ALGO_SETUP && _algoOcrRoi1Box) hideAlgoSetupRois();

	//alignment feature ROIs are page-scoped too
	if (page != UIPage::LASER && _alignCircleRoi) hideAlignRois();
	bool shown;
	bool isTop = true;
	QString facing;
	switch (page) {
	case UIPage::RECIPE:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Recipe Tab"));
		showPath(false);
		showView(false);
		showLineScans(false);
		showVisionObject(true);
		ui.gridLayout_propertyTabUnassignedObject->addWidget(ui.unassignedViewsFrame);
		return true;
	case UIPage::ROI_EDITOR:
		unlockAllROIs();
		shown = showRightTab((int)page, QStringLiteral("Open ROI Editor"));
		showVisionObject(true);
		showView(false);
		showLineScans(false);
		showPath(false);
		ui.gridLayout_roiTabUnassignedObject->addWidget(ui.unassignedViewsFrame);
		//showLogTab(3, shown); // setup checklist

		return true;
	case UIPage::SCALING:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Scaling"));
		return true;
	case UIPage::PATH:
		unlockAllROIs();
		shown = showRightTab((int)page, QStringLiteral("Open Path Editor"));
		showView(true);
		showLineScans(true);
		showPath(true);
		showVisionObject(false);
		//showLogTab(3, shown); // setup checklist

		return true;
	case UIPage::LIGHTING:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Lighting Control"));
		return true;
	case UIPage::TEMPLATE_LIB:
		unlockAllROIs();
		restoreBorderColors();
		showRightTab((int)page, QStringLiteral("Open Template Library Tab"));
		showPath(false);
		showView(true);
		showLineScans(true);
		showVisionObject(true);
		return true;
	case UIPage::RECIPE_SETUP:
		unlockAllROIs();
		shown = showRightTab((int)page, QStringLiteral("Open Recipe Setup"));
		showView(true);
		showLineScans(true);
		showVisionObject(false);
		showPath(false);
		prepareRecipeSetupPage();
		//showLogTab(3, shown); // setup checklist

		return true;
	case UIPage::NAMING_CONVENTION:
		unlockAllROIs();
		shown = showRightTab((int)page, QStringLiteral("Open Naming Convention"));
		showView(false);
		showLineScans(false);
		showVisionObject(true);
		showPath(false);
		//showLogTab(3, shown); // setup checklist	
		
		checkRecipeFacing(Common::Directory::CurrentRecipe, isTop);

		facing = isTop ?
			"<b> + Top recipe Detected!"
			: "<b> + Bot recipe Detected!";
		ui.label_recipeFacing->setText(facing);

		return true;
	case UIPage::CONFIG:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Config"));
		return true;
	case UIPage::ANALYSIS:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Analysis"));
		return true;
	case UIPage::TESTRUN:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Test Run"));
		return true;
	case UIPage::LASER:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Laser"));
		updateAlignRoiVisibility();
		return true;
	case UIPage::PORTABILITY:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Portability")); 
		return true;
	case UIPage::AIMODEL:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open AI Model"));
		return true;
	case UIPage::COLOR_SEGMENT:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Color Segment"));
		return true;
	case UIPage::ZSTACK:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Z Stack"));
		return true;
	case UIPage::BARCODE_READER:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Barcode Reader"));
		return true;
	case UIPage::ALGO_SETUP:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Algo Setup"));
		updateAlgoRoiVisibility();
		return true;
	case UIPage::DRY_RUN:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open Dry Run"));
		return true;
	case UIPage::UNIT_CONFIG:
		lockAllROIs();
		saveBorderColors();
		applySkipColors();
		showView(false);
		showLineScans(false);
		showVisionObject(true);
		showPath(false);
		showRightTab((int)page, QStringLiteral("Open Unit Config"));
		return true;
	case UIPage::OPTICS3D:
		unlockAllROIs();
		showRightTab((int)page, QStringLiteral("Open 3DOptics"));
		return true;
	};
	

	return false;
}

void VisionApp::prepareRecipeSetupPage()
{
	toggleFiducialUI(true);
	showFiducial(ui.comboBox_fids->currentIndex());

	//update plane
	auto& fl = _plane.corner_points[(int)Corner::FRONTLEFT];
	ui.label_frontLeft->setText(QString("x: %1\ny: %2\nz: %3\n").arg(fl.wx).arg(fl.wy).arg(fl.wz));

	auto& br = _plane.corner_points[(int)Corner::BACKRIGHT];
	ui.label_backRight->setText(QString("x: %1\ny: %2\nz: %3\n").arg(br.wx).arg(br.wy).arg(br.wz));
}

void VisionApp::toggleMenu()
{
	int widthExtended = 0;

	//GET WIDTH
	int maxWidth = 220;
	auto width = ui.frame_leftMenuBar->width();
	int maxExtend = maxWidth;
	int standard = 70;

	// SET MAX WIDTH
	if (width == 70)
	{
		widthExtended = maxExtend;
	}
	else
	{
		widthExtended = standard;

	}

	// ANIMATION
	QPropertyAnimation *animation = new QPropertyAnimation(ui.frame_leftMenuBar, "minimumWidth");
	animation->setDuration(300);
	animation->setStartValue(width);
	animation->setEndValue(widthExtended);
	animation->setEasingCurve(QEasingCurve::InOutQuad);
	animation->start();
	connect(animation, &QPropertyAnimation::finished, this, [=]() { 

		/*if (windowState() == Qt::WindowFullScreen) setMask(QRegion());
		else updateWindowMask(14);*/
	});
}

void VisionApp::showSetupPage()
{
	ui.stackedWidgetViewSelection->setCurrentIndex(0);
	if (ui.frame_rightTab->isHidden())
	{
		showRightTab(ui.stackedWidget->currentIndex(), "");

	}
	if (ui.frame_leftTab->isHidden())
	{
		if (g_viewMode == (int)ViewMode::SINGLE) showLeftTab(ui.stackedWidget_leftTab->currentIndex(), "");
	}

}

void VisionApp::showDatasetPage()
{
	if (!ui.frame_rightTab->isHidden())
	{
		showRightTab(ui.stackedWidget->currentIndex(), "");
	}
	if (!ui.frame_leftTab->isHidden())
	{
		if (g_viewMode == (int)ViewMode::SINGLE) showLeftTab(ui.stackedWidget_leftTab->currentIndex(), "");
	}
	ui.stackedWidgetViewSelection->setCurrentIndex(3);
}

void VisionApp::showProductionPage()
{
	/*if (!ui.frame_rightTab->isHidden())
	{
		showRightTab((int)UIPage::RECIPE, "");
	}
	if (!ui.frame_leftTab->isHidden())
	{
		if (g_viewMode == (int)ViewMode::SINGLE) showLeftTab(ui.stackedWidget_leftTab->currentIndex(), "");
	}*/


	ui.stackedWidget->setCurrentIndex(0);
	if (!ui.frame_rightTab->isHidden()) showRightTab((int)UIPage::RECIPE, "");
	
	showProductionFOV(720);
	ui.stackedWidgetViewSelection->setCurrentIndex(6);
	ui.page_23->show();
	//showRightTabFOV();

	stopLiveView();

}

void VisionApp::showRecipeSettingsMenu()
{
	auto toggle = _recipeSettingsMenu->isHidden();
	toggleRecipeSettingsMenu(toggle);
	toggleSystemSettingsMenu(false);
}

void VisionApp::showSystemSettingsMenu() {
	auto toggle = _systemSettingsMenu->isHidden();
	toggleRecipeSettingsMenu(false);
	toggleSystemSettingsMenu(toggle);
}

void VisionApp::toggleRecipeSettingsMenu(bool enable)
{
	auto x = ui.toolButtonRecipeSettings->x() + ui.toolButtonRecipeSettings->width()*1.1;
	auto y = ui.toolButtonRecipeSettings->y() + ui.toolButtonRecipeSettings->height();
	int w = _recipeSettingsMenu->width(), h = _recipeSettingsMenu->height();
	// set recipe settings Menu to right side if the position is at 0,0
	if (_recipeSettingsMenu->pos() == QPoint(0, 0) && _recipeSettingsMenu->isHidden())
	{
		_recipeSettingsMenu->move(x - w, y);
	}


	// ANIMATION
	QPropertyAnimation *animation = new QPropertyAnimation(_recipeSettingsMenu, "geometry");
	animation->setDuration(300);
	QRect startValue = _recipeSettingsMenu->geometry();
	animation->setStartValue(startValue);
	animation->setEasingCurve(QEasingCurve::InOutQuad);

	if (enable)
	{
		_recipeSettingsMenu->show();
		_recipeSettingsMenu->activateWindow();
		_recipeSettingsMenu->raise();
		animation->setEndValue(QRect(x, y, w, h));
		animation->start();

	}
	else
	{
		animation->setEndValue(QRect(x - w, y, w, h));
		animation->start();
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			_recipeSettingsMenu->hide();
		});
	}
}

void VisionApp::toggleSystemSettingsMenu(bool enable)
{
	auto x = ui.toolButtonSystemSettings->x() + ui.toolButtonSystemSettings->width()*1.1;
	auto y = ui.toolButtonSystemSettings->y() + ui.toolButtonSystemSettings->height();
	int w = _systemSettingsMenu->width(), h = _systemSettingsMenu->height();

	// set recipe settings Menu to right side if the position is at 0,0
	if (_systemSettingsMenu->pos() == QPoint(0, 0) && _systemSettingsMenu->isHidden())
	{
		_systemSettingsMenu->move(x - w, y);
	}


	// ANIMATION
	QPropertyAnimation *animation = new QPropertyAnimation(_systemSettingsMenu, "geometry");
	animation->setDuration(300);
	QRect startValue = _systemSettingsMenu->geometry();
	animation->setStartValue(startValue);
	animation->setEasingCurve(QEasingCurve::InOutQuad);

	if (enable)
	{
		_systemSettingsMenu->show();
		_systemSettingsMenu->activateWindow();
		_systemSettingsMenu->raise();
		animation->setEndValue(QRect(x, y, w, h));
		animation->start();

	}
	else
	{
		animation->setEndValue(QRect(x - w, y, w, h));
		animation->start();
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			_systemSettingsMenu->hide();
		});
	}
}


void VisionApp::toggleRightMenu()
{
}

void VisionApp::recipeSettingsMenuBtnPressed(int btn)
{
	switch (btn)
	{
	case NEWRECIPE:
		//createRecipe("NEWRECIPE");
		newRecipe();
		openRecipe(Common::Directory::CurrentRecipe);
		break;
	case LOADRECIPE:
		openRecipe();
		break;
	case SAVERECIPE:
		saveRecipe();
		break;
	case DUPLICATERECIPE:
		duplicateRecipe();
		break;
	case ARCHIVERECIPE:
		archiveRecipe();
		break;
	case RESTORERECIPE:
		restoreRecipe();
		break;
	case SHOWRECIPEINEXPLORER:
		showRecipeInExplorer();
		break;
	case PATHEDITOR:
		toPage(UIPage::PATH);
		break;
	}
}

void VisionApp::systemSettingsMenuBtnPressed(int btn)
{
	switch (btn)
	{
	case CONFIG:
		openSystemSetting();
		break;
	case SCALING:
		toPage(UIPage::SCALING);
		break;
	case LIGHTING:
		toPage(UIPage::LIGHTING);
		break;
	case ANALYSIS:
		toPage(UIPage::ANALYSIS);
		break;
	case TESTRUN:
		toPage(UIPage::TESTRUN);
		break;
	case LASER:
		toPage(UIPage::LASER);
		break;
	case PORTABILITY:
		toPage(UIPage::PORTABILITY);
		break;
	case AIMODEL:
		toPage(UIPage::AIMODEL);
		break;
	case ZSTACK:
		toPage(UIPage::ZSTACK);
		break;
	case OPTICS3D:
		toPage(UIPage::OPTICS3D);
		break;
	case BARCODEREADER:
		toPage(UIPage::BARCODE_READER);
		break;
	case ALGOSETUP:
		toPage(UIPage::ALGO_SETUP);
		break;
	case DRYRUN:
		toPage(UIPage::DRY_RUN);
		break;
	}
}

bool VisionApp::showRightTab(int index, QString status)
{
	if (ui.stackedWidget->currentIndex() == index &&
		ui.frame_rightTab->isVisible())
	{
		for (auto* roi : _dragROI)
			roi->setLocked(false);   
	}

	int rightTabsize = 660; //must be 660 so user dont have to scroll horizontally for right tab

	//size against the screen THIS window is on (the app may run on the second monitor)
	const int screenIndex = QApplication::desktop()->screenNumber(this);
	const int resoWidth = QApplication::desktop()->screenGeometry(screenIndex).width();

	const int leftMenuWidth = 67;
	int workspaceMaxWidth = resoWidth - rightTabsize - leftMenuWidth;
	if (workspaceMaxWidth < 800) workspaceMaxWidth = 800; //at least need 800 px

	ct::logger::trace("Workspace max width: %d, reso: %d, righttab: %d, leftWidth: %d", workspaceMaxWidth, resoWidth, rightTabsize, leftMenuWidth);

	ui.frame_workSpace->setMaximumWidth(workspaceMaxWidth);

	if (g_viewMode == (int)ViewMode::SINGLE) rightTabsize = 480;
	for (int i = 0; i < _dragROI.size(); i++) _dragROI[i]->setFrozen(true);

	bool propertyShown = false;
	if (ui.stackedWidget->currentIndex() != index) 
	{
		ui.stackedWidget->currentWidget()->hide();
		if (ui.frame_rightTab->isHidden())
		{
			ui.stackedWidget->setCurrentIndex(index);
			ui.frame_rightTab->show();
			resizeWidgetAnimation(ui.frame_rightTab, 0, rightTabsize, false, index, ui.stackedWidget);
		}
		else
		{
			resizeWidgetAnimation(ui.frame_rightTab, 0, rightTabsize, true, index, ui.stackedWidget);
		}

		propertyShown = true;
	}

	if (!propertyShown)
	{
		if (ui.frame_rightTab->isHidden()) {
			ui.frame_rightTab->show();
			resizeWidgetAnimation(ui.frame_rightTab, 0, rightTabsize, false, index, ui.stackedWidget);
			propertyShown = true;
		}
		else {
			resizeWidgetAnimation(ui.frame_rightTab, 0, rightTabsize, false, index, ui.stackedWidget);
		}
	}

	showStatus(status);
	return propertyShown;
}

void VisionApp::showLogTab()
{
	ui.tabWidgetOutput->setMinimumWidth(this->width() / 3.0);
	ui.tabWidgetOutput->setMaximumWidth(this->width() / 3.0);

	// ANIMATION
	QPropertyAnimation *animation = new QPropertyAnimation(ui.tabWidgetOutput, "geometry");
	animation->setDuration(300);
	QRect startValue = ui.tabWidgetOutput->geometry();
	animation->setStartValue(startValue);
	animation->setEasingCurve(QEasingCurve::InOutQuad);

	if (ui.tabWidgetOutput->isHidden())
	{
		ui.tabWidgetOutput->show();
		ui.tabWidgetOutput->activateWindow();
		ui.tabWidgetOutput->raise();
		ui.tabWidgetOutput->setParent(this);
		animation->setEndValue(QRect(0, ui.statusBar->y() - ui.tabWidgetOutput->height(), ui.tabWidgetOutput->width(), ui.tabWidgetOutput->height()));
		animation->start();
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
		});
	}
	else
	{
		animation->setEndValue(QRect(0, this->height(), ui.tabWidgetOutput->width(), ui.tabWidgetOutput->height()));
		animation->start();
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			ui.tabWidgetOutput->hide();
		});
	}
	updateSetupCheckList();
}

void VisionApp::showLogTab(int index, bool isShow)
{
	ui.tabWidgetOutput->setMinimumWidth(this->width() / 3.0);
	ui.tabWidgetOutput->setMaximumWidth(this->width() / 3.0);

	// ANIMATION
	QPropertyAnimation *animation = new QPropertyAnimation(ui.tabWidgetOutput, "geometry");
	animation->setDuration(300);
	QRect startValue = ui.tabWidgetOutput->geometry();
	animation->setStartValue(startValue);
	animation->setEasingCurve(QEasingCurve::InOutQuad);

	if (isShow)
	{
		ui.tabWidgetOutput->show();
		ui.tabWidgetOutput->activateWindow();
		ui.tabWidgetOutput->raise();
		ui.tabWidgetOutput->setParent(this);
		animation->setEndValue(QRect(0, ui.statusBar->y() - ui.tabWidgetOutput->height(), ui.tabWidgetOutput->width(), ui.tabWidgetOutput->height()));
		animation->start();
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
		});
	}
	else
	{
		animation->setEndValue(QRect(0, this->height(), ui.tabWidgetOutput->width(), ui.tabWidgetOutput->height()));
		animation->start();
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			ui.tabWidgetOutput->hide();
		});
	}
	ui.tabWidgetOutput->setCurrentIndex(index);
	updateSetupCheckList();
}

void VisionApp::hideRightMenu(bool blockEventFilter)
{
	if (!_rightMenu->isHidden()) rightMenuMouseMoveEvent(QPoint(0, 0), blockEventFilter);
}

void VisionApp::blockRightMenu(bool block)
{
	_blockEventFilter = block;
}

void VisionApp::toPageLeft()
{
	if (g_viewMode != (int)ViewMode::SINGLE) return;

	QObject* senderObj = sender();
	if (senderObj == ui.toolButton_ImageViewer)
	{
		showLeftTab(0, QStringLiteral("Open Image Viewer Tab"));
	}
}

bool VisionApp::showLeftTab(int index, QString status)
{
	if (g_viewMode != (int)ViewMode::SINGLE) return true;

	for (int i = 0; i < _dragROI.size(); i++)
	{
		_dragROI[i]->setFrozen(true);
	}

	bool propertyShown = false;
	if (ui.stackedWidget_leftTab->currentIndex() != index)
	{
		ui.stackedWidget_leftTab->currentWidget()->hide();
		if (ui.frame_leftTab->isHidden())
		{
			ui.stackedWidget_leftTab->setCurrentIndex(index);
			resizeWidgetAnimation(ui.frame_leftTab, 0, 250, false, index, ui.stackedWidget_leftTab);
		}
		else
		{
			resizeWidgetAnimation(ui.frame_leftTab, 0, 250, true, index, ui.stackedWidget_leftTab);
		}

		propertyShown = true;
	}

	if (!propertyShown)
	{
		if (ui.frame_leftTab->isHidden()) {
			ui.frame_leftTab->show();
			resizeWidgetAnimation(ui.frame_leftTab, 0, 200, false, index, ui.stackedWidget_leftTab);
			propertyShown = true;
		}
		else {
			resizeWidgetAnimation(ui.frame_leftTab, 0, 200, false, index, ui.stackedWidget_leftTab);
		}
	}

	showStatus(status);
	return propertyShown;
}


void VisionApp::resizeLogTab()
{
	if (ui.tabWidgetOutput->isHidden()) return;
	ui.tabWidgetOutput->setMinimumWidth(this->width() / 3.0);
	ui.tabWidgetOutput->setMaximumWidth(this->width() / 3.0);

	auto x = 0;
	auto y = ui.statusBar->y() - ui.tabWidgetOutput->height();
	auto w = ui.tabWidgetOutput->width();
	auto h = ui.tabWidgetOutput->height();
	moveWidgetAnimation(ui.tabWidgetOutput, QRect(0, y, w, h), true);
}

void VisionApp::maximize_restoreWindow()
{
	QPropertyAnimation *animation = new QPropertyAnimation(this, "geometry");
	animation->setDuration(100);

	int screenIndex = QApplication::desktop()->screenNumber(this);

	if (!_maximizedState)
	{
		_maximizedState = true;
		setMask(QRegion());
		animation->setEndValue(QRect(QApplication::desktop()->screenGeometry(screenIndex).x(), QApplication::desktop()->screenGeometry(screenIndex).y(), QApplication::desktop()->screenGeometry(screenIndex).width(), QApplication::desktop()->screenGeometry(screenIndex).height()));
		animation->start();
		ui.toolButton_maximize_restore->setIcon(QIcon(":/16x16/Icon/16x16/cil-window-restore.png"));
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			setWindowState(Qt::WindowFullScreen);
			hideRightMenu(false);
			resizeLogTab();
		});

	}
	else
	{
		_maximizedState = false;
		animation->setEndValue(QRect(QApplication::desktop()->screenGeometry(screenIndex).x() + 100, QApplication::desktop()->screenGeometry(screenIndex).y(), 1566, 1010));
		animation->start();
		ui.toolButton_maximize_restore->setIcon(QIcon(":/16x16/Icon/16x16/cil-window-maximize.png"));
		connect(animation, &QPropertyAnimation::finished, this, [=]() {
			setWindowState(Qt::WindowNoState);
			updateWindowMask(14);
			hideRightMenu(false);
			resizeLogTab();
		});

	}
}

void VisionApp::maximizedWindow()
{
	QPropertyAnimation *animation = new QPropertyAnimation(this, "geometry");
	animation->setDuration(100);

	int screenIndex = QApplication::desktop()->screenNumber(this);

	_maximizedState = true;
	setMask(QRegion());
	animation->setEndValue(QRect(QApplication::desktop()->screenGeometry(screenIndex).x(), QApplication::desktop()->screenGeometry(screenIndex).y(), QApplication::desktop()->screenGeometry(screenIndex).width(), QApplication::desktop()->screenGeometry(screenIndex).height()));
	animation->start();
	ui.toolButton_maximize_restore->setIcon(QIcon(":/16x16/Icon/16x16/cil-window-restore.png"));
	connect(animation, &QPropertyAnimation::finished, this, [=]() {
		setWindowState(Qt::WindowFullScreen);
		hideRightMenu(false);
		resizeLogTab();
	});
}

void VisionApp::closeWindow() //fade out
{
	ui.tabWidgetOutput->hide();
	//saveRecipe();

	//fade out
	QGraphicsOpacityEffect *eff = new QGraphicsOpacityEffect(this);
	setGraphicsEffect(eff);
	QPropertyAnimation *a = new QPropertyAnimation(eff, "opacity");
	a->setDuration(500);
	a->setStartValue(1);
	a->setEndValue(0);
	a->setEasingCurve(QEasingCurve::OutBack);
	a->start(QPropertyAnimation::DeleteWhenStopped);
	connect(a, &QPropertyAnimation::finished, this, [=]() {
		setGraphicsEffect(nullptr);
		close();
	});
}

void VisionApp::fadeIn()
{
	QGraphicsOpacityEffect *eff = new QGraphicsOpacityEffect();
	setGraphicsEffect(eff);
	QPropertyAnimation *a = new QPropertyAnimation(eff, "opacity");
	a->setDuration(500);
	a->setStartValue(0);
	a->setEndValue(1);
	a->setEasingCurve(QEasingCurve::InBack);
	a->start(QPropertyAnimation::DeleteWhenStopped);
	connect(a, &QPropertyAnimation::finished, this, [=]() {
		setGraphicsEffect(nullptr);
	});
}

void VisionApp::visionObjectMode()
{
	ui.toolButtonDrawVisionObjMode->setChecked(true);
	setEditMode(EditMode::VISION_OBJECT);
	ui.toolButtonSelectMode->setChecked(false);
	setdragMode(true);
}

void VisionApp::selectMode()
{
	ui.toolButtonSelectMode->setChecked(true);
	setEditMode(EditMode::SELECT);
	ui.toolButtonDrawVisionObjMode->setChecked(false);
	setdragMode(false);
}

void VisionApp::selectAssigned()
{
	for (const auto& v : _views) {
		if (v.pDragBox->isSelected()) {
			for (const auto& vo : v.vision_obj_IDs) {
				_visionObject[vo].pDragBox->setSelected(true);
			}
			return;
		}
	}

	for (const auto& v : _lineScans) {
		if (v.pDragBox->isSelected()) {
			for (const auto& vo : v.vision_obj_IDs) {
				_visionObject[vo].pDragBox->setSelected(true);
			}
			return;
		}
	}
}

void VisionApp::guidedPositionPortabilityMode()
{
	setEditMode(EditMode::POSITION_PORTABILITY_MODE);
	ui.toolButtonSelectMode->setChecked(false);
	for (int i = 0; i < _dragROI.size(); i++)
	{
		_dragROI.at(i)->setFlag(QGraphicsItem::ItemIsSelectable, false);
		_dragROI.at(i)->setFlag(QGraphicsItem::ItemIsMovable, false);
		_dragROI.at(i)->setDragable(false);
	}
	//setdragMode(true);
}

void VisionApp::setdragMode(bool flag)
{
	_dragMode = flag;

	// If somebody is trying to *unlock* but we�re still in locked-mode, do nothing:
	if (!flag && roiLocked) {
		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI.at(i)->setFlag(QGraphicsItem::ItemIsSelectable, true);
			_dragROI.at(i)->setFlag(QGraphicsItem::ItemIsMovable, false);
			_dragROI.at(i)->setDragable(false);
		}

		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI[i]->setFrozen(false);
		}
		return;
	}

	if (_dragMode)
	{
		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI.at(i)->setFlag(QGraphicsItem::ItemIsSelectable, false);
			_dragROI.at(i)->setFlag(QGraphicsItem::ItemIsMovable, false);
			_dragROI.at(i)->setDragable(false);
		}

		for (int i = 0; i < _viewROI.size(); i++)
		{
			_viewROI.at(i)->setFlag(QGraphicsItem::ItemIsSelectable, false);
		}

		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI[i]->setFrozen(true);
		}
	}
	else
	{
		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI.at(i)->setFlag(QGraphicsItem::ItemIsSelectable, true);
			_dragROI.at(i)->setFlag(QGraphicsItem::ItemIsMovable, true);
			_dragROI.at(i)->setDragable(true);
		}

		for (int i = 0; i < _viewROI.size(); i++)
		{
			_viewROI.at(i)->setFlag(QGraphicsItem::ItemIsSelectable, true);
		}

		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI[i]->setFrozen(false);
		}
	}

}

void VisionApp::setRightMousePressed(QPoint point)
{
	QPoint pointCursor = QCursor::pos();
	qDebug() << "RightMouseis Pressed";

	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->isSelected())
		{
			_dragROI[i]->setSelected(false);
		}
	}

	VisionAppQDragBox* dragbox = nullptr;
	int maxZ = -99999;
	for (int i = 0; i < _dragROI.size(); i++)
	{
		auto x = _dragROI[i]->pos().x();
		auto y = _dragROI[i]->pos().y();
		auto w = _dragROI[i]->getGeometry().width();
		auto h = _dragROI[i]->getGeometry().height();
		if ((point.x() < x + w) && (point.x() > x) && (point.y() < y + h) && (point.y() > y) && _dragROI[i]->isVisible())
		{
			if (_dragROI[i]->zValue() > maxZ)
			{
				maxZ = _dragROI[i]->zValue();
				dragbox = _dragROI[i];
			}
		}
	}

	if (dragbox != nullptr)
	{
		qDebug() << "selectDragBox";
		dragbox->setSelected(true);
	}
	else
	{
		qDebug() << "show Roi Menu";
		actionMenu.exec(pointCursor);
	}
}

void VisionApp::toggleDualView()
{
	QSignalBlocker sb1(ui.toolButton_toggleDualView);
	QSignalBlocker sb2(ui.toolButton_toggleWorldView);
	QSignalBlocker sb3(ui.toolButton_toggleFovView);
	ui.toolButton_toggleDualView->setChecked(true);
	ui.toolButton_toggleWorldView->setChecked(false);
	ui.toolButton_toggleFovView->setChecked(false);

	ui.stackedWidgetViewSelection->setCurrentIndex(0);
	ui.stackedWidget_graphicViews->setCurrentIndex(1);
	ui.graphicsViewFOV->setMaximumSize(QSize(16777215, 16777215));
	ui.graphicsViewFOV->show();
	ui.gridLayout_MiniFOV->addWidget(ui.graphicsViewFOV);
}

void VisionApp::toggleWorldView()
{
	QSignalBlocker sb1(ui.toolButton_toggleDualView);
	QSignalBlocker sb2(ui.toolButton_toggleWorldView);
	QSignalBlocker sb3(ui.toolButton_toggleFovView);
	ui.toolButton_toggleDualView->setChecked(false);
	ui.toolButton_toggleWorldView->setChecked(true);
	ui.toolButton_toggleFovView->setChecked(false);

	ui.stackedWidget_graphicViews->setCurrentIndex(1);
	ui.graphicsViewFOV->hide();
}

void VisionApp::toggleFOVView()
{
	QSignalBlocker sb1(ui.toolButton_toggleDualView);
	QSignalBlocker sb2(ui.toolButton_toggleWorldView);
	QSignalBlocker sb3(ui.toolButton_toggleFovView);
	ui.toolButton_toggleDualView->setChecked(false);
	ui.toolButton_toggleWorldView->setChecked(false);
	ui.toolButton_toggleFovView->setChecked(true);

	ui.stackedWidgetViewSelection->setCurrentIndex(0);
	ui.stackedWidget_graphicViews->setCurrentIndex(0);
	ui.graphicsViewFOV->setMaximumSize(QSize(16777215, 16777215));
	ui.graphicsViewFOV->show();
	ui.gridLayout_MainFOV->addWidget(ui.graphicsViewFOV);
}

void VisionApp::showRightTabFOV()
{
	QSignalBlocker sb1(ui.toolButton_toggleDualView);
	QSignalBlocker sb2(ui.toolButton_toggleWorldView);
	QSignalBlocker sb3(ui.toolButton_toggleFovView);
	ui.toolButton_toggleDualView->setChecked(true);
	ui.toolButton_toggleWorldView->setChecked(false);
	ui.toolButton_toggleFovView->setChecked(false);
	ui.graphicsViewFOV->setMaximumSize(QSize(500, 500));
	ui.graphicsViewFOV->show();
	ui.gridLayout_MiniFOV->addWidget(ui.graphicsViewFOV);
}

void VisionApp::showProductionFOV(const int& viewSize)
{
	//the FOV view is shared between pages, so it must be re-added here; sizing
	//is left to the .ui layout (no fixed size), only a sane minimum is kept
	Q_UNUSED(viewSize);
	ui.graphicsViewFOV->setMinimumSize(QSize(400, 300));
	ui.graphicsViewFOV->setMaximumSize(QSize(16777215, 16777215));
	ui.graphicsViewFOV->show();
	ui.gridLayout_ProductionFOV->addWidget(ui.graphicsViewFOV);
}

void VisionApp::wheelEventStart()
{
	for (int i = 0; i < _dragROI.size(); i++)
	{
		_dragROI[i]->setFrozen(true);
	}

	QTimer::singleShot(500, this, &VisionApp::wheelEventEnd);
}

void VisionApp::wheelEventEnd()
{
	if (!_dragMode)
	{
		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI[i]->setFrozen(false);
		}

		_pGraphicsSceneMain->update();
	
	}
	
}

void VisionApp::toggleOfflineRun()
{
	QSignalBlocker sb1(ui.toolButton_toggleOfflineRun);
	QSignalBlocker sb2(ui.toolButton_toggleOnlineRun);
	ui.toolButton_toggleOfflineRun->setChecked(true);
	ui.toolButton_toggleOnlineRun->setChecked(false);

	ui.comboBox_runType->clear();
	QStringList runType = {
		"2D Inspection",
		"3D Inspection",
		"Full Inspection"
	};
	ui.comboBox_runType->addItems(runType);
	ui.comboBox_runType->setCurrentText("Full Inspection");
}

void VisionApp::toggleOnlineRun()
{
	QSignalBlocker sb1(ui.toolButton_toggleOfflineRun);
	QSignalBlocker sb2(ui.toolButton_toggleOnlineRun);
	ui.toolButton_toggleOfflineRun->setChecked(false);
	ui.toolButton_toggleOnlineRun->setChecked(true);

	ui.comboBox_runType->clear();
	QStringList runType = {
		"2D Acquisition",
		"3D Acquisition",
		"Full Acquisition",
		"2D Inspection",
		"3D Inspection",
		"Full Inspection",
		"Full Stationary",
		//Profiler bring-up only: a bare 3D acquisition with no views, no 2D and no
		//production error handling. Handled by an exact-match branch in testRun().
		"Profiler Scan Test",
		//Same acquisition production uses, via JobThread::scan(). Exact-match branch too -
		//no "2D"/"3D"/"Full"/"Inspection" substring, or the contains() chain would claim it.
		"Production Scan Check"
	};
	ui.comboBox_runType->addItems(runType);
	ui.comboBox_runType->setCurrentText("Full Inspection");
}

void VisionApp::setVisionObjectAsDefaultTemplate()
{
	QString templateImagesPath = createTemplateImagesDirectory();
	for (int i = 0; i < _dragROI.size(); i++)
	{

		if (_dragROI[i]->isSelected())
		{

			QRectF FOVroi = ScaleManager::instance().world_to_fov(_dragROI[i]->getGeometry());
			QImage cropped = _imageWorld.copy(_dragROI[i]->getGeometry().toRect());
			QImage scaledCropped = cropped.scaled(FOVroi.width(), FOVroi.height());
			QString id = _templateLibraryTab->currentTemplateId();
			QString name = _templateLibraryTab->currentTemplateName();
			QColor color = _templateLibraryTab->currentTemplateColor();
			AlgoTemplate* algoTemplate = _templateLibraryTab->currentAlgoTemplate();
			if (algoTemplate == nullptr)
			{
				QMessageBox::warning(this, tr("Error setting default template."),
					tr("Please create a template in Template Library Tab."));
				return;
			}

			QString imageDir = QStringLiteral("%1/%2_%3/").arg(templateImagesPath).arg(id).arg(name);
			CreateDirectoryA(imageDir.toStdString().c_str(), NULL);
			QString imageFilePath = QStringLiteral("%1img.jpg").arg(imageDir);
			scaledCropped.save(imageFilePath);

			QSize size = QSize(scaledCropped.width(), scaledCropped.height());

			QString imageFileRelativePath = QStringLiteral("/%1_%2/img.jpg").arg(id).arg(name);
			qDebug() << "imageFileRelativePath:" << imageFileRelativePath;
			_templateLibraryTab->setTemplateImagePath(id, imageFileRelativePath, size);

			_dragROI[i]->algoTemplate(algoTemplate);

			_dragROI[i]->setBorderColor(color);

			auto vo = _visionObject.value(_dragROI[i]->getId());

			if (algoTemplate == nullptr)
			{
				qDebug() << "Algograph is nullptr";
			}
			vo.templateID = algoTemplate->templateId();

			vo.templateName = algoTemplate->templateName();

			_visionObject.insert(_dragROI[i]->getId(), vo);

			updateVisionObjectSize(algoTemplate);
			break;

		}

	}

}

void VisionApp::deleteVisionObjectTemplate(const QString & templateId)
{
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->algoTemplate() == nullptr) continue;

		if (_dragROI[i]->algoTemplate()->templateId() == templateId)
		{
			_dragROI[i]->setBorderColor(Qt::white);
			_dragROI[i]->algoTemplate(nullptr);
			_dragROI[i]->update();
		}
	}
}

void VisionApp::updateVisionObjectTemplate(AlgoTemplate* algoTemplate)
{
	if (algoTemplate == nullptr) return;

	bool uniform = algoTemplate->uniformBox();
	uniform = true;
	QSize size(algoTemplate->w(), algoTemplate->h());
	QColor color(algoTemplate->color());

	//check if VisionObject Size is same as template or not
	bool visionObjectResizeFlag = false;
	if (uniform)
	{
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->isSelected())
			{
				QRectF FOVroi = ScaleManager::instance().world_to_fov(_dragROI[i]->getGeometry());
				double w = FOVroi.width();
				double h = FOVroi.height();
				if (w != size.width() || h != size.height())
				{
					QMessageBox::StandardButton reply = QMessageBox::question(this, "Resize Vision Object", "Do you want to resize Vision Object to template size?", QMessageBox::Yes | QMessageBox::No);
					if (reply == QMessageBox::Yes) visionObjectResizeFlag = true;
					else return;
					break;
				}
			}
		}
	}

	bool roiIncluded_into_ViewFlag = false;
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->isSelected())
		{

			if (visionObjectResizeFlag)
			{
				QRectF FOVroi = ScaleManager::instance().world_to_fov(_dragROI[i]->getGeometry());

				double wFOV = FOVroi.width();
				double hFOV = FOVroi.height();

				auto x = _dragROI[i]->getGeometry().x();
				auto y = _dragROI[i]->getGeometry().y();
				auto oldW = _dragROI[i]->getGeometry().width();
				auto oldH = _dragROI[i]->getGeometry().height();

				if (wFOV != size.width() || hFOV != size.height())
				{
					double w = ScaleManager::instance().fov_to_world(size.width());
					double h = ScaleManager::instance().fov_to_world(size.height());
					x = x - (w- oldW)/2;
					y = y - (h -oldH)/2;
					_dragROI[i]->setGeometry(QRectF(x, y, (int)w, (int)h));
				}
			}

			_dragROI[i]->algoTemplate(algoTemplate);
			_dragROI[i]->setBorderColor(color);

			auto vo = _visionObject.value(_dragROI[i]->getId());
			vo.templateID = algoTemplate->templateId();
			vo.templateName = algoTemplate->templateName();
			_visionObject.insert(_dragROI[i]->getId(), vo);

			if (includeVisionObject_into_View(_dragROI[i]), true)
			{
				_dragROI[i]->update();
				roiIncluded_into_ViewFlag = true;
			}

			if (includeVisionObject_into_HeightMap(_dragROI[i])) {
				_dragROI[i]->update();
			}
		}
	}

	if (visionObjectResizeFlag) updateVisionObjectGeometry();
	if (roiIncluded_into_ViewFlag) updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	update();
	updateSetupCheckList();
	saveRecipe();
}

void VisionApp::updateVisionObjectSize(AlgoTemplate * algoTemplate)
{
	if (algoTemplate == nullptr) return;

	bool uniform = algoTemplate->uniformBox();
	uniform = true;
	QSize size(algoTemplate->w(), algoTemplate->h());
	QColor color(algoTemplate->color());
	qDebug() << "updateVisionObjectSIze:" << size;
	//check if VisionObject Size is same as template or not
	bool visionObjectResizeFlag = false;
	if (uniform)
	{
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->algoTemplate() != nullptr)
			{
				if (_dragROI[i]->algoTemplate()->templateId() == algoTemplate->templateId())
				{
					QRectF FOVroi = ScaleManager::instance().world_to_fov(_dragROI[i]->getGeometry());
					double w = FOVroi.width();
					double h = FOVroi.height();
					if (w != size.width() || h != size.height())
					{
						QMessageBox::StandardButton reply = QMessageBox::question(this, "Resize Vision Object", "Do you want to resize Vision Object to template size? If No is clicked then uniformBox setting will not be checked.", QMessageBox::Yes | QMessageBox::No);
						if (reply == QMessageBox::Yes) visionObjectResizeFlag = true;
						else
						{
							_templateLibraryTab->updateUniformBoxFlag(false);
							return;
						}
						break;
					}
				}

			}
		}
	}

	bool roiIncluded_into_ViewFlag = false;
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->algoTemplate())
		{
			if (_dragROI[i]->algoTemplate()->templateId() == algoTemplate->templateId())
			{
				if (visionObjectResizeFlag)
				{
					QRectF FOVroi = ScaleManager::instance().world_to_fov(_dragROI[i]->getGeometry());

					double wFOV = FOVroi.width();
					double hFOV = FOVroi.height();

					auto x = _dragROI[i]->getGeometry().x();
					auto y = _dragROI[i]->getGeometry().y();
					if (wFOV != size.width() || hFOV != size.height())
					{
						int w = ScaleManager::instance().fov_to_world(size.width());
						int h = ScaleManager::instance().fov_to_world(size.height());
						_dragROI[i]->setGeometry(QRectF(x, y, w, h));
					}
				}

				_dragROI[i]->algoTemplate(algoTemplate);
				_dragROI[i]->setBorderColor(color);


				auto vo = _visionObject.value(_dragROI[i]->getId());
				vo.templateID = algoTemplate->templateId();
				vo.templateName = algoTemplate->templateName();
				_visionObject.insert(_dragROI[i]->getId(), vo);

				if (includeVisionObject_into_View(_dragROI[i]), true)
				{
					_dragROI[i]->update();
					roiIncluded_into_ViewFlag = true;
				}

				if (includeVisionObject_into_HeightMap(_dragROI[i]), true)
				{
					_dragROI[i]->update();
				}
			}
		}
	}

	if (visionObjectResizeFlag) updateVisionObjectGeometry();
	if (roiIncluded_into_ViewFlag) updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	update();
	saveRecipe();
}

void VisionApp::updateVisionObjectColor(AlgoTemplate * algoTemplate)
{
	if (algoTemplate == nullptr) return;

	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->algoTemplate() == algoTemplate)
		{
			_dragROI[i]->setBorderColor(algoTemplate->color());
		}
	}
}

//need to improve to crop and generate multiple images
void VisionApp::generateVIDIImages(AlgoTemplate * algoTemplate, bool enablePreprocess)
{
	if (algoTemplate == nullptr) return;

	bool algoTemplateUsed = false;;
	QVector<QString> viewIDs;
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->algoTemplate() == algoTemplate)
		{
			algoTemplateUsed = true;
			QString viewID = _visionObject.value(_dragROI[i]->getId()).viewID;
			if (!viewIDs.contains(viewID)) viewIDs.append(viewID);
		}
	}

	if (algoTemplateUsed)
	{
		uidGenerator idGen;
		QString folderID = idGen.id().c_str();
		QString vidiImageTemplatePath = Common::Directory::getRecipeVidiImagePath() + algoTemplate->templateId() + "\\";
		Common::Directory::createDir(vidiImageTemplatePath);
		QString vidiImagePath = vidiImageTemplatePath + folderID + "\\";
		Common::Directory::createDir(vidiImagePath);
		
		//image preprocess deprecated
		//temp hardcoded		
		QHash<QString, util::ImagePreprocess*> imagePreprocessTools;
		if (false)
		{
			auto optic = _recipeOptics.constBegin();
			while (optic != _recipeOptics.constEnd())
			{
				QString vidiImageOpticPath = vidiImagePath + optic.value().name + "\\";
				Common::Directory::createDir(vidiImageOpticPath);

				if (enablePreprocess)
				{
					if (optic.value().name == "RGB")
					{
						vidiImageOpticPath = vidiImagePath + optic.value().name + "_HighlightDefects" + "\\";
						Common::Directory::createDir(vidiImageOpticPath);
					}
					else if (optic.value().name == "RB")
					{
						vidiImageOpticPath = vidiImagePath + optic.value().name + "_DiffOfMedianFilter" + "\\";
						Common::Directory::createDir(vidiImageOpticPath);
					}
					else if (optic.value().name == "DieLight")
					{
						vidiImageOpticPath = vidiImagePath + optic.value().name + "_DiffOfMedianFilter" + "\\";
						Common::Directory::createDir(vidiImageOpticPath);
					}

					QString refImgPath = Common::Directory::getRecipeVidiImagePath() + algoTemplate->templateId() + "\\ref_" + optic.value().name + g_imgExtension;
					QString maskImgPath = Common::Directory::getRecipeVidiImagePath() + algoTemplate->templateId() + "\\mask" + g_imgExtension;

					if (!QFileInfo::exists(refImgPath))
					{
						QMessageBox::warning(this, QString("No Ref Image"),
							QString("No Reference Image is found, please go to vision app and save the reference image by right clicking the vision Object!!!"));
						return;
					}


					util::ImagePreprocess* imagePreprocess;
					imagePreprocess = new util::ImagePreprocess(refImgPath, maskImgPath);
					imagePreprocessTools.insert(optic.value().name, imagePreprocess);
				}
				optic++;
			}
		}
		//temp hardcoded - end

		int maxValue = viewIDs.size();
		progressBarSetup("Generating VIDI Images...", maxValue);

		for (int i = 0; i < viewIDs.size(); i++)
		{
			//load all Images
			if (!_views.contains(viewIDs[i])) {
				ct::logger::error("[JobThread] Failed to generate VIDI images. Invalid view ID: %s", viewIDs[i].toStdString().c_str());
				continue;
			}

			auto v = _views.value(viewIDs[i]);
			auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), v, _mainOptics[_camID], _recipeOptics);

			QHash<QString, MIL_ID> multiLightingImages;
			QVector<ImageInfo> imageInfos;
			QSize imgSize;

			//populate multiimages
			auto viewPaths = ipf.getAllOpticPaths();
			QHash<QString, QString>::const_iterator viewPath = viewPaths.constBegin();
			while (viewPath != viewPaths.constEnd())
			{
				/*QImage img;
				bool loadSuccess = img.load(viewPath.value());*/
				MIL_ID img = M_NULL;
				MIL_INT sizeX, sizeY, bandSize;
				MbufDiskInquireA(viewPath.value().toStdString().c_str(), M_SIZE_X, &sizeX);
				MbufDiskInquireA(viewPath.value().toStdString().c_str(), M_SIZE_Y, &sizeY);
				MbufDiskInquireA(viewPath.value().toStdString().c_str(), M_SIZE_BAND, &bandSize);

				MbufAllocColor(M_DEFAULT, bandSize, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &img);
				MIL_INT imgType = M_JPEG_LOSSY;
				if (util::isPNG(viewPath.value())) imgType = M_PNG;
				if (util::isBMP(viewPath.value())) imgType = M_BMP;
				MbufImportA(viewPath.value().toStdString().c_str(), imgType, M_LOAD, M_DEFAULT_HOST, &img);
				multiLightingImages.insert(viewPath.key(), img);

				MIL_INT width, height;
				MbufInquire(img, M_SIZE_X, &width);
				MbufInquire(img, M_SIZE_Y, &height);
				imgSize = QSize(width, height);

				ImageInfo imageInfo;
				imageInfo._opticName = _recipeOptics[viewPath.key()].name;
				imageInfo._opticID = _recipeOptics[viewPath.key()].id;
				imageInfo._imageType = _recipeOptics[viewPath.key()].type;
				imageInfo.imgSize = imgSize;
				imageInfos.append(imageInfo);

				viewPath++;
			}

			//populate heightmap
			QHash<QString, MIL_ID> heightImages;
			for (const auto& l : _lineScans) {
				auto path = Common::Directory::getRecipeSetupImagePath() + l.id + ".tiff";
				auto mBuf = MbufRestoreA(path.toStdString().c_str(), M_DEFAULT_HOST, M_NULL);

				heightImages.insert(l.id, mBuf);

				auto w = mtrx::get_width(mBuf);
				auto h = mtrx::get_height(mBuf);

				ImageInfo imageInfo;
				imageInfo._opticName = "HeightMap";
				imageInfo._opticID = "HeightMap";
				imageInfo._imageType = ct::s_height_map;
				imageInfo.imgSize = QSize(w, h);
				imageInfos.append(imageInfo);
			}


			for (int j = 0; j < _dragROI.size(); j++)
			{
				QString viewID = _visionObject.value(_dragROI[j]->getId()).viewID;
				if (viewIDs[i] == viewID)
				{
					QRectF FOVrect = ScaleManager::instance().world_to_fov(_dragROI[j]->getGeometry());
					QPointF fovView = { 0,0 };
					if (g_viewMode == int(ViewMode::PLANE)) fovView = ScaleManager::instance().to_fov_px(v);
					auto x = FOVrect.x() - fovView.x();
					auto y = FOVrect.y() - fovView.y();
					auto w = FOVrect.width();
					auto h = FOVrect.height();

					//locator removed with the Algo library - crop at the drawn geometry
					QPointF offset = { 0,0 };
					{
						auto newX = x + offset.x();
						auto newY = y + offset.y();
						if (newX < 0) newX = 0;
						if (newY < 0) newY = 0;

						if (newX + w > imgSize.width()) newX = newX - ((newX + w) - imgSize.width());
						if (newY + h > imgSize.height()) newY = newY - ((newY + h) - imgSize.height());

						// crop Vision Object from all images
						auto lImage = multiLightingImages.constBegin();
						while (lImage != multiLightingImages.constEnd())
						{

							MIL_ID cropped = M_NULL;
							MIL_INT bandSize = mtrx::get_band(lImage.value());
							cropped = MbufAllocColor(M_DEFAULT, bandSize, w, h, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
							MbufCopyColor2d(lImage.value(), cropped, M_ALL_BANDS, newX, newY, M_ALL_BANDS, 0, 0, w, h);

							QString opticName = _recipeOptics[lImage.key()].name;
							QString imagePath = vidiImagePath + opticName + "//" + _dragROI[j]->getName() + g_imgExtension;

							//temp hardCoded jet
							if (enablePreprocess)
							{
								auto imagePreprocess = imagePreprocessTools[opticName];
								if (opticName == "RB")
								{
									MbufExportA(imagePath.toStdString().c_str(), M_JPEG_LOSSY, cropped);
									imagePath = vidiImagePath + opticName + "_DiffOfMedianFilter" + "//" + _dragROI[j]->getName() + ".jpg";
									imagePreprocess->Diff_of_medianFilter(cropped, cropped);
								}
								else if (opticName == "RGB")
								{
									MbufExportA(imagePath.toStdString().c_str(), M_JPEG_LOSSY, cropped);
									imagePath = vidiImagePath + opticName + "_HighlightDefects" + "//" + _dragROI[j]->getName() + ".jpg";
									imagePreprocess->HighlightDarkDefects(cropped, cropped);
								}
								else if (opticName == "DieLight")
								{
									imagePreprocess->medianFilter(cropped, cropped);
									MbufExportA(imagePath.toStdString().c_str(), M_JPEG_LOSSY, cropped);
									imagePath = vidiImagePath + opticName + "_DiffOfMedianFilter" + "//" + _dragROI[j]->getName() + ".jpg";
									imagePreprocess->Diff_of_medianFilter(cropped, cropped, 8, 255, 0, 3, 15);
								}
							}

							//temp hardcoded - ended
							MbufExportA(imagePath.toStdString().c_str(), M_JPEG_LOSSY, cropped);
							MbufFree(cropped);
							lImage++;
						}
					}
				}
			}

			incrementProgressBar();

			auto lImage = multiLightingImages.constBegin();
			while (lImage != multiLightingImages.constEnd())
			{
				MbufFree(lImage.value());
				lImage++;
			}
			multiLightingImages.clear();
		}
		progressBarRelease();


		auto imageP = imagePreprocessTools.constBegin();
		while (imageP != imagePreprocessTools.constEnd())
		{
			delete imageP.value();
			imageP++;
		}
	}


	return;
}

void VisionApp::addVisionObjectPadding(AlgoTemplate * algoTemplate, int paddingSize)
{
	if (algoTemplate == nullptr) return;

	storeVisionObjectInfo(true);

	bool visionObjectResizeFlag = false;
	double roiWidth = 0;
	double roiHeight = 0;
	//bool roiIncluded_into_ViewFlag = false;
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->algoTemplate() == algoTemplate)
		{
			double cx = _dragROI[i]->getGeometry().center().x();
			double cy = _dragROI[i]->getGeometry().center().y();
			auto w = _dragROI[i]->getGeometry().width() + paddingSize * 2;
			auto h = _dragROI[i]->getGeometry().height() + paddingSize * 2;

			if (roiWidth == 0) roiWidth = w;
			if (roiHeight == 0) roiHeight = h;

			double x = cx - w / 2;
			double y = cy - h / 2;

			_dragROI[i]->setGeometry(QRectF(x, y, w, h));

			/*	if (includeVisionObject_into_View(_dragROI[i]), true)
				{
					_dragROI[i]->update();
					roiIncluded_into_ViewFlag = true;
				}*/

			visionObjectResizeFlag = true;
		}
	}

	/*if (algoTemplate->uniformBox())
	{*/
	auto FOVWidth = ScaleManager::instance().world_to_fov(roiWidth);
	auto FOVHeight = ScaleManager::instance().world_to_fov(roiHeight);

	auto FOVpaddingSize = (FOVWidth - algoTemplate->w()) / 2;

	algoTemplate->w(FOVWidth);
	algoTemplate->h(FOVHeight);

	QString imageFilePath = algoTemplate->templateImagePath();
	imageFilePath = Common::Directory::getRecipeCurrentPath() + "template_Images\\" + algoTemplate->templateImagePath();
	if (algoTemplate->templateImagePath().contains("c:\\Advanced\\Data\\recipe")) imageFilePath = algoTemplate->templateImagePath();
	QImage templateImg;
	if (templateImg.load(imageFilePath))
	{
		QImage paddedImage(algoTemplate->w(), algoTemplate->h(), QImage::Format_RGB32);
		paddedImage.fill(Qt::black); // Fill the image with black color

		QPainter painter(&paddedImage);
		painter.drawImage(QPoint(FOVpaddingSize, FOVpaddingSize), templateImg);
		painter.end();
		paddedImage.save(imageFilePath);
		_templateLibraryTab->setTemplateImagePath(algoTemplate->templateId(), imageFilePath, QSize(algoTemplate->w(), algoTemplate->h()));
	}
	//}


	updateVisionObjectInfo(true);
	if (visionObjectResizeFlag) updateVisionObjectGeometry();
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	update();
	saveRecipe();
}

void VisionApp::saveTemplateReferenceImage(AlgoTemplate * templateAlgoTemplate)
{
	auto msgBox = QMessageBox(QMessageBox::Information, tr("Confirmation"),
		tr("Saving reference image will overwrite existing reference image!!!\nPress Yes to save.\nPress No to cancel."),
		QMessageBox::Yes | QMessageBox::No);
	msgBox.setWindowModality(Qt::NonModal);

	//if no is clicked
	if (QMessageBox::Yes == msgBox.exec())
	{

		//copy existing ref image to the backup folder

		for (int i = 0; i < _dragROI.count(); i++)
		{
			if (_dragROI.at(i)->isSelected() == true)
			{
				auto algoTemplate = _dragROI[i]->algoTemplate();
				if (algoTemplate)
				{
					QString vidiImageTemplatePath = Common::Directory::getRecipeVidiImagePath() + algoTemplate->templateId() + "\\";
					Common::Directory::createDir(vidiImageTemplatePath);

					//create backup folder
					uidGenerator uid;

					QString bkFolderPath = vidiImageTemplatePath + "backupRefImages" + uid.id().c_str() + "\\";
					Common::Directory::createDir(bkFolderPath);

					if (!_views.contains(_dragROI[i]->viewID())) {
						ct::logger::error("[JobThread] Failed to save template reference image. Invalid view ID: %s", _dragROI[i]->viewID().toStdString().c_str());
						return;
					}

					auto v = _views.value(_dragROI[i]->viewID());
					auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), v, _mainOptics[_camID], _recipeOptics);

					for (auto view : _views)
					{
						ct::logger::debug("_views: %s", view.id.toStdString().c_str());
					}
					ct::logger::debug("roiViewID: %s, saveReferenceImageViewID: %s", _dragROI[i]->viewID().toStdString().c_str(), v.id.toStdString().c_str());

					//populate multiImages
					QHash<QString, MIL_ID> multiLightingImages;
					QVector<ImageInfo> imageInfos;
					QSize imgSize;
					
					auto viewPaths = ipf.getAllOpticPaths();
					QHash<QString, QString>::const_iterator viewPath = viewPaths.constBegin();
					while (viewPath != viewPaths.constEnd())
					{
						ct::logger::debug("viewPath: %s", viewPath.value().toStdString().c_str());
						if (!QFileInfo::exists(viewPath.value()))
						{
							QMessageBox::warning(this, tr("View Image Does'nt exist"),
								tr("View Image is missing. Please collect View Images!!!"));
							return;
						}

						MIL_ID img = M_NULL;
						MIL_INT sizeX, sizeY, bandSize;
						MbufDiskInquireA(viewPath.value().toStdString().c_str(), M_SIZE_X, &sizeX);
						MbufDiskInquireA(viewPath.value().toStdString().c_str(), M_SIZE_Y, &sizeY);
						MbufDiskInquireA(viewPath.value().toStdString().c_str(), M_SIZE_BAND, &bandSize);
						ct::logger::debug("sizeX: %d, sizeY: %d, bandSize: %d", sizeX, sizeY, bandSize);
						MbufAllocColor(M_DEFAULT, bandSize, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &img);
						MIL_INT imgType = M_JPEG_LOSSY;
						if (util::isPNG(viewPath.value())) imgType = M_PNG;
						if (util::isBMP(viewPath.value())) imgType = M_BMP;
						MbufImportA(viewPath.value().toStdString().c_str(), imgType, M_LOAD, M_DEFAULT_HOST, &img);
						multiLightingImages.insert(viewPath.key(), img);

						MIL_INT width, height;
						MbufInquire(img, M_SIZE_X, &width);
						MbufInquire(img, M_SIZE_Y, &height);
						imgSize = QSize(width, height);

						ImageInfo imageInfo;
						imageInfo._opticName = _recipeOptics[viewPath.key()].name;
						imageInfo._opticID = _recipeOptics[viewPath.key()].id;
						imageInfo._imageType = _recipeOptics[viewPath.key()].type;
						imageInfo.imgSize = imgSize;
						imageInfos.append(imageInfo);

						viewPath++;
					}

					QRectF FOVrect = ScaleManager::instance().world_to_fov(_dragROI[i]->getGeometry());
					QPointF fovView = { 0,0 }; 
					if (g_viewMode == int(ViewMode::PLANE)) fovView = ScaleManager::instance().to_fov_px(v);
					auto x = FOVrect.x() - fovView.x();
					auto y = FOVrect.y() - fovView.y();
					auto w = FOVrect.width();
					auto h = FOVrect.height();

					//locator removed with the Algo library
					QPointF offset = { 0,0 };
					auto newX = x + offset.x();
					auto newY = y + offset.y();

					if (newX + w > imgSize.width()) newX = newX - ((newX + w) - imgSize.width());
					if (newY + h > imgSize.height()) newY = newY - ((newY + h) - imgSize.height());

					// crop Vision Object from all images
					auto lImage = multiLightingImages.constBegin();
					while (lImage != multiLightingImages.constEnd())
					{
						ct::logger::debug("newX: %f, newY: %f, w: %f, h: %f", newX, newY, w, h);
						MIL_ID cropped = M_NULL;
						MbufChildColor2d(lImage.value(), M_ALL_BANDS, newX, newY, w, h, &cropped);

						QString opticName = _recipeOptics[lImage.key()].name;
						QString imagePath = vidiImageTemplatePath + "ref_" + opticName + g_imgExtension;
						QString bkDestinationImagePath = bkFolderPath + "ref_" + opticName + g_imgExtension;
						if (QFileInfo::exists(imagePath)) MoveFileA(imagePath.toStdString().c_str(), bkDestinationImagePath.toStdString().c_str());
						
						MbufExportA(imagePath.toStdString().c_str(), g_imgType, cropped);

						////temporary for inspection
						QString refImgPath = Common::Directory::getRecipeVidiImagePath() + "ref_" + opticName + g_imgExtension;
						MbufExportA(refImgPath.toStdString().c_str(), g_imgType, cropped);
	
						MbufFree(cropped);
						MbufFree(lImage.value());
						lImage++;
					}

					multiLightingImages.clear();
				}
			}
		}
	}

}

bool VisionApp::referenceImageExistTest()
{
	// need template ID, currentView opticIDs
	bool referenceImageExist = true;
	auto optic = _recipeOptics.constBegin();
	while (optic != _recipeOptics.constEnd())
	{
		QString refImgPath = Common::Directory::getRecipeVidiImagePath() + "ref_" + optic.value().name + g_imgExtension;
		if (!QFileInfo::exists(refImgPath)) referenceImageExist = false;
		optic++;
	}
	return referenceImageExist;
}

void VisionApp::displayCurrentView(QString viewID, QString opticID)
{
	qDebug() << "displayCurrentView:" << viewID;
	ui.comboBox_ImageOptics->clear();
	clearCropGuidingRoi();

	auto unitConfigInfos = _unitConfigTab->getUnifConfigInfos();

	QString imageIndex;
	if (ui.lineEdit_currentImageIndex->text().isEmpty())
	{
		imageIndex = _unitConfigTab->getFirstID(viewID);
		ui.lineEdit_currentImageIndex->setText(imageIndex);
	}
	else imageIndex = ui.lineEdit_currentImageIndex->text();

	bool imageLoaded = false;
	for (const auto& v : _views)
	{
		if (v.id == viewID)
		{
			ui.label_curViewName->setText(v.name);
			ui.label_curViewName->setWhatsThis(v.id);

			ui.lineEdit_zstack_selectedView->setText(viewID);
			updateViewEditorSettingUI(viewID);

			for (const auto& o : v.opticIDs)
			{
				for (const auto& ro : _recipeOptics)
				{
					if (ro.id == o)
					{
						ui.comboBox_ImageOptics->addItem(ro.name, o);
						break;
					}
				}			
			}

			imageIndex = "R0C0";
			for (const auto& o : v.opticIDs)
			{
				if (opticID.isEmpty()) opticID = o;

				if (o == opticID)
				{
					QString imagePath = Common::Directory::CurrentImageSetPath + "\\" + viewID + "_" + o + "_" + imageIndex + ".jpg";
					qDebug() << "imagePath:" << imagePath;
					if (_imageWorld.load(imagePath))
					{
						ui.toolButton_toggleWorldView->animateClick();
						displayImage(_imageWorld);
						imageLoaded = true;
						break;
					}
				}
				
			}

		}
	}
	qDebug() << "imageLoaded:" << imageLoaded;
	if (!imageLoaded)
	{
		auto w = CAMManager::instance().getWidth(_camID);
		auto h = CAMManager::instance().getWidth(_camID);
		_imageWorld = QImage(w, h, QImage::Format_RGB32);
		_imageWorld.fill(Qt::black);
		ui.toolButton_toggleWorldView->animateClick();
		displayImage(_imageWorld);
	}

	showVisionObject(ui.toolButton_showVisionObject->isChecked());
	showDefectRect(true);
}

void VisionApp::displayCurrentView()
{
	qDebug() << "displayCurrentView";
	bool imageLoaded = false;
	for (const auto& v : _views)
	{
		if (v.id == ui.label_curViewName->whatsThis())
		{
			auto unitConfigInfos = _unitConfigTab->getUnifConfigInfos();

			QString imageIndex;
			if (ui.lineEdit_currentImageIndex->text().isEmpty())
			{
				imageIndex = _unitConfigTab->getFirstID(ui.label_curViewName->whatsThis());
				ui.lineEdit_currentImageIndex->setText(imageIndex);
			}
			else imageIndex = ui.lineEdit_currentImageIndex->text();

			for (const auto& o : v.opticIDs)
			{
				if (o == ui.comboBox_ImageOptics->currentData().toString())
				{				
					QString imagePath = Common::Directory::CurrentImageSetPath + "\\" + v.id + "_" + o + "_" + imageIndex + ".jpg";
					qDebug() << "imagePath:" << imagePath;
					if (_imageWorld.load(imagePath))
					{
						ui.toolButton_toggleWorldView->animateClick();
						displayImage(_imageWorld);
						imageLoaded = true;
						break;
					}
				}

			}

		}
	}

	qDebug() << "imageLoaded:" << imageLoaded;
	if (!imageLoaded)
	{
		auto w = CAMManager::instance().getWidth(_camID);
		auto h = CAMManager::instance().getWidth(_camID);
		_imageWorld = QImage(w, h, QImage::Format_RGB32);
		_imageWorld.fill(Qt::black);
		ui.toolButton_toggleWorldView->animateClick();
		displayImage(_imageWorld);
	}

	showVisionObject(ui.toolButton_showVisionObject->isChecked());
	showDefectRect(true);
}

void VisionApp::updateTreeViewExplorer(QString & recipeName, QHash<QString, QView> views, QHash<QString, QVisionObject> visionObjects, QVector<ct::DefectResult> defectResults)
{
	_recipeModel.clear();
	_objectModel.clear();
	ui.listWidget_unassignedVisionObject->clear();

	clearEmptyViewKey();

	//setup RecipeTree
	QStringList header;
	header.append(QStringLiteral(""));
	_recipeModel.setHorizontalHeaderLabels(header);

	QStandardItem *parentItem = _recipeModel.invisibleRootItem();
	parentItem->setColumnCount(2);

	Common::Directory::CurrentRecipe = recipeName;
	_pRecipeItem = new QStandardItem(recipeName); //wont have leak, qt holds ownership
	_pRecipeItem->setEditable(false);
	_pRecipeItem->setColumnCount(2);

	parentItem->appendRow(_pRecipeItem);

	//setup viewTree
	QHash<QString, QVector<QString>> viewMap;
	QHash<QString, QVisionObject>::const_iterator i = visionObjects.constBegin();
	while (i != visionObjects.constEnd())
	{
		if (i.value().viewID == "") {
			ui.listWidget_unassignedVisionObject->addItem(i.value().objectName);
			ui.listWidget_unassignedVisionObject->item(ui.listWidget_unassignedVisionObject->count() - 1)->setWhatsThis(i.value().objectID);

		}
		else {
			if (viewMap.contains(i.value().viewID)) {
				viewMap[i.value().viewID].push_back(i.value().objectID);
			}
			else {
				viewMap.insert(i.value().viewID, QVector<QString>());
				viewMap[i.value().viewID].push_back(i.value().objectID);
			}
		}
		++i;
	}

	verifyUnassignedVisionObject();

	ui.listWidget_viewSelection->clear();

	for (auto v : views) {

		QListWidgetItem * list_item = new QListWidgetItem; //wont have leak, qt holds ownership
		list_item->setText(v.name);
		list_item->setWhatsThis(v.id);
		list_item->setFlags(list_item->flags() | Qt::ItemIsUserCheckable);
		ui.listWidget_viewSelection->addItem(list_item);

		QStandardItem *item = new QStandardItem(v.name);
		item->setWhatsThis(v.id);
		item->setIcon(*_viewIcon);
		item->setColumnCount(2);

		auto vos = viewMap[v.id];

		for (auto vo : vos) {
			auto vo_name = _visionObject.find(vo).value().objectName;
			QStandardItem *child = new QStandardItem(vo_name);
			child->setWhatsThis(vo);
			child->setIcon(*_objectIcon);
			child->setEditable(false);

			int defectCount = 0;
			for (int i = 0; i < defectResults.size(); i++)
			{
				if (defectResults[i].algoDefResult.vo_id == vo.toStdString())
				{
					std::string defectName = defectResults[i].algoDefResult.def_name + "_" + defectResults[i].index;
					if (defectResults[i].index.empty()) defectName = defectResults[i].algoDefResult.def_name;
					QStandardItem * defect = new QStandardItem(defectName.c_str());
					defect->setWhatsThis(defectResults[i].algoDefResult.def_id.c_str());
					child->appendRow(defect);
					defectCount++;
				}
			}

			QStandardItem *child1 = new QStandardItem("");
			if (defectCount > 0) child1->setIcon(*_failIcon);
			else if (defectCount == 0) child1->setIcon(*_passIcon);

			QList<QStandardItem*> row;
			row << child << child1;
			item->appendRow(row);
		}

		item->setEditable(false);
		_pRecipeItem->appendRow(item);
	}

	ui.listWidget_viewSelection->sortItems();

	ui.treeViewRecipeExplorer->expandAll();
	ui.treeViewRecipeExplorer->resizeColumnToContents(0);
	ui.treeViewRecipeExplorer->resizeColumnToContents(1);
	updateSetupCheckList();
}

bool VisionApp::readUserInfo(QJsonObject& userObj)
{
	bool flag = true;

	QString fileName = QStringLiteral("%1config//user.json").arg(Common::Directory::LocalPath);

	if (QFile::exists(fileName) == true)
	{
		QString val;
		QFile file;

		file.setFileName(fileName);

		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			return false;
		}

		val = file.readAll();
		file.close();

		QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
		userObj = doc.object();
	}
	else
	{
		flag = false;
	}

	return flag;
}

bool VisionApp::readSystemInfo(QJsonObject& systemObj)
{
	bool flag = true;

	QString fileName = QStringLiteral("%1config//system.json").arg(Common::Directory::LocalPath);

	if (QFile::exists(fileName) == true)
	{
		QString val;
		QFile file;

		file.setFileName(fileName);
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			return false;
		}

		val = file.readAll();
		file.close();

		QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
		systemObj = doc.object();

		if (systemObj.contains("Enable_Fiducial")) {
			auto toggle = jsonHelper::getBool(_systemObj, "Enable_Fiducial", true);
			if (toggle) ui.checkBox_enableFiducial->setCheckState(Qt::Checked);
			else ui.checkBox_enableFiducial->setCheckState(Qt::Unchecked);
			_useFiducial = toggle;
			_jobThread.enableFiducial(_useFiducial);
		}
		else {
			systemObj.insert("Enable_Fiducial", true);
			_useFiducial = true;
		}
		nvs::set_background_color(ui.toolButton_enableFiducial, _useFiducial ? Qt::green : Qt::red);


		if (systemObj.contains("Enable_Classification_Data_Collection")) {
			auto toggle = jsonHelper::getBool(_systemObj, "Enable_Classification_Data_Collection", false);
			_enableClassificationDataCollection = toggle;
		}
		else
		{
			systemObj.insert("Enable_Classification_Data_Collection", false);
		}

		if (!systemObj.contains("LSC_IP_Address")) {
			systemObj.insert("LSC_IP_Address", "192.168.11.20");
		}

		if (!systemObj.contains("Laser_IP_Address")) {
			systemObj.insert("Laser_IP_Address", "192.168.1.10");
		}

		if (!systemObj.contains("Machine_Share_Folder_Path")) {
			systemObj.insert("Machine_Share_Folder_Path", "C:/Advanced/Data/");
		}
		if (!systemObj.contains("Emap_Fail_Alarm_Percentage")) {
			systemObj.insert("Emap_Fail_Alarm_Percentage", 50);
		}
		// -- 
		if (!systemObj.contains("Emap_Path")) {
			systemObj.insert("Emap_Path", "C:/Advanced/Data/SampleEmap.dat");
		}
		if (!systemObj.contains("Verification_Station_Ip_Address")) {
			systemObj.insert("Verification_Station_Ip_Address", "C:/");
			ui.radioButton_rvLocalPC->setChecked(true);
			ui.label_89->hide();
			ui.lineEdit_rvIpAddress->hide();
			ui.toolButton_rvConnect->hide();
		}
		else
		{
			QString rvStationPath = jsonHelper::getString(_systemObj, "Verification_Station_Ip_Address", "C:/");
			if (rvStationPath == "C:/")
			{
				ui.radioButton_rvLocalPC->setChecked(true);
				ui.label_89->hide();
				ui.lineEdit_rvIpAddress->hide();
				ui.toolButton_rvConnect->hide();
			}
			else
			{
				ui.radioButton_rvOfflineStation->setChecked(true);
				if (rvStationPath.contains("//")) rvStationPath.remove("//");
				ui.lineEdit_rvIpAddress->setText(rvStationPath);
				ui.label_89->show();
				ui.lineEdit_rvIpAddress->show();
				ui.toolButton_rvConnect->show();
			}
		}
		// -- 
		if (!systemObj.contains("Emap_Mode")) {
			systemObj.insert("Emap_Mode", EmapMode::AUTO);
			ui.radioButton_emapAuto->setChecked(true);
		}
		else
		{
			_emapLocalSetting.mode = static_cast<EmapMode>(jsonHelper::getInteger(_systemObj, "Emap_Mode", EmapMode::AUTO));
			if (_emapLocalSetting.mode == EmapMode::AUTO)
			{
				ui.radioButton_emapAuto->setChecked(true);
			}
			else if (_emapLocalSetting.mode == EmapMode::CSV01)
			{
				ui.radioButton_csv01Emap->setChecked(true);
			}
			else if (_emapLocalSetting.mode == EmapMode::CSV34)
			{
				ui.radioButton_csv34Emap->setChecked(true);
			}
			else if (_emapLocalSetting.mode == EmapMode::TEXT_FILE)
			{
				ui.radioButton_textFileEmap->setChecked(true);
			}

		}
		if (!systemObj.contains("Emap_Top_Insp")) {
			systemObj.insert("Emap_Top_Insp", EmapType::TEXT_FILE_EMAP);
		}
		else
		{
			_emapLocalSetting.topInspEmap = static_cast<EmapType>(jsonHelper::getInteger(_systemObj, "Emap_Top_Insp", EmapType::TEXT_FILE_EMAP));
		}
		if (!systemObj.contains("Emap_Bot_Insp")) {
			systemObj.insert("Emap_Bot_Insp", EmapType::TEXT_FILE_EMAP);
		}
		else
		{
			_emapLocalSetting.botInspEmap = static_cast<EmapType>(jsonHelper::getInteger(_systemObj, "Emap_Bot_Insp", EmapType::TEXT_FILE_EMAP));
		}


		if (!systemObj.contains("Emap_Csv_Dir")) {
			QJsonArray a;
			systemObj.insert("Emap_Csv_Dir", a);
		}
		else
		{
			_emapLocalSetting.csvEmapDir.clear();
			QJsonArray arrayDefault;
			QJsonArray pathArray = jsonHelper::getArray(_systemObj, "Emap_Csv_Dir", arrayDefault);
			for (auto p : pathArray)
			{
				_emapLocalSetting.csvEmapDir.append(p.toString());
			}
			ui.listWidget_csvEmapDir->clear();
			ui.listWidget_csvEmapDir->addItems(_emapLocalSetting.csvEmapDir);

		}
		if (!systemObj.contains("Emap_Text_File_Dir")) {
			QJsonArray a;
			systemObj.insert("Emap_Text_File_Dir", a);
		}
		else
		{
			_emapLocalSetting.textFileEmapDir.clear();
			QJsonArray arrayDefault;
			QJsonArray pathArray = jsonHelper::getArray(_systemObj, "Emap_Text_File_Dir", arrayDefault);
			for (auto p : pathArray)
			{
				_emapLocalSetting.textFileEmapDir.append(p.toString());
			}
			ui.listWidget_txtEmapDir->clear();
			ui.listWidget_txtEmapDir->addItems(_emapLocalSetting.textFileEmapDir);
		}

		if (!systemObj.contains("Save_Defect_Vo_Image")) {
			systemObj.insert("Save_Defect_Vo_Image", true);
		}
		else
		{
			_saveDefectVoImg = jsonHelper::getBool(_systemObj, "Save_Defect_Vo_Image", true);
				ui.checkBox_saveDefectVoImg->setChecked(_saveDefectVoImg);
		}

		if (!systemObj.contains("Save_Defect_Rect_Vo_Image")) {
			systemObj.insert("Save_Defect_Rect_Vo_Image", true);
		}
		else
		{
			_saveDefectRectVoImg = jsonHelper::getBool(_systemObj, "Save_Defect_Rect_Vo_Image", true);
				ui.checkBox_saveDefectRectVoImg->setChecked(_saveDefectRectVoImg);
		}

		if (!systemObj.contains("Save_Inspection_Image")) {
			systemObj.insert("Save_Inspection_Image", true);
			_saveInspImg = true;
		}
		else
		{
			_saveInspImg = jsonHelper::getBool(_systemObj, "Save_Inspection_Image", true);
			ui.checkBox_EnableSaveInspectionImage->setChecked(_saveInspImg);
		}
		SystemData::instance()._saveInspImages = _saveInspImg; //worker-thread mirror (reader image saving)
		nvs::set_background_color(ui.toolButton_enableSaveInspImages, _saveInspImg ? Qt::green : Qt::red);


		if (!systemObj.contains("Emap_Template_Name")) {
			systemObj.insert("Emap_Template_Name", "");
		}
		else
		{
			_emapTemplate = jsonHelper::getString(_systemObj, "Emap_Template_Name", "");
			ui.comboBox_emapTemplate->setCurrentIndex(ui.comboBox_emapTemplate->findText(_emapTemplate));
		}

		if (!systemObj.contains("Enable_Emap")) {
			systemObj.insert("Enable_Emap", true);
		}
		else
		{
			_enableEmap = jsonHelper::getBool(_systemObj, "Enable_Emap", true);
			ui.checkBox_enableEmap->setChecked(_enableEmap);
		}

		if (!systemObj.contains("Enable_Emap_Template")) {
			systemObj.insert("Enable_Emap_Template", false);
		}
		else
		{
			_isUseEmapTemplate = jsonHelper::getBool(_systemObj, "Enable_Emap_Template", true);
		/*	if (_isUseEmapTemplate) ui.radioButton_useEmapTemplate->setChecked(true);
			else ui.radioButton_useLocalEmapSetting->setChecked(true);*/
		}
		
		if (!systemObj.contains("Storage_Limit")) {
			systemObj.insert("Storage_Limit", 90);
		}
		else
		{
			_storageLimit = jsonHelper::getInteger(_systemObj, "Storage_Limit", 90);
			ui.spinBox_storageLimit->setValue(_storageLimit);
		}
		if (!systemObj.contains("Auto_Delete_Production_File")) {
			systemObj.insert("Auto_Delete_Production_File", false);
		}
		else
		{
			_autoDeleteProductionFile = jsonHelper::getBool(_systemObj, "Auto_Delete_Production_File", false);
			ui.checkBox_autoDeleteProductionFile->setChecked(_autoDeleteProductionFile);
		}
		if (!systemObj.contains("Clearing_Path_List")) {
			QJsonArray clearingPathArray;
			systemObj.insert("Clearing_Path_List", clearingPathArray);
		}
		else
		{
			QJsonArray clearingPathArrayDefault;
			QJsonArray clearingPathArray = jsonHelper::getArray(_systemObj, "Clearing_Path_List", clearingPathArrayDefault);
			for (auto cp : clearingPathArray)
			{
				_clearingPathList.append(cp.toString());
			}
		}
		if (!systemObj.contains("Enable_RMS_Recipe")) {
			systemObj.insert("Enable_RMS_Recipe", false);
		}
		else
		{
			_enableRmsRecipe = jsonHelper::getBool(_systemObj, "Enable_RMS_Recipe", false);
			ui.checkBox_enableRmsRecipe->setChecked(_enableRmsRecipe);
		}
		if (!systemObj.contains("Enable_Mounter_Checking")) {
			systemObj.insert("Enable_Mounter_Checking", false);
		}
		else
		{
			_enableMounterChecking = jsonHelper::getBool(_systemObj, "Enable_Mounter_Checking", false);
			ui.checkBox_enableMounterChecking->setChecked(_enableMounterChecking);
		}
		if (!systemObj.contains("Enable_Golden_Recipe_Checking")) {
			systemObj.insert("Enable_Golden_Recipe_Checking", false);
		}
		else
		{
			_enableGoldenRecipeChecking = jsonHelper::getBool(_systemObj, "Enable_Golden_Recipe_Checking", false);
			ui.checkBox_enableGoldenRecipeChecking->setChecked(_enableGoldenRecipeChecking);
		}

		if (systemObj.contains("Camera")) {
			auto camObj = _systemObj["Camera"].toObject();
			CAMManager::instance().setDefaultWidth(jsonHelper::getInteger(camObj, "Width", 5120));
			CAMManager::instance().setDefaultHeight(jsonHelper::getInteger(camObj, "Height", 5120));
			CAMManager::instance().setDefaultChannel(jsonHelper::getInteger(camObj, "Channel", 3));
		}

		if (systemObj.contains("Camera_Angle")) {
			auto ca = jsonHelper::getDouble(_systemObj, "Camera_Angle", 0.0);
			_prevCamAlignedAngle = ca;
			setCameraAngle(ca);
		}
		else {
			_prevCamAlignedAngle = 0.0;
			setCameraAngle(0.0);
		}

		if (systemObj.contains("LSC_Trigger_Mode")) {
			SystemData::instance()._lscTriggerMode = jsonHelper::getInteger(systemObj, "LSC_Trigger_Mode", 1);
		}

		if (!systemObj.contains("Use_Recipe_Scale")) {
			systemObj.insert("Use_Recipe_Scale", false);
		}
		else
		{
			SystemData::instance()._useRecipeScale = jsonHelper::getBool(_systemObj, "Use_Recipe_Scale", false);
			ui.checkBox_useRecipeScaling->setChecked(SystemData::instance()._useRecipeScale);
		}

		if (!systemObj.contains("Used_As_Recipe1")) {
			systemObj.insert("Used_As_Recipe1", false);
		}
		if (!systemObj.contains("Used_As_Recipe2")) {
			systemObj.insert("Used_As_Recipe2", false);
		}

		bool usedAsRecipe1 = jsonHelper::getBool(systemObj, "Used_As_Recipe1", false);
		bool usedAsRecipe2 = jsonHelper::getBool(systemObj, "Used_As_Recipe2", false);
		if (usedAsRecipe1 && usedAsRecipe2) {
			usedAsRecipe2 = false;
			systemObj.insert("Used_As_Recipe2", false);
		}
		ui.checkBox_usedAsRecipe1->setChecked(usedAsRecipe1);
		ui.checkBox_usedAsRecipe2->setChecked(usedAsRecipe2);

		if (!systemObj.contains("PSP")) {
			systemObj.insert("PSP", false);
		}
		else
		{
			SystemData::instance()._psp = jsonHelper::getBool(_systemObj, "PSP", false);
			ui.checkBox_psp->setChecked(SystemData::instance()._psp);
		}

		if (!systemObj.contains("Machine_Debug_Mode")) {
			systemObj.insert("Machine_Debug_Mode", false);
		}
		else
		{
			SystemData::instance()._machineDebugMode = jsonHelper::getBool(_systemObj, "Machine_Debug_Mode", false);
			ui.checkBox_machineDebugMode->setChecked(SystemData::instance()._machineDebugMode);
		}

		if (!systemObj.contains("Save_Unstacked_Images")) {
			systemObj.insert("Save_Unstacked_Images", false);
		}
		else
		{
			SystemData::instance()._saveUnstackedImages = jsonHelper::getBool(_systemObj, "Save_Unstacked_Images", false);
			ui.checkBox_saveUnstackedImages->setChecked(SystemData::instance()._saveUnstackedImages);
		}

		if (!systemObj.contains("Save_Unstitched_Images")) {
			systemObj.insert("Save_Unstitched_Images", false);
		}
		else
		{
			SystemData::instance()._saveUnstitchedImages = jsonHelper::getBool(_systemObj, "Save_Unstitched_Images", false);
			ui.checkBox_saveUnstitchedImages->setChecked(SystemData::instance()._saveUnstitchedImages);
		}

		if (!systemObj.contains("Stitching_Method")) {
			systemObj.insert("Stitching_Method", 3);
		}
		else
		{
			/*SystemData::instance()._stitchingMethod = jsonHelper::getInteger(_systemObj, "Stitching_Method", 3);*/
			//Use recipeConfig
		}

		if (!systemObj.contains("Max_Allowable_RAM_Usage")) {
			systemObj.insert("Max_Allowable_RAM_Usage", 80.0);
		}
		else
		{
			SystemData::instance()._maxAllowRamUsage = jsonHelper::getInteger(_systemObj, "Max_Allowable_RAM_Usage", 80.0);
		}

		if (!systemObj.contains("Snap_Delay(ms)")) {
			systemObj.insert("Snap_Delay(ms)", 0);
		}
		else
		{
			SystemData::instance()._snapDelay_ms = jsonHelper::getInteger(_systemObj, "Snap_Delay(ms)", 0);
		}

		if (!systemObj.contains("enable_multi_thread")) {
			systemObj.insert("enable_multi_thread", true);
		}
		else
		{
			SystemData::instance()._enable_multi_thread = jsonHelper::getBool(_systemObj, "enable_multi_thread", true);
		}

		if (!systemObj.contains("num_threads")) {
			systemObj.insert("num_threads", 4);
		}
		else
		{
			SystemData::instance()._num_threads = jsonHelper::getInteger(_systemObj, "num_thread", 4);
		}

		if (!systemObj.contains("Recent_Open_Recipe")) {
			systemObj.insert("Recent_Open_Recipe", "");
		}
		else
		{
			Common::Directory::CurrentRecipe = jsonHelper::getString(_systemObj, QStringLiteral("Recent_Open_Recipe"));
		}
		
		if (!systemObj.contains("Load_Production_After_Run")) {
			systemObj.insert("Load_Production_After_Run", true);
		}
		else
		{
			SystemData::instance()._loadProductionAfterRun = jsonHelper::getBool(_systemObj, "Load_Production_After_Run", true);
		}

		SystemData::instance()._bypassInspection = systemObj.contains("Bypass_Inspection_Mode")
			? jsonHelper::getBool(_systemObj, "Bypass_Inspection_Mode", false)
			: (systemObj.insert("Bypass_Inspection_Mode", false), false);
		ui.checkBox_bypassInspectionMode->setChecked(SystemData::instance()._bypassInspection);

		SystemData::instance()._enableFiducialRotate = systemObj.contains("Enable_Fiducial_Rotate")
			? jsonHelper::getBool(_systemObj, "Enable_Fiducial_Rotate", true)
			: (systemObj.insert("Enable_Fiducial_Rotate", true), true);
		ui.checkBox_enableFiducialRotate->setChecked(SystemData::instance()._enableFiducialRotate);

		Common::Directory::ProductionDrive = systemObj.contains("Setup_Production_Drive")
			? jsonHelper::getString(_systemObj, "Setup_Production_Drive", "C:\\")
			: (systemObj.insert("Setup_Production_Drive", "C:\\"), "C:\\");
		ct::logger::warn(QStringLiteral("ProductionPath: %1").arg(Common::Directory::ProductionPath()).toStdString().c_str());

		Common::Directory::LearningImageDrive = systemObj.contains("Setup_Learning_Image_Drive")
			? jsonHelper::getString(_systemObj, "Setup_Learning_Image_Drive", "C:\\")
			: (systemObj.insert("Setup_Learning_Image_Drive", "C:\\"), "C:\\");
		ct::logger::warn(QStringLiteral("LearningImagePath: %1").arg(Common::Directory::LearningImagePath()).toStdString().c_str());


	}
	else
	{
		flag = false;
	}

	return flag;
}

bool VisionApp::updateSystemInfo(const QJsonObject& systemObj)
{
	bool flag;

	QString fileName = QStringLiteral("%1config/system.json").arg(Common::Directory::LocalPath);

	// Diff against what's currently on disk so the audit log records WHICH fields changed.
	QJsonObject prev;
	{
		QFile old(fileName);
		if (old.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			prev = QJsonDocument::fromJson(old.readAll()).object();
			old.close();
		}
	}

	auto valStr = [](const QJsonValue& v) -> QString {
		if (v.isObject() || v.isArray()) return QStringLiteral("[...]");
		return v.toVariant().toString();
	};

	QStringList changes;
	for (auto it = systemObj.begin(); it != systemObj.end(); ++it)
	{
		if (!prev.contains(it.key()))
			changes << QStringLiteral("%1=%2 (new)").arg(it.key(), valStr(it.value()));
		else if (prev.value(it.key()) != it.value())
			changes << QStringLiteral("%1: %2->%3").arg(it.key(), valStr(prev.value(it.key())), valStr(it.value()));
	}
	for (auto it = prev.begin(); it != prev.end(); ++it)
		if (!systemObj.contains(it.key()))
			changes << QStringLiteral("%1 (removed)").arg(it.key());

	flag = saveJson(fileName, QJsonDocument(systemObj));

	// Only log when something actually changed (avoids noise from redundant re-saves).
	if (!changes.isEmpty())
	{
		QString detail = changes.size() > 8
			? QStringLiteral("%1 fields: %2, ...").arg(changes.size()).arg(QStringList(changes.mid(0, 8)).join(", "))
			: changes.join(", ");
		AuditLog::instance().log(QStringLiteral("SYSTEM_CONFIG_CHANGE"), detail, flag ? QStringLiteral("OK") : QStringLiteral("FAILED"));
	}

	return flag;
}

bool VisionApp::readBufferInfo(QJsonObject& bufferInfoObj)
{
	bool flag = true;

	QString fileName = QStringLiteral("%1config/buffer.json").arg(Common::Directory::LocalPath);

	if (QFile::exists(fileName) == true)
	{
		QString val;
		QFile file;

		file.setFileName(fileName);

		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			return false;
		}

		val = file.readAll();
		file.close();

		QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
		QJsonObject root = doc.object();

		bufferInfoObj = root[QStringLiteral("Buffer_Info")].toObject();
	}
	else
	{
		flag = false;
	}

	return flag;
}

bool VisionApp::createBufferInfo(QJsonObject & bufferInfoObj, QSize & imgSize)
{
	bufferInfoObj = QJsonObject();

	bool flag = false;
	QJsonObject bufferObj;
	QList<QVariant> bufferObjList;

	if (!imgSize.isEmpty())
	{
		_imageSize = imgSize;
		bufferObj.insert(QStringLiteral("Buffer_Name"), QStringLiteral("Red"));
		bufferObj.insert(QStringLiteral("Buffer_Width"), _imageSize.width());
		bufferObj.insert(QStringLiteral("Buffer_Height"), _imageSize.height());
		bufferObj.insert(QStringLiteral("Camera"), QStringLiteral("Default"));
		bufferObj.insert(QStringLiteral("Micrometer_Per_Pixel"), 1);
		bufferObjList.append(bufferObj);

		bufferObj.insert(QStringLiteral("Buffer_Name"), QStringLiteral("Green"));
		bufferObj.insert(QStringLiteral("Buffer_Width"), _imageSize.width());
		bufferObj.insert(QStringLiteral("Buffer_Height"), _imageSize.height());
		bufferObj.insert(QStringLiteral("Camera"), QStringLiteral("Default"));
		bufferObj.insert(QStringLiteral("Micrometer_Per_Pixel"), 1);
		bufferObjList.append(bufferObj);

		bufferObj.insert(QStringLiteral("Buffer_Name"), QStringLiteral("Blue"));
		bufferObj.insert(QStringLiteral("Buffer_Width"), _imageSize.width());
		bufferObj.insert(QStringLiteral("Buffer_Height"), _imageSize.height());
		bufferObj.insert(QStringLiteral("Camera"), QStringLiteral("Default"));
		bufferObj.insert(QStringLiteral("Micrometer_Per_Pixel"), 1);
		bufferObjList.append(bufferObj);

		bufferInfoObj.insert(QString("Buffer"), QJsonArray::fromVariantList(bufferObjList));

		bufferInfoObj.insert(QStringLiteral("R_Buffer"), QStringLiteral("Red"));
		bufferInfoObj.insert(QStringLiteral("G_Buffer"), QStringLiteral("Green"));
		bufferInfoObj.insert(QStringLiteral("B_Buffer"), QStringLiteral("Blue"));
		bufferInfoObj.insert(QStringLiteral("Unit_Measurement"), QStringLiteral("px"));

		flag = true;
	}

	return flag;
}

void VisionApp::assignDisplayBuffer(const QJsonObject& bufferInfoObj)
{
	_displayBuffer.clear();

	_displayBuffer.insert(QStringLiteral("R_Buffer"), jsonHelper::getString(bufferInfoObj, QStringLiteral("R_Buffer")));
	_displayBuffer.insert(QStringLiteral("G_Buffer"), jsonHelper::getString(bufferInfoObj, QStringLiteral("G_Buffer")));
	_displayBuffer.insert(QStringLiteral("B_Buffer"), jsonHelper::getString(bufferInfoObj, QStringLiteral("B_Buffer")));
}

void VisionApp::formImage(QImage& img, int w, int h, const unsigned char* pRedBuf, const unsigned char* pGreenBuf, const unsigned char* pBlueBuf, const unsigned char* pAlphaBuf)
{
	ct::logger::trace("Start - Form image");
	QRgb *pBuf;
	int offset = 0;

	if (img.format() != QImage::Format_RGB32 || img.width() != w || img.height() != h) {
		img = QImage(w, h, QImage::Format_RGB32);
	}

	if (!pAlphaBuf)
	{
		for (int i = 0; i < img.height(); ++i)
		{
			pBuf = reinterpret_cast<QRgb *>(img.scanLine(i));
			offset = i * img.width();

			for (int j = 0; j < img.width(); ++j)
			{
				pBuf[j] = qRgb(pRedBuf[offset + j], pGreenBuf[offset + j], pBlueBuf[offset + j]);
			}
		}
	}
	else
	{
		for (int i = 0; i < img.height(); ++i)
		{
			pBuf = reinterpret_cast<QRgb *>(img.scanLine(i));
			offset = i * img.width();

			for (int j = 0; j < img.width(); ++j)
			{
				pBuf[j] = qRgba(pRedBuf[offset + j], pGreenBuf[offset + j], pBlueBuf[offset + j], pAlphaBuf[offset + j]);
			}
		}
	}
	ct::logger::trace("End - Form image");
}

void VisionApp::loadImage2Mem(const QJsonObject& bufferInfoObj)
{
	QRgb *pBuf;
	QString name;
	int milMemOffset;
	int sharedMemOffset;
	BufferInfo bufInfo;
	QJsonObject bufferObj;
	unsigned char* pSharedMem;
	unsigned char pixelValue;

	QJsonArray bufferList = jsonHelper::getArray(bufferInfoObj, QStringLiteral("Buffer"));
	for (int i = 0; i < bufferList.count(); i++)
	{
		bufferObj = bufferList[i].toObject();
		name = jsonHelper::getString(bufferObj, QStringLiteral("Buffer_Name"));

		_algo.bufferInfo(name, bufInfo);
		pSharedMem = reinterpret_cast<unsigned char*>(_appSharedMem.getMemory(name.toStdString()));

		for (int i = 0; i < bufInfo.height; ++i)
		{
			pBuf = reinterpret_cast<QRgb *>(_imageMain.scanLine(i));

			milMemOffset = i * bufInfo.pitch;
			sharedMemOffset = i * bufInfo.width;

			for (int j = 0; j < bufInfo.width; ++j)
			{
				if (name == QStringLiteral("Red"))
				{
					pixelValue = qRed(pBuf[j]);
				}
				else if (name == QStringLiteral("Green"))
				{
					pixelValue = qGreen(pBuf[j]);
				}
				else if (name == QStringLiteral("blue"))
				{
					pixelValue = qBlue(pBuf[j]);
				}

				// copy image to shared memory and algo buffer
				bufInfo.pBuf[j + milMemOffset] = pSharedMem[j + sharedMemOffset] = pixelValue;
			}
		}
	}
}

void VisionApp::updateMainDisp()
{
	_pPixmapItemMain->setPixmap(_pixmapMain);
}

void VisionApp::promptLoadImageType()
{
	QStringList items;
	items << "Image Set" << "FOV" << "World View" << "Plane" << "HeatMap";
	auto type = promptComboBox(items, "Load Image", "Loading Type");

	if (type == "Image Set")
	{
		QString imageSetPath = QFileDialog::getExistingDirectory(nullptr, "Select Folder", Common::Directory::getRecipeImagesPath());
		loadImageSet(imageSetPath);
	}
	else if (type == "FOV") loadFOV();
	else if (type == "World View") loadWorldView();
	else if (type == "Plane") loadPlaneView(false);
	else if (type == "HeatMap")loadHeatMap();
}

void VisionApp::showBufOnMainDisp(const QRgb& oColor)
{
	QRgb *pBuf;
	BufferInfo rBufInfo;
	BufferInfo gBufInfo;
	BufferInfo bBufInfo;
	BufferInfo oBufInfo;
	unsigned char* pRline;
	unsigned char* pGline;
	unsigned char* pBline;
	unsigned char* pOline;

	_algo.bufferInfo(_displayBuffer.value(QStringLiteral("R_Buffer")), rBufInfo);
	_algo.bufferInfo(_displayBuffer.value(QStringLiteral("G_Buffer")), gBufInfo);
	_algo.bufferInfo(_displayBuffer.value(QStringLiteral("B_Buffer")), bBufInfo);
	_algo.bufferInfo(QStringLiteral("O_Buffer"), oBufInfo);

	for (int i = 0; i < _imageMain.height(); ++i)
	{
		pBuf = reinterpret_cast<QRgb *>(_imageMain.scanLine(i));

		pRline = rBufInfo.pBuf + (i * rBufInfo.pitch);
		pGline = gBufInfo.pBuf + (i * gBufInfo.pitch);
		pBline = bBufInfo.pBuf + (i * bBufInfo.pitch);
		pOline = oBufInfo.pBuf + (i * oBufInfo.pitch);

		for (int j = 0; j < _imageMain.width(); ++j)
		{
			if (pOline[j] == 0)
			{
				pBuf[j] = qRgb(pRline[j], pGline[j], pBline[j]);
			}
			else
			{
				pBuf[j] = oColor;
			}
		}
	}

	_pixmapMain = QPixmap::fromImage(_imageMain, Qt::ColorOnly);
	updateMainDisp();
}

void VisionApp::initDisplaySize()
{
	BufferInfo bufInfo;
	_algo.bufferInfo(_displayBuffer.value(QStringLiteral("R_Buffer")), bufInfo);
	_imageSize.rwidth() = bufInfo.width;
	_imageSize.rheight() = bufInfo.height;
}

void VisionApp::drawCropGuidingRoi()
{
	qDebug() << "drawCropGuidingRoi";
	if (!_cropGuidingRoi.isEmpty()) return;

	VisionAppQDragBox* pShape = new VisionAppQDragBox();
	connect(pShape, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(guidingRoiResize(QDragBox*, QString, QPointF)));

	QRectF guidingRect(QPoint(0,0), QSize(100, 100));
	qDebug() << "Guiding Rect: " << guidingRect;

	_pGraphicsSceneMain->addItem(pShape);
	pShape->setup(guidingRect, Qt::yellow, QString("Guiding roi"));
	pShape->setOutterBarrier(_sceneBound);
	pShape->setDragable(true);
	pShape->setZValue((int)UIHierarchy::DRAGGABLES);

	pShape->setZValue(9999999);
	_cropGuidingRoi.append(pShape);
	return ;
}

void VisionApp::guidingRoiResize(QDragBox* roi, QString name, QPointF point)
{
	qDebug() << "guiding roi resized!";
	qDebug() << roi->getGeometry().toRect();
	ui.lineEdit_ve_cropX->setText(QString::number(roi->getGeometry().toRect().x()));
	ui.lineEdit_ve_cropY->setText(QString::number(roi->getGeometry().toRect().y()));
	ui.lineEdit_ve_cropW->setText(QString::number(roi->getGeometry().toRect().width()));
	ui.lineEdit_ve_cropH->setText(QString::number(roi->getGeometry().toRect().height()));
}

void VisionApp::updateCameraTypeUI(const QString& camID)
{
	auto channel = CAMManager::instance().getChannel(camID);
	bool isMonoCam = true;

	if (channel == 1) {
		ui.toolButton_snapMono->setText("Snap Mono");
	}
	else {
		ui.toolButton_snapMono->setText("Snap");
		isMonoCam = false;
	}

	ui.toolButton_snapColor->setVisible(isMonoCam);

	/*
	ui.label_64->setVisible(isMonoCam);
	ui.lineEdit_redBuffer->setVisible(isMonoCam);
	ui.toolButton_updateRedBuffer->setVisible(isMonoCam);
	ui.label_65->setVisible(isMonoCam);
	ui.lineEdit_greenBuffer->setVisible(isMonoCam);
	ui.toolButton_updateGreenBuffer->setVisible(isMonoCam);
	ui.label_66->setVisible(isMonoCam);
	ui.lineEdit_blueBuffer->setVisible(isMonoCam);
	ui.toolButton_updateBlueBuffer->setVisible(isMonoCam);
	ui.toolButton_updateAllBuffer->setVisible(isMonoCam);*/
}

void VisionApp::updatePortabilityFeatureUI(int index)
{
	if (index == 0) {
		ui.lineEdit_featureLearningStatus->setVisible(true);
		ui.toolButton_portabilityLearnPattern->setVisible(true);
		ui.toolButton_portabilityFindPattern->setVisible(true);

		ui.label_110->setVisible(false);
		ui.lineEdit_minDiameter->setVisible(false);
		ui.label_111->setVisible(false);
		ui.lineEdit_maxDiameter->setVisible(false);
		ui.toolButton_portabilityFindCircle->setVisible(false);
	}
	else {
		ui.lineEdit_featureLearningStatus->setVisible(false);
		ui.toolButton_portabilityLearnPattern->setVisible(false);
		ui.toolButton_portabilityFindPattern->setVisible(false);

		ui.label_110->setVisible(true);
		ui.lineEdit_minDiameter->setVisible(true);
		ui.label_111->setVisible(true);
		ui.lineEdit_maxDiameter->setVisible(true);
		ui.toolButton_portabilityFindCircle->setVisible(true);
	}
}

QRectItem * VisionApp::drawDefectRect(const QRectF & rect, const QString & defectID, const QString & defectName, const QString& viewId, const QString& indexId, const QString & opticId, const QColor & borderColor, const QColor & innerColor)
{
	QRectItem *pShape = new QRectItem();
	_pGraphicsSceneMain->addItem(pShape);
	pShape->setup(rect, borderColor, innerColor);
	pShape->setZValue(9999);
	_defectRectShape.append(pShape);
	return nullptr;
}

VisionAppQDragBox * VisionApp::drawVisionAppDragBox(const QRectF & rect, const QColor & color, const QString & name, const QString & viewID)
{
	VisionAppQDragBox * pShape = new VisionAppQDragBox();

	pShape->setup(rect, color, name);
	pShape->setOutterBarrier(_sceneBound);
	pShape->setOutterBarrier(_pGraphicsSceneMain->sceneRect());

	auto r = _pGraphicsSceneMain->sceneRect();

	pShape->setDragable(true);
	pShape->setZValue((int)UIHierarchy::DRAGGABLES);
	pShape->type((int)DragBoxType::VISIONOBJECT);
	pShape->viewID(viewID);
	pShape->addNoViewPixmap(&_noViewPixmap);

	_pGraphicsSceneMain->addItem(pShape);

	connect(pShape, SIGNAL(dragBoxCreated(QRectF)), this, SLOT(dragBoxCreatedEvent(QRectF)));
	connect(pShape, SIGNAL(dragBoxMousePressed(QDragBox*, QString, QPointF)), this, SLOT(dragBoxMousePressedEvent(QDragBox*, QString, QPointF)));
	connect(pShape, SIGNAL(dragBoxMouseReleased(QDragBox*, QString, QPointF)), this, SLOT(dragBoxMouseReleasedEvent(QDragBox*, QString, QPointF)));
	connect(pShape, SIGNAL(dragBoxMouseHoverEntered(QDragBox*)), this, SLOT(dragBoxMouseHoverEntered(QDragBox*)));
	connect(pShape, SIGNAL(dragBoxMouseHoverLeaved(QDragBox*, QString)), this, SLOT(dragBoxMouseHoverLeaved(QDragBox*, QString)));
	connect(pShape, SIGNAL(dragBoxContextMenuEvent(QDragBox*, QString, QPointF)), this, SLOT(dragBoxContextMenuEvent(QDragBox*, QString, QPointF)));
	connect(pShape, SIGNAL(dragBoxResized(QDragBox*, QString)), this, SLOT(dragBoxResized(QDragBox*, QString)));
	connect(pShape, SIGNAL(grabberPressed(QDragBox*)), this, SLOT(grabberPressed(QDragBox*)));
	connect(pShape, SIGNAL(grabberReleased(QDragBox*)), this, SLOT(grabberReleased(QDragBox*)));

	_dragROI.append(pShape);

	return pShape;
}

QDragBox * VisionApp::drawViewBox(const QRectF & rect, const QColor & color, const QString & name, QString id)
{
	QDragBox * pShape = new QDragBox();

	pShape->setup(rect, color, name);
	pShape->setID(id);
	pShape->setOutterBarrier(_sceneBound);
	pShape->setOutterBarrier(_pGraphicsSceneMain->sceneRect());
	pShape->setDragable(false);
	pShape->setZValue((int)UIHierarchy::VIEW);
	pShape->type((int)DragBoxType::VIEW);

	_pGraphicsSceneMain->addItem(pShape);

	connect(pShape, SIGNAL(dragBoxMousePressed(QDragBox*, QString, QPointF)), this, SLOT(viewBoxMousePressedEvent(QDragBox*, QString, QPointF)));
	connect(pShape, SIGNAL(dragBoxMouseHoverEntered(QDragBox*)), this, SLOT(viewBoxMouseHoverEntered(QDragBox*)));

	auto fontSize = rect.width() / 30;
	if (fontSize <= 0) fontSize = 10;

	pShape->text = drawText(name, QPointF(rect.x(), rect.y()), getColor(Representation::UNASSIGNED_VIEW), fontSize);
	excludeFromRenderedShape(pShape->text);
	_viewROI.append(pShape);

	return pShape;
}

QDragBox * VisionApp::drawLineScan(const QRectF & rect, const QColor & color, const QString & name, QString id)
{
	QDragBox * pShape = new QDragBox();

	pShape->setup(rect, color, name);
	pShape->setID(id);
	pShape->setOutterBarrier(_sceneBound);
	pShape->setOutterBarrier(_pGraphicsSceneMain->sceneRect());
	pShape->setDragable(false);
	pShape->setZValue((int)UIHierarchy::VIEW);
	pShape->type((int)DragBoxType::VIEW);

	_pGraphicsSceneMain->addItem(pShape);

	auto fontSize = rect.width() / 30;
	if (fontSize <= 0) fontSize = 10;

	pShape->text = drawText(name + "@@" + id, QPointF(rect.x(), rect.y()), color, fontSize);
	excludeFromRenderedShape(pShape->text);
	_lineScanROI.append(pShape);

	return pShape;
}

void VisionApp::dragBoxCreatedEvent(QRectF rect)
{
}

void VisionApp::dragBoxMouseReleasedEvent(QDragBox* pDragBox, QString name, QPointF pos)
{
	qDebug() << "----";
	qDebug() << "Object Id: " << pDragBox->getId();
	qDebug() << "ObjectName: " << _visionObject[pDragBox->getId()].objectName;;
	qDebug() << "Row ID: " << _visionObject[pDragBox->getId()].row_id << " Row Name: "<< _visionObject[pDragBox->getId()].row;
	qDebug() << "Col ID: " << _visionObject[pDragBox->getId()].col_id << " Col Name: " << _visionObject[pDragBox->getId()].col;
	qDebug() << "Island ID: " << _visionObject[pDragBox->getId()].island_id << " Island Name: " << _visionObject[pDragBox->getId()].island;
	qDebug() << "Ignore: " << _visionObject[pDragBox->getId()].ignore;
	qDebug() << "Skip: " << _visionObject[pDragBox->getId()].skip;
	qDebug() << "ForcedSkip: " << _visionObject[pDragBox->getId()].forcedSkip;
	qDebug() << "Circuit ID: " << _visionObject[pDragBox->getId()].circuitID;
	qDebug() << "Mounter ID: " << _visionObject[pDragBox->getId()].cadMounterId;

	QString row = QString::number(_visionObject[pDragBox->getId()].row_id);
	QString col = QString::number(_visionObject[pDragBox->getId()].col_id);
	QString island = QString::number(_visionObject[pDragBox->getId()].island_id);


	updateVisionObjectInfo();

	QVector<QDragBox*> items;

	for (auto roi : _dragROI) {
		if (roi->isSelected()) {
			items.push_back(roi);
	
		}
	}

	auto num = numBoxSelected();
	qDebug() << "name:" << name;
	qDebug() << "num:" << num;
	if (num == 1) {
		QList<QStandardItem *> itemFound = _recipeModel.findItems(name, Qt::MatchExactly | Qt::MatchRecursive);

		if (itemFound.count() == 1)
		{

			ui.treeViewRecipeExplorer->setCurrentIndex(_recipeModel.indexFromItem(itemFound.at(0)));

		}

		itemFound.clear();

		itemFound = _resultModel.findItems(name, Qt::MatchExactly | Qt::MatchRecursive);

		if (itemFound.count() == 1)
		{
			ui.treeViewResultExplorer->setCurrentIndex(_resultModel.indexFromItem(itemFound.at(0)));
		}

		auto items = ui.listWidget_unassignedVisionObject->findItems(name, Qt::MatchExactly);

		for (auto item : items) {
			ui.listWidget_unassignedVisionObject->setCurrentItem(item);
		}
	}

	_undoStack->push(new MultiMoveDragBoxCommand(items, pDragBox, _pressedDragPos));
}

void VisionApp::dragBoxMousePressedEvent(QDragBox * pDragBox, QString name, QPointF pos)
{
	storeVisionObjectInfo();
	_pressedDragPos = pos;
}

void VisionApp::dragBoxMouseHoverEntered(QDragBox* pDragBox)
{
}

void VisionApp::dragBoxMouseHoverLeaved(QDragBox* pDragBox, QString name)
{
}

void VisionApp::dragBoxContextMenuEvent(QDragBox * pDragBox, QString name, QPointF pos)
{
	
	if ((pDragBox->isSelected() == true) && (numBoxSelected() == 1))
	{
		QMenu menu(this);

		if (isPage(UIPage::TEMPLATE_LIB)) menu.addAction(ui.actionAdd_Vision_Object_as_Default_Template);
		menu.addAction(ui.actionEditTemplate);
		menu.addAction(ui.actionDuplicateVisionObjects);
		menu.addAction(ui.action_saveReferenceVOImage);
		menu.exec(QCursor::pos());
	}
}


void VisionApp::dragBoxResized(QDragBox* pDragBox, QString name)
{
	qDebug() << pDragBox->getGeometry();
	QRectF rect = pDragBox->getGeometry();

	auto worldScale = ScaleManager::instance().world_scale();

	// check if bigger than view rect
	int viewWidth = CAMManager::instance().getWidth(_camID) * worldScale;
	int viewHeight = CAMManager::instance().getHeight(_camID) * worldScale;
	/*if (rect.width() > viewWidth || rect.height() > viewHeight)
	{
		QMessageBox::warning(this, "Oversize Vision Object", "Vision Object cannot be greater than View size! <br> Vision Object will be resized!");
		if (rect.width() > viewWidth)
		{
			rect.setWidth(viewWidth - 5);
		}
		if (rect.height() > viewHeight)
		{

			rect.setHeight(viewHeight - 5);
		}

		pDragBox->setGeometry(rect);
	}*/
	
	for (int i = 0; i < _dragROI.count(); i++)
	{
		if (pDragBox == _dragROI.at(i)) { // Found the box that was resized
			QString currentVoId = _dragROI.at(i)->getId();
			if (_visionObject.contains(currentVoId)) {
				// This line updates the 'rect' field in your QVisionObject.
				// As discussed, ensure the coordinate system transformation here is what you intend
				// if _dragROI.at(i)->getGeometry() is scene pixels.
				_visionObject[currentVoId].rect = ScaleManager::instance().world_to_fov(_dragROI.at(i)->getGeometry());
				qDebug() << "Updated QVisionObject.rect for ID" << currentVoId << "to" << _visionObject[currentVoId].rect;
			}
			else {
				qWarning() << "QDragBox ID" << currentVoId << "from _dragROI not found in _visionObject map.";
			}
			// break; // Optimization: once found, no need to continue loop if only one match expected
		}
	}
	QString resizedVoId = pDragBox->getId();
	if (!resizedVoId.isEmpty()) {
		qDebug() << "Triggering circle redraw for VO ID:" << resizedVoId;
		redrawCirclesForVO(resizedVoId); // Call the helper function
	}
	else {
		qWarning() << "Cannot redraw circles: Resized QDragBox has an empty ID.";
	}


	//QRectF intersectedRect = pDragBox->getGeometry().intersected(pDragBox->getOutterBarrier());

	//pDragBox->setGeometry(intersectedRect);
}

void VisionApp::redrawCirclesForVO(const QString& voId)
{
	if (!_visionObject.contains(voId)) {
		qWarning() << "redrawCirclesForVO: VO ID not found in map:" << voId;
		return;
	}

	QVisionObject& currentVO = _visionObject[voId]; // Get reference
	QDragBox* boxToRedraw = currentVO.pDragBox;

	if (!boxToRedraw) {
		qWarning() << "redrawCirclesForVO: pDragBox is null for VO ID" << voId;
		return;
	}

	// --- "Dots by demand": Check if this VO is supposed to have dots ---
	if (currentVO.overlayPoints.isEmpty() || currentVO.overlayWorldClusterRect.isNull() || currentVO.overlayWorldClusterRect.isEmpty()) {
		qDebug() << "redrawCirclesForVO: VO ID" << voId << "has no overlay data. Clearing any existing circles.";
		// Clear any existing circles if there's no data (or if data was removed)
		for (QGraphicsEllipseItem* item : currentVO.circleItems) {
			if (item) {
				if (item->scene()) item->scene()->removeItem(item);
				delete item;
			}
		}
		currentVO.circleItems.clear();
		if (boxToRedraw->scene()) boxToRedraw->update(); // Update box to show cleared circles
		return; // Nothing more to draw
	}

	// Proceed with redrawing
	QRectF currentBoxLocalGeom = boxToRedraw->getGeometry(); // (0,0, pixel_width, pixel_height)
	qreal boxCurrentPixelWidth = currentBoxLocalGeom.width();
	qreal boxCurrentPixelHeight = currentBoxLocalGeom.height();

	qreal marginPixels = currentVO.overlayContentMarginPixels;

	qreal newAvailableContentWidth = boxCurrentPixelWidth - (2 * marginPixels);
	qreal newAvailableContentHeight = boxCurrentPixelHeight - (2 * marginPixels);

	if (newAvailableContentWidth < 1) newAvailableContentWidth = 1;
	if (newAvailableContentHeight < 1) newAvailableContentHeight = 1;

	QRectF newTargetContentAreaInBox = QRectF(marginPixels, marginPixels, newAvailableContentWidth, newAvailableContentHeight);

	qDebug() << "Redrawing circles for VO ID:" << voId
		<< "Box new pixel size:" << QSizeF(boxCurrentPixelWidth, boxCurrentPixelHeight)
		<< "New target content area:" << newTargetContentAreaInBox;

	overlayPadCircles( // Your aspect ratio preserving function
		boxToRedraw,
		currentVO.circleItems, // Pass the vector to be cleared and refilled
		currentVO.overlayPoints,
		currentVO.overlayWorldClusterRect,
		newTargetContentAreaInBox,
		currentVO.overlayCircleRadius,
		currentVO.overlayCircleColor
	);
}

void VisionApp::grabberReleased(QDragBox * pDragBox)
{
	qDebug() << "grabberReleased";
	auto vo = (VisionAppQDragBox*)pDragBox;
	auto algoTemplate = vo->algoTemplate();
	if (algoTemplate)
	{
		auto FOVWidth = ScaleManager::instance().world_to_fov(vo->getGeometry().width());
		auto FOVHeight = ScaleManager::instance().world_to_fov(vo->getGeometry().height());
		algoTemplate->w(FOVWidth);
		algoTemplate->h(FOVHeight);

		QString imageFilePath = algoTemplate->templateImagePath();
		imageFilePath = Common::Directory::getRecipeCurrentPath() + "template_Images\\" + algoTemplate->templateImagePath();
		if (algoTemplate->templateImagePath().contains("c:\\Advanced\\Data\\recipe")) imageFilePath = algoTemplate->templateImagePath();
		QImage templateImg;
		if (templateImg.load(imageFilePath))
		{
			if (templateImg.size().width() < algoTemplate->w() && templateImg.size().height() < algoTemplate->h())
			{
				QImage paddedImage(algoTemplate->w(), algoTemplate->h(), QImage::Format_RGB32);
				paddedImage.fill(Qt::black); // Fill the image with black color

				QPainter painter(&paddedImage);
				painter.drawImage(QPoint(0, 0), templateImg);
				painter.end();
				paddedImage.save(imageFilePath);

			}
			else
			{
				templateImg = templateImg.copy(QRect(0, 0, algoTemplate->w(), algoTemplate->h()));
				templateImg.save(imageFilePath);
			}

			_templateLibraryTab->setTemplateImagePath(algoTemplate->templateId(), imageFilePath, QSize(algoTemplate->w(), algoTemplate->h()));
		}

		updateVisionObjectSize(vo->algoTemplate());


	}
	updateVisionObjectInfo();
}

void VisionApp::grabberPressed(QDragBox * pDragBox)
{

	storeVisionObjectInfo();
}

void VisionApp::refreshDragBoxSequence()
{
	if (!_dragROI.isEmpty()) {
		QVector<double>Area;
		for (int i = 0; i < _dragROI.size(); i++)
		{
			double area = _dragROI[i]->getGeometry().width() * _dragROI[i]->getGeometry().height();
			Area.append(area);
		}
		for (int i = 0; i < _viewROI.size(); i++)
		{
			double area = _viewROI[i]->getGeometry().width() * _viewROI[i]->getGeometry().height();
			Area.append(area);
		}
		std::sort(Area.begin(), Area.end(), compareArea);

		for (int i = 0; i < _dragROI.size(); i++)
		{
			double w = _dragROI[i]->getGeometry().width();
			double h = _dragROI[i]->getGeometry().height();


			for (int j = 0; j < Area.size(); j++)
			{
				if (Area[j] == w*h) _dragROI[i]->setZValue(_dragROI[i]->zValue() + j);
			}
		}

		for (int i = 0; i < _viewROI.size(); i++)
		{
			int w = _viewROI[i]->getGeometry().width();
			int h = _viewROI[i]->getGeometry().height();

			for (int j = 0; j < Area.size(); j++)
			{
				if (Area[j] == w*h) _viewROI[i]->setZValue(_viewROI[i]->zValue() + j);
			}
		}
	}
}

void VisionApp::viewBoxMousePressedEvent(QDragBox * pDragBox, QString name, QPointF pos)
{
	_lastViewPressed = pDragBox->getId();

	//auto items = ui.listWidget_viewSelection->findItems(name, Qt::MatchExactly);

	//if (items.count() > 0) {
		//if (items.at(0)->checkState() == Qt::Checked) {
	emit viewBoxPressed(pDragBox, name, pos);
	//}
//}
}

void VisionApp::viewBoxMouseHoverEntered(QDragBox * pDragBox)
{
	if (_editMode == EditMode::PATH_ASSIGNMENT) { //edit mode == view sequence

		_lastSelectedViewHovered = pDragBox->getId();

		if (QGuiApplication::keyboardModifiers() & Qt::ControlModifier) {
			addViewToPath(_lastSelectedViewHovered);
		}

		/*for (int i = 0; i < ui.listWidget_viewSelection->count(); i++) {
			auto item = ui.listWidget_viewSelection->item(i);

			if (item->whatsThis() == pDragBox->getId()) {
				if (item->checkState() == Qt::Checked) {
					_lastSelectedViewHovered = item->whatsThis();

					if (QGuiApplication::keyboardModifiers() & Qt::ControlModifier) {
						addViewToPath(_lastSelectedViewHovered);
					}

				}
				break;
			}
		}*/
	}
}

void VisionApp::mouseMove(QPoint pt)
{
	if (_sceneBound.contains(pt) == true)
	{
		_scenePos = pt;
		statusBar()->showMessage(QStringLiteral("  %1  %2").arg(pt.x()).arg(pt.y()), 0);
	}
}

void VisionApp::doDrawRect(const QRectF& rect, const QColor& borderColor, const QString& toolTip)
{
	QRectItem *pShape = drawRect(rect, borderColor);
	pShape->setToolTip(toolTip);
}

void VisionApp::doDrawLine(const QLineF& line, const QColor& borderColor, const QString& toolTip)
{
	QLineItem *pShape = drawLine(line, borderColor);
	pShape->setToolTip(toolTip);
}

void VisionApp::doDrawCross(const QRectF& rect, const QColor& borderColor, const QString& toolTip)
{
	QCrossItem *pShape = drawCross(rect, borderColor);
	pShape->setToolTip(toolTip);
}

void VisionApp::doDrawText(const QString& text, const QPointF& pos, const QColor& color, const int& pointSize)
{
	drawText(text, pos, color, pointSize);
}

void VisionApp::doDrawCircle(const QPointF& pt, const qreal& radius, const QColor& color, const QString& toolTip)
{
	QEllipseItem* pShape = drawEllipse(pt.x(), pt.y(), radius, radius, color);
	pShape->setToolTip(toolTip);
}

void VisionApp::clearAllDrawings()
{
	for (int i = 0; i < _renderedShape.count(); i++)
	{
		if (_renderedShape[i] == nullptr) {
			_pGraphicsSceneMain->removeItem(_renderedShape.at(i));
			delete _renderedShape.at(i);
			_renderedShape[i] = nullptr;
		}
	}

	_renderedShape.clear();
	_algo.clearOverlay();

	clearAllDefectRectShape();
}

void VisionApp::clearDrawing(QGraphicsItem * pShape)
{
	for (int i = 0; i < _renderedShape.count(); i++)
	{
		if (_renderedShape.at(i) == pShape)
		{
			_pGraphicsSceneMain->removeItem(_renderedShape.at(i));
			delete _renderedShape.at(i);

			_renderedShape.removeAt(i);
			break;
		}
	}
}

void VisionApp::excludeFromRenderedShape(QGraphicsItem * pShape)
{
	for (int i = 0; i < _renderedShape.count(); i++)
	{
		if (_renderedShape.at(i) == pShape)
		{
			_renderedShape.removeAt(i);
			break;
		}
	}
}

void VisionApp::clearDragBox(QDragBox * pDragBox)
{
	QDragBox* pDeleted = nullptr;
	for (int i = 0; i < _dragROI.count(); i++)
	{
		if (_dragROI.at(i) == pDragBox)
		{
			_visionObject.remove(_dragROI[i]->getId());

			_dragROI.at(i)->deleteLater();

			if (pDragBox != nullptr) _dragROI.removeOne((VisionAppQDragBox*)pDragBox);
			break;
		}
	}

	for (int i = 0; i < _viewROI.count(); i++)
	{
		if (_viewROI.at(i) == pDragBox)
		{
			if (_viewROI.at(i)->text) _viewROI.at(i)->text->deleteLater();
			_viewROI.at(i)->deleteLater();

			if (pDragBox != nullptr) _viewROI.removeOne(pDragBox);
			break;
		}
	}

	for (int i = 0; i < _lineScanROI.count(); i++)
	{
		if (_lineScanROI.at(i) == pDragBox)
		{
			if (_lineScanROI.at(i)->text) _lineScanROI.at(i)->text->deleteLater();
			_lineScanROI.at(i)->deleteLater();

			if (pDragBox != nullptr) _lineScanROI.removeOne(pDragBox);
			break;
		}
	}
}

void VisionApp::loadImage(const QString& fileName)
{
	qDebug() << "loading image...";
	QPixmap img;

	if (img.load(fileName) == true)
	{
		if ((img.width() == _imageSize.width()) && (img.height() == _imageSize.height()))
		{
			_pixmapMain = img;

			_imageMain = _pixmapMain.toImage();
			loadImage2Mem(_bufferInfoObj);
			showBufOnMainDisp();
		}
		else
		{
			QString error = QStringLiteral("Image dimension not supported.") + QStringLiteral("ImageWidth:") + QString::number(_imageSize.width()) + QStringLiteral(" ImageHeight:") + QString::number(_imageSize.height());
			showMsg(error);
		}
	}
}

void VisionApp::loadImageSet(QString imageSetPath)
{
	QString folderPath = imageSetPath;
	

	if (!folderPath.isEmpty()) {

		ct::logger::debug("Selected folder path: %s", folderPath.toStdString().c_str());

		Common::Directory::CurrentImageSetPath = folderPath + "\\";

		progressBarSetup("Loading Images...", _views.count());

		if (g_viewMode == (int)ViewMode::PLANE)
		{
			QString planeImgPath = Common::Directory::CurrentImageSetPath + "plane.jpg";
			
			QImage _imagePlane = QImage(_imageWorld.width(), _imageWorld.height(), QImage::Format_RGB32);
			_imagePlane.fill(Qt::transparent);

			bool planeLoaded = false;
			if (QFileInfo::exists(planeImgPath))
			{
				planeLoaded = _imagePlane.load(planeImgPath);
			}

			if (!planeLoaded)
			{
				QPainter painter(&_imagePlane);

				for (const auto& v : _views)
				{

					if (v.type == ct::s_child_view) {
						continue;
					}

					auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), v, _mainOptics[_camID], _recipeOptics);

					ct::logger::debug("Load image: %s", ipf.getMainOpticPath().c_str());
					QImage viewImg(ipf.getMainOpticPath().c_str());
					ct::logger::debug("Loaded image");
					viewImg = viewImg.convertToFormat(QImage::Format_ARGB32);
					ct::logger::debug("1");
					int width = viewImg.width();
					int height = viewImg.height();


					auto px = ScaleManager::instance().fov_to_world(v.px);
					viewImg = viewImg.scaled(px.w, px.h, Qt::KeepAspectRatio);


					QPointF viewWorld = ScaleManager::instance().to_world_px(v);
					painter.drawImage(viewWorld, viewImg);

					incrementProgressBar();
				}
				painter.end();
				_imagePlane.save(planeImgPath);
			}

			QPainter wpainter(&_imageWorld);
			_imageWorld.fill(Qt::black);
			wpainter.drawImage(QPointF(0, 0), _imagePlane);

			displayImage(_imageWorld);
			progressBarRelease();
		}
		else
		{
			displayCurrentView();
			_datasetPage->updateDatasetView(_unitConfigTab->getUnifConfigInfos(), _unitConfigTab, _views, _recipeOptics);
			progressBarRelease();
		}
		
	}
	else {
		qDebug() << "No folder selected.";
		return;
	}
}

void VisionApp::loadProcessedImage(QString imageSetPath, QString savePath)
{
	QString folderPath = imageSetPath;


	if (!folderPath.isEmpty()) {

		ct::logger::debug("Selected folder path: %s", folderPath.toStdString().c_str());

		Common::Directory::CurrentImageSetPath = folderPath + "\\";

		progressBarSetup("Loading Images...", _views.count());

		if (g_viewMode == (int)ViewMode::PLANE)
		{
			QImage _imagePlane = QImage(_imageWorld.width(), _imageWorld.height(), QImage::Format_RGB32);
			_imagePlane.fill(Qt::transparent);

			QPainter painter(&_imagePlane);

			for (const auto& v : _views)
			{

				if (v.type == ct::s_child_view) {
					continue;
				}

				auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), v, _mainOptics[_camID], _recipeOptics);

				ct::logger::debug("Load image: %s", ipf.getMainOpticPath().c_str());
				QImage viewImg(ipf.getMainOpticPath().c_str());
				ct::logger::debug("Loaded image");
				viewImg = viewImg.convertToFormat(QImage::Format_ARGB32);
				ct::logger::debug("1");
				int width = viewImg.width();
				int height = viewImg.height();


				auto px = ScaleManager::instance().fov_to_world(v.px);
				viewImg = viewImg.scaled(px.w, px.h, Qt::KeepAspectRatio);

				QPointF viewWorld = ScaleManager::instance().to_world_px(v);
				painter.drawImage(viewWorld, viewImg);

				incrementProgressBar();
			}
			painter.end();

			QPainter wpainter(&_imageWorld);
			wpainter.drawImage(QPointF(0, 0), _imagePlane);  //  Keep heatmap intact

			//  Define legend dimensions
			int legendWidth = 200;
			int legendHeight = _imageWorld.height() / 4;  //  Half the height
			int legendX =  500;  //  Move slightly inward
			int legendY = ((_imageWorld.height() - legendHeight) / 2)+1000;


			QLinearGradient gradient(legendX, legendY, legendX, legendY + legendHeight);
			gradient.setColorAt(0.0, QColor(255, 255, 255)); // High values (white)
			gradient.setColorAt(0.15, QColor(255, 255, 0)); // Yellow
			gradient.setColorAt(0.4, QColor(255, 0, 0));  // Bright red
			gradient.setColorAt(0.7, QColor(50, 0, 0));   // Dark red
			gradient.setColorAt(1.0, Qt::black);    // Low values

			wpainter.fillRect(QRectF(legendX, legendY, legendWidth, legendHeight), QBrush(gradient));

			//  Add labels to the **right** of the legend
			wpainter.setPen(Qt::white);
			wpainter.setFont(QFont("Arial", 200));
			wpainter.drawText(legendX  , legendY -50, "High Edge Score");  // Top label
			wpainter.drawText(legendX , legendY + legendHeight - 50, "Low Edge Score");  // Bottom label

			wpainter.end();  

			//  Save and display
			_imageWorld.save(savePath);
			displayImage(_imageWorld);
			progressBarRelease();

		}
		else
		{
			displayCurrentView();
			_datasetPage->updateDatasetView(_unitConfigTab->getUnifConfigInfos(), _unitConfigTab, _views, _recipeOptics);
			progressBarRelease();
		}

	}
	else {
		qDebug() << "No folder selected.";
		return;
	}
}

void VisionApp::saveImage()
{
	bool ok;
	bool flag = true;
	QString imageName = QInputDialog::getText(this, tr("Save Image"), tr("Image:"), QLineEdit::Normal, "", &ok, Qt::CoverWindow);

	if (imageName.contains(QStringLiteral(".")) == true)
	{
		imageName = imageName.split(".").at(0);
	}

	if (ok && !imageName.isEmpty())
	{
		QDir dir(Common::Directory::getRecipeImagesPath());

		if (dir.exists(QStringLiteral("%1.jpg").arg(imageName)) == true)
		{
			QMessageBox::StandardButton reply = QMessageBox::question(this, "Save Image", "Overwrite Image?", QMessageBox::Yes | QMessageBox::No);

			if (reply == QMessageBox::No)
			{
				flag = false;
			}
		}

		if (flag == true)
		{
			logMsg(QStringLiteral("%1/%2.jpg").arg(Common::Directory::getRecipeImagesPath()).arg(imageName));

			//auto path = QStringLiteral("%1/%2.jpg").arg(Common::Directory::getRecipeImagesPath()).arg(imageName);

			//auto mBuf = mtrx::to_milID(_imageFOV);
			//MbufExportA(path.toStdString().c_str(), M_JPEG_LOSSY, mBuf);
			//mtrx::free_buffer(mBuf);

			auto path = QStringLiteral("%1/%2.bmp").arg(Common::Directory::getRecipeImagesPath()).arg(imageName);

			auto mBuf = mtrx::to_milID(_imageFOV);
			MbufExportA(path.toStdString().c_str(), M_BMP, mBuf);
			mtrx::free_buffer(mBuf);
		}
	}
}

bool VisionApp::saveJson(const QString& fileName, const QJsonDocument& doc)
{
	bool flag = false;

	QFile file(fileName);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(doc.toJson());
		file.flush();
		file.close();

		flag = true;
	}

	return flag;
}

void VisionApp::showMsg(const QString& msg, QMessageBox::StandardButtons buttons)
{
	_msg.setText(msg);
	_msg.setStandardButtons(buttons);
	_msg.setDefaultButton(QMessageBox::Ok);
	_msg.setWindowFlags(Qt::WindowStaysOnTopHint | Qt::WindowCloseButtonHint);
	// Qt::FramelessWindowHint
	//_msg.setStyleSheet(
	//	"QMessageBox {color: white; background-color: rgb(27, 29, 35); border: ; border-radius: 12px; padding: 10px;}"
	//	"QMessageBox QPushButton {font-size: 12px; color: white; background-color: rgb(39, 44, 54); border-radius: 5px; padding: 5px;}"
	//	"QMessageBox QLabel {color: white}"
	//	);

	/*_msg.setStyleSheet(
		"QMessageBox {color: white; background-color: rgb(39, 44, 54); border: none; border-radius: 12px; padding: 10px;}"
		"QMessageBox QPushButton {font-size: 12px; color: white; background-color: rgb(27, 29, 35); border-radius: 5px; padding: 5px;}"
		"QMessageBox QPushButton:hover {background-color: rgb(57, 65, 80); border: 2px solid rgb(61, 70, 86);}"
		"QMessageBox QPushButton:pressed {background-color: rgb(35, 40, 49); border: 2px solid rgb(43, 50, 61);}"
		"QMessageBox QLabel {color: white; background-color: rgb(39, 44, 54); border-radius: 5px;}"
	);*/

	//setMessageBoxTitleColor(_msg, Qt::red);
	//updateMsgBoxBorder();
	//emit updateMsgBoxBorderSignal();
	_msg.exec();


}

void VisionApp::showStatus(const QString& msg, int timeout)
{
	showInfo(msg);
	statusBar()->showMessage(msg, timeout);
}

void VisionApp::processEvents()
{
	QCoreApplication::processEvents();
}

void VisionApp::updateVisionObjectGeometry()
{
	for (int i = 0; i < _dragROI.count(); i++)
	{
		_visionObject[_dragROI.at(i)->getId()].rect = ScaleManager::instance().world_to_fov(_dragROI.at(i)->getGeometry());
	}
}

void VisionApp::loadWorldView()
{
	QFileDialog dialog(this, tr("Open Image File"), QDir::currentPath(), tr("Image Files (*.jpg *.png *.bmp)"));

	// Set the file dialog to allow only single file selection
	dialog.setFileMode(QFileDialog::ExistingFile);

	// Show the file dialog and wait for the user's selection
	if (dialog.exec() == QDialog::Accepted) {
		// Get the selected file path
		QString filePath = dialog.selectedFiles().at(0);
		_imageWorld.load(filePath);
		displayImage(_imageWorld);
	}

	return;
}

void VisionApp::loadPlaneView(bool autoLoad)
{
	if (!autoLoad)
	{
		QFileDialog dialog(this, tr("Open Image File"), QDir::currentPath(), tr("Image Files (*.jpg *.png *.bmp)"));

		// Set the file dialog to allow only single file selection
		dialog.setFileMode(QFileDialog::ExistingFile);

		// Show the file dialog and wait for the user's selection
		if (dialog.exec() == QDialog::Accepted) {

			_enableSingleViewRecipe = true;

			ScaleManager::instance().set_world_scale(1);

			// Get the selected file path
			QString filePath = dialog.selectedFiles().at(0);
			_imageWorld.load(filePath);
			auto worldWidth = ScaleManager::instance().fov_to_world(_imageWorld.width());
			auto worldHeight = ScaleManager::instance().fov_to_world(_imageWorld.height());
			_imageWorld = _imageWorld.scaled(worldWidth, worldHeight, Qt::AspectRatioMode::KeepAspectRatio, Qt::TransformationMode::SmoothTransformation);
			displayImage(_imageWorld);

			//updatePlaneView
			QString planeImgPath = Common::Directory::getRecipeImagesPath() + "plane.jpg";
			_imageWorld.save(planeImgPath);

			saveRecipeConfig();
		}
	}
	else
	{
		_imageWorld.load(Common::Directory::getRecipeImagesPath() + "plane.jpg");
		qDebug() << "planePath:" << Common::Directory::getRecipeImagesPath() + "plane.jpg";
		displayImage(_imageWorld);
	}
	

	return;
}
void VisionApp::loadHeatMap()
{
	// Input & Output Folder Paths
	QString folderPath = QFileDialog::getExistingDirectory(nullptr, "Select Folder", Common::Directory::getRecipeImagesPath());
	QString GPfolderPath = QFileInfo(folderPath).absoluteDir().absolutePath();
	QString saveFolderPath = QFileInfo(GPfolderPath).absoluteDir().absolutePath() + "/ProcessedImages";
	QString saveProcessPath = QFileInfo(GPfolderPath).absoluteDir().absolutePath() + "/tempImages";

	QDir dir(folderPath);
	if (!dir.exists()) {
		ct::logger::error("[HeatMapDirectory does not exist:%s",folderPath.toStdString()) ;
		return;
	}

	QDir saveDir(saveFolderPath);
	if (!saveDir.exists()) {
		saveDir.mkdir(saveFolderPath);
	}

	QDir saveDir1(saveProcessPath);
	if (!saveDir1.exists()) {
		saveDir1.mkdir(saveProcessPath);
	}

	QStringList folderList = saveDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
	int maxNumber = 0;
	for (const QString& folder : folderList) {
		QString prefix = "HeatMap";
		QString numPart = folder.section('_', 0, 0);
		if (numPart.startsWith(prefix)) {
			numPart = numPart.mid(prefix.length());  // Extract the substring after "HeatMap"
		}
		bool ok;
		int number = numPart.toInt(&ok);
		if (ok && number > maxNumber) {
			maxNumber = number;
		}
	}
	int nextNumber = maxNumber + 1;

	QString timeStamp = QDateTime::currentDateTime().toString("MM-dd_hhmm");
	QString uniqureFoldername = QString("HeatMap%1_%2").arg(nextNumber, 1, 10, QChar('0')) .arg(timeStamp);
	QString uniqueSaveFolderPath = saveFolderPath + "/" + uniqureFoldername;
	QDir uniqueDir;
	if (!uniqueDir.mkpath(uniqueSaveFolderPath)) {
		ct::logger::error("Failed to create the unique folder:", uniqueSaveFolderPath);
		return;
	}
	else {
		ct::logger::info( "Unique folder created at: %s" ,uniqueSaveFolderPath);
	}

	QString heatmapID = QStringLiteral(".*_(") + getMainOpticsID + QStringLiteral(").*");
	QStringList imagePaths;
	QRegularExpression regex(heatmapID);
	QFileInfoList fileInfoList = dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot);

	QMap<QString, double> edgeScores;
	QList<double> scoreList;

	progressBarSetup("Preparing Images...", fileInfoList.count());
	//Compute Laplacian edge scores for each image
	for (const QFileInfo& fileInfo : fileInfoList) {
		QString fileName = fileInfo.fileName();
		QRegularExpressionMatch match = regex.match(fileName);
		if (match.hasMatch()) {
			QString imagePath = fileInfo.absoluteFilePath();
			imagePaths << imagePath;

			// Load grayscale image
			cv::Mat gray = cv::imread(imagePath.toStdString(), cv::IMREAD_GRAYSCALE);
			if (gray.empty()) {
				ct::logger::error("[HeatMapCant open image:%s", imagePath.toStdString());
				continue;
			}

			// Compute Laplacian 
			cv::Mat laplacian;
			cv::Laplacian(gray, laplacian, CV_64F);
			cv::Mat absLaplacian;
			cv::convertScaleAbs(laplacian, absLaplacian);
			double meanEdgeScore = cv::mean(absLaplacian)[0];

			// Store scores
			edgeScores[imagePath] = meanEdgeScore;
			scoreList.append(meanEdgeScore);
		}
		incrementProgressBar();
	}
	progressBarRelease();
	std::sort(scoreList.begin(), scoreList.end());
	double minEdgeScore = scoreList.first();
	double maxEdgeScore = scoreList.last();

	progressBarSetup("Processing Images...", imagePaths.count());
	for (const QString& imagePath : imagePaths) {
		double edgeScore = edgeScores[imagePath];

		// Normalize edge score (0 to 255)
		double normalizedScore = 255.0 * ((edgeScore - minEdgeScore) / (maxEdgeScore - minEdgeScore));

		// Load original image (color)
		cv::Mat original = cv::imread(imagePath.toStdString(), cv::IMREAD_COLOR);
		if (original.empty()) {
			ct::logger::error("[HeatMap]Cant open image:%s", imagePath.toStdString());
			continue;
		}

		cv::Mat scoreMap(original.size(), CV_8UC1, cv::Scalar(normalizedScore));

		// Apply color map based on edge score
		cv::Mat colorMap;
		cv::applyColorMap(scoreMap, colorMap, cv::COLORMAP_HOT);


		int boxWidth = 1000;
		int boxHeight = 1000;
		cv::Rect box(10, original.rows - boxHeight - 10, boxWidth, boxHeight);
		cv::Vec3b boxColor = colorMap.at<cv::Vec3b>(cv::Point(normalizedScore, 0));

		cv::Mat Box = original.clone(); // Clone original to keep it intact for other operations
		cv::rectangle(Box, box, boxColor, cv::FILLED);  // Draw filled small rectangle

		double alpha = 0.7;  // 50% transparency
		double beta = 1.0 - alpha;

		// Blend the colormap with the original image
		cv::addWeighted(colorMap, alpha, original, beta, 0, original);

		// Save the processed image with edge score appended to filename
		QString savedfileName = QFileInfo(imagePath).baseName() + "_score" + QString::number(edgeScore) + ".jpg";
		QString savedPath = uniqueSaveFolderPath + "/" + savedfileName;
		cv::imwrite(savedPath.toStdString(), Box);
		QString fileName = QFileInfo(imagePath).baseName();
		QString tempPath = saveProcessPath + "/" + fileName;
		cv::imwrite(tempPath.toStdString() + ".jpg", original);
		incrementProgressBar();

		//QString imageBaseName = QFileInfo(imagePath).baseName();
		//QString parentFolderPath = QFileInfo(imagePath).absolutePath();
		//QString sessionFolderName = "session_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
		//QString sessionFolderPath = parentFolderPath + "/" + sessionFolderName;
		//QDir dir;
		//if (!dir.exists(sessionFolderPath)) {
		//	dir.mkpath(sessionFolderPath);
		//}
		//QString savedFileName = imageBaseName + "_score" + QString::number(edgeScore) + ".jpg";
		//QString savedPath = sessionFolderPath + "/" + savedFileName;
		//cv::imwrite(savedPath.toStdString(), Box);
		//QString originalSavedPath = sessionFolderPath + "/" + imageBaseName + ".jpg";
		//cv::imwrite(originalSavedPath.toStdString(), original);
		//incrementProgressBar();

	}
	progressBarRelease();
	loadProcessedImage(saveProcessPath, uniqueSaveFolderPath + "/Overall.jpg");
	QDir dirRemove(saveProcessPath);
	dirRemove.removeRecursively();
	ct::logger::info("[HeatMap]HeatMap Image generated");
}

void VisionApp::loadFOV()
{
	QFileDialog dialog(this, tr("Open Image File"), QDir::currentPath(), tr("Image Files (*.jpg *.png *.bmp)"));

	// Set the file dialog to allow only single file selection
	dialog.setFileMode(QFileDialog::ExistingFile);

	// Show the file dialog and wait for the user's selection
	if (dialog.exec() == QDialog::Accepted) {
		// Get the selected file path
		QString filePath = dialog.selectedFiles().at(0);
		_imageFOV.load(filePath);
		displayFOV(_imageFOV);
	}

	return;
}

bool VisionApp::passwordPromptCorrect()
{
	QString password = QInputDialog::getText(nullptr, "Admin Level", "Enter Admin Password:", QLineEdit::Password);
	if (password != "3df") {
		showMsg("Incorrect Password! Failed to perform action");
		return false;
	}

	return true;
}

QVector<AccountInfo> VisionApp::loadUserAccounts()
{
	QVector<AccountInfo> accounts;

	QString fileName = QStringLiteral("%1config//user.json").arg(Common::Directory::LocalPath);
	QFile file(fileName);
	if (!file.exists() || !file.open(QIODevice::ReadOnly | QIODevice::Text))
		return accounts;

	QByteArray raw = file.readAll();
	file.close();

	QJsonObject root = QJsonDocument::fromJson(raw).object();

	QJsonArray users = root.value("users").toArray();
	QString sig = root.value("sig").toString();

	// Reject an unsigned or tampered file. When rejected, only the emergency
	// recovery admin can log in, which is how you re-provision accounts.
	if (sig.isEmpty() || sig != computeUsersSignature(users))
	{
		qWarning() << "user.json has a missing/invalid signature - ignoring file";
		return accounts;   // empty
	}

	for (const QJsonValue& v : users)
	{
		QJsonObject o = v.toObject();
		AccountInfo a;
		a.userName = o.value("userName").toString();
		a.accessLevel = static_cast<AccessLevel>(o.value("accessLevel").toInt(AccessLevel::OPERATOR));
		a.salt = o.value("salt").toString();
		a.pwHash = o.value("pwHash").toString();
		if (!a.userName.isEmpty()) accounts.append(a);
	}

	return accounts;
}

bool VisionApp::saveUserAccounts(const QVector<AccountInfo>& accounts)
{
	QJsonArray arr;
	for (const AccountInfo& a : accounts)
	{
		QString salt = a.salt;
		QString pwHash = a.pwHash;

		// New user (or reset password): no hash yet -> generate salt and hash now.
		if (pwHash.isEmpty())
		{
			salt = QUuid::createUuid().toString();
			pwHash = hashPassword(salt, a.password);
		}

		QJsonObject o;
		o["userName"] = a.userName;
		o["accessLevel"] = static_cast<int>(a.accessLevel);
		o["salt"] = salt;
		o["pwHash"] = pwHash;
		arr.append(o);
	}

	QJsonObject root;
	root["users"] = arr;
	root["sig"] = computeUsersSignature(arr);

	QString fileName = QStringLiteral("%1config//user.json").arg(Common::Directory::LocalPath);
	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return false;

	file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
	file.close();

	// keep the in-memory copy in sync
	readUserInfo(_userObj);
	return true;
}

void VisionApp::openUserManagementDialog()
{
	QDialog dialog(this);
	dialog.setWindowTitle("User Management");
	dialog.resize(420, 360);

	QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);

	QTableWidget* table = new QTableWidget(&dialog);
	table->setColumnCount(2);
	table->setHorizontalHeaderLabels(QStringList() << "User" << "Access Level");
	table->horizontalHeader()->setStretchLastSection(true);
	table->setSelectionBehavior(QAbstractItemView::SelectRows);
	table->setSelectionMode(QAbstractItemView::SingleSelection);
	table->setEditTriggers(QAbstractItemView::NoEditTriggers);
	mainLayout->addWidget(table);

	// working copy; only written to disk on Save
	QVector<AccountInfo> accounts = loadUserAccounts();

	auto levelName = [](AccessLevel lvl) -> QString {
		if (lvl == AccessLevel::ADMIN) return "Admin";
		if (lvl == AccessLevel::ENGINEER) return "Engineer";
		return "Operator";
	};

	auto refreshTable = [&]() {
		table->setRowCount(accounts.size());
		for (int i = 0; i < accounts.size(); ++i)
		{
			table->setItem(i, 0, new QTableWidgetItem(accounts[i].userName));
			table->setItem(i, 1, new QTableWidgetItem(levelName(accounts[i].accessLevel)));
		}
	};
	refreshTable();

	QHBoxLayout* btnLayout = new QHBoxLayout();
	QPushButton* addBtn = new QPushButton("Add", &dialog);
	QPushButton* delBtn = new QPushButton("Delete", &dialog);
	QPushButton* saveBtn = new QPushButton("Save", &dialog);
	btnLayout->addWidget(addBtn);
	btnLayout->addWidget(delBtn);
	btnLayout->addStretch();
	btnLayout->addWidget(saveBtn);
	mainLayout->addLayout(btnLayout);

	QObject::connect(addBtn, &QPushButton::clicked, [&]() {
		if (!passwordPromptCorrect()) return;

		bool ok = false;
		QString name = QInputDialog::getText(&dialog, "Add User", "User name:", QLineEdit::Normal, "", &ok).trimmed();
		if (!ok || name.isEmpty()) return;

		for (const AccountInfo& acc : accounts)
		{
			if (acc.userName == name) { showMsg("User already exists!"); return; }
		}

		QString pass = QInputDialog::getText(&dialog, "Add User", "Password:", QLineEdit::Password, "", &ok);
		if (!ok || pass.isEmpty()) return;

		QStringList levels; levels << "Admin" << "Engineer" << "Operator";
		QString level = QInputDialog::getItem(&dialog, "Add User", "Access level:", levels, 2, false, &ok);
		if (!ok) return;

		AccountInfo a;
		a.userName = name;
		a.password = pass;
		a.accessLevel = (level == "Admin") ? AccessLevel::ADMIN
			: (level == "Engineer") ? AccessLevel::ENGINEER : AccessLevel::OPERATOR;
		accounts.append(a);
		AuditLog::instance().log(QStringLiteral("USER_ADD"), QStringLiteral("%1 (%2)").arg(name, level));
		refreshTable();
	});

	QObject::connect(delBtn, &QPushButton::clicked, [&]() {
		int row = table->currentRow();
		if (row < 0 || row >= accounts.size()) { showMsg("Select a user to delete."); return; }
		if (!passwordPromptCorrect()) return;

		if (QMessageBox::question(&dialog, "Delete User",
			QStringLiteral("Delete user '%1'?").arg(accounts[row].userName)) != QMessageBox::Yes) return;

		AuditLog::instance().log(QStringLiteral("USER_DELETE"), accounts[row].userName);
		accounts.removeAt(row);
		refreshTable();
	});

	QObject::connect(saveBtn, &QPushButton::clicked, [&]() {
		if (saveUserAccounts(accounts)) { AuditLog::instance().log(QStringLiteral("USER_ACCOUNTS_SAVED"), QStringLiteral("%1 users").arg(accounts.size())); showMsg("Users saved."); dialog.accept(); }
		else showMsg("Failed to save users!");
	});

	dialog.exec();
}

QString VisionApp::promptComboBox(QStringList items, QString title, QString msg)
{
	if (items.isEmpty()) return "";

	QInputDialog dialog;
	QString selection = items[0];
	
	dialog.setComboBoxItems(items);
	dialog.setWindowTitle(title);
	dialog.setLabelText(msg);

	QObject::connect(&dialog, &QInputDialog::textValueChanged, [=, &dialog, &selection] {
		selection = dialog.textValue();
	});

	if (dialog.exec()) {
		return selection;
	}

	return "";
}

QStringList VisionApp::promptFolderSelection(const QString& defaultPath, QWidget* parent)
{
	QFileDialog dialog(parent);
	dialog.setFileMode(QFileDialog::Directory);
	dialog.setOption(QFileDialog::ShowDirsOnly, true);
	dialog.setOption(QFileDialog::DontUseNativeDialog, true);

	// Set the default directory
	if (!defaultPath.isEmpty()) {
		dialog.setDirectory(defaultPath);
	}

	// Enable multiple folder selection
	QListView* listView = dialog.findChild<QListView*>("listView");
	if (listView) {
		listView->setSelectionMode(QAbstractItemView::ExtendedSelection);
	}
	QTreeView* treeView = dialog.findChild<QTreeView*>();
	if (treeView) {
		treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
	}

	QStringList folders;
	if (dialog.exec()) {
		folders = dialog.selectedFiles();
	}

	return folders;
}

bool VisionApp::promptQuestion(QString title, QString msg)
{
	auto msgBox = QMessageBox(QMessageBox::Information, title, msg, QMessageBox::Yes | QMessageBox::No);
	msgBox.setWindowModality(Qt::NonModal);
	if (QMessageBox::Yes == msgBox.exec()) return true;
	return false;
}

void VisionApp::executePathAssignment()
{
	setEditMode(EditMode::PATH_ASSIGNMENT);
	clearPath();
}

void VisionApp::redrawPath()
{
	QVector<QString> paths;
	for (int i = 0; i < ui.listWidget_paths->count(); i++) {
		auto id = ui.listWidget_paths->item(i)->whatsThis();
		paths.push_back(id);
	}

	clearPath();

	for (auto id : paths) {
		addViewToPath(id);
	}
}

void VisionApp::selectAllPath()
{
	for (int i = 0; i < ui.listWidget_paths->count(); i++) {
		ui.listWidget_paths->item(i)->setCheckState(Qt::Checked);
	}
}

void VisionApp::unselectAllPath()
{
	for (int i = 0; i < ui.listWidget_paths->count(); i++) {
		ui.listWidget_paths->item(i)->setCheckState(Qt::Unchecked);
	}
}

int VisionApp::numOfCheckedListItems(QListWidget * lists)
{
	int count = 0;
	for (int i = 0; i < lists->count(); i++) {
		auto item = lists->item(i);
		if (item->checkState() == Qt::Checked) {
			count++;
		}
	}
	return count;
}

void VisionApp::verifyUnassignedVisionObject()
{
	ct::logger::debug("verifyUnassignedVisionObject");
	for (auto vo : _visionObject) {
		if (vo.viewID == "") {
			ct::logger::debug("Vo Name: %s", vo.objectName.toStdString().c_str());
			ui.unassignedViewsFrame->show();
			return;
		}
	}
	ui.unassignedViewsFrame->hide();
}


void VisionApp::duplicateROI(Direction direction)
{
	TimeLogger tl;

	int count = ui.lineEdit_duplicateCount->text().toInt();
	float pitch = ui.lineEdit_duplicatePitchMM->text().toDouble(); 
	pitch = util::mm_to_px(pitch, ((ScaleManager::instance().horizontal_um_per_px() + ScaleManager::instance().vertical_um_per_px()) / 2));
	pitch = ScaleManager::instance().fov_to_world(pitch);

	std::vector<QDragBox*> _selectedROIs; //keep track selected ROIs the moment user click duplicate to avoid endless loop

	for (auto roi : _dragROI) {
		if (roi->isSelected()) {
			_selectedROIs.emplace_back(roi);
		}
	}

	for (auto roi : _selectedROIs) {
		QRectF rect = roi->getGeometry();
		auto w = rect.width();
		auto h = rect.height();

		for (int i = 0; i < count; i++) {
			if (direction == Direction::UP) {
				rect.setY(rect.y() - pitch);
				rect.setHeight(h);
			}
			else if (direction == Direction::DOWN) {
				rect.setY(rect.y() + pitch);
				rect.setHeight(h);
			}
			else if (direction == Direction::LEFT) {
				rect.setX(rect.x() - pitch);
				rect.setWidth(w);
			}
			else if (direction == Direction::RIGHT) {
				rect.setX(rect.x() + pitch);
				rect.setWidth(w);
			}

			auto id = addVisionObject(rect);

			if (!id.isEmpty())
			{
				_visionObject[id].pDragBox->setSelected(true);
			}

		}
	}

	refreshDragBoxSequence();
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);

	auto newBound = _pGraphicsSceneMain->sceneRect();

	//add margin, can remove this code if not needed
	auto margin = 0.1;
	auto widthOffset = _pGraphicsSceneMain->sceneRect().width() * margin;
	auto heightOffset = _pGraphicsSceneMain->sceneRect().height() * margin;
	newBound.setX(_pGraphicsSceneMain->sceneRect().x() - widthOffset);
	newBound.setY(_pGraphicsSceneMain->sceneRect().y() - heightOffset);
	newBound.setWidth(_pGraphicsSceneMain->sceneRect().width() + widthOffset);
	newBound.setHeight(_pGraphicsSceneMain->sceneRect().height() + heightOffset);
	//m_view->setSceneRect(newBound);
	//ui.graphicsViewMain->fitInView(newBound, Qt::KeepAspectRatio);

	saveRecipe();
}

void VisionApp::duplicateFromSelectedVisionObject()
{
	loadingBarSetup(QString("Searching Unit Object..."));

	TimeLogger tl;
	QImage qimg = _pixmapMain.toImage();

	bool _suppressBox = false;
	double _IoUthreshold = 0.1;

	for (int i = 0; i < _dragROI.count(); i++)
	{
		if (_dragROI.at(i)->isSelected() == true)
		{
			QImage croppedImage = qimg.copy(_dragROI.at(i)->x(), _dragROI.at(i)->y(), _dragROI.at(i)->getGeometry().width(), _dragROI.at(i)->getGeometry().height());

			MIL_ID mBuf = mtrx::to_milID(croppedImage);
			MIL_ID mMono = mtrx::to_mono(mBuf);
			mtrx::PatternInput input;
			mtrx::PatternOutput output;

			auto w = mtrx::get_width(mMono);
			auto h = mtrx::get_height(mMono);
			input.filename = "pat.pat";
			input.learn_x = 0;
			input.learn_y = 0;
			input.learn_w = w;
			input.learn_h = h;
			input.min_score = ui.lineEdit_similarityScore->text().toDouble();
			input.angle_step = ui.lineEdit_angleStep->text().toDouble();
			mtrx::learn_pattern(mMono, input, output);

			MIL_ID mBufH = mtrx::to_milID(qimg);
			MIL_ID mMonoH = mtrx::to_mono(mBufH);
			mtrx::PatternOutput outputH;

			MIL_INT numMatch = 0;
			MIL_ID patternCtx = M_NULL;
			MIL_ID patternResults = M_NULL;
			double finalScore = -1;


			patternCtx = MpatRestoreA(M_DEFAULT_HOST, input.filename.c_str(), M_NULL);
			patternResults = MpatAllocResult(M_DEFAULT_HOST, M_DEFAULT, M_NULL);
			MpatControl(patternCtx, M_ALL, M_NUMBER, M_ALL);

			MIL_DOUBLE modelSizeX = 0.0, modelSizeY = 0.0;
			MpatInquire(patternCtx, M_ALLOC_SIZE_X, &modelSizeX);
			MpatInquire(patternCtx, M_ALLOC_SIZE_Y, &modelSizeY);

			MpatPreprocess(patternCtx, M_DEFAULT, M_NULL);

			//add
			MpatFind(patternCtx, mMonoH, patternResults);
			MpatGetResult(patternResults, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &numMatch);

			const int size = 10000;
			MIL_DOUBLE posx[size], posy[size], score[size], angle[size];
			MpatGetResult(patternResults, M_POSITION_X, &posx);
			MpatGetResult(patternResults, M_POSITION_Y, &posy);
			MpatGetResult(patternResults, M_SCORE, &score);
			MpatGetResult(patternResults, M_ANGLE, &angle);

			for (int i = 0; i < numMatch; i++)
			{
				//offset
				int rot = angle[i], _width = modelSizeX, _height = modelSizeY;
				int nWidth, nHeight, offsetX, offsetY;
				if (rot >= 0 && rot <= 90)//quadrant 1
				{
					nWidth = _width * cos(rot * 3.14159265 / 180) + _height * sin(rot * 3.14159265 / 180); //where A is in radians
					nHeight = _width * sin(rot * 3.14159265 / 180) + _height * cos(rot * 3.14159265 / 180);
					offsetX = _width / 2 - nWidth / 2;
					offsetY = _height / 2 - nHeight / 2;
				}
				else if (rot >= 180 && rot <= 270) //quadrant 2
				{
					auto angle = rot - 180;
					nWidth = _width * cos(angle * 3.14159265 / 180) + _height * sin(angle * 3.14159265 / 180); //where A is in radians
					nHeight = _width * sin(angle * 3.14159265 / 180) + _height * cos(angle * 3.14159265 / 180);
					offsetX = _width / 2 - nWidth / 2;
					offsetY = _height / 2 - nHeight / 2;
				}
				else if (rot > 90 && rot < 180) //quadrant 3
				{
					auto angle = rot - 90;
					nWidth = _height * cos(angle * 3.14159265 / 180) + _width * sin(angle * 3.14159265 / 180); //where A is in radians
					nHeight = _height * sin(angle * 3.14159265 / 180) + _width * cos(angle * 3.14159265 / 180);
					offsetX = _width / 2 - nWidth / 2;
					offsetY = _height / 2 - nHeight / 2;
				}
				else if (rot > 270 && rot < 360) //quadrant 4
				{
					auto angle = rot - 270;
					nWidth = _height * cos(angle * 3.14159265 / 180) + _width * sin(angle * 3.14159265 / 180); //where A is in radians
					nHeight = _height * sin(angle * 3.14159265 / 180) + _width * cos(angle * 3.14159265 / 180);
					offsetX = _width / 2 - nWidth / 2;
					offsetY = _height / 2 - nHeight / 2;
				}

				outputH.x = posx[i] - nWidth / 2;
				outputH.y = posy[i] - nHeight / 2;
				outputH.w = nWidth;
				outputH.h = nHeight;
				outputH.angle = angle[i];
				finalScore = score[i];

				_suppressBox = false;
				auto TargetArea = ((outputH.w + 1) * (outputH.h + 1));

				for (int j = 0; j < _dragROI.count(); j++)
				{
					auto oldrectX = _dragROI[j]->getGeometry().x();
					auto oldrectY = _dragROI[j]->getGeometry().y();
					auto oldrectW = _dragROI[j]->getGeometry().width();
					auto oldrectH = _dragROI[j]->getGeometry().height();

					auto area = ((oldrectW + 1) * (oldrectH + 1));

					double x2 = fmin(outputH.x + outputH.w, oldrectX + oldrectW);
					double x1 = fmax(outputH.x, oldrectX);
					double y2 = fmin(outputH.y + outputH.h, oldrectY + oldrectH);
					double y1 = fmax(outputH.y, oldrectY);

					double iWidth = x2 - x1;
					if (iWidth < 0) iWidth = 0;
					double iHeight = y2 - y1;
					if (iHeight < 0) iHeight = 0;

					auto intersection = iWidth * iHeight;
					auto Union = TargetArea + area - intersection;
					auto IoU = intersection / Union;
					//qDebug() <<i<< "IoU" << IoU;
					if (IoU > _IoUthreshold) {
						_suppressBox = true;
						break;
					}
				}
				if (_suppressBox != true) addVisionObject(QRectF(outputH.x, outputH.y, outputH.w, outputH.h));
				//addVisionObject(QRectF(outputH.x, outputH.y, outputH.w, outputH.h));
			}

			if (patternCtx) MpatFree(patternCtx);
			if (patternResults) MpatFree(patternResults);

			mtrx::free_buffer(mBuf);
			mtrx::free_buffer(mMono);
			mtrx::free_buffer(mBufH);
			mtrx::free_buffer(mMonoH);

			break;
		}
	}
	tl.log_duration("Draw ROI", true);

	refreshDragBoxSequence();
	tl.log_duration("Refresh ROI sequence", true);
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	tl.log_duration("Update tree", true);

	saveRecipe();
	loadingBarRelease();
}

void VisionApp::duplicateFromSelectedVisionObject_cropped()
{
	loadingBarSetup(QString("Searching Unit Object partially..."));

	TimeLogger tl;
	QImage qimg = _pixmapMain.toImage();


	QPointF wpx = ScaleManager::instance().to_world_px(QPointF(_plane.corner_points[(int)Corner::FRONTLEFT].wx, _plane.corner_points[(int)Corner::FRONTLEFT].wy));
	QImage planeImg(Common::Directory::getRecipeImagesPath() + "plane.jpg");

	ui.graphicsViewMain->fitInView(_pPixmapItemMain, Qt::KeepAspectRatio);

	for (int k = 0; k < 2; k++)
	{
		QImage qimgCropped;
		if(k==0) qimgCropped = qimg.copy(wpx.x(), wpx.y(), planeImg.width(), planeImg.height() * 0.1);
		else if (k == 1) qimgCropped = qimg.copy(wpx.x(), wpx.y(), planeImg.width()* 0.1, planeImg.height());

		bool _suppressBox = false;
		double _IoUthreshold = 0.1;

		for (int i = 0; i < _dragROI.count(); i++)
		{
			if (_dragROI.at(i)->isSelected() == true)
			{
				QImage croppedImage = qimgCropped.copy(_dragROI.at(i)->x() - wpx.x(), _dragROI.at(i)->y() - wpx.y(), _dragROI.at(i)->getGeometry().width(), _dragROI.at(i)->getGeometry().height());

				MIL_ID mBuf = mtrx::to_milID(croppedImage);
				MIL_ID mMono = mtrx::to_mono(mBuf);
				mtrx::PatternInput input;
				mtrx::PatternOutput output;

				auto w = mtrx::get_width(mMono);
				auto h = mtrx::get_height(mMono);
				input.filename = "pat.pat";
				input.learn_x = 0;
				input.learn_y = 0;
				input.learn_w = w;
				input.learn_h = h;
				input.min_score = ui.lineEdit_similarityScore->text().toDouble();
				input.angle_step = ui.lineEdit_angleStep->text().toDouble();
				mtrx::learn_pattern(mMono, input, output);

				MIL_ID mBufH = mtrx::to_milID(qimgCropped);
				MIL_ID mMonoH = mtrx::to_mono(mBufH);
				mtrx::PatternOutput outputH;

				MIL_INT numMatch = 0;
				MIL_ID patternCtx = M_NULL;
				MIL_ID patternResults = M_NULL;
				double finalScore = -1;


				patternCtx = MpatRestoreA(M_DEFAULT_HOST, input.filename.c_str(), M_NULL);
				patternResults = MpatAllocResult(M_DEFAULT_HOST, M_DEFAULT, M_NULL);
				MpatControl(patternCtx, M_ALL, M_NUMBER, M_ALL);

				MIL_DOUBLE modelSizeX = 0.0, modelSizeY = 0.0;
				MpatInquire(patternCtx, M_ALLOC_SIZE_X, &modelSizeX);
				MpatInquire(patternCtx, M_ALLOC_SIZE_Y, &modelSizeY);

				MpatPreprocess(patternCtx, M_DEFAULT, M_NULL);

				//add
				MpatFind(patternCtx, mMonoH, patternResults);
				MpatGetResult(patternResults, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &numMatch);

				const int size = 10000;
				MIL_DOUBLE posx[size], posy[size], score[size], angle[size];
				MpatGetResult(patternResults, M_POSITION_X, &posx);
				MpatGetResult(patternResults, M_POSITION_Y, &posy);
				MpatGetResult(patternResults, M_SCORE, &score);
				MpatGetResult(patternResults, M_ANGLE, &angle);

				for (int i = 0; i < numMatch; i++)
				{
					//offset
					int rot = angle[i], _width = modelSizeX, _height = modelSizeY;
					int nWidth, nHeight, offsetX, offsetY;
					if (rot >= 0 && rot <= 90)//quadrant 1
					{
						nWidth = _width * cos(rot * 3.14159265 / 180) + _height * sin(rot * 3.14159265 / 180); //where A is in radians
						nHeight = _width * sin(rot * 3.14159265 / 180) + _height * cos(rot * 3.14159265 / 180);
						offsetX = _width / 2 - nWidth / 2;
						offsetY = _height / 2 - nHeight / 2;
					}
					else if (rot >= 180 && rot <= 270) //quadrant 2
					{
						auto angle = rot - 180;
						nWidth = _width * cos(angle * 3.14159265 / 180) + _height * sin(angle * 3.14159265 / 180); //where A is in radians
						nHeight = _width * sin(angle * 3.14159265 / 180) + _height * cos(angle * 3.14159265 / 180);
						offsetX = _width / 2 - nWidth / 2;
						offsetY = _height / 2 - nHeight / 2;
					}
					else if (rot > 90 && rot < 180) //quadrant 3
					{
						auto angle = rot - 90;
						nWidth = _height * cos(angle * 3.14159265 / 180) + _width * sin(angle * 3.14159265 / 180); //where A is in radians
						nHeight = _height * sin(angle * 3.14159265 / 180) + _width * cos(angle * 3.14159265 / 180);
						offsetX = _width / 2 - nWidth / 2;
						offsetY = _height / 2 - nHeight / 2;
					}
					else if (rot > 270 && rot < 360) //quadrant 4
					{
						auto angle = rot - 270;
						nWidth = _height * cos(angle * 3.14159265 / 180) + _width * sin(angle * 3.14159265 / 180); //where A is in radians
						nHeight = _height * sin(angle * 3.14159265 / 180) + _width * cos(angle * 3.14159265 / 180);
						offsetX = _width / 2 - nWidth / 2;
						offsetY = _height / 2 - nHeight / 2;
					}

					outputH.x = posx[i] - nWidth / 2 + +wpx.x();
					outputH.y = posy[i] - nHeight / 2+ +wpx.y();
					outputH.w = nWidth;
					outputH.h = nHeight;
					outputH.angle = angle[i];
					finalScore = score[i];

					_suppressBox = false;
					auto TargetArea = ((outputH.w + 1) * (outputH.h + 1));

					
					
					for (int j = 0; j < _dragROI.count(); j++)
					{
						auto oldrectX = _dragROI[j]->getGeometry().x();
						auto oldrectY = _dragROI[j]->getGeometry().y();
						auto oldrectW = _dragROI[j]->getGeometry().width();
						auto oldrectH = _dragROI[j]->getGeometry().height();

						auto area = ((oldrectW + 1) * (oldrectH + 1));

						double x2 = fmin(outputH.x + outputH.w, oldrectX + oldrectW);
						double x1 = fmax(outputH.x, oldrectX);
						double y2 = fmin(outputH.y + outputH.h, oldrectY + oldrectH);
						double y1 = fmax(outputH.y, oldrectY);

						double iWidth = x2 - x1;
						if (iWidth < 0) iWidth = 0;
						double iHeight = y2 - y1;
						if (iHeight < 0) iHeight = 0;

						auto intersection = iWidth * iHeight;
						auto Union = TargetArea + area - intersection;
						auto IoU = intersection / Union;
						//qDebug() <<i<< "IoU" << IoU;
						if (IoU > _IoUthreshold) {
							_suppressBox = true;
							break;
						}
					}
					if (_suppressBox != true)
					{
						QString objectkey = addVisionObject(QRectF(outputH.x , outputH.y , outputH.w, outputH.h), false);
					}
					
				}

				if (patternCtx) MpatFree(patternCtx);
				if (patternResults) MpatFree(patternResults);

				mtrx::free_buffer(mBuf);
				mtrx::free_buffer(mMono);
				mtrx::free_buffer(mBufH);
				mtrx::free_buffer(mMonoH);

				break;
			}
		}
	}

	


	tl.log_duration("Draw ROI", true);

	refreshDragBoxSequence();
	tl.log_duration("Refresh ROI sequence", true);
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	tl.log_duration("Update tree", true);

	saveRecipe();
	loadingBarRelease();
	
}

bool VisionApp::isPlaneValid(const QViewPlane & plane)
{
	auto br = plane.corner_points[(int)Corner::BACKRIGHT];
	auto fl = plane.corner_points[(int)Corner::FRONTLEFT];

	if (em::is_equal(br.wx, 0.0)) return false;
	if (em::is_equal(br.wy, 0.0)) return false;
	if (em::is_equal(fl.wy, 0.0)) return false;
	if (em::is_equal(fl.wy, 0.0)) return false;

	return true;
}

void VisionApp::generatePlane(QViewPlane& plane)
{
	if (!isPlaneValid(plane))
	{
		ct::logger::error("[warning] Invalid plane parameters.");
		return;
	}

	//convert all into horizontal and vertical axis
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	auto br = plane.corner_points[(int)Corner::BACKRIGHT];
	auto fl = plane.corner_points[(int)Corner::FRONTLEFT];

	//Get FOV
	double h_cam_mm = util::px_to_mm(CAMManager::instance().getWidth(_camID), h_scale);
	double v_cam_mm = util::px_to_mm(CAMManager::instance().getHeight(_camID), v_scale);

	//get corner as point
	br.wx = br.wx + h_cam_mm/2;
	br.wy = br.wy + v_cam_mm/2;

	fl.wx = fl.wx - h_cam_mm/2;
	fl.wy = fl.wy - v_cam_mm/2;

	//Get Target Size to fill up
	double h_target_mm = abs(br.wx - fl.wx);
	double v_target_mm = abs(br.wy - fl.wy);

	//Set minimum overlap percentage
	double overlap_percentage = 5;
	double h_percentage = (double)overlap_percentage / 100.0;
	double v_percentage = (double)overlap_percentage / 100.0;
	double h_overlap_mm = h_cam_mm * h_percentage;
	double v_overlap_mm = v_cam_mm * v_percentage;

	/*
	* n: Minimum number of FOV
	* t: Target size in mm
	* o: Minimum overlap in mm
	* f: FOV size in mm
	*
	* To get minimum number of FOV
	* n = ceil((t - o)/(f - o))
	*
	* To get minimum overlap in mm
	* o = (n * f - t) / n - 1
	*/
	//Calculate minimum number of FOV required
	int h_num = std::ceil((h_target_mm - h_overlap_mm) / (h_cam_mm - h_overlap_mm));
	int v_num = std::ceil((v_target_mm - v_overlap_mm) / (v_cam_mm - v_overlap_mm));

	//Recalculate minimum overlap based on minimum number of FOV
	h_overlap_mm = ((h_num * h_cam_mm) - h_target_mm) / (h_num - 1);
	v_overlap_mm = ((v_num * v_cam_mm) - v_target_mm) / (v_num - 1);

	//Handle target that is smaller than FOV
	double h_centerOffset_mm = 0.0;
	if (h_num == 1) {
		h_overlap_mm = 0.0;
		h_centerOffset_mm = (h_target_mm / 2) - (h_cam_mm / 2);
	}

	double v_centerOffset_mm = 0.0;
	if (v_num == 1) {
		v_overlap_mm = 0.0;
		v_centerOffset_mm = (v_target_mm / 2) - (v_cam_mm / 2);
	}

	//Calculate each iteration's offset
	double h_offset_mm = h_cam_mm - h_overlap_mm;
	double v_offset_mm = v_cam_mm - v_overlap_mm;

	ct::logger::trace("h_num: %d", h_num);
	ct::logger::trace("v_num: %d", v_num);

	ct::logger::trace("h_overlap: %f", h_overlap_mm);
	ct::logger::trace("v_overlap: %f", v_overlap_mm);

	ct::logger::trace("h_offset: %f", h_offset_mm);
	ct::logger::trace("v_offset: %f", v_offset_mm);

	ct::logger::trace("h_cam_mm: %f", h_cam_mm);
	ct::logger::trace("v_cam_mm: %f", v_cam_mm);

	ct::logger::trace("h_target_mm: %f", h_target_mm);
	ct::logger::trace("v_target_mm: %f", v_target_mm);

	ct::logger::trace("h_centerOffset_mm: %f", h_centerOffset_mm);
	ct::logger::trace("v_centerOffset_mm: %f", v_centerOffset_mm);

	plane.width_px = ScaleManager::instance().fov_to_world(util::mm_to_px(h_target_mm, h_scale));
	plane.height_px = ScaleManager::instance().fov_to_world(util::mm_to_px(v_target_mm, v_scale));
	plane.width_mm = h_target_mm;
	plane.height_mm = v_target_mm;
	plane.horizontal_overlap_percentage = h_percentage;
	plane.vertical_overlap_percentage = v_percentage;
	plane.horizontal_overlap_mm = h_overlap_mm;
	plane.vertical_overlap_mm = v_overlap_mm;
	plane.horizontal_overlap_px = ScaleManager::instance().fov_to_world(util::mm_to_px(h_overlap_mm, h_scale));
	plane.vertical_overlap_px = ScaleManager::instance().fov_to_world(util::mm_to_px(v_overlap_mm, v_scale));
	plane.horizontal_num = h_num;
	plane.vertical_num = v_num;

	plane.views.clear();

	for (int r = 0; r < v_num; r++) {
		for (int c = 0; c < h_num; c++) {
			QString id = QString("R%1C%2").arg(r).arg(c);
			QView v;
			v.id = id;
			v.name = id;
			v.world.wx = h_offset_mm * c + fl.wx + (h_cam_mm / 2) + h_centerOffset_mm;
			v.world.wy = fl.wy + (v_cam_mm / 2) + (v_offset_mm * r) + v_centerOffset_mm;
			v.world.wz = fl.wz;

			auto wpx = getAbsoluteFOVCoordinates(QPointF(v.world.wx, v.world.wy));
			v.px.cx = ScaleManager::instance().fov_to_world(util::mm_to_px(v.world.wx, h_scale));
			v.px.cy = ScaleManager::instance().fov_to_world(util::mm_to_px(v.world.wy, v_scale));
			v.px.w = ScaleManager::instance().fov_to_world(CAMManager::instance().getWidth(_camID));
			v.px.h = ScaleManager::instance().fov_to_world(CAMManager::instance().getHeight(_camID));
			v.px.compute_extremum();

			plane.views.push_back(v);
		}
	}

	savePlane();

	return;
}

void VisionApp::collectPlaneViews(const QViewPlane& plane)
{
	auto rootPath = Common::Directory::getRecipeImagesPath() + "PlaneImages/";
	_jobThread.setRootPath(rootPath);
	QMetaObject::invokeMethod(&_jobThread, "runPlaneCollection", Qt::QueuedConnection);
}

void VisionApp::stitchPlaneImage(const QViewPlane & plane)
{
	progressBarSetup("Stitching plane image...", 0);

	//Ammend: Not needed anymore, only using z for global passing, can improve this code
	_currentOriginInMM = plane.corner_points[(int)Corner::FRONTLEFT];

	//convert all into horizontal and vertical axis
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	//Get FOV
	double h_cam_mm = util::px_to_mm(CAMManager::instance().getWidth(_camID), h_scale);
	double v_cam_mm = util::px_to_mm(CAMManager::instance().getHeight(_camID), v_scale);

	//get corner as point
	auto topleft_x_mm = _currentOriginInMM.wx - h_cam_mm / 2;
	auto topleft_y_mm = _currentOriginInMM.wy - v_cam_mm / 2;

	//convert to wpx
	QPointF frontLeft_wpx = ScaleManager::instance().to_world_px(QPointF(topleft_x_mm, topleft_y_mm));

	//convert FOV px format to world px format
	auto w_plane_width_px = plane.width_px;
	auto w_plane_height_px = plane.height_px;

	QImage _imagePlane = QImage(w_plane_width_px, w_plane_height_px, QImage::Format_RGB32);
	_imagePlane.fill(Qt::transparent);
	printf("Plane size: %f, %f\n", w_plane_width_px, w_plane_height_px);

	QPainter painter(&_imagePlane);

	for (const auto& v : plane.views) {
		//load image
		auto root = Common::Directory::getRecipeImagesPath() + "PlaneImages\\";
		auto path = root + v.id + ".jpg";
		QImage qimg;

		if (!qimg.load(path)) {
			printf("[UB] {collectPlaneViews} Failed to load image: %s\n", path.toStdString().c_str());
			return;
		}

		auto w_view_width_px = v.px.w;
		auto w_view_height_px = v.px.h;

		auto wpx = ScaleManager::instance().to_world_px(QPointF(v.world.wx - _currentOriginInMM.wx, v.world.wy - _currentOriginInMM.wy));
		auto w_view_cx = wpx.x();// - w_view_width_px / 2;
		auto w_view_cy = wpx.y();// -w_view_height_px / 2;

		qimg = qimg.convertToFormat(QImage::Format_ARGB32);

		int width = qimg.width();
		int height = qimg.height();

		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				// Get the ARGB pixel value at (x, y)
				QRgb pixel = qimg.pixel(x, y);

				// Get the current alpha value
				int alpha = qAlpha(pixel);

				// Set the new alpha value from the unsigned char array
				alpha = _camAlpha[y * width + x];

				// Create a new pixel value with updated alpha
				pixel = qRgba(qRed(pixel), qGreen(pixel), qBlue(pixel), alpha);

				// Set the updated pixel value in the image
				qimg.setPixel(x, y, pixel);
			}
		}

		QImage scaled = qimg.scaled(w_view_width_px, w_view_height_px, Qt::AspectRatioMode::KeepAspectRatio, Qt::TransformationMode::SmoothTransformation);
		QPoint destPos = QPoint(w_view_cx, w_view_cy);
		painter.drawImage(destPos, scaled);
	}

	painter.end();
	_imagePlane.save(Common::Directory::getRecipeImagesPath() + "plane.jpg");

	_imageWorld.fill(Qt::black);
	QPainter wpainter(&_imageWorld);
	wpainter.drawImage(frontLeft_wpx, _imagePlane);

	displayImage(_imageWorld);

	progressBarRelease();
}

void VisionApp::initPathSM()
{
	QState* state_waitViewSelection = new QState();
	QFinalState* state_final = new QFinalState();

	state_waitViewSelection->addTransition(this, &VisionApp::viewBoxPressed, state_final);

	_pathSM.addState(state_waitViewSelection);
	_pathSM.addState(state_final);
	_pathSM.setInitialState(state_waitViewSelection);

	connect(&_pathSM, &QStateMachine::started, [this]() {
	});

	connect(&_pathSM, &QStateMachine::stopped, [this]() {
	});

	QObject::connect(state_waitViewSelection, &QState::entered, [this]() {
	});

	QObject::connect(state_waitViewSelection, &QState::exited, [this]() {

		if (!_views.contains(_lastViewPressed)) {
			ct::logger::error("[JobThread] Failed to wait view selection signal. Invalid view ID: %s", _lastViewPressed.toStdString().c_str());
			return;
		}

		auto& v = _views[_lastViewPressed];

		if (_currentSetPoint == "start") {
			ui.tb_setStartPoint->setText(v.name);
			ui.tb_setStartPoint->setWhatsThis(v.id);
		}
		else {
			ui.tb_setEndPoint->setText(v.name);
			ui.tb_setEndPoint->setWhatsThis(v.id);
		}
	});
}

void VisionApp::addViewToPath(QString viewID)
{
	if (viewID == "") return;

	for (int i = 0; i < ui.listWidget_paths->count(); i++) {
		auto id = ui.listWidget_paths->item(i)->whatsThis();
		if (id == viewID) {
			return;
		}
	}

	if (!_views.contains(viewID)) {
		ct::logger::error("[JobThread] Failed to add view to path. Invalid view ID: %s", viewID.toStdString().c_str());
		return;
	}

	auto& v = _views[viewID];

	if (v.type == ct::s_stitch_view) return;

	auto viewROI = v.pDragBox;
	if (viewROI == nullptr) {
		printf("[UB] {Add view to path} Invalid view ID.\n");
		return;
	}

	auto wpx = ScaleManager::instance().fov_to_world(v.px);

	viewROI->setBorderColor(getColor(Representation::ASSIGNED_VIEW));
	auto fontSize = wpx.w / 10;
	auto txt = drawText(QString::number(ui.listWidget_paths->count()), QPointF(wpx.cx, wpx.cy), QColor(Qt::red), fontSize);
	excludeFromRenderedShape(txt);
	txt->setZValue((int)UIHierarchy::SHAPE);

	QFont font;
	font.setPointSize(fontSize);
	txt->setFont(font);
	_pathGraphicItems.push_back(txt);

	//connect line with previous path
	if (_views.contains(_lastViewAddedToPath)) {
		auto prev_v = _views[_lastViewAddedToPath];
		auto prev_wpx = ScaleManager::instance().fov_to_world(prev_v.px);

		auto line = drawLine(QLine(wpx.cx, wpx.cy, prev_wpx.cx, prev_wpx.cy), QColor(Qt::red), 8);
		excludeFromRenderedShape(line);

		line->setZValue((int)UIHierarchy::SHAPE);
		_pathGraphicItems.push_back(line);
	}

	QListWidgetItem* item = new QListWidgetItem();
	item->setWhatsThis(v.id);
	item->setText(v.name);
	item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
	item->setCheckState(Qt::Checked);
	ui.listWidget_paths->addItem(item);
	_lastViewAddedToPath = v.id;

	//if (ui.listWidget_paths->count() == numOfCheckedListItems(ui.listWidget_viewSelection)) {
	if (ui.listWidget_paths->count() == _views.size()) {
		setEditMode(EditMode::SELECT);

		//if (ui.tb_setStartPoint->text() == "" && ui.listWidget_paths->count() > 0) {
		auto startItem = ui.listWidget_paths->item(0);
		auto endItem = ui.listWidget_paths->item(ui.listWidget_paths->count() - 1);

		ui.tb_setStartPoint->setText(startItem->text());
		ui.tb_setStartPoint->setWhatsThis(startItem->whatsThis());

		ui.tb_setEndPoint->setText(endItem->text());
		ui.tb_setEndPoint->setWhatsThis(endItem->whatsThis());
		QGuiApplication::processEvents();
		//}
	}

	_pGraphicsSceneMain->update();
}

void VisionApp::setViewSelectionCheckState(Qt::CheckState state)
{
	for (int i = 0; i < ui.listWidget_viewSelection->count(); i++) {
		auto item = ui.listWidget_viewSelection->item(i);
		if (item->isSelected()) {
			item->setCheckState(state);
		}
	}
}

void VisionApp::initManualScalingSM()
{
	QState* state_origin = new QState();
	QState* state_horizontal = new QState();
	QState* state_vertical = new QState();
	QFinalState* state_final = new QFinalState();

	state_origin->addTransition(ui.graphicsViewFOV, &QMainGraphicsView::mousePress, state_horizontal);
	state_horizontal->addTransition(ui.graphicsViewFOV, &QMainGraphicsView::mousePress, state_vertical);
	state_vertical->addTransition(ui.graphicsViewFOV, &QMainGraphicsView::mousePress, state_final);

	_manualScalingSM.addState(state_origin);
	_manualScalingSM.addState(state_horizontal);
	_manualScalingSM.addState(state_vertical);
	_manualScalingSM.addState(state_final);
	_manualScalingSM.setInitialState(state_origin);

	connect(&_manualScalingSM, &QStateMachine::started, [this]() {
		_cursor.setShape(Qt::CrossCursor);
		ui.graphicsViewFOV->setCursor(_cursor);
	});

	connect(&_manualScalingSM, &QStateMachine::stopped, [this]() {
		_cursor.setShape(Qt::ArrowCursor);
		ui.graphicsViewFOV->setCursor(_cursor);
	});

	QObject::connect(state_origin, &QState::entered, [this]() {
		_manualScalingInfo = {};

		snapImage(_mainOptics[_camID], "", "");
		auto ret = QMessageBox::question(this, tr("Scaling"), tr("Select a feature to track"), QMessageBox::Ok | QMessageBox::Cancel);
		if (ret == QMessageBox::Cancel) {
			_manualScalingSM.stop();
		}
	});

	QObject::connect(state_origin, &QState::exited, [this]() {
		_manualScalingInfo.origin_pos = _lastMousePressPos;
	});

	QObject::connect(state_horizontal, &QState::entered, [this]() {
		double step_mm = ui.lineEdit_scalingStep->text().toDouble();
		bool is_right = true;

		//if (!jogRight(step_mm)) {
		//	is_right = false;
		//	if (!jogLeft(step_mm)) {
		//		showMsg(tr(qPrintable("Failed to jog horizontally!"))); //errMsg
		//	}
		//}

		snapImage(_mainOptics[_camID], "", "");

		auto ret = QMessageBox::question(this, tr("Scaling"), tr("Click on the feature"), QMessageBox::Ok | QMessageBox::Cancel);
		if (ret == QMessageBox::Cancel) {
			_manualScalingSM.stop();
		}
	});

	QObject::connect(state_horizontal, &QState::exited, [this]() {
		double step_mm = ui.lineEdit_scalingStep->text().toDouble();

		auto& m = _manualScalingInfo;
		m.horizontal_pos = _lastMousePressPos;
		//_worldEnv.horizontal_scale = step_mm * 1000 / abs(m.origin_pos.x() - m.horizontal_pos.x());

		//if (true) { //if jog right
		//	jogLeft(step_mm);
		//}
		//else {
		//	jogRight(step_mm);
		//}
	});

	QObject::connect(state_vertical, &QState::entered, [this]() {
		double step_mm = ui.lineEdit_scalingStep->text().toDouble();
		bool is_front = true;

		//if (!jogFront(step_mm)) {
		//	is_front = false;
		//	if (!jogBack(step_mm)) {
		//		showMsg(tr(qPrintable("Failed to jog vertically!"))); //errMsg
		//	}
		//}

		snapImage(_mainOptics[_camID], "", "");

		auto ret = QMessageBox::question(this, tr("Scaling"), tr("Click on the feature"), QMessageBox::Ok | QMessageBox::Cancel);
		if (ret == QMessageBox::Cancel) {
			_manualScalingSM.stop();
		}
	});

	QObject::connect(state_vertical, &QState::exited, [this]() {
		double step_mm = ui.lineEdit_scalingStep->text().toDouble();

		auto& m = _manualScalingInfo;
		m.vertical_pos = _lastMousePressPos;
		saveWorldEnv();

		showMsg(QStringLiteral("Scaling: %1, %2\n").arg(ScaleManager::instance().horizontal_um_per_px()).arg(ScaleManager::instance().vertical_um_per_px()));
		_manualScalingSM.stop();


		//if (true) { //if jog front
		//	jogBack(step_mm);
		//}
		//else {
		//	jogFront(step_mm);
		//}
	});
}

void VisionApp::displayDummyImage(int w, int h)
{
	//capped at 1000
	int cap = 1000;
	if (w < cap) w = cap;
	if (h < cap) h = cap;

	_dummyImage = QImage(w, h, QImage::Format_RGB32);
	_dummyImage.fill(Qt::black);
	QPainter painter(&_dummyImage);
	QPen pen;
	pen.setColor(Qt::red);
	QBrush brush;
	brush.setColor(Qt::red);
	painter.setPen(pen);
	painter.setBrush(brush);
	QPainterPath path;
	QPainterPath pathFid;

	int r = 30;

	for (int x = 100; x < w; x += 285) {
		for (int y = 100; y < h; y += 285) {
			path.addRect(QRectF(x + 50, y + 50, 100, 100));
			path.addRect(QRectF(x + 160, y + 160, 100, 100));

			pathFid.addEllipse(x + 5, y + 10, r, r);
			//path.addEllipse(x + 25, y + 285, r, r);
			//path.addEllipse(x + 285, y + 25, r, r);
			//path.addEllipse(x + 285, y + 285, r, r);
		}
	}

	painter.fillPath(path, Qt::red);
	painter.drawPath(path);
	painter.fillPath(pathFid, Qt::yellow);
	painter.drawPath(pathFid);
	painter.end();

	displayImage(_dummyImage);
}

void VisionApp::recordMemory(QString msg)
{
	double workingSetSizeInMB = 0.0;


	auto byteToMB = [](DWORDLONG value) -> double {
		double dvalue = static_cast<double>(value) / (1024 * 1024);
		return dvalue;
	};

	auto sbyteToMB = [](SIZE_T value) -> double {
		double dvalue = static_cast<double>(value) / (1024 * 1024);
		return dvalue;
	};

	// Get a handle to the current process
	HANDLE processHandle = GetCurrentProcess();

	MEMORYSTATUSEX memInfo;
	memInfo.dwLength = sizeof(MEMORYSTATUSEX);
	GlobalMemoryStatusEx(&memInfo);


	// Query process memory information
	PROCESS_MEMORY_COUNTERS pmc;
	if (GetProcessMemoryInfo(processHandle, &pmc, sizeof(pmc))) {
		SIZE_T workingSetSize = pmc.WorkingSetSize;
		workingSetSizeInMB = sbyteToMB(workingSetSize);

		DWORDLONG virtualMemUsed = memInfo.ullTotalPageFile - memInfo.ullAvailPageFile;
		auto virtualMemUsedMB = byteToMB(virtualMemUsed);

		qDebug() << "Virtual memory: " << virtualMemUsedMB;
	}
	else {
		qDebug() << "Failed to retrieve memory info";
	}

	/*std::ofstream fout("test/loop.txt", std::ios::app);
	fout << loopIndex << ": " << workingSetSizeInMB << std::endl;
	fout.close();*/

	ui.textEdit_loopStatus->append(QString("%1: %2MB").arg(msg).arg(workingSetSizeInMB));
	processEvents();
}

void VisionApp::resetLoopFlags()
{
	ui.checkBox_runLooping->setChecked(false);
	ui.checkBox_runOneFOVonly->setChecked(false);
	_enable2D = !ui.checkBox_disable2D->isChecked();
	_enable3D = ui.checkBox_enable3D->isChecked();
	_enableVisionObjectSampling = ui.checkBox_enableVisionObjectSampling->isChecked();
}

void VisionApp::initMovieIcons()
{
	_movieShowLineScan = new QMovie(":/8Icon/Icon/icon8/icons8-two-sided-scanning.gif");
	connect(_movieShowLineScan, &QMovie::frameChanged, this, [=](int frame) {
		ui.toolButton_showLineScan->setIcon(QIcon(_movieShowLineScan->currentPixmap()));
	});

	// if movie doesn't loop forever, force it to.
	if (_movieShowLineScan->loopCount() != -1) connect(_movieShowLineScan, SIGNAL(finished()), _movieShowLineScan, SLOT(start()));
	_movieShowLineScan->start();
}

void VisionApp::log(std::string msg, const dat::WorldCoordinate& w)
{
	ct::logger::debug("%s: %f, %f, %f", msg.c_str(), w.wx, w.wy, w.wz);
}

QPointF VisionApp::getCenterPointFrom4Side(QRectF rect)
{
	int w = rect.width();
	int h = rect.height();
	int x = rect.x();
	int y = rect.y();

	auto img = mtrx::to_milID(_imageFOV);

	MIL_ID mono = M_NULL;
	if (mtrx::is_color(img)) {
		mono = mtrx::to_mono(img);
	}
	else {
		mono = mtrx::alloc_buffer(img);
		MbufCopy(img, mono);
	}

	auto roi = MbufAlloc2d(M_DEFAULT_HOST, w, h, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
	MbufCopyColor2d(mono, roi, M_ALL_BANDS, x, y, M_ALL_BANDS, 0, 0, w, h);

	mtrx::BufferCollector bc1(img);
	mtrx::BufferCollector bc2(mono);
	mtrx::BufferCollector bc3(roi);

	auto _mMarker = MmeasAllocMarker(M_DEFAULT, M_STRIPE, M_DEFAULT, M_NULL);

	MmeasSetMarker(_mMarker, M_SUB_REGIONS_NUMBER, M_DEFAULT, M_NULL);
	MmeasSetMarker(_mMarker, M_EDGEVALUE_MIN, M_DEFAULT, M_NULL);

	// Remove strength score function.
	MmeasSetScore(_mMarker, M_STRENGTH_SCORE, 0, 0, M_MAX_POSSIBLE_VALUE, M_MAX_POSSIBLE_VALUE, M_DEFAULT, M_DEFAULT, M_DEFAULT);

	MmeasSetMarker(_mMarker, M_NUMBER, 1, M_NULL);

	// Set the width score function to find the widest stripe.
	MmeasSetScore(_mMarker, M_STRIPE_WIDTH_SCORE, 0, 0, 300, M_MAX_POSSIBLE_VALUE, M_DEFAULT, M_DEFAULT, M_DEFAULT);

	auto getCenter = [&](MIL_INT64 orientation) -> double {
		MmeasSetMarker(_mMarker, M_ORIENTATION, orientation, M_NULL);
		if (orientation == M_VERTICAL) {
			MmeasSetMarker(_mMarker, M_POLARITY, M_NEGATIVE, M_POSITIVE);
		}
		else {
			MmeasSetMarker(_mMarker, M_POLARITY, M_NEGATIVE, M_POSITIVE);
		}

		MmeasFindMarker(M_DEFAULT, roi, _mMarker, M_DEFAULT);

		MIL_DOUBLE num;
		MmeasGetResult(_mMarker, M_NUMBER, &num, M_NULL);
		ct::logger::debug("Num line: %f", num);
		double center = 0.0;

		if (num > 0) {
			MIL_DOUBLE* angle = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge1_x = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge1_y = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge2_x = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge2_y = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge_width = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge_str = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge_contrast = new MIL_DOUBLE[num];
			MIL_DOUBLE* length = new MIL_DOUBLE[num];
			MIL_DOUBLE* score = new MIL_DOUBLE[num];
			MIL_DOUBLE* stripe_width = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge_start_x = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge_start_y = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge_end_x = new MIL_DOUBLE[num];
			MIL_DOUBLE* edge_end_y = new MIL_DOUBLE[num];
			MIL_DOUBLE* orientation = new MIL_DOUBLE[num];

			MmeasGetResult(_mMarker, M_ANGLE, angle, M_NULL);
			MmeasGetResult(_mMarker, M_POSITION + M_EDGE_FIRST, edge1_x, edge1_y);
			MmeasGetResult(_mMarker, M_POSITION + M_EDGE_SECOND, edge2_x, edge2_y);
			MmeasGetResult(_mMarker, M_EDGE_WIDTH, edge_width, M_NULL);
			MmeasGetResult(_mMarker, M_EDGE_STRENGTH, edge_str, M_NULL);
			MmeasGetResult(_mMarker, M_EDGE_CONTRAST, edge_contrast, M_NULL);
			MmeasGetResult(_mMarker, M_EDGE_START, edge_start_x, edge_start_y);
			MmeasGetResult(_mMarker, M_EDGE_END, edge_end_x, edge_end_y);
			MmeasGetResult(_mMarker, M_LENGTH, length, M_NULL);
			MmeasGetResult(_mMarker, M_SCORE, score, M_NULL);
			MmeasGetResult(_mMarker, M_STRIPE_WIDTH, stripe_width, M_NULL);
			MmeasGetResult(_mMarker, M_ORIENTATION, orientation, M_NULL);

			for (int i = 0; i < num; i++) {
				auto e_w = abs(edge1_x[i] - edge2_x[i]);
				auto e_h = abs(edge1_y[i] - edge2_y[i]);

				QLineF line1, line2;
				double distance = 0.0;

				if (orientation[i] == M_VERTICAL) {
					ct::logger::debug("Edge1: %f, %f", edge1_x[i], edge1_y[i]);
					ct::logger::debug("Edge2: %f, %f", edge2_x[i], edge2_y[i]);
					line1 = QLineF(edge1_x[i] + x, 0 + y, edge1_x[i] + x, h + y);
					line2 = QLineF(edge2_x[i] + x, 0 + y, edge2_x[i] + x, h + y);
					//drawRect(QRectF(edge1_x[i] + roi.x(), edge1_y[i] + roi.y(), e_w, 20), Qt::blue, QStringLiteral("Length below lower limit"));
					center = abs(line1.x1() + line2.x1()) / 2;
					ct::logger::debug("Vertical: %f, %f", line1.x1(), line2.x1());
				}
				else {
					ct::logger::debug("Edge1: %f, %f", edge1_x[i], edge1_y[i]);
					ct::logger::debug("Edge2: %f, %f", edge2_x[i], edge2_y[i]);
					line1 = QLineF(0 + x, edge1_y[i] + y, w + x, edge1_y[i] + y);
					line2 = QLineF(0 + x, edge2_y[i] + y, w + x, edge2_y[i] + y);
					//drawRect(QRectF(edge1_x[i] + x, edge1_y[i] + y, 20, e_h), Qt::yellow, QStringLiteral("Length below lower limit"));
					center = abs(line1.y2() + line2.y1()) / 2;
					ct::logger::debug("Horizontal: %f, %f", line1.y1(), line2.y1());
				}

				//drawLine(line1, Qt::blue, 1);
				//drawLine(line2, Qt::blue, 1);
			}

			delete[] angle;
			delete[] edge1_x;
			delete[] edge1_y;
			delete[] edge2_x;
			delete[] edge2_y;
			delete[] edge_width;
			delete[] edge_str;
			delete[] edge_contrast;
			delete[] length;
			delete[] score;
			delete[] stripe_width;
			delete[] edge_start_x;
			delete[] edge_start_y;
			delete[] edge_end_x;
			delete[] edge_end_y;
			delete[] orientation;
		}
		return center;
	};

	QPointF c;
	c.setX(getCenter(M_VERTICAL));
	c.setY(getCenter(M_HORIZONTAL));
	ct::logger::debug("Center: %f, %f", c.x(), c.y());

	if (_mMarker != M_NULL) {
		MmeasFree(_mMarker);
		_mMarker = M_NULL;
	}

	return c;
}

MIL_ID VisionApp::getCameraMilMono()
{
	auto img = mtrx::to_milID(_imageFOV);

	if (mtrx::is_color(img)) {
		auto mono = mtrx::to_mono(img);
		mtrx::free_buffer(img);
		return mono;
	}
	else {
		return img;
	}
}

std::string VisionApp::generateTimeStampID()
{
	auto now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);

	std::tm tm_now;
#ifdef _WIN32
	localtime_s(&tm_now, &now_c);
#else
	localtime_r(&now_c, &tm_now);
#endif

	std::ostringstream oss;
	oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
	return oss.str();  // Example output: "20250630_212045"
}

void VisionApp::processUtilityInfo()
{
	int num = 0;

	std::vector<VisionAppQDragBox*> selectedROI;
	for (int i = 0; i < _dragROI.count(); i++)
	{
		if (_dragROI.at(i)->isSelected() == true)
		{
			selectedROI.emplace_back(_dragROI.at(i));
			num++;
		}
	}

	_utilityInfo.num = num;

	if (num == 1) {
		auto bc = selectedROI.at(0)->getGeometry().center();
		_utilityInfo.center.x() = bc.x();
		_utilityInfo.center.y() = bc.y();
	}
	else if (num == 2) {
		VisionAppQDragBox* box1 = selectedROI.at(0);
		VisionAppQDragBox* box2 = selectedROI.at(1);

		if (box1 != nullptr && box2 != nullptr) {

			auto bc1 = box1->getGeometry().center();
			auto bc2 = box2->getGeometry().center();

			//auto dif_x = abs(box1->getGeometry().center().x() - box2->getGeometry().center().x());
			//auto dif_y = abs(box1->getGeometry().center().y() - box2->getGeometry().center().y());

			auto v1 = em::Vertex(bc1.x(), bc1.y());
			auto v2 = em::Vertex(bc2.x(), bc2.y());
			auto V1 = em::V2d(bc1.x(), bc1.y());
			auto V2 = em::V2d(bc2.x(), bc2.y());

			_utilityInfo.distance = em::distance(v1, v2);

			auto angle = em::to_degree(atan((bc2.y() - bc1.y()) / (bc2.x() - bc1.x())));
			_utilityInfo.angle = angle;
		}
	}
	else if (num > 2) {
		_utilityInfo.distance = 0.0;

		auto x_mean = 0.0;
		auto y_mean = 0.0;

		_utilityInfo.distance = 0.0;
		for (int i = 0; i < selectedROI.size(); i++) {

			auto& fi = selectedROI[i];

			//calculate average distance

			for (int j = 0; j < selectedROI.size(); j++) {
				if (i == j) continue;

				auto& fj = selectedROI[j];

				auto distance = em::distance(fi->getGeometry().center().x(), fi->getGeometry().center().y(), fj->getGeometry().center().x(), fj->getGeometry().center().y());

				if (_utilityInfo.distance < distance) {
					_utilityInfo.distance = distance;
				}
			}

			////calculate average distance
			//double closest_distance = 9999999999.99999;

			//for (int j = 0; j < selectedROI.size(); j++) {
			//	if (i == j) continue;

			//	auto& fj = selectedROI[j];

			//	auto distance = em::distance(fi->getGeometry().center().x(), fi->getGeometry().center().y(), fj->getGeometry().center().x(), fj->getGeometry().center().y());

			//	if (closest_distance > distance) {
			//		closest_distance = distance;
			//	}
			//}

			//_utilityInfo.distance += closest_distance;

			//mean of x and y
			x_mean += fi->getGeometry().center().x();
			y_mean += fi->getGeometry().center().y();
		}

		//_utilityInfo.distance /= selectedROI.size();
		_utilityInfo.distance /= (selectedROI.size() - 1);


		//line of best fit 
		//m = (xi - x_mean)(yi - y_mean) / (xi - x_mean)^2
		x_mean /= selectedROI.size();
		y_mean /= selectedROI.size();

		//a = sum of (xi - x_mean)(yi - y_mean)
		//b = sum of  (xi - x_mean)^2
		double a = 0.0, b = 0.0;
		double m = 0.0;

		for (auto s : selectedROI) {
			auto x = s->getGeometry().center().x();
			auto y = s->getGeometry().center().y();

			auto x_minus_mean = x - x_mean;
			a += x_minus_mean * (y - y_mean);
			b += pow(x_minus_mean, 2);
		}

		//get slope
		m = a / b;
		_utilityInfo.angle = em::to_degree(atan(m));
	}

	updateUtilityInfoUI();
}

void VisionApp::updateUtilityInfoUI()
{
	const auto& u = _utilityInfo;

	ui.textEditInfo->clear();
	ui.textEditInfo->append(QStringLiteral("Count: %1").arg(u.num));
	ui.textEditInfo->append(QStringLiteral("Center (x,y): %1, %2").arg(u.center.x()).arg(u.center.y()));
	ui.textEditInfo->append(QStringLiteral("Distance: %1wpx, %2mm").arg(u.distance).arg(ScaleManager::instance().to_mm(ScaleManager::instance().world_to_fov(u.distance))));
	ui.textEditInfo->append(QStringLiteral("Angle: %1").arg(u.angle));

	ui.textEdit_utility->clear();
	ui.textEdit_utility->append(QStringLiteral("Count: %1").arg(u.num));
	ui.textEdit_utility->append(QStringLiteral("Center (x,y): %1, %2").arg(u.center.x()).arg(u.center.y()));
	ui.textEdit_utility->append(QStringLiteral("Distance: %1wpx, %2mm").arg(u.distance).arg(ScaleManager::instance().to_mm(ScaleManager::instance().world_to_fov(u.distance))));
	ui.textEdit_utility->append(QStringLiteral("Angle: %1").arg(u.angle));
}

void VisionApp::startThread(QObject* worker)
{
	QThread* thread = new QThread;
	worker->moveToThread(thread);
	//connect(worker, SIGNAL(error(QString)), this, SLOT(error(QString)));

	connect(thread, SIGNAL(started()), worker, SLOT(process()));
	connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
	connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
	connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
	thread->start();
}

void VisionApp::getEncoder(const QString & data, dat::WorldCoordinate & encoder)
{
	QString x_direction = data.mid(0, 1);
	QString x_value = data.mid(1, 7);
	QString y_direction = data.mid(8, 1);
	QString y_value = data.mid(9, 7);
	QString z_direction = data.mid(16, 1);
	QString z_value = data.mid(17, 7);

	encoder.wx = x_value.toDouble() / 1000;
	encoder.wy = y_value.toDouble() / 1000;
	encoder.wz = z_value.toDouble() / 1000;

	if (x_direction == "N") encoder.wx = -encoder.wx;
	if (y_direction == "N") encoder.wy = -encoder.wy;
	if (z_direction == "N") encoder.wz = -encoder.wz;

	emit signalEncoderChanged(encoder.wx, encoder.wy, encoder.wz);
}

void VisionApp::removeWhitespace(QString & str)
{
	str = str.simplified();
	str.replace(" ", "");
}

void VisionApp::createTrayIcon()
{
	_minimizeAction = new QAction(tr("Mi&nimize"), this);
	connect(_minimizeAction, &QAction::triggered, this, &VisionApp::hideExe);

	_maximizeAction = new QAction(tr("Ma&ximize"), this);
	connect(_maximizeAction, &QAction::triggered, this, &VisionApp::showExe);

	_quitAction = new QAction(tr("&Quit"), this);
	connect(_quitAction, &QAction::triggered, this, &VisionApp::quitExe);

	trayIconMenu = new QMenu(this);
	trayIconMenu->addAction(_minimizeAction);
	trayIconMenu->addAction(_maximizeAction);
	//trayIconMenu->addAction(restoreAction);
	trayIconMenu->addSeparator();
	trayIconMenu->addAction(_quitAction);

	trayIcon = new QSystemTrayIcon(this);
	trayIcon->setContextMenu(trayIconMenu);

	QIcon *icon;
	icon = new QIcon(":/yellowIcon/Icon/yellowIcon/deployment.png");
	trayIcon->setIcon(*icon);
	setWindowIcon(*icon);
	trayIcon->show();
}

bool VisionApp::autoScaling(double step_mm, double& h_scale, double& v_scale)
{
	//TODO: Write auto scale with enqueue
	return true;
}

void VisionApp::setCameraAngle(double angle)
{
	int camIndex = 0;
	SystemData::instance()._camAngles[_camID] = angle;
	ui.lineEdit_camAngle->setText(QString::number(angle));
	createCamAlpha();
}

void VisionApp::cameraAlignment()
{
	bool ok = false;
	auto featureParams = buildAlignFeatureParams(ok);
	if (!ok) return;

	auto step_mm = ui.lineEdit_step_mm->text().toDouble();

	_prevCamAlignedAngle = SystemData::instance()._camAngles[_camID];
	setCameraAngle(0.0);

	emit performCameraAlignment(SystemData::instance().currentCoordinate(), step_mm, featureParams);
}

void VisionApp::performScaling()
{
	auto ret = promptQuestion("Scaling", "This is a destructive action, are you sure you want to proceed? As the difference in scaling will affect existing recipe.");

	if (!ret) return;

	if (passwordPromptCorrect()) {
		bool ok = false;
		auto featureParams = buildAlignFeatureParams(ok);
		if (!ok) return;

		auto step_mm = ui.lineEdit_step_mm->text().toDouble();

		emit performCameraScaling(SystemData::instance().currentCoordinate(), step_mm, featureParams);
	}
}

double VisionApp::mm_to_pulse(double mm)
{
	//1 pulse = 1um
	return mm * 1000;
}

double VisionApp::um_to_mm(double um)
{
	return um / 1000;
}

QString VisionApp::getJogCommand(double x, double y, double z)
{
	auto sx = util::num_to_str(x, 3, 0);
	auto sy = util::num_to_str(y, 3, 0);
	auto sz = util::num_to_str(z, 3, 0);
	auto s = sx + "_" + sy + "_" + sz;
	auto QS = s.c_str();
	return QS;
}

QString VisionApp::getJogCommand(dat::WorldCoordinate world)
{
	return getJogCommand(world.wx, world.wy, world.wz);
}

QString VisionApp::getFidCompensatedJogCommand(double x, double y, double z)
{
	double sx = x, sy = y;

	//if (_useFiducial) {
	//	auto shifted = _fiducial.getShiftedPoint(em::V2d(x, y));
	//	sx = shifted.x();
	//	sy = shifted.y();
	//}

	auto s = getJogCommand(sx, sy, z) + "_fid";
	return s;
}

QString VisionApp::getFidCompensatedJogCommand(dat::WorldCoordinate world)
{
	return getFidCompensatedJogCommand(world.wx, world.wy, world.wz);
}

QString VisionApp::getLaserFidCompensatedJogCommand(dat::WorldCoordinate world)
{
	double sx = world.wx, sy = world.wy;

	/*if (_useFiducial) {
		auto shifted = _fiducial.getShiftedPoint(em::V2d(world.wx, world.wy));
		sx = shifted.x();
		sy = shifted.y();
	}*/

	sx += _laserConfig.offset.wx;
	sy += _laserConfig.offset.wy;
	world.wz += _laserConfig.offset.wz;

	//auto s = getJogCommand(sx, sy, world.wz) + "_fid";
	auto s = getJogCommand(sx, sy, world.wz);
	return s;
}

bool VisionApp::jogToView(const QView & v)
{
	double new_z = v.world.wz;

	if (_compensateMap.contains(v.id)) {
		new_z += _compensateMap[v.id];
	}

	jogToFidCompensatedXYZ(v.world.wx, v.world.wy, new_z);

	return true;
}

bool VisionApp::jogToFidCompensatedXYZ(double x, double y, double z, QString type)
{
	if (_useFiducial) {
		auto shifted = _fiducial.getShiftedPoint(em::V2d(x, y));
		emit jogTo(shifted.x(), shifted.y(), z, type);
	}

	emit jogTo(x, y, z, type);
	return true;
}

bool VisionApp::jogToLaserFidCompensatedXYZ(double x, double y, double z, QString type)
{
	x += _laserConfig.offset.wx;
	y += _laserConfig.offset.wy;
	z += _laserConfig.offset.wz;

	emit jogTo(x, y, z, type);
	return true;
}

bool VisionApp::jogToCamView(double x, double y, double z, QString type)
{
	auto expectedHeight = 32;
	auto range = 5;

	dat::WorldCoordinate point;
	point.wx = x - _laserConfig.offset.wx;
	point.wy = y - _laserConfig.offset.wy;
	point.wz = z - _laserConfig.offset.wz;

	/*if (point.wz > expectedHeight + range || point.wz < expectedHeight - range) {
		ct::logger::info("Already in camera view");
		return false;
	}*/

	emit jogTo(point.wx, point.wy, point.wz, "2D");
	return true;
}

bool VisionApp::jogToLaserView(double x, double y, double z, QString type)
{
	auto expectedHeight = 68;
	auto range = 5;

	dat::WorldCoordinate point;
	point.wx = x + _laserConfig.offset.wx;
	point.wy = y + _laserConfig.offset.wy;
	point.wz = z + _laserConfig.offset.wz;

	/*if (point.wz > expectedHeight + range || point.wz < expectedHeight - range) {
		ct::logger::info("Already in laser view");
		return false;
	}*/

	emit jogTo(point.wx, point.wy, point.wz, "2D");
	return true;
}

bool VisionApp::getCurrentPoint(double& x, double& y, double& z)
{
	const auto& cc = SystemData::instance().currentCoordinate();
	x = cc.wx;
	y = cc.wy;
	z = cc.wz;

	return true;
}

void VisionApp::testFunction()
{
	/*QString imagePath = "C:/Advanced/Data/recipe/2SZ1_1703/Images/VidiImages/Bad/1683019371774177.jpg";
	runImageVidi(imagePath);*/
}

void VisionApp::saveOffsettedVIDIImage(const QString & visionObjectID, const QString & lightingID, const QString & imagePath, const QPointF & offset)
{
	auto vo = _visionObject.find(visionObjectID);
	if (vo == _visionObject.end()) return;

	auto viewID = vo.value().viewID; //need to change to ID

	if (!_views.contains(viewID)) {
		ct::logger::error("[JobThread] Failed to save offsetted VIDI image. Invalid view ID: %s", viewID.toStdString().c_str());
		return;
	}

	auto iView = _views.find(viewID);
	if (iView == _views.end())	return;

	auto view = iView.value();
	auto path = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), view, _mainOptics[_camID], _recipeOptics);

	ct::logger::info("offsetX: %d, offsetY: %d", offset.toPoint().x(), offset.toPoint().y());
	ct::logger::info("saveOffsettedVIDIImagePath:%s", path.getOpticPath(lightingID.toStdString(), ui.lineEdit_currentImageIndex->text()).c_str());

	QPixmap img;
	bool loadSuccess = img.load(path.getOpticPath(lightingID.toStdString(), ui.lineEdit_currentImageIndex->text()).c_str());

	if (loadSuccess)
	{
		QPointF fovView = { 0,0 };
		if (g_viewMode == int(ViewMode::PLANE)) fovView = ScaleManager::instance().to_fov_px(view);
		int ix = vo.value().rect.x() - fovView.x() + (int)offset.x();
		int iy = vo.value().rect.y() - fovView.y() + (int)offset.y();
		int iw = vo.value().rect.width();
		int ih = vo.value().rect.height();

		QPixmap padded(iw, ih);
		padded.fill(Qt::black); // Fill the entire image with black

		int src_x = std::max(0, ix);
		int src_y = std::max(0, iy);
		int src_w = std::min(iw, img.width() - src_x);
		int src_h = std::min(ih, img.height() - src_y);

		if (src_w > 0 && src_h > 0) {
			QPixmap cropped = img.copy(src_x, src_y, src_w, src_h);
			QPainter painter(&padded);
			int dst_x = (ix < 0) ? -ix : 0;
			int dst_y = (iy < 0) ? -iy : 0;
			painter.drawPixmap(dst_x, dst_y, cropped);
		}

		padded.save(imagePath);
	}


}

void VisionApp::slotDatabaseStatus(bool status)
{
	qDebug() << "Insert DataBase status: " << status;
	if (status == false)
	{
		if (_inspStatus.productionMode)sendToClient("01VISIONEMAPFAIL\r");
	}
	else
	{
		clearCacheFolder();
	}	
}

void VisionApp::insertProductionToDataBase(QVector<ct::DefectResult>& defectResults)
{
	
	ProductionInfo p;
	p.timestamp = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
	p.recipeID = Common::Directory::CurrentRecipe;;
	p.recipeName = Common::Directory::CurrentRecipe;

	p.recipeID.replace("/", "-");
	p.recipeName.replace("/", "-");

	p.productionID = Common::Directory::productionName;
	p.productionFileName = Common::Directory::productionName;
	p.cycleTime = g_time.duration();
	p.totalDefect = defectResults.size();
	p.isMounterPass = true;
	
	p.scalingUmPixel = ScaleManager::instance().um_per_px();

	p.inspectionStartDate = _inspStatus.inspectionStartTime;
	p.inspectionEndDate = _inspStatus.inspectionEndTime;

	p.isError = false;
	p.report = false;
	p.stripeID = SystemData::instance()._currentBarcode.c_str();
	p.assemblyNumber = "-";

	qDebug() << "CompleteInspection Cycle Time:" << p.cycleTime;
	// island info
	p.totalRow = _islandInfo.totalRow;
	p.totalCol = _islandInfo.totalCol;
	p.totalIsland = _islandInfo.totalIsland;
	p.rowStartingIndex = _islandInfo.rowStartingIndex;
	p.colStartingIndex = _islandInfo.colStartingIndex;

	p.lotNumber = _lotInfo.lotNumber;
	p._operator = _lotInfo.operatorID;
	p.productNumber = _lotInfo.productNumber;
	
	QString facing;
	
	bool isTop = true;
	checkRecipeFacing(Common::Directory::CurrentRecipe, isTop);
	facing = isTop? "Top":"Bottom";

	p.inspectionType = facing; // top or bottom

	p.isSampling = _enableVisionObjectSampling;
	p.incomingEmapPath = _emapInfo.incomingEmapPath;

	int defectUnits = 0;
	int skipUnits = 0;
	int incomingUnits = 0;
	int dieShearUnits = 0;
	for (auto& vo : _visionObject)
	{
		
		if (vo.skip == true||vo.forcedSkip == true) skipUnits++;
		else if (vo.ignore == true) incomingUnits++;
		else if (vo.isDieShear == true) dieShearUnits++;
		else if (vo.isPass == false) defectUnits++;
		
	}

	qDebug() << "skipUnits:" << skipUnits << "incomingUnits:" << incomingUnits << "dieShearUnit:" << dieShearUnits;
	p.totalUnit = _visionObject.size() - skipUnits - incomingUnits - dieShearUnits;
	p.defectUnits = defectUnits;

	double defectPercentage = 0.0;
	if (p.totalUnit > 0)
	{
		defectPercentage = (static_cast<double>(p.defectUnits) / static_cast<double>(p.totalUnit)) * 100.0;
	}
	double yieldPercentage = 100.00 - defectPercentage;
	p.yieldPerc = yieldPercentage;
	p.passYieldPerc = 100;
	if (yieldPercentage >= _passYieldPerc)
	{
		p.isPass = true;
	}
	else
	{
		p.isPass = false;
	}

	if (p.lotNumber.isEmpty())
		p.lotNumber = "-";
	/*if (key == "Recipe Name")
		p.recipeName = value;*/
	if (p._operator.isEmpty())
		p._operator = "-";
	if (p.productNumber.isEmpty())
		p.productNumber = "-";
	if (p.deviceGroup.isEmpty())
		p.deviceGroup = "-";
	if (p.inspectionType.isEmpty())
		p.inspectionType = "-";

	// inspectionStatus
	for (auto fidStatus : _inspStatus.fiducialHash)
	{
		p.fiducialStatus.append(fidStatus.fiducialStatus);
	}

	// emap template
	p.emapTemplate = _emapInfo.templateName;
	p.productionMode = _inspStatus.productionMode;


	PackageInfo packageInfo;
	packageInfo.timeStamp = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
	packageInfo.packageUuid = _currentProductionID;
	bool readyToPackage = true;




	DataBaseThread::ProductionExportMode exportMode = DataBaseThread::ProductionExportMode::Normal;
	if (ui.checkBox_usedAsRecipe1->isChecked())
	{
		exportMode = DataBaseThread::ProductionExportMode::Recipe1;
	}
	else if (ui.checkBox_usedAsRecipe2->isChecked())
	{
		exportMode = DataBaseThread::ProductionExportMode::Recipe2;
	}
	_databaseThread.setProductionExportMode(exportMode);

	_databaseThread.setSetting(
		_sqliteDatabase, 
		defectResults, 
		p,
		_templateLibraryTab->getAllTemplateID(),
		readyToPackage,
		packageInfo
	);
	_databaseThread.start();

	// Wait for the thread to finish before proceeding
	qDebug() << "Waiting result compilation...";
	//_databaseThread.wait();
	qDebug() << "Result compilation completed!";


}

void VisionApp::outputOpticInfo()
{
	QJsonArray vArray;
	for (auto optic : _recipeOptics)
	{
		QJsonObject vObject;
		vObject.insert("opticID", optic.id);
		vObject.insert("opticName", optic.name);
		vArray.append(vObject);

	}
	QJsonObject vInfo;
	vInfo.insert("Optic_Info", vArray);
	QJsonDocument jsonDoc(vInfo);
	QString jsonPath = Common::Directory::getProductionResultPath() + "/OpticInfo.json";
	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());

		file.close();

	}
	else
	{
		qDebug() << "Write OpticInfo failed";
	}
}

void VisionApp::outputUnitIndexInfo(QVector<ct::DefectResult>& defectResults, QVector<BarcodeDecoderInfo>& barcodeInfoVector)
{
	qDebug() << "outputUnitIndexInfo";

	int totalIndex = 0;
	for(auto v: _views)
	{
		if (v.id == "Mark")
		{
			totalIndex = _unitConfigTab->getTotalIndex(v.id);
			break;
		}	
	}
	QJsonArray uArray;
	for (int i=0; i< totalIndex; i++ )
	{
		QString barcode;
		QString index = QString::number(i + 1);
		bool isPass = true;

		for (auto bInfo : barcodeInfoVector)
		{
			if (bInfo.indexId == index)
			{
				barcode = bInfo.decodedString;
				break;
			}		
		}
		for (auto bInfo : defectResults)
		{
			if (bInfo.view_id == "Pocket") continue;
			if (QString::fromStdString(bInfo.index) == index)
			{
				isPass = false;
				break;
			}
		}
		if (barcode.isEmpty()) barcode = "unknown_barcode";

		QJsonObject unitObject;
		unitObject.insert("indexId", index);
		unitObject.insert("barcode", barcode);
		unitObject.insert("isPass", isPass);
		unitObject.insert("verificationStatus", 0);
		
		uArray.append(unitObject);
	}

	QJsonObject vInfo;
	vInfo.insert("Unit_Info", uArray);
	QJsonDocument jsonDoc(vInfo);
	QString jsonPath = Common::Directory::getProductionResultPath() + "/UnitInfo.json";
	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());
		file.close();
	}
	else
	{
		qDebug() << "Write UnitInfo failed";
	}
}

void VisionApp::outputBoardInfo()
{
	QHash<int, BoardInfo> boardInfoHash;

	for (auto& vo : _visionObject)
	{	
		if (!boardInfoHash.contains(vo.island_id))
		{
			BoardInfo bInfo;
			bInfo.serialNumber = vo.island_id;
			bInfo.boardBarcode = vo.localBarcode;
			boardInfoHash[vo.island_id] = bInfo;
		}
	}

	QJsonArray vArray;
	for (auto bInfo : boardInfoHash)
	{
		QJsonObject vObject;
		vObject.insert("BoardSerial", bInfo.serialNumber);
		vObject.insert("BoardBarcode", bInfo.boardBarcode);
		vArray.append(vObject);
	}
	QJsonObject vInfo;
	vInfo.insert("Boards", vArray);
	QJsonDocument jsonDoc(vInfo);
	QString jsonPath = Common::Directory::getProductionResultPath() + "/BoardInfoList.json";
	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());
		file.close();
	}
	else
	{
		qDebug() << "Write Board Info failed";
	}
}

void VisionApp::outputRecipeSettings()
{
	QString recipeMode = "Single";

	QJsonObject recipeInfo;
	recipeInfo.insert("RecipeMode", recipeMode);
	QJsonDocument jsonDoc(recipeInfo);
	QString jsonPath = Common::Directory::getProductionResultPath() + "/RecipeInfos.json";
	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());
		file.close();
	}
	else
	{
		qDebug() << "Write Recipe Info failed";
	}
}

void VisionApp::outputViewInfo()
{
	double h_scale = ScaleManager::instance().horizontal_um_per_px();
	double v_scale = ScaleManager::instance().vertical_um_per_px();

	//Get FOV
	double h_cam_mm = util::px_to_mm(CAMManager::instance().getWidth(_camID), h_scale);
	double v_cam_mm = util::px_to_mm(CAMManager::instance().getHeight(_camID), v_scale);

	//get corner as point
	auto topleft_x_mm = _plane.corner_points[(int)Corner::FRONTLEFT].wx - h_cam_mm / 2;
	auto topleft_y_mm = _plane.corner_points[(int)Corner::FRONTLEFT].wy - v_cam_mm / 2;

	QPointF wpx = ScaleManager::instance().to_world_px(QPointF(topleft_x_mm, topleft_y_mm));
	QJsonArray vArray;
	for (auto v : _views)
	{
		if (v.id.isEmpty()) {
			ct::logger::error("Empty view found in output view info");
			continue;
		}

		QJsonObject vObject;
		vObject.insert("ViewID", v.id);
		vObject.insert("ViewName", v.name); 
		vObject.insert("ViewX", v.pDragBox->getGeometry().x());
		vObject.insert("ViewY", v.pDragBox->getGeometry().y());
		vObject.insert("ViewW", v.pDragBox->getGeometry().width());
		vObject.insert("ViewH", v.pDragBox->getGeometry().height()); 
		vArray.append(vObject);
		
	}
	
	QJsonObject vInfo;
	vInfo.insert("Views", vArray);
	vInfo.insert("WorldScale", ScaleManager::instance().world_scale());
	vInfo.insert("PlaneX", wpx.x());
	vInfo.insert("PlaneY", wpx.y());
	QJsonDocument jsonDoc(vInfo);
	QString jsonPath = Common::Directory::getProductionResultPath() + "/ViewInfoList.json";
	
	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());

		file.close();

	}
	else
	{
		qDebug() << "Write Views Info failed";
	}
}

void VisionApp::outputLineScanInfo()
{
	QJsonArray lArray;
	for (auto l : _lineScans)
	{
		if (l.id.isEmpty()) {
			ct::logger::error("Empty view found in output view info");
			continue;
		}

			QPointF fovLine;

		if (l.type == ct::s_stitch_linescan) {
			for (auto& childL : _lineScans) {
				if (childL.map_to_slinescan == l.id) {
					if (childL.id.contains("-0")) {

						fovLine = ScaleManager::instance().to_fov_px(childL);
					}
				}
			}
		}
		else {
			fovLine = ScaleManager::instance().to_fov_px(l);
		}

		QJsonObject lObject;
		lObject.insert("LineScanID", l.id);
		lObject.insert("LineScanName", l.name);
		lObject.insert("LineScanX", fovLine.x());
		lObject.insert("LineScanY", fovLine.y());
		lArray.append(lObject);

	}

	QJsonObject vInfo;
	vInfo.insert("LineScans", lArray);
	QJsonDocument jsonDoc(vInfo);
	QString jsonPath = Common::Directory::getProductionResultPath() + "/LineScansInfoList.json";

	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());

		file.close();

	}
	else
	{
		qDebug() << "Write Views Info failed";
	}
}

void VisionApp::outputVoInfo()
{
	QJsonArray voArray;
	for (auto& vo : _visionObject)
	{
		QJsonObject voObject;
		voObject.insert("Name", vo.objectName);
		voObject.insert("Id", vo.objectID);
		voObject.insert("ViewID", vo.viewID);
		voObject.insert("LineScanId", vo.lineScanID);
		voObject.insert("x", vo.rect.x());
		voObject.insert("y", vo.rect.y());
		voObject.insert("w", vo.rect.width());
		voObject.insert("h", vo.rect.height());
		voObject.insert("RowName", vo.row);
		voObject.insert("RowId", vo.row_id);
		voObject.insert("ColName", vo.col);
		voObject.insert("ColId", vo.col_id);
		voObject.insert("IslandId", vo.island_id);
		voObject.insert("IslandName", vo.island);
		voObject.insert("Ignore", vo.ignore);
		voObject.insert("IsPass", vo.isPass);
		voObject.insert("ForcedSkip", vo.forcedSkip);
		voObject.insert("Skip", vo.skip);
		voObject.insert("CircuitId", vo.circuitID);
		voObject.insert("LocOffsetX", vo.locOffsetX);
		voObject.insert("LocOffsetY", vo.locOffsetY);
		voObject.insert("LocalBarcode", vo.localBarcode);
		voObject.insert("VoBarcode", vo.voBarcode);
		voObject.insert("VoBarcodeName", vo.voBarcodeName);
		voObject.insert("TemplateName", vo.templateName);
		voObject.insert("TemplateID", vo.templateID);
		voObject.insert("IsDieShear", vo.isDieShear);
		
		QJsonArray cadMounterArr;
		for (int i = 0; i < vo.cadMounterId.size(); i++)
		{
			QString cadName = vo.cadMounterId.keys()[i];
			QString mounterId = vo.cadMounterId[cadName];

			QJsonObject cObject;
			cObject.insert("CadName", cadName);
			cObject.insert("MounterId", mounterId);
			cadMounterArr.append(cObject);
		}
		voObject.insert("CadMounterMapping", cadMounterArr);
		voArray.append(voObject);
	}

	QJsonObject voInfo;
	voInfo.insert("VoInfo", voArray);
	voInfo.insert("MainOpticId", _mainOptics[_camID].id);
	QJsonDocument jsonDoc(voInfo);
	QString jsonPath = Common::Directory::getProductionResultPath() + "/VoInfoList.json";
	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());

		file.close();

	}
	else
	{
		qDebug() << "write Vo Info failed";
	}
}

bool VisionApp::readOutputVoInfo(QString filePath)
{

	QString fileName = Common::Directory::getProductionResultPath() + "VoInfoList.json";
	if (!filePath.isEmpty()) fileName = filePath;
	QString val;
	QFile file;

	qDebug() << "outputVoFileName:" << fileName;
	file.setFileName(fileName);

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "readOutputVoInfo failed";
		return false;
	}

	val = file.readAll();
	file.close();

	QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
	QJsonObject root = doc.object();

	QJsonArray VoInfoList = jsonHelper::getArray(root, QStringLiteral("VoInfo"));

	for (auto& vo : _visionObject)
	{
		vo.isPass = true;
	}

	for (auto& vo : _visionObject)
	{
		if (!vo.isPass)
		{
			qDebug() << "vostatus is not reset";
		}
	}
	qDebug() << "voInfoList:" << VoInfoList.size();
	for (int i = 0; i < VoInfoList.size(); i++)
	{
		auto jsonObj = VoInfoList[i].toObject();
		bool isPass = jsonHelper::getBool(jsonObj, QStringLiteral("IsPass"));
		QString id = jsonHelper::getString(jsonObj, QStringLiteral("Id"));
		_visionObject[id].isPass = isPass;
	}

	return true;

}

void VisionApp::outputCustomResultJson(QVector<ct::DefectResult>& defectResults)
{
	qDebug() << "outputCustomResultJson";
	struct VoInfo
	{
		QString island;
		QString row;
		QString col;
		QString id;
		QString defectCode;
	};

	QHash<QString, VoInfo> voResult; // key:voName, value: isPass
	// insert all vo from voHash
	for (int i = 0; i < _visionObject.size(); i++)
	{
		QString voName = _visionObject[_visionObject.keys()[i]].objectName;
		if (voName.contains("I") && voName.contains("R") && voName.contains("C"))
		{

			VoInfo v;
			v.island = voName.mid(voName.indexOf("I") + 1, voName.indexOf("R") - 1 - voName.indexOf("I"));
			v.row = voName.mid(voName.indexOf("R") + 1, voName.indexOf("C") - 1 - voName.indexOf("R"));
			v.col = voName.mid(voName.indexOf("C") + 1, voName.length() - voName.indexOf("C"));
			v.id = _visionObject[_visionObject.keys()[i]].objectID;
			v.defectCode = "P";


			voResult.insert(voName, v);
		}
		else
		{
			qDebug() << "Invalid Vision Object Name: " << voName;
			break;
		}

	}


	//QString defectCodeFilePath = "C:/Advanced/Data/VisionDefectName.txt";
	QString defectCodeFilePath = jsonHelper::getString(_systemObj, QStringLiteral("Defect_Drive")) + "VisionDefectName.txt";
	QHash<QString, QString> defectCodeHash;
	defectCodeHash = getDefectCode(defectCodeFilePath);
	qDebug() << "defectCodeHash:" << defectCodeHash;
	// detect defect visionObj
	for (int i = 0; i < defectResults.size(); i++)
	{
		QString voName = QString::fromStdString(defectResults[i].algoDefResult.vo_name);
		if (voResult.contains(voName))
		{
			QString tagName = !defectResults[i].tagNames.isEmpty() ? defectResults[i].tagNames[0] : "Unknown";
			qDebug() << "tagname:" << tagName;
			if (defectCodeHash.contains(tagName))
			{
				voResult[voName].defectCode = defectCodeHash[tagName];
			}
			else
			{
				voResult[voName].defectCode = "B";
			}

		}
	}

	// output json file
	QJsonArray voArray;
	for (int i = 0; i < voResult.size(); i++)
	{
		VoInfo v = voResult[voResult.keys()[i]];
		QJsonObject voObject;
		voObject.insert("island", v.island);
		voObject.insert("row", v.row);
		voObject.insert("col", v.col);
		voObject.insert("id", v.id);
		voObject.insert("defectCode", v.defectCode);

		voArray.append(voObject);
	}



	QJsonObject defVo;
	defVo.insert("defects", voArray);
	QJsonDocument jsonDoc(defVo);
	QString jsonPath = Common::Directory::getProductionResultPath() + "/defectList.json";
	QFile file(jsonPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());

		file.close();

	}
	else
	{
		qDebug() << "write json failed";
	}
	QString copyPath = jsonHelper::getString(_systemObj, QStringLiteral("Defect_Drive")) + "defectList.json";

	QFile copyFile(copyPath);
	if (copyFile.open(QIODevice::WriteOnly))
	{
		copyFile.write(jsonDoc.toJson());

		copyFile.close();

		qDebug() << "copy json suc";

	}
	else
	{
		qDebug() << "copy json failed";
	}



}

QHash<QString, QString> VisionApp::getDefectCode(QString&  filePath)
{
	QHash<QString, QString> infoHash;

	// Open the file
	QFile file(filePath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Failed to open the file:" << file.errorString();
		return infoHash;
	}

	// Read the file content
	QTextStream in(&file);
	while (!in.atEnd())
	{
		QString line = in.readLine().trimmed();
		if (line.isEmpty())
			continue;

		// Extract the key and value from the line
		QStringList parts = line.split(':', QString::SkipEmptyParts);
		if (parts.size() == 2)
		{
			QString key = parts[1].trimmed();
			QString value = parts[0].trimmed();
			infoHash.insert(key, value);
		}
	}

	// Close the file
	file.close();

	return infoHash;
}

void VisionApp::updateVoStatus(QVector<ct::DefectResult>& defectResults, QVector<BarcodeDecoderInfo>& barcodeInfoVector)
{
	// reset
	for (auto& vo : _visionObject)
	{
	}

	// collect barcode hash 
	QHash<int, QString> barcodeHash; // key: island ID
	for (auto b : barcodeInfoVector)
	{
		int islandID = _visionObject[b.voId].island_id;
		barcodeHash[islandID] = b.decodedString;
		
		if(_visionObject.contains(b.voId))
		{
			_visionObject[b.voId].voBarcode = b.decodedString;
			_visionObject[b.voId].voBarcodeName = b.barcodeName;;
		}
	}


	for (auto& vo : _visionObject)
	{
		vo.isPass = true;
		vo.isDieShear = false;

		// here add in the local barcode
		if (barcodeHash.contains(vo.island_id)) vo.localBarcode = barcodeHash[vo.island_id];
		else vo.localBarcode = "No_Barcode";
	}

	for (int i = 0; i < defectResults.size(); i++)
	{
		QString voID = QString::fromStdString(defectResults[i].algoDefResult.vo_id);

		if (defectResults[i].algoDefResult.dieShear == true)
		{	
			_visionObject[voID].isDieShear = true;

		}
	

		if (_visionObject.contains(voID))
		{
			_visionObject[voID].isPass = false;
			if (_visionObject[voID].skip == true || _visionObject[voID].forcedSkip == true || _visionObject[voID].ignore == true || _visionObject[voID].isDieShear == true)
			{
				_visionObject[voID].isPass = true;
			}


			_visionObject[voID].locOffsetX = defectResults[i].algoDefResult.loc_offset_x;
			_visionObject[voID].locOffsetY = defectResults[i].algoDefResult.loc_offset_y;
		}
	}
}

void VisionApp::updateRowColIslandID(QVector<ct::DefectResult>& defectResults)
{
	for (int i = 0; i < defectResults.size(); i++)
	{
		QString voID = QString::fromStdString(defectResults[i].algoDefResult.vo_id);
		int rowID = _visionObject[voID].row_id;
		int colID = _visionObject[voID].col_id;
		int islandID = _visionObject[voID].island_id;

		defectResults[i].algoDefResult.vo_row_id = rowID;
		defectResults[i].algoDefResult.vo_col_id = colID;
		defectResults[i].algoDefResult.vo_island_id = islandID;
	}
}



void VisionApp::inspectionDone(QVector<ct::DefectResult>& defectResults, QVector<BarcodeDecoderInfo>& barcodeInfoVector)
{
	ct::logger::info("InspectionDone");


	QDateTime endMS = QDateTime::currentDateTime();
	
	_inspStatus.inspectionEndTime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
	_productionPage->stopElapseTime();

	
	QDateTime startMS = SystemData::instance().StartInspectionTimer;
	int cycleTime_ms = startMS.msecsTo(endMS);
	double cycleTime_sec = cycleTime_ms / 1000.0;

	ui.lineEdit_inspectionTimeMain->setText(QString::number(cycleTime_sec, 'f', 2));


	TimeLogger time;
	updateVoStatus(defectResults, barcodeInfoVector);
	updateRowColIslandID(defectResults);
	time.log_duration("[InspectionDone] Update VO Status");
	

	updateEmap();
	time.log_duration("[InspectionDone] Update Emap");

	
	outputVoInfo(); 
	outputViewInfo(); 
	outputLineScanInfo();
	outputOpticInfo(); 
	outputBoardInfo();
	outputRecipeSettings();
	//outputUnitIndexInfo(defectResults, barcodeInfoVector); //this for rtr/e250 single view 
	
	time.log_duration("[InspectionDone] Output results");

	
	//copy cadLibInfo to production
	QString srcCadLibInfo = Common::Directory::getRecipeCurrentPath() + "/ComponentCadTypeLibrary.json";
	QString destCadLibInfo = Common::Directory::getProductionResultPath() + "/ComponentCadTypeLibrary.json";
	QFile::copy(srcCadLibInfo, destCadLibInfo);
	time.log_duration("[InspectionDone] Copy CAD info to production folder");


	//copy unitConfig to production
	QString srcUnitConfig = Common::Directory::getRecipeCurrentPath() + "/unitConfig.json";
	QString destUnitConfig = Common::Directory::getProductionResultPath() + "/unitConfig.json";
	QFile::copy(srcUnitConfig, destUnitConfig);
	time.log_duration("[InspectionDone] Copy unit config to production folder");


	if (!_inspStatus.productionMode)
	{
		drawDefectResults(defectResults);
		time.log_duration("[InspectionDone] Draw defect results");

		_datasetPage->addDefectResults(defectResults);
		time.log_duration("[InspectionDone] Add defect results to dataset page");

		refreshDatasetView();
		time.log_duration("[InspectionDone] Refresh dataset view");

		showDefectRect(true);
		time.log_duration("[InspectionDone] Show defect rect");
	}

	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject, defectResults);
	_inspectionThreadBusy = false;
	time.log_duration("[InspectionDone] Update tree view");
	

	// copy blob details json to production
	QString srcCompJsonPath = Common::Directory::CachePath + "/CompPos";
	QString srcAIJsonPath = Common::Directory::CachePath + "/aiBlobDetail";
	QString destAIJsonPath = Common::Directory::getProductionResultPath() + "/aiBlobDetail";
	if (copyFolderRecursively(srcAIJsonPath, destAIJsonPath)) {
		ct::logger::debug("Ai Blob Detail copied success");
	}
	else {
		ct::logger::debug("Ai Blob Detail copied failed");
	}
	time.log_duration("[InspectionDone] Copy blob details");



	QDir dir(srcCompJsonPath);
	if (dir.exists()) {
		QStringList files = dir.entryList(QDir::Files);
		for (const QString& file : files) {
			dir.remove(file);
		}
	}


	ct::logger::info("[InspectionDone] Wait for image saving...");
	while (IST::instance().size()) { os_tool::goSleep(500); }
	time.log_duration("[InspectionDone] Wait for image saving");
	
	insertProductionToDataBase(defectResults); // mini defect packager is inside here
	time.log_duration("[InspectionDone] Insert production info to database");
	
	
	if (_inspMode) {
		//if (_enableBarcode)
		//{
		//	QString barcodeID = QStringLiteral("01ID") + SystemData::instance()._currentBarcode.c_str() + QStringLiteral("\r");
		//	//sendToClient(barcodeID);
		//}
		//sendToClient("01END\r");
		_inspMode = false;
	}

	
	// clear cache of locator roi 
	clearDirectory(Common::Directory::CachePath + "/LocatorPos");

	_stopRun = false;
	clearBufferQueue();
	stopRun(false);
	time.log_duration("[InspectionDone] Stop run");


	//ui.label_status->hide();
	time.log_duration("[InspectionDone] Clear production status");


	//Set machine mode
	//here get lot info
	_productionPage->inspectionDone();
	progressBarRelease(_stopRun);
	reloadStyleSheet();
	time.log_duration("[InspectionDone] UI Update");


	if (_inbbaInspection)
	{
		runBareBoardAnalysis();
		openRecipe("", true);
		_inbbaInspection = false;
	}
	time.log_duration("[InspectionDone] Bare board analysis");


	/*Common::Directory::CurrentImageSetPath = SystemData::instance()._workingPath;
	ct::logger::info("Production path: %s", SystemData::instance()._workingPath.toStdString().c_str());

	if (SystemData::instance()._loadProductionAfterRun && !SystemData::instance()._offlineRun) {
		loadImageSet(Common::Directory::CurrentImageSetPath);
		time.log_duration("[InspectionDone] Load image set");
	}*/

	

	// Auto-calibration report: the run started by the auto-cal button has finished.
	// Fires for both online and offline completion paths. Generate it once here.
	if (_autoCalPending) {
		_autoCalPending = false;
		generateAutoCalReport();
	}

	if (!SystemData::instance()._offlineRun) {
		if (_testRunLoopingNoUnload && ui.checkBox_runLooping->isChecked()) {
			ct::logger::info("[InspectionDone] Test-run loop enabled; skip unload and load next loop position.");
			runLooping();
		}
		else {
			unloadBoard();

			auto& sys = SystemData::instance();
			if (sys._InspectionCompleted) sys._BoardEntryQty = sys._BoardEntryQty + 2;
		}
	}
	else runLooping();
}


void VisionApp::updateInspectionProgressBar()
{
	incrementProgressBar();
}

void VisionApp::runLooping()
{
	ct::logger::info("runLooping");
	auto online = ui.toolButton_toggleOnlineRun->isChecked();

	if (!_inspQueue.empty()) {
		runQueuedInsp();
	}
	else if (ui.checkBox_runLooping->isChecked())
	{
		recordMemory(QString("Start #%1").arg(_loop));

		if (_loop > 0)
		{
			if (online) {
				startProduction();
			}
			else {
				run();
			}

			_loop--;
		}
		else {
			_testRunLoopingNoUnload = false;
		}
	}
	else {
		_testRunLoopingNoUnload = false;
	}

}

void VisionApp::locatorInfo(QPointF locatorOffsets, double locatorAngle, QString viewID, QString indexID, bool locatorFail, bool locatorAngleFail)
{
	ct::logger::info("[VisionApp] LocatorInfoEmitted: x:%.5f, y: %.5f, angle: %.5f,%s, %s", locatorOffsets.x(), locatorOffsets.y(), locatorAngle, viewID.toStdString(), indexID.toStdString());
}

void VisionApp::displayLiveImage(QVector<FrameInfo> frameInfos, QHash<QString, ct::UnitResultInfo> unitResultInfo)
{
	TimeLogger timer;

	for (int i = 0; i < frameInfos.size(); i++)
	{
		auto frameInfo = frameInfos[i];
		auto imgType = frameInfo.type;
		QImage img;
		if (imgType == ct::s_mono || imgType == ct::s_color) util::Mil_to_qImg(frameInfo.pImage->id(), img);
		else continue;
		
		timer.log_duration("Display live image: Convert to qimg");

		_productionPage->updateCamGraphicViews(frameInfo.viewID, frameInfo.opticID, QString::number(frameInfo.index), &img, unitResultInfo);
		timer.log_duration("Display live image: Update camera graphic view");
	}
}

void VisionApp::drawDefectResults(QVector<ct::DefectResult>& defectResults)
{
	for (int i = 0; i < defectResults.size(); i++)
	{
		auto x = ScaleManager::instance().fov_to_world(defectResults[i].algoDefResult.def_x) + defectResults[i].algoDefResult.vo_x;
		auto y = ScaleManager::instance().fov_to_world(defectResults[i].algoDefResult.def_y) + defectResults[i].algoDefResult.vo_y;
		auto w = ScaleManager::instance().fov_to_world(defectResults[i].algoDefResult.def_w);
		auto h = ScaleManager::instance().fov_to_world(defectResults[i].algoDefResult.def_h);

		auto def_id = defectResults[i].algoDefResult.def_id;
		auto def_name = defectResults[i].algoDefResult.def_name;
		auto index_id = defectResults[i].index;
		auto view_id = defectResults[i].view_id;
		auto optic_id = defectResults[i].algoDefResult.optic_id;

		QString s = def_name.c_str();
		QColor color = Qt::red;
		if (s.contains("Height")) color = QColor(255, 192, 203);

		if(defectResults[i].algoDefResult.skip) drawDefectRect(QRect(x, y, w, h), def_id.c_str(), def_name.c_str(), view_id.c_str(), index_id.c_str(),optic_id.c_str(), Qt::black);
		else if(defectResults[i].algoDefResult.ignore) drawDefectRect(QRect(x, y, w, h), def_id.c_str(), def_name.c_str(), view_id.c_str(), index_id.c_str(), optic_id.c_str(), Qt::yellow);
		else drawDefectRect(QRect(x, y, w, h), def_id.c_str(), def_name.c_str(), view_id.c_str(), index_id.c_str(), optic_id.c_str(), color);
	}
}

void VisionApp::clearCropGuidingRoi()
{
	for (int i = 0; i < _cropGuidingRoi.count(); i++)
	{
		if (_cropGuidingRoi[i] != nullptr) {
			_pGraphicsSceneMain->removeItem(_cropGuidingRoi.at(i));
			delete _cropGuidingRoi.at(i);
			_cropGuidingRoi[i] = nullptr;
		}
	}
	_cropGuidingRoi.clear();
}

void VisionApp::clearAllDefectRectShape()
{
	for (int i = 0; i < _defectRectShape.count(); i++)
	{
		if (_defectRectShape[i] != nullptr) {
			_pGraphicsSceneMain->removeItem(_defectRectShape.at(i));
			delete _defectRectShape.at(i);
			_defectRectShape[i] = nullptr;
		}
	}

	_defectRectShape.clear();
}

void VisionApp::storeVisionObjectInfo(bool forceCheck)
{
	_visionObjectInfo.clear();
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->isSelected() || forceCheck)
		{
			DragBoxInfo bbox;
			bbox.id = _dragROI[i]->getId();
			bbox.rect = _dragROI[i]->getGeometry();
			_visionObjectInfo.append(bbox);
		}
	}
}

void VisionApp::updateVisionObjectInfo(bool forceCheck)
{
	if (g_viewMode == int(ViewMode::SINGLE))
	{
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->isSelected() || forceCheck)
			{
				includeVisionObject_into_View(_dragROI[i]);
				_dragROI[i]->update();
			}
		}
	}
	else
	{
		//check if roi is out of view if yes, prompt warning for user to select to proceed with changes or revert
		bool roiOutofViewflag = false;
		bool roiIncluded_into_ViewFlag = false;
		for (int i = 0; i < _dragROI.size(); i++)
		{
			if (_dragROI[i]->isSelected() || forceCheck)
			{
				int rectX = _dragROI.at(i)->getGeometry().x();
				int rectY = _dragROI.at(i)->getGeometry().y();
				int rectW = _dragROI.at(i)->getGeometry().width();
				int rectH = _dragROI.at(i)->getGeometry().height();

				auto vo = _visionObject.find(_dragROI[i]->getId());
				if (vo != _visionObject.end())
				{
					for (int j = 0; j < _viewROI.size(); j++)
					{
						if (_viewROI[j]->getId() == vo.value().viewID && !vo.value().viewID.isEmpty())
						{

							auto viewRect = _viewROI[j]->getGeometry();
							if (rectX < (int)viewRect.x() || rectX + rectW >(int)viewRect.x() + (int)viewRect.width() ||
								rectY < (int)viewRect.y() || rectY + rectH >(int)viewRect.y() + (int)viewRect.height())
							{
								roiOutofViewflag = true;
							}
						}
						else if (vo.value().viewID.isEmpty())
						{
							if (includeVisionObject_into_View(_dragROI[i]))
							{
								_dragROI[i]->update();
								roiIncluded_into_ViewFlag = true;
							}
						}
					}
				}
				_visionObject[_dragROI.at(i)->getId()].rect = ScaleManager::instance().world_to_fov(_dragROI.at(i)->getGeometry());
			}
		}

		if (roiOutofViewflag)
		{
			auto msgBox = QMessageBox(QMessageBox::Information, tr("Confirmation"),
				tr("Some ROIs are moved out of it's view!!!\nPress Yes to proceed with the current changes.\nPress No to revert the changes."),
				QMessageBox::Yes | QMessageBox::No);
			msgBox.setWindowModality(Qt::NonModal);

			//if no is clicked
			if (QMessageBox::No == msgBox.exec())
			{

				for (int i = 0; i < _dragROI.size(); i++)
				{
					if (_dragROI[i]->isSelected() || forceCheck)
					{
						auto id = _dragROI[i]->getId();
						for (int j = 0; j < _visionObjectInfo.size(); j++)
						{
							if (id == _visionObjectInfo[j].id)
							{
								_dragROI[i]->setGeometry(_visionObjectInfo[j].rect);
								_visionObject[_dragROI.at(i)->getId()].rect = ScaleManager::instance().world_to_fov(_dragROI.at(i)->getGeometry());
								break;
							}
						}
					}
				}

				if (roiIncluded_into_ViewFlag) updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
				return;
			}
			else
			{
				//if yes is clicked
				for (int i = 0; i < _dragROI.size(); i++)
				{
					if (_dragROI[i]->isSelected() || forceCheck)
					{
						_visionObject[_dragROI.at(i)->getId()].rect = ScaleManager::instance().world_to_fov(_dragROI.at(i)->getGeometry());
						//if roi is moved from one view to another view
						if (includeVisionObject_into_View(_dragROI[i], true))
						{
							_dragROI[i]->update();
							roiIncluded_into_ViewFlag = true;
						}
						else
						{	//if roi is moved out from a view
							QString viewID;
							auto id = _dragROI[i]->getId();
							auto vo = _visionObject.find(_dragROI[i]->getId());
							if (vo != _visionObject.end())
							{
								viewID = vo.value().viewID;
								vo.value().viewID = "";
								_dragROI[i]->viewID("");

								if (!_views.contains(viewID)) {
									ct::logger::error("[JobThread] Failed to update vision object info. Invalid view ID: %s", viewID.toStdString().c_str());
									return;
								}

								auto v = _views.find(viewID);
								if (v != _views.end())
								{
									for (int j = 0; j < v.value().vision_obj_IDs.size(); j++)
									{
										if (v.value().vision_obj_IDs[j] == id)
										{
											v.value().vision_obj_IDs.remove(j);
											break;
										}
									}
								}
							}
						}

					}
				}

				updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
				return;
			}
		}

		if (roiIncluded_into_ViewFlag) updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
	}
	
}

bool VisionApp::includeVisionObject_into_View(VisionAppQDragBox * dragBox, bool forceCheck)
{
	qDebug() << "includeVisionObject_into_View";
	if (g_viewMode == int(ViewMode::SINGLE))
	{
		qDebug() << "currentViewID:" << ui.label_curViewName->whatsThis();
		QString viewID = ui.label_curViewName->whatsThis();
		dragBox->viewID(viewID);
		auto vo = _visionObject.find(dragBox->getId());
		if (vo != _visionObject.end())
		{
			vo.value().viewID = viewID;
		}
		qDebug() << "dragBoxViewID:" << dragBox->viewID();

		return true;
	}
	else
	{
		bool voIncludedFlag = false;
		if (dragBox->viewID().isEmpty() || forceCheck == true)
		{
			for (int i = 0; i < _viewROI.size(); i++)
			{
				int rectX = dragBox->getGeometry().x();
				int rectY = dragBox->getGeometry().y();
				int rectW = dragBox->getGeometry().width();
				int rectH = dragBox->getGeometry().height();

				auto viewRect = _viewROI[i]->getGeometry();
				if (rectX >= (int)viewRect.x() && rectX + rectW <= (int)viewRect.x() + (int)viewRect.width() &&
					rectY >= (int)viewRect.y() && rectY + rectH <= (int)viewRect.y() + (int)viewRect.height())
				{
					dragBox->viewID(_viewROI[i]->getId());
					auto vo = _visionObject.find(dragBox->getId());
					if (vo != _visionObject.end())
					{
						vo.value().viewID = _viewROI[i]->getId();
						voIncludedFlag = true;
					}
				}
			}
			if (!voIncludedFlag)
			{
				auto vo = _visionObject.find(dragBox->getId());
				dragBox->viewID("");
				vo.value().viewID = "";
			}
		}

		return voIncludedFlag;
	}
	
}

bool VisionApp::includeVisionObject_into_HeightMap(VisionAppQDragBox * dragBox, bool forceCheck)
{
	bool voIncludedFlag = false;
	//if (dragBox->lineScanID().isEmpty() || forceCheck == true)
	//{
	//	for (int i = 0; i < _lineScanROI.size(); i++)
	//	{
	//		int rectX = dragBox->getGeometry().x();
	//		int rectY = dragBox->getGeometry().y();
	//		int rectW = dragBox->getGeometry().width();
	//		int rectH = dragBox->getGeometry().height();

	//		auto hm_id = _lineScanROI[i]->getId();
	//		auto hm = _lineScanROI[i]->getGeometry();
	//		if (rectX >(int)hm.x() && rectX + rectW <(int)hm.x() + (int)hm.width() &&
	//			rectY >(int)hm.y() && rectY + rectH <(int)hm.y() + (int)hm.height())
	//		{
	//			dragBox->lineScanID(hm_id);
	//			auto vo = _visionObject.find(dragBox->getId());

	//			_lineScans[hm_id].vision_obj_IDs.append(dragBox->getId());

	//			if (vo != _visionObject.end())
	//			{
	//				vo.value().lineScanID = hm_id;
	//				voIncludedFlag = true;
	//			}
	//		}
	//	}

	//	if (!voIncludedFlag)
	//	{
	//		auto vo = _visionObject.find(dragBox->getId());
	//		dragBox->viewID("");
	//		vo.value().viewID = "";
	//	}
	//}

	return voIncludedFlag;
}

void VisionApp::copyVisionObject()
{
	qDebug() << "CopyVisionObject";
	_copiedVisionObjectIDs.clear();
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->isSelected())
		{
			_copiedVisionObjectIDs.append(_dragROI[i]->getId());
		}
	}
}

void VisionApp::pasteVisionObject()
{
	qDebug() << "PasteVisionObject";
	for (int i = 0; i < _dragROI.count(); i++)
	{
		_dragROI[i]->setSelected(false);
	}

	for (int i = 0; i < _copiedVisionObjectIDs.size(); i++)
	{

		QString defaultKey = QStringLiteral("object_");
		QString newKey = dragROINameGenerator(defaultKey);

		auto vo = _visionObject.find(_copiedVisionObjectIDs[i]);
		if (vo != _visionObject.end())
		{
			QVisionObject visionObject;
			uidGenerator uidGen;
			visionObject.objectName = newKey;
			visionObject.objectID = "object" + QString(uidGen.id().c_str());

			visionObject.templateName = vo.value().templateName;
			visionObject.templateID = vo.value().templateID;
			//visionObject.skip = vo.value().skip;
			visionObject.locked = false;
			visionObject.angle = vo.value().angle;

			int offset = 50;
			QRectF roi = vo.value().rect.adjusted(offset, offset, offset, offset);
			visionObject.rect = roi;



			roi = ScaleManager::instance().fov_to_world(roi);

			qDebug() << "visionObjectRoi:" << roi;
			QColor color = _templateLibraryTab->getTemplateColor(visionObject.templateID);
			auto visionAppDragBox = drawVisionAppDragBox(roi, color, visionObject.objectName, visionObject.viewID);
			visionAppDragBox->algoTemplate(_templateLibraryTab->getAlgoTemplate(visionObject.templateID));
			visionAppDragBox->setID(visionObject.objectID);
			visionAppDragBox->setSelected(true);

			visionObject.pDragBox = visionAppDragBox;
			visionObject.pDragBox->type((int)DragBoxType::VISIONOBJECT);
			_visionObject.insert(visionObject.objectID, visionObject);

			if (includeVisionObject_into_View(visionAppDragBox))
			{
				visionAppDragBox->update();
			}

			if (includeVisionObject_into_HeightMap(visionAppDragBox)) {
				visionAppDragBox->update();
			}
		}
	}

	refreshDragBoxSequence();
	updateTreeViewExplorer(Common::Directory::CurrentRecipe, _views, _visionObject);
}

void VisionApp::updateEmap()
{
	// 0= fail, 1 = pass;
	qDebug() << "update Emap";
	// update emap hash first
	for (auto& vo : _visionObject)
	{
		QString key = QString::number(vo.row_id) + "[@]" + QString::number(vo.col_id) + "[@]" + QString::number(vo.island_id);

		if (!vo.isPass || vo.ignore)
		{
			// fail and incoming considered defect in emap
			_eMapHash[key] = false;
			if(vo.skip||vo.forcedSkip) _eMapHash[key] = true; // if skip vo consider pass
		}
		else
		{
			
			_eMapHash[key] = true;
		}
	
	}




	// now write a new emap file 
	QString stripeID = SystemData::instance()._currentBarcode.c_str();


	QString updatedEmapFilePath = Common::Directory::getProductionResultPath() + "/" + stripeID + ".dat";
	QFile file1(updatedEmapFilePath);
	bool fileSuc = file1.open(QIODevice::WriteOnly | QIODevice::Text);
	if (fileSuc)
	{
		QTextStream out(&file1);

		out << "STRIP MAP = {" << "\n";
		out << "STRIP_ID = \"" << stripeID << "\"\n";
		out << "MAP = {" << "\n";

		for (int i = 0; i < _islandInfo.totalRow; i++)
		{
			for (int j = 0; j < _islandInfo.totalIsland; j++)
			{
				for (int k = 0; k < _islandInfo.totalCol; k++)
				{
					QString key = QString::number(i) + "[@]" + QString::number(k) + "[@]" + QString::number(j);
					QString value = _eMapHash[key] == true ? "1" : "0";
					out << value;
					if (k == _islandInfo.totalCol - 1 && j == _islandInfo.totalIsland - 1 && i == _islandInfo.totalRow - 1)
					{
						// last row
						out << "}\n}";
					}
				}
				out << " ";
			}
			out << "\n";
		}

	}
	file1.close();

}

void VisionApp::readIVEmap(QString productionFolderPath)
{
	QString EmapPath = QString(Common::Directory::getRecipeCurrentPath());
	qDebug() << "Emap Path:" << EmapPath;
	QString filePath = QFileDialog::getOpenFileName(this, tr("Select Emap file  to compare"), EmapPath, "txt File (*.txt)");
	QString IVfolderPath = QFileDialog::getExistingDirectory(nullptr, "Select Folder", Common::Directory::getRecipeCurrentPath());
	//QString filePath = "C:/Advanced/Data/recipe/GRNL/resultsEmap/GRNLX2001_MGP830803900_3508.txt"; // Replace with the actual file path

	QFile file(filePath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Failed to open emap file.";
		return;
	}

	// Read the file contents
	QTextStream in(&file);

	QVector<QVector<int>> matrix;
	QVector<bool> resultsIsPass;
	int count = 0;
	while (!in.atEnd()) {
		QString line = in.readLine();
		QVector<int> row;
		if (line.contains(";"))
		{
			for (int i = 0; i < line.length(); i++) {
				if (line[i] == '0')
				{
					resultsIsPass.append(false);
					row.append(0);
					count++;
				}
				else if (line[i] == '1')
				{
					resultsIsPass.append(true);
					row.append(1);
					count++;
				}

			}
		}

		/*qDebug() << "colCount:" << row.count();
		qDebug() << "rows:" << row;*/
		matrix.append(row);
	}

	// Close the file
	file.close();

	QStringList viewIDs;
	QHash<QString, QString> mismatchedVODefects;
	if (resultsIsPass.size() == _dragROI.size())
	{
		int escapeeCount = 0;
		int falseCallCount = 0;
		for (int i = 0; i < _dragROI.size(); i++)
		{
			QString id = _dragROI[i]->getId();
			auto vo = _visionObject.find(id);
			vo.value().viewID;
			if (resultsIsPass[i] != vo.value().isPass)
			{
				if (!viewIDs.contains(vo.value().viewID))
				{
					viewIDs.append(vo.value().viewID);
				}
				QString status;
				if (resultsIsPass[i])
				{
					falseCallCount++;
					status = "falseCall";
					qDebug() << "index:" << _dragROI[i]->_index << " voName:" << vo.value().objectName << "  FalseCall" << " isPass:" << resultsIsPass[i];

				}
				else
				{
					escapeeCount++;
					status = "escapee";
					qDebug() << "index:" << _dragROI[i]->_index << " voName:" << vo.value().objectName << "  Escapee" << " isPass:" << resultsIsPass[i];
				}
				_dragROI[i]->_comparisonStatus = status;
				mismatchedVODefects.insert(vo.value().objectName, status);
			}
			else
			{
				_dragROI[i]->_comparisonStatus.clear();
			}
		}
		qDebug() << "escapeeCount:" << escapeeCount;
		qDebug() << "falseCallCount:" << falseCallCount;
	}

	QString defectPath = Common::Directory::getProductionDefectPath();
	if (!productionFolderPath.isEmpty()) defectPath = productionFolderPath + "/Defects/";
	QDir directory(defectPath);

	// Get a list of all image files in the directory
	QStringList subDirectories = directory.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
	// Loop through each subdirectory and recursively search for image files
	QStringList imageFiles;
	QStringList imageFilePaths;
	for (const QString& subDirName : subDirectories) {
		QDir subDir(directory.absoluteFilePath(subDirName));
		QStringList files = subDir.entryList(QStringList() << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp" << "*.gif", QDir::Files);
		imageFiles.append(files);

		for (const QString& filename : files) {
			imageFilePaths.append(subDir.absoluteFilePath(filename));
		}
	}

	// Loop through each image file in the directory and print their names
	/*for (const QString& filename : imageFilePaths) {
		qDebug() << "Image file Path:" << filename;
	}*/

	QString falseCallPath = defectPath + QString("FalseCall/");
	CreateDirectoryA(falseCallPath.toStdString().c_str(), NULL);
	QString escapeePath = defectPath + QString("Escapee/");
	CreateDirectoryA(escapeePath.toStdString().c_str(), NULL);
	QHash<QString, QString>::const_iterator def = mismatchedVODefects.constBegin();

	while (def != mismatchedVODefects.constEnd())
	{
		for (int i = 0; i < imageFiles.size(); i++)
		{
			auto optic = _recipeOptics.constBegin();
			while (optic != _recipeOptics.constEnd())
			{

				QString voName = def.key() + "_" + optic.value().name;
				//QString voName = def.key() + "_RB";
				QFileInfo file(imageFiles[i]);
				if (file.baseName() == voName || file.baseName() == QString(voName + "_def"))
				{
					QString destinationPath;
					if (def.value() == "falseCall")
					{
						destinationPath = falseCallPath + imageFiles[i];
					}
					else if (def.value() == "escapee")
					{
						destinationPath = escapeePath + imageFiles[i];
					}

					QFile::copy(imageFilePaths[i], destinationPath);
				}
				optic++;
			}
		}
		def++;
	}

	QStringList IVimageFiles;
	QStringList IVimageFilePaths;

	QDir IVsubDir(IVfolderPath);
	QStringList IVfiles = IVsubDir.entryList(QStringList() << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp" << "*.gif", QDir::Files);
	IVimageFiles.append(IVfiles);

	for (const QString& IVfilename : IVfiles) {
		IVimageFilePaths.append(IVsubDir.absoluteFilePath(IVfilename));
	}

	QHash<QString, QString>::const_iterator def1 = mismatchedVODefects.constBegin();
	while (def1 != mismatchedVODefects.constEnd())
	{
		for (int i = 0; i < IVimageFiles.size(); i++)
		{
			QString voName = def1.key();

			QFileInfo file(IVimageFiles[i]);
			QStringList splitName = file.baseName().split("_");
			QString IVvoName;
			QString IVdefName = "unknown";
			if (splitName.size() > 4)
			{
				IVvoName = voName.left(2) + splitName[2] + splitName[3];
				IVdefName = splitName[1];
			}
			if (IVvoName == voName)
			{
				QString destinationPath;
				if (def1.value() == "falseCall")
				{
					destinationPath = falseCallPath + IVvoName + "_IV_" + IVdefName + "_" + splitName[4] + ".jpg";
				}
				else if (def1.value() == "escapee")
				{
					destinationPath = escapeePath + IVvoName + "_IV_" + IVdefName + "_" + splitName[4] + ".jpg";
				}

				QFile::copy(IVimageFilePaths[i], destinationPath);
			}
		}
		def1++;
	}

	//for (int i = 0; i < viewIDs.size(); i++)
	//{
	//	auto v = _views.value(viewIDs[i]);
	//	auto ipf = path::getViewPath(Common::Directory::CurrentImageSetPath.toStdString(), v);

	//	QImage img;
	//	img.load(ipf.GetPath().c_str());

	//	for (auto& def : mismatchedVODefects)
	//	{
	//		auto vo = _visionObject.find(def);
	//	/*	auto x = FOVrect.x() - v.px.xmin;
	//		auto y = FOVrect.y() - v.px.ymin;
	//		auto w = FOVrect.width();
	//		auto h = FOVrect.height();*/
	//	}
	//}
}

void VisionApp::saveComparisonEmap()
{
	auto col = ui.lineEdit_column->text().toInt() * 2;
	auto row = ui.lineEdit_row->text().toInt();

	QString comparisonEmapPath = Common::Directory::CachePath + "comparisonEmap.txt";
	std::ofstream outFile(comparisonEmapPath.toStdString().c_str());

	if (outFile.is_open()) {
		for (int i = 0; i < _dragROI.size(); i++) {
			// Write the value to the file

			int status = 0;
			if (_dragROI[i]->_comparisonStatus == "falseCall") status = 1;
			else if (_dragROI[i]->_comparisonStatus == "escapee") status = 2;
			outFile << status << "";

			// Check if it's time to start a new row
			if ((i + 1) % col == 0) {
				outFile << "\n"; // Move to the next line
			}
		}

		// Close the file
		outFile.close();
		std::cout << "Data saved to output.txt." << std::endl;
	}
	else {
		std::cout << "Unable to open the file." << std::endl;
	}


}

void VisionApp::readEmap()
{
	qDebug() << "ReadEmap: "<< _enableEmap;
	qDebug() << "Force Read Emap: " << _lotInfo.forceEmap;
	Timer time;
	// reset Emap;
	_eMapHash.clear();
	for (auto& vo : _visionObject)
	{
		vo.ignore = false;
	}

	
	if (_enableEmap)
	{
		bool readEmapSuc;
		// reset ignore vo 
		for (auto& vo : _visionObject)
		{
			vo.ignore = false;
		}

		if (_emapInfo.mode == EmapMode::AUTO)
		{
			if (_lotInfo.isTop)
			{
				if (_emapInfo.topInspEmap == EmapType::CSV01_EMAP)readEmapSuc = readEmap_Csv01();
				else if (_emapInfo.topInspEmap == EmapType::CSV34_EMAP)readEmapSuc = readEmap_Csv34();
				else if (_emapInfo.topInspEmap == EmapType::TEXT_FILE_EMAP)readEmapSuc = readEmap_textFile();
				
			}
			else
			{
				if (_emapInfo.botInspEmap == EmapType::CSV01_EMAP)	readEmapSuc = readEmap_Csv01();
				else if (_emapInfo.botInspEmap == EmapType::CSV34_EMAP)	readEmapSuc = readEmap_Csv34();
				else if (_emapInfo.botInspEmap == EmapType::TEXT_FILE_EMAP)readEmapSuc = readEmap_textFile();
				
			}
		}
		else if (_emapInfo.mode == EmapMode::CSV01)	readEmapSuc = readEmap_Csv01();
		else if (_emapInfo.mode == EmapMode::CSV34)	readEmapSuc = readEmap_Csv34();
		else if (_emapInfo.mode == EmapMode::TEXT_FILE)readEmapSuc = readEmap_textFile();
	
		if (!readEmapSuc && _inspStatus.productionMode == true) {
			stopRun();
			sendToClient("01VISIONEMAPFAIL\r");
		}

		if (readEmapSuc && !_eMapHash.isEmpty() && _inspStatus.productionMode == true)
		{
			// if incoming fails more than 50% prompt alarm
			int failIncomingCount = 0;
			for (auto i : _eMapHash)
			{
				if (i == false) failIncomingCount++;
			}
			double percentage = (double(failIncomingCount) / double(_eMapHash.size())) * 100;
			if (percentage > jsonHelper::getInteger(_systemObj, QStringLiteral("Emap_Fail_Alarm_Percentage")))
			{
				stopRun();
				sendToClient("01VISIONEMAP50FAIL\r");
			}
		}

		qDebug() << "ReadEmapStatus: " << readEmapSuc;
		qDebug() << "_inspStatus.productionMode " << _inspStatus.productionMode;
	}

	
	qDebug() << "Emap Processing time: " << time.duration();
}

bool VisionApp::readEmap_Csv01()
{
	qDebug() << "readEmap_csv01";
	QHash<QString, bool> emapHash;
	QString emapPath;

	//SystemData::instance()._currentBarcode = "GRNLX2001_MGP850505700_0402";
	bool emapFound = false;
	for (auto p : _emapInfo.csvEmapDir)
	{

		QDir directory(p);
		emapPath = p + "/" + SystemData::instance()._currentBarcode.c_str() + ".csv";
		qDebug() << "Emap Searching: " << emapPath;
		if (QFileInfo::exists(emapPath))
		{
			qDebug() << "Emap found: " << emapPath;
			_emapInfo.incomingEmapPath = emapPath;
			emapFound = true;
			break;
		}

	}
	if (emapFound == false) return false;


	if (emapPath.isEmpty())
	{
		qDebug() << "StripeID: " << SystemData::instance()._currentBarcode.c_str();
		qDebug() << "Read CSV_Emap Failed";
		return false;
	}

	QFile file(emapPath);

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Failed to open csv emap file.";
		qDebug() << "Emap Path: " << emapPath;
		return false;
	}
	else
	{
		qDebug() << "Reading csv Emap: " << emapPath;
	}

	QTextStream in(&file);
	int readingLine = 0;

	QStringList headers;

	QStringList numberList;

	while (!in.atEnd())
	{
		readingLine++;
		QString line = in.readLine();
		if (readingLine == 2)
		{
			numberList = line.split(",");
		}
		else
		{
			headers.append(line);
		}
	}

	int tRow = _islandInfo.totalRow;
	int tCol = _islandInfo.totalCol;
	int tIsland = _islandInfo.totalIsland;

	int row = 0;
	int col = 0;
	int island = 0;
	for (int i = 0; i < numberList.size(); i++)
	{
		//bool isPass = numberList[i] == "4" ? true : false; // custom emap for declan
		bool isPass = numberList[i] == "1" ? true : false;

		if (col >= tCol)
		{
			island++;
			if (island >= tIsland)
			{
				island = 0;
				row++;
			}

			col = 0;
		}
		if (row >= tRow) break;
		QString key = QString::number(row) + "[@]" + QString::number(col) + "[@]" + QString::number(island);

		emapHash.insert(key, isPass);
		col++;
	}
	file.close();


	if (emapHash.isEmpty()) return false;
	_eMapHash = emapHash;

	for (auto& vo : _visionObject)
	{
		QString key = QString::number(vo.row_id) + "[@]" + QString::number(vo.col_id) + "[@]" + QString::number(vo.island_id);
		if (_eMapHash.contains(key))
		{
			vo.ignore = !_eMapHash[key];
		}

	}

	// copy emap to production
	QString emapDestination = Common::Directory::getProductionResultPath() + "/IncomingMap[@]" + SystemData::instance()._currentBarcode.c_str() + ".csv";
	bool emapCopy = QFile::copy(emapPath, emapDestination);

	qDebug() << "Total Vo Unit: " << _visionObject.size();
	qDebug() << "Emap Hash size: " << _eMapHash.size();

	return true;

}

bool VisionApp::readEmap_Csv34()
{
	qDebug() << "readEmap_csv";
	QHash<QString, bool> emapHash;
	QString emapPath;

	bool emapFound = false;
	for (auto p : _emapInfo.csvEmapDir)
	{

		QDir directory(p);
		emapPath = p + "/Badmark_Inf_" + SystemData::instance()._currentBarcode.c_str() + ".csv";
		qDebug() << "Emap Searching: " << emapPath;
		if (QFileInfo::exists(emapPath))
		{
			qDebug() << "Emap found: " << emapPath;
			_emapInfo.incomingEmapPath = emapPath;
			emapFound = true;
			break;
		}
		
	}
	if (emapFound == false) return false;


	if (emapPath.isEmpty())
	{
		qDebug() << "StripeID: " << SystemData::instance()._currentBarcode.c_str();
		qDebug() << "Read CSV_Emap Failed";
		return false;
	}

	QFile file(emapPath);

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Failed to open csv emap file.";
		qDebug() << "Emap Path: " << emapPath;
		return false;
	}
	else
	{
		qDebug() << "Reading csv Emap: " << emapPath;
	}

	QTextStream in(&file);
	int readingLine = 0;

	QStringList headers;

	QStringList numberList;

	while (!in.atEnd())
	{
		readingLine++;
		QString line = in.readLine();
		if (readingLine == 4) 
		{
			numberList = line.split(",");
		}
		else
		{
			headers.append(line);
		}
	}

	int tRow = _islandInfo.totalRow;
	int tCol = _islandInfo.totalCol;
	int tIsland = _islandInfo.totalIsland;

	int row = 0;
	int col = 0;
	int island = 0;
	for (int i = 0; i < numberList.size(); i++)
	{
		bool isPass = numberList[i] == "4" ? true : false; // custom emap for declan
		//bool isPass = numberList[i] == "1" ? true : false;
		if (col >= tCol)
		{
			island++;
			if (island >= tIsland)
			{
				island = 0;
				row++;
			}

			col = 0;
		}
		if (row >= tRow) break;
		QString key = QString::number(row) + "[@]" + QString::number(col) + "[@]" + QString::number(island);
		emapHash.insert(key, isPass);
		col++;
	}
	file.close();


	if (emapHash.isEmpty()) return false;
	_eMapHash = emapHash;

	for (auto& vo : _visionObject)
	{
		QString key = QString::number(vo.row_id) + "[@]" + QString::number(vo.col_id) + "[@]" + QString::number(vo.island_id);
		if (_eMapHash.contains(key))
		{
			vo.ignore = !_eMapHash[key];
		}

	}
	

	// copy emap to production
	QString emapDestination = Common::Directory::getProductionResultPath() +"/IncomingMap[@]"+  SystemData::instance()._currentBarcode.c_str() + ".csv";
	bool emapCopy = QFile::copy(emapPath, emapDestination);

	qDebug() << "Total Vo Unit: " << _visionObject.size();
	qDebug() << "Emap Hash size: " << _eMapHash.size();

	return true;

}


bool VisionApp::readEmap_textFile()
{
	Timer time;
	qDebug() << "readEmap_textFile";
	QHash<QString, bool> emapHash;

	QString emapPath;

	bool emapFound = false;
	for (auto p : _emapInfo.textFileEmapDir)
	{
		QDir directory(p);

		emapPath = p + "/" + SystemData::instance()._currentBarcode.c_str() + ".txt";
		qDebug() << "Emap Searching: " << emapPath;
		if (QFileInfo::exists(emapPath))
		{
			qDebug() << "Emap found: " << emapPath;
			_emapInfo.incomingEmapPath = emapPath;
			emapFound = true;
			break;
		}
	}
	if (emapFound == false) return false;


	//QString emapPath;
	//emapPath = _emapInfo.csvEmapDir + "/" + SystemData::instance()._currentBarcode.c_str() + ".txt";
	
	if (emapPath.isEmpty())
	{
		qDebug() << "StripeID: " << SystemData::instance()._currentBarcode.c_str();
		qDebug() << "Read TXT_Emap Failed";
		return false;
	}

	QFile file(emapPath);

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Failed to open txt emap file.";
		qDebug() << "Emap Path: " << emapPath;
		return false;
	}
	else
	{
		qDebug() << "Reading txt Emap: " << emapPath;
	}


	QTextStream in(&file);
	int readingLine = 0;


	QStringList numberList;

	while (!in.atEnd())
	{
		QString line = in.readLine();

		if (line.isEmpty()) continue;
		readingLine++;
		if (readingLine > 3)
		{
			//qDebug() << line;
			QStringList value = line.split(";", QString::SkipEmptyParts);
			for (QString& v : value) 
			{
				v = v.trimmed();
			}
			if (!value.isEmpty())
			{
				numberList.append(value);
			}
		}


	}

	int tRow = _islandInfo.totalRow;
	int tCol = _islandInfo.totalCol;
	int tIsland = _islandInfo.totalIsland;

	int row = 0;
	int col = 0;
	int island = 0;
	for (int i = 0; i < numberList.size(); i++)
	{
		//bool isPass = numberList[i] == "1" ? true : false;
		bool isPass = numberList[i] == "0" ? false : true;

		if (col >= tCol)
		{
			island++;
			if (island >= tIsland)
			{
				island = 0;
				row++;
			}

			col = 0;
		}
		if (row >= tRow) break;
		QString key = QString::number(row) + "[@]" + QString::number(col) + "[@]" + QString::number(island);
		
		emapHash.insert(key, isPass);
		col++;
	}

	file.close();

	if (emapHash.isEmpty()) return false;
	_eMapHash = emapHash;
	

	for (auto& vo : _visionObject)
	{
		QString key = QString::number(vo.row_id) + "[@]" + QString::number(vo.col_id) + "[@]" + QString::number(vo.island_id);
		if (_eMapHash.contains(key))
		{
			vo.ignore = !_eMapHash[key];
		}

	}

	// copy emap to production
	QString emapDestination = Common::Directory::getProductionResultPath() + "/IncomingMap[@]" + SystemData::instance()._currentBarcode.c_str() + ".txt";
	bool emapCopy = QFile::copy(emapPath, emapDestination);

	qDebug() << "Total Vo Unit: " << _visionObject.size();
	qDebug() << "Emap Hash size: " << _eMapHash.size();

	qDebug() << "txt Emap searching time: " << time.duration();

	return true;
}

void VisionApp::visionObjectSampling()
{
	

	auto col = ui.lineEdit_column->text().toInt() * _islandInfo.totalIsland;
	auto row = ui.lineEdit_row->text().toInt();

	qDebug() << "Sampling started....";
	qDebug() << "Row: " << row;
	qDebug() << "Column: " << col;
	qDebug() << "Unit size: " << _dragROI.size();
	if (row * col != _dragROI.size())
	{
		qDebug() << "Sampling failed, row col number not match to unit size!";
		return;
	}

	int inspectRow = 3;
	// reset skip vo 
	for (auto& vo : _visionObject)
	{
		vo.skip = true;
	}


	std::sort(_dragROI.begin(), _dragROI.end(), compareRectanglesByY_ascending);
	for (int i = 0; i < row; i++)
	{
		std::sort(_dragROI.begin() + i*col,
			_dragROI.begin() + ((i + 1)*col),
			compareRectanglesByX_ascending
		);
	}
	for (int i = 0; i < _dragROI.size(); i++)
	{
		_dragROI[i]->_index = i + 1;
		//qDebug() << "index" << i << " voWidth:" << _dragROI[i]->getGeometry().x() << " voHeight:" << _dragROI[i]->getGeometry().y();
	}
	int split = (row / 5) + 1;
	for (int a = 0; a < 5; a++)
	{
		if ((a + 1) % 2 != 0) 
		{
			for (int i = 0; i < split; i++)
			{
				if (i < inspectRow)
				{
					int curRow = a*split + i;
					for (int j = 0; j < col; j++)
					{
					
						int dragROIIndex = curRow*col + j;
						if (curRow > row) continue;
						if (_dragROI.size() <= dragROIIndex) continue;
					
						auto v = _visionObject.find(_dragROI[dragROIIndex]->getId());
						if (v != _visionObject.end())
						{
							v.value().skip = false;
							
						}
						
					}
				}
					
			}	
		}
	}



	qDebug() << "doneVisionObjectSampling";
}

bool VisionApp::testEmapID()
{
	int tIsland = _islandInfo.totalIsland;
	auto col = ui.lineEdit_column->text().toInt() * tIsland;
	auto row = ui.lineEdit_row->text().toInt();

	

	if ((col * row) != _dragROI.size())
	{
		qDebug() << "rowColumnCount:" << (col * row) << " voSize:" << _dragROI.size();
		QMessageBox::warning(this, ("Invalid Row Column!"),
			"Invalid Row Column with number of vision object!!!! Please recheck!!!");

		return false;
	}

	clearAllDefectRectShape();

	bool isTop = true;
	checkRecipeFacing(Common::Directory::CurrentRecipe, isTop);
	//top
	if (isTop)
	{
		//qDebug() << "isTOP";
		std::sort(_dragROI.begin(), _dragROI.end(), compareRectanglesByY_ascending);
		for (int i = 0; i < row; i++)
		{
			//qDebug() << "begin:" << i*col << " end:" << ((i + 1)*col - 1);
			std::sort(_dragROI.begin() + i*col, _dragROI.begin() + ((i + 1)*col), compareRectanglesByX_ascending);
		}
		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI[i]->_index = i + 1;
		}
	}
	else //btm
	{
		//qDebug() << "isBtm";
		std::sort(_dragROI.begin(), _dragROI.end(), compareRectanglesByY_descending);

		for (int i = 0; i < row; i++)
		{
			//qDebug() << "begin:" << i*col << " end:" << ((i + 1)*col - 1);
			std::sort(_dragROI.begin() + i*col, _dragROI.begin() + ((i + 1)*col), compareRectanglesByX_ascending);
		}
		for (int i = 0; i < _dragROI.size(); i++)
		{
			_dragROI[i]->_index = i + 1;
		}
		
	}
	
	bool isPass = true;
	for (int i = 0; i < _dragROI.size(); i++)
	{
		int rowID = _visionObject[_dragROI[i]->getId()].row_id;
		int colID = _visionObject[_dragROI[i]->getId()].col_id;
		int islandID = _visionObject[_dragROI[i]->getId()].island_id;

		int rowSize = ui.lineEdit_row->text().toInt();
		int colSize = ui.lineEdit_column->text().toInt();

		int index = _dragROI[i]->_index;

		int voID = rowID*(colSize * tIsland) + (colID + 1) + (colSize*islandID);
		if (voID != index)
		{
			isPass = false;
			QString invalid_voID = QString("INVALID voID:") + QString::number(voID) + "index:" + QString::number(index);
			qDebug() << "INVALID voID:" << voID << " index:" << index;
			auto rectItem = drawDefectRect(_dragROI[i]->getGeometry(), invalid_voID, invalid_voID,QString(), QString(),QString(), Qt::magenta);
		}
	}
	if (isPass)
	{
		ui.label_testEmapStatus->setText("Success");
		ui.label_testEmapStatus->setStyleSheet("color: green;");
	}
	else
	{
		ui.label_testEmapStatus->setText("Failed");
		ui.label_testEmapStatus->setStyleSheet("color: red;");
	}

	return true;
}

void VisionApp::clearDirectory(const QString& path)
{
	QDir dir(path);

	if (!dir.exists()) {
		qWarning() << "Directory does not exist:" << path;
		return;
	}

	// Get the list of all files in the directory
	QStringList files = dir.entryList(QDir::Files);

	for (const QString& file : files) {
		QString filePath = dir.absoluteFilePath(file);
		if (!dir.remove(filePath)) {
			qWarning() << "Failed to remove file:" << filePath;
		}
	}

	qDebug() << "All files in" << path << "have been removed.";
}

bool VisionApp::copyFolderRecursively(const QString & srcFolderPath, const QString & destFolderPath)
{
	QDir sourceFolder(srcFolderPath);
	QDir destinationFolder(destFolderPath);

	// Ensure the source folder exists
	if (!sourceFolder.exists()) {
		qDebug() << "Source folder does not exist: " << srcFolderPath;
		return false;
	}

	// Create the destination folder if it doesn't exist
	if (!destinationFolder.exists()) {
		if (!destinationFolder.mkpath(".")) {
			qDebug() << "Failed to create destination folder: " << destFolderPath;
			return false;
		}
	}

	QStringList files = sourceFolder.entryList(QDir::Files);
	foreach(const QString &file, files) {
		QString srcFilePath = sourceFolder.filePath(file);
		QString destFilePath = destinationFolder.filePath(file);

		// Copy the file from source to destination
		if (!QFile::copy(srcFilePath, destFilePath)) {
			qDebug() << "Failed to copy file: " << srcFilePath;
			return false;
		}

		// Optionally, you can remove the file from the source folder
		if (!QFile::remove(srcFilePath)) {
			qDebug() << "Failed to remove file from source: " << srcFilePath;
			return false;
		}
	}

	QStringList folders = sourceFolder.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
	foreach(const QString &folder, folders) {
		QString srcFolderPath = sourceFolder.filePath(folder);
		QString destFolderPath = destinationFolder.filePath(folder);

		// Recursively copy subfolders
		if (!copyFolderRecursively(srcFolderPath, destFolderPath)) {
			return false;
		}

		// Optionally, you can remove the folder from the source folder
		if (!sourceFolder.rmdir(folder)) {
			qDebug() << "Failed to remove folder from source: " << srcFolderPath;
			return false;
		}
	}

	return true;
}



void VisionApp::importOptic()
{
	QString filePath = QFileDialog::getOpenFileName(
		this,
		tr("Open File"),
		Common::Directory::RecipePath(),
		tr("JSON Files (optics.json);;All Files (*)"));

	if (!filePath.isEmpty())
	{
		QFileInfo f(filePath);
		if (f.absoluteDir().dirName() == Common::Directory::CurrentRecipe)
		{
			QMessageBox::information(this, ("Import Failed"),
				"Cannot import optics from own recipe!");
			return;
		}



		QString originalOpticPath = QStringLiteral("%1recipe/%2/optics.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

		if (QFile::exists(originalOpticPath))
		{
			qDebug() << "Backup old optic folder to backup optic folder";
			QString backupFolderPath = QStringLiteral("%1recipe/%2/BackupOpticJson").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

			// Create the backup folder if it doesn't exist
			QDir backupFolder(backupFolderPath);
			if (!backupFolder.exists())
			{
				if (!backupFolder.mkpath("."))
				{
					qDebug() << "Error creating backup folder.";
					return;
				}
			}

			// Get the file name without the path
			QFileInfo fileInfo(originalOpticPath);
			QDir directory(backupFolderPath);
			directory.setFilter(QDir::Files | QDir::NoDotAndDotDot);
			QString fileName = fileInfo.fileName() + "_" + QString::number(directory.entryList().size());

			// Construct the destination path in the backup folder
			QString backupFilePath = QStringLiteral("%1/%2").arg(backupFolderPath).arg(fileName);

			// Try to move the file to the backup folder
			if (QFile::rename(originalOpticPath, backupFilePath))
			{
				qDebug() << "File moved to backup folder.";
			}
			else
			{
				qDebug() << "Error moving file to backup folder.";
				// Handle the error as needed
			}



		}
		if (QFile::copy(filePath, originalOpticPath))
		{
			loadRecipeOptics();
		}
	}
}

void VisionApp::saveAsLightingTemplate()
{
	QString templateLightingName = ui.lineEdit_lightingTemplateName->text();
	if (templateLightingName.isEmpty())
	{
		QMessageBox::warning(this, ("Lighting Template name cannot be emptied!"),
			"A lighting template name must be given before creating one.");

		return;
	}

	AuditLog::instance().log(QStringLiteral("LIGHTING_TEMPLATE_SAVE"), templateLightingName);

	QString lightingTemplateDir = Common::Directory::getLightingTemplatePath();
	QDir dir(lightingTemplateDir);
	if (!dir.exists())
		dir.mkpath(lightingTemplateDir);

	QString lightingTemplatePath = lightingTemplateDir + templateLightingName + ".json";

	if (QFileInfo::exists(lightingTemplatePath))
	{
		if (QMessageBox::Yes == QMessageBox(QMessageBox::Information,
			"Lighting Template Existed!", "Do you want to replace existed lighting template?"
			, QMessageBox::Yes | QMessageBox::No).exec())
		{
			QFile::remove(lightingTemplatePath);
		}
		else
		{
			return;
		}
	}
	saveLightingTemplateJson(lightingTemplatePath);
	refreshLightingTemplateComboBox();
	
}

void VisionApp::saveLightingTemplateJson(QString filePath)
{
	// save json
	auto jsonPath = filePath;

	QJsonArray j_optics;

	for (auto o : _recipeOptics) {
		QJsonObject j_optic;
		j_optic.insert(QStringLiteral("id"), o.id);
		j_optic.insert(QStringLiteral("name"), o.name);
		j_optic.insert(QStringLiteral("type"), o.type);
		j_optic.insert(QStringLiteral("tag"), o.tag);

		if (o.type == ct::s_color && CAMManager::instance().getChannel(_camID) == 1) {
			QJsonArray j_R;
			for (auto r : o.R) {
				j_R.append(r);
			}

			QJsonArray j_G;
			for (auto g : o.G) {
				j_G.append(g);
			}

			QJsonArray j_B;
			for (auto b : o.B) {
				j_B.append(b);
			}

			j_optic.insert(QStringLiteral("R"), j_R);
			j_optic.insert(QStringLiteral("G"), j_G);
			j_optic.insert(QStringLiteral("B"), j_B);
		}
		else {
			QJsonArray j_M;
			for (auto m : o.M) {
				j_M.append(m);
			}

			j_optic.insert(QStringLiteral("M"), j_M);
		}

		j_optics.append(j_optic);
	}

	QJsonObject obj;
	obj.insert(QStringLiteral("optics"), j_optics);

	auto ret = saveJson(jsonPath, QJsonDocument(obj));

	if (ret)
	{
		QMessageBox::information(this, ("Template created successlly"),
			"Template created successlly");
		ui.lineEdit_lightingTemplateName->clear();
	}
	else
	{
		QMessageBox::warning(this, ("Template failed to create!"),
			"Template failed to create!");

	}
	
}

void VisionApp::loadLightingTemplate()
{

	QString lightingTemplateName = ui.comboBox_ligtingTemplateList->currentText();
	if (lightingTemplateName.isEmpty())
	{
		QMessageBox::warning(this, ("Lighting Template name cannot be emptied!"),
			"Error in loading lighting template");
		return;
	}
	QString filePath = Common::Directory::getLightingTemplatePath() + "/" + lightingTemplateName + ".json";
	if (!QFileInfo::exists(filePath))
	{
		QMessageBox::warning(this, ("Error in loading lighting template"),
			"Lighting Template does not exist!");
		AuditLog::instance().log(QStringLiteral("LIGHTING_TEMPLATE_IMPORT"), lightingTemplateName, QStringLiteral("FAILED"));
		return;
	}

	if (QMessageBox::Yes == QMessageBox(QMessageBox::Information,
		"Do you want to load " + lightingTemplateName + " lighting template?", "Current lighting setting will not be saved!"
		, QMessageBox::Yes | QMessageBox::No).exec())
	{
		AuditLog::instance().log(QStringLiteral("LIGHTING_TEMPLATE_IMPORT"), lightingTemplateName);
		if (!filePath.isEmpty())
		{
			QFileInfo f(filePath);
			QString originalOpticPath = QStringLiteral("%1recipe/%2/optics.json").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

			if (QFile::exists(originalOpticPath))
			{
				qDebug() << "Backup old optic folder to backup optic folder";
				QString backupFolderPath = QStringLiteral("%1recipe/%2/BackupOpticJson").arg(Common::Directory::LocalPath).arg(Common::Directory::CurrentRecipe);

				// Create the backup folder if it doesn't exist
				QDir backupFolder(backupFolderPath);
				if (!backupFolder.exists())
				{
					if (!backupFolder.mkpath("."))
					{
						qDebug() << "Error creating backup folder.";
						return;
					}
				}

				// Get the file name without the path
				QFileInfo fileInfo(originalOpticPath);
				QDir directory(backupFolderPath);
				directory.setFilter(QDir::Files | QDir::NoDotAndDotDot);
				QString fileName = fileInfo.fileName() + "_" + QString::number(directory.entryList().size());

				// Construct the destination path in the backup folder
				QString backupFilePath = QStringLiteral("%1/%2").arg(backupFolderPath).arg(fileName);

				// Try to move the file to the backup folder
				if (QFile::rename(originalOpticPath, backupFilePath))
				{
					qDebug() << "File moved to backup folder.";
				}
				else
				{
					qDebug() << "Error moving file to backup folder.";
				
				}

			}
			if (QFile::copy(filePath, originalOpticPath))
			{
				loadRecipeOptics();
			}
		}
	}
	else
	{
		return;
	}

}

void VisionApp::deleteLightingTemplate()
{
	QString lightingTemplateName = ui.comboBox_ligtingTemplateList->currentText();
	QString filePath = Common::Directory::getLightingTemplatePath() + "/" + lightingTemplateName + ".json";

	if (!QFileInfo::exists(filePath))
	{
		QMessageBox::warning(this, ("Error in deleting lighting template"),
			"Lighting Template does not exist!");
		return;
	}

	if (QMessageBox::Yes == QMessageBox(QMessageBox::Information,
		"Confirmation", "Do you want to remove " + lightingTemplateName + " lighting template?"
		, QMessageBox::Yes | QMessageBox::No).exec())
	{
		AuditLog::instance().log(QStringLiteral("LIGHTING_TEMPLATE_DELETE"), lightingTemplateName);
		QFile::remove(filePath);
		refreshLightingTemplateComboBox();
	}
	else
	{
		return;
	}
}

void VisionApp::updateLightingTemplate()
{
	QString lightingTemplateName = ui.comboBox_ligtingTemplateList->currentText();
	QString filePath = Common::Directory::getLightingTemplatePath() + "/" + lightingTemplateName + ".json";

	if (!QFileInfo::exists(filePath))
	{
		QMessageBox::warning(this, ("Error in updating lighting template"),
			"Lighting Template does not exist!");
		return;
	}

	if (QMessageBox::Yes == QMessageBox(QMessageBox::Information,
		"Confirmation", "Do you want to update " + lightingTemplateName + " lighting template?"
		, QMessageBox::Yes | QMessageBox::No).exec())
	{
		AuditLog::instance().log(QStringLiteral("LIGHTING_TEMPLATE_UPDATE"), lightingTemplateName);
		QFile::remove(filePath);
		saveLightingTemplateJson(filePath);
		refreshLightingTemplateComboBox();
	}
	else
	{
		return;
	}
}

void VisionApp::refreshLightingTemplateComboBox()
{
	// input algo template
	ui.comboBox_ligtingTemplateList->clear();
	QStringList nameFilters;
	nameFilters << "*.json";
	QDir lightingTemplateDirectory(Common::Directory::getLightingTemplatePath());
	QFileInfoList fileInfoList = lightingTemplateDirectory.entryInfoList(nameFilters, QDir::Files);
	for (const QFileInfo& fileInfo : fileInfoList) {
		ui.comboBox_ligtingTemplateList->addItem(fileInfo.baseName());
	}
}


void VisionApp::searchVo()
{

	QString searchVo = ui.lineEdit_searchVo->text();
	for (int i = 0; i < _dragROI.size(); i++)
	{
		if (_dragROI[i]->getName() == searchVo)
		{
			_dragROI[i]->setSelected(true);

			auto rect = _dragROI[i]->getGeometry();
			if (rect.width() != 0 && rect.height() != 0)
			{
				ui.graphicsViewMain->fitInView(_pPixmapItemMain, Qt::KeepAspectRatio);
				ui.graphicsViewMain->centerOn(_pPixmapItemMain);
				ui.graphicsViewMain->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
				ui.graphicsViewMain->show();

				qreal sx = 1;
				qreal sy = 1;
				qreal ratioX = rect.width() / _pGraphicsSceneMain->width();
				qreal ratioY = rect.height() / _pGraphicsSceneMain->height();

				if (ratioX < 0.40) {
					sx = _pGraphicsSceneMain->width() / ((rect.width() / 2) * 5);

					if (ratioY < 0.40) {
						sy = _pGraphicsSceneMain->height() / ((rect.height() / 2) * 5);
					}
					else if ((0.40 >= ratioY) && (ratioY < 1.0)) {
						sx = 1; sy = 1;
					}
				}
				else {
					sx = 1; sy = 1;
				}

				if (sx < sy) { sy = sx; }
				if (sy < sx) { sx = sy; }
				if (sx > 30) { sx = 30; }
				if (sy > 30) { sy = 30; }
				if (sx <= 0) { sx = 1; }
				if (sy <= 0) { sy = 1; }

				ui.graphicsViewMain->scale(sx, sy);
				QPointF centerPt(rect.center());
				ui.graphicsViewMain->centerOn(centerPt);
			}

			QList<QStandardItem *> itemFound = _recipeModel.findItems(searchVo, Qt::MatchExactly | Qt::MatchRecursive);
			if (itemFound.count() == 1)
			{
				ui.treeViewRecipeExplorer->setCurrentIndex(_recipeModel.indexFromItem(itemFound.at(0)));
			}

			break;

		}
	}
}

void VisionApp::updateSetupCheckList()
{

	ui.textEdit_setupCheckList->setFontPointSize(9);
	ui.textEdit_setupCheckList->setTextColor(QColor(200, 200, 200));
	ui.textEdit_setupCheckList->clear();

	// Check Top Left
	bool status;
	auto& fl = _plane.corner_points[(int)Corner::FRONTLEFT];
	if (fl.wx <= 0.0 && fl.wy <= 0.0 && fl.wz <= 0.0) status = false;
	else status = true;
	writeSetupCheckListTextEdit("Top Left Coordinate: ", status);

	// Check Bottom Right
	auto& br = _plane.corner_points[(int)Corner::BACKRIGHT];
	if (br.wx <= 0.0 && br.wy <= 0.0 && br.wz <= 0.0) status = false;
	else status = true;
	writeSetupCheckListTextEdit("Bottom Right Coordinate: ", status);

	// Check Fiducial 1
	QString fid1Path = Common::Directory::getRecipeImagesPath() + "Fiducial/feature1.pat";
	if (QFile::exists(fid1Path)) status = true;
	else status = false;
	writeSetupCheckListTextEdit("Fiducial 1: ", status);

	// Check Fiducial 2
	QString fid2Path = Common::Directory::getRecipeImagesPath() + "Fiducial/feature2.pat";
	if (QFile::exists(fid2Path)) status = true;
	else status = false;
	writeSetupCheckListTextEdit("Fiducial 2: ", status);

	// Check Barcode
	status = false;
	for (auto barcode : _barcodeInfos)
	{
		if (!barcode.id.isEmpty())
		{
			status = true;
			break;
		}
		else status = false;
	}
	writeSetupCheckListTextEdit("Barcode Setup: ", status);

	// check scaling 
	status = false;
	auto jsonPath = QStringLiteral("%1config/world.json").arg(Common::Directory::LocalPath);
	QJsonObject root;
	double horizontalScale;
	if (loadJson(jsonPath, root)) {
		horizontalScale = jsonHelper::getDouble(root, "horizontal_scale");
	}
	if (horizontalScale != ScaleManager::instance().horizontal_um_per_px()) status = true;
	writeSetupCheckListTextEdit("Scaling Setup: ", status);

	// check cam alignment
	if (QString::number(SystemData::instance()._camAngles[_camID]) == "0")
		status = false;
	else status = true;
	
	writeSetupCheckListTextEdit("Camera Alignment: ", status);

	// check plane image 
	QString planeImagePath = Common::Directory::getRecipeImagesPath() + "plane.jpg";
	if (QFile::exists(planeImagePath)) status = true;
	else status = false;
	writeSetupCheckListTextEdit("Plane Image: ", status);

	// check vision apps ROI
	status = false;
	if (_visionObject.size() > 100) status = true;
	writeSetupCheckListTextEdit("Unit Roi: ", status, QString::number(_visionObject.size()));

	// Check Unit Naming
	status = true;
	for (auto vo : _visionObject)
	{
		if (vo.objectName.contains("object")) status = false;
	}
	if (_visionObject.isEmpty()) status = false;
	writeSetupCheckListTextEdit("Unit Naming: ", status);

	// Check Unit Naming
	status = true;
	for (auto vo : _visionObject)
	{
		if (vo.viewID == "") status = false;
	}
	if (_visionObject.isEmpty()) status = false;
	writeSetupCheckListTextEdit("Unit View: ", status);

	// check path setup
	status = true;
	for (int i = 0; i < ui.listWidget_paths->count(); i++) {
		auto id = ui.listWidget_paths->item(i)->whatsThis();
		if (id.isEmpty()) status = false;
	}
	if (ui.listWidget_paths->count() <= 0) status = false;

	writeSetupCheckListTextEdit("Path Setup: ", status);

	// Check vo Algo Template
	status = true;
	int noTemplateCount = 0;
	for (auto vo : _visionObject)
	{
		if (vo.templateID == "")
		{
			noTemplateCount++;
			status = false;
		}
	}
	if (_visionObject.isEmpty()) status = false;
	if (status) 	writeSetupCheckListTextEdit("Unit Template setup: ", status, "All Unit has template ID");
	else writeSetupCheckListTextEdit("Unit Template setup: ", status, QString::number(noTemplateCount) + " unit no Template ID");
}

void VisionApp::writeSetupCheckListTextEdit(QString header, bool info, QString extraInfo)
{

	QString status;
	if (extraInfo == "")
	{
		if (info)
		{
			status = "<font color='green'> True</font>";
		}
		else
		{
			status = "<font color='red'> False</font>";
		}
	}
	else
	{
		if (info)
		{
			status = "<font color='green'> " + extraInfo + "</font>";
		}
		else
		{
			status = "<font color='red'> " + extraInfo + "</font>";
		}
	}


	QString set = "<font color='white'>" + header + "</font>" + status;
	ui.textEdit_setupCheckList->append(set);

}

void VisionApp::iniFileRemover()
{
	qDebug() << "iniFileRemover";
	connect(ui.checkBox_autoDeleteProductionFile, &QCheckBox::stateChanged, this, [=](int state) {
		if (state == Qt::Checked) {
			_autoDeleteProductionFile = true;
		}
		else if (state == Qt::Unchecked) {
			_autoDeleteProductionFile = false;
		}
		_fileRemovingThread->setSetting(_autoDeleteProductionFile, _clearingPathList, _storageLimit);

		QJsonArray jA;
		for (auto a : _clearingPathList)
		{
			jA.append(a);
		}
		jsonHelper::setJsonValue(_systemObj, "Storage_Limit", _storageLimit);
		jsonHelper::setJsonValue(_systemObj, "Auto_Delete_Production_File", _autoDeleteProductionFile);
		jsonHelper::setArrayValue(_systemObj, "Clearing_Path_List", jA);
		updateSystemInfo(_systemObj);
	});

	connect(ui.toolButton_browseClearingPath, &QToolButton::clicked, this, [&]() {
		QFileDialog dialog;
		dialog.setFileMode(QFileDialog::Directory);
		dialog.setOption(QFileDialog::ShowDirsOnly);
		QString clearingPathDir = QFileDialog::getExistingDirectory(this, tr("Choose Directory"), Common::Directory::ProductionPath());
		if (!clearingPathDir.isEmpty())
		{
			_clearingPathList.append(clearingPathDir);
			refreshClearingPathListWidget();
			_fileRemovingThread->setSetting(_autoDeleteProductionFile, _clearingPathList, _storageLimit);

			QJsonArray jA;
			for (auto a : _clearingPathList)
			{
				jA.append(a);
			}
			jsonHelper::setJsonValue(_systemObj, "Storage_Limit", _storageLimit);
			jsonHelper::setJsonValue(_systemObj, "Auto_Delete_Production_File", _autoDeleteProductionFile);
			jsonHelper::setArrayValue(_systemObj, "Clearing_Path_List", jA);
			updateSystemInfo(_systemObj);

		}

	});

	connect(ui.toolButton_deleteClearingPath, &QToolButton::clicked, this, [&]() {
		QString selectedPath = ui.listWidget_clearingPathList->currentItem()->text();
		if (!selectedPath.isEmpty())
		{
			_clearingPathList.removeAt(_clearingPathList.indexOf(selectedPath));
			refreshClearingPathListWidget();
			_fileRemovingThread->setSetting(_autoDeleteProductionFile, _clearingPathList, _storageLimit);

			QJsonArray jA;
			for (auto a : _clearingPathList)
			{
				jA.append(a);
			}
			jsonHelper::setJsonValue(_systemObj, "Storage_Limit", _storageLimit);
			jsonHelper::setJsonValue(_systemObj, "Auto_Delete_Production_File", _autoDeleteProductionFile);
			jsonHelper::setArrayValue(_systemObj, "Clearing_Path_List", jA);
			updateSystemInfo(_systemObj);
		}
	});

	connect(ui.spinBox_storageLimit, QOverload<int>::of(&QSpinBox::valueChanged), this, [&](int value) {	
		_storageLimit = value;
		_fileRemovingThread->setSetting(_autoDeleteProductionFile, _clearingPathList, _storageLimit);

		QJsonArray jA;
		for (auto a : _clearingPathList)
		{
			jA.append(a);
		}
		jsonHelper::setJsonValue(_systemObj, "Storage_Limit", _storageLimit);
		jsonHelper::setJsonValue(_systemObj, "Auto_Delete_Production_File", _autoDeleteProductionFile);
		jsonHelper::setArrayValue(_systemObj, "Clearing_Path_List", jA);
		updateSystemInfo(_systemObj);
	});

	refreshClearingPathListWidget();
	ui.checkBox_autoDeleteProductionFile->setChecked(_autoDeleteProductionFile);
	ui.spinBox_storageLimit->setValue(_storageLimit);

	_fileRemovingThread = new FileRemovingThread();
	_fileRemovingThread->setSetting(_autoDeleteProductionFile, _clearingPathList, _storageLimit);
	

	connect(_fileRemovingThread, SIGNAL(updateDriveSpace()), this, SLOT(updateDriveSpace()));	
	connect(&_fileRemoverTimer, &QTimer::timeout, this, [this]() {
		_fileRemovingThread->start();
	});

	_fileRemoverTimer.start(2000);
}

void VisionApp::refreshClearingPathListWidget()
{
	ui.listWidget_clearingPathList->clear();
	ui.listWidget_clearingPathList->addItems(_clearingPathList);
}

void VisionApp::updateDriveSpace()
{
	ui.label_storageUsage->setText(QString::number(_fileRemovingThread->_percentageFilled));
	ui.label_spaceLeft->setText(QString::number(_fileRemovingThread->_availableSpace));
}

// --- RMS ---

bool VisionApp::pullFromRmsRecipe(QString recipeName)
{
	qDebug() << "pullFromRmsRecipe: "<< _enableRmsRecipe;
	if (!_enableRmsRecipe) return false;
	
	QString sourcePath;
	QString destinationPath;

	QString rmsDir = jsonHelper::getString(_systemObj, QStringLiteral("Machine_Share_Folder_Path"))  + "/RMS Recipe/"+ recipeName;
	QString recipeDir = Common::Directory::RecipePath() + "/" + recipeName;

	sourcePath = rmsDir;
	destinationPath = recipeDir;

	if (!QFileInfo::exists(sourcePath))
	{
		return false;
	}

	qDebug() << "sourcePath: " << sourcePath;
	qDebug() << "destinationPath: " << destinationPath;

	rmsRecipeUpdate(sourcePath, destinationPath);
	return true;
}

bool VisionApp::pushToRmsRecipe(QString recipeName)
{
	qDebug() << "pushToRmsRecipe: " << _enableRmsRecipe;
	if (!_enableRmsRecipe) return false;

	QString sourcePath;
	QString destinationPath;

	QString rmsDir = jsonHelper::getString(_systemObj, QStringLiteral("Machine_Share_Folder_Path")) + "RMS Recipe/" + recipeName;
	QString recipeDir = Common::Directory::RecipePath() + "/" + recipeName;
	
	sourcePath = recipeDir;
	destinationPath = rmsDir;

	if (!QFileInfo::exists(sourcePath))
	{
		return false;
	}

	qDebug() << "sourcePath: " << sourcePath;
	qDebug() << "destinationPath: " << destinationPath;

	rmsRecipeUpdate(sourcePath, destinationPath);
	return true;
}

void VisionApp::rmsRecipeUpdate(const QString& sourceFolder, const QString& destFolder) {

	qDebug() << "rmsRecipeUpdate";
	
	QDir sourceDir(sourceFolder);
	QDir destDir(destFolder);

	// Create the destination directory if it doesn't exist
	if (!destDir.exists())
		destDir.mkpath(".");

	// Copy files from source to destination
	QStringList files = sourceDir.entryList(QDir::Files);
	
	for (const QString& file : files) {
		if (file != "SetupImages"&& file != "PlaneImages" && file!= "SampleImages" && file != "RefImages" && file != "ProcessedImages") {
			QString sourceFilePath = sourceDir.filePath(file);
			QString destFilePath = destDir.filePath(file);		

			if (QFileInfo::exists(destFilePath))
			{
				bool removeSuc = QFile::remove(destFilePath);
			
			}
			bool copyStatus = QFile::copy(sourceFilePath, destFilePath);
			
		}
	}

	// Recursively copy subdirectories
	QStringList subDirs = sourceDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
	for (const QString& subDir : subDirs) {
		if (subDir != "SetupImages" && subDir != "PlaneImages"&& subDir != "SampleImages" && subDir != "RefImages" && subDir != "ProcessedImages") {
			QString sourceSubDir = sourceFolder + QDir::separator() + subDir;
			QString destSubDir = destFolder + QDir::separator() + subDir;
			rmsRecipeUpdate(sourceSubDir, destSubDir);
		}
	}

}

// --- RMS ---


void VisionApp::udpateRecipeVersion(const QString& folderPath)
{
	// 3 place to call this function
	// recipe save
	// recipe open
	// apps destructor 

	QDir dir(folderPath);
	if (!dir.exists()) {
		qDebug() << "Folder does not exist.";
		return;
	}

	QJsonObject versionObject;



	QFileInfo recipeFileInfo(folderPath);
	QString name = Common::Directory::CurrentRecipe;
	QDateTime lastModified = recipeFileInfo.lastModified();
	versionObject.insert(name, lastModified.toString(Qt::ISODate));

	qDebug() << "folderPath: " << folderPath;
	qDebug() << "name: " << name;

	//// Iterate over each file and folder in the directory
	//QFileInfoList fileList = dir.entryInfoList(QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);
	//for (const QFileInfo& fileInfo : fileList) {
	//	QString name = fileInfo.fileName();
	//	QDateTime lastModified = fileInfo.lastModified();

	//	if (fileInfo.isDir()) {
	//		// Recursively update version for subfolders
	//		udpateRecipeVersion(fileInfo.filePath());
	//	}

	//	versionObject.insert(name, lastModified.toString(Qt::ISODate));
	//}

	// Write JSON to version file
	QString versionFilePath = QDir::cleanPath(folderPath + QDir::separator() + "RecipeVersion.json");
	QFile versionFile(versionFilePath);
	if (versionFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QJsonDocument doc(versionObject);
		versionFile.write(doc.toJson());
		versionFile.close();
		qDebug() << "Version file updated: " << versionFilePath;
	}
	else {
		qDebug() << "Failed to open version file for writing: " << versionFilePath;
	}
}


bool VisionApp::csv_readCircuitIdMapping(QString &csvPath)
{
	qDebug() << "csv_readCircuitIdMapping";

	QFile file(csvPath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Failed to open circuitId emap file.";
		qDebug() << "circuitId Path: " << csvPath;
		return false;
	}
	else
	{
		qDebug() << "Reading circuitId: " << csvPath;
	}

	QTextStream in(&file);
	int readingLine = 0;


	QStringList numberList;

	while (!in.atEnd())
	{
		QString line = in.readLine();

		if (line.isEmpty()) continue;
		readingLine++;
		if (readingLine > 6)
		{
			//qDebug() << line;
			QStringList value = line.split(",", QString::SkipEmptyParts);
			if (!value.isEmpty())
			{
				numberList.append(value);
			}
		}
	}

	int tRow = _islandInfo.totalRow;
	int tCol = _islandInfo.totalCol;
	int tIsland = _islandInfo.totalIsland;

	qDebug() << "numberList size: " << numberList.size();
	qDebug() << "vo size: " << _visionObject.size();

	if (numberList.size() != _visionObject.size()) return false;

	int row = 0;
	int col = 0;
	int island = 0;
	for (int i = 0; i < numberList.size(); i++)
	{
		QString circuitID = numberList[i];

		if (col >= tCol)
		{
			island++;
			if (island >= tIsland)
			{
				island = 0;
				row++;
			}

			col = 0;
		}
		if (row >= tRow) break;
		QString key = QString::number(row) + "[@]" + QString::number(col) + "[@]" + QString::number(island);

		for (auto& vo : _visionObject)
		{
			if (vo.row_id == row && vo.col_id == col && vo.island_id == island)
			{
				vo.circuitID = circuitID;
				_visionObjectCircuitId.insert(circuitID, vo.objectID);
				
			}
		}

		col++;
	}
	file.close();
	return true;
}

bool VisionApp::csv_readMounterIdMapping(QString& csvPath)
{
	qDebug() << "csv_readMounterIdMapping";


	QFile file(csvPath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Failed to open mounterId file.";
		qDebug() << "mounterId Path: " << csvPath;
		return false;
	}
	else
	{
		qDebug() << "Reading mounterId: " << csvPath;
	}

	QTextStream in(&file);
	int readingLine = 0;

	QHash<QString, QHash<QString, QString>> circuit_cadMounterId; // First layer key: circuitId, second layer key: cadName, value: mounterId

	while (!in.atEnd())
	{
		QString line = in.readLine();

		if (line.isEmpty()) continue;
		readingLine++;
		if (readingLine == 1) // for checking valid csv
		{
			QStringList value = line.split(",", QString::SkipEmptyParts);
			if (!value.isEmpty())
			{
				if (value[0] != "Reference Discreption")
					return false;
			}
			else
			{
				return false;
			}
		}

		if (readingLine > 1)
		{
			//qDebug() << line;
			QStringList value= line.split(",", QString::SkipEmptyParts);

			if (value.size() > 3)
			{
				if (!value[0].isEmpty() && !value[1].isEmpty() && !value[2].isEmpty())
				{
					QString cadName = value[0];
					QString circuitId = value[1];
					QString mounterId = value[2];


					if (circuit_cadMounterId.contains(circuitId)) circuit_cadMounterId[circuitId].insert(cadName, mounterId);
					else
					{
						QHash<QString, QString> cadMounterId;
						cadMounterId.insert(cadName, mounterId);
						circuit_cadMounterId.insert(circuitId, cadMounterId);
					}
				}
			}	
		}
	}


	for (int i = 0; i < circuit_cadMounterId.size(); i++)
	{
		QString thisCircuitId = circuit_cadMounterId.keys()[i];
	

		QString relativeVoId;
		if (_visionObjectCircuitId.contains(thisCircuitId))
		{
			relativeVoId = _visionObjectCircuitId[thisCircuitId];
			if (_visionObject.contains(relativeVoId))
			{
				_visionObject[relativeVoId].cadMounterId = circuit_cadMounterId[thisCircuitId];
			}
			else
			{
				qDebug() << "Could not find vision object Id: " << relativeVoId;
			}
		}
		else
		{
			qDebug() << "Could not find circuit Id: " << thisCircuitId;
		}
	}
	return true;
}

void VisionApp::arrangeTagNameBasedOnPriority(QStringList& tagNames)
{
	QStringList reorderedTagNames;

	for (int i = 0; i < _defectPriorityList.size(); i++)
	{
		for (int j = 0; j < tagNames.size(); j++)
		{
			if (tagNames[j] == _defectPriorityList[i]) reorderedTagNames.append(tagNames[j]);
		}
	}

	tagNames.clear();
	if (reorderedTagNames.size() == 0) reorderedTagNames.append("Unknown");
	tagNames = reorderedTagNames;

}



void VisionApp::editGoldenRecipeCheckList(QString recipeName, bool runStatus, bool reset)
{
	QHash<QString, bool> goldenRecipeRunStatusHash;

	// read first 
	QJsonFile goldenRecipeFile;
	if (goldenRecipeFile.load(_goldenRecipeCheckListPath))
	{
		auto recipeList = goldenRecipeFile.getArray("goldenRecipeList");
		for (int i = 0; i < recipeList.size(); i++)
		{
			auto recipe = recipeList[i].toObject();
			QString tRecipeName = recipe["recipeName"].toString();
			bool tRunStatus = recipe["runStatus"].toBool();
			goldenRecipeRunStatusHash.insert(tRecipeName, tRunStatus);

		}
	}

	if (!goldenRecipeRunStatusHash.contains(recipeName))
	{
		goldenRecipeRunStatusHash.insert(recipeName, runStatus);
	}
	else
	{
		goldenRecipeRunStatusHash[recipeName] = runStatus;
	}

	if (reset)
	{
		for (auto& rStatus : goldenRecipeRunStatusHash)
		{
			rStatus = false;
		}
	}


	// re write 
	QJsonDocument jsonDoc;
	QJsonObject rootObj;

	QJsonArray recipeArr;

	for (int i = 0; i < goldenRecipeRunStatusHash.size(); i++)
	{
		QString recipeName = goldenRecipeRunStatusHash.keys()[i];
		bool runStatus =goldenRecipeRunStatusHash[recipeName];

		QJsonObject recipeObj;
		recipeObj.insert(QStringLiteral("recipeName"), recipeName);
		recipeObj.insert(QStringLiteral("runStatus"), runStatus);

		recipeArr.append(recipeObj);
	}

	rootObj.insert(QStringLiteral("lastEditDateTime"), QDateTime::currentDateTime().toString());
	rootObj.insert(QStringLiteral("goldenRecipeList"), recipeArr);
	jsonDoc.setObject(rootObj);

	QFile file(_goldenRecipeCheckListPath);
	if (file.open(QIODevice::WriteOnly))
	{
		file.write(jsonDoc.toJson());
		file.flush();
		file.close();

		qDebug() << "Write golden checkList suc";
	}
	else
	{
		qDebug() << "Write golden checkList failed";
	}
}

void VisionApp::setupGoldenRecipeTimer()
{
	qDebug() << "setupGoldenRecipeTimer";

	// Get the current date and time
	QDateTime currentTime = QDateTime::currentDateTime();

	// Get the next 12 PM time
	QDateTime nextTriggerTime = currentTime.addDays(0);
	nextTriggerTime.setTime(QTime(12,0));

	// Calculate the milliseconds until the next 12 PM
	qint64 millisecondsToNextTrigger = currentTime.msecsTo(nextTriggerTime);

	qDebug() << "secondsToNextTrigger: " << QString::number(millisecondsToNextTrigger / 1000);

	// Set a single-shot timer to trigger the function at the calculated time
	QTimer::singleShot(millisecondsToNextTrigger, this, &VisionApp::triggerOnNewDay);

}


void VisionApp::triggerOnNewDay()
{
	qDebug() << "Triggered on new day at 12 PM";

	// reset the golden recipe check list
	editGoldenRecipeCheckList(Common::Directory::CurrentRecipe, false, true);


	// After the initial trigger, set the timer to trigger every 24 hours
	QTimer::singleShot(24 * 60 * 60 * 1000, this, &VisionApp::triggerOnNewDay);
}

bool VisionApp::checkGoldenRecipeRunStatus(QString recipeName)
{
	QHash<QString, bool> goldenRecipeRunStatusHash;

	// read first 
	QJsonFile goldenRecipeFile;
	if (goldenRecipeFile.load(_goldenRecipeCheckListPath))
	{
		auto recipeList = goldenRecipeFile.getArray("goldenRecipeList");
		for (int i = 0; i < recipeList.size(); i++)
		{
			auto recipe = recipeList[i].toObject();
			QString tRecipeName = recipe["recipeName"].toString();
			bool tRunStatus = recipe["runStatus"].toBool();
			goldenRecipeRunStatusHash.insert(tRecipeName, tRunStatus);

		}
	}

	bool runStatus = false;
	if (goldenRecipeRunStatusHash.contains(recipeName))
	{
		runStatus = goldenRecipeRunStatusHash[recipeName];
	}

	return runStatus;

}

void VisionApp::runGoldenRecipe()
{

	ui.label_status->show();


	QString text = "RUNNING GOLDEN RECIPE! Please do not interrupt this process";
	ui.label_status->setStyleSheet("QLabel { color : yellow; font-size: 24px; }");
	ui.label_status->setText(text);

	_grDialog->runGoldenRecipe(true);

}

void VisionApp::showPreviousImage()
{
	qDebug() << "showPreviousImage";
	bool imageLoaded = false;
	for (const auto& v : _views)
	{
		if (v.id == ui.label_curViewName->whatsThis())
		{
			auto unitConfigInfos = _unitConfigTab->getUnifConfigInfos();

			QString imageIndex;
			if (ui.lineEdit_currentImageIndex->text().isEmpty())
			{
				imageIndex = _unitConfigTab->getFirstID(ui.label_curViewName->whatsThis());
				ui.lineEdit_currentImageIndex->setText(imageIndex);
			}
			else
			{
				imageIndex = _unitConfigTab->getPreviousID(ui.lineEdit_currentImageIndex->text(), ui.label_curViewName->whatsThis());
				ui.lineEdit_currentImageIndex->setText(imageIndex);
			}

			for (const auto& o : v.opticIDs)
			{
				if (o == ui.comboBox_ImageOptics->currentData().toString())
				{
					QString imagePath = Common::Directory::CurrentImageSetPath + "\\" + v.id + "_" + o + "_" + imageIndex + ".jpg";
					qDebug() << "imagePath:" << imagePath;
					if (_imageWorld.load(imagePath))
					{
						ui.toolButton_toggleWorldView->animateClick();
						displayImage(_imageWorld);
						imageLoaded = true;
						break;
					}
				}

			}

		}
	}

	qDebug() << "imageLoaded:" << imageLoaded;
	if (!imageLoaded)
	{
		auto w = CAMManager::instance().getWidth(_camID);
		auto h = CAMManager::instance().getWidth(_camID);
		_imageWorld = QImage(w, h, QImage::Format_RGB32);
		_imageWorld.fill(Qt::black);
		ui.toolButton_toggleWorldView->animateClick();
		displayImage(_imageWorld);
	}

	showDefectRect(true);
}

void VisionApp::showNextImage()
{
	qDebug() << "showNextImage";
	bool imageLoaded = false;
	for (const auto& v : _views)
	{
		if (v.id == ui.label_curViewName->whatsThis())
		{
			auto unitConfigInfos = _unitConfigTab->getUnifConfigInfos();

			QString imageIndex;
			if (ui.lineEdit_currentImageIndex->text().isEmpty())
			{
				imageIndex = _unitConfigTab->getFirstID(ui.label_curViewName->whatsThis());
				ui.lineEdit_currentImageIndex->setText(imageIndex);
			}
			else
			{
				imageIndex = _unitConfigTab->getNextID(ui.lineEdit_currentImageIndex->text(), ui.label_curViewName->whatsThis());
				ui.lineEdit_currentImageIndex->setText(imageIndex);
			}

			for (const auto& o : v.opticIDs)
			{
				if (o == ui.comboBox_ImageOptics->currentData().toString())
				{
					QString imagePath = Common::Directory::CurrentImageSetPath + "\\" + v.id + "_" + o + "_" + imageIndex + ".jpg";
					qDebug() << "imagePath:" << imagePath;
					if (_imageWorld.load(imagePath))
					{
						ui.toolButton_toggleWorldView->animateClick();
						displayImage(_imageWorld);
						imageLoaded = true;
						break;
					}
				}

			}

		}
	}

	qDebug() << "imageLoaded:" << imageLoaded;
	if (!imageLoaded)
	{
		auto w = CAMManager::instance().getWidth(_camID);
		auto h = CAMManager::instance().getWidth(_camID);
		_imageWorld = QImage(w, h, QImage::Format_RGB32);
		_imageWorld.fill(Qt::black);
		ui.toolButton_toggleWorldView->animateClick();
		displayImage(_imageWorld);
	}

	showDefectRect(true);
}





QRect VisionApp::getQRectBasedOnCam(int percentage)
{
	double originOffset = (double)percentage / 100.0;
	double sizePercentage = 1.0 - originOffset;
	auto w = CAMManager::instance().getWidth(_camID);
	auto h = CAMManager::instance().getHeight(_camID);

	return QRect((double)w * originOffset, (double)h * originOffset, (double)w * sizePercentage, (double)h * sizePercentage);
}

QRectF VisionApp::getQRectFBasedOnCam(int percentage)
{
	double originOffset = (double) percentage / 100.0;
	double sizePercentage = 1.0 - originOffset;

	auto w = CAMManager::instance().getWidth(_camID);
	auto h = CAMManager::instance().getHeight(_camID);
	return QRectF((double)w * originOffset, (double)h * originOffset, (double)w * sizePercentage, (double)h * sizePercentage);
}

// new golden recipe 

void VisionApp::openGoldenRecipeDialog()
{
	qDebug() << "openGoldenRecipeDialog";

	_grDialog->setOptics(_recipeOptics, _recipeOptics3D);
	_grDialog->initRefGoldenRecipe();
	_grDialog->setModal(true);
	_grDialog->exec();
	
	

	//_grDialog->show();
}

void VisionApp::goldenRecipeRunComplete()
{
	_grDialog->goldenRecipeRunComplete();
}

void VisionApp::slotRunGoldenRecipeComplete(bool gr_isPass, QString gr_message)
{
	if (gr_isPass)
	{
		editGoldenRecipeCheckList(Common::Directory::CurrentRecipe, true, false);
		sendToClient(QStringLiteral("R\r"));

		QString text = "GOLDEN RECIPE --- PASS!";
		ui.label_status->setStyleSheet("QLabel { color : green; font-size: 24px; }");
		ui.label_status->setText(text);
	}
	else
	{
		sendToClient(QStringLiteral("01VISIONGOLDENRUNFAIL\r"));

		QString text = "GOLDEN RECIPE --- FAILED! "+ gr_message;
		ui.label_status->setStyleSheet("QLabel { color : red; font-size: 24px; }");
		ui.label_status->setText(text);
	}
	
}

void VisionApp::productionRunGoldenRecipeComplete()
{

}

//DatasetPage

void VisionApp::refreshDatasetView()
{
	_datasetPage->updateDatasetView(_unitConfigTab->getUnifConfigInfos(), _unitConfigTab, _views, _recipeOptics);
}

void VisionApp::displayCurrentView(QString viewID, QString opticID, QString indexID)
{
	showSetupPage();
	ui.lineEdit_currentImageIndex->setText(indexID);
	displayCurrentView(viewID, opticID);
}

void VisionApp::runStoredUnitsInspection(QStringList storedIndexIDs)
{
	_datasetIndexIds = storedIndexIDs;
	resetLoopFlags();
	_inspQueue.push(Common::Directory::CurrentImageSetPath);
	runQueuedInsp(); 
}


void VisionApp::clearCacheFolder()
{
	QString path = Common::Directory::CachePath;
	qDebug() << "Clearing cache folder: " << path;

	QDir dir(path);

	if (!dir.exists()) {
		qDebug() << "Directory does not exist: " << path;
		return;
	}

	// Get all files in the directory and remove them
	QStringList files = dir.entryList(QDir::Files);
	for (const QString& file : files) {
		if (!dir.remove(file)) {
			qDebug() << "Failed to remove file: " << file;
		}
	}

	// Get all subdirectories and recursively remove them
	QStringList subDirs = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
	for (const QString& subDir : subDirs) {
		QDir subDirPath(dir.filePath(subDir));
		if (!subDirPath.removeRecursively()) {
			qDebug() << "Failed to remove directory: " << subDirPath.absolutePath();
		}
	}

	qDebug() << "Cache folder cleared successfully.";
}

bool VisionApp::deleteAllFilesInFolder(const QString& folderPath)
{
	QDir dir(folderPath);
	if (!dir.exists())
		return false;

	// Set filter to only get files (not directories)
	dir.setFilter(QDir::Files);

	QFileInfoList fileList = dir.entryInfoList();
	bool success = true;

	for (const QFileInfo& fileInfo : fileList) {
		if (!dir.remove(fileInfo.fileName())) {
			success = false; // If any delete fails, mark it
		}
	}

	return success;
}

void VisionApp::lockAllROIs()
{
	for (auto roi : _dragROI)
		roi->setLocked(true);
		roiLocked = true;
}

void VisionApp::unlockAllROIs()
{
	for (auto roi : _dragROI)
		roi->setLocked(false);
		roiLocked = false;
}

void VisionApp::reloadStyleSheet()
{
	QFile file(":/VisionApp/Resources/styleSheet.qss");
	if (file.open(QFile::ReadOnly)) {
		QString styleSheet = QLatin1String(file.readAll());
		qApp->setStyleSheet(styleSheet);
		qApp->processEvents();  // ensure the style takes effect immediately
	}
	else {
		qDebug() << "Failed to reload stylesheet";
	}

	for (QWidget* w : QApplication::allWidgets())
		w->update();  // force repaint

	qApp->processEvents();

	qDebug() << "done reload styleSheet";
}

void VisionApp::runStoredUnits()
{
	qDebug() << "Not available";
}
void VisionApp::storeSkippedUnits()
{
	int totalSelected = 0;
	for (auto* pDragBox : _dragROI) {
		if (!pDragBox->isSelected())
			continue;
		++totalSelected;

		auto id = pDragBox->getId();
		auto& info = _visionObject[id];
		/*qDebug() << "----";
		qDebug() << "ObjectName:   " << info.objectName;
		qDebug() << "Skip (before):" << info.skip;*/
		info.forcedSkip = true;
		//qDebug() << "Skip (after): " << info.skip;
	}
	//qDebug() << "Total units selected:" << totalSelected;
	applySkipColors();
	saveVisionObject();
}

void VisionApp::removeSkippedUnits()
{
	int totalSelected = 0;
	for (auto* pDragBox : _dragROI) {
		if (!pDragBox->isSelected())
			continue;
		++totalSelected;

		auto id = pDragBox->getId();
		auto& info = _visionObject[id];
		/*qDebug() << "----";
		qDebug() << "ObjectName:   " << info.objectName;
		qDebug() << "Skip (before):" << info.skip;*/
		info.forcedSkip = false;
		//qDebug() << "Skip (after): " << info.skip;
	}
	//qDebug() << "Total units selected:" << totalSelected;
	applySkipColors();
	saveVisionObject();
}

void VisionApp::saveBorderColors()
{
	_savedBorderColors.clear();
	for (auto* roi : _dragROI) {
		_savedBorderColors[roi->getId()] = roi->getBorderColor();
	}
}

void VisionApp::applySkipColors()
{
	for (auto* roi : _dragROI) {
		const auto& info = _visionObject[roi->getId()];
		QColor color = info.forcedSkip ? Qt::red : Qt::green;
		roi->setBorderColor(color);
		roi->update();
	}
}

void VisionApp::restoreBorderColors()
{
	for (auto* roi : _dragROI) {
		auto restore = _savedBorderColors.find(roi->getId());
		if (restore != _savedBorderColors.end()) {
			roi->setBorderColor(restore.value());
			roi->update();
		}
	}
	_savedBorderColors.clear();
}

void VisionApp::runBareBoardAnalysis()
{
//// 1) Get the Results folder (contains measurement.json)
//	const QString productionResultPath = Common::Directory::getProductionResultPath();
//	if (productionResultPath.isEmpty() || !QDir(productionResultPath).exists()) {
//		ct::logger::error("Invalid production result path: %s", qPrintable(productionResultPath));
//		return;
//	}
//
//	// 2) Build path to measurement.json inside Results/
//	const QString selectedJsonFile = QDir(productionResultPath).filePath(QLatin1String("measurement.json"));
//	if (!QFileInfo::exists(selectedJsonFile)) {
//		ct::logger::error("Measurement file not found: %s", qPrintable(selectedJsonFile));
//		return;
//	}
//
//	// 3) Load & parse array
//	QFile inFile(selectedJsonFile);
//	if (!inFile.open(QIODevice::ReadOnly)) {
//		ct::logger::error("Failed to open input JSON: %s", qPrintable(selectedJsonFile));
//		return;
//	}
//	QJsonDocument loadDoc = QJsonDocument::fromJson(inFile.readAll());
//	inFile.close();
//
//	if (!loadDoc.isArray()) {
//		ct::logger::error("Expected top‐level array in JSON");
//		return;
//	}
//	QJsonArray measurements = loadDoc.array();
//
//	// 4) Bucket heights/volumes by roiName
//	QMap<QString, QList<double>> heightBuckets;
//	QMap<QString, QList<double>> volumeBuckets;
//	for (const QJsonValue& val : measurements) {
//		if (!val.isObject()) continue;
//		const QJsonObject obj = val.toObject();
//		const QString roiName = obj.value("roiName").toString();
//		const double  heightUm = obj.value("height(um)").toDouble();
//		const double  volumeUm = obj.value("volume(um)").toDouble();
//		heightBuckets[roiName].append(heightUm);
//		volumeBuckets[roiName].append(volumeUm);
//	}
//
//	// 5) Compute averages
//	QJsonArray outArray;
//	for (auto it = heightBuckets.constBegin(); it != heightBuckets.constEnd(); ++it) {
//		const QString& roi = it.key();
//		const QList<double>& hlist = it.value();
//		const QList<double>& vlist = volumeBuckets.value(roi);
//
//		const double sumH = std::accumulate(hlist.begin(), hlist.end(), 0.0);
//		const double avgH = hlist.isEmpty() ? 0.0 : sumH / hlist.size();
//
//		const double sumV = std::accumulate(vlist.begin(), vlist.end(), 0.0);
//		const double avgV = vlist.isEmpty() ? 0.0 : sumV / vlist.size();
//
//		QJsonObject o;
//		o["roiName"] = roi;
//		o["averageHeight(um)"] = avgH;
//		o["averageVolume(um)"] = avgV;
//		outArray.append(o);
//	}
//
//	// 6) Read barcode suffix from ../Images/info.json (parent of Results)
//	// productionResultPath -> .../<Base>/Results
//	QDir resultsDir(productionResultPath);
//	if (!resultsDir.cdUp()) {
//		ct::logger::error("Cannot ascend from Results to base folder: %s", qPrintable(productionResultPath));
//		return;
//	}
//	const QString baseFolder = resultsDir.absolutePath();
//	const QString infoPath = QDir(baseFolder).filePath(QLatin1String("Images/info.json"));
//
//	QString barcodeSuffix = QLatin1String("NoBarcode");
//	QFile infoFile(infoPath);
//	if (infoFile.open(QIODevice::ReadOnly)) {
//		const QJsonDocument infoDoc = QJsonDocument::fromJson(infoFile.readAll());
//		infoFile.close();
//		if (infoDoc.isObject()) {
//			const QString barcode = infoDoc.object().value("barcode_id").toString();
//			QRegularExpression re(QLatin1String("\\D+(\\d{5})"));
//			const auto match = re.match(barcode);
//			if (match.hasMatch()) {
//				barcodeSuffix = match.captured(1);
//			} else {
//				ct::logger::error("Cannot extract 5-digit code from barcode_id: %s", qPrintable(barcode));
//			}
//		}
//	} else {
//		ct::logger::error("Cannot open info.json at %s", qPrintable(infoPath));
//	}
//
//	// 7) Build save path under current recipe path (folder name with "_BBA" removed)
//	const QString currentRecipePath = Common::Directory::getRecipeCurrentPath();
//	QDir recipeDir(currentRecipePath);
//
//	// Work out parent folder and recipe folder name
//	QDir parentDir = recipeDir;
//	parentDir.cdUp();  // parent of the current recipe folder
//
//	QString recipeFolderName = recipeDir.dirName();        // e.g. "MyRecipe_BBA"
//	const QString suffix = QLatin1String("_BBA");
//	if (recipeFolderName.endsWith(suffix)) {
//		recipeFolderName.chop(suffix.size());              // -> "MyRecipe"
//	}
//
//	// Reconstruct the cleaned recipe path (…/MyRecipe)
//	const QString cleanedRecipePath = parentDir.filePath(recipeFolderName);
//
//	// Save directory: …/MyRecipe/BareBoardAnalysisData
//	const QString saveDir = QDir(cleanedRecipePath).filePath(QLatin1String("BareBoardAnalysisData"));
//	QDir().mkpath(saveDir);
//
//	// File name: BareBoardAnalysis_<barcode>.json
//	const QString saveJsonFile = QDir(saveDir).filePath(
//		QLatin1String("BareBoardAnalysis_") + barcodeSuffix + QLatin1String(".json")
//	);
//
//	// 8) Write out the averaged results
//	QFile outFile(saveJsonFile);
//	if (!outFile.open(QIODevice::WriteOnly)) {
//		ct::logger::error("Failed to open output file: %s", qPrintable(saveJsonFile));
//		return;
//	}
//	const QJsonDocument saveDoc(outArray);
//	outFile.write(saveDoc.toJson(QJsonDocument::Indented));
//	outFile.close();

	// 9) Done
	//QMessageBox::information(this,
	//	tr("BareBoardAnalysis Complete"),
	//	tr("Computed %1 pads and saved to:\n%2")
	//	.arg(outArray.size())
	//	.arg(saveJsonFile)
	//);

 // 1) Get the Results folder (contains measurement.json)
const QString productionResultPath = Common::Directory::getProductionResultPath();
if (productionResultPath.isEmpty() || !QDir(productionResultPath).exists()) {
	ct::logger::error("Invalid production result path: %s", qPrintable(productionResultPath));
	return;
}

// 2) Build path to measurement.json inside Results/
const QString selectedJsonFile = QDir(productionResultPath).filePath(QLatin1String("measurement.json"));
if (!QFileInfo::exists(selectedJsonFile)) {
	ct::logger::error("Measurement file not found: %s", qPrintable(selectedJsonFile));
	return;
}

// 3) Load & parse array
QFile inFile(selectedJsonFile);
if (!inFile.open(QIODevice::ReadOnly)) {
	ct::logger::error("Failed to open input JSON: %s", qPrintable(selectedJsonFile));
	return;
}
const QJsonDocument loadDoc = QJsonDocument::fromJson(inFile.readAll());
inFile.close();

if (!loadDoc.isArray()) {
	ct::logger::error("Expected top-level array in JSON");
	return;
}
const QJsonArray measurements = loadDoc.array();

// 4) Bucket heights/volumes by roiName (use only finite values)
QMap<QString, QList<double>> heightBuckets;
QMap<QString, QList<double>> volumeBuckets;
// (no reserve() for QMap)

for (const QJsonValue& val : measurements) {
	if (!val.isObject()) continue;
	const QJsonObject obj = val.toObject();
	const QString roiName = obj.value("roiName").toString();
	if (roiName.isEmpty()) continue;

	const auto hVal = obj.value("height(um)");
	if (hVal.isDouble()) {
		const double h = hVal.toDouble();
		if (std::isfinite(h)) heightBuckets[roiName].append(h);
	}

	const auto vVal = obj.value("volume(um)");
	if (vVal.isDouble()) {
		const double v = vVal.toDouble();
		if (std::isfinite(v)) volumeBuckets[roiName].append(v);
	}
}

// 5) Compute medians from ALL collected data (full sort for clarity) + keep averages (optional)
QJsonArray outArray;   // no reserve()

for (auto it = heightBuckets.constBegin(); it != heightBuckets.constEnd(); ++it) {
	const QString& roi = it.key();
	const QList<double>& hlist = it.value();
	const QList<double>& vlist = volumeBuckets.value(roi);

	std::vector<double> hvec; hvec.reserve(hlist.size());
	for (double x : hlist) if (std::isfinite(x)) hvec.push_back(x);

	std::vector<double> vvec; vvec.reserve(vlist.size());
	for (double x : vlist) if (std::isfinite(x)) vvec.push_back(x);

	double medH = 0.0, medV = 0.0;
	if (!hvec.empty()) {
		std::sort(hvec.begin(), hvec.end());
		const size_t n = hvec.size();
		medH = (n % 2) ? hvec[n / 2] : 0.5 * (hvec[n / 2 - 1] + hvec[n / 2]);
	}
	if (!vvec.empty()) {
		std::sort(vvec.begin(), vvec.end());
		const size_t n = vvec.size();
		medV = (n % 2) ? vvec[n / 2] : 0.5 * (vvec[n / 2 - 1] + vvec[n / 2]);
	}

	const double avgH = hlist.isEmpty() ? 0.0
		: std::accumulate(hlist.begin(), hlist.end(), 0.0) / double(hlist.size());
	const double avgV = vlist.isEmpty() ? 0.0
		: std::accumulate(vlist.begin(), vlist.end(), 0.0) / double(vlist.size());

	QJsonObject o;
	o["roiName"] = roi;
	o["countHeight"] = int(hvec.size());
	o["countVolume"] = int(vvec.size());
	if (!hvec.empty()) o["medianHeight(um)"] = medH;
	if (!vvec.empty()) o["medianVolume(um)"] = medV;
	o["averageHeight(um)"] = avgH;  // remove if you only want medians
	o["averageVolume(um)"] = avgV;  // remove if you only want medians

	outArray.append(o);
}

// 6) Read barcode suffix from ../Images/info.json (parent of Results)
QDir resultsDir(productionResultPath);
if (!resultsDir.cdUp()) {
	ct::logger::error("Cannot ascend from Results to base folder: %s", qPrintable(productionResultPath));
	return;
}
const QString baseFolder = resultsDir.absolutePath();
const QString infoPath = QDir(baseFolder).filePath(QLatin1String("Images/info.json"));

QString barcodeSuffix = QLatin1String("NoBarcode");
QFile infoFile(infoPath);
if (infoFile.open(QIODevice::ReadOnly)) {
	const QJsonDocument infoDoc = QJsonDocument::fromJson(infoFile.readAll());
	infoFile.close();
	if (infoDoc.isObject()) {
		const QString barcode = infoDoc.object().value("barcode_id").toString();
		QRegularExpression re(QLatin1String("\\D+(\\d{5})"));
		const auto match = re.match(barcode);
		if (match.hasMatch()) {
			barcodeSuffix = match.captured(1);
		}
		else {
			ct::logger::error("Cannot extract 5-digit code from barcode_id: %s", qPrintable(barcode));
		}
	}
}
else {
	ct::logger::error("Cannot open info.json at %s", qPrintable(infoPath));
}

// 7) Build save path under current recipe path (folder name with "_BBA" removed)
const QString currentRecipePath = Common::Directory::getRecipeCurrentPath();
if (currentRecipePath.isEmpty()) {
	ct::logger::error("Empty current recipe path.");
	return;
}

QDir recipeDir(currentRecipePath);
QDir parentDir = recipeDir;
if (!parentDir.cdUp()) {
	ct::logger::error("Cannot ascend from recipe path: %s", qPrintable(currentRecipePath));
	return;
}

QString recipeFolderName = recipeDir.dirName();      // e.g., "MyRecipe_BBA" or "MyRecipe"
const QString suffix = QLatin1String("_BBA");
if (recipeFolderName.endsWith(suffix))
recipeFolderName.chop(suffix.size());            // -> "MyRecipe"

const QString cleanedRecipePath = parentDir.filePath(recipeFolderName);
const QString saveDir = QDir(cleanedRecipePath).filePath(QLatin1String("BareBoardAnalysisData"));
QDir().mkpath(saveDir);

// 8) Write out the results file
const QString saveJsonFile = QDir(saveDir).filePath(
	QLatin1String("BareBoardAnalysis_") + barcodeSuffix + QLatin1String(".json")
);

QFile outFile(saveJsonFile);
if (!outFile.open(QIODevice::WriteOnly)) {
	ct::logger::error("Failed to open output file: %s", qPrintable(saveJsonFile));
	return;
}
const QJsonDocument saveDoc(outArray);
outFile.write(saveDoc.toJson(QJsonDocument::Indented));
outFile.close();


}

bool VisionApp::patchTemplateKeys(const QString& recipeDir, const QStringList& rulesList)
{
	const QString jsonPath = QDir(recipeDir).filePath("templateList.json");

	QFile f(jsonPath);
	if (!f.exists() || !f.open(QIODevice::ReadOnly)) {
		ct::logger::warn("[TemplatePatch] Cannot open: %s", qPrintable(jsonPath));
		return false;
	}
	QJsonParseError perr{};
	QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &perr);
	f.close();
	if (perr.error != QJsonParseError::NoError || doc.isNull()) {
		ct::logger::warn("[TemplatePatch] Invalid JSON: %s", qPrintable(jsonPath));
		return false;
	}

	// ---- everything below is local to this function ----

	// tiny “smart” value caster
	auto toJsonValue = [](QString s) -> QJsonValue {
		s = s.trimmed();
		if (s.compare("true", Qt::CaseInsensitive) == 0)  return QJsonValue(true);
		if (s.compare("false", Qt::CaseInsensitive) == 0) return QJsonValue(false);
		if (s.compare("null", Qt::CaseInsensitive) == 0)  return QJsonValue(); // null

		// try object/array if it looks like JSON
		if (!s.isEmpty() && (s.startsWith(QLatin1Char('{')) || s.startsWith(QLatin1Char('[')))) {
			QJsonParseError e{};
			QJsonDocument d = QJsonDocument::fromJson(s.toUtf8(), &e);
			if (e.error == QJsonParseError::NoError) {
				if (d.isObject()) return d.object();
				if (d.isArray())  return d.array();
			}
		}

		// number?
		static const QRegularExpression numRe(R"(^-?(?:\d+|\d*\.\d+)$)");
		if (numRe.match(s).hasMatch()) {
			bool ok = false; double v = s.toDouble(&ok);
			if (ok) return QJsonValue(v);
		}
		return QJsonValue(s); // fallback string
		};

	// wildcard key (“*”) → anchored regex; exact else. case-insensitive.
	auto makeKeyRe = [](const QString& key) -> QRegularExpression {
		const auto ci = QRegularExpression::CaseInsensitiveOption;
		bool hasWildcard = key.contains('*');
		QString pat = QRegularExpression::escape(key);
		if (hasWildcard) pat.replace("\\*", ".*");
		pat.prepend('^'); pat.append('$');
		return QRegularExpression(pat, ci);
		};

	struct Rule { QRegularExpression key; QJsonValue value; };
	QList<Rule> rules; rules.reserve(rulesList.size());

	// parse "key=value" or "key->value"
	for (const QString& raw : rulesList) {
		QString s = raw.trimmed();
		if (s.isEmpty()) continue;
		int pos = s.indexOf("->");
		if (pos < 0) pos = s.indexOf('=');
		if (pos <= 0) { ct::logger::warn("[TemplatePatch] Bad rule: %s", qPrintable(raw)); continue; }

		const QString keyToken = s.left(pos).trimmed();
		const QString valToken = s.mid(pos + ((s[pos] == '-') ? 2 : 1)).trimmed();
		if (keyToken.isEmpty()) { ct::logger::warn("[TemplatePatch] Bad rule: %s", qPrintable(raw)); continue; }

		QRegularExpression re = makeKeyRe(keyToken);
		if (!re.isValid()) { ct::logger::warn("[TemplatePatch] Invalid key regex: %s", qPrintable(keyToken)); continue; }
		rules.push_back({ re, toJsonValue(valToken) });
	}
	if (rules.isEmpty()) {
		ct::logger::warn("[TemplatePatch] No valid rules provided.");
		return false;
	}

	// recursive patcher
	std::function<QJsonValue(const QJsonValue&)> patch = [&](const QJsonValue& v) -> QJsonValue {
		if (v.isObject()) {
			QJsonObject out;
			const auto obj = v.toObject();
			for (auto it = obj.begin(); it != obj.end(); ++it) {
				const QString key = it.key();
				QJsonValue val = patch(it.value()); // recurse first

				// apply matching rules in order; last one wins
				for (const Rule& r : rules) {
					if (r.key.match(key).hasMatch()) val = r.value;
				}
				out.insert(key, val);
			}
			return out;
		}
		if (v.isArray()) {
			QJsonArray out = v.toArray();         // copy
			for (int i = 0; i < out.size(); ++i) {
				out[i] = patch(out.at(i));        // QJsonValueRef assignment
			}
			return out;
		}
		return v; // primitive
		};

	QJsonValue root = doc.isObject() ? QJsonValue(doc.object()) : QJsonValue(doc.array());
	QJsonValue patched = patch(root);

	QJsonDocument outDoc = patched.isObject() ? QJsonDocument(patched.toObject())
		: QJsonDocument(patched.toArray());
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
		ct::logger::warn("[TemplatePatch] Cannot write: %s", qPrintable(jsonPath));
		return false;
	}
	f.write(outDoc.toJson(QJsonDocument::Indented));
	f.close();

	ct::logger::info("[TemplatePatch] Patched %s with %d rule(s).", qPrintable(jsonPath), rules.size());
	return true;
}

bool VisionApp::copyAndPatchRecipe(const QString& sourceDir, const QString& destinationDir)
{
	// ---- sanitize
	const QString src = QDir::cleanPath(sourceDir);
	const QString dst = QDir::cleanPath(destinationDir);

	if (src.isEmpty() || dst.isEmpty()) {
		ct::logger::warn("[CopyPatch] Empty source or destination.");
		return false;
	}
	if (!QDir(src).exists()) {
		ct::logger::warn("[CopyPatch] Source does not exist: %s", qPrintable(src));
		return false;
	}
	if (src == dst) {
		ct::logger::warn("[CopyPatch] Source and destination are the same: %s", qPrintable(src));
		return false;
	}
	if (dst.startsWith(src + QDir::separator())) {
		ct::logger::warn("[CopyPatch] Destination is inside source (would recurse): %s", qPrintable(dst));
		return false;
	}
	if (!QDir(dst).exists() && !QDir().mkpath(dst)) {
		ct::logger::warn("[CopyPatch] Cannot create destination: %s", qPrintable(dst));
		return false;
	}

	// ---- YOUR hardcoded knobs
	const bool addBbaSuffix = true;                   // set false if you don’t want _BBA
	QStringList excludes{ "Images" };                // top-level subfolders to skip (inside source base)
	QStringList rules{
		"Registration_Method*=None",
		"Get_UpperHeight*=false",
		"Height_Range*=0, 300"  ,
		"Check_Volume*=false",
		"Check_Height*=false",
		/*"Measurement_Method*=Plane Fitting"*/
	};

	// ---- copy: src → dst/<base or base_BBA> (always overwrite)
	util::copyRecipe(src, dst, excludes, addBbaSuffix);

	// figure final destination root path (matches util::copyRecipe naming)
	const QString baseName = QFileInfo(src).fileName();
	QString targetName = baseName;
	if (addBbaSuffix && !baseName.endsWith("_BBA", Qt::CaseInsensitive))
		targetName += "_BBA";
	const QString dstRoot = QDir(dst).filePath(targetName);

	// ---- patch templateList.json under dstRoot
	if (!this->patchTemplateKeys(dstRoot, rules)) {
		ct::logger::warn("[CopyPatch] JSON patch failed in: %s", qPrintable(dstRoot));
		return false;
	}

	ct::logger::info("[CopyPatch] Done. From: %s  →  To: %s", qPrintable(src), qPrintable(dstRoot));
	return true;
}

void VisionApp::saveStitchingMethod()
{
	const int method = ui.comboBox_stitchingMethod->currentIndex()+1;
	SystemData::instance()._stitchingMethod = method;
	saveRecipeConfig();
	AuditLog::instance().log(QStringLiteral("STITCHING_METHOD"), QStringLiteral("method=%1").arg(method));
}

void VisionApp::initStitchingMethod()
{
	QSignalBlocker b1(ui.comboBox_stitchingMethod);
	ui.comboBox_stitchingMethod->setCurrentIndex(1);
}

//bool VisionApp::loadStitchingMethod()
//{
//	const QString recipeDir = QStringLiteral("%1recipe/%2").arg(Common::Directory::LocalPath, Common::Directory::CurrentRecipe);
//	const QString jsonPath = recipeDir + "/recipeConfig.json";
//
//	QFile file(jsonPath);
//	if (!file.open(QIODevice::ReadOnly)) {
//		qWarning() << "loadRecipeStitchingMethod: cannot open" << jsonPath << file.errorString();
//		return false;
//	}
//
//	QJsonParseError perr;
//	const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &perr);
//	file.close();
//
//	if (perr.error != QJsonParseError::NoError || !doc.isObject()) {
//		qWarning() << "loadRecipeStitchingMethod: bad JSON in" << jsonPath << perr.errorString();
//		return false;
//	}
//
//	const QJsonObject json = doc.object();
//
//	// Read value; empty means "missing"
//	const QString method = json.value(QStringLiteral("stitchingMethod")).toString();
//
//	ui.comboBox_stitchingMethod->blockSignals(true);
//	if (!method.isEmpty()) {
//		int idx = ui.comboBox_stitchingMethod->findText(method, Qt::MatchFixedString);
//		if (idx < 0) {
//
//			 idx = 0;
//		}
//		ui.comboBox_stitchingMethod->setCurrentIndex(idx);
//	}
//	else {
//		if (ui.comboBox_stitchingMethod->count() > 0)
//			ui.comboBox_stitchingMethod->setCurrentIndex(0);
//	}
//	ui.comboBox_stitchingMethod->blockSignals(false);
//
//	qDebug() << "Loaded stitchingMethod =" << ui.comboBox_stitchingMethod->currentText();
//	return true;
//
//}

bool VisionApp::cropVOFromViewImage(const QString& viewImagePath, const QRectF& voRectWorldPx, const QPointF& fovOffsetPx,
	MIL_ID& outParent, MIL_ID& outChild, QRect& outCropRect, QString& outRectString) const
{
	outParent = M_NULL;
	outChild = M_NULL;
	outCropRect = QRect();
	outRectString.clear();

	const int ix = int(voRectWorldPx.x() - fovOffsetPx.x());
	const int iy = int(voRectWorldPx.y() - fovOffsetPx.y());
	const int iw = int(voRectWorldPx.width());
	const int ih = int(voRectWorldPx.height());
	outCropRect = QRect(ix, iy, iw, ih);

	outRectString = QString::number(outCropRect.x()) + "_" + QString::number(outCropRect.y()) + "_" +
		QString::number(outCropRect.width()) + "_" + QString::number(outCropRect.height());

	MIL_INT sizeX = 0, sizeY = 0, bandSize = 0;
	const std::string p = viewImagePath.toStdString();

	MbufDiskInquireA(p.c_str(), M_SIZE_BAND, &bandSize);
	MbufDiskInquireA(p.c_str(), M_SIZE_X, &sizeX);
	MbufDiskInquireA(p.c_str(), M_SIZE_Y, &sizeY);

	if (bandSize == 1) MbufAlloc2d(M_DEFAULT_HOST, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &outParent);
	else              MbufAllocColor(M_DEFAULT_HOST, 3, sizeX, sizeY, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &outParent);

	MIL_INT imgType = M_JPEG_LOSSY;
	const QString low = viewImagePath.toLower();
	if (low.endsWith(".png")) imgType = M_PNG;
	if (low.endsWith(".bmp")) imgType = M_BMP;

	MbufImportA(p.c_str(), imgType, M_LOAD, M_DEFAULT_HOST, &outParent);

	outChild = MbufChild2d(outParent, outCropRect.x(), outCropRect.y(), outCropRect.width(), outCropRect.height(), M_NULL);

	return (outParent != M_NULL && outChild != M_NULL);
}

void VisionApp::enableFiducial(bool enable)
{
	QSignalBlocker sb(ui.checkBox_enableFiducial);
	QSignalBlocker sb1(ui.toolButton_enableFiducial);

	ui.checkBox_enableFiducial->setChecked(enable);
	nvs::set_background_color(ui.toolButton_enableFiducial, enable ? Qt::green : Qt::red);
	
	_useFiducial = enable;
	_jobThread.enableFiducial(_useFiducial);
	jsonHelper::setJsonValue(_systemObj, "Enable_Fiducial", _useFiducial);
	updateSystemInfo(_systemObj);
}

void VisionApp::enableSaveInspectionImage(bool enable)
{
	QSignalBlocker sb(ui.checkBox_EnableSaveInspectionImage);
	QSignalBlocker sb1(ui.toolButton_enableSaveInspImages);

	ui.checkBox_EnableSaveInspectionImage->setChecked(enable);
	nvs::set_background_color(ui.toolButton_enableSaveInspImages, enable ? Qt::green : Qt::red);

	_saveInspImg = enable;
	SystemData::instance()._saveInspImages = enable;
	jsonHelper::setJsonValue(_systemObj, "Save_Inspection_Image", _saveInspImg);
	updateSystemInfo(_systemObj);
}

//void VisionApp::clear2DOnlyOverlay()
//{
//	if (_no3DZoneItem) {
//		_no3DZoneItem->setPath(QPainterPath());
//		_no3DZoneItem->setVisible(false);
//	}
//}
//
//static QRect worldBoundsWpx()
//{
//	auto& sm = ScaleManager::instance();
//	auto env = sm.world_env();
//
//	// World bounds in mm (x: left->right, y: front->back)
//	QRectF worldMm(env.left, env.front,
//		env.right - env.left,
//		env.back - env.front);
//
//	// Top-left position: mm -> world px
//	QPointF tl = sm.to_world_px(QPointF(worldMm.left(), worldMm.top()));
//
//	// Size: mm -> (fov px) -> (world px)
//	double w = sm.fov_to_world(sm.mm_to_horizontal_px(worldMm.width()));
//	double h = sm.fov_to_world(sm.mm_to_vertical_px(worldMm.height()));
//
//	return QRect((int)std::lround(tl.x()),
//		(int)std::lround(tl.y()),
//		(int)std::lround(w),
//		(int)std::lround(h));
//}
//
//void VisionApp::update2DOnlyOverlay()
//{
//	// 0) Basic guards
//	if (!_show2DOnlyOverlay) {
//		clear2DOnlyOverlay();
//		return;
//	}
//
//	if (!_pGraphicsSceneMain) {
//		ct::logger::error("[No3DZone] _pGraphicsSceneMain is null");
//		return;
//	}
//
//	// Create overlay item once (built-in Qt item, no custom class)
//	if (!_no3DZoneItem) {
//		_no3DZoneItem = new QGraphicsPathItem();
//		_no3DZoneItem->setPen(Qt::NoPen);                          // shade only
//		_no3DZoneItem->setBrush(QBrush(QColor(255, 0, 0, 30)));    // translucent red
//		_no3DZoneItem->setAcceptedMouseButtons(Qt::NoButton);
//		_no3DZoneItem->setFlag(QGraphicsItem::ItemIsSelectable, false);
//		_no3DZoneItem->setZValue(9999);                            
//		_pGraphicsSceneMain->addItem(_no3DZoneItem);
//		ct::logger::info("[No3DZone] Created _no3DZoneItem");
//	}
//
//	// 1) 2D reachable = whole world (world px)
//	QRect world2D = worldBoundsWpx();
//	QRegion region2D(world2D);
//
//	ct::logger::info("[No3DZone] world2D(wpx) x=%d y=%d w=%d h=%d",
//		world2D.x(), world2D.y(), world2D.width(), world2D.height());
//
//	// 2) Get offset in mm from memory (already loaded by loadLaserConfig)
//	const double dx_mm = _laserConfig.offset.wx;
//	const double dy_mm = _laserConfig.offset.wy;
//
//	ct::logger::info("[No3DZone] offset mm: dx=%.3f dy=%.3f", dx_mm, dy_mm);
//
//	// 3) Convert offset mm -> world px
//	auto& sm = ScaleManager::instance();
//
//	const double dx_fov_px = sm.mm_to_horizontal_px(dx_mm);
//	const double dy_fov_px = sm.mm_to_vertical_px(dy_mm);
//
//	ct::logger::info("[No3DZone] offset fov_px: dx=%.3f dy=%.3f", dx_fov_px, dy_fov_px);
//	ct::logger::info("[No3DZone] worldScale=%.6f", sm.world_scale());
//
//	const double dx_wpx = sm.fov_to_world(dx_fov_px);
//	const double dy_wpx = sm.fov_to_world(dy_fov_px);
//
//	ct::logger::info("[No3DZone] offset world_px: dx=%.3f dy=%.3f", dx_wpx, dy_wpx);
//
//	// 4) 3D reachable expressed in 2D frame:
//	// Since 3D = 2D + offset, shift by (-offset)
//	QRect reach3D_in2D = world2D.translated(
//		(int)std::lround(-dx_wpx),
//		(int)std::lround(-dy_wpx)
//	);
//
//	ct::logger::info("[No3DZone] reach3D_in2D(wpx) x=%d y=%d w=%d h=%d",
//		reach3D_in2D.x(), reach3D_in2D.y(), reach3D_in2D.width(), reach3D_in2D.height());
//
//	QRegion region3D(reach3D_in2D);
//
//	// 5) Red region = 2D - 3D
//	QRegion red = region2D.subtracted(region3D);
//
//	ct::logger::info("[No3DZone] red rect count = %d", (int)red.rects().size());
//
//	if (red.isEmpty()) {
//		// Nothing to show
//		_no3DZoneItem->setPath(QPainterPath());
//		_no3DZoneItem->setVisible(false);
//		_pGraphicsSceneMain->update();
//		return;
//	}
//
//	// 6) Convert red region to path and set it
//	QPainterPath path;
//	for (const QRect& rr : red.rects()) {
//		path.addRect(QRectF(rr));
//	}
//
//	_no3DZoneItem->setPath(path);
//	_no3DZoneItem->setVisible(true);
//
//	_pGraphicsSceneMain->update();
//}

void VisionApp::vs_updateUptimer()
{
	if (!isStartTimer) return;
	if (!vs_elapsedTimer.isValid()) return;

	QTime t(0, 0);
	t = t.addMSecs(vs_elapsedTimer.elapsed());

	ui.lineEdit_timeElapsed->setText(t.toString("hh:mm:ss"));

}

void VisionApp::vs_startElapseTimer()
{
	vs_elapsedTimer.start();
	isStartTimer = true;

}

void VisionApp::vs_stopElapseTimer()
{
	vs_elapsedTimer.invalidate();
	isStartTimer = false;

}

bool VisionApp::blockJogSignal()
{
	if (ui.stackedWidgetViewSelection->currentIndex() == 6) return false;

	if (SystemData::instance()._MotoIsMoving) return false;
	SystemData::instance()._MotoIsMoving = true;
	return true;
}

void VisionApp::enableUIControl(const bool& flag)
{
	ui.toolButtonMenu->setEnabled(flag);
	ui.toolButtonWorkingMode->setEnabled(flag);
	ui.toolButton_ProductionMode->setEnabled(flag);
	ui.toolButtonRecipeSettings->setEnabled(flag);
	ui.toolButtonSystemSettings->setEnabled(flag);
	ui.toolButtonOpenImage->setEnabled(flag);
	ui.toolButton_userAccount->setEnabled(flag);

	ui.frame_helperButtons->setEnabled(flag);
	ui.frame_displayToggle->setEnabled(flag);
	ui.comboBox_ImageOptics->setEnabled(flag);

	ui.toolButton_toggleDualView->setEnabled(flag);
	ui.toolButton_toggleWorldView->setEnabled(flag);
	ui.toolButton_toggleFovView->setEnabled(flag);
	ui.toolButton_connectServer->setEnabled(flag);

	ui.toolButton_toggleMotionControl->setEnabled(flag);

	_isAutoMode = !flag;

}

bool VisionApp::notAllowToAccess(AccessLevel accessLevel)
{
	if (_curUserAccInfo.userName.isEmpty()) return true;
	return _curUserAccInfo.accessLevel == accessLevel;
}
