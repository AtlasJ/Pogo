//
//  Guided Setup (new): the glue between VisionApp and the new page / layout view.
//
//  Built alongside the old Guided Setup, which it never touches: its own right-panel page (added
//  to stackedWidget in code, so VisionApp.ui is not edited), its own layout view in the workspace
//  stack, and its own recipe file guidedSetup2.json. Build step 1 = the page, Save / Discard and
//  the layout view; nothing here moves the machine.
//
#include "VisionApp.h"
#include "GuidedSetup2Page.h"
#include "GuidedSetup2View.h"
#include "MotionController.h"
#include "ScaleManager.h"
#include "CAMManager.h"
#include "Logger.h"

#include <QTimer>

void VisionApp::initGuidedSetup2()
{
	_gs2Page = new GuidedSetup2Page();

	//the right tab is chosen by index == UIPage value, so the page has to land exactly there
	const int index = (int)UIPage::GUIDED_SETUP2;
	while (ui.stackedWidget->count() < index) ui.stackedWidget->addWidget(new QWidget()); //PMF has no right page
	if (ui.stackedWidget->count() == index) {
		ui.stackedWidget->addWidget(_gs2Page);
	}
	else {
		ct::logger::error("[GuidedSetup2] stackedWidget already has %d pages; the page is inserted at %d and later pages shift",
			ui.stackedWidget->count(), index);
		ui.stackedWidget->insertWidget(index, _gs2Page);
	}

	_gs2View = new GuidedSetup2View();
	_gs2ViewIndex = ui.stackedWidgetViewSelection->addWidget(_gs2View);

	_gs2CameraTimer = new QTimer(this);
	_gs2CameraTimer->setInterval(200);
	connect(_gs2CameraTimer, &QTimer::timeout, this, [=]() { gs2RefreshCamera(); });

	connect(_gs2Page, &GuidedSetup2Page::setupChanged, this, [=]() { if (_gs2View && _gs2Page) _gs2View->setSetup(_gs2Page->setup()); });
	connect(_gs2Page, &GuidedSetup2Page::showLayoutToggled, this, [=](bool on) { gs2ShowLayout(on); });

	//the layout can be reached from the World button or the page's Show layout button, and left by
	//the FOV / Dual buttons or another page: whichever way, the page button and the camera refresh
	//follow what the big image area actually shows
	_gs2ViewConn = connect(ui.stackedWidgetViewSelection, &QStackedWidget::currentChanged, this, [=](int i) {
		if (!_gs2Page) return;
		const bool shown = i == _gs2ViewIndex;
		_gs2Page->setLayoutShown(shown);
		if (shown) {
			gs2RefreshWorld();
			gs2RefreshFov();
			_gs2CameraTimer->start();
		}
		else {
			_gs2CameraTimer->stop();
		}
	});
}

void VisionApp::gs2PageOpened()
{
	if (!_gs2Page) return;
	//a recipe opened by a route that skipped gs2RecipeOpened() still shows up here
	if (_gs2Page->recipeDir() != Common::Directory::getRecipeCurrentPath()) {
		_gs2Page->confirmLeave();
		gs2RecipeOpened();
	}
	_gs2Open = true;
	_gs2Page->setLayoutShown(ui.stackedWidgetViewSelection->currentIndex() == _gs2ViewIndex);
	_gs2Page->refreshScale();
	gs2RefreshWorld();
}

void VisionApp::gs2PageLeft()
{
	if (!_gs2Page || !_gs2Open) return;
	_gs2Open = false;
	//the layout stays up if it is showing: it belongs to the World button now, not only to this page
	_gs2Page->confirmLeave();
}

void VisionApp::gs2ShowLayout(bool shown)
{
	if (!_gs2View) return;
	if (shown) gs2ShowLayoutView();
	else toggleFOVView();
}

void VisionApp::gs2ShowLayoutView()
{
	if (!_gs2View) return;
	QSignalBlocker sb1(ui.toolButton_toggleDualView);
	QSignalBlocker sb2(ui.toolButton_toggleWorldView);
	QSignalBlocker sb3(ui.toolButton_toggleFovView);
	ui.toolButton_toggleDualView->setChecked(false);
	ui.toolButton_toggleWorldView->setChecked(true);
	ui.toolButton_toggleFovView->setChecked(false);
	gs2RefreshWorld();
	ui.stackedWidgetViewSelection->setCurrentIndex(_gs2ViewIndex);
}

void VisionApp::gs2RefreshWorld()
{
	if (!_gs2View) return;
	double xLo = 0.0, xHi = 0.0, yLo = 0.0, yHi = 0.0;
	const bool known = MotionController::instance().limits_mm(_motionID, (int)Axis::X, xLo, xHi)
		&& MotionController::instance().limits_mm(_motionID, (int)Axis::Y, yLo, yHi);
	_gs2View->setWorld(known, xLo, xHi, yLo, yHi);
}

void VisionApp::gs2RefreshCamera()
{
	if (!_gs2View || !_gs2View->isVisible()) return;
	double x = 0.0, y = 0.0, z = 0.0;
	getCurrentPoint(x, y, z);
	_gs2View->setCamera(true, x, y, _gs2FovWmm, _gs2FovHmm);
}

void VisionApp::gs2RefreshFov()
{
	const int wPx = CAMManager::instance().getWidth(QStringLiteral("cam1"));
	const int hPx = CAMManager::instance().getHeight(QStringLiteral("cam1"));
	_gs2FovWmm = wPx > 0 ? ScaleManager::instance().to_horizontal_mm(wPx) : 0.0;
	_gs2FovHmm = hPx > 0 ? ScaleManager::instance().to_vertical_mm(hPx) : 0.0;
}

void VisionApp::gs2Shutdown()
{
	//the stacked widgets delete their pages after ~VisionApp's body and may emit currentChanged on
	//the way; with the connection cut and the pointers cleared every gs2 function is a no-op
	QObject::disconnect(_gs2ViewConn);
	if (_gs2CameraTimer) _gs2CameraTimer->stop();
	_gs2Page = nullptr;
	_gs2View = nullptr;
}

void VisionApp::gs2BeforeRecipeChange()
{
	if (_gs2Page) _gs2Page->confirmLeave();
}

void VisionApp::gs2RecipeOpened()
{
	if (!_gs2Page) return;
	_gs2Page->loadRecipe(Common::Directory::getRecipeCurrentPath(), Common::Directory::CurrentRecipe);
}
