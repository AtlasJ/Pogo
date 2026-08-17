#include "ExtendedMenu.h"
#include "VisionAppStruct.h"

ExtendedMenu::ExtendedMenu(int index,int width, int height, QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	hide();
	ui.stackedWidget_menu->setCurrentIndex(index);
	setMinimumWidth(width);
	setMinimumHeight(height);
	setMaximumWidth(width);
	setMaximumHeight(height);

	connect(ui.toolButtonNewRecipe, &QToolButton::pressed, this, [=]() { emit recipeSettingsMenuBtnPressed(NEWRECIPE);});
	connect(ui.toolButtonLoadRecipe, &QToolButton::pressed, this, [=]() { emit recipeSettingsMenuBtnPressed(LOADRECIPE); });
	connect(ui.toolButtonSaveRecipe, &QToolButton::pressed, this, [=]() { emit recipeSettingsMenuBtnPressed(SAVERECIPE); });
	connect(ui.toolButtonDuplicateRecipe, &QToolButton::pressed, this, [=]() { emit recipeSettingsMenuBtnPressed(DUPLICATERECIPE); });
	connect(ui.toolButtonArchiveRecipe, &QToolButton::pressed, this, [=]() { emit recipeSettingsMenuBtnPressed(ARCHIVERECIPE); });
	connect(ui.toolButtonRestoreRecipe, &QToolButton::pressed, this, [=]() { emit recipeSettingsMenuBtnPressed(RESTORERECIPE); });
	connect(ui.toolButtonShowRecipeInExplorer, &QToolButton::pressed, this, [=]() { emit recipeSettingsMenuBtnPressed(SHOWRECIPEINEXPLORER); });

	connect(ui.toolButtonConfig, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(CONFIG); });
	connect(ui.toolButtonScaling, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(SCALING); });
	connect(ui.toolButtonLighting, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(LIGHTING); });
	connect(ui.toolButton3DOptics, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(OPTICS3D); });
	connect(ui.toolButtonAnalysis, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(ANALYSIS); });
	connect(ui.toolButtonTestRun, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(TESTRUN); });
	connect(ui.toolButtonLaser, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(LASER); });
	connect(ui.toolButtonPortability, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(PORTABILITY); });
	connect(ui.toolButtonAIModel, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(AIMODEL); });
	connect(ui.toolButtonZStack, &QToolButton::pressed, this, [=]() { emit systemSettingsMenuBtnPressed(ZSTACK); });
	
	//rightMenu
	connect(ui.toolButtonPropertyTab, &QToolButton::pressed, this, [=]() { emit showPropertyTab(); });
	connect(ui.toolButtonTemplateLibrary, &QToolButton::pressed, this, [=]() { emit showTemplateLibraryTab(); });
	connect(ui.toolButtonLogTab, &QToolButton::pressed, this, [=]() { emit showLogTab(); });
	connect(ui.toolButtonViewTab, &QToolButton::pressed, this, [=]() { emit showRecipeSetupTab(); });
	connect(ui.toolButtonVisionTab, &QToolButton::pressed, this, [=]() { emit showVisionObjectTab(); });
	connect(ui.toolButtonPathTab, &QToolButton::pressed, this, [=]() { emit showPathTab(); });
	connect(ui.toolButtonNamingConvention, &QToolButton::pressed, this, [=]() { emit showNamingConvention(); });
	connect(ui.toolButtonUnitConfig, &QToolButton::pressed, this, [=]() { emit showUnitConfigTab(); });

	// hide ui
	ui.toolButtonScaling->hide();
	/*ui.toolButtonAnalysis->hide();*/
	ui.toolButtonDuplicateRecipe->hide();
	ui.toolButtonArchiveRecipe->hide();
	ui.toolButtonRestoreRecipe->hide();
	ui.toolButtonShowRecipeInExplorer->hide();
	ui.toolButtonPathEditor->hide();
	ui.toolButtonUnitConfig->hide();

	//ui.toolButtonPortability->hide();
	//ui.toolButtonLaser->hide();
	//ui.toolButtonTestRun->hide();
	//ui.toolButtonLogTab->hide();
	//ui.toolButtonViewTab->hide();
	//ui.toolButtonVisionTab->hide();
	//ui.toolButtonPathTab->hide();
	//ui.toolButtonNamingConvention->hide();
}

ExtendedMenu::~ExtendedMenu()
{
}

void ExtendedMenu::setHeight(int height)
{
	setMinimumHeight(height);
	setMaximumHeight(height);
}
