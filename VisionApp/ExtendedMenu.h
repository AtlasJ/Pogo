#pragma once

#include <QWidget>
#include "ui_ExtendedMenu.h"

class ExtendedMenu : public QWidget
{
	Q_OBJECT

public:
	ExtendedMenu(int index, int width, int height, QWidget *parent = Q_NULLPTR);
	~ExtendedMenu();

	void setHeight(int height);

private:
	Ui::ExtendedMenu ui;

signals:
	void recipeSettingsMenuBtnPressed(int);
	void systemSettingsMenuBtnPressed(int);
	void showLogTab();
	void showPropertyTab();                                     
	void showTemplateLibraryTab();
	void showRecipeSetupTab();
	void showVisionObjectTab();
	void showPathTab();
	void showNamingConvention();
	void showUnitConfigTab();
};
