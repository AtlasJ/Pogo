#pragma once
#include <QWidget>
#include "ui_GuidedSetup2Page.h"
#include "GuidedSetup2Types.h"

/*
* The right-panel page of the Guided Setup (new). It edits a working copy of one recipe's
* guidedSetup2.json; nothing reaches the file until Save, and Discard goes back to what was saved.
* Taught values (positions, sizes measured by the machine) are shown read-only here; the buttons
* that teach them arrive in later build steps. Typed settings can be edited at any time.
*/
class GuidedSetup2Page : public QWidget
{
	Q_OBJECT

public:
	explicit GuidedSetup2Page(QWidget* parent = nullptr);

	//read the recipe's file into the page; recipeDir ends with '/'. Unsaved edits are dropped, so
	//call confirmLeave() first.
	void loadRecipe(const QString& recipeDir, const QString& recipeName);
	const QString& recipeDir() const { return _recipeDir; }
	const gs2::Setup& setup() const { return _work; }
	bool isDirty() const;

	//unsaved edits: ask Save or Discard and do it. Leaving is never blocked.
	void confirmLeave();

	//keep the Show layout button in step when something else changes the big image area
	void setLayoutShown(bool shown);
	//the px figures beside the mm diameters follow the camera scale
	void refreshScale();

signals:
	void setupChanged();                 //the working copy changed (edit, Save, Discard or load)
	void showLayoutToggled(bool shown);

private:
	Ui::GuidedSetup2Page ui;
	QString _recipeDir, _recipeName;
	gs2::Setup _work, _saved;
	gs2::LoadResult _loadResult = gs2::LoadResult::Missing;
	QString _loadWhy;
	bool _filling = false;   //true while the widgets are being filled from _work

	void connectEdits();
	void fillWidgets();      //_work -> widgets
	void refreshStatus();    //ticks, read-only values, the save state
	void edited();           //a widget changed: widgets -> _work
	bool save();
	void discard();
	QString pxText(double minMm, double maxMm) const;
};
