#pragma once
#include <QWidget>
#include <functional>
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
	//the app is closing with unsaved edits: Save and close / Discard and close / Cancel. True = close
	//(a Save that fails keeps the app open)
	bool confirmClose();

	//keep the Show layout button in step when something else changes the big image area
	void setLayoutShown(bool shown);
	//the px figures beside the mm diameters follow the camera scale
	void refreshScale();

	//the Camera and circle finder tools; the actions themselves run in VisionApp
	enum ToolAction { ToolFindInImage = 0, ToolSnapFind, ToolCentre, ToolDirection, ToolSaveMeasured, ToolStability };
	//the circle the tools look for: the settings of the kind picked in Circle (unsaved edits count), and its name
	gs2::CircleFinder toolCircle(QString* what = nullptr) const;
	void showToolResult(const QString& text, bool ok);
	void setMachineBusy(bool busy);      //tools greyed out while the machine is being driven
	//the machine-wide pixel-to-mm choice: which one is in use, what it is, and whether a check is waiting to be saved
	void setCameraConversion(bool useMeasured, const QString& text, bool warn, bool canSaveMeasured);

	//sections A and B (build step 3a): VisionApp drives the machine and writes the result back with modify()
	enum TeachAction { TeachSnapEmpty = 0, TeachScore, TeachXY, TeachJogXY, TeachSetStart, TeachSetStop, TeachLearnPattern, TeachFind };
	//a taught value into the working copy (unsaved until Save); a changed fid0 setting clears its last Find
	void modify(const std::function<void(gs2::Setup&)>& change);
	//a passed Find. A run result, not an edit: when the file holds the fid0 settings it ran with it is
	//written there at once and never asked Save or Discard; run with unsaved fid0 settings it stays
	//with them in the working copy and goes wherever they go
	void recordFid0Found(const gs2::Fid0Found& found);
	void showSafetyScore(const QString& text, bool ok);
	void showFid0Result(const QString& text, bool ok);
	void setPatternBoxShown(bool shown);  //keep the Pattern box button in step when the box is hidden elsewhere

signals:
	void setupChanged();                 //the working copy changed (edit, Save, Discard or load)
	void showLayoutToggled(bool shown);
	void toolRequested(int action);      //a ToolAction
	void conversionChosen(bool measured); //the operator picked Set or Measured
	void teachRequested(int action);     //a TeachAction
	void patternBoxToggled(bool shown);

private:
	Ui::GuidedSetup2Page ui;
	QString _recipeDir, _recipeName;
	gs2::Setup _work, _saved;
	gs2::LoadResult _loadResult = gs2::LoadResult::Missing;
	QString _loadWhy;
	//the file read cleanly when the recipe opened, so what it refers to is known: only then do Save and
	//Discard remove unused gs2 files (an unreadable or missing file would leave nothing "in use")
	bool _mayClean = false;
	bool _filling = false;   //true while the widgets are being filled from _work
	bool _canSaveMeasured = false;

	void connectEdits();
	void fillWidgets();      //_work -> widgets
	void refreshStatus();    //ticks, read-only values, the save state
	void edited();           //a widget changed: widgets -> _work
	bool save();
	void discard();
	QString pxText(double minMm, double maxMm) const;
	//the Find result that goes with these fid0 settings: the saved one when they are the saved settings
	gs2::Fid0Found savedFoundFor(const gs2::Setup& s) const;
	int circleKind() const;                 //the kind picked in Circle: gs2::CircleKind
	QString circleSummary(int kind) const;  //one line for sections B, D and E
};
