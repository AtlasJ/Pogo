#pragma once
#include <QWidget>
#include <functional>
#include "ui_GuidedSetup2Page.h"
#include "GuidedSetup2Types.h"

/*
* The right-panel page of the Guided Setup (new). It edits a working copy of one recipe's
* guidedSetup2.json; nothing reaches the file until Save, and Discard goes back to what was saved.
* Taught values (positions, sizes measured by the machine) are shown read-only here and taught by
* the section buttons, which VisionApp carries out. Typed settings can be edited at any time.
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

	//sections A-D and the Locate test (build step 3): VisionApp drives the machine and writes the result
	//back with modify() or a record...() call
	enum TeachAction { TeachSnapEmpty = 0, TeachScore, TeachXY, TeachJogXY, TeachSetStart, TeachSetStop, TeachLearnPattern, TeachFind,
		TeachZ3DSpot, TeachZ3DFind, TeachZ3DDiff,
		TeachCornerTL, TeachCornerTR, TeachCornerBR, TeachCornerBL,   //in gs2geo::Corner order
		TeachRectCheck, TeachLocate,
		TeachJogCornerTL, TeachJogCornerTR, TeachJogCornerBR, TeachJogCornerBL,   //in gs2geo::Corner order
		TeachSlotTL, TeachSlotBR, TeachSlotAdd, TeachSlotInterpolate,          //sections E and F (build step 4)
		TeachInsp2D, TeachInspBarcodeOcr, TeachInsp3D,                         //section H's Set, in gs2::InspType order
		TeachMoveOnly, TeachGoTo,                                              //section I's tests (build step 4b)
		TeachSnapScan };                                                       //...and production's capture (build step 5)
	//I's choices: Move only's points (0 = 2D, 1 = 3D, 2 = 2D + 3D); Go to's position (gs2GoPos order) and tool
	enum MoveWhat { Move2D = 0, Move3D, MoveBoth };
	enum GoPos { GoCentre = 0, GoTopLeft, GoTop, GoTopRight, GoLeft, GoRight, GoBottomLeft, GoBottom, GoBottomRight,
		Go2DPoint, GoBarcodePoint, Go3DStart };
	int moveOnlyWhat() const;
	int snapScanWhat() const;   //Snap and scan's points, as MoveWhat
	int goToPosition() const;
	bool goToWithProfiler() const;
	//what F's Add and Interpolate are asked to do: the corner the camera is on (gs2geo::Corner), the slots
	//selected in the list (ids) and how many go between them
	int slotAddCorner() const;
	QStringList selectedSlotIds() const;
	int slotsBetween() const;
	void selectSlots(const QStringList& ids);   //the layout view picked these; no slotSelectionChanged back
	//a taught value into the working copy (unsaved until Save); a changed setting clears the run result
	//it no longer goes with (B's Find, C's Find, D's Check)
	void modify(const std::function<void(gs2::Setup&)>& change);
	//a passed Find or Check. A run result, not an edit: when the file holds the settings it ran with it
	//is written there at once and never asked Save or Discard; run with unsaved settings it stays with
	//them in the working copy and goes wherever they go
	void recordFid0Found(const gs2::Fid0Found& found);
	void recordZ3DFound(const gs2::Z3DFound& found);
	void recordFidRectFound(const gs2::FidRectFound& found);
	void showSafetyScore(const QString& text, bool ok);
	void showFid0Result(const QString& text, bool ok);
	void showZ3DResult(const QString& text, bool ok);
	void showRectResult(const QString& text, bool ok);
	void showLocateResult(const QString& text, bool ok);
	void showSlotSizeResult(const QString& text, bool ok);
	void showSlotsResult(const QString& text, bool ok);
	void showInspResult(const QString& text, bool ok);
	void setPatternBoxShown(bool shown);  //keep the Pattern box button in step when the box is hidden elsewhere

signals:
	void setupChanged();                 //the working copy changed (edit, Save, Discard or load)
	void showLayoutToggled(bool shown);
	void toolRequested(int action);      //a ToolAction
	void conversionChosen(bool measured); //the operator picked Set or Measured
	void teachRequested(int action);     //a TeachAction
	void patternBoxToggled(bool shown);
	void slotSelectionChanged(const QStringList& ids);   //the operator changed the selection in F's list

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
	//s changed from before: each run result whose settings changed becomes the saved one when they are
	//back to the saved settings, else none
	void keepResults(const gs2::Setup& before, gs2::Setup& s) const;
	//put the result into the working copy, and into the file at once when it holds the settings the
	//result proves (same); what = its name in the log
	void recordResult(const char* what, const std::function<void(gs2::Setup&)>& put, bool (*same)(const gs2::Setup&, const gs2::Setup&));
	void deleteSlots();                     //F's Delete and Rename: edits, no machine
	void renameSlot();
	int circleKind() const;                 //the kind picked in Circle: gs2::CircleKind
	QString circleSummary(int kind) const;  //one line for sections B, D and E
};
