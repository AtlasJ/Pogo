// =============================================================================
//  VisionApp_MotionTest.cpp
//  Test Run page -> Online -> "Motion Accuracy Test" -> Run. The dialog only;
//  the test itself is JobThread::motionAccuracyTest (JobThread_MotionTest.cpp).
// =============================================================================

#include "VisionApp.h"

/*
* Two questions the motion team needs answered with numbers rather than impressions, after the
* 2026-10-27 IPC logs: does X (and Y) park short of its command, and is that speed or tuning; and
* do the axes move between a curtain trip / e-stop and the moment their drives really hold again.
*/
void VisionApp::runMotionAccuracyTest()
{
	if (!MotionController::instance().is_init(_motionID)) { showMsg("Motion is not initialised."); return; }
	if (_productionSetupLocked || isStartTimer) { showMsg("Not while a production run is in progress."); return; }
	if (_zsBusy) { showMsg("A Z search is moving the machine."); return; }
	if (_motionTestRunning) { showMsg("A Motion Accuracy Test is already running."); return; }

	QDialog dlg(this);
	dlg.setWindowTitle(QStringLiteral("Motion Accuracy Test"));

	const auto here = SystemData::instance().currentCoordinate();

	auto* info = new QLabel(QStringLiteral(
		"Part A - landing accuracy. Each ticked axis moves back and forth over the distance below,\n"
		"arriving from both directions, and after every landing the encoder and the card's command\n"
		"counter are read at 0, 0.1, 0.25, 0.5, 1 and 2 s: an error that stays is a static offset,\n"
		"one that shrinks is settling. The slow pass repeats it all at a fraction of the speed.\n\n"
		"  X and Y move AT THE CURRENT Z (now %1 mm) - lift Z clear of the table first.\n"
		"  Z only moves UP from where it is now and back down to it, never lower.\n"
		"  Every target is checked against the soft limits first. Stop ends Part A.\n\n"
		"Part C - random moves. X, Y and Z move together to random points ANYWHERE inside their\n"
		"soft limits, at the speeds set now, until the time is up; every landing is followed for 2 s.\n"
		"  Z GOES DOWN TO ITS LOWEST SOFT LIMIT - no trolley, nothing on the table.\n"
		"  Refused while a trolley is attached. Stop ends it. Returns to the start point at the end.\n\n"
		"Part D - short vs long moves. Each ticked axis on its own, the others standing still: a block\n"
		"of SHORT moves (2-20 mm, sweeping the whole travel - about 3 min per sweep for X or Y), a block\n"
		"of LONG moves (40 % of its travel or more), SHORT again, each block for the time below.\n"
		"Shows which kind of move makes the offset grow, where along the axis, and whether the axes\n"
		"standing still get pushed by the one moving.\n"
		"  Z can go down to its lowest soft limit; X and Y move at the current Z. Refused with a trolley.\n\n"
		"Part B - curtain trip / e-stop and reset. Nothing moves. When asked, trip the curtain (or\n"
		"press an e-stop), then press Reset as usual. It records what each axis and its drive do\n"
		"through that, and stops 30 s after the drives hold again (or at the time limit).\n\n"
		"A report and CSV files are written to C:\\Advanced\\Data\\MotionTest.")
		.arg(here.wz, 0, 'f', 3), &dlg);
	info->setStyleSheet(QStringLiteral("color: #929aaa;"));

	auto* checkX = new QCheckBox(QStringLiteral("X"), &dlg);
	auto* checkY = new QCheckBox(QStringLiteral("Y"), &dlg);
	auto* checkZ = new QCheckBox(QStringLiteral("Z"), &dlg);
	checkX->setChecked(true);
	checkY->setChecked(true);
	auto* axesRow = new QHBoxLayout();
	axesRow->addWidget(checkX);
	axesRow->addWidget(checkY);
	axesRow->addWidget(checkZ);
	axesRow->addStretch();

	auto* spinDistance = new QDoubleSpinBox(&dlg);
	spinDistance->setRange(0.1, 100.0);
	spinDistance->setDecimals(2);
	spinDistance->setValue(10.0);
	spinDistance->setSuffix(QStringLiteral(" mm"));

	auto* spinRepeats = new QSpinBox(&dlg);
	spinRepeats->setRange(1, 50);
	spinRepeats->setValue(10);

	auto* checkSlow = new QCheckBox(QStringLiteral("Also run slow, at"), &dlg);
	checkSlow->setChecked(true);
	auto* spinSlow = new QSpinBox(&dlg);
	spinSlow->setRange(5, 90);
	spinSlow->setValue(10);
	spinSlow->setSuffix(QStringLiteral(" % of the speed"));
	auto* slowRow = new QHBoxLayout();
	slowRow->addWidget(checkSlow);
	slowRow->addWidget(spinSlow);
	slowRow->addStretch();

	auto* checkPartA = new QCheckBox(QStringLiteral("Part A - landing accuracy"), &dlg);

	auto* checkPartC = new QCheckBox(QStringLiteral("Part C - random X/Y/Z moves for"), &dlg);
	auto* spinRandom = new QSpinBox(&dlg);
	spinRandom->setRange(1, 60);
	spinRandom->setValue(2);
	spinRandom->setSuffix(QStringLiteral(" min"));
	auto* partCRow = new QHBoxLayout();
	partCRow->addWidget(checkPartC);
	partCRow->addWidget(spinRandom);
	partCRow->addStretch();

	auto* checkPartD = new QCheckBox(QStringLiteral("Part D - short vs long moves, one axis at a time:"), &dlg);
	checkPartD->setChecked(true);
	auto* checkDX = new QCheckBox(QStringLiteral("X"), &dlg);
	auto* checkDY = new QCheckBox(QStringLiteral("Y"), &dlg);
	auto* checkDZ = new QCheckBox(QStringLiteral("Z"), &dlg);
	checkDX->setChecked(true);
	auto* spinBlock = new QSpinBox(&dlg);
	spinBlock->setRange(1, 20);
	spinBlock->setValue(3); //one full short sweep of X or Y
	spinBlock->setSuffix(QStringLiteral(" min per block"));
	auto* partDRow = new QHBoxLayout();
	partDRow->addWidget(checkPartD);
	partDRow->addWidget(checkDX);
	partDRow->addWidget(checkDY);
	partDRow->addWidget(checkDZ);
	partDRow->addWidget(spinBlock);
	partDRow->addStretch();

	auto* checkPartB = new QCheckBox(QStringLiteral("Part B - curtain trip / e-stop and reset, for up to"), &dlg);
	auto* spinWatch = new QSpinBox(&dlg);
	spinWatch->setRange(30, 600);
	spinWatch->setValue(180);
	spinWatch->setSuffix(QStringLiteral(" s"));
	auto* partBRow = new QHBoxLayout();
	partBRow->addWidget(checkPartB);
	partBRow->addWidget(spinWatch);
	partBRow->addStretch();

	auto* form = new QFormLayout();
	form->addRow(checkPartA);
	form->addRow(QStringLiteral("Axes"), axesRow);
	form->addRow(QStringLiteral("Distance"), spinDistance);
	form->addRow(QStringLiteral("Repeats"), spinRepeats);
	form->addRow(slowRow);
	form->addRow(partCRow);
	form->addRow(partDRow);
	form->addRow(partBRow);

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("Run"));
	connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

	auto* layout = new QVBoxLayout(&dlg);
	layout->addWidget(info);
	layout->addLayout(form);
	layout->addWidget(buttons);

	if (dlg.exec() != QDialog::Accepted) return;

	int mask = 0;
	if (checkPartA->isChecked()) {
		if (checkX->isChecked()) mask |= 1 << (int)Axis::X;
		if (checkY->isChecked()) mask |= 1 << (int)Axis::Y;
		if (checkZ->isChecked()) mask |= 1 << (int)Axis::Z;
	}
	const int randomSeconds = checkPartC->isChecked() ? spinRandom->value() * 60 : 0;
	int travelMask = 0;
	if (checkPartD->isChecked()) {
		if (checkDX->isChecked()) travelMask |= 1 << (int)Axis::X;
		if (checkDY->isChecked()) travelMask |= 1 << (int)Axis::Y;
		if (checkDZ->isChecked()) travelMask |= 1 << (int)Axis::Z;
		if (travelMask == 0) { showMsg("Part D is ticked but no axis is: tick X, Y or Z for it."); return; }
	}
	const int blockSeconds = travelMask ? spinBlock->value() * 60 : 0;
	if (mask == 0 && randomSeconds == 0 && travelMask == 0 && !checkPartB->isChecked()) {
		showMsg("Nothing to test: tick Part A with an axis, Part C, Part D or Part B.");
		return;
	}
	//the job thread refuses too; saying it here saves queuing a run that only reports it
	if ((randomSeconds > 0 || travelMask != 0) && MachineController::instance().trolleyAttached()) {
		showMsg("Parts C and D move Z down to its lowest soft limit - remove the trolley first.");
		return;
	}

	const int slow = checkSlow->isChecked() ? spinSlow->value() : 0;
	ui.textEdit_loopStatus->append(QStringLiteral("Motion Accuracy Test: axes mask %1, %2 mm x %3%4%5%6%7")
		.arg(mask).arg(spinDistance->value()).arg(spinRepeats->value())
		.arg(slow > 0 ? QStringLiteral(", slow pass %1 %").arg(slow) : QString())
		.arg(randomSeconds > 0 ? QStringLiteral(", Part C %1 s").arg(randomSeconds) : QString())
		.arg(travelMask ? QStringLiteral(", Part D axes mask %1, %2 s per block").arg(travelMask).arg(blockSeconds) : QString())
		.arg(checkPartB->isChecked() ? QStringLiteral(", Part B %1 s").arg(spinWatch->value()) : QString()));
	_motionTestRunning = true; //cleared by motionAccuracyTestDone (VisionApp_JobThread.cpp)
	emit signalMotionAccuracyTest(mask, spinDistance->value(), spinRepeats->value(), slow,
		randomSeconds, 0, travelMask, blockSeconds, 0, checkPartB->isChecked(), spinWatch->value());
}

/*
* Test Run page -> Online -> "Full Motion Accuracy Test": every moving part of the Motion Accuracy
* Test with fixed settings, about 55 minutes, one report - for an unattended run (Jimmy, 2026-10-28:
* "test all axis, all position, all speed, all distance, mixed axis, single axis"). Fewest axes
* first, so whatever makes X drift shows up at the first part that has it.
*/
void VisionApp::runFullMotionAccuracyTest()
{
	if (!MotionController::instance().is_init(_motionID)) { showMsg("Motion is not initialised."); return; }
	if (_productionSetupLocked || isStartTimer) { showMsg("Not while a production run is in progress."); return; }
	if (_zsBusy) { showMsg("A Z search is moving the machine."); return; }
	if (_motionTestRunning) { showMsg("A Motion Accuracy Test is already running."); return; }
	if (MachineController::instance().trolleyAttached()) { showMsg("The full test moves Z down to its lowest soft limit - remove the trolley first."); return; }

	constexpr int kAllAxes = 7;            //X, Y and Z
	constexpr double kDistanceMm = 30.0;   //Part A: Z only goes UP from where it starts, and a homed Z has about 33 mm above it
	constexpr int kRepeats = 10;
	constexpr int kSlowPercent = 25;
	constexpr int kBlockSeconds = 120;     //Part D: 3 blocks x 3 axes = 18 min
	constexpr int kPairSeconds = 240;      //Part E: 3 pairs = 12 min
	constexpr int kRandomSeconds = 600;    //Part C: 10 min at 100 %, then 5 min at 50 %
	constexpr int kRandomSlowPercent = 50;

	const auto answer = QMessageBox::question(this, QStringLiteral("Full Motion Accuracy Test"), QStringLiteral(
		"Runs every moving test in one go, about 55 minutes, and writes one report to\n"
		"C:\\Advanced\\Data\\MotionTest.\n\n"
		"  1. Each axis alone, back and forth over 30 mm, 10 times each way, at 100 % and 25 % speed (~6 min)\n"
		"  2. Each axis alone, short moves sweeping the whole travel and long moves, 2 min per block,\n"
		"     the other two axes watched (18 min)\n"
		"  3. Two axes together, X+Y, X+Z, Y+Z, the third watched, 4 min each (12 min)\n"
		"  4. All three together, random points, 10 min at 100 % then 5 min at 50 % speed (15 min)\n\n"
		"Every target stays in the middle 98 % of the soft limits. Z goes down to its lowest test point:\n"
		"no trolley, nothing on the table. Home the machine first. Stop ends it at any time.\n\n"
		"Start now?"),
		QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
	if (answer != QMessageBox::Yes) return;

	ui.textEdit_loopStatus->append(QStringLiteral("Full Motion Accuracy Test started - about 55 minutes"));
	_motionTestRunning = true; //cleared by motionAccuracyTestDone (VisionApp_JobThread.cpp)
	emit signalMotionAccuracyTest(kAllAxes, kDistanceMm, kRepeats, kSlowPercent, kRandomSeconds, kRandomSlowPercent,
		kAllAxes, kBlockSeconds, kPairSeconds, false, 0);
}
