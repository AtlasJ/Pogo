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
		"soft limits, at the speeds set now, until the time is up; every landing is followed for 1 s.\n"
		"  Z GOES DOWN TO ITS LOWEST SOFT LIMIT - no trolley, nothing on the table.\n"
		"  Refused while a trolley is attached. Stop ends it. Returns to the start point at the end.\n\n"
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
	checkPartC->setChecked(true);
	auto* spinRandom = new QSpinBox(&dlg);
	spinRandom->setRange(1, 60);
	spinRandom->setValue(2);
	spinRandom->setSuffix(QStringLiteral(" min"));
	auto* partCRow = new QHBoxLayout();
	partCRow->addWidget(checkPartC);
	partCRow->addWidget(spinRandom);
	partCRow->addStretch();

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
	if (mask == 0 && randomSeconds == 0 && !checkPartB->isChecked()) { showMsg("Nothing to test: tick Part A with an axis, Part C or Part B."); return; }
	//the job thread refuses too; saying it here saves queuing a run that only reports it
	if (randomSeconds > 0 && MachineController::instance().trolleyAttached()) {
		showMsg("Part C moves Z down to its lowest soft limit - remove the trolley first.");
		return;
	}

	const int slow = checkSlow->isChecked() ? spinSlow->value() : 0;
	ui.textEdit_loopStatus->append(QStringLiteral("Motion Accuracy Test: axes mask %1, %2 mm x %3%4%5%6")
		.arg(mask).arg(spinDistance->value()).arg(spinRepeats->value())
		.arg(slow > 0 ? QStringLiteral(", slow pass %1 %").arg(slow) : QString())
		.arg(randomSeconds > 0 ? QStringLiteral(", Part C %1 s").arg(randomSeconds) : QString())
		.arg(checkPartB->isChecked() ? QStringLiteral(", Part B %1 s").arg(spinWatch->value()) : QString()));
	emit signalMotionAccuracyTest(mask, spinDistance->value(), spinRepeats->value(), slow,
		randomSeconds, checkPartB->isChecked(), spinWatch->value());
}
