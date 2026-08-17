#include "VisionApp.h"

void VisionApp::rtrComms(QString data) {
	return;
	_rtr_teachMode[0] = false;
	_rtr_teachMode[1] = false;
	_rtr_teachMode[2] = false;

	//ct::logger::debug("Running: %s", data.toStdString().c_str());
	//for (const QChar& ch : data) {
	//	qDebug() << "CH: " << ch << "Unicode: " << int(ch.unicode());
	//}

	double fX = 0.0;
	double fY = 0.0;
	double fT = 0.0;
	QString rc = "0";
	int dataValid = 1;

	// => Open Lot Sequence
	// Handler request Recipe Mode from vision.
	// GP reply Recipe Mode. 0: Manual, 1: Auto
	if (data == "#61") {
		sendToClient("#61,0\r"); //TODO:
	}
	else if (data == "#62,A") {
		//Do something for track A or B 
	}
	else if (data.contains("#63")) { //UPH,MTBA
		auto infos = data.split(',');
		auto UPH = infos[1];
		auto MTBA = infos[2];
		sendToClient("R\r");
	}
	else if (data == "#65,0") {
		//TODO: disable secgem
		sendToClient("R\r");
	}
	else if (data == "#65,1") {
		//TODO: enable secgem
		sendToClient("R\r");
	}
	else if (data == "#53") { //Handler request package name
		sendToClient(QString("#53,%1\r").arg(Common::Directory::CurrentRecipe));
	}
	else if (data == "#54") {//Handler request package ID
		sendToClient(QString("#54,%1\r").arg(Common::Directory::CurrentRecipe));
	}
	else if (data == "#88,1") {
		//Reset Statistic
		sendToClient("R\r");
	}
	else if (data.contains("#95")) { //set current app
		QString appID = data.section(',', 1);
		sendToClient("R\r");
	}
	else if (data == "%151,0,0,4") { //Request true reject status for app1
		sendToClient("%151,0,0,4,0\r"); //0: Off, 1: On
	}
	else if (data == "%152,0,0,4") { //Request true reject status for app2
		sendToClient("%152,0,0,4,0\r"); //0: Off, 1: On
	}
	else if (data == "Z") {//END LOT
		sendToClient("R\r");
	}
	else if (data == "S") {// Data start
		sendToClient("R\r");
	}
	else if (data == "E") {// Data End
		sendToClient("R\r");
	}
	else if (data == "C") {// Check mode
		sendToClient("U\r"); //Run mode
		//u: Stop mode;
	}
	else if (data == "U") { //Run Mode
		sendToClient("R\r");
	}
	else if (data.contains("#01")) {
		QString lotNumber = data.section(',', 1);
		printf("01\n");
	}
	else if (data.contains("#02")) {
		QString lotSize = data.section(',', 1);
		printf("02\n");
	}
	else if (data.contains("#03")) {
		QString packageID = data.section(',', 1);
		printf("03\n");
	}
	else if (data.contains("#04")) {
		QString operatorID = data.section(',', 1);
		printf("04\n");
	}
	else if (data.contains("#05")) {
		QString machineName = data.section(',', 1);
		printf("05\n");
	}
	else if (data.contains("#06")) {
		QString grossUPH = data.section(',', 1);
		printf("06\n");
	}
	else if (data.contains("#07")) {
		QString netUPH = data.section(',', 1);
		printf("07\n");
	}
	else if (data.contains("#08")) {
		QString shiftID = data.section(',', 1);
		printf("08\n");
	}
	else if (data.contains("#09")) {
		QString deviceID = data.section(',', 1);
		printf("09\n");
	}
	else if (data.contains("#60")) { //RTR Lot Info
		if (data == "#60,1") { //Teach Status, 0: Skip Teach, 1: Perform Teach
			sendToClient("#60,1,1\r");
		}
		else if (data == "#60,2") { //Recipe Name
			sendToClient(QString("#60,2,%1\r").arg(Common::Directory::CurrentRecipe));
		}
		else if (data == "#60,3") { //Tape Width
			sendToClient("#60,3\r");
		}
		else if (data == "#60,4") { //Pocket Pitch
			sendToClient("#60,4\r");
		}
		else if (data == "#60,5") { //Reel Quantity
			sendToClient("#60,5\r");
		}
	}
	else if (data.contains("#99") || data.contains("#96")) { //Open Lot Recipe ID
		QString recipe = data.section(',', 1);
		//if (openRecipe(recipe)) {
		if (true) {
			sendToClient("R\r");
		}
		else {
			sendToClient("F\r");
		}
	}
	//=> App1
	else if (data == "#141,1") { //Teaching
		_rtr_teachMode[0] = true;
		printf("141,1\n");
	}
	else if (data == "%111,2,1,0") { //Verify teach status
		//%111,2,1,0,1: Valid teach
		//%111,2,1,0,0: Invalid teach
		sendToClient("%111,2,1,0,1\r");
	}
	else if (data.contains("#111")) { //Notify unit being inspect
		_rtr_unitID[0] = data.section(',', 1);
		printf("Set Unit ID: %s\n", _rtr_unitID[0].toStdString().c_str());
	}
	else if (data == "1") { //Inspecting App1
		//trigger cam1, unitID set to viewID, 
		if (_rtr_teachMode[0]) {
			sendToClient("#101,1\r"); //Pic taken, start teach
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			sendToClient("#101,0\r"); //End of teach
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			sendToClient("?101,1,0\r"); //Teach succeed
			//?101,1,p: Teach fail
		}
		else {
			sendToClient("#101,0\r");
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			sendToClient(QString("?101,%1,%2\r").arg(_rtr_unitID[0]).arg(rc));
			//?101,unitID,resultCode (Check doc pg10)
		}
	}
	//=> App2
	else if (data == "#142,1") { //Teaching
		_rtr_teachMode[1] = true;
		printf("141,2\n");
	}
	else if (data == "%112,2,1,0") { //Verify teach status
		//%112,2,1,0,1: Valid teach
		//%112,2,1,0,0: Invalid teach
		sendToClient("%112,2,1,0,1\r");
	}
	else if (data.contains("#112")) { //Notify unit being inspect
		_rtr_unitID[1] = data.section(',', 1);
		printf("Set Unit ID: %s\n", _rtr_unitID[1].toStdString().c_str());
	}
	else if (data == "2") { //Inspecting App2
		//trigger cam2, light1
		if (_rtr_teachMode[1]) {
			sendToClient("#102,1\r"); //Pic taken, start teach
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			sendToClient("#102,0\r"); //End of teach
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			sendToClient(QString("&102,1,%1,%2,%3,%4,%5\r") //&102,1,0,1,fX,fY,fT: Teach succeed
				.arg(rc).arg(dataValid)
				.arg(fX, 0, 'f', 3)
				.arg(fY, 0, 'f', 3)
				.arg(fT, 0, 'f', 3));
			//&102,1,p,1,0,0,0: Teach fail
		}
		else {
			sendToClient("#102,0\r");
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			sendToClient(QString("&102,%1,%2,%3,%4,%5,%6\r")
				.arg(_rtr_unitID[1]).arg(rc).arg(dataValid)
				.arg(fX, 0, 'f', 3)
				.arg(fY, 0, 'f', 3)
				.arg(fT, 0, 'f', 3));
			//&102,unitID,resultCode,dataValid,fX,fY,fT (Check doc pg14)
		}
	}
	//=> Sampling Inspection
	else if (data == "%102,3,ModuleID,0,1") {
		//Skip an inspection module for next SOI.
	}
	else if (data == "%102,8,0,0,0") {
		//Skip all count for next SOI.
	}
	else if (data == "#112,") {
		//Skip all count for next SOI.
	}
	//=> App3
	else if (data == "#143,1") { //Teaching
		_rtr_teachMode[2] = true;
	}
	else if (data == "%113,2,1,0") { //Verify teach status
		//%113,2,1,0,1: Valid teach
		//%113,2,1,0,0: Invalid teach
		sendToClient("%113,2,1,0,1\r");
	}
	else if (data.contains("#113")) { //Notify unit being inspect
		_rtr_unitID[2] = data.section(',', 1);
	}
	else if (data == "3") { //Inspecting App3
		//trigger cam2, light2
		if (_rtr_teachMode[2]) {
			sendToClient("#103,1\r"); //Pic taken, start teach
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			sendToClient("#103,0\r"); //End of teach
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			sendToClient(QString("&103,1,%1,%2,%3,%4,%5\r") //&102,1,0,1,fX,fY,fT: Teach succeed
				.arg(rc).arg(dataValid)
				.arg(fX, 0, 'f', 3)
				.arg(fY, 0, 'f', 3)
				.arg(fT, 0, 'f', 3));
			//&103,1,p,1,0,0,0: Teach fail
		}
		else {
			if (_first) {
				_first = false;
				sendToClient("F\r");
			}
			else {
				sendToClient("#103,0\r");
				std::this_thread::sleep_for(std::chrono::milliseconds(50));

				_toggle = !_toggle;

				if (_toggle) {
					sendToClient("&103,1,A,0\r");
				}
				else {
					sendToClient(QString("&103,1,%1,%2,%3,%4,%5\r")
						.arg(rc).arg(dataValid)
						.arg(fX, 0, 'f', 3)
						.arg(fY, 0, 'f', 3)
						.arg(fT, 0, 'f', 3));
					//&103,1,resultCode,dataValid,fX,fY,fT (Check doc pg18)
				}
			}
		}
	}
}