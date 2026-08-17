#pragma once
#include <vector>
#include <QJsonObject>
#include <QHash>
#include "QOrderedHash.h"
#include "ILSC.h"
#include "AdvantechDigitalIO.h"
#include "OpticsInfo.h"

class CameraControl {
private:
	CameraControl();
	~CameraControl();
	CameraControl(const CameraControl&) = delete;
	CameraControl& operator=(const CameraControl&) = delete;

	static CameraControl m_instance;

public:
	static CameraControl& instance();

};