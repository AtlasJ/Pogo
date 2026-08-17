#pragma once
#include "EM_Core.h"
#include "EM_Vector.h"
#include <QVector>

class Fiducial {
private:
	em::V2d fid1, fid2;
	em::V2d shifted_fid1, shifted_fid2;
	em::V2d offset;
	em::M2d R;
	bool fid1_isSet = false, fid2_isSet = false;
	double m_angle = 0;

public:
	void reset();
	void compute();

	void setLearntFid(int index, const em::V2d& fid);
	void setShiftedFid(int index, const em::V2d& fid);
	void setLearntFid(const em::V2d& fid1, const em::V2d& fid2); 
	void setShiftedFid(const em::V2d& fid1, const em::V2d& fid2);
	em::V2d getShiftedFid(int index);
	em::V2d getShiftedPoint(const em::V2d& point);
	double getAngle();
	em::V2d getOffset();
	bool isSet(int index);
};