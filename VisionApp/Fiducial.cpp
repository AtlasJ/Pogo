#include "Fiducial.h"
#include <iostream>
#include "Logger.h"
#include "SystemData.h"

void Fiducial::reset()
{
	fid1_isSet = false;
	fid2_isSet = false;
	fid1 = em::V2d(0,0);
	fid2 = em::V2d(0,0);
	shifted_fid1 = em::V2d(0,0);
	shifted_fid2 = em::V2d(0,0);
	//R = em::M2d();
	//m_angle = 0;
	ct::logger::debug("Reset Fiducial");
}

void Fiducial::setLearntFid(int index, const em::V2d & fid)
{
	if (index == 0) this->fid1 = fid;
	else if (index == 1) this->fid2 = fid;
}

void Fiducial::setShiftedFid(int index, const em::V2d & fid)
{
	if (index == 0) {
		this->shifted_fid1 = fid;
		fid1_isSet = true;
		//printf("SET sfid1: %f, %f\n", shifted_fid1.x(), shifted_fid1.y());
	}
	else if (index == 1) {
		this->shifted_fid2 = fid;
		fid2_isSet = true;
		//printf("SET sfid2: %f, %f\n", shifted_fid2.x(), shifted_fid2.y());
	}
}

void Fiducial::setLearntFid(const em::V2d & fid1, const em::V2d & fid2)
{
	this->fid1 = fid1;
	this->fid2 = fid2;
}

void Fiducial::setShiftedFid(const em::V2d & fid1, const em::V2d & fid2)
{
	this->shifted_fid1 = fid1;
	this->shifted_fid2 = fid2;
	fid1_isSet = true;
	fid2_isSet = true;
}

em::V2d Fiducial::getShiftedFid(int index)
{
	if (index == 0) {
		//printf("GET sfid1: %f, %f\n", shifted_fid1.x(), shifted_fid1.y());
		return shifted_fid1;
	}
	if (index == 1) {
		//printf("GET sfid2: %f, %f\n", shifted_fid2.x(), shifted_fid2.y());
		return shifted_fid2;
	}
	return em::V2d(0,0);
}

void Fiducial::compute()
{

	if (fid1_isSet && fid2_isSet) {
		//shift to origin
		auto o_fid1 = fid1 - fid1;
		auto o_fid2 = fid2 - fid1;
		em::V2d o_shifted_fid1 = shifted_fid1 - shifted_fid1;
		em::V2d o_shifted_fid2 = shifted_fid2 - shifted_fid1;


		auto a = o_fid2 - o_fid1;
		auto b = o_shifted_fid2 - o_shifted_fid1;

		//get rotated angle
		m_angle = em::compute_angleBetweenVectors(a, b);
		ct::logger::debug("Learnt Fiducial (2): %f, %f", fid2.x(), fid2.y());
		ct::logger::debug("Shifted Fiducial (2): %f, %f", shifted_fid2.x(), shifted_fid2.y());
		ct::logger::debug("Angle: %f", m_angle);

		//add a checking if angle greater than 3 degrees absolute value

		//add a checking if offset exceed more than 1/2 of the fov size then fail the fiducial


		

		double rad = em::to_radian(m_angle);
		R = em::compute_2DRotationMatrix(rad);
		std::cout << "R:" << R << std::endl;

		//offset 
		offset = shifted_fid1 - fid1;
		ct::logger::debug("Offset (1&2): %f, %f", offset.x(), offset.y());

		//Rotation + Translation Matrix
		/*M(0, 0) = R(0, 0);
		M(0, 1) = R(0, 1);
		M(1, 0) = R(1, 0);
		M(1, 1) = R(1, 1);
		M(0, 2) = fid1.x() + offset.x();
		M(1, 2) = fid1.y() + offset.y();
		M(2, 0) = 0;
		M(2, 1) = 0;
		M(2, 2) = 1;*/
	}
	else if (fid1_isSet) {
		offset = shifted_fid1 - fid1;
		ct::logger::debug("Learnt Fiducial (1): %f, %f", fid1.x(), fid1.y());
		ct::logger::debug("Shifted Fiducial (1): %f, %f", shifted_fid1.x(), shifted_fid1.y());
		ct::logger::debug("Offset (1): %f, %f", offset.x(), offset.y());
	}
	else if (fid2_isSet) {
		offset = shifted_fid2 - fid2;
		ct::logger::debug("Learnt Fiducial (2): %f, %f", fid2.x(), fid2.y());
		ct::logger::debug("Shifted Fiducial (2): %f, %f", shifted_fid2.x(), shifted_fid2.y());
		ct::logger::debug("Offset (2): %f, %f", offset.x(), offset.y());
	}

	QVector<em::V2d> fiducialPoints;
	fiducialPoints.append(fid1);
	fiducialPoints.append(fid2);

}

em::V2d Fiducial::getShiftedPoint(const em::V2d & point)
{
	em::V2d N = point;

	if (SystemData::instance()._enableFiducialRotate)
	{
		// apply fiducial rotate
		if (fid1_isSet && fid2_isSet) {
			/*auto o_point = point + offset - fid1;
			N = R * o_point + fid1;*/

			auto o_point = point - fid1;
			N = R * o_point + fid1 + offset;
			//em::V2d T = R * offset;
			//em::V2d O = R * o_point + fid1;
		}
		else if (fid1_isSet) {
			N = point + offset;
		}
		else if (fid2_isSet) {
			N = point + offset;
		}
	}
	else
	{
		// apply normal fiducial
		if (fid1_isSet) {
			N = point + offset;
		}
		else if (fid2_isSet) {
			N = point + offset;
		}
	}

	return N;
}

double Fiducial::getAngle()
{
	return m_angle;
}

em::V2d Fiducial::getOffset()
{
	return offset;
}

bool Fiducial::isSet(int index)
{
	if (index == 0) return fid1_isSet;
	if (index == 1) return fid2_isSet;
	return false;
}
