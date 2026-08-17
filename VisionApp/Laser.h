#pragma once
#include <GoSdk/GoSdk.h>
#include <QDebug>

#define GDK_SAMPLE_CLIENT_DEFAULT_MAIN_ID      (13806)
#define GDK_SAMPLE_CLIENT_DEFAULT_BUDDY_ID     (-1)
#define RECEIVE_TIMEOUT         (20000000) 
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.    
#define SENSOR_IP               "192.168.1.10"                      

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)



class Laser
{
public:
	void retrieveMeasurementData();

	Laser();
	~Laser();
};

