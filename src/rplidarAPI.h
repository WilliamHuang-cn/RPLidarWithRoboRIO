/*
 * rplidarAPI.h
 *
 */
#include <iostream>
#include "rplidar.h"

#ifndef RPLIDARAPI_H_
#define RPLIDARAPI_H_

#endif /* RPLIDARAPI_H_ */

using namespace rp::standalone::rplidar;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

typedef struct _rplidar_date_t {
	bool	sync;
    float   theta;
    float 	distance;
    int		quality;
} rplidar_date_t;


//opt_com_path = "/dev/tty.SLAB_USBtoUART";
RPlidarDriver * openLidar(int &errorCode, const char * opt_com_path, bool debugInfo);
void closeLidar(RPlidarDriver * drv);
bool readData(RPlidarDriver * drv, bool debugInfo = false);

