#include <rplidarAPI.h>
#include <stdio.h>
#include <iostream>

#include "rplidar.h"

using namespace std;
using namespace rp::standalone::rplidar;

// raw result datum
rplidar_response_measurement_node_t nodes[360*2];
size_t   nodescount = _countof(nodes);
// measurement result datum
rplidar_date_t datum[360*2];

bool checkRPLIDARHealth(RPlidarDriver * drv, bool debugInfo = false)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
    	if (debugInfo) printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
        	if (debugInfo) fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
    	if (debugInfo) fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

//opt_com_path = "/dev/tty.SLAB_USBtoUART";
RPlidarDriver * openLidar(int &errorCode, const char * opt_com_path, bool debugInfo)
{
    _u32        opt_com_baudrate = 115200;
    u_result    op_result;
    RPlidarDriver *drv;

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        if (debugInfo) fprintf(stderr, "insufficent memory, exit\n");
        return NULL;
    }

    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
    	if (debugInfo) fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
    	errorCode = -2;
        goto on_finished;
    }

    rplidar_response_device_info_t devinfo;

    // retrieving the device info
    ////////////////////////////////////////
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
    	if (debugInfo) fprintf(stderr, "Error, cannot get device info.\n");
    	errorCode = -3;
        goto on_finished;

    }

    // print out the device serial number, firmware and hardware version number..
    if (debugInfo) {
    	printf("RPLIDAR S/N: ");
    	for (int pos = 0; pos < 16 ;++pos) {
    		printf("%02X", devinfo.serialnum[pos]);
    	}

    	printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
           , devinfo.firmware_version>>8
           , devinfo.firmware_version & 0xFF
           , (int)devinfo.hardware_version);
    }

    // check health...
    if (!checkRPLIDARHealth(drv, debugInfo)) {
    	errorCode = -4;
        goto on_finished;
    }

//    drv->startMotor();
    // start scan...
//    drv->startScan();
    return drv;

on_finished:
	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;
    return drv;
}

void closeLidar(RPlidarDriver * drv)
{
	if (drv) {
		drv->stop();
		drv->stopMotor();
		RPlidarDriver::DisposeDriver(drv);
		drv = NULL;
	}
}

bool readData(RPlidarDriver * drv, bool debugInfo)
{
	u_result     op_result;

	op_result = drv->grabScanData(nodes, nodescount);

	if (IS_OK(op_result)) {
		drv->ascendScanData(nodes, nodescount);
		for (int pos = 0; pos < (int)nodescount ; ++pos) {
			datum[pos].sync = (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? true:false;
			datum[pos].theta = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
			datum[pos].distance = nodes[pos].distance_q2/4.0f;
			datum[pos].quality = nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

			if (debugInfo) printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
					datum[pos].sync ?"S ":"  ",
					datum[pos].theta,
					datum[pos].distance,
					datum[pos].quality);
		}
	}
	return IS_OK(op_result);
}
