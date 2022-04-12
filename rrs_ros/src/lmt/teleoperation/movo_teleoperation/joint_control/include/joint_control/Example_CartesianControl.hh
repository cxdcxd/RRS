#ifndef _JOINT_CONTROL_EXAMPLES_HH_
#define _JOINT_CONTROL_EXAMPLES_HH_

#include "KinovaTypes.h"
#include <iostream>

#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <stdio.h>
#include <unistd.h>

using namespace std;

class Examples
{

void * commandLayer_handle;

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetCartesianCommand)(CartesianPosition &);

public:
int runMain();


};
#endif