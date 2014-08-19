#include "../librobosim/inc/nxt.hpp"
#ifdef _WIN32
#include <windows.h>
#endif
#include <ch.h>

struct langflags {
	int tmp1;
	char *tmp2;
	int embedch;
};
extern struct langflags *e_lang;

EXPORTCH void CNXT_CNXT_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *c=new CNXT();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CNXT));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CNXT_dCNXT_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CNXT *);
	if(Ch_CppIsArrayElement(interp))
		c->~CNXT();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CNXT_blinkLED_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double delay;
	int numBlinks;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	delay = Ch_VaArg(interp, ap, double);
	numBlinks = Ch_VaArg(interp, ap, int);
	retval = robot->blinkLED(delay, numBlinks);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	char *name;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	name = Ch_VaArg(interp, ap, char *);

	// pause if not debugging
	int pause = 3;
	if (e_lang != NULL) {
		pause = 3;	// no pause
	}
	else {
		pause = 0;	// pause
	}

	retval = robot->connect(name, pause);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_delay_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double milliseconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	milliseconds = Ch_VaArg(interp, ap, double);
	retval = robot->delay(milliseconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_delaySeconds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->delaySeconds(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_disableRecordDataShift_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->disableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_disconnect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->disconnect();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->driveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->driveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_driveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->driveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_drivexy_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexy(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_drivexyNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyNB(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_drivexyTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyTo(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_drivexyToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyToNB(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

typedef double (*IdrivexyFuncHandle)(double);
static ChInterp_t interpI;
static double IdrivexyFunc_chdl_funarg(double x);
static void *IdrivexyFunc_chdl_funptr;
EXPORTCH int CNXT_drivexyToFunc_chdl(void *varg) {
	ChVaList_t ap;
	class CNXT *robot;
	double x0;
	double xf;
	int n;
	IdrivexyFuncHandle handle_ch, handle_c = NULL;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interpI, ap, varg);
	robot = Ch_VaArg(interpI, ap, class CNXT *);
	x0 = Ch_VaArg(interpI, ap, double);
	xf = Ch_VaArg(interpI, ap, double);
	n = Ch_VaArg(interpI, ap, int);
	handle_ch = Ch_VaArg(interpI, ap, IdrivexyFuncHandle);
	IdrivexyFunc_chdl_funptr = (void *)handle_ch;
	if (handle_ch != NULL) {
		handle_c = (IdrivexyFuncHandle)IdrivexyFunc_chdl_funarg;
	}
	radius = Ch_VaArg(interpI, ap, double);
	trackwidth = Ch_VaArg(interpI, ap, double);
	retval = robot->drivexyToFunc(x0, xf, n, handle_c, radius, trackwidth);
	Ch_VaEnd(interpI, ap);
	return retval;
}
static double IdrivexyFunc_chdl_funarg(double x) {
	double retval;
	Ch_CallFuncByAddr(interpI, IdrivexyFunc_chdl_funptr, &retval, x);
	return retval;
}

EXPORTCH int CNXT_drivexyToPoly_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double x0;
	double xf;
	int n;
	char *poly;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x0 = Ch_VaArg(interp, ap, double);
	xf = Ch_VaArg(interp, ap, double);
	n = Ch_VaArg(interp, ap, int);
	poly = Ch_VaArg(interp, ap, char *);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyToPoly(x0, xf, n, poly, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_drivexyToPolyNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double x0;
	double xf;
	int n;
	char *poly;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x0 = Ch_VaArg(interp, ap, double);
	xf = Ch_VaArg(interp, ap, double);
	n = Ch_VaArg(interp, ap, int);
	poly = Ch_VaArg(interp, ap, char *);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyToPolyNB(x0, xf, n, poly, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_drivexyWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->drivexyWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_enableRecordDataShift_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->enableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getAccelerometerData_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double *x, *y, *z;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x = Ch_VaArg(interp, ap, double *);
	y = Ch_VaArg(interp, ap, double *);
	z = Ch_VaArg(interp, ap, double *);
	retval = robot->getAccelerometerData(*x, *y, *z);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getBatteryVoltage_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double *voltage;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	voltage = Ch_VaArg(interp, ap, double *);
	retval = robot->getBatteryVoltage(*voltage);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getLEDColorName_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	char *color;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	color = Ch_VaArg(interp, ap, char *);
	retval = robot->getLEDColorName(color);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getLEDColorRGB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int *r, *g, *b;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	r = Ch_VaArg(interp, ap, int *);
	g = Ch_VaArg(interp, ap, int *);
	b = Ch_VaArg(interp, ap, int *);
	retval = robot->getLEDColorRGB(*r, *g, *b);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double *distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	distance = Ch_VaArg(interp, ap, double *);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->getDistance(*distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getFormFactor_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int* formFactor;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	formFactor = Ch_VaArg(interp, ap, int *);
	retval = robot->getFormFactor(*formFactor);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getID_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->getID();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int id;
	double* angle;
	int numReadings;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double *);
	if(Ch_VaCount(interp, ap) == 1) {
	  numReadings = Ch_VaArg(interp, ap, int);
	  retval = robot->getJointAngle((robotJointId_t)id, *angle, numReadings);
	} else {
	  retval = robot->getJointAngle((robotJointId_t)id, *angle);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointAngleInstant_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int id;
	double* angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointAngleInstant((robotJointId_t)id, *angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointAngles_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double* angle1;
	double* angle2;
	int numReadings;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle1 = Ch_VaArg(interp, ap, double *);
	angle2 = Ch_VaArg(interp, ap, double *);
	if(Ch_VaCount(interp ,ap) == 1) {
	  numReadings = Ch_VaArg(interp, ap, int);
	  retval = robot->getJointAngles(*angle1, *angle2, numReadings);
	} else {
	  retval = robot->getJointAngles(*angle1, *angle2);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointAnglesInstant_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double* angle1;
	double* angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle1 = Ch_VaArg(interp, ap, double *);
	angle2 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointAnglesInstant(*angle1, *angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointMaxSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointMaxSpeed((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double* angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSafetyAngle(*angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double* seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	seconds = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSafetyAngleTimeout(*seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeed((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double *speed1;
	double *speed2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	speed1 = Ch_VaArg(interp, ap, double *);
	speed2 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeeds(*speed1, *speed2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeedRatio((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double *ratio1;
	double *ratio2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	ratio1 = Ch_VaArg(interp, ap, double *);
	ratio2 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeedRatios(*ratio1, *ratio2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_getxy_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double *x;
	double *y;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x = Ch_VaArg(interp, ap, double *);
	y = Ch_VaArg(interp, ap, double *);
	retval = robot->getxy(*x, *y);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_holdJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->holdJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_holdJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->holdJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_holdJointsAtExit_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_isConnected_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->isConnected();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_isMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->isMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_isNotMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->isNotMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_move_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->move(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveNB(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->moveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJoint(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointByPowerNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	int power;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	power = Ch_VaArg(interp, ap, int);
	retval = robot->moveJointByPowerNB(id, power);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t );
	retval = robot->moveJointForeverNB(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTime(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTimeNB(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTo(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPos(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPosNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveJointWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->moveJointWait(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveTo(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToNB(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPos(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPosNB(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->moveToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->moveToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_moveWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->moveWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double* time;
	double* angle;
	int num;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	time = Ch_VaArg(interp, ap, double*);
	angle = Ch_VaArg(interp, ap, double*);
	num = Ch_VaArg(interp, ap, int);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
	  shiftData = Ch_VaArg(interp, ap, int);
	  retval = robot->recordAngle(id, time, angle, num, seconds, shiftData);
	} else {
	  retval = robot->recordAngle(id, time, angle, num, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordAngleBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double** time;
	double** angle;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	time = Ch_VaArg(interp, ap, double**);
	angle = Ch_VaArg(interp, ap, double**);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
	  shiftData = Ch_VaArg(interp, ap, int);
	  retval = robot->recordAngleBegin(id, *time, *angle, seconds, shiftData);
	} else {
	  retval = robot->recordAngleBegin(id, *time, *angle, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordAngleEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	int *num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	num = Ch_VaArg(interp, ap, int* );
	retval = robot->recordAngleEnd(id, *num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordAngles_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double* time;
	double* angle1;
	double* angle2;
	int num;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	time = Ch_VaArg(interp, ap, double*);
	angle1 = Ch_VaArg(interp, ap, double*);
	angle2 = Ch_VaArg(interp, ap, double*);
	num = Ch_VaArg(interp, ap, int);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
	  shiftData = Ch_VaArg(interp, ap, int);
	  retval = robot->recordAngles(time, angle1, angle2, num, seconds, shiftData);
	} else {
	  retval = robot->recordAngles(time, angle1, angle2, num, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordAnglesBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double** time;
	double** angle1;
	double** angle2;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	time = Ch_VaArg(interp, ap, double**);
	angle1 = Ch_VaArg(interp, ap, double**);
	angle2 = Ch_VaArg(interp, ap, double**);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordAnglesBegin(*time, *angle1, *angle2, seconds, shiftData);
	}
	else {
		retval = robot->recordAnglesBegin(*time, *angle1, *angle2, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordAnglesEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	num = Ch_VaArg(interp, ap, int*);
	retval = robot->recordAnglesEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordDistanceBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double** time;
	double** angle;
	double radius;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	time = Ch_VaArg(interp, ap, double**);
	angle = Ch_VaArg(interp, ap, double**);
	radius = Ch_VaArg(interp, ap, double);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordDistanceBegin(id, *time, *angle, radius, seconds, shiftData);
	} else {
		retval = robot->recordDistanceBegin(id, *time, *angle, radius, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordDistanceEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	int *num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	num = Ch_VaArg(interp, ap, int* );
	retval = robot->recordDistanceEnd(id, *num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordDistanceOffset_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->recordDistanceOffset(distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordDistancesBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double** time;
	double** angle1;
	double** angle2;
	double radius;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	time = Ch_VaArg(interp, ap, double**);
	angle1 = Ch_VaArg(interp, ap, double**);
	angle2 = Ch_VaArg(interp, ap, double**);
	radius = Ch_VaArg(interp, ap, double);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordDistancesBegin(*time, *angle1, *angle2, radius, seconds, shiftData);
	}
	else {
		retval = robot->recordDistancesBegin(*time, *angle1, *angle2, radius, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordDistancesEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	num = Ch_VaArg(interp, ap, int*);
	retval = robot->recordDistancesEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->recordWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordxyBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double** x;
	double** y;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	x = Ch_VaArg(interp, ap, double**);
	y = Ch_VaArg(interp, ap, double**);
	seconds = Ch_VaArg(interp, ap, double);
	if (Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordxyBegin(*x, *y, seconds, shiftData);
	}
	else {
		retval = robot->recordxyBegin(*x, *y, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_recordxyEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	num = Ch_VaArg(interp, ap, int *);
	retval = robot->recordxyEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_relaxJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->relaxJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_relaxJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->relaxJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_resetToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->resetToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_resetToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->resetToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setBuzzerFrequency_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int frequency;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	frequency = Ch_VaArg(interp, ap, int);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->setBuzzerFrequency(frequency, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setBuzzerFrequencyOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->setBuzzerFrequencyOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setBuzzerFrequencyOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int frequency;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	frequency = Ch_VaArg(interp, ap, int);
	retval = robot->setBuzzerFrequencyOn(frequency);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setLEDColor_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	char *color;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	color = Ch_VaArg(interp, ap, char *);
	retval = robot->setLEDColor(color);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setLEDColorRGB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int r, g, b;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	r = Ch_VaArg(interp, ap, int);
	g = Ch_VaArg(interp, ap, int);
	b = Ch_VaArg(interp, ap, int);
	retval = robot->setLEDColorRGB(r, g, b);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngle(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngleTimeout(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeed(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double speed1;
	double speed2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	speed1 = Ch_VaArg(interp, ap, double);
	speed2 = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeeds(speed1, speed2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeedRatio(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double ratio1;
	double ratio2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	ratio1 = Ch_VaArg(interp, ap, double );
	ratio2 = Ch_VaArg(interp, ap, double );
	retval = robot->setJointSpeedRatios(ratio1, ratio2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_setSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double speed;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	speed = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->setSpeed(speed, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_systemTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double *systemTime;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	systemTime = Ch_VaArg(interp, ap, double *);
	retval = robot->systemTime(*systemTime);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_traceOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->traceOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_traceOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = robot->traceOn();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_turnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeft(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_turnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeftNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_turnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRight(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXT_turnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXT *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRightNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH void CNXTG_CNXTGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *c=new CNXTGroup();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CNXTGroup));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CNXTG_dCNXTGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CNXTGroup *);
	if(Ch_CppIsArrayElement(interp))
		c->~CNXTGroup();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CNXTG_addRobot_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *group;
	class CNXT *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CNXTGroup *);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	retval = group->addRobot(*robot);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_addRobots_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *group;
	class CNXT *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CNXTGroup *);
	robot = Ch_VaArg(interp, ap, class CNXT *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->addRobots(robot, num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_blinkLED_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double delay;
	int numBlinks;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	delay = Ch_VaArg(interp, ap, double);
	numBlinks = Ch_VaArg(interp, ap, int);
	retval = robot->blinkLED(delay, numBlinks);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->connect();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->driveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->driveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_driveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->driveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_holdJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->holdJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_holdJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->holdJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_holdJointsAtExit_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_isMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->isMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_isNotMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->isNotMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_move_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->move(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveNB(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->moveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJoint(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointByPowerNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	int power;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	power = Ch_VaArg(interp, ap, int);
	retval = robot->moveJointByPowerNB(id, power);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t );
	retval = robot->moveJointForeverNB(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTime(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTimeNB(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTo(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPos(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPosNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveJointWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->moveJointWait(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->moveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->moveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveTo(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToNB(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPos(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle1;
	double angle2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPosNB(angle1, angle2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->moveToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->moveToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_moveWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->moveWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_relaxJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->relaxJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_relaxJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->relaxJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_resetToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->resetToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_resetToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->resetToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setBuzzerFrequency_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int frequency;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	frequency = Ch_VaArg(interp, ap, int);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->setBuzzerFrequency(frequency, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setBuzzerFrequencyOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->setBuzzerFrequencyOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setBuzzerFrequencyOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int frequency;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	frequency = Ch_VaArg(interp, ap, int);
	retval = robot->setBuzzerFrequencyOn(frequency);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setLEDColor_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	char *color;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	color = Ch_VaArg(interp, ap, char *);
	retval = robot->setLEDColor(color);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setLEDColorRGB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int r, g, b;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	r = Ch_VaArg(interp, ap, int);
	g = Ch_VaArg(interp, ap, int);
	b = Ch_VaArg(interp, ap, int);
	retval = robot->setLEDColorRGB(r, g, b);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngle(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngleTimeout(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeed(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double speed1, speed2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	speed1 = Ch_VaArg(interp, ap, double);
	speed2 = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeeds(speed1, speed2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeedRatio(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	robotJointId_t id;
	double ratio1;
	double ratio2;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	ratio1 = Ch_VaArg(interp, ap, double);
	ratio2 = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeedRatios(ratio1, ratio2);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_setSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double speed;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	speed = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->setSpeed(speed, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_traceOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->traceOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_traceOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	retval = robot->traceOn();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_turnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeft(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_turnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeftNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_turnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRight(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CNXTG_turnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CNXTGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CNXTGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRightNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}
