#include "../librobosim/inc/linkbot.hpp"
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

EXPORTCH void CLinkbotI_CLinkbotI_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *c=new CLinkbotI();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotI));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CLinkbotI_dCLinkbotI_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CLinkbotI *);
	if(Ch_CppIsArrayElement(interp))
		c->~CLinkbotI();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CLinkbotI_accelJointAngleNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double acceleration;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointAngleNB((robotJointId_t)id, acceleration, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_accelJointCycloidalNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double angle;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointCycloidalNB((robotJointId_t)id, angle, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_accelJointHarmonicNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double angle;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointHarmonicNB((robotJointId_t)id, angle, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_accelJointSmoothNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double accel0;
	double accelf;
	double vmax;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	accel0 = Ch_VaArg(interp, ap, double);
	accelf = Ch_VaArg(interp, ap, double);
	vmax = Ch_VaArg(interp, ap, double);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointSmoothNB((robotJointId_t)id, accel0, accelf, vmax, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_accelJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double acceleration;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointTimeNB((robotJointId_t)id, acceleration, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_accelJointToVelocityNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double acceleration;
	double v;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	v = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointToVelocityNB((robotJointId_t)id, acceleration, v);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_accelJointToMaxSpeedNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double acceleration;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointToMaxSpeedNB((robotJointId_t)id, acceleration);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_blinkLED_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double delay;
	int numBlinks;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	delay = Ch_VaArg(interp, ap, double);
	numBlinks = Ch_VaArg(interp, ap, int);
	retval = robot->blinkLED(delay, numBlinks);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_closeGripper_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->closeGripper();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_closeGripperNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->closeGripperNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	char *name;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_delay_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double milliseconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	milliseconds = Ch_VaArg(interp, ap, double);
	retval = robot->delay(milliseconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_delaySeconds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->delaySeconds(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_disableRecordDataShift_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->disableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_disconnect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->disconnect();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveAccelCycloidalNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double radius;
	double distance;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	radius = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelCycloidalNB(radius, distance, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveAccelDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double radius;
	double acceleration;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelDistanceNB(radius, acceleration, distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveAccelHarmonicNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double radius;
	double distance;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	radius = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelHarmonicNB(radius, distance, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveAccelSmoothNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double radius;
	double accel0;
	double accelf;
	double vmax;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	radius = Ch_VaArg(interp, ap, double);
	accel0 = Ch_VaArg(interp, ap, double);
	accelf = Ch_VaArg(interp, ap, double);
	vmax = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelSmoothNB(radius, accel0, accelf, vmax, distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveAccelTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double radius;
	double acceleration;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelTimeNB(radius, acceleration, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveAccelToMaxSpeedNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double radius;
	double acceleration;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelToMaxSpeedNB(radius, acceleration);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveAccelToVelocityNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double radius;
	double acceleration;
	double v;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	v = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelToVelocityNB(radius, acceleration, v);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->driveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->driveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_driveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->driveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_drivexy_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexy(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_drivexyNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyNB(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_drivexyTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyTo(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_drivexyToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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
EXPORTCH int CLinkbotI_drivexyToFunc_chdl(void *varg) {
	ChVaList_t ap;
	class CLinkbotI *robot;
	double x0;
	double xf;
	int n;
	IdrivexyFuncHandle handle_ch, handle_c = NULL;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interpI, ap, varg);
	robot = Ch_VaArg(interpI, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_drivexyToPoly_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double x0;
	double xf;
	int n;
	char *poly;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_drivexyToPolyNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double x0;
	double xf;
	int n;
	char *poly;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_drivexyWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->drivexyWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_enableRecordDataShift_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->enableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getAccelerometerData_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double *x, *y, *z;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	x = Ch_VaArg(interp, ap, double *);
	y = Ch_VaArg(interp, ap, double *);
	z = Ch_VaArg(interp, ap, double *);
	retval = robot->getAccelerometerData(*x, *y, *z);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getBatteryVoltage_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double *voltage;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	voltage = Ch_VaArg(interp, ap, double *);
	retval = robot->getBatteryVoltage(*voltage);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getLEDColorName_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	char *color;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	color = Ch_VaArg(interp, ap, char *);
	retval = robot->getLEDColorName(color);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getLEDColorRGB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int *r, *g, *b;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	r = Ch_VaArg(interp, ap, int *);
	g = Ch_VaArg(interp, ap, int *);
	b = Ch_VaArg(interp, ap, int *);
	retval = robot->getLEDColorRGB(*r, *g, *b);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double *distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	distance = Ch_VaArg(interp, ap, double *);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->getDistance(*distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getFormFactor_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int* formFactor;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	formFactor = Ch_VaArg(interp, ap, int *);
	retval = robot->getFormFactor(*formFactor);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getID_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->getID();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double* angle;
	int numReadings;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_getJointAngleInstant_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double* angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointAngleInstant((robotJointId_t)id, *angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointAngles_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double* angle1;
	double* angle2;
	double* angle3;
	int numReadings;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle1 = Ch_VaArg(interp, ap, double *);
	angle2 = Ch_VaArg(interp, ap, double *);
	angle3 = Ch_VaArg(interp, ap, double *);
	if(Ch_VaCount(interp ,ap) == 1) {
	  numReadings = Ch_VaArg(interp, ap, int);
	  retval = robot->getJointAngles(*angle1, *angle2, *angle3, numReadings);
	} else {
	  retval = robot->getJointAngles(*angle1, *angle2, *angle3);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointAnglesInstant_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double* angle1;
	double* angle2;
	double* angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle1 = Ch_VaArg(interp, ap, double *);
	angle2 = Ch_VaArg(interp, ap, double *);
	angle3 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointAnglesInstant(*angle1, *angle2, *angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointMaxSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointMaxSpeed((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double* angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSafetyAngle(*angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double* seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	seconds = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSafetyAngleTimeout(*seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeed((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double *speed1;
	double *speed2;
	double *speed3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	speed1 = Ch_VaArg(interp, ap, double *);
	speed2 = Ch_VaArg(interp, ap, double *);
	speed3 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeeds(*speed1, *speed2, *speed3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeedRatio((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double *ratio1;
	double *ratio2;
	double *ratio3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	ratio1 = Ch_VaArg(interp, ap, double *);
	ratio2 = Ch_VaArg(interp, ap, double *);
	ratio3 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_getxy_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double *x;
	double *y;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	x = Ch_VaArg(interp, ap, double *);
	y = Ch_VaArg(interp, ap, double *);
	retval = robot->getxy(*x, *y);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_holdJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->holdJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_holdJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->holdJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_holdJointsAtExit_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_isConnected_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->isConnected();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_isMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->isMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_isNotMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->isNotMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_move_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->move(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveNB(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->moveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJoint(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointByPowerNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	int power;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	power = Ch_VaArg(interp, ap, int);
	retval = robot->moveJointByPowerNB(id, power);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t );
	retval = robot->moveJointForeverNB(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTime(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTimeNB(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTo(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPos(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPosNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveJointWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->moveJointWait(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveTo(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToNB(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPos(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPosNB(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->moveToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->moveToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_moveWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->moveWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_openGripper_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->openGripper(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_openGripperNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->openGripperNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double* time;
	double* angle;
	int num;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_recordAngleBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double** time;
	double** angle;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_recordAngleEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	int *num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	num = Ch_VaArg(interp, ap, int* );
	retval = robot->recordAngleEnd(id, *num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordAngles_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double* time;
	double* angle1;
	double* angle2;
	double* angle3;
	int num;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	time = Ch_VaArg(interp, ap, double*);
	angle1 = Ch_VaArg(interp, ap, double*);
	angle2 = Ch_VaArg(interp, ap, double*);
	angle3 = Ch_VaArg(interp, ap, double*);
	num = Ch_VaArg(interp, ap, int);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
	  shiftData = Ch_VaArg(interp, ap, int);
	  retval = robot->recordAngles(time, angle1, angle2, angle3, num, seconds, shiftData);
	} else {
	  retval = robot->recordAngles(time, angle1, angle2, angle3, num, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordAnglesBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double** time;
	double** angle1;
	double** angle2;
	double** angle3;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	time = Ch_VaArg(interp, ap, double**);
	angle1 = Ch_VaArg(interp, ap, double**);
	angle2 = Ch_VaArg(interp, ap, double**);
	angle3 = Ch_VaArg(interp, ap, double**);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordAnglesBegin(*time, *angle1, *angle2, *angle3, seconds, shiftData);
	}
	else {
		retval = robot->recordAnglesBegin(*time, *angle1, *angle2, *angle3, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordAnglesEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	num = Ch_VaArg(interp, ap, int*);
	retval = robot->recordAnglesEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordDistanceBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double** time;
	double** angle;
	double radius;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_recordDistanceEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	int *num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	num = Ch_VaArg(interp, ap, int* );
	retval = robot->recordDistanceEnd(id, *num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordDistanceOffset_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->recordDistanceOffset(distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordDistancesBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double** time;
	double** angle1;
	double** angle2;
	double** angle3;
	double radius;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	time = Ch_VaArg(interp, ap, double**);
	angle1 = Ch_VaArg(interp, ap, double**);
	angle2 = Ch_VaArg(interp, ap, double**);
	angle3 = Ch_VaArg(interp, ap, double**);
	radius = Ch_VaArg(interp, ap, double);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordDistancesBegin(*time, *angle1, *angle2, *angle3, radius, seconds, shiftData);
	}
	else {
		retval = robot->recordDistancesBegin(*time, *angle1, *angle2, *angle3, radius, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordDistancesEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	num = Ch_VaArg(interp, ap, int*);
	retval = robot->recordDistancesEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->recordWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_recordxyBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double** x;
	double** y;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
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

EXPORTCH int CLinkbotI_recordxyEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	num = Ch_VaArg(interp, ap, int *);
	retval = robot->recordxyEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_relaxJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->relaxJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_relaxJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->relaxJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_resetToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->resetToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_resetToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->resetToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setBuzzerFrequency_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int frequency;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	frequency = Ch_VaArg(interp, ap, int);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->setBuzzerFrequency(frequency, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setBuzzerFrequencyOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->setBuzzerFrequencyOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setBuzzerFrequencyOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int frequency;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	frequency = Ch_VaArg(interp, ap, int);
	retval = robot->setBuzzerFrequencyOn(frequency);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setLEDColor_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	char *color;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	color = Ch_VaArg(interp, ap, char *);
	retval = robot->setLEDColor(color);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setLEDColorRGB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int r, g, b;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	r = Ch_VaArg(interp, ap, int);
	g = Ch_VaArg(interp, ap, int);
	b = Ch_VaArg(interp, ap, int);
	retval = robot->setLEDColorRGB(r, g, b);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngle(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngleTimeout(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeed(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double speed1;
	double speed2;
	double speed3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	speed1 = Ch_VaArg(interp, ap, double);
	speed2 = Ch_VaArg(interp, ap, double);
	speed3 = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeeds(speed1, speed2, speed3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeedRatio(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double ratio1;
	double ratio2;
	double ratio3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	ratio1 = Ch_VaArg(interp, ap, double );
	ratio2 = Ch_VaArg(interp, ap, double );
	ratio3 = Ch_VaArg(interp, ap, double );
	retval = robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_setSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double speed;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	speed = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->setSpeed(speed, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_systemTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double *systemTime;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	systemTime = Ch_VaArg(interp, ap, double *);
	retval = robot->systemTime(*systemTime);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_traceOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->traceOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_traceOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = robot->traceOn();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_turnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeft(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_turnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeftNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_turnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRight(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotI_turnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotI *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRightNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH void CLIG_CLinkbotIGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *c=new CLinkbotIGroup();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotIGroup));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CLIG_dCLinkbotIGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	if(Ch_CppIsArrayElement(interp))
		c->~CLinkbotIGroup();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CLIG_accelJointAngleNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int id;
	double acceleration;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointAngleNB((robotJointId_t)id, acceleration, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_accelJointCycloidalNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int id;
	double angle;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointCycloidalNB((robotJointId_t)id, angle, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_accelJointHarmonicNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int id;
	double angle;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointHarmonicNB((robotJointId_t)id, angle, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_accelJointSmoothNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int id;
	double accel0;
	double accelf;
	double vmax;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, int);
	accel0 = Ch_VaArg(interp, ap, double);
	accelf = Ch_VaArg(interp, ap, double);
	vmax = Ch_VaArg(interp, ap, double);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointSmoothNB((robotJointId_t)id, accel0, accelf, vmax, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_accelJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int id;
	double acceleration;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointTimeNB((robotJointId_t)id, acceleration, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_accelJointToVelocityNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int id;
	double acceleration;
	double v;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	v = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointToVelocityNB((robotJointId_t)id, acceleration, v);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_accelJointToMaxSpeedNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int id;
	double acceleration;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointToMaxSpeedNB((robotJointId_t)id, acceleration);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_addRobot_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *group;
	class CLinkbotI *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = group->addRobot(*robot);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_addRobots_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *group;
	class CLinkbotI *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->addRobots(robot, num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_blinkLED_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double delay;
	int numBlinks;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	delay = Ch_VaArg(interp, ap, double);
	numBlinks = Ch_VaArg(interp, ap, int);
	retval = robot->blinkLED(delay, numBlinks);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_closeGripper_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->closeGripper();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_closeGripperNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->closeGripperNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->connect();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveAccelCycloidalNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double radius;
	double distance;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	radius = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelCycloidalNB(radius, distance, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveAccelDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double radius;
	double acceleration;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelDistanceNB(radius, acceleration, distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveAccelHarmonicNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double radius;
	double distance;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	radius = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelHarmonicNB(radius, distance, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveAccelSmoothNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double radius;
	double accel0;
	double accelf;
	double vmax;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	radius = Ch_VaArg(interp, ap, double);
	accel0 = Ch_VaArg(interp, ap, double);
	accelf = Ch_VaArg(interp, ap, double);
	vmax = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelSmoothNB(radius, accel0, accelf, vmax, distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveAccelTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double radius;
	double acceleration;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelTimeNB(radius, acceleration, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveAccelToMaxSpeedNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double radius;
	double acceleration;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelToMaxSpeedNB(radius, acceleration);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveAccelToVelocityNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double radius;
	double acceleration;
	double v;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	v = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelToVelocityNB(radius, acceleration, v);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->driveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->driveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_driveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->driveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_holdJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->holdJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_holdJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->holdJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_holdJointsAtExit_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_isMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->isMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_isNotMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->isNotMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_move_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->move(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveNB(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->moveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJoint(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointByPowerNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	int power;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	power = Ch_VaArg(interp, ap, int);
	retval = robot->moveJointByPowerNB(id, power);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t );
	retval = robot->moveJointForeverNB(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTime(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTimeNB(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTo(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPos(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPosNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveJointWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->moveJointWait(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->moveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->moveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveTo(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToNB(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPos(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle1;
	double angle2;
	double angle3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPosNB(angle1, angle2, angle3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->moveToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->moveToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_moveWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->moveWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_openGripper_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->openGripper(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_openGripperNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->openGripperNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_relaxJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->relaxJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_relaxJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->relaxJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_resetToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->resetToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_resetToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->resetToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setBuzzerFrequency_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int frequency;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	frequency = Ch_VaArg(interp, ap, int);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->setBuzzerFrequency(frequency, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setBuzzerFrequencyOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->setBuzzerFrequencyOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setBuzzerFrequencyOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int frequency;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	frequency = Ch_VaArg(interp, ap, int);
	retval = robot->setBuzzerFrequencyOn(frequency);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setLEDColor_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	char *color;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	color = Ch_VaArg(interp, ap, char *);
	retval = robot->setLEDColor(color);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setLEDColorRGB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int r, g, b;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	r = Ch_VaArg(interp, ap, int);
	g = Ch_VaArg(interp, ap, int);
	b = Ch_VaArg(interp, ap, int);
	retval = robot->setLEDColorRGB(r, g, b);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngle(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngleTimeout(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeed(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double speed1, speed2, speed3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	speed1 = Ch_VaArg(interp, ap, double);
	speed2 = Ch_VaArg(interp, ap, double);
	speed3 = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeeds(speed1, speed2, speed3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeedRatio(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	robotJointId_t id;
	double ratio1;
	double ratio2;
	double ratio3;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	ratio1 = Ch_VaArg(interp, ap, double);
	ratio2 = Ch_VaArg(interp, ap, double);
	ratio3 = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_setSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double speed;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	speed = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->setSpeed(speed, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_traceOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->traceOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_traceOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	retval = robot->traceOn();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_turnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeft(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_turnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeftNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_turnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRight(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLIG_turnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotIGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotIGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRightNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}
