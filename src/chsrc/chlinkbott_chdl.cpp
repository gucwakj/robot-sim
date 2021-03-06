#include "../librobosim/linkbot.h"
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

EXPORTCH void CLinkbotT_CLinkbotT_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *c=new CLinkbotT();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotT));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CLinkbotT_dCLinkbotT_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CLinkbotT *);
	if(Ch_CppIsArrayElement(interp))
		c->~CLinkbotT();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CLinkbotT_accelJointAngleNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	int id;
	double acceleration;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointAngleNB((robotJointId_t)id, acceleration, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_accelJointCycloidalNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	int id;
	double angle;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointCycloidalNB((robotJointId_t)id, angle, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_accelJointHarmonicNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	int id;
	double angle;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointHarmonicNB((robotJointId_t)id, angle, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_accelJointSmoothNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	int id;
	double accel0;
	double accelf;
	double vmax;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	id = Ch_VaArg(interp, ap, int);
	accel0 = Ch_VaArg(interp, ap, double);
	accelf = Ch_VaArg(interp, ap, double);
	vmax = Ch_VaArg(interp, ap, double);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointSmoothNB((robotJointId_t)id, accel0, accelf, vmax, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_accelJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	int id;
	double acceleration;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointTimeNB((robotJointId_t)id, acceleration, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_accelJointToVelocityNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	int id;
	double acceleration;
	double v;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	v = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointToVelocityNB((robotJointId_t)id, acceleration, v);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_accelJointToMaxSpeedNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	int id;
	double acceleration;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointToMaxSpeedNB((robotJointId_t)id, acceleration);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_blinkLED_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double delay;
    int numBlinks;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    delay = Ch_VaArg(interp, ap, double);
    numBlinks = Ch_VaArg(interp, ap, int);
    retval = robot->blinkLED(delay, numBlinks);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_closeGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->closeGripper();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_closeGripperNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->closeGripperNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	char *name;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_delay_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double milliseconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    milliseconds = Ch_VaArg(interp, ap, double);
    retval = robot->delay(milliseconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_delaySeconds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->delaySeconds(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_disableRecordDataShift_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->disableRecordDataShift();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_disconnect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->disconnect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveAccelCycloidalNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	double radius;
	double distance;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	radius = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelCycloidalNB(radius, distance, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_driveAccelDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	double radius;
	double acceleration;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelDistanceNB(radius, acceleration, distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_driveAccelHarmonicNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	double radius;
	double distance;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	radius = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelHarmonicNB(radius, distance, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_driveAccelSmoothNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	double radius;
	double accel0;
	double accelf;
	double vmax;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	radius = Ch_VaArg(interp, ap, double);
	accel0 = Ch_VaArg(interp, ap, double);
	accelf = Ch_VaArg(interp, ap, double);
	vmax = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelSmoothNB(radius, accel0, accelf, vmax, distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_driveAccelTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	double radius;
	double acceleration;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelTimeNB(radius, acceleration, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_driveAccelToMaxSpeedNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	double radius;
	double acceleration;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelToMaxSpeedNB(radius, acceleration);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_driveAccelToVelocityNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	double radius;
	double acceleration;
	double v;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	v = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelToVelocityNB(radius, acceleration, v);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_driveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->driveBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->driveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->driveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->driveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->driveForeverNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->driveForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->driveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->driveTime(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->driveTimeNB(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_drivexy_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double x;
    double y;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->drivexy(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_drivexyNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double x;
    double y;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->drivexyNB(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_drivexyTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double x;
    double y;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->drivexyTo(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_drivexyToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double x;
    double y;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->drivexyToNB(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

typedef double (*TdrivexyFuncHandle)(double);
static ChInterp_t interpT;
static double TdrivexyFunc_chdl_funarg(double x);
static void *TdrivexyFunc_chdl_funptr;
EXPORTCH int CLinkbotT_drivexyToFunc_chdl(void *varg) {
    ChVaList_t ap;
    class CLinkbotT *robot;
	double x0;
	double xf;
	int n;
	TdrivexyFuncHandle handle_ch, handle_c = NULL;
	double radius;
	double trackwidth;
    int retval;

    Ch_VaStart(interpT, ap, varg);
    robot = Ch_VaArg(interpT, ap, class CLinkbotT *);
    x0 = Ch_VaArg(interpT, ap, double);
    xf = Ch_VaArg(interpT, ap, double);
    n = Ch_VaArg(interpT, ap, int);
	handle_ch = Ch_VaArg(interpT, ap, TdrivexyFuncHandle);
	TdrivexyFunc_chdl_funptr = (void *)handle_ch;
	if (handle_ch != NULL) {
		handle_c = (TdrivexyFuncHandle)TdrivexyFunc_chdl_funarg;
	}
    radius = Ch_VaArg(interpT, ap, double);
    trackwidth = Ch_VaArg(interpT, ap, double);
    retval = robot->drivexyToFunc(x0, xf, n, handle_c, radius, trackwidth);
    Ch_VaEnd(interpT, ap);
    return retval;
}
static double TdrivexyFunc_chdl_funarg(double x) {
	double retval;
	Ch_CallFuncByAddr(interpT, TdrivexyFunc_chdl_funptr, &retval, x);
	return retval;
}

EXPORTCH int CLinkbotT_drivexyToPoly_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double x0;
    double xf;
    int n;
	char *poly;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_drivexyToPolyNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double x0;
    double xf;
    int n;
	char *poly;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_drivexyWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->drivexyWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_enableRecordDataShift_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->enableRecordDataShift();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getAccelerometerData_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double *x, *y, *z;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x = Ch_VaArg(interp, ap, double *);
    y = Ch_VaArg(interp, ap, double *);
    z = Ch_VaArg(interp, ap, double *);
    retval = robot->getAccelerometerData(*x, *y, *z);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getBatteryVoltage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double *voltage;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    voltage = Ch_VaArg(interp, ap, double *);
    retval = robot->getBatteryVoltage(*voltage);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getLEDColorName_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->getLEDColorName(color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int *r, *g, *b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    r = Ch_VaArg(interp, ap, int *);
    g = Ch_VaArg(interp, ap, int *);
    b = Ch_VaArg(interp, ap, int *);
    retval = robot->getLEDColorRGB(*r, *g, *b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	double *distance;
	double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    distance = Ch_VaArg(interp, ap, double *);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->getDistance(*distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getFormFactor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int* formFactor;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    formFactor = Ch_VaArg(interp, ap, int *);
    retval = robot->getFormFactor(*formFactor);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getID_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->getID();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int id;
    double* angle;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_getJointAngleInstant_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int id;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointAngleInstant((robotJointId_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double* angle1;
    double* angle2;
    double* angle3;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_getJointAnglesInstant_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double* angle1;
    double* angle2;
    double* angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointAnglesInstant(*angle1, *angle2, *angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointMaxSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointMaxSpeed((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSafetyAngle(*angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double* seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    seconds = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSafetyAngleTimeout(*seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSpeed((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double *speed1;
    double *speed2;
    double *speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    speed1 = Ch_VaArg(interp, ap, double *);
    speed2 = Ch_VaArg(interp, ap, double *);
    speed3 = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSpeeds(*speed1, *speed2, *speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSpeedRatio((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double *ratio1;
    double *ratio2;
    double *ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    ratio1 = Ch_VaArg(interp, ap, double *);
    ratio2 = Ch_VaArg(interp, ap, double *);
    ratio3 = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getxy_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double *x;
    double *y;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x = Ch_VaArg(interp, ap, double *);
    y = Ch_VaArg(interp, ap, double *);
    retval = robot->getxy(*x, *y);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->holdJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_holdJointsAtExit_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->holdJointsAtExit();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_isConnected_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->isConnected();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_isNotMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->isNotMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_jumpJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->jumpJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_jumpJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->jumpJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_jumpTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->jumpTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_jumpToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->jumpToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_line_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	double x1, y1, z1;
	double x2, y2, z2;
	int linewidth;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x1 = Ch_VaArg(interp, ap, double);
    y1 = Ch_VaArg(interp, ap, double);
    z1 = Ch_VaArg(interp, ap, double);
    x2 = Ch_VaArg(interp, ap, double);
    y2 = Ch_VaArg(interp, ap, double);
    z2 = Ch_VaArg(interp, ap, double);
    linewidth = Ch_VaArg(interp, ap, int);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->line(x1, y1, z1, x2, y2, z2, linewidth, color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTime(id, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTimeNB(id, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveTime(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveTimeNB(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_openGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->openGripper(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_openGripperNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->openGripperNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_point_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	double x;
	double y;
	double z;
	int pointsize;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    z = Ch_VaArg(interp, ap, double);
    pointsize = Ch_VaArg(interp, ap, int);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->point(x, y, z, pointsize, color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_recordAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double* time;
    double* angle;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_recordAngleBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double** time;
    double** angle;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_recordAngleEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = robot->recordAngleEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_recordAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double* time;
    double* angle1;
    double* angle2;
    double* angle3;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_recordAnglesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_recordAnglesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    num = Ch_VaArg(interp, ap, int*);
    retval = robot->recordAnglesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_recordDistanceBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double** time;
    double** angle;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_recordDistanceEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = robot->recordDistanceEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_recordDistanceOffset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double distance;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    distance = Ch_VaArg(interp, ap, double);
    retval = robot->recordDistanceOffset(distance);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_recordDistancesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_recordDistancesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    num = Ch_VaArg(interp, ap, int*);
    retval = robot->recordDistancesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_recordWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->recordWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_recordxyBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	double** x;
	double** y;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
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

EXPORTCH int CLinkbotT_recordxyEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotT *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotT *);
	num = Ch_VaArg(interp, ap, int *);
	retval = robot->recordxyEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotT_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->relaxJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->relaxJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setBuzzerFrequency_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int frequency;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    frequency = Ch_VaArg(interp, ap, int);
    time = Ch_VaArg(interp, ap, double);
    retval = robot->setBuzzerFrequency(frequency, time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setBuzzerFrequencyOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->setBuzzerFrequencyOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setBuzzerFrequencyOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int frequency;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    frequency = Ch_VaArg(interp, ap, int);
    retval = robot->setBuzzerFrequencyOn(frequency);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setLEDColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->setLEDColor(color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int r, g, b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    r = Ch_VaArg(interp, ap, int);
    g = Ch_VaArg(interp, ap, int);
    b = Ch_VaArg(interp, ap, int);
    retval = robot->setLEDColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    int power;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    power = Ch_VaArg(interp, ap, int);
    retval = robot->setJointPower(id, power);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSafetyAngleTimeout(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double speed1;
    double speed2;
    double speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double ratio1;
    double ratio2;
    double ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    ratio1 = Ch_VaArg(interp, ap, double );
    ratio2 = Ch_VaArg(interp, ap, double );
    ratio3 = Ch_VaArg(interp, ap, double );
    retval = robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_systemTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double *systemTime;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    systemTime = Ch_VaArg(interp, ap, double *);
    retval = robot->systemTime(*systemTime);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_text_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	double x;
	double y;
	double z;
	char *text;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    z = Ch_VaArg(interp, ap, double);
    text = Ch_VaArg(interp, ap, char *);
    retval = robot->text(x, y, z, text);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_traceOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->traceOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_traceOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->traceOn();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeft(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeftNB(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnRight(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnRightNB(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH void CLTG_CLinkbotTGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *c=new CLinkbotTGroup();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotTGroup));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CLTG_dCLinkbotTGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	if(Ch_CppIsArrayElement(interp))
		c->~CLinkbotTGroup();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CLTG_accelJointAngleNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	int id;
	double acceleration;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointAngleNB((robotJointId_t)id, acceleration, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_accelJointCycloidalNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	int id;
	double angle;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointCycloidalNB((robotJointId_t)id, angle, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_accelJointHarmonicNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	int id;
	double angle;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointHarmonicNB((robotJointId_t)id, angle, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_accelJointSmoothNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	int id;
	double accel0;
	double accelf;
	double vmax;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	id = Ch_VaArg(interp, ap, int);
	accel0 = Ch_VaArg(interp, ap, double);
	accelf = Ch_VaArg(interp, ap, double);
	vmax = Ch_VaArg(interp, ap, double);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointSmoothNB((robotJointId_t)id, accel0, accelf, vmax, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_accelJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	int id;
	double acceleration;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointTimeNB((robotJointId_t)id, acceleration, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_accelJointToVelocityNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	int id;
	double acceleration;
	double v;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	v = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointToVelocityNB((robotJointId_t)id, acceleration, v);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_accelJointToMaxSpeedNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	int id;
	double acceleration;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	id = Ch_VaArg(interp, ap, int);
	acceleration = Ch_VaArg(interp, ap, double);
	retval = robot->accelJointToMaxSpeedNB((robotJointId_t)id, acceleration);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_addRobot_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *group;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    group = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = group->addRobot(*robot);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_addRobots_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *group;
    class CLinkbotT *robot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    group = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    num = Ch_VaArg(interp, ap, int);
    retval = group->addRobots(robot, num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_blinkLED_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double delay;
    int numBlinks;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    delay = Ch_VaArg(interp, ap, double);
    numBlinks = Ch_VaArg(interp, ap, int);
    retval = robot->blinkLED(delay, numBlinks);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_closeGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->closeGripper();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_closeGripperNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->closeGripperNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveAccelCycloidalNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	double radius;
	double distance;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	radius = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelCycloidalNB(radius, distance, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_driveAccelDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	double radius;
	double acceleration;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelDistanceNB(radius, acceleration, distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_driveAccelHarmonicNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	double radius;
	double distance;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	radius = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelHarmonicNB(radius, distance, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_driveAccelSmoothNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	double radius;
	double accel0;
	double accelf;
	double vmax;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	radius = Ch_VaArg(interp, ap, double);
	accel0 = Ch_VaArg(interp, ap, double);
	accelf = Ch_VaArg(interp, ap, double);
	vmax = Ch_VaArg(interp, ap, double);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelSmoothNB(radius, accel0, accelf, vmax, distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_driveAccelTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	double radius;
	double acceleration;
	double timeout;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	timeout = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelTimeNB(radius, acceleration, timeout);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_driveAccelToMaxSpeedNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	double radius;
	double acceleration;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelToMaxSpeedNB(radius, acceleration);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_driveAccelToVelocityNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotTGroup *robot;
	double radius;
	double acceleration;
	double v;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
	radius = Ch_VaArg(interp, ap, double);
	acceleration = Ch_VaArg(interp, ap, double);
	v = Ch_VaArg(interp, ap, double);
	retval = robot->driveAccelToVelocityNB(radius, acceleration, v);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLTG_driveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->driveBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->driveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->driveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->driveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->driveForeverNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->driveForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->driveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->driveTime(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->driveTimeNB(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->holdJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_holdJointsAtExit_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->holdJointsAtExit();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_isNotMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->isNotMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_jumpJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->jumpJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_jumpJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->jumpJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_jumpTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->jumpTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_jumpToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->jumpToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    retval = robot->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTime(id, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTimeNB(id, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->moveTime(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->moveTimeNB(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_openGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->openGripper(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_openGripperNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->openGripperNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->relaxJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->relaxJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setBuzzerFrequency_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
	int frequency;
	double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    frequency = Ch_VaArg(interp, ap, int);
    time = Ch_VaArg(interp, ap, double);
    retval = robot->setBuzzerFrequency(frequency, time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setBuzzerFrequencyOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->setBuzzerFrequencyOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setBuzzerFrequencyOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
	int frequency;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    frequency = Ch_VaArg(interp, ap, int);
    retval = robot->setBuzzerFrequencyOn(frequency);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setLEDColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->setLEDColor(color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
	int r, g, b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    r = Ch_VaArg(interp, ap, int);
    g = Ch_VaArg(interp, ap, int);
    b = Ch_VaArg(interp, ap, int);
    retval = robot->setLEDColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    int power;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    power = Ch_VaArg(interp, ap, int);
    retval = robot->setJointPower(id, power);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSafetyAngleTimeout(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double speed1, speed2, speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    double ratio1;
    double ratio2;
    double ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    ratio1 = Ch_VaArg(interp, ap, double);
    ratio2 = Ch_VaArg(interp, ap, double);
    ratio3 = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_traceOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->traceOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_traceOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->traceOn();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeft(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeftNB(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnRight(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnRightNB(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}
