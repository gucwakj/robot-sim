#include "../librobosim/linkbotsim.h"
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

EXPORTCH void CLinkbotL_CLinkbotL_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotL *c=new CLinkbotL();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotL));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CLinkbotL_dCLinkbotL_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotL *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CLinkbotL *);
	if(Ch_CppIsArrayElement(interp))
		c->~CLinkbotL();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CLinkbotL_blinkLED_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double delay;
    int numBlinks;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    delay = Ch_VaArg(interp, ap, double);
    numBlinks = Ch_VaArg(interp, ap, int);
    retval = robot->blinkLED(delay, numBlinks);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_closeGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->closeGripper();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotL *robot;
	char *name;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_delay_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double milliseconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    milliseconds = Ch_VaArg(interp, ap, double);
    retval = robot->delay(milliseconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_delaySeconds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->delaySeconds(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_disableRecordDataShift_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->disableRecordDataShift();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_disconnect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->disconnect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_enableRecordDataShift_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->enableRecordDataShift();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getAccelerometerData_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double *x, *y, *z;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double *);
    y = Ch_VaArg(interp, ap, double *);
    z = Ch_VaArg(interp, ap, double *);
    retval = robot->getAccelerometerData(*x, *y, *z);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getBatteryVoltage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double *voltage;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    voltage = Ch_VaArg(interp, ap, double *);
    retval = robot->getBatteryVoltage(*voltage);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getLEDColorName_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->getLEDColorName(color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int *r, *g, *b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    r = Ch_VaArg(interp, ap, int *);
    g = Ch_VaArg(interp, ap, int *);
    b = Ch_VaArg(interp, ap, int *);
    retval = robot->getLEDColorRGB(*r, *g, *b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	double *distance;
	double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    distance = Ch_VaArg(interp, ap, double *);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->getDistance(*distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getFormFactor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int* formFactor;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    formFactor = Ch_VaArg(interp, ap, int *);
    retval = robot->getFormFactor(*formFactor);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getID_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->getID();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int id;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointAngle((robotJointId_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointAngleAverage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int id;
    double* angle;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    if(Ch_VaCount(interp, ap) == 1) {
      numReadings = Ch_VaArg(interp, ap, int);
      retval = robot->getJointAngleAverage((robotJointId_t)id, *angle, numReadings);
    } else {
      retval = robot->getJointAngleAverage((robotJointId_t)id, *angle);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double* angle1;
    double* angle2;
    double* angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointAngles(*angle1, *angle2, *angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointAnglesAverage_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double* angle1;
    double* angle2;
    double* angle3;
    int numReadings;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    if(Ch_VaCount(interp ,ap) == 1) {
      numReadings = Ch_VaArg(interp, ap, int);
      retval = robot->getJointAnglesAverage(*angle1, *angle2, *angle3, numReadings);
    } else {
      retval = robot->getJointAnglesAverage(*angle1, *angle2, *angle3);
    }
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointMaxSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointMaxSpeed((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double* angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSafetyAngle(*angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double* seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    seconds = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSafetyAngleTimeout(*seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSpeed((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double *speed1;
    double *speed2;
    double *speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed1 = Ch_VaArg(interp, ap, double *);
    speed2 = Ch_VaArg(interp, ap, double *);
    speed3 = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSpeeds(*speed1, *speed2, *speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSpeedRatio((robotJointId_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double *ratio1;
    double *ratio2;
    double *ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    ratio1 = Ch_VaArg(interp, ap, double *);
    ratio2 = Ch_VaArg(interp, ap, double *);
    ratio3 = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getxy_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double *x;
    double *y;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double *);
    y = Ch_VaArg(interp, ap, double *);
    retval = robot->getxy(*x, *y);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->holdJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_holdJointsAtExit_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->holdJointsAtExit();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_isConnected_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->isConnected();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_isNotMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->isNotMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_jumpJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->jumpJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_jumpJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->jumpJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_jumpTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->jumpTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_jumpToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->jumpToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_line_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	double x1, y1, z1;
	double x2, y2, z2;
	int linewidth;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    retval = robot->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTime(id, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTimeNB(id, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveTime(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveTimeNB(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_movexy_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double x;
    double y;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->movexy(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_movexyNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double x;
    double y;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->movexyNB(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_movexyTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double x;
    double y;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->movexyTo(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_movexyToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double x;
    double y;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->movexyToNB(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_movexyWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->movexyWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_openGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->openGripper(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_openGripperNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->openGripperNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_point_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	double x;
	double y;
	double z;
	int pointsize;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    z = Ch_VaArg(interp, ap, double);
    pointsize = Ch_VaArg(interp, ap, int);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->point(x, y, z, pointsize, color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double* time;
    double* angle;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_recordAngleBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double** time;
    double** angle;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_recordAngleEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = robot->recordAngleEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double* time;
    double* angle1;
    double* angle2;
    double* angle3;
    int num;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_recordAnglesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_recordAnglesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int*);
    retval = robot->recordAnglesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordDistanceBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double** time;
    double** angle;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_recordDistanceEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    int *num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    num = Ch_VaArg(interp, ap, int* );
    retval = robot->recordDistanceEnd(id, *num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordDistanceOffset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double distance;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    distance = Ch_VaArg(interp, ap, double);
    retval = robot->recordDistanceOffset(distance);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordDistancesBegin_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double** time;
    double** angle1;
    double** angle2;
    double** angle3;
    double radius;
    double seconds;
    int shiftData;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_recordDistancesEnd_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;
    int *num;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int*);
    retval = robot->recordDistancesEnd(*num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->recordWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordxyBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotL *robot;
	double** x;
	double** y;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotL *);
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

EXPORTCH int CLinkbotL_recordxyEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotL *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CLinkbotL *);
	num = Ch_VaArg(interp, ap, int *);
	retval = robot->recordxyEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CLinkbotL_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->relaxJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->relaxJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->reset();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setBuzzerFrequency_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int frequency;
    double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    frequency = Ch_VaArg(interp, ap, int);
    time = Ch_VaArg(interp, ap, double);
    retval = robot->setBuzzerFrequency(frequency, time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setBuzzerFrequencyOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->setBuzzerFrequencyOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setBuzzerFrequencyOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int frequency;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    frequency = Ch_VaArg(interp, ap, int);
    retval = robot->setBuzzerFrequencyOn(frequency);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setLEDColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->setLEDColor(color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int r, g, b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    r = Ch_VaArg(interp, ap, int);
    g = Ch_VaArg(interp, ap, int);
    b = Ch_VaArg(interp, ap, int);
    retval = robot->setLEDColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSafetyAngleTimeout(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double speed1;
    double speed2;
    double speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double ratio1;
    double ratio2;
    double ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    ratio1 = Ch_VaArg(interp, ap, double );
    ratio2 = Ch_VaArg(interp, ap, double );
    ratio3 = Ch_VaArg(interp, ap, double );
    retval = robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setMotorPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    int power;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    power = Ch_VaArg(interp, ap, int);
    retval = robot->setMotorPower(id, power);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->stop();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_stopTwoJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id1;
    robotJointId_t id2;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id1 = Ch_VaArg(interp, ap, robotJointId_t);
    id2 = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->stopTwoJoints(id1, id2);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_stopThreeJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id1;
    robotJointId_t id2;
    robotJointId_t id3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id1 = Ch_VaArg(interp, ap, robotJointId_t);
    id2 = Ch_VaArg(interp, ap, robotJointId_t);
    id3 = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->stopThreeJoints(id1, id2, id3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_systemTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double *systemTime;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    systemTime = Ch_VaArg(interp, ap, double *);
    retval = robot->systemTime(*systemTime);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_text_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
	double x;
	double y;
	double z;
	char *text;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    x = Ch_VaArg(interp, ap, double);
    y = Ch_VaArg(interp, ap, double);
    z = Ch_VaArg(interp, ap, double);
    text = Ch_VaArg(interp, ap, char *);
    retval = robot->text(x, y, z, text);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_traceOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->traceOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_traceOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->traceOn();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeft(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeftNB(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnRight(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnRightNB(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH void CLLG_CLinkbotLGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotLGroup *c=new CLinkbotLGroup();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotLGroup));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CLLG_dCLinkbotLGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CLinkbotLGroup *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
	if(Ch_CppIsArrayElement(interp))
		c->~CLinkbotLGroup();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CLLG_addRobot_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *group;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    group = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = group->addRobot(*robot);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_addRobots_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *group;
    class CLinkbotL *robot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    group = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int);
    retval = group->addRobots(robot, num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_blinkLED_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double delay;
    int numBlinks;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    delay = Ch_VaArg(interp, ap, double);
    numBlinks = Ch_VaArg(interp, ap, int);
    retval = robot->blinkLED(delay, numBlinks);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_closeGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->closeGripper();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->holdJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_holdJointsAtExit_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->holdJointsAtExit();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->isMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_isNotMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->isNotMoving();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_jumpJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->jumpJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_jumpJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->jumpJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_jumpTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->jumpTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_jumpToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->jumpToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    retval = robot->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTime(id, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTimeNB(id, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->moveTime(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->moveTimeNB(seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = robot->moveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_openGripper_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->openGripper(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_openGripperNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
	double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->openGripperNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
	robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->relaxJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->relaxJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->reset();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setBuzzerFrequency_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
	int frequency;
	double time;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    frequency = Ch_VaArg(interp, ap, int);
    time = Ch_VaArg(interp, ap, double);
    retval = robot->setBuzzerFrequency(frequency, time);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setBuzzerFrequencyOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->setBuzzerFrequencyOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setBuzzerFrequencyOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
	int frequency;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    frequency = Ch_VaArg(interp, ap, int);
    retval = robot->setBuzzerFrequencyOn(frequency);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setLEDColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->setLEDColor(color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
	int r, g, b;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    r = Ch_VaArg(interp, ap, int);
    g = Ch_VaArg(interp, ap, int);
    b = Ch_VaArg(interp, ap, int);
    retval = robot->setLEDColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setJointSafetyAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSafetyAngle(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setJointSafetyAngleTimeout_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSafetyAngleTimeout(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double speed1, speed2, speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeedRatio(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    double ratio1;
    double ratio2;
    double ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    ratio1 = Ch_VaArg(interp, ap, double);
    ratio2 = Ch_VaArg(interp, ap, double);
    ratio3 = Ch_VaArg(interp, ap, double);
    retval = robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setMotorPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    int power;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    power = Ch_VaArg(interp, ap, int);
    retval = robot->setMotorPower(id, power);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double speed;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    speed = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->stop();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_stopTwoJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id1;
    robotJointId_t id2;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id1 = Ch_VaArg(interp, ap, robotJointId_t);
    id2 = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->stopTwoJoints(id1, id2);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_stopThreeJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id1;
    robotJointId_t id2;
    robotJointId_t id3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id1 = Ch_VaArg(interp, ap, robotJointId_t);
    id2 = Ch_VaArg(interp, ap, robotJointId_t);
    id3 = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->stopThreeJoints(id1, id2, id3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_traceOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->traceOff();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_traceOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->traceOn();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeft(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeftNB(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnRight(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    double radius;
    double trackwidth;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    trackwidth = Ch_VaArg(interp, ap, double);
    retval = robot->turnRightNB(angle, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}
