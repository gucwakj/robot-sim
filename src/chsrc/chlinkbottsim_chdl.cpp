#include "../librobosim/linkbotsim.h"
#ifdef _WIN32
#include <windows.h>
#endif
#include <ch.h>

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

EXPORTCH int CLinkbotT_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

	int embed = 0;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->connect(NULL, !embed, 1);
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

EXPORTCH int CLinkbotT_getColorName_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->getColorName(color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getColorRGB_chdl(void *varg) {
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
    retval = robot->getColorRGB(*r, *g, *b);
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
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointAngle((robotJointId_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointAngleAverage_chdl(void *varg) {
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
      retval = robot->getJointAngleAverage((robotJointId_t)id, *angle, numReadings);
    } else {
      retval = robot->getJointAngleAverage((robotJointId_t)id, *angle);
    }
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
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle1 = Ch_VaArg(interp, ap, double *);
    angle2 = Ch_VaArg(interp, ap, double *);
    angle3 = Ch_VaArg(interp, ap, double *);
    retval = robot->getJointAngles(*angle1, *angle2, *angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_getJointAnglesAverage_chdl(void *varg) {
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
      retval = robot->getJointAnglesAverage(*angle1, *angle2, *angle3, numReadings);
    } else {
      retval = robot->getJointAnglesAverage(*angle1, *angle2, *angle3);
    }
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

EXPORTCH int CLinkbotT_getJointState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int id;
    int * state;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, int);
    state = Ch_VaArg(interp, ap, int *);
    retval = robot->getJointState((robotJointId_t)id, (robotJointState_t&)(*state));
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

EXPORTCH int CLinkbotT_motionDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double radius;
    double distance;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->motionDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotT_motionDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double radius;
    double distance;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->motionDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotT_motionRollBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotT_motionRollForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;
    
    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotT_motionTurnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnLeftNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotT_motionTurnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnRightNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->motionWait();
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

EXPORTCH int CLinkbotT_moveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveDistance_chdl(void *varg) {
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
    retval = robot->moveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveDistanceNB_chdl(void *varg) {
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
    retval = robot->moveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointContinuousTime(id, dir, seconds);
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

EXPORTCH int CLinkbotT_moveJointToDirect_chdl(void *varg) {
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
    retval = robot->moveJointToDirect(id, angle);
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

EXPORTCH int CLinkbotT_moveJointToDirectNB_chdl(void *varg) {
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
    retval = robot->moveJointToDirectNB(id, angle);
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

EXPORTCH int CLinkbotT_movePoly_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    double x0;
    double xf;
    int n;
	char *poly;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    x0 = Ch_VaArg(interp, ap, double);
    xf = Ch_VaArg(interp, ap, double);
    n = Ch_VaArg(interp, ap, int);
    poly = Ch_VaArg(interp, ap, char *);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->movePoly(x0, xf, n, poly, radius);
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

EXPORTCH int CLinkbotT_moveToDirect_chdl(void *varg) {
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
    retval = robot->moveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveToDirect_chdl(void *varg) {
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
    retval = robot->driveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotT_driveTo_chdl(void *varg) {
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
    retval = robot->driveTo(angle1, angle2, angle3);
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

EXPORTCH int CLinkbotT_moveToDirectNB_chdl(void *varg) {
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
    retval = robot->moveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_driveToDirectNB_chdl(void *varg) {
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
    retval = robot->driveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotT_driveToNB_chdl(void *varg) {
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
    retval = robot->driveToNB(angle1, angle2, angle3);
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

EXPORTCH int CLinkbotT_movexy_chdl(void *varg) {
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
    retval = robot->movexy(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_movexyNB_chdl(void *varg) {
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
    retval = robot->movexyNB(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

typedef double (*TmovexyFuncHandle)(double);
static ChInterp_t interpT;
static double TmovexyFunc_chdl_funarg(double x);
static void *TmovexyFunc_chdl_funptr;
EXPORTCH int CLinkbotT_movexyFunc_chdl(void *varg) {
    ChVaList_t ap;
    class CLinkbotT *robot;
	double x0;
	double xf;
	int n;
	TmovexyFuncHandle handle_ch, handle_c = NULL;
	double radius;
    int retval;

    Ch_VaStart(interpT, ap, varg);
    robot = Ch_VaArg(interpT, ap, class CLinkbotT *);
    x0 = Ch_VaArg(interpT, ap, double);
    xf = Ch_VaArg(interpT, ap, double);
    n = Ch_VaArg(interpT, ap, int);
	handle_ch = Ch_VaArg(interpT, ap, TmovexyFuncHandle);
	TmovexyFunc_chdl_funptr = (void *)handle_ch;
	if (handle_ch != NULL) {
		handle_c = (TmovexyFuncHandle)TmovexyFunc_chdl_funarg;
	}
    radius = Ch_VaArg(interpT, ap, double);
    retval = robot->movexyFunc(x0, xf, n, handle_c, radius);
    Ch_VaEnd(interpT, ap);
    return retval;
}
static double TmovexyFunc_chdl_funarg(double x) {
	double retval;
	Ch_CallFuncByAddr(interpT, TmovexyFunc_chdl_funptr, &retval, x);
	return retval;
}

typedef double (*TmovexyFuncNBHandle)(double);
static ChInterp_t interpTNB;
static double TmovexyFuncNB_chdl_funarg(double x);
static void *TmovexyFuncNB_chdl_funptr;
EXPORTCH int CLinkbotT_movexyFuncNB_chdl(void *varg) {
    ChVaList_t ap;
    class CLinkbotT *robot;
	double x0;
	double xf;
	int n;
	TmovexyFuncNBHandle handle_ch, handle_c = NULL;
	double radius;
    int retval;

    Ch_VaStart(interpTNB, ap, varg);
    robot = Ch_VaArg(interpTNB, ap, class CLinkbotT *);
    x0 = Ch_VaArg(interpTNB, ap, double);
    xf = Ch_VaArg(interpTNB, ap, double);
    n = Ch_VaArg(interpTNB, ap, int);
	handle_ch = Ch_VaArg(interpTNB, ap, TmovexyFuncNBHandle);
	TmovexyFuncNB_chdl_funptr = (void *)handle_ch;
	if (handle_ch != NULL) {
		handle_c = (TmovexyFuncNBHandle)TmovexyFuncNB_chdl_funarg;
	}
    radius = Ch_VaArg(interpTNB, ap, double);
    retval = robot->movexyFuncNB(x0, xf, n, handle_c, radius);
    Ch_VaEnd(interpTNB, ap);
    return retval;
}
static double TmovexyFuncNB_chdl_funarg(double x) {
	double retval;
	Ch_CallFuncByAddr(interpTNB, TmovexyFuncNB_chdl_funptr, &retval, x);
	return retval;
}

EXPORTCH int CLinkbotT_movexyTo_chdl(void *varg) {
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
    retval = robot->movexyTo(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_movexyToNB_chdl(void *varg) {
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
    retval = robot->movexyToNB(x, y, radius, trackwidth);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_movexyWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->movexyWait();
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

EXPORTCH int CLinkbotT_reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->reset();
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

EXPORTCH int CLinkbotT_setColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
	char *color;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    color = Ch_VaArg(interp, ap, char *);
    retval = robot->setColor(color);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setColorRGB_chdl(void *varg) {
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
    retval = robot->setColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->setExitState(dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setJointMovementStateTime(id, dir, seconds);
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

EXPORTCH int CLinkbotT_setMotorPower_chdl(void *varg) {
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
    retval = robot->setMotorPower(id, power);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = robot->setMovementStateNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setMovementStateTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_setTwoWheelRobotSpeed_chdl(void *varg) {
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
    retval = robot->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->stopOneJoint(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->stopAllJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotT_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotT *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotT *);
    retval = robot->stop();
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

EXPORTCH int CLTG_driveJointToDirect_chdl(void *varg) {
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
    retval = robot->driveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveToDirect_chdl(void *varg) {
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
    retval = robot->driveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLTG_driveTo_chdl(void *varg) {
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
    retval = robot->driveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveToDirectNB_chdl(void *varg) {
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
    retval = robot->driveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLTG_driveToNB_chdl(void *varg) {
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
    retval = robot->driveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_driveJointToDirectNB_chdl(void *varg) {
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
    retval = robot->driveJointToDirectNB(id, angle);
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

EXPORTCH int CLTG_moveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveDistance_chdl(void *varg) {
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
    retval = robot->moveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveDistanceNB_chdl(void *varg) {
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
    retval = robot->moveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_moveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveForwardNB(angle);
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

EXPORTCH int CLTG_moveJointToDirect_chdl(void *varg) {
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
    retval = robot->moveJointToDirect(id, angle);
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

EXPORTCH int CLTG_moveJointToDirectNB_chdl(void *varg) {
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
    retval = robot->moveJointToDirectNB(id, angle);
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

EXPORTCH int CLTG_moveToDirect_chdl(void *varg) {
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
    retval = robot->moveToDirect(angle1, angle2, angle3);
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

EXPORTCH int CLTG_moveToDirectNB_chdl(void *varg) {
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
    retval = robot->moveToDirectNB(angle1, angle2, angle3);
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

EXPORTCH int CLTG_reset_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->reset();
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

EXPORTCH int CLTG_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointState_t exitState;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    exitState = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->setExitState(exitState);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setJointMovementStateTime(id, dir, seconds);
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

EXPORTCH int CLTG_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = robot->setMovementStateNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->setMovementStateTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_setTwoWheelRobotSpeed_chdl(void *varg) {
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
    retval = robot->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->stopAllJoints();
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLTG_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    robotJointId_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    retval = robot->stopOneJoint(id);
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

EXPORTCH int CLTG_motionDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double distance, radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->motionDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLTG_motionDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double distance, radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->motionDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLTG_motionRollBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLTG_motionRollForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;
    
    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLTG_motionTurnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnLeftNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLTG_motionTurnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnRightNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLTG_motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotTGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotTGroup *);
    retval = robot->motionWait();
    Ch_VaEnd(interp, ap);
    return retval;
}
