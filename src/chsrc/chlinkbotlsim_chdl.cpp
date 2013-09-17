#include "../librobosim/linkbotsim.h"
#ifdef _WIN32
#include <windows.h>
#endif
#include <ch.h>

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

EXPORTCH int CLinkbotL_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->connect();
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

EXPORTCH int CLinkbotL_getColorRGB_chdl(void *varg) {
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
    retval = robot->getColorRGB(*r, *g, *b);
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

EXPORTCH int CLinkbotL_getJointState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int id;
    int * state;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    state = Ch_VaArg(interp, ap, int *);
    retval = robot->getJointState((robotJointId_t)id, (robotJointState_t&)(*state));
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


EXPORTCH int CLinkbotL_motionDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double radius;
    double distance;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->motionDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotL_motionDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double radius;
    double distance;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->motionDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotL_motionRollBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotL_motionRollForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;
    
    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotL_motionTurnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnLeftNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotL_motionTurnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnRightNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->motionWait();
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

EXPORTCH int CLinkbotL_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = robot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->moveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->moveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveForwardNB(angle);
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

EXPORTCH int CLinkbotL_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveJointContinuousTime(id, dir, seconds);
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

EXPORTCH int CLinkbotL_driveToDirect_chdl(void *varg) {
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
    retval = robot->driveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotL_driveTo_chdl(void *varg) {
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
    retval = robot->driveTo(angle1, angle2, angle3);
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

EXPORTCH int CLinkbotL_driveToDirectNB_chdl(void *varg) {
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
    retval = robot->driveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLinkbotL_driveToNB_chdl(void *varg) {
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
    retval = robot->driveToNB(angle1, angle2, angle3);
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

EXPORTCH int CLinkbotL_setColorRGB_chdl(void *varg) {
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
    retval = robot->setColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->setExitState(dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setJointMovementStateTime(id, dir, seconds);
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

EXPORTCH int CLinkbotL_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = robot->setMovementStateNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setMovementStateTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setTwoWheelRobotSpeed_chdl(void *varg) {
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
    retval = robot->setTwoWheelRobotSpeed(speed, radius);
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

EXPORTCH int CLinkbotL_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = robot->stopAllJoints();
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

EXPORTCH int CLinkbotL_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    double radius;
    double tracklength;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    tracklength = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeft(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    double radius;
    double tracklength;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    tracklength = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeftNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    double radius;
    double tracklength;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    tracklength = Ch_VaArg(interp, ap, double);
    retval = robot->turnRight(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *robot;
    double angle;
    double radius;
    double tracklength;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    tracklength = Ch_VaArg(interp, ap, double);
    retval = robot->turnRightNB(angle, radius, tracklength);
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

EXPORTCH int CLLG_driveJointToDirect_chdl(void *varg) {
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
    retval = robot->driveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_driveToDirect_chdl(void *varg) {
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
    retval = robot->driveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLLG_driveTo_chdl(void *varg) {
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
    retval = robot->driveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_driveToDirectNB_chdl(void *varg) {
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
    retval = robot->driveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLLG_driveToNB_chdl(void *varg) {
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
    retval = robot->driveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_driveJointToDirectNB_chdl(void *varg) {
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
    retval = robot->driveJointToDirectNB(id, angle);
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

EXPORTCH int CLLG_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->moveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double distance;
    double radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->moveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_moveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->moveForwardNB(angle);
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

EXPORTCH int CLLG_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointState_t exitState;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    exitState = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->setExitState(exitState);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    robotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t );
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    retval = robot->setJointMovementStateNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointId_t id;
    robotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    id = Ch_VaArg(interp, ap, robotJointId_t);
    dir = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = robot->setJointMovementStateTime(id, dir, seconds);
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

EXPORTCH int CLLG_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (robotJointState_t)Ch_VaArg(interp, ap, int);
    retval = robot->setMovementStateNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    dir1 = Ch_VaArg(interp, ap, robotJointState_t);
    dir2 = Ch_VaArg(interp, ap, robotJointState_t);
    dir3 = Ch_VaArg(interp, ap, robotJointState_t);
    seconds = Ch_VaArg(interp, ap, double );
    retval = robot->setMovementStateTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_setTwoWheelRobotSpeed_chdl(void *varg) {
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
    retval = robot->setTwoWheelRobotSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_stopAllJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->stopAllJoints();
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

EXPORTCH int CLLG_turnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    double radius;
    double tracklength;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    tracklength = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeft(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_turnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    double radius;
    double tracklength;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    tracklength = Ch_VaArg(interp, ap, double);
    retval = robot->turnLeftNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_turnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    double radius;
    double tracklength;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    tracklength = Ch_VaArg(interp, ap, double);
    retval = robot->turnRight(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_turnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    double radius;
    double tracklength;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    tracklength = Ch_VaArg(interp, ap, double);
    retval = robot->turnRightNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_motionDistance_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double distance, radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->motionDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLLG_motionDistanceNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double distance, radius;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    distance = Ch_VaArg(interp, ap, double);
    radius = Ch_VaArg(interp, ap, double);
    retval = robot->motionDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLLG_motionRollBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLLG_motionRollForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;
    
    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionRollForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLLG_motionTurnLeftNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnLeftNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}
EXPORTCH int CLLG_motionTurnRightNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    angle = Ch_VaArg(interp, ap, double);
    retval = robot->motionTurnRightNB(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLLG_motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotLGroup *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    robot = Ch_VaArg(interp, ap, class CLinkbotLGroup *);
    retval = robot->motionWait();
    Ch_VaEnd(interp, ap);
    return retval;
}
