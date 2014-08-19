#include "../librobosim/inc/mobot.hpp"
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

EXPORTCH void CMobot_CMobot_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *c=new CMobot();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CMobot));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CMobot_dCMobot_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CMobot *);
	if(Ch_CppIsArrayElement(interp))
		c->~CMobot();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CMobot_blinkLED_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double delay;
	int numBlinks;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	delay = Ch_VaArg(interp, ap, double);
	numBlinks = Ch_VaArg(interp, ap, int);
	retval = robot->blinkLED(delay, numBlinks);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	char *name;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
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

EXPORTCH int CMobot_delay_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double milliseconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	milliseconds = Ch_VaArg(interp, ap, double);
	retval = robot->delay(milliseconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_delaySeconds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->delaySeconds(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_disableRecordDataShift_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->disableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_disconnect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->disconnect();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->driveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->driveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_driveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->driveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_drivexy_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexy(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_drivexyNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyNB(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_drivexyTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyTo(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_drivexyToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double x;
	double y;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	x = Ch_VaArg(interp, ap, double);
	y = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->drivexyToNB(x, y, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_drivexyWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->drivexyWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_enableRecordDataShift_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->enableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double *distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	distance = Ch_VaArg(interp, ap, double *);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->getDistance(*distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getFormFactor_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int* formFactor;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	formFactor = Ch_VaArg(interp, ap, int *);
	retval = robot->getFormFactor(*formFactor);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int id;
	double* angle;
	int numReadings;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
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

EXPORTCH int CMobot_getJointAngleInstant_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int id;
	double *angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, int);
	angle = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointAngleInstant((robotJointId_t)id, *angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointAngles_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double* angle1;
	double* angle2;
	double* angle3;
	double* angle4;
	int numReadings;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle1 = Ch_VaArg(interp, ap, double *);
	angle2 = Ch_VaArg(interp, ap, double *);
	angle3 = Ch_VaArg(interp, ap, double *);
	angle4 = Ch_VaArg(interp, ap, double *);
	if(Ch_VaCount(interp ,ap) == 1) {
	  numReadings = Ch_VaArg(interp, ap, int);
	  retval = robot->getJointAngles(*angle1, *angle2, *angle3, *angle4, numReadings);
	} else {
	  retval = robot->getJointAngles(*angle1, *angle2, *angle3, *angle4);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointAnglesInstant_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double* angle1;
	double* angle2;
	double* angle3;
	double* angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle1 = Ch_VaArg(interp, ap, double *);
	angle2 = Ch_VaArg(interp, ap, double *);
	angle3 = Ch_VaArg(interp, ap, double *);
	angle4 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointAnglesInstant(*angle1, *angle2, *angle3, *angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointMaxSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointMaxSpeed((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double* angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSafetyAngle(*angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double* seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	seconds = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSafetyAngleTimeout(*seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeed((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double *speed1;
	double *speed2;
	double *speed3;
	double *speed4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	speed1 = Ch_VaArg(interp, ap, double *);
	speed2 = Ch_VaArg(interp, ap, double *);
	speed3 = Ch_VaArg(interp, ap, double *);
	speed4 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeeds(*speed1, *speed2, *speed3, *speed4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int id;
	double *speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, int);
	speed = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeedRatio((robotJointId_t)id, *speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double *ratio1;
	double *ratio2;
	double *ratio3;
	double *ratio4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	ratio1 = Ch_VaArg(interp, ap, double *);
	ratio2 = Ch_VaArg(interp, ap, double *);
	ratio3 = Ch_VaArg(interp, ap, double *);
	ratio4 = Ch_VaArg(interp, ap, double *);
	retval = robot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3, *ratio4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_getxy_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double *x;
	double *y;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	x = Ch_VaArg(interp, ap, double *);
	y = Ch_VaArg(interp, ap, double *);
	retval = robot->getxy(*x, *y);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_holdJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->holdJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_holdJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->holdJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_holdJointsAtExit_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_isConnected_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->isConnected();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_isMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->isMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_isNotMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->isNotMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionArch_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionArch(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionArchNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionArchNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double radius;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->motionDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double radius;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->motionDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionInchwormLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int);
	retval = robot->motionInchwormLeft(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionInchwormLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int);
	retval = robot->motionInchwormLeftNB(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionInchwormRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int);
	retval = robot->motionInchwormRight(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionInchwormRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int);
	retval = robot->motionInchwormRightNB(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionRollBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionRollBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionRollBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionRollBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionRollForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionRollForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionRollForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionRollForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionStand_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->motionStand();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionStandNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->motionStandNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionTumbleLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int);
	retval = robot->motionTumbleLeft(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionTumbleLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int);
	retval = robot->motionTumbleLeftNB(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionTumbleRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int);
	retval = robot->motionTumbleRight(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionTumbleRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int);
	retval = robot->motionTumbleRightNB(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionTurnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionTurnLeft(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionTurnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionTurnLeftNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionTurnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionTurnRight(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionTurnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->motionTurnRightNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionUnstand_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->motionUnstand();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionUnstandNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->motionUnstandNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_motionWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->motionWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_move_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = robot->move(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = robot->moveNB(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->moveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJoint(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointByPower_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	int power;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	power = Ch_VaArg(interp, ap, int);
	retval = robot->moveJointByPowerNB(id, power);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t );
	retval = robot->moveJointForeverNB(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTime(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTimeNB(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTo(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPos(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPosNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveJointWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->moveJointWait(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = robot->moveTo(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToNB(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPos(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = robot->moveToByTrackPosNB(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->moveToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->moveToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_moveWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->moveWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double* time;
	double* angle;
	int num;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
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

EXPORTCH int CMobot_recordAngleBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double **time;
	double **angle;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
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

EXPORTCH int CMobot_recordAngleEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	int *num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	num = Ch_VaArg(interp, ap, int* );
	retval = robot->recordAngleEnd(id, *num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordAngles_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double* time;
	double* angle1;
	double* angle2;
	double* angle3;
	double* angle4;
	int num;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	time = Ch_VaArg(interp, ap, double*);
	angle1 = Ch_VaArg(interp, ap, double*);
	angle2 = Ch_VaArg(interp, ap, double*);
	angle3 = Ch_VaArg(interp, ap, double*);
	angle4 = Ch_VaArg(interp, ap, double*);
	num = Ch_VaArg(interp, ap, int);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordAngles(time, angle1, angle2, angle3, angle4, num, seconds, shiftData);
	} else {
		retval = robot->recordAngles(time, angle1, angle2, angle3, angle4, num, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordAnglesBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double** time;
	double** angle1;
	double** angle2;
	double** angle3;
	double** angle4;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	time = Ch_VaArg(interp, ap, double**);
	angle1 = Ch_VaArg(interp, ap, double**);
	angle2 = Ch_VaArg(interp, ap, double**);
	angle3 = Ch_VaArg(interp, ap, double**);
	angle4 = Ch_VaArg(interp, ap, double**);
	seconds = Ch_VaArg(interp, ap, double);
	if (Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordAnglesBegin(*time, *angle1, *angle2, *angle3, *angle4, seconds, shiftData);
	}
	else {
		retval = robot->recordAnglesBegin(*time, *angle1, *angle2, *angle3, *angle4, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordAnglesEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int*);
	retval = robot->recordAnglesEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordDistanceBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double** time;
	double** angle;
	double radius;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
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

EXPORTCH int CMobot_recordDistanceEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	int *num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	num = Ch_VaArg(interp, ap, int* );
	retval = robot->recordDistanceEnd(id, *num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordDistanceOffset_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double distance;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	distance = Ch_VaArg(interp, ap, double);
	retval = robot->recordDistanceOffset(distance);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordDistancesBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double** time;
	double** angle1;
	double** angle2;
	double** angle3;
	double** angle4;
	double radius;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	time = Ch_VaArg(interp, ap, double**);
	angle1 = Ch_VaArg(interp, ap, double**);
	angle2 = Ch_VaArg(interp, ap, double**);
	angle3 = Ch_VaArg(interp, ap, double**);
	angle4 = Ch_VaArg(interp, ap, double**);
	radius = Ch_VaArg(interp, ap, double);
	seconds = Ch_VaArg(interp, ap, double);
	if(Ch_VaCount(interp, ap) == 1) {
		shiftData = Ch_VaArg(interp, ap, int);
		retval = robot->recordDistancesBegin(*time, *angle1, *angle2, *angle3, *angle4, radius, seconds, shiftData);
	}
	else {
		retval = robot->recordDistancesBegin(*time, *angle1, *angle2, *angle3, *angle4, radius, seconds);
	}
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordDistancesEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int*);
	retval = robot->recordDistancesEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->recordWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_recordxyBegin_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double** x;
	double** y;
	double seconds;
	int shiftData;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
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

EXPORTCH int CMobot_recordxyEnd_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;
	int *num;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	num = Ch_VaArg(interp, ap, int *);
	retval = robot->recordxyEnd(*num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_relaxJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->relaxJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_relaxJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->relaxJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_resetToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->resetToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_resetToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->resetToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_setJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngle(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_setJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngleTimeout(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_setJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeed(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_setJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double speed1;
	double speed2;
	double speed3;
	double speed4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	speed1 = Ch_VaArg(interp, ap, double);
	speed2 = Ch_VaArg(interp, ap, double);
	speed3 = Ch_VaArg(interp, ap, double);
	speed4 = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeeds(speed1, speed2, speed3, speed4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_setJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeedRatio(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_setJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double ratio1;
	double ratio2;
	double ratio3;
	double ratio4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	ratio1 = Ch_VaArg(interp, ap, double );
	ratio2 = Ch_VaArg(interp, ap, double );
	ratio3 = Ch_VaArg(interp, ap, double );
	ratio4 = Ch_VaArg(interp, ap, double );
	retval = robot->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_setSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double speed;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	speed = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->setSpeed(speed, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_systemTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double *systemTime;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	systemTime = Ch_VaArg(interp, ap, double *);
	retval = robot->systemTime(*systemTime);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_traceOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->traceOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_traceOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	retval = robot->traceOn();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_turnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeft(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_turnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeftNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_turnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRight(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMobot_turnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobot *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobot *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRightNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH void CMG_CMobotGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *c=new CMobotGroup();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(CMobotGroup));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void CMG_dCMobotGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class CMobotGroup *);
	if(Ch_CppIsArrayElement(interp))
		c->~CMobotGroup();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int CMG_addRobot_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	class CMobot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	robot = Ch_VaArg(interp, ap, class CMobot*);
	retval = group->addRobot(*robot);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_addRobots_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	class CMobot *robot;
	int numRobots;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	robot = Ch_VaArg(interp, ap, class CMobot*);
	numRobots = Ch_VaArg(interp, ap, int);
	retval = group->addRobots(robot, numRobots);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_blinkLED_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double delay;
	int numBlinks;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	delay = Ch_VaArg(interp, ap, double);
	numBlinks = Ch_VaArg(interp, ap, int);
	retval = robot->blinkLED(delay, numBlinks);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->connect();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = robot->driveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->driveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_driveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->driveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_holdJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->holdJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_holdJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = robot->holdJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_holdJointsAtExit_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_isMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->isMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_isNotMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->isNotMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionArch_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionArch(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionArchNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionArchNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double distance, radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = group->motionDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double distance, radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = group->motionDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionInchwormLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->motionInchwormLeft(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionInchwormLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->motionInchwormLeftNB(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionInchwormRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->motionInchwormRight(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionInchwormRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->motionInchwormRightNB(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionRollBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionRollBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionRollBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionRollBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionRollForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionRollForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionRollForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionRollForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionSkinny_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionSkinny(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionSkinnyNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionSkinnyNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionStand_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->motionStand();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionStandNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->motionStandNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionTurnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionTurnLeft(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionTurnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionTurnLeftNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionTurnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionTurnRight(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionTurnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->motionTurnRightNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionTumbleLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->motionTumbleLeft(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionTumbleLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->motionTumbleLeftNB(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionTumbleRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->motionTumbleRight(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionTumbleRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->motionTumbleRightNB(num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionUnstand_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->motionUnstand();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionUnstandNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->motionUnstandNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_motionWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->motionWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_move_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = group->move(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = group->moveNB(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->moveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->moveJoint(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->moveJointNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointByPowerNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	robotJointId_t id;
	int power;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	power = Ch_VaArg(interp, ap, int);
	retval = robot->moveJointByPowerNB(id, power);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t );
	retval = group->moveJointForeverNB(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = group->moveJointTime(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = group->moveJointTimeNB(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->moveJointTo(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->moveJointToNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->moveJointToByTrackPos(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->moveJointToByTrackPosNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveJointWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = group->moveJointWait(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = group->moveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = group->moveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = group->moveTo(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = group->moveToNB(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = group->moveToByTrackPos(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle1;
	double angle2;
	double angle3;
	double angle4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle1 = Ch_VaArg(interp, ap, double);
	angle2 = Ch_VaArg(interp, ap, double);
	angle3 = Ch_VaArg(interp, ap, double);
	angle4 = Ch_VaArg(interp, ap, double);
	retval = group->moveToByTrackPosNB(angle1, angle2, angle3, angle4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->moveToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->moveToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_moveWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->moveWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_relaxJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->relaxJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_relaxJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->relaxJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_resetToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->resetToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_resetToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	retval = group->resetToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_setJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->setJointSafetyAngle(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_setJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = group->setJointSafetyAngleTimeout(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_setJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = group->setJointSpeed(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_setJointSpeeds_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double speed1, speed2, speed3, speed4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	speed1 = Ch_VaArg(interp, ap, double);
	speed2 = Ch_VaArg(interp, ap, double);
	speed3 = Ch_VaArg(interp, ap, double);
	speed4 = Ch_VaArg(interp, ap, double);
	retval = group->setJointSpeeds(speed1, speed2, speed3, speed4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_setJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = group->setJointSpeedRatio(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_setJointSpeedRatios_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	robotJointId_t id;
	double ratio1;
	double ratio2;
	double ratio3;
	double ratio4;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	ratio1 = Ch_VaArg(interp, ap, double);
	ratio2 = Ch_VaArg(interp, ap, double);
	ratio3 = Ch_VaArg(interp, ap, double);
	ratio4 = Ch_VaArg(interp, ap, double);
	retval = group->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_setSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double speed;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	speed = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = group->setSpeed(speed, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_turnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = group->turnLeft(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_turnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = group->turnLeftNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_turnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = group->turnRight(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int CMG_turnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class CMobotGroup *group;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class CMobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = group->turnRightNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

