#include "../librobosim/inc/robot.hpp"
#ifdef _WIN32
#include <windows.h>
#endif
#include <ch.h>

EXPORTCH void RG_RobotGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *c = new RobotGroup();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(RobotGroup));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void RG_dRobotGroup_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class RobotGroup *);
	if(Ch_CppIsArrayElement(interp))
		c->~RobotGroup();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}

EXPORTCH int RG_addRobot_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *group;
	class Robot *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class RobotGroup *);
	robot = Ch_VaArg(interp, ap, class Robot *);
	retval = group->addRobot(*robot);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_addRobots_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *group;
	class Robot *robot;
	int num;
	int retval;

	Ch_VaStart(interp, ap, varg);
	group = Ch_VaArg(interp, ap, class RobotGroup *);
	robot = Ch_VaArg(interp, ap, class Robot *);
	num = Ch_VaArg(interp, ap, int);
	retval = group->addRobots(robot, num);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_blinkLED_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double delay;
	int numBlinks;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	delay = Ch_VaArg(interp, ap, double);
	numBlinks = Ch_VaArg(interp, ap, int);
	retval = robot->blinkLED(delay, numBlinks);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_connect_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->connect();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveBackward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveBackwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveBackwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveDistance_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistance(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveDistanceNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double distance;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	distance = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->driveDistanceNB(distance, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->driveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveForward_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForward(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveForwardNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->driveForwardNB(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->driveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_driveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->driveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_holdJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->holdJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_holdJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->holdJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_holdJointsAtExit_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_isMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->isMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_isNotMoving_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->isNotMoving();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->moveForeverNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJoint(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointByPowerNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	int power;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	power = Ch_VaArg(interp, ap, int);
	retval = robot->moveJointByPowerNB(id, power);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointForeverNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t );
	retval = robot->moveJointForeverNB(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTime(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	seconds = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTimeNB(id, seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointTo_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointTo(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointToNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointToByTrackPos_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPos(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointToByTrackPosNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->moveJointToByTrackPosNB(id, angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveJointWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->moveJointWait(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveTime_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->moveTime(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveTimeNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double seconds;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	seconds = Ch_VaArg(interp, ap, double );
	retval = robot->moveTimeNB(seconds);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->moveToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->moveToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_moveWait_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->moveWait();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_relaxJoint_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	retval = robot->relaxJoint(id);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_relaxJoints_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->relaxJoints();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_resetToZero_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->resetToZero();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_resetToZeroNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->resetToZeroNB();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setBuzzerFrequency_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int frequency;
	double time;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	frequency = Ch_VaArg(interp, ap, int);
	time = Ch_VaArg(interp, ap, double);
	retval = robot->setBuzzerFrequency(frequency, time);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setBuzzerFrequencyOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->setBuzzerFrequencyOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setBuzzerFrequencyOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int frequency;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	frequency = Ch_VaArg(interp, ap, int);
	retval = robot->setBuzzerFrequencyOn(frequency);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setLEDColor_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	char *color;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	color = Ch_VaArg(interp, ap, char *);
	retval = robot->setLEDColor(color);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setLEDColorRGB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int r, g, b;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	r = Ch_VaArg(interp, ap, int);
	g = Ch_VaArg(interp, ap, int);
	b = Ch_VaArg(interp, ap, int);
	retval = robot->setLEDColorRGB(r, g, b);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngle(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSafetyAngleTimeout(angle);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setJointSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeed(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setJointSpeedRatio_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	robotJointId_t id;
	double speed;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	id = Ch_VaArg(interp, ap, robotJointId_t);
	speed = Ch_VaArg(interp, ap, double);
	retval = robot->setJointSpeedRatio(id, speed);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_setSpeed_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double speed;
	double radius;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	speed = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	retval = robot->setSpeed(speed, radius);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_traceOff_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->traceOff();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_traceOn_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	retval = robot->traceOn();
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_turnLeft_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeft(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_turnLeftNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnLeftNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_turnRight_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRight(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

EXPORTCH int RG_turnRightNB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RobotGroup *robot;
	double angle;
	double radius;
	double trackwidth;
	int retval;

	Ch_VaStart(interp, ap, varg);
	robot = Ch_VaArg(interp, ap, class RobotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	radius = Ch_VaArg(interp, ap, double);
	trackwidth = Ch_VaArg(interp, ap, double);
	retval = robot->turnRightNB(angle, radius, trackwidth);
	Ch_VaEnd(interp, ap);
	return retval;
}

