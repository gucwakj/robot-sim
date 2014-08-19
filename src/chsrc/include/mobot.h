#ifndef MOBOT_H_
#define MOBOT_H_

#include <array.h>
#include "macros.hpp"

#define NUM_DOF 4

#ifndef MACROS
#define MACROS
#define DLLIMPORT
#pragma package <chrobosim>
#define dDOUBLE
#define ENABLE_GRAPHICS
#define ROBOT_NEUTRAL 0
#define ROBOT_HOLD 1
#define ROBOT_POSITIVE 2
#define ROBOT_NEGATIVE 3
#define ROBOT_FORWARD 2
#define ROBOT_BACKWARD 3
#define ROBOT_JOINT1 0
#define ROBOT_JOINT2 1
#define ROBOT_JOINT3 2
#define ROBOT_JOINT4 3
#endif // MACROS

#include "robot.h"

class DLLIMPORT CMobot {
	public:
		CMobot();
		~CMobot();

		int blinkLED(double delay, int num);
		int connect(...);
		int delay(double milliseconds);
		int delaySeconds(double seconds);
		int disableRecordDataShift(void);
		int disconnect(void);
		int driveBackward(double angle);
		int driveBackwardNB(double angle);
		int driveDistance(double distance, double radius);
		int driveDistanceNB(double distance, double radius);
		int driveForeverNB(void);
		int driveForward(double angle);
		int driveForwardNB(double angle);
		int driveTime(double seconds);
		int driveTimeNB(double seconds);
		int drivexy(double x, double y, double radius, double trackwidth);
		int drivexyNB(double x, double y, double radius, double trackwidth);
		int drivexyTo(double x, double y, double radius, double trackwidth);
		int drivexyToNB(double x, double y, double radius, double trackwidth);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getJointAngle(robotJointId_t id, double &angle, ...);
		int getJointAngleAverage(robotJointId_t id, double &angle, ...);		// deprecated
		int getJointAngleInstant(robotJointId_t id, double &angle);
		int getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, ...);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, double &angle4, ...);		// deprecated
		int getJointAnglesInstant(double &angle1, double &angle2, double &angle3, double &angle4);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4);
		int getxy(double &x, double &y);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);		// deprecated
		int jumpJointToNB(robotJointId_t id, double angle);		// deprecated
		int jumpTo(double angle1, double angle2, double angle3, double angle4);		// deprecated
		int jumpToNB(double angle1, double angle2, double angle3, double angle4);		// deprecated
		int line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color);
		int motionArch(double angle);
		int motionArchNB(double angle);
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionInchwormLeft(int num);
		int motionInchwormLeftNB(int num);
		int motionInchwormRight(int num);
		int motionInchwormRightNB(int num);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionSkinny(double angle);
		int motionSkinnyNB(double angle);
		int motionStand(void);
		int motionStandNB(void);
		int motionTumbleLeft(int num);
		int motionTumbleLeftNB(int num);
		int motionTumbleRight(int num);
		int motionTumbleRightNB(int num);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionUnstand(void);
		int motionUnstandNB(void);
		int motionWait(void);
		int move(double angle1, double angle2, double angle3, double angle4);
		int moveNB(double angle1, double angle2, double angle3, double angle4);
		int moveForeverNB(void);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointByPowerNB(robotJointId_t id, int power);
		int moveJointForeverNB(robotJointId_t id);
		int moveJointTime(robotJointId_t id, double seconds);
		int moveJointTimeNB(robotJointId_t id, double seconds);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointToByTrackPos(robotJointId_t id, double angle);
		int moveJointToByTrackPosNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTime(double seconds);
		int moveTimeNB(double seconds);
		int moveTo(double angle1, double angle2, double angle3, double angle4);
		int moveToNB(double angle1, double angle2, double angle3, double angle4);
		int moveToByTrackPos(double angle1, double angle2, double angle3, double angle4);
		int moveToByTrackPosNB(double angle1, double angle2, double angle3, double angle4);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int point(double x, double y, double z, int pointsize, char *color);
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[:], double angle1[:], double angle2[:], double angle3[:], double angle4[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &a1, robotRecordData_t &a2, robotRecordData_t &a3, robotRecordData_t &a4, double seconds, ...);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistancesBegin(robotRecordData_t &t, robotRecordData_t &d1, robotRecordData_t &d2, robotRecordData_t &d3, robotRecordData_t &d4, double radius, double seconds, ...);
		int recordDistanceOffset(double distance);
		int recordDistancesEnd(int &num);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, ...);
		int recordxyEnd(int &num);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);		// deprecated
		int resetToZero(void);
		int resetToZeroNB(void);
		int setJointPower(robotJointId_t id, int power);		// deprecated
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
		int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4);
		int setMotorPower(robotJointId_t id, int power);		// deprecated
		int setSpeed(double speed, double radius);
		int stop(void);		// deprecated
		int stopAllJoints(void);		// deprecated
		int stopOneJoint(robotJointId_t id);		// deprecated
		int stopFourJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3, robotJointId_t id4);		// deprecated
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);		// deprecated
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);		// deprecated
		int systemTime(double &time);
		int text(double x, double y, double z, char *text);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};

class DLLIMPORT CMobotGroup {
	public:
		CMobotGroup();
		virtual ~CMobotGroup();

		int addRobot(CMobot& robot);
		int addRobots(array CMobot robots[], ...);
		int blinkLED(double delay, int num);
		int connect(void);
		int driveBackward(double angle);
		int driveBackwardNB(double angle);
		int driveDistance(double distance, double radius);
		int driveDistanceNB(double distance, double radius);
		int driveForeverNB(void);
		int driveForward(double angle);
		int driveForwardNB(double angle);
		int driveTime(double seconds);
		int driveTimeNB(double seconds);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);		// deprecated
		int jumpJointToNB(robotJointId_t id, double angle);		// deprecated
		int jumpTo(double angle1, double angle2, double angle3, double angle4);		// deprecated
		int jumpToNB(double angle1, double angle2, double angle3, double angle4);		// deprecated
		int motionArch(double angle);
		int motionArchNB(double angle);
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionInchwormLeftNB(int num);
		int motionInchwormRight(int num);
		int motionInchwormRightNB(int num);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForwardNB(double angle);
		int motionSkinny(double angle);
		int motionSkinnyNB(double angle);
		int motionRollForward(double angle);
		int motionInchwormLeft(int num);
		int motionStand(void);
		int motionStandNB(void);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionTumbleRight(int num);
		int motionTumbleRightNB(int num);
		int motionTumbleLeft(int num);
		int motionTumbleLeftNB(int num);
		int motionUnstand(void);
		int motionUnstandNB(void);
		int motionWait(void);
		int move(double angle1, double angle2, double angle3, double angle4);
		int moveNB(double angle1, double angle2, double angle3, double angle4);
		int moveForeverNB(void);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointByPowerNB(robotJointId_t id, int power);
		int moveJointForeverNB(robotJointId_t id);
		int moveJointTime(robotJointId_t id, double seconds);
		int moveJointTimeNB(robotJointId_t id, double seconds);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointToByTrackPos(robotJointId_t id, double angle);
		int moveJointToByTrackPosNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTime(double seconds);
		int moveTimeNB(double seconds);
		int moveTo(double angle1, double angle2, double angle3, double angle4);
		int moveToNB(double angle1, double angle2, double angle3, double angle4);
		int moveToByTrackPos(double angle1, double angle2, double angle3, double angle4);
		int moveToByTrackPosNB(double angle1, double angle2, double angle3, double angle4);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);		// deprecated
		int resetToZero(void);
		int resetToZeroNB(void);
		int setJointPower(robotJointId_t id, int power);		// deprecated
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double angle);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4);
		int setMotorPower(robotJointId_t id, int power);		// deprecated
		int setSpeed(double speed, double radius);
		int stop(void);		// deprecated
		int stopAllJoints(void);		// deprecated
		int stopOneJoint(robotJointId_t id);		// deprecated
		int stopFourJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3, robotJointId_t id4);		// deprecated
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);		// deprecated
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);		// deprecated
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};

#pragma importf "cmobot.chf"

#endif  // MOBOT_H_

