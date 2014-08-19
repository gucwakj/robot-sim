#ifndef LINKBOT_H_
#define LINKBOT_H_

#include <array.h>
#include "macros.hpp"

#define NUM_DOF 3

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
#endif // MACROS

#include "robot.h"

class DLLIMPORT CLinkbotI {
	public:
		CLinkbotI();
		virtual ~CLinkbotI();
		int accelJointAngleNB(robotJointId_t id, double a, double angle);
		int accelJointCycloidalNB(robotJointId_t id, double angle, double t);
		int accelJointHarmonicNB(robotJointId_t id, double angle, double t);
		int accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle);
		int accelJointTimeNB(robotJointId_t id, double a, double t);
		int accelJointToMaxSpeedNB(robotJointId_t id, double a);
		int accelJointToVelocityNB(robotJointId_t id, double a, double v);
		int blinkLED(double delay, int num);
		int closeGripper(void);
		int closeGripperNB(void);
		int connect(...);
		int delay(double milliseconds);
		int delaySeconds(double seconds);
		int disableRecordDataShift(void);
		int disconnect(void);
		int driveAccelCycloidalNB(double radius, double d, double t);
		int driveAccelDistanceNB(double radius, double a, double d);
		int driveAccelHarmonicNB(double radius, double d, double t);
		int driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d);
		int driveAccelTimeNB(double radius, double a, double t);
		int driveAccelToMaxSpeedNB(double radius, double a);
		int driveAccelToVelocityNB(double radius, double a, double v);
		int driveBackward(double angle);
		int driveBackwardNB(double angle);
		int driveDistance(double distance, double radius);
		int driveDistanceNB(double distance, double radius);
		int driveForeverNB(void);
		int driveForward(double angle);
		int driveForwardNB(double angle);
		int driveJointTo(robotJointId_t id, double angle);		// deprecated
		int driveJointToNB(robotJointId_t id, double angle);		// deprecated
		int driveTime(double seconds);
		int driveTimeNB(double seconds);
		int driveTo(double angle1, double angle2, double angle3);		// deprecated
		int driveToNB(double angle1, double angle2, double angle3);		// deprecated
		int drivexy(double x, double y, double radius, double trackwidth);
		int drivexyNB(double x, double y, double radius, double trackwidth);
		int drivexyTo(double x, double y, double radius, double trackwidth);
		int drivexyToNB(double x, double y, double radius, double trackwidth);
		int drivexyToExpr(double x0, double xf, int n, char *expr, double radius, double trackwidth);
		int drivexyToExprNB(double x0, double xf, int n, char *expr, double radius, double trackwidth);
		int drivexyToFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int drivexyToFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int drivexyToPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int drivexyToPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getColor(string_t &color);		// deprecated
		int getColorName(char color[]);		// deprecated
		int getColorRGB(int &r, int &g, int &b);		//deprecated
		int getLEDColor(string_t &color);
		int getLEDColorName(char color[]);
		int getLEDColorRGB(int &r, int &g, int &b);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getID(void);
		int getJointAngle(robotJointId_t id, double &angle, ... );
		int getJointAngleAverage(robotJointId_t id, double &angle, ... );		// deprecated
		int getJointAngleInstant(robotJointId_t id, double &angle);
		int getJointAngles(double &angle1, double &angle2, double &angle3, ...);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);		// deprecated
		int getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getxy(double &x, double &y);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);		// deprecated
		int jumpJointToNB(robotJointId_t id, double angle);		// deprecated
		int jumpTo(double angle1, double angle2, double angle3);		// deprecated
		int jumpToNB(double angle1, double angle2, double angle3);		// deprecated
		int line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color);
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);		// deprecated
		int moveBackwardNB(double angle);		// deprecated
		int moveDistance(double distance, double radius);		// deprecated
		int moveDistanceNB(double distance, double radius);		// deprecated
		int moveForeverNB(void);
		int moveForward(double angle);		// deprecated
		int moveForwardNB(double angle);		// deprecated
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
		int moveTo(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToByTrackPos(double angle1, double angle2, double angle3);
		int moveToByTrackPosNB(double angle1, double angle2, double angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int movexy(double x, double y, double radius, double trackwidth);		// deprecated
		int movexyNB(double x, double y, double radius, double trackwidth);		// deprecated
		int movexyTo(double x, double y, double radius, double trackwidth);		// deprecated
		int movexyToNB(double x, double y, double radius, double trackwidth);		// deprecated
		int movexyToExpr(double x0, double xf, int n, char *expr, double radius, double trackwidth);		// deprecated
		int movexyToExprNB(double x0, double xf, int n, char *expr, double radius, double trackwidth);		// deprecated
		int movexyToFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);		// deprecated
		int movexyToFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);		// deprecated
		int movexyToPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth);		// deprecated
		int movexyToPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth);		// deprecated
		int movexyWait(void);		// deprecated
		int openGripper(double angle);
		int openGripperNB(double angle);
		int point(double x, double y, double z, int pointsize, char *color);
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[:], double angle1[:], double angle2[:], double angle3[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &a1, robotRecordData_t &a2, robotRecordData_t &a3, double seconds, ...);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistanceOffset(double distance);
		int recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &d1, robotRecordData_t &d2, robotRecordData_t &d3, double radius, double seconds, ...);
		int recordDistancesEnd(int &num);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, ...);
		int recordxyEnd(int &num);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);		// deprecated
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int frequency);
		int setColor(char *color);		// deprecated
		int setColorRGB(int r, int g, int b);		// deprecated
		int setExitState(int exitState);		// deprecated
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setJointMovementStateNB(int id, int dir);		// deprecated
		int setJointMovementStateTime(int id, int dir, double seconds);		// deprecated
		int setJointMovementStateTimeNB(int id, int dir, double seconds);		// deprecated
		int setJointPower(robotJointId_t id, int power);		// deprecated
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);		// deprecated
		int setMovementStateNB(int dir1, int dir2, int dir3);		// deprecated
		int setMovementStateTime(int dir1, int dir2, int dir3, double seconds);		// deprecated
		int setMovementStateTimeNB(int dir1, int dir2, int dir3, double seconds);		// deprecated
		int setSpeed(double speed, double radius);
		int setTwoWheelRobotSpeed(double speed, double radius);		// deprecated
		int stop(void);		// deprecated
		int stopAllJoints(void);		// deprecated
		int stopOneJoint(robotJointId_t id);		// deprecated
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

class DLLIMPORT CLinkbotIGroup {
	public:
		CLinkbotIGroup();
		virtual ~CLinkbotIGroup();
		int accelJointAngleNB(robotJointId_t id, double a, double angle);
		int accelJointCycloidalNB(robotJointId_t id, double angle, double t);
		int accelJointHarmonicNB(robotJointId_t id, double angle, double t);
		int accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle);
		int accelJointTimeNB(robotJointId_t id, double a, double t);
		int accelJointToMaxSpeedNB(robotJointId_t id, double a);
		int accelJointToVelocityNB(robotJointId_t id, double a, double v);
		int addRobot(CLinkbotI& robot);
		int addRobots(array CLinkbotI robots[], ...);
		int blinkLED(double delay, int num);
		int closeGripper(void);
		int closeGripperNB(void);
		int connect(void);
		int driveAccelCycloidalNB(double radius, double d, double t);
		int driveAccelDistanceNB(double radius, double a, double d);
		int driveAccelHarmonicNB(double radius, double d, double t);
		int driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d);
		int driveAccelTimeNB(double radius, double a, double t);
		int driveAccelToMaxSpeedNB(double radius, double a);
		int driveAccelToVelocityNB(double radius, double a, double v);
		int driveBackward(double angle);
		int driveBackwardNB(double angle);
		int driveDistance(double distance, double radius);
		int driveDistanceNB(double distance, double radius);
		int driveForeverNB(void);
		int driveForward(double angle);
		int driveForwardNB(double angle);
		int driveJointTo(robotJointId_t id, double angle);		// deprecated
		int driveJointToNB(robotJointId_t id, double angle);		// deprecated
		int driveTime(double seconds);
		int driveTimeNB(double seconds);
		int driveTo(double angle1, double angle2, double angle3);		// deprecated
		int driveToNB(double angle1, double angle2, double angle3);		// deprecated
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);		// deprecated
		int jumpJointToNB(robotJointId_t id, double angle);		// deprecated
		int jumpTo(double angle1, double angle2, double angle3);		// deprecated
		int jumpToNB(double angle1, double angle2, double angle3);		// deprecated
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);		// deprecated
		int moveBackwardNB(double angle);		// deprecated
		int moveDistance(double distance, double radius);		// deprecated
		int moveDistanceNB(double distance, double radius);		// deprecated
		int moveForeverNB(void);
		int moveForward(double angle);		// deprecated
		int moveForwardNB(double angle);		// deprecated
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
		int moveTo(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToByTrackPos(double angle1, double angle2, double angle3);
		int moveToByTrackPosNB(double angle1, double angle2, double angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int openGripper(double angle);
		int openGripperNB(double angle);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);		// deprecated
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int frequency);
		int setColor(char *color);		// deprecated
		int setColorRGB(int r, int g, int b);		// deprecated
		int setExitState(int exitState);		// deprecated
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setJointMovementStateNB(int id, int dir);		// deprecated
		int setJointMovementStateTime(int id, int dir, double seconds);		// deprecated
		int setJointMovementStateTimeNB(int id, int dir, double seconds);		// deprecated
		int setJointPower(robotJointId_t id, int power);		// deprecated
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);		// deprecated
		int setMovementStateNB(int dir1, int dir2, int dir3);		// deprecated
		int setMovementStateTime(int dir1, int dir2, int dir3, double seconds);		// deprecated
		int setMovementStateTimeNB(int dir1, int dir2, int dir3, double seconds);		// deprecated
		int setSpeed(double speed, double radius);
		int setTwoWheelRobotSpeed(double speed, double radius);		// deprecated
		int stop(void);		// deprecated
		int stopAllJoints(void);		// deprecated
		int stopOneJoint(robotJointId_t id);		// deprecated
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);		// deprecated
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);		// deprecated
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};

class DLLIMPORT CLinkbotL {
	public:
		CLinkbotL();
		virtual ~CLinkbotL();
		int accelJointAngleNB(robotJointId_t id, double a, double angle);
		int accelJointCycloidalNB(robotJointId_t id, double angle, double t);
		int accelJointHarmonicNB(robotJointId_t id, double angle, double t);
		int accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle);
		int accelJointTimeNB(robotJointId_t id, double a, double t);
		int accelJointToMaxSpeedNB(robotJointId_t id, double a);
		int accelJointToVelocityNB(robotJointId_t id, double a, double v);
		int blinkLED(double delay, int num);
		int closeGripper(void);
		int closeGripperNB(void);
		int connect(...);
		int delay(double milliseconds);
		int delaySeconds(double seconds);
		int disableRecordDataShift(void);
		int disconnect(void);
		int driveJointTo(robotJointId_t id, double angle);		// deprecated
		int driveJointToNB(robotJointId_t id, double angle);		// deprecated
		int driveTo(double angle1, double angle2, double angle3);		// deprecated
		int driveToNB(double angle1, double angle2, double angle3);		// deprecated
		int enableRecordDataShift(void);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getColor(string_t &color);		// deprecated
		int getColorName(char color[]);		// deprecated
		int getColorRGB(int &r, int &g, int &b);		//deprecated
		int getLEDColor(string_t &color);
		int getLEDColorName(char color[]);
		int getLEDColorRGB(int &r, int &g, int &b);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getID(void);
		int getJointAngle(robotJointId_t id, double &angle, ... );
		int getJointAngleAverage(robotJointId_t id, double &angle, ... );		// deprecated
		int getJointAngleInstant(robotJointId_t id, double &angle);
		int getJointAngles(double &angle1, double &angle2, double &angle3, ...);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);		// deprecated
		int getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getxy(double &x, double &y);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);		// deprecated
		int jumpJointToNB(robotJointId_t id, double angle);		// deprecated
		int jumpTo(double angle1, double angle2, double angle3);		// deprecated
		int jumpToNB(double angle1, double angle2, double angle3);		// deprecated
		int line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color);
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
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
		int moveTo(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToByTrackPos(double angle1, double angle2, double angle3);
		int moveToByTrackPosNB(double angle1, double angle2, double angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int openGripper(double angle);
		int openGripperNB(double angle);
		int point(double x, double y, double z, int pointsize, char *color);
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[:], double angle1[:], double angle2[:], double angle3[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &a1, robotRecordData_t &a2, robotRecordData_t &a3, double seconds, ...);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistanceOffset(double distance);
		int recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &d1, robotRecordData_t &d2, robotRecordData_t &d3, double radius, double seconds, ...);
		int recordDistancesEnd(int &num);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, ...);
		int recordxyEnd(int &num);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);		// deprecated
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int frequency);
		int setColor(char *color);		// deprecated
		int setColorRGB(int r, int g, int b);		// deprecated
		int setExitState(int exitState);		// deprecated
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setJointMovementStateNB(int id, int dir);		// deprecated
		int setJointMovementStateTime(int id, int dir, double seconds);		// deprecated
		int setJointMovementStateTimeNB(int id, int dir, double seconds);		// deprecated
		int setJointPower(robotJointId_t id, int power);		// deprecated
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);		// deprecated
		int setMovementStateNB(int dir1, int dir2, int dir3);		// deprecated
		int setMovementStateTime(int dir1, int dir2, int dir3, double seconds);		// deprecated
		int setMovementStateTimeNB(int dir1, int dir2, int dir3, double seconds);		// deprecated
		int setSpeed(double speed, double radius);
		int stop(void);		// deprecated
		int stopAllJoints(void);		// deprecated
		int stopOneJoint(robotJointId_t id);		// deprecated
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

class DLLIMPORT CLinkbotLGroup {
	public:
		CLinkbotLGroup();
		virtual ~CLinkbotLGroup();
		int accelJointAngleNB(robotJointId_t id, double a, double angle);
		int accelJointCycloidalNB(robotJointId_t id, double angle, double t);
		int accelJointHarmonicNB(robotJointId_t id, double angle, double t);
		int accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle);
		int accelJointTimeNB(robotJointId_t id, double a, double t);
		int accelJointToMaxSpeedNB(robotJointId_t id, double a);
		int accelJointToVelocityNB(robotJointId_t id, double a, double v);
		int addRobot(CLinkbotL& robot);
		int addRobots(array CLinkbotL robots[], ...);
		int blinkLED(double delay, int num);
		int closeGripper(void);
		int closeGripperNB(void);
		int connect(void);
		int driveJointTo(robotJointId_t id, double angle);		// deprecated
		int driveJointToNB(robotJointId_t id, double angle);		// deprecated
		int driveTo(double angle1, double angle2, double angle3);		// deprecated
		int driveToNB(double angle1, double angle2, double angle3);		// deprecated
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);		// deprecated
		int jumpJointToNB(robotJointId_t id, double angle);		// deprecated
		int jumpTo(double angle1, double angle2, double angle3);		// deprecated
		int jumpToNB(double angle1, double angle2, double angle3);		// deprecated
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
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
		int moveTo(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToByTrackPos(double angle1, double angle2, double angle3);
		int moveToByTrackPosNB(double angle1, double angle2, double angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int openGripper(double angle);
		int openGripperNB(double angle);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);		// deprecated
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int frequency);
		int setColor(char *color);		// deprecated
		int setColorRGB(int r, int g, int b);		// deprecated
		int setExitState(int exitState);		// deprecated
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setJointMovementStateNB(int id, int dir);		// deprecated
		int setJointMovementStateTime(int id, int dir, double seconds);		// deprecated
		int setJointMovementStateTimeNB(int id, int dir, double seconds);		// deprecated
		int setJointPower(robotJointId_t id, int power);		// deprecated
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);		// deprecated
		int setMovementStateNB(int dir1, int dir2, int dir3);		// deprecated
		int setMovementStateTime(int dir1, int dir2, int dir3, double seconds);		// deprecated
		int setMovementStateTimeNB(int dir1, int dir2, int dir3, double seconds);		// deprecated
		int setSpeed(double speed, double radius);
		int stop(void);		// deprecated
		int stopAllJoints(void);		// deprecated
		int stopOneJoint(robotJointId_t id);		// deprecated
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);		// deprecated
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);		// deprecated
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};

#pragma importf "clinkboti.chf"
#pragma importf "clinkbotl.chf"

#endif // LINKBOT_H_

