#ifndef NXT_H_
#define NXT_H_

#include <array.h>
#include "macros.hpp"

#define NUM_DOF 2

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
#endif // MACROS

#include "robot.h"

class DLLIMPORT CNXT {
	public:
		CNXT();
		~CNXT();

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
		int drivexyToFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int drivexyToFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int drivexyToPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int drivexyToPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getLEDColor(string_t &color);
		int getLEDColorName(char color[]);
		int getLEDColorRGB(int &r, int &g, int &b);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getID(void);
		int getJointAngle(robotJointId_t id, double &angle, ...);
		int getJointAngleInstant(robotJointId_t id, double &angle);
		int getJointAngles(double &angle1, double &angle2, ...);
		int getJointAnglesInstant(double &angle1, double &angle2);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeeds(double &speed1, double &speed2);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeedRatios(double &ratio1, double &ratio2);
		int getxy(double &x, double &y);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int move(double angle1, double angle2);
		int moveNB(double angle1, double angle2);
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
		int moveTo(double angle1, double angle2);
		int moveToNB(double angle1, double angle2);
		int moveToByTrackPos(double angle1, double angle2);
		int moveToByTrackPosNB(double angle1, double angle2);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[:], double angle1[:], double angle2[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &a1, robotRecordData_t &a2, double seconds, ...);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistancesBegin(robotRecordData_t &t, robotRecordData_t &d1, robotRecordData_t &d2, double radius, double seconds, ...);
		int recordDistanceOffset(double distance);
		int recordDistancesEnd(int &num);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, ...);
		int recordxyEnd(int &num);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int frequency);
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratio1, double ratio2);
		int setSpeed(double speed, double radius);
		int systemTime(double &time);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};

class DLLIMPORT CNXTGroup {
	public:
		CNXTGroup();
		virtual ~CNXTGroup();

		int addRobot(CNXT& robot);
		int addRobots(array CNXT robots[], ...);
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
		int move(double angle1, double angle2);
		int moveNB(double angle1, double angle2);
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
		int moveTo(double angle1, double angle2);
		int moveToNB(double angle1, double angle2);
		int moveToByTrackPos(double angle1, double angle2);
		int moveToByTrackPosNB(double angle1, double angle2);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int frequency);
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double angle);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratio1, double ratio2);
		int setSpeed(double speed, double radius);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};

#pragma importf "cnxt.chf"

#endif  // NXT_H_

