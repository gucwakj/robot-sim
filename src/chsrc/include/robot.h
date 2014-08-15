#ifndef ROBOT_H_
#define ROBOT_H_

class DLLIMPORT RoboSim {
	public:
		RoboSim(char *name, int pause);
		virtual ~RoboSim();
		static void *_dlhandle;
		static int _dlcount;
};

class DLLIMPORT Robot {
	public:
		Robot(robotJointId_t, robotJointId_t);
		virtual ~Robot();

		int blinkLED(double, int);
		int connect(...);
		int delay(double);
		int delaySeconds(double);
		int disableRecordDataShift(void);
		int disconnect(void);
		int driveBackward(double);
		int driveBackwardNB(double);
		int driveDistance(double, double);
		int driveDistanceNB(double, double);
		int driveForever(void);
		int driveForeverNB(void);
		int driveForward(double);
		int driveForwardNB(double);
		int driveTime(double);
		int driveTimeNB(double);
		int drivexy(double, double, double, double);
		int drivexyNB(double, double, double, double);
		int drivexyTo(double, double, double, double);
		int drivexyToNB(double, double, double, double);
		int drivexyToFunc(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncNB(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncSmooth(double, double, int, double (*func)(double), double, double);
		int drivexyToPoly(double, double, int, char*, double, double);
		int drivexyToPolyNB(double, double, int, char*, double, double);
		int drivexyToSmooth(double, double, double, double, double, double, double, double);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getAccelerometerData(double&, double&, double&);
		int getBatteryVoltage(double&);
		int getDistance(double&, double);
		int getFormFactor(int&);
		int getID(void);
		int getJointAngle(robotJointId_t, double&, ...);
		int getJointAngleInstant(robotJointId_t, double&);
		int getJointMaxSpeed(robotJointId_t, double&);
		int getJointSafetyAngle(double&);
		int getJointSafetyAngleTimeout(double&);
		int getJointSpeed(robotJointId_t, double&);
		int getJointSpeedRatio(robotJointId_t, double&);
		int getLEDColorName(char[]);
		int getLEDColorRGB(int&, int&, int&);
		int getxy(double&, double&);
		int holdJoint(robotJointId_t);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int moveForeverNB(void);
		int moveJoint(robotJointId_t, double);
		int moveJointNB(robotJointId_t, double);
		int moveJointByPowerNB(robotJointId_t, int);
		int moveJointForeverNB(robotJointId_t);
		int moveJointTime(robotJointId_t, double);
		int moveJointTimeNB(robotJointId_t, double);
		int moveJointTo(robotJointId_t, double);
		int moveJointToNB(robotJointId_t, double);
		int moveJointToByTrackPos(robotJointId_t, double);
		int moveJointToByTrackPosNB(robotJointId_t, double);
		int moveJointWait(robotJointId_t);
		int moveTime(double);
		int moveTimeNB(double);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngle(robotJointId_t, double[], double[], int, double, ...);
		int recordAngleBegin(robotJointId_t, robotRecordData_t&, robotRecordData_t&, double, ...);
		int recordAngleEnd(robotJointId_t, int&);
		int recordAnglesEnd(int&);
		int recordDistanceBegin(robotJointId_t, robotRecordData_t&, robotRecordData_t&, double, double, ...);
		int recordDistanceEnd(robotJointId_t, int&);
		int recordDistanceOffset(double);
		int recordDistancesEnd(int&);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t&, robotRecordData_t&, double, ...);
		int recordxyEnd(int&);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int, double);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int);
		int setLEDColor(char*);
		int setLEDColorRGB(int, int, int);
		int setJointSafetyAngle(double);
		int setJointSafetyAngleTimeout(double);
		int setJointSpeed(robotJointId_t, double);
		int setJointSpeedRatio(robotJointId_t, double);
		int setSpeed(double, double);
		int systemTime(double&);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		int turnRightNB(double, double, double);
};

class DLLIMPORT RobotGroup {
	public:
		RobotGroup();
		virtual ~RobotGroup();
		int addRobot(Robot& robot);
		int addRobots(array Robot robots[], ...);

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
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int, double);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int);
		int setLEDColor(char*);
		int setLEDColorRGB(int, int, int);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double angle);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setSpeed(double speed, double radius);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};

void* RoboSim::_dlhandle = NULL;
int RoboSim::_dlcount = 0;
#pragma importf "robotgroup.chf"

#endif  // ROBOT_H_

