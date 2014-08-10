#ifndef NXT_H_
#define NXT_H_

#include "robosim.h"
#include "config.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CNXT : virtual public Robot {
		friend class nxtNodeCallback;

	// public api
	public:
		CNXT();
		virtual ~CNXT();

		int accelJointAngleNB(robotJointId_t, double, double);
		int accelJointCycloidalNB(robotJointId_t, double, double);
		int accelJointHarmonicNB(robotJointId_t, double, double);
		int accelJointSmoothNB(robotJointId_t, double, double, double, double);
		int accelJointTimeNB(robotJointId_t, double, double);
		int accelJointToMaxSpeedNB(robotJointId_t, double);
		int accelJointToVelocityNB(robotJointId_t, double, double);
		int driveAccelCycloidalNB(double, double, double);
		int driveAccelDistanceNB(double, double, double);
		int driveAccelHarmonicNB(double, double, double);
		int driveAccelSmoothNB(double, double, double, double, double);
		int driveAccelTimeNB(double, double, double);
		int driveAccelToMaxSpeedNB(double, double);
		int driveAccelToVelocityNB(double, double, double);
		int driveBackward(double);
		int driveBackwardNB(double);
		int driveDistance(double, double);
		int driveDistanceNB(double, double);
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
		int drivexyToPoly(double, double, int, char*, double, double);
		int drivexyToPolyNB(double, double, int, char*, double, double);
		int getAccelerometerData(double&, double&, double&);
		int getLEDColorName(char[]);
		int getLEDColorRGB(int&, int&, int&);
		int getJointAngles(double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&);
		int getJointSpeeds(double&, double&);
		int getJointSpeedRatios(double&, double&);
		int jumpTo(double, double);
		int jumpToNB(double, double);
		int move(double, double);
		int moveNB(double, double);
		int moveTo(double, double);
		int moveToNB(double, double);
		int recordAngles(double[], double[], double[], int, double, int = 1);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int setJointSpeeds(double, double);
		int setJointSpeedRatios(double, double);
		int setSpeed(double, double);
		int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		int turnRightNB(double, double, double);

	// inherited functions
    private:
		virtual int build(xml_robot_t);
		virtual int buildIndividual(double, double, double, dMatrix3, double*);
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group*, int);
#endif // ENABLE_GRAPHICS
		virtual double getAngle(int);
		virtual int initParams(int, int);
		virtual int initDims(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

	// private functions
    private:
		int build_body(double, double, double, dMatrix3, double);		// build body of linkbot
		int build_wheel(int, double, double, double, dMatrix3, double);	// build wheels of nxt
		static void* driveTimeNBThread(void*);							// thread to drive robot
		static void* drivexyThread(void*);								// thread to run drivexy
		static void* drivexyToThread(void*);							// thread to run drivexyTo
		static void* drivexyToFuncThread(void*);						// thread to run drivexyFunc
		static void* drivexyToPolyThread(void*);						// thread to run drivexyPoly

	// private data
	private:
		// robot body parts
		enum robot_pieces_e {
			BODY,
			WHEEL1,
			WHEEL2,
			NUM_PARTS
		};
};

class DLLIMPORT CNXTGroup {
	// public api
	public:
		CNXTGroup();
		virtual ~CNXTGroup();
		int addRobot(CNXT&);
		int addRobots(CNXT[], int);

		int accelJointAngleNB(robotJointId_t, double, double);
		int accelJointCycloidalNB(robotJointId_t, double, double);
		int accelJointHarmonicNB(robotJointId_t, double, double);
		int accelJointSmoothNB(robotJointId_t, double, double, double, double);
		int accelJointTimeNB(robotJointId_t, double, double);
		int accelJointToMaxSpeedNB(robotJointId_t, double);
		int accelJointToVelocityNB(robotJointId_t, double, double);
		int blinkLED(double, int);
		int connect(void);
		int driveAccelCycloidalNB(double, double, double);
		int driveAccelDistanceNB(double, double, double);
		int driveAccelHarmonicNB(double, double, double);
		int driveAccelSmoothNB(double, double, double, double, double);
		int driveAccelTimeNB(double, double, double);
		int driveAccelToMaxSpeedNB(double, double);
		int driveAccelToVelocityNB(double, double, double);
		int driveBackward(double);
		int driveBackwardNB(double);
		int driveDistance(double, double);
		int driveDistanceNB(double, double);
		int driveForeverNB(void);
		int driveForward(double);
		int driveForwardNB(double);
		int driveTime(double);
		int driveTimeNB(double);
		int holdJoint(robotJointId_t);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t, double);
		int jumpJointToNB(robotJointId_t, double);
		int jumpTo(double, double);
		int jumpToNB(double, double);
		int move(double, double);
		int moveNB(double, double);
		int moveForeverNB(void);
		int moveJoint(robotJointId_t, double);
		int moveJointNB(robotJointId_t, double);
		int moveJointForeverNB(robotJointId_t);
		int moveJointTime(robotJointId_t, double);
		int moveJointTimeNB(robotJointId_t, double);
		int moveJointTo(robotJointId_t, double);
		int moveJointToNB(robotJointId_t, double);
		int moveJointWait(robotJointId_t);
		int moveTime(double);
		int moveTimeNB(double);
		int moveTo(double, double);
		int moveToNB(double, double);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int relaxJoint(robotJointId_t);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int, double);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int);
		int setLEDColor(char*);
		int setLEDColorRGB(int, int, int);
		int setJointPower(robotJointId_t, int);
		int setJointSafetyAngle(double);
		int setJointSafetyAngleTimeout(double);
		int setJointSpeed(robotJointId_t, double);
		int setJointSpeeds(double, double);
		int setJointSpeedRatio(robotJointId_t, double);
		int setJointSpeedRatios(double, double);
		int setSpeed(double, double);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		int turnRightNB(double, double, double);

	// private data
	private:
		typedef struct robots_s {
			CNXT *robot;
			struct robots_s *next;
		} *robots_t;

		robots_t _robots;
		double _d;
		int _i;
		THREAD_T *_thread;
};

// global structs for threading
typedef struct nxtMoveArg_s {
	double x, y, radius, trackwidth;
	int i;
	double (*func)(double x);
	char *expr;
	CNXT *robot;
} nxtMoveArg_t;

// simulation
extern RoboSim *g_sim;

#endif // NXT_H_

