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
		CNXT(void);
		virtual ~CNXT(void);

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

	// inherited functions from Robot class
    private:
		int build(xml_robot_t);
		int buildIndividual(double, double, double, dMatrix3, double*);
#ifdef ENABLE_GRAPHICS
		int draw(osg::Group*, int);
#endif // ENABLE_GRAPHICS
		double getAngle(int);
		int initParams(int, int);
		int initDims(void);
		void simPreCollisionThread(void);
		void simPostCollisionThread(void);

	// private functions
    private:
		int build_body(double, double, double, dMatrix3, double);		// build body of linkbot
		int build_wheel(int, double, double, double, dMatrix3, double);	// build wheels of nxt

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

		int blinkLED(double, int);
		int connect(void);
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

// simulation
extern RoboSim *g_sim;

#endif // NXT_H_

