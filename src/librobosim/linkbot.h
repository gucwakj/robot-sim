#ifndef LINKBOT_H_
#define LINKBOT_H_

#include "config.h"
#include "robosim.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CLinkbotT : public ModularRobot {
		friend class linkbotNodeCallback;

	// public api
	public:
		CLinkbotT(int disabled = -1, int type = LINKBOTT);
		virtual ~CLinkbotT();

		int accelJointAngleNB(robotJointId_t, double, double);
		int accelJointCycloidalNB(robotJointId_t, double, double);
		int accelJointHarmonicNB(robotJointId_t, double, double);
		int accelJointSmoothNB(robotJointId_t, double, double, double, double);
		int accelJointTimeNB(robotJointId_t, double, double);
		int accelJointToMaxSpeedNB(robotJointId_t, double);
		int accelJointToVelocityNB(robotJointId_t, double, double);
		int closeGripper(void);
		int closeGripperNB(void);
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
		int getJointAngles(double&, double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&, double&);
		int getJointSpeeds(double&, double&, double&);
		int getJointSpeedRatios(double&, double&, double&);
		int jumpTo(double, double, double);
		int jumpToNB(double, double, double);
		int move(double, double, double);
		int moveNB(double, double, double);
		int moveTo(double, double, double);
		int moveToNB(double, double, double);
		int openGripper(double);
		int openGripperNB(double);
		int recordAngles(double[], double[], double[], double[], int, double, int = 1);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int setJointSpeeds(double, double, double);
		int setJointSpeedRatios(double, double, double);
		int setSpeed(double, double);
		int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		int turnRightNB(double, double, double);

	// inherited functions
	private:
		virtual int build(xml_robot_t);
		virtual int build(xml_robot_t, ModularRobot*, xml_conn_t);
		virtual int buildIndividual(double, double, double, dMatrix3, double*);
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group*, int);
#endif // ENABLE_GRAPHICS
		virtual double getAngle(int);
		virtual int getConnectionParams(int, dMatrix3, double*);
		virtual int initParams(int, int);
		virtual int initDims(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

	// private functions
	private:
		int add_connector(int, int, double);							// add connector
		int add_connector_daisy(int, int, double, int, int);			// add daisy chained connector
		int build_bigwheel(conn_t, int, int = -1, int = -1);			// build big wheel connector
		int build_body(double, double, double, dMatrix3, double);		// build body of linkbot
		int build_bridge(conn_t, int, int = -1, int = -1);				// build bridge connector
		int build_caster(conn_t, int, int = -1, int = -1);				// build caster connector
		int build_cube(conn_t, int, int = -1, int = -1);				// build cube connector
		int build_face(int, double, double, double, dMatrix3, double);	// build face of linkbot
		int build_faceplate(conn_t, int, int = -1, int = -1);			// build faceplate connector
		int build_gripper(conn_t, int);									// build gripper connector
		int build_omnidrive(conn_t, int, int = -1, int = -1);			// build omnidrive connector
		int build_simple(conn_t, int face, int = -1, int = -1);			// build simple connector
		int build_smallwheel(conn_t, int, int = -1, int = -1);			// build small wheel connector
		int build_tinywheel(conn_t, int, int = -1, int = -1);			// build tiny wheel connector
		int build_wheel(conn_t, int, double, int = -1, int = -1);		// build custom wheel connector
#ifdef ENABLE_GRAPHICS
		void draw_connector(conn_t, osg::Group*);						// load connector stl
#endif // ENABLE_GRAPHICS
		int fix_body_to_connector(dBodyID, int);						// fix body onto connector
		int fix_connector_to_body(dBodyID, dBodyID);					// fix connector onto body
		int get_connector_params(int, int, dMatrix3, double*);			// get params of connector face
		static void* closeGripperNBThread(void*);						// thread to close gripper
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
			FACE1,
			FACE2,
			FACE3,
			NUM_PARTS
		};

		// dimensions
		double	_bridge_length,
				_cubic_length,
				_face_depth,
				_face_radius,
				_omni_length,
				_tinywheel_radius;
};

class DLLIMPORT CLinkbotTGroup {
	// public api
	public:
		CLinkbotTGroup();
		virtual ~CLinkbotTGroup();
		int addRobot(CLinkbotT&);
		int addRobots(CLinkbotT[], int);

		int accelJointAngleNB(robotJointId_t, double, double);
		int accelJointCycloidalNB(robotJointId_t, double, double);
		int accelJointHarmonicNB(robotJointId_t, double, double);
		int accelJointSmoothNB(robotJointId_t, double, double, double, double);
		int accelJointTimeNB(robotJointId_t, double, double);
		int accelJointToMaxSpeedNB(robotJointId_t, double);
		int accelJointToVelocityNB(robotJointId_t, double, double);
		int blinkLED(double, int);
		int closeGripper(void);
		int closeGripperNB(void);
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
		int jumpTo(double, double, double);
		int jumpToNB(double, double, double);
		int move(double, double, double);
		int moveNB(double, double, double);
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
		int moveTo(double, double, double);
		int moveToNB(double, double, double);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int openGripper(double);
		int openGripperNB(double);
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
		int setJointSpeeds(double, double, double);
		int setJointSpeedRatio(robotJointId_t, double);
		int setJointSpeedRatios(double, double, double);
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
			CLinkbotT *robot;
			struct robots_s *next;
		} *robots_t;

		robots_t _robots;
		double _d;
		int _i;
		THREAD_T *_thread;
};

class DLLIMPORT CLinkbotI : public CLinkbotT {
	public:
		CLinkbotI(void) : CLinkbotT(1, LINKBOTI) {}
};
class DLLIMPORT CLinkbotL : public CLinkbotT {
	public:
		CLinkbotL(void) : CLinkbotT(2, LINKBOTL) {}
};
class DLLIMPORT CLinkbotIGroup : public CLinkbotTGroup {};
class DLLIMPORT CLinkbotLGroup : public CLinkbotTGroup {};

// global structs for threading
typedef struct linkbotMoveArg_s {
	double x, y, radius, trackwidth;
	int i;
	double (*func)(double x);
	char *expr;
	CLinkbotT *robot;
} linkbotMoveArg_t;

// simulation
extern RoboSim *g_sim;

#endif // LINKBOT_H_

