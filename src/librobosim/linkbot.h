#ifndef LINKBOT_H_
#define LINKBOT_H_

#include "robosim.h"
#include "config.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CLinkbotT : virtual public CRobot {
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
		virtual int build(xml_robot_t, CRobot*, xml_conn_t);
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
	public:
		CLinkbotTGroup();
		virtual ~CLinkbotTGroup();

		int accelJointAngleNB(robotJointId_t id, double a, double angle);
		int accelJointCycloidalNB(robotJointId_t id, double angle, double t);
		int accelJointHarmonicNB(robotJointId_t id, double angle, double t);
		int accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle);
		int accelJointTimeNB(robotJointId_t id, double a, double t);
		int accelJointToMaxSpeedNB(robotJointId_t id, double a);
		int accelJointToVelocityNB(robotJointId_t id, double a, double v);
		int addRobot(CLinkbotT& robot);
		int addRobots(CLinkbotT robots[], int num);
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
		int driveTime(double seconds);
		int driveTimeNB(double seconds);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);
		int jumpJointToNB(robotJointId_t id, double angle);
		int jumpTo(double angle1, double angle2, double angle3);
		int jumpToNB(double angle1, double angle2, double angle3);
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveForeverNB(void);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointForeverNB(robotJointId_t id);
		int moveJointTime(robotJointId_t id, double seconds);
		int moveJointTimeNB(robotJointId_t id, double seconds);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTime(double seconds);
		int moveTimeNB(double seconds);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int openGripper(double angle);
		int openGripperNB(double angle);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int frequency);
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setJointPower(robotJointId_t id, int power);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setSpeed(double speed, double radius);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
	private:
		typedef struct robots_s {
			CLinkbotT *robot;
			struct robots_s *next;
		} *robots_t;
		double _d;
		int _i;
		robots_t _robots;
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

