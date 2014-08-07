#ifndef LINKBOT_H_
#define LINKBOT_H_

#include "robosim.h"
#include "config.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CLinkbotT : virtual public CRobot {
	public:
		CLinkbotT(int disabled = -1, int type = LINKBOTT);
		virtual ~CLinkbotT();

		int accelJointAngleNB(robotJointId_t id, double a, double angle);
		int accelJointCycloidalNB(robotJointId_t id, double angle, double t);
		int accelJointHarmonicNB(robotJointId_t id, double angle, double t);
		int accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle);
		int accelJointTimeNB(robotJointId_t id, double a, double t);
		int accelJointToMaxSpeedNB(robotJointId_t id, double a);
		int accelJointToVelocityNB(robotJointId_t id, double a, double v);
		int closeGripper(void);
		int closeGripperNB(void);
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
		int drivexy(double x, double y, double radius, double trackwidth);
		int drivexyNB(double x, double y, double radius, double trackwidth);
		int drivexyTo(double x, double y, double radius, double trackwidth);
		int drivexyToNB(double x, double y, double radius, double trackwidth);
		int drivexyToFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int drivexyToFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int drivexyToPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int drivexyToPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int drivexyWait(void);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getLEDColorName(char color[]);
		int getLEDColorRGB(int &r, int &g, int &b);
		int getJointAngles(double &angle1, double &angle2, double &angle3, int numReadings = 10);
		int getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getxy(double &x, double &y);
		int jumpJointTo(robotJointId_t id, double angle);
		int jumpJointToNB(robotJointId_t id, double angle);
		int jumpTo(double angle1, double angle2, double angle3);
		int jumpToNB(double angle1, double angle2, double angle3);
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveTime(double seconds);
		int moveTimeNB(double seconds);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int openGripper(double angle);
		int openGripperNB(double angle);
		int recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData = 1);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[], double angle1[], double angle2[], double angle3[], int num, double seconds, int shiftData = 1);
		int recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &a1, robotRecordData_t &a2, robotRecordData_t &a3, double seconds, int shiftData = 1);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistanceOffset(double distance);
		int recordDistancesBegin(robotRecordData_t &t, robotRecordData_t &d1, robotRecordData_t &d2, robotRecordData_t &d3, double radius, double seconds, int shiftData = 1);
		int recordDistancesEnd(int &num);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, int shiftData = 1);
		int recordxyEnd(int &num);
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
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setSpeed(double speed, double radius);
		int systemTime(double &time);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
    private:
		enum robot_pieces_e {       // each body part which is built
			BODY,
			FACE1,
			FACE2,
			FACE3,
			NUM_PARTS
		};

		// private functions inherited from CRobot class
		virtual int build(xml_robot_t robot);
		virtual int build(xml_robot_t robot, CRobot *base, xml_conn_t conn);
		virtual int buildIndividual(double x, double y, double z, dMatrix3 R, double *rot);
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group *root, int tracking);
#endif // ENABLE_GRAPHICS
		virtual int getConnectionParams(int face, dMatrix3 R, double *p);
		virtual int initParams(int disabled, int type);
		virtual int initDims(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

		// private functions
		int add_connector(int type, int face, double size);									// add connector
		int add_connector_daisy(int conn, int side, double size, int face, int type);		// add daisy chained connector
		int build_bigwheel(conn_t conn, int face, int side = -1, int type = -1);			// build big wheel connector
		int build_body(double x, double y, double z, dMatrix3 R, double theta);				// build body of linkbot
		int build_bridge(conn_t conn, int face, int side = -1, int type = -1);				// build bridge connector
		int build_caster(conn_t conn, int face, int side = -1, int type = -1);				// build caster connector
		int build_cube(conn_t conn, int face, int side = -1, int type = -1);				// build cube connector
		int build_face(int id, double x, double y, double z, dMatrix3 R, double theta);		// build face of linkbot
		int build_faceplate(conn_t conn, int face, int side = -1, int type = -1);			// build faceplate connector
		int build_gripper(conn_t conn, int face);											// build gripper connector
		int build_omnidrive(conn_t conn, int face, int side = -1, int type = -1);			// build omnidrive connector
		int build_simple(conn_t conn, int face, int side = -1, int type = -1);				// build simple connector
		int build_smallwheel(conn_t conn, int face, int side = -1, int type = -1);			// build small wheel connector
		int build_tinywheel(conn_t conn, int face, int side = -1, int type = -1);			// build tiny wheel connector
		int build_wheel(conn_t conn, int face, double size, int side = -1, int type = -1);	// build custom wheel connector
#ifdef ENABLE_GRAPHICS
		void draw_connector(conn_t conn, osg::Group *robot);								// load connector stl
#endif // ENABLE_GRAPHICS
		int fix_body_to_connector(dBodyID cBody, int face);									// fix body onto connector
		int fix_connector_to_body(dBodyID rBody, dBodyID cBody);							// fix connector onto body
		static void* closeGripperNBThread(void *arg);										// thread to close gripper
		static void* driveTimeNBThread(void *arg);											// thread to drive robot
		static void* drivexyThread(void *arg);												// thread to run drivexy
		static void* drivexyToThread(void *arg);											// thread to run drivexyTo
		static void* drivexyToFuncThread(void *arg);										// thread to run drivexyFunc
		static void* drivexyToPolyThread(void *arg);										// thread to run drivexyPoly
		//static void* recordAngleThread(void *arg);											// thread to record angle
		//static void* recordAngleBeginThread(void *arg);										// thread to record angle
		//static void* recordAnglesThread(void *arg);											// thread to record angles
		//static void* recordAnglesBeginThread(void *arg);									// thread to record angles
		//static void* recordxyBeginThread(void *arg);										// thread to record positions
		static void* moveTimeNBThread(void *arg);											// thread to move all joints

		// private data members
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

typedef struct linkbotMoveArg_s {
	double x, y, radius, trackwidth;
	int i;
	double (*func)(double x);
	char *expr;
	CLinkbotT *robot;
} linkbotMoveArg_t;

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

extern RoboSim *g_sim;

#endif // LINKBOT_H_
