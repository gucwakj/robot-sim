#ifndef LINKBOT_H_
#define LINKBOT_H_

#include "robosim.h"

#ifdef _CH_
#include <array.h>
class DLLIMPORT CLinkbotT {
#else
#include "config.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS
class DLLIMPORT CLinkbotT : virtual public CRobot {
#endif
	public:
#ifdef _CH_
		CLinkbotT();
#else
		CLinkbotT(int disabled = -1, int type = LINKBOTT);
#endif
		virtual ~CLinkbotT();

		int blinkLED(double delay, int num);
#ifdef _CH_
		int connect();
#else
		int connect(char *name = NULL, int pause = 1, int rt = 1);
#endif
		int delay(double milliseconds);
		int delaySeconds(double seconds);
		int disableRecordDataShift();
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int enableRecordDataShift();
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
#ifdef _CH_
		int getLEDColor(string_t &color);
#endif
		int getLEDColorName(char color[]);
		int getLEDColorRGB(int &r, int &g, int &b);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getID();
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngles(double &angle1, double &angle2, double &angle3);
#ifdef _CH_
		int getJointAngleAverage(robotJointId_t id, double &angle, ...);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
#else
		int getJointAngleAverage(robotJointId_t id, double &angle, int numReadings = 10);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings = 10);
#endif
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int getxy(double &x, double &y);
		int isConnected(void);
		int isMoving(void);
#ifdef ENABLE_GRAPHICS
		int line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color);
#endif // ENABLE_GRAPHICS
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
		int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int movexy(double x, double y, double radius, double trackwidth);
		int movexyNB(double x, double y, double radius, double trackwidth);
#ifdef _CH_
		int movexyExpr(double x0, double xf, int n, char *expr, double radius, double trackwidth);
		int movexyExprNB(double x0, double xf, int n, char *expr, double radius, double trackwidth);
#endif
		int movexyFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int movexyFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int movexyPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int movexyPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int movexyTo(double x, double y, double radius, double trackwidth);
		int movexyToNB(double x, double y, double radius, double trackwidth);
		int movexyWait(void);
#ifdef ENABLE_GRAPHICS
		int point(double x, double y, double z, int pointsize, char *color);
#endif // ENABLE_GRAPHICS
#ifdef _CH_
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id,
							 robotRecordData_t &time,
							 robotRecordData_t &angle, double seconds, ...);
		int recordAngles(double time[:],
						 double angle1[:],
						 double angle2[:],
						 double angle3[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3, double seconds, ...);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3, double radius, double seconds, ...);
#else
		int recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
		int recordAngleBegin(robotJointId_t id,
							 robotRecordData_t &time,
							 robotRecordData_t &angle, double seconds, int shiftData = 1);
		int recordAngles(double time[], double angle1[],
										double angle2[],
										double angle3[], int num, double seconds, int shiftData = 1);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3, double seconds, int shiftData = 1);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3, double radius, double seconds, int shiftData = 1);
#endif
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAnglesEnd(int &num);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistanceOffset(double distance);
		int recordDistancesEnd(int &num);
		int recordWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOn(int frequency);
		int setBuzzerFrequencyOff();
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setExitState(robotJointState_t exitState);
		int setGoal(double x, double y, double z);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setTwoWheelRobotSpeed(double speed, double radius);
		int stop(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int stopAllJoints(void);
		int systemTime(double &time);
#ifdef ENABLE_GRAPHICS
		int text(double x, double y, double z, char *text);
#endif // ENABLE_GRAPHICS
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
#ifndef _CH_
    private:
		enum robot_pieces_e {       // each body part which is built
			BODY,
			FACE1,
			FACE2,
			FACE3,
			NUM_PARTS
		};
		enum robot_bodies_e {       // each body which has a degree of freedom
			F1,
			F2,
			F3,
			NUM_DOF
		};

		// private functions inherited from CRobot class
		virtual int addBuddy(int i, CRobot *robot);
		virtual int addToSim(dWorldID &world, dSpaceID &space, double *clock);
		virtual int build(xml_robot_t robot);
		virtual int build(xml_robot_t robot, CRobot *base, xml_conn_t conn);
		virtual double getAngle(int i);
		virtual double getAngularRate(int i);
		virtual dBodyID getBodyID(int id);
		virtual double getCenter(int i);
		virtual int getConnectionParams(int face, dMatrix3 R, double *p);
		virtual dBodyID getConnectorBodyID(int face);
		virtual dBodyID getConnectorBodyIDs(int num);
		virtual int getRobotID(void);
		virtual dJointID getMotorID(int id);
		virtual double getPosition(int body, int i);
		virtual double getRotation(int body, int i);
		virtual bool getSuccess(int i);
		virtual int getType(void);
		virtual bool isHome(void);
		virtual int isShiftEnabled(void);
		virtual int setID(int id);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

		// private functions
		int add_connector(int type, int face, double size);									// add connector to robot
		int add_daisy_chain(int conn, int side, double size, int face, int type);			// add daisy chained connector
		int build_individual(double x, double y, double z, dMatrix3 R,						// build individual robot
							 double r_f1, double r_f2, double r_f3);
		int build_attached(xml_robot_t robot, CRobot *base, xml_conn_t conn);				// build rotated and attached robot
		int build_body(double x, double y, double z, dMatrix3 R, double theta);				// build body of mobot
		int build_face(int id, double x, double y, double z, dMatrix3 R, double theta);		// build face of mobot
		int build_bigwheel(conn_t conn, int face, int side = -1, int type = -1);			// build big wheel
		int build_bridge(conn_t conn, int face, int side = -1, int type = -1);				// build bridge
		int build_caster(conn_t conn, int face, int side = -1, int type = -1);				// build caster
		int build_cube(conn_t conn, int face, int side = -1, int type = -1);				// build cube
		int build_faceplate(conn_t conn, int face, int side = -1, int type = -1);			// build faceplate connector
		int build_gripper(conn_t conn, int face);											// build gripper
		int build_omnidrive(conn_t conn, int face, int side = -1, int type = -1);			// build omnidrive plate
		int build_simple(conn_t conn, int face, int side = -1, int type = -1);				// build simple connector
		int build_smallwheel(conn_t conn, int face, int side = -1, int type = -1);			// build small wheel
		int build_tinywheel(conn_t conn, int face, int side = -1, int type = -1);			// build tiny wheel
		int build_wheel(conn_t conn, int face, double size, int side = -1, int type = -1);	// build tiny wheel
		int fix_body_to_connector(dBodyID cBody, int face);									// fix second body to connector
		int fix_body_to_ground(dBodyID cbody);												// fix body to ground
		int fix_connector_to_body(dBodyID rBody, dBodyID cBody);							// fix connector to robot body
		int get_body_params(double angle, int face, double rotation, dMatrix3 R, double *p);// get parameters for attaching robot
		int get_connector_params(int type, int side, dMatrix3 R, double *p);				// get parameters of connector
		int init_params(int disabled, int type);											// initialize robot parameters
		int init_dims(void);																// initialize robot dimensions
		int get_error(void);																// get position error
		double mod_angle(double past_ang, double cur_ang, double ang_rate);					// modify angle to count continuously
		static void* motionDistanceThread(void *arg);										// thread to run motion distance
		static void* motionRollBackwardThread(void *arg);									// thread to run motion roll backward
		static void* motionRollForwardThread(void *arg);									// thread to run motion roll forward
		static void* motionTurnLeftThread(void *arg);										// thread to run motion turn left
		static void* motionTurnRightThread(void *arg);										// thread to run motion turn right
		static void* movexyThread(void *arg);												// thread to run movexy
		static void* movexyFuncThread(void *arg);											// thread to run movexyFunc
		static void* movexyPolyThread(void *arg);											// thread to run movexyPoly
		static void* movexyToThread(void *arg);												// thread to run movexy
		static void* recordAngleThread(void *arg);											// thread to record angle
		static void* recordAngleBeginThread(void *arg);										// thread to record angle
		static void* recordAnglesThread(void *arg);											// thread to record angles
		static void* recordAnglesBeginThread(void *arg);									// thread to record angles
		static void* setMovementStateTimeNBThread(void *arg);								// thread to set movement state
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group *root, int tracking);
		void draw_bigwheel(conn_t conn, osg::Group *robot);
		void draw_bridge(conn_t conn, osg::Group *robot);
		void draw_caster(conn_t conn, osg::Group *robot);
		void draw_cube(conn_t conn, osg::Group *robot);
		void draw_faceplate(conn_t conn, osg::Group *robot);
		void draw_gripper(conn_t conn, osg::Group *robot);
		void draw_omnidrive(conn_t conn, osg::Group *robot);
		void draw_simple(conn_t conn, osg::Group *robot);
		void draw_smallwheel(conn_t conn, osg::Group *robot);
		void draw_tinywheel(conn_t conn, osg::Group *robot);
		void draw_wheel(conn_t conn, osg::Group *robot);
#endif // ENABLE_GRAPHICS
#endif // not _CH_
};

class DLLIMPORT CLinkbotTGroup {
	public:
		CLinkbotTGroup();
		virtual ~CLinkbotTGroup();

		int blinkLED(double delay, int num);
		int addRobot(CLinkbotT& robot);
#ifdef _CH_
		int addRobots(array CLinkbotT robots[], ...);
#else
		int addRobots(CLinkbotT robots[], int num);
#endif
		int connect(void);
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int isMoving();
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setTwoWheelRobotSpeed(double speed, double radius);
		int stop(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int stopAllJoints(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
#ifndef _CH_
	private:
		typedef struct robots_s {
			CLinkbotT *robot;
			struct robots_s *next;
		} *robots_t;
		double _d;
		int _i;
		int _motion;
		robots_t _robots;
		THREAD_T *_thread;

		static void* motionDistanceThread(void*);
		static void* motionRollBackwardThread(void*);
		static void* motionRollForwardThread(void*);
		static void* motionTurnLeftThread(void*);
		static void* motionTurnRightThread(void*);
#endif // not _CH_
};

#ifdef _CH_
class DLLIMPORT CLinkbotI {
	public:
		CLinkbotI();
		virtual ~CLinkbotI();

		int blinkLED(double delay, int num);
		int connect();
		int delay(double milliseconds);
		int delaySeconds(double seconds);
		int disableRecordDataShift();
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int enableRecordDataShift();
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getLEDColor(string_t &color);
		int getLEDColorName(char color[]);
		int getLEDColorRGB(int &r, int &g, int &b);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getID();
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngleAverage(robotJointId_t id, double &angle, ... );
		int getJointAngles(double &angle1, double &angle2, double &angle3);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int getxy(double &x, double &y);
		int isConnected();
		int isMoving();
		int line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color);
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
		int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
		int movexy(double x, double y, double radius, double trackwidth);
		int movexyNB(double x, double y, double radius, double trackwidth);
		int movexyExpr(double x0, double xf, int n, char *expr, double radius, double trackwidth);
		int movexyExprNB(double x0, double xf, int n, char *expr, double radius, double trackwidth);
		int movexyFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int movexyFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth);
		int movexyPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int movexyPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth);
		int movexyTo(double x, double y, double radius, double trackwidth);
		int movexyToNB(double x, double y, double radius, double trackwidth);
		int movexyWait();
		int point(double x, double y, double z, int pointsize, char *color);
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[:], double angle1[:], double angle2[:], double angle3[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3, double seconds, ...);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistanceOffset(double distance);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3, double radius, double seconds, ...);
		int recordDistancesEnd(int &num);
		int recordWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOn(int frequency);
		int setBuzzerFrequencyOff();
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setTwoWheelRobotSpeed(double speed, double radius);
		int stop(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int stopAllJoints(void);
		int systemTime(double &time);
		int text(double x, double y, double z, char *text);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};
class DLLIMPORT CLinkbotIGroup {
	public:
		CLinkbotIGroup();
		virtual ~CLinkbotIGroup();

		int addRobot(CLinkbotI& robot);
		int addRobots(array CLinkbotI robots[], ...);
		int blinkLED(double delay, int num);
		int connect(void);
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int isMoving();
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setTwoWheelRobotSpeed(double speed, double radius);
		int stop(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int stopAllJoints(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};

class DLLIMPORT CLinkbotL {
	public:
		CLinkbotL();
		virtual ~CLinkbotL();

		int blinkLED(double delay, int num);
		int connect();
		int delay(double milliseconds);
		int delaySeconds(double seconds);
		int disableRecordDataShift();
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int enableRecordDataShift();
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getLEDColor(string_t &color);
		int getLEDColorName(char color[]);
		int getLEDColorRGB(int &r, int &g, int &b);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getID();
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngleAverage(robotJointId_t id, double &angle, ... );
		int getJointAngles(double &angle1, double &angle2, double &angle3);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int getxy(double &x, double &y);
		int isConnected();
		int isMoving();
		int line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color);
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
		int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
		int movexy(double x, double y, double radius, double trackwidth);
		int movexyNB(double x, double y, double radius, double trackwidth);
		int movexyTo(double x, double y, double radius, double trackwidth);
		int movexyToNB(double x, double y, double radius, double trackwidth);
		int movexyWait();
		int point(double x, double y, double z, int pointsize, char *color);
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[:], double angle1[:], double angle2[:], double angle3[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3, double seconds, ...);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistanceOffset(double distance);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3, double radius, double seconds, ...);
		int recordDistancesEnd(int &num);
		int recordWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOn(int frequency);
		int setBuzzerFrequencyOff();
		int setLEDColor(char *color);
		int setLEDColorRGB(int r, int g, int b);
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setTwoWheelRobotSpeed(double speed, double radius);
		int stop(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int stopAllJoints(void);
		int systemTime(double &time);
		int text(double x, double y, double z, char *text);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};
class DLLIMPORT CLinkbotLGroup {
	public:
		CLinkbotLGroup();
		virtual ~CLinkbotLGroup();

		int addRobot(CLinkbotL& robot);
		int addRobots(array CLinkbotL robots[], ...);
		int blinkLED(double delay, int num);
		int connect(void);
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngleAverage(robotJointId_t id, double &angle, ... );
		int getJointAngles(double &angle1, double &angle2, double &angle3);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int isMoving();
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setTwoWheelRobotSpeed(double speed, double radius);
		int stop(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int stopAllJoints(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
};
void* RoboSim::_dlhandle = NULL;
int RoboSim::_dlcount = 0;
#pragma importf "chlinkbotisim.chf"
#pragma importf "chlinkbotlsim.chf"
#pragma importf "chlinkbottsim.chf"
#else
typedef struct motionArg_s {
	int i;
	double d;
	CLinkbotT *robot;
} motionArg_t;
typedef struct moveArg_s {
	double x, y, radius, trackwidth;
	int i;
	double (*func)(double x);
	char *expr;
	CLinkbotT *robot;
} moveArg_t;
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
extern RoboSim *_simObject;
#endif // _CH_
#endif // LINKBOT_H_
