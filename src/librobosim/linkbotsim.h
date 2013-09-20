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
//#include "pid.h"
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
		int connect();
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getColorRGB(int &r, int &g, int &b);
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
		int isConnected(void);
		int isMoving(void);
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
		int recordDistancesEnd(int &num);
		int recordWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOn(int frequency);
		int setBuzzerFrequencyOff();
		int setColorRGB(int r, int g, int b);
		int setExitState(robotJointState_t exitState);
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
		int turnLeft(double angle, double radius, double tracklength);
		int turnLeftNB(double angle, double radius, double tracklength);
		int turnRight(double angle, double radius, double tracklength);
		int turnRightNB(double angle, double radius, double tracklength);
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
		virtual int addToSim(dWorldID &world, dSpaceID &space, dReal *clock);
		virtual int build(xml_robot_t robot);
		virtual int build(xml_robot_t robot, CRobot *base, xml_conn_t *conn);
		virtual dReal getAngle(int i);
		virtual dBodyID getBodyID(int id);
		virtual int getConnectionParams(int face, dMatrix3 R, dReal *p);
		virtual dBodyID getConnectorBodyID(int face);
		virtual dBodyID getConnectorBodyIDs(int num);
		virtual int getRobotID(void);
		virtual dJointID getMotorID(int id);
		virtual dReal getPosition(int body, int i);
		virtual dReal getRotation(int body, int i);
		virtual bool getSuccess(int i);
		virtual int getType(void);
		virtual bool isHome(void);
		virtual int setID(int id);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

		// private functions
		int add_connector(int type, int face);										// add connector to robot
		int add_daisy_chain(int conn, int side, int face, int type);				// add daisy chained connector
		int build_individual(dReal x, dReal y, dReal z, dMatrix3 R,					// build individual robot
							 dReal r_f1, dReal r_f2, dReal r_f3);
		int build_attached(xml_robot_t robot, CRobot *base, xml_conn_t *conn);				// build rotated and attached robot
		int build_body(dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);			// build body of mobot
		int build_face(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);	// build face of mobot
		int build_bigwheel(conn_t conn, int face, int side = -1, int type = -1);	// build big wheel
		int build_bridge(conn_t conn, int face, int side = -1, int type = -1);		// build bridge
		int build_caster(conn_t conn, int face, int side = -1, int type = -1);		// build caster
		int build_cube(conn_t conn, int face, int side = -1, int type = -1);		// build cube
		int build_faceplate(conn_t conn, int face, int side = -1, int type = -1);	// build faceplate connector
		int build_gripper(conn_t conn, int face);									// build gripper
		int build_simple(conn_t conn, int face, int side = -1, int type = -1);		// build simple connector
		int build_smallwheel(conn_t conn, int face, int side = -1, int type = -1);	// build small wheel
		int fix_body_to_connector(dBodyID cBody, int face);							// fix second body to connector
		int fix_connector_to_body(int face, dBodyID cBody);							// fix connector to robot body
		int get_body_params(double angle, int face, double rotation, dMatrix3 R, double *p);	// get parameters for attaching robot
		int get_connector_params(int type, int side, dMatrix3 R, double *p);		// get parameters of connector
		int init_params(int disabled, int type);									// initialize robot parameters
		int init_dims(void);														// initialize robot dimensions
		dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate);				// modify angle to count continuously
        //void resetPID(int i = NUM_DOF);											// reset PID controller
		static void* motionDistanceThread(void *arg);								// thread to run motion distance
		static void* motionRollBackwardThread(void *arg);							// thread to run motion roll backward
		static void* motionRollForwardThread(void *arg);							// thread to run motion roll forward
		static void* motionTurnLeftThread(void *arg);								// thread to run motion turn left
		static void* motionTurnRightThread(void *arg);								// thread to run motion turn right
		static void* recordAngleThread(void *arg);									// thread to record angle
		static void* recordAngleBeginThread(void *arg);								// thread to record angle
		static void* recordAnglesThread(void *arg);									// thread to record angles
		static void* recordAnglesBeginThread(void *arg);							// thread to record angles
		static void* setMovementStateTimeNBThread(void *arg);						// thread to set movement state
#ifdef ENABLE_GRAPHICS
		virtual void draw(osg::Group *root);
		void draw_bigwheel(conn_t conn, osg::Group *robot);
		void draw_bridge(conn_t conn, osg::Group *robot);
		void draw_caster(conn_t conn, osg::Group *robot);
		void draw_cube(conn_t conn, osg::Group *robot);
		void draw_faceplate(conn_t conn, osg::Group *robot);
		void draw_gripper(conn_t conn, osg::Group *robot);
		void draw_simple(conn_t conn, osg::Group *robot);
		void draw_smallwheel(conn_t conn, osg::Group *robot);
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
		int turnLeft(double angle, double radius, double tracklength);
		int turnLeftNB(double angle, double radius, double tracklength);
		int turnRight(double angle, double radius, double tracklength);
		int turnRightNB(double angle, double radius, double tracklength);
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
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getColorRGB(int &r, int &g, int &b);
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
		int isConnected();
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
		int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
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
		int setColorRGB(int r, int g, int b);
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
		int turnLeft(double angle, double radius, double tracklength);
		int turnLeftNB(double angle, double radius, double tracklength);
		int turnRight(double angle, double radius, double tracklength);
		int turnRightNB(double angle, double radius, double tracklength);
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
		int turnLeft(double angle, double radius, double tracklength);
		int turnLeftNB(double angle, double radius, double tracklength);
		int turnRight(double angle, double radius, double tracklength);
		int turnRightNB(double angle, double radius, double tracklength);
};

class DLLIMPORT CLinkbotL {
	public:
		CLinkbotL();
		virtual ~CLinkbotL();

		int blinkLED(double delay, int num);
		int connect();
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getColorRGB(int &r, int &g, int &b);
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
		int isConnected();
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
		int setColorRGB(int r, int g, int b);
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
		int turnLeft(double angle, double radius, double tracklength);
		int turnLeftNB(double angle, double radius, double tracklength);
		int turnRight(double angle, double radius, double tracklength);
		int turnRightNB(double angle, double radius, double tracklength);
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
		int turnLeft(double angle, double radius, double tracklength);
		int turnLeftNB(double angle, double radius, double tracklength);
		int turnRight(double angle, double radius, double tracklength);
		int turnRightNB(double angle, double radius, double tracklength);
};
#ifndef ROBOSIM_DLHANDLE
#define ROBOSIM_DLHANDLE
void* RoboSim::g_chrobosim_dlhandle = NULL;
int RoboSim::g_chrobosim_dlcount = 0;
#pragma importf "chrobosim.chf"
#endif
#pragma importf "chlinkbottsim.chf"
#pragma importf "chlinkbotisim.chf"
#pragma importf "chlinkbotlsim.chf"
#else
typedef struct motionArg_s {
	int i;
	double d;
	CLinkbotT *robot;
} motionArg_t;
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
