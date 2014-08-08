#ifndef BASE_H_
#define BASE_H_

#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#include <Shlwapi.h>
#else
#include <pthread.h>
#include <unistd.h>
#endif // _WIN32

#include <ode/ode.h>
#include <cctype>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <ctime>
#include "config.h"
#include "macros.h"
#include "rgbhashtable.h"

#ifdef ENABLE_GRAPHICS
#include <osg/Group>
#include <osg/ShapeDrawable>
#include "graphics.h"
#endif // ENABLE_GRAPHICS

// connector
typedef struct conn_s {
	int face, type;
	int d_side, d_type;
	dBodyID body;
	dGeomID *geom;
	struct conn_s *next;
} *conn_t;

class linkbotNodeCallback;
class mobotNodeCallback;
class RoboSim;

class DLLIMPORT CRobot {
		friend class linkbotNodeCallback;
		friend class mobotNodeCallback;
		friend class RoboSim;

	public:
		CRobot(void);
		~CRobot(void);

		int blinkLED(double delay, int num);
		int connect(char *name = NULL, int pause = 3);
		int delay(double milliseconds);
		int delaySeconds(double seconds);
		int disableRecordDataShift(void);
		int disconnect(void);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getBatteryVoltage(double &voltage);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getID(void);
		int getJointAngle(robotJointId_t id, double &angle, int numReadings = 10);
		int getJointAngleInstant(robotJointId_t id, double &angle);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getxy(double &x, double &y);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);
		int jumpJointToNB(robotJointId_t id, double angle);
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
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngle(robotJointId_t, double[], double[], int, double, int = 1);
		int recordAngleBegin(robotJointId_t, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordAngleEnd(robotJointId_t, int&);
		int recordDistanceBegin(robotJointId_t, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int recordDistanceEnd(robotJointId_t, int&);
		int recordDistanceOffset(double);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordxyEnd(int&);
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
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int systemTime(double &time);
		int traceOff(void);
		int traceOn(void);

		// TODO: make private-ish functions protected
		dBodyID getConnectorBodyID(int face);
		virtual int getConnectionParams(int face, dMatrix3 R, double *p) = 0;

	protected:
		dBodyID getBodyID(int body);
		dBodyID getConnectorBodyIDs(int num);
		dJointID getMotorID(int motor);
		double getAngle(int id);
		double getCenter(int i);
		double getRotation(int body, int i);
		int addToSim(dWorldID &world, dSpaceID &space);
		int doze(double ms);
		int fixBodyToGround(dBodyID cbody);
		int getConnectorParams(int type, int side, dMatrix3 R, double *p);
		int isShiftEnabled(void);
		int noisy(double *a, int length, double sigma);
		int setID(int id);
		static void* simPreCollisionThreadEntry(void *arg);
		static void* simPostCollisionThreadEntry(void *arg);

		// pure virtual functions to be overridden by inherited classes of each robot
		virtual int build(xml_robot_t robot) = 0;
		virtual int build(xml_robot_t robot, CRobot *base, xml_conn_t conn) = 0;
		virtual int buildIndividual(double x, double y, double z, dMatrix3 R, double *rot) = 0;
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group *root, int tracking) = 0;
#endif // ENABLE_GRAPHICS
		virtual int initParams(int disabled, int type) = 0;
		virtual int initDims(void) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;

		typedef enum robot_joint_state_e {
			NEUTRAL = 0,
			HOLD,
			POSITIVE,
			NEGATIVE,
		} robotJointState_t;
		typedef enum robot_motor_mode_e {
			ACCEL_CONSTANT = 0,
			ACCEL_CYCLOIDAL,
			ACCEL_HARMONIC,
			CONTINUOUS,
			SEEK,
		} robotMotorMode_t;

		// recording
		typedef struct recordAngleArg_s {
			CRobot *robot;
			robotJointId_t id;
			int num;
			int msecs;
			double *time;
			double **ptime;
			double *angle1;
			double **pangle1;
			double *angle2;
			double **pangle2;
			double *angle3;
			double **pangle3;
			double *angle4;
			double **pangle4;
		} recordAngleArg_t;
		// motor accelerations
		typedef struct accel_s {
			double init;			// motion initial angle
			double start;			// motion start time
			double period;			// motion period
			double run;				// number of motions
		} accel_t;
		// motor
		typedef struct motor_s {
			bool success;			// trigger for motion completion
			dJointID id;			// motors
			double alpha;			// angular acceleration
			double encoder;			// encoder resolution
			double goal;			// goal theta value
			double offset;			// offset from zero for resetting
			double omega;			// angular rate
			double omega_max;		// maximum rate
			double safety_angle;	// safety angle
			double safety_timeout;	// safety timeout
			double tau_max;			// maximum force
			double theta;			// theta
			int mode;				// modes
			int timeout;			// mode timeout
			int starting;			// starting movement
			int stopping;			// stopping movement
			int state;				// state
			accel_t accel;			// acceleration variables
			MUTEX_T success_mutex;
			COND_T success_cond;
		} *motor_t;

		conn_t _conn;				// connectors
		dBodyID *_body;				// body parts
		dGeomID **_geom;			// geometries of each body part
		dJointID *_joint;			// joints between body parts
		dSpaceID _space;			// space for this robot
		dWorldID _world;			// world for all robots
		motor_t _motor;				// motors
		bool _motion;				// motion in progress
		bool *_recording;			// recording in progress
		bool *_rec_active;			// actively recording a new value
		double _accel[3];			// accelerometer data
		double _bigwheel_radius;	// dimension: big wheel radius
		double _body_length;		// dimension: body length
		double _body_height;		// dimension: body height
		double _body_radius;		// dimension: body radius
		double _body_width;			// dimension: body width
		double _center[3];			// offset of body from initial (x,y,z)
		double _conn_depth;			// dimension: connector depth
		double _conn_height;		// dimension: connector height
		double _conn_radius;		// dimension: connector radius
		double _distOffset;			// offset for recorded distance
		double _radius;				// wheel radius
		double ***_rec_angles;		// recorded angles from thread
		double _rgb[3];				// rgb of 'led'
		double _smallwheel_radius;	// dimension: small wheel radius
		double _trackwidth;			// trackwidth of robot
		double _wheel_depth;		// dimension: wheel depth
		double _wheel_radius;		// dimension: custom wheel radius
		int _connected;				// connected to controller
		int _disabled;				// which joint is disabled
		int _dof;					// number of DOF
		int *_enabled;				// list of enabled motors
		int _id;					// robot id
		int *_rec_num;				// recording data points
		int _seed;					// seed for random number generation
		int _shift_data;			// shift recorded data or not
		int _g_shift_data;			// globally shift data for robot
		int _g_shift_data_en;		// globally shift data for robot enable/disable flag
		int _trace;					// tracing on or off
		int _type;					// type of robot

#ifdef ENABLE_GRAPHICS
		osg::Group *_robot;
		osg::ShapeDrawable *_led;
#endif // ENABLE_GRAPHICS

		// threading locks for each robot
		MUTEX_T _active_mutex;
		COND_T _active_cond;
		MUTEX_T _goal_mutex;
		MUTEX_T _motion_mutex;
		COND_T _motion_cond;
		MUTEX_T _recording_mutex;
		COND_T _recording_cond;
		MUTEX_T _success_mutex;
		COND_T _success_cond;
		MUTEX_T _theta_mutex;
	private:
		double mod_angle(double, double, double);		// modify angle for continuous rotation
		double normal(double sigma);					// get random value from normal distribution
		double uniform(void);							// get random value from uniform distribution
		static void* moveJointTimeNBThread(void*);		// thread to move a joint
		static void* moveTimeNBThread(void*);			// thread to move all joints
		static void* recordAngleThread(void*);			// thread to record an angle
		static void* recordAngleBeginThread(void*);		// thread to record an angle indefinitely
		static void* recordxyBeginThread(void*);		// thread to record (x,y) positions
};

#endif // BASE_H_
