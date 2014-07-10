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
#include "rgbhashtable.h"
#include "config.h"
#include "macros.h"

#ifdef ENABLE_GRAPHICS
#include <osg/Group>
#include <osg/ShapeDrawable>
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CRobot {
	public:
		CRobot();
		~CRobot();

		// entry functions for pre- and post-collision updates
		static void* simPreCollisionThreadEntry(void *arg);
		static void* simPostCollisionThreadEntry(void *arg);

		// public functions
		double normal(double sigma);
		double uniform(void);
		int noisy(double *a, int length, double sigma);
		void doze(double ms);

		// pure virtual functions to be overridden by inherited classes of each robot
		virtual int addToSim(dWorldID &world, dSpaceID &space) = 0;
		virtual int build(xml_robot_t robot) = 0;
		virtual int build(xml_robot_t robot, CRobot *base, xml_conn_t conn) = 0;
		virtual bool getSuccess(int i) = 0;
		virtual int getType(void) = 0;
		virtual dBodyID getBodyID(int body) = 0;
		virtual dBodyID getConnectorBodyID(int face) = 0;
		virtual dBodyID getConnectorBodyIDs(int num) = 0;
		virtual int getConnectionParams(int face, dMatrix3 R, double *p) = 0;
		virtual int getRobotID(void) = 0;
		virtual dJointID getMotorID(int motor) = 0;
		virtual double getAngle(int i) = 0;
		virtual double getAngularRate(int i) = 0;
		virtual double getCenter(int i) = 0;
		virtual double getPosition(int body, int i) = 0;
		virtual double getRotation(int body, int i) = 0;
		virtual bool isHome(void) = 0;
		virtual int isShiftEnabled(void) = 0;
		virtual int setID(int id) = 0;
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
		// connector
		typedef struct conn_s {
			int face, type;
			dBodyID body;
			dGeomID *geom;
			struct conn_s *next;
		} *conn_t;
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
			int mode_timeout;		// mode timeout
			int starting;			// starting movement
			int stopping;			// stopping movement
			int state;				// state
			accel_t accel;			// acceleration variables
		} *motor_t;

		conn_t _conn;			// connectors
		dBodyID *_body;			// body parts
		dGeomID **_geom;		// geometries of each body part
		dJointID *_joint;		// joints between body parts
		dSpaceID _space;		// space for this robot
		dWorldID _world;		// world for all robots
		motor_t _motor;			// motors
		bool _motion;			// motion in progress
		bool *_recording;		// recording in progress
		bool *_rec_active;		// actively recording a new value
		double _accel[3];		// accelerometer data
		double _center[3];		// offset of body from initial (x,y,z)
		double _distOffset;		// offset for recorded distance
		double _radius;			// wheel radius
		double ***_rec_angles;	// recorded angles from thread
		double _rgb[3];			// rgb of 'led'
		double _trackwidth;		// trackwidth of robot
		double	_center_length, _center_width, _center_height, _center_radius, _center_offset,
				_body_length, _body_width, _body_height, _body_radius,
				_body_inner_width_left, _body_inner_width_right, _body_end_depth, _body_mount_center,
				_end_width, _end_height, _end_depth, _end_radius,
				_face_depth, _face_radius;
		double	_connector_depth, _connector_height, _connector_radius,
				_bigwheel_radius, _smallwheel_radius, _wheel_depth, _wheel_radius,
				// mobot
				_tank_height, _tank_depth,
				// linkbot
				_bridge_length, _cubic_length, _omni_length, _tinywheel_radius;
		int _connected;			// connected to controller
		int _disabled;			// which joint is disabled
		int *_enabled;			// list of enabled motors
		int _id;				// robot id
		int *_rec_num;			// recording data points
		int _seed;				// seed for random number generation
		int _shift_data;		// shift recorded data or not
		int _g_shift_data;		// globally shift data for robot
		int _g_shift_data_en;	// globally shift data for robot enable/disable flag
		int _trace;				// tracing on or off
		int _type;				// type of robot

		// threading locks for each robot
		MUTEX_T _angle_mutex;
		MUTEX_T _goal_mutex;
		MUTEX_T _motion_mutex;
		COND_T _motion_cond;
		MUTEX_T _recording_mutex;
		COND_T _recording_cond;
		MUTEX_T _active_mutex;
		COND_T _active_cond;
		MUTEX_T _success_mutex;
		COND_T _success_cond;

#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group *root, int tracking) = 0;
		osg::Group *_robot;
		osg::ShapeDrawable *_led;
#endif // ENABLE_GRAPHICS
};
#endif // BASE_H_
