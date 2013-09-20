#ifndef BASE_H_
#define BASE_H_

#ifndef _CH_
#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#include <Shlwapi.h>
#define DLLIMPORT __declspec(dllexport)
#define TEXTURE_PATH(end) "C:/Ch/package/chrobosim/data/" #end
#else
#include <pthread.h>
#include <unistd.h>
#define DLLIMPORT
#define TEXTURE_PATH(end) "/usr/local/ch/package/chrobosim/data/" #end
#endif // _WIN32
#include "config.h"
#include <ode/ode.h>
#include <cctype>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS
#else
#define DLLIMPORT
#pragma package <chrobosim>
#define dDOUBLE
#define dReal double
#define ENABLE_GRAPHICS
extern void delay(double seconds);
#endif // no _CH_

#define EPSILON DBL_EPSILON
#define RECORD_ANGLE_ALLOC_SIZE 16
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define angle2distance(radius, angle) ((radius) * (angle * 0.01745329251994329547))
#define distance2angle(radius, distance) (((distance)/(radius))*57.29577951308232286465)

#ifdef _WIN32
//   THREADS
#define THREAD_T HANDLE
#define THREAD_CANCEL(thread_handle) TerminateThread( thread_handle, 0)
#define THREAD_CREATE(thread_handle, function, arg) \
	*(thread_handle) = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)function, arg, 0, NULL)
#define THREAD_JOIN(thread_handle) WaitForSingleObject(thread_handle, INFINITE)
//   MUTEX
#define MUTEX_T HANDLE
#define MUTEX_INIT(mutex) *mutex = CreateMutex(NULL, FALSE, NULL)
#define MUTEX_DESTROY(mutex)
#define MUTEX_LOCK(mutex) WaitForSingleObject(*mutex, INFINITE)
#define MUTEX_UNLOCK(mutex) ReleaseMutex(*mutex)
//   COND
#define COND_T HANDLE
#define COND_INIT(cond) *cond = CreateEvent(NULL, TRUE, TRUE, NULL); ResetEvent(*cond)
#define COND_DESTROY(cond)
#define COND_WAIT(cond , mutex) ResetEvent(*cond); ReleaseMutex(*mutex); WaitForSingleObject(*cond, INFINITE)
#define SIGNAL(cond, mutex, action) action; SetEvent(*cond)
#define COND_SIGNAL(cond) SetEvent(*cond)
#else
//   THREADS
#define THREAD_T pthread_t
#define THREAD_CANCEL(thread_handle) pthread_cancel(thread_handle)
#define THREAD_CREATE(thread_handle, function, arg) \
	while (pthread_create(thread_handle, NULL, function, (void*) arg) < 0) { \
		fprintf(stderr, "pthread_create failed. Trying again...\n"); \
	}
#define THREAD_JOIN(thread_handle) pthread_join(thread_handle, NULL)
//   MUTEX
#define MUTEX_T pthread_mutex_t
#define MUTEX_INIT(mutex) pthread_mutex_init(mutex, NULL)
#define MUTEX_DESTROY(mutex) pthread_mutex_destroy(mutex)
#define MUTEX_LOCK(mutex) \
	if (pthread_mutex_lock(mutex)) { \
		fprintf(stderr, "pthread lock error: %s:%d\n", __FILE__, __LINE__); \
	}
#define MUTEX_UNLOCK(mutex) pthread_mutex_unlock(mutex)
//   COND
#define COND_T pthread_cond_t
#define COND_INIT(cond) pthread_cond_init(cond, NULL)
#define COND_DESTROY(cond) pthread_cond_destroy(cond)
#define COND_WAIT(cond, mutex) pthread_cond_wait(cond, mutex)
#define SIGNAL(cond, mutex, action) pthread_mutex_lock(mutex); action; pthread_cond_signal(cond); pthread_mutex_unlock(mutex)
#define COND_SIGNAL(cond) pthread_cond_signal(cond)
//   RWLOCK
#define RWLOCK_T pthread_rwlock_t
#define RWLOCK_INIT(rwlock) pthread_rwlock_init(rwlock, NULL)
#define RWLOCK_DESTROY(rwlock) pthread_rwlock_destroy(rwlock)
#define RWLOCK_RLOCK(rwlock) \
	if (pthread_rwlock_rdlock(rwlock)) { \
		fprintf(stderr, "rwlock error: %s:%d\n", __FILE__, __LINE__); \
	}
#define RWLOCK_RUNLOCK(rwlock) \
	if (pthread_rwlock_unlock(rwlock)) { \
		fprintf(stderr, "rwunlock error: %s:%d\n", __FILE__, __LINE__); \
	}
#define RWLOCK_WLOCK(rwlock) \
	if (pthread_rwlock_wrlock(rwlock)) { \
		fprintf(stderr, "rwlock error: %s:%d\n", __FILE__, __LINE__); \
	}
#define RWLOCK_WUNLOCK(rwlock) \
	if (pthread_rwlock_unlock(rwlock)) { \
		fprintf(stderr, "rwunlock error: %s:%d\n", __FILE__, __LINE__); \
	}
#endif

typedef enum robot_type_e {
	MOBOT,
	LINKBOTI,
	LINKBOTL,
	LINKBOTT,
	NUM_TYPES
} robotType_t;
typedef enum robot_joint_id_e {
	ROBOT_JOINT1,
	ROBOT_JOINT2,
	ROBOT_JOINT3,
	ROBOT_JOINT4
} robotJointId_t;
typedef enum robot_joint_state_e {
	ROBOT_NEUTRAL = 0,
	ROBOT_FORWARD,
	ROBOT_BACKWARD,
	ROBOT_HOLD,
	ROBOT_POSITIVE,
#ifndef _CH_
	ROBOT_NEGATIVE,
	NaN = 0
#else
	ROBOT_NEGATIVE
#endif
} robotJointState_t;
typedef enum robot_connector_e {
	BIGWHEEL,
	BRIDGE,
	CASTER,
	CUBE,
	FACEPLATE,
	GRIPPER,
	L,
	SIMPLE,
	SMALLWHEEL,
	SQUARE,
	TANK,
	NUM_CONNECTORS
} robotConnector_t;

// connector
typedef struct xml_conn_s {
	int robot, type, side, face1, face2, conn;
	struct xml_conn_s *next;
} xml_conn_t;
// robot
typedef struct xml_robot_s {
	int type;
	int id;
	double x, y, z;
	double psi, theta, phi;
	double angle1, angle2, angle3, angle4;
	struct xml_conn_s *conn;
	struct xml_robot_s *next;
} *xml_robot_t;

typedef double* robotRecordData_t;

#ifndef _CH_
class DLLIMPORT CRobot {
	public:
		CRobot();
		~CRobot();

		// entry functions for pre- and post-collision updates
		static void* simPreCollisionThreadEntry(void *arg);
		static void* simPostCollisionThreadEntry(void *arg);

		double uniform(void);
		double normal(double sigma);
		int noisy(double *a, int length, double sigma);

		// pure virtual functions to be overridden by inherited classes of each robot
		virtual int addToSim(dWorldID &world, dSpaceID &space, dReal *clock) = 0;
		virtual int build(xml_robot_t robot) = 0;
		virtual int build(xml_robot_t robot, CRobot *base, xml_conn_t *conn) = 0;
		virtual bool getSuccess(int i) = 0;
		virtual int getType(void) = 0;
		virtual dBodyID getBodyID(int body) = 0;
		virtual dBodyID getConnectorBodyID(int face) = 0;
		virtual dBodyID getConnectorBodyIDs(int num) = 0;
		virtual int getConnectionParams(int face, dMatrix3 R, dReal *p) = 0;
		virtual int getRobotID(void) = 0;
		virtual dJointID getMotorID(int motor) = 0;
		virtual dReal getAngle(int i) = 0;
		virtual dReal getPosition(int body, int i) = 0;
		virtual dReal getRotation(int body, int i) = 0;
		virtual bool isHome(void) = 0;
		virtual int setID(int id) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;

		// recording angles struct
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

		bool _motion;			// motion in progress
		bool *_recording;		// recording in progress
		bool *_rec_active;		// actively recording a new value
		bool *_seek;			// currently seeking goal?
		conn_t _conn;			// connectors
		dBodyID *_body;			// body parts
		dGeomID **_geom;		// geometries of each body part
		dJointID *_joint;		// joints between body parts
		dJointID *_motor;		// motors
		dReal *_clock;			// world clock
		dReal *_angle;			// angles
		dReal *_goal;			// goals
		dReal *_max_force;		// maximum joint forces
		dReal *_max_speed;		// maximum joint speeds
		dReal *_speed;			// speed
		dSpaceID _space;		// space for this robot
		dWorldID _world;		// world for all robots
		double _accel[3];		// accelerometer data
		double _encoder;		// motor encoder resolution
		double	_center_length, _center_width, _center_height, _center_radius, _center_offset,
				_body_length, _body_width, _body_height, _body_radius,
				_body_inner_width_left, _body_inner_width_right, _body_end_depth, _body_mount_center,
				_end_width, _end_height, _end_depth, _end_radius,
				_face_depth, _face_radius;
		double	_connector_depth, _connector_height, _connector_radius,
				_bigwheel_radius, _smallwheel_radius,
				// mobot
				_tank_height, _tank_depth,
				// linkbot
				_cubic_length, _bridge_length;
		double _safety_angle;	// joint safety angle
		double _safety_timeout;	// joint safety timeout
		double *_offset;		// offset from zero for resetting
		double _radius;			// wheel radius
		double ***_rec_angles;	// recorded angles from thread
		double _rgb[3];			// rgb of 'led'
		int _connected; 		// connected to controller
		int _disabled;			// which joint is disabled
		int *_enabled;			// list of enabled motors
		int _id;				// robot id
		int *_rec_num;			// recording data points
		int _seed;				// seed for random number generation
		int *_state;			// joint states
		int *_success;			// trigger for goal
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
		virtual void draw(osg::Group *root) {};
		osg::ShapeDrawable *_led;
#endif // ENABLE_GRAPHICS
};
#endif // not _CH_
#endif // BASE_H_
