#ifndef BASE2_H_
#define BASE2_H_

#ifndef _CH_
#ifdef _WIN32
#include <windows.h>
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
#include <osg/Group>
#endif // ENABLE_GRAPHICS
#else
#pragma package <chrobotsim>
#define dDOUBLE
#define dReal double
#define ENABLE_GRAPHICS
#endif // no _CH_

#ifdef _WIN32
#ifdef _CH_
#define DLLIMPORT
#else
#define DLLIMPORT __declspec(dllexport)
#endif // _CH_
#else
#define DLLIMPORT
#endif // _WIN32

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

#ifdef _WIN32
//   THREADS
#ifndef THREAD_T
#define THREAD_T HANDLE
#endif
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
#ifndef _CH_
#include <pthread.h>
#include <unistd.h>
#endif
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

// types of robots available for simulation
enum robot_types_e {
	MOBOT,
	LINKBOT,
	LINKBOTI,
	LINKBOTL,
	NUM_TYPES
};

// connector
typedef struct Conn_s {
	int robot, type, side, face1, face2;
	struct Conn_s *next;
} Conn_t;
// robot
typedef struct bot_s {
	int type;
	int id;
	double x, y, z;
	double psi, theta, phi;
	double angle1, angle2, angle3, angle4;
	struct Conn_s *conn;
	struct bot_s *next;
} *bot_t;

class DLLIMPORT CRobot {
	public:
		CRobot();
		~CRobot();

		// entry functions for pre- and post-collision updates
		static void* simPreCollisionThreadEntry(void *arg);
		static void* simPostCollisionThreadEntry(void *arg);

#ifndef _CH_
		// pure virtual functions to be overridden by inherited classes of each robot
		virtual int addToSim(dWorldID &world, dSpaceID &space, dReal *clock) = 0;
		virtual int build(bot_t robot) = 0;
		virtual int build(bot_t robot, CRobot *base, Conn_t *conn) = 0;
		virtual bool getSuccess(int i) = 0;
		virtual int getID(void) = 0;
		virtual int getType(void) = 0;
		virtual dBodyID getBodyID(int body) = 0;
		virtual dBodyID getConnectorBodyID(int face) = 0;
		virtual dBodyID getConnectorBodyIDs(int num) = 0;
		virtual int getConnectionParams(int face, dMatrix3 R, dReal *p) = 0;
		virtual dJointID getMotorID(int motor) = 0;
		virtual dReal getAngle(int i) = 0;
		virtual dReal getPosition(int body, int i) = 0;
		virtual dReal getRotation(int body, int i) = 0;
		virtual bool isHome(void) = 0;
		virtual int setID(int id) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;
#ifdef ENABLE_GRAPHICS
		virtual void draw(osg::Group *root) = 0;
#endif // ENABLE_GRAPHICS

		// threading locks for each robot
		MUTEX_T _angle_mutex;
		MUTEX_T _goal_mutex;
		//RWLOCK_T _goal_rwlock;
		MUTEX_T _recording_mutex;
		COND_T _recording_cond;
		MUTEX_T _success_mutex;
		COND_T _success_cond;
#endif
};

#endif // BASE2_H_
