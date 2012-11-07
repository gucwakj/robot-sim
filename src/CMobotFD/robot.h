#ifndef ROBOT_H_
#define ROBOT_H_

#include <ode/ode.h>
#include <unistd.h>
#include <pthread.h>

// single read/write lock
typedef struct rw_var {
	bool lock;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
} pthread_rw_t;

class CRobot4Sim;

class robotSim {
	public:
		static void* simPreCollisionThreadEntry(void *arg) { robotSim *p = (robotSim *)arg; p->simPreCollisionThread(); }
		static void* simPostCollisionThreadEntry(void *arg) { robotSim *p = (robotSim *)arg; p->simPostCollisionThread(); }
		
		void simThreadsAngleInit(void);
		void simThreadsAngleLock(void);
		void simThreadsAngleUnlock(void);
		void simThreadsSuccessInit(void);
		void simThreadsSuccessLock(void);
		void simThreadsSuccessUnlock(void);
		void simThreadsSuccessSignal(void);
		void simThreadsSuccessWait(void);

		// pthreads single access r/w attr
		typedef void *pthread_rwattr_t;

		int simThreadsGoalInit(pthread_rwattr_t *attrp);
		int simThreadsGoalRLock(void);
		int simThreadsGoalRUnlock(void);
		int simThreadsGoalWLock(void);
		int simThreadsGoalWUnlock(void);		

		virtual dReal getAngle(int i) = 0;
		virtual bool getSuccess(int i) = 0;
		virtual dReal getPosition(int i) = 0;
		virtual dReal getRotation(int i) = 0;
		virtual dBodyID getBodyID(int body) = 0;
		virtual dJointID getMotorID(int motor) = 0;

		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) = 0;
		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;
		virtual void buildAttached00(CRobot4Sim *attach, int face1, int face2) = 0;
		virtual void buildAttached10(CRobot4Sim *attach, int face1, int face2) = 0;
		virtual void buildAttached01(CRobot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;
		virtual void buildAttached11(CRobot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;

		virtual bool isHome(void) = 0;

		virtual void simAddRobot(dWorldID &world, dSpaceID &space) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;
	private:
		// pthread single access r/w lock functions
		int simThreadsRWInit(pthread_rw_t *rwp, pthread_rwattr_t *attrp);
		int simThreadsRWRLock(pthread_rw_t *rwp);
		int simThreadsRWRUnlock(pthread_rw_t *rwp);
		int simThreadsRWWLock(pthread_rw_t *rwp);
		int simThreadsRWWUnlock(pthread_rw_t *rwp);		
		pthread_mutex_t angle_mutex;
		pthread_rw_t goal_rwlock;
		pthread_mutex_t success_mutex;
		pthread_cond_t success_cond;
};

#endif /* ROBOT_H_ */
