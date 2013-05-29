#include "base.h"

CRobot::CRobot(void) {
}

CRobot::~CRobot(void) {
	// destory locks
	MUTEX_DESTROY(&_angle_mutex);
	RWLOCK_DESTROY(&_goal_rwlock);
	MUTEX_DESTROY(&_recording_mutex);
	COND_DESTROY(&_recording_cond);
	MUTEX_DESTROY(&_success_mutex);
	COND_DESTROY(&_success_cond);
}

void* CRobot::simPreCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPreCollisionThread();
}

void* CRobot::simPostCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPostCollisionThread();
}

