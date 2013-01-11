#include "robot.h"

void* robotSim::simPreCollisionThreadEntry(void *arg) {
	robotSim *p = (robotSim *)arg;
	p->simPreCollisionThread();
}

void* robotSim::simPostCollisionThreadEntry(void *arg) {
	robotSim *p = (robotSim *)arg;
	p->simPostCollisionThread();
}

void robotSim::simThreadsAngleInit(void) {
	pthread_mutex_init(&_angle_mutex, NULL);
}

void robotSim::simThreadsAngleLock(void) {
	pthread_mutex_lock(&_angle_mutex);
}

void robotSim::simThreadsAngleUnlock(void) {
	pthread_mutex_unlock(&_angle_mutex);
}

void robotSim::simThreadsRecordingInit(void) {
	pthread_mutex_init(&_recording_mutex, NULL);
	pthread_cond_init(&_recording_cond, NULL);
}

void robotSim::simThreadsRecordingLock(void) {
	pthread_mutex_lock(&_recording_mutex);
}

void robotSim::simThreadsRecordingUnlock(void) {
	pthread_mutex_unlock(&_recording_mutex);
}

void robotSim::simThreadsRecordingSignal(void) {
	pthread_cond_signal(&_recording_cond);
}

void robotSim::simThreadsRecordingWait(void) {
	pthread_cond_wait(&_recording_cond, &_recording_mutex);
}

void robotSim::simThreadsSuccessInit(void) {
	pthread_mutex_init(&_success_mutex, NULL);
	pthread_cond_init(&_success_cond, NULL);
}

void robotSim::simThreadsSuccessLock(void) {
	pthread_mutex_lock(&_success_mutex);
}

void robotSim::simThreadsSuccessUnlock(void) {
	pthread_mutex_unlock(&_success_mutex);
}

void robotSim::simThreadsSuccessSignal(void) {
	pthread_cond_signal(&_success_cond);
}

void robotSim::simThreadsSuccessWait(void) {
	pthread_cond_wait(&_success_cond, &_success_mutex);
}

int robotSim::simThreadsRWInit(pthread_rw_t *rwp) {
	rwp->lock = 0;
	pthread_mutex_init(&(rwp->mutex), NULL);
	pthread_cond_init(&(rwp->cond), NULL);
	return 0;
}

int robotSim::simThreadsRWRLock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	while (rwp->lock) {
		pthread_cond_wait(&(rwp->cond), &(rwp->mutex));
	}
	rwp->lock = true;
	pthread_mutex_unlock(&(rwp->mutex));
	return 0;
}

int robotSim::simThreadsRWWLock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	while (rwp->lock) {
		pthread_cond_wait(&(rwp->cond), &(rwp->mutex));
	}
	rwp->lock = true;
	pthread_mutex_unlock(&(rwp->mutex));
	return 0;
}

int robotSim::simThreadsRWRUnlock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	if (!rwp->lock) {
		pthread_mutex_unlock(&(rwp->mutex));
		return -1;
	}
	else {
		rwp->lock = false;
		pthread_cond_signal(&(rwp->cond));
		pthread_mutex_unlock(&(rwp->mutex));
		return 0;
	}
}

int robotSim::simThreadsRWWUnlock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	if (!rwp->lock) {
		pthread_mutex_unlock(&(rwp->mutex));
		return -1;
	}
	else {
		rwp->lock = false;
		pthread_cond_broadcast(&(rwp->cond));
		pthread_mutex_unlock(&(rwp->mutex));
		return 0;
	}
}

int robotSim::simThreadsGoalInit(void) { simThreadsRWInit(&_goal_rwlock); }
int robotSim::simThreadsGoalRLock(void) { simThreadsRWRLock(&_goal_rwlock); }
int robotSim::simThreadsGoalRUnlock(void) { simThreadsRWRUnlock(&_goal_rwlock); }
int robotSim::simThreadsGoalWLock(void) { simThreadsRWWLock(&_goal_rwlock); }
int robotSim::simThreadsGoalWUnlock(void) { simThreadsRWWUnlock(&_goal_rwlock); }

