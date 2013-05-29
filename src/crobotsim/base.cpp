#include "base.h"

void* CRobot::simPreCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPreCollisionThread();
}

void* CRobot::simPostCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPostCollisionThread();
}

