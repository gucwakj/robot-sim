int CNXTGroup::move(double angle1, double angle2) {
	moveNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::moveNB(double angle1, double angle2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveNB(angle1, angle2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveTo(double angle1, double angle2) {
	moveToNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::moveToNB(double angle1, double angle2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToNB(angle1, angle2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveToByTrackPos(double angle1, double angle2) {
	moveToNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::moveToByTrackPosNB(double angle1, double angle2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToByTrackPosNB(angle1, angle2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setJointSpeeds(double speed1, double speed2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeeds(speed1, speed2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setJointSpeedRatios(double ratio1, double ratio2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatios(ratio1, ratio2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

