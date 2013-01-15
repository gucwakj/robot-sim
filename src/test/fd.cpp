#include <iostream>
#include "irse.h"
using namespace std;

int main(int argc, char *argv[]) {
	IRSE sim;
	mobotSim robot1/*, robot2*/;
	//double time[20], angle1[20],angle2[20],angle3[20],angle4[20];

	robot1.connect(sim);

	//robot1.recordAngles(time, angle1, angle2, angle3, angle4, 20, 0.1);
	//robot1.recordAngle(1, time, angle1, 20, 0.1);

	robot1.move(0, 45, -45, 0);

	//robot1.recordWait();
	//printf("recorded data\n");
	//for ( int i = 0; i < 20; i++) {
		//printf("%lf: %lf %lf %lf %lf\n", time[i], angle1[i], angle2[i], angle3[i], angle4[i]);
	//	printf("%lf: %lf\n", time[i], angle1[i]);
	//}
	//robot1.moveNB(0, 0, 0, 45);
	//robot1.moveWait();
	//robot1.moveJoint(IMOBOT_JOINT2, 45);
	//robot1.moveJointNB(IMOBOT_JOINT2, 45);
	//robot1.moveJointWait(IMOBOT_JOINT2);
	//robot1.moveTo(0, 45, 45, 45);
	//robot1.moveTo(0, 45, 45, 45);
	//robot1.moveJointTo(IMOBOT_JOINT2, 45);
	//robot1.moveJointToNB(IMOBOT_JOINT2, 45);
	//robot1.moveJointWait(IMOBOT_JOINT2);
	//robot2.move(0, 45, 65, 45);
	//robot2.moveNB(0, 45, 45, 0);
	//robot2.moveWait();

	//double speed[4];
	//robot1.setJointSpeeds(87, 23, 35, 22);
	//robot1.getJointSpeeds(speed[0], speed[1], speed[2], speed[3]);
	//printf("joint speed = %lf %lf %lf %lf\n", speed[0], speed[1], speed[2], speed[3]);

	// motions
	//robot1.motionArch(10);
	return 0;
}
