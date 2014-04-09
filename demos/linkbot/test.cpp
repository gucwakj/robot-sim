#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
CLinkbotI robot1, robot2;

robot1.connect();
robot2.connect();

robot1.resetToZero();
robot2.resetToZero();

/* move forward */
robot1.moveForwardNB(360);
robot2.moveBackwardNB(360);
robot1.moveWait();
robot2.moveWait();

/* move backward */
robot1.moveBackwardNB(360);
robot2.moveForwardNB(360);
robot1.moveWait();
robot2.moveWait();

	return 0;
}
