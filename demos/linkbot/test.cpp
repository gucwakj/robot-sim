#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
CLinkbotI robot1, robot2;

robot1.connect();
robot2.connect();

robot1.resetToZeroNB();
robot2.resetToZeroNB();
robot1.moveWait();
robot2.moveWait();

	return 0;
}
