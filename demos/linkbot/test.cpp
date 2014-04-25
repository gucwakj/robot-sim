#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;

	robot.connect();
	robot.recordDistanceOffset(6);

	return 0;
}
