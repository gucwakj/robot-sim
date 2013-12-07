#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();
	double time1, time2;
	robot1.systemTime(time1);
	robot1.moveDistance(1, 1.75);
	robot1.systemTime(time2);
	printf("time: %lf %lf\n", time1, time2);

	return 0;
}
