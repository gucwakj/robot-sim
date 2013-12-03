#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();
	robot1.moveDistance(1, 1.75);
	double distance;
	robot1.getDistance(distance, 1.75);
	printf("distance = %lf\n", distance);

	return 0;
}
