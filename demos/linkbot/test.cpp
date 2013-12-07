#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();
	robot1.moveDistance(1, 1.75);
	robot1.delay(10);

	return 0;
}
