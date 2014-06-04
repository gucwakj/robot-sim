#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	int i;
	CLinkbotI robot[32];

	for (i = 0; i < 32; i++) {
		robot[i].connect();
		robot[i].resetToZero();
	}
	robot[0].moveDistance(3, 1.75);

	return 0;
}
