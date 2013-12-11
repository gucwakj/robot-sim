#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1, robot2, robot3, robot4, robot5;

	robot1.connect();
	robot2.connect();
	robot3.connect();
	robot4.connect();
	robot5.connect();

	return 0;
}
