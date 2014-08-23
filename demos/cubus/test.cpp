#include <iostream>
#include "cubus.hpp"
using namespace std;

int main(int argc, char *argv[]) {
	Cubus robot1, robot2, robot3, robot4, robot5;
	robot1.connect("testrc");
	robot2.connect();
	robot3.connect();
	robot4.connect();
	robot5.connect();

	g_sim->runSimulation();

	return 0;
}
