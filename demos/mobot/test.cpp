#include <iostream>
#include "mobot.hpp"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot robot;
	robot.connect("testrc");

	return 0;
}
