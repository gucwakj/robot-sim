#include <iostream>
#include "linkbot.hpp"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	robot.connect("testrc");

	return 0;
}
