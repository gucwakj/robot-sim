#include <iostream>
#include "nxt.hpp"
using namespace std;

int main(int argc, char *argv[]) {
	CNXT robot;
	robot.connect("testrc");

	return 0;
}
