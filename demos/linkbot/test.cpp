#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();
	robot1.point(12, 5.5, 1, 1, "red");
	robot1.line(1, 2, 3, 5, 2, 0, 1, "red");
	robot1.move(45, 0, -45);
	char color[20];
	robot1.getColorName(color);
	printf("%s\n", color);
	usleep(100000);

	return 0;
}
