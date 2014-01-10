#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();

	robot1.move(45, 0, -45);
	char color[20];
	robot1.getColor(color);
	printf("%s\n", color);
	usleep(100000);

	return 0;
}
