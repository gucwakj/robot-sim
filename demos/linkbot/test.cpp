#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1/*, robot2*/;
	robot1.connect();
	//robot2.connect();

	robot1.moveDistance(7, 1.75);
	//robot2.moveDistance(-10, 1.75);

	return 0;
}
