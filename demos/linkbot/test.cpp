#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();
	robot1.movexy(-1, 1, 1.75, 3.69);

	return 0;
}
