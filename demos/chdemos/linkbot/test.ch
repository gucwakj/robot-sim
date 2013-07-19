#include <linkbotsim.h>

CLinkbotI robot;

robot.connect();
robot.resetToZero();
robot.moveTo(0, 0, 45);
