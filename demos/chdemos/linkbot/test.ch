#include <linkbotsim.h>

CLinkbotI robot;

robot.connect();
//robot.resetToZero();
robot.moveTo(45, 0, 45);
