#include <linkbotsim.h>

CLinkbotT robot;

robot.connect();
robot.resetToZero();
robot.moveTo(45, 0, 45);
