#include "../CRobotSim/robotsim.h"
#include "../CRobotSim/mobotsim.h"
#include <ch.h>

EXPORTCH void CRobotSim_CRobotSim_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CRobotSim *c=new CRobotSim();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CRobotSim));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CRobotSim_dCRobotSim_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CRobotSim *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CRobotSim *);
  if(Ch_CppIsArrayElement(interp))
    c->~CRobotSim();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int CRobotSim_addRobot_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CRobotSim *sim;
	class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    sim = Ch_VaArg(interp, ap, class CRobotSim *);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = sim->addRobot(*mobot);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CRobotSim_setExitState_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CRobotSim *sim;
	int state;
    int retval;

    Ch_VaStart(interp, ap, varg);
    sim = Ch_VaArg(interp, ap, class CRobotSim *);
    state = Ch_VaArg(interp, ap, int);
    retval = sim->setExitState(state);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH void CMobot_CMobot_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CMobot *c=new CMobot();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CMobot));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CMobot_dCMobot_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CMobot *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CMobot *);
  if(Ch_CppIsArrayElement(interp))
    c->~CMobot();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int CMobot_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double *angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointAngle((mobotJointID_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeed((mobotJointID_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double *speed1;
    double *speed2;
    double *speed3;
    double *speed4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    speed1 = Ch_VaArg(interp, ap, double *);
    speed2 = Ch_VaArg(interp, ap, double *);
    speed3 = Ch_VaArg(interp, ap, double *);
    speed4 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeeds(*speed1, *speed2, *speed3, *speed4);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CMobot_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeedRatio((mobotJointID_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double *ratio1;
    double *ratio2;
    double *ratio3;
    double *ratio4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    ratio1 = Ch_VaArg(interp, ap, double *);
    ratio2 = Ch_VaArg(interp, ap, double *);
    ratio3 = Ch_VaArg(interp, ap, double *);
    ratio4 = Ch_VaArg(interp, ap, double *);
    retval = mobot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3, *ratio4);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CMobot_motionArch_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionArch(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionInchwormLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionInchwormRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionInchwormRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionStand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionStand();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionTumbleLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionTumbleRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    num = Ch_VaArg(interp, ap, int);
    retval = mobot->motionTumbleRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_motionUnstand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionUnstand();
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CMobot_motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->motionWait();
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CMobot_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->move(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CMobot_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    dir4 = (mobotJointState_t)Ch_VaArg(interp, ap, int);
    retval = mobot->moveContinuousNB(dir1, dir2, dir3, dir4);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CMobot_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointState_t dir1;
    mobotJointState_t dir2;
    mobotJointState_t dir3;
    mobotJointState_t dir4;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    dir1 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir2 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir3 = Ch_VaArg(interp, ap, mobotJointState_t);
    dir4 = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveContinuousTime(dir1, dir2, dir3, dir4, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CMobot_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    mobotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t );
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    retval = mobot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CMobot_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    mobotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    dir = Ch_VaArg(interp, ap, mobotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointContinuousTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CMobot_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CMobot_moveJointToAbs_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToAbs(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CMobot_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CMobot_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CMobot_moveJointToAbsNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToAbsNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CMobot_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = mobot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CMobot_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    retval = mobot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveTo(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CMobot_moveToAbs_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToAbs(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CMobot_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirect(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CMobot_moveToAbsNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToAbsNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CMobot_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    angle4 = Ch_VaArg(interp, ap, double);
    retval = mobot->moveToDirectNB(angle1, angle2, angle3, angle4);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CMobot_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_recordAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double* time;
    double* angle;
    int num;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    time = Ch_VaArg(interp, ap, double*);
    angle = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->recordAngle(id, time, angle, num, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_recordAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    double* time;
    double* angle1;
    double* angle2;
    double* angle3;
    double* angle4;
    int num;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    time = Ch_VaArg(interp, ap, double*);
    angle1 = Ch_VaArg(interp, ap, double*);
    angle2 = Ch_VaArg(interp, ap, double*);
    angle3 = Ch_VaArg(interp, ap, double*);
    angle4 = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    retval = mobot->recordAngles(time, angle1, angle2, angle3, angle4, num, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_recordWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->recordWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    retval = mobot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    id = Ch_VaArg(interp, ap, mobotJointID_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CMobot_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CMobot *mobot;
    mobotJointID_t id;
    double speed1;
    double speed2;
    double speed3;
    double speed4;
    int retval;

    Ch_VaStart(interp, ap, varg);
    mobot = Ch_VaArg(interp, ap, class CMobot *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    speed4 = Ch_VaArg(interp, ap, double);
    retval = mobot->setJointSpeeds(speed1, speed2, speed3, speed4);
    Ch_VaEnd(interp, ap);
    return retval;
}
