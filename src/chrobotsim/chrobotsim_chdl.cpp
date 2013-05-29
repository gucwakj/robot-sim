#include "../crobotsim/robotsim.h"
#include "../crobotsim/mobotsim.h"
#include "../crobotsim/linkbotsim.h"
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
	void *ap_c;
	ChVaList_t ap_ch;
    class CRobotSim *sim;
	class CMobot *mobot;
	char *name;
    int retval;
	void *memhandle;

    Ch_VaStart(interp, ap, varg);
    sim = Ch_VaArg(interp, ap, class CRobotSim *);
	ap_ch = Ch_VaArg(interp, ap, ChVaList_t);
	ap_c = Ch_VaVarArgsCreate(interp, ap_ch, &memhandle);
	name = Ch_VaUserDefinedName(interp, ap_c);
	printf("name: %s\n", name); exit(1);
    //retval = sim->addRobot(*robot);
	Ch_VaVarArgsDelete(interp, memhandle);
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

EXPORTCH void CLinkbotI_CLinkbotI_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CLinkbotI *c=new CLinkbotI();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotI));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CLinkbotI_dCLinkbotI_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CLinkbotI *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CLinkbotI *);
  if(Ch_CppIsArrayElement(interp))
    c->~CLinkbotI();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int CLinkbotI_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int id;
    double *angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointAngle((linkbotJointID_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointSpeed((linkbotJointID_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double *speed1;
    double *speed2;
    double *speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    speed1 = Ch_VaArg(interp, ap, double *);
    speed2 = Ch_VaArg(interp, ap, double *);
    speed3 = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointSpeeds(*speed1, *speed2, *speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotI_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointSpeedRatio((linkbotJointID_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double *ratio1;
    double *ratio2;
    double *ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    ratio1 = Ch_VaArg(interp, ap, double *);
    ratio2 = Ch_VaArg(interp, ap, double *);
    ratio3 = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotI_motionArch_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionArch(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionInchwormLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    num = Ch_VaArg(interp, ap, int);
    retval = linkbot->motionInchwormLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionInchwormRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    num = Ch_VaArg(interp, ap, int);
    retval = linkbot->motionInchwormRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionStand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->motionStand();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionTumbleLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    num = Ch_VaArg(interp, ap, int);
    retval = linkbot->motionTumbleLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionTumbleRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    num = Ch_VaArg(interp, ap, int);
    retval = linkbot->motionTumbleRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_motionUnstand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->motionUnstand();
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotI_motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->motionWait();
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotI_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotI_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointState_t dir1;
    linkbotJointState_t dir2;
    linkbotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    dir1 = (linkbotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (linkbotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (linkbotJointState_t)Ch_VaArg(interp, ap, int);
    retval = linkbot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotI_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointState_t dir1;
    linkbotJointState_t dir2;
    linkbotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    dir1 = Ch_VaArg(interp, ap, linkbotJointState_t);
    dir2 = Ch_VaArg(interp, ap, linkbotJointState_t);
    dir3 = Ch_VaArg(interp, ap, linkbotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotI_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    linkbotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t );
    dir = Ch_VaArg(interp, ap, linkbotJointState_t);
    retval = linkbot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotI_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    linkbotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    dir = Ch_VaArg(interp, ap, linkbotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointContinuousTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotI_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotI_moveJointToAbs_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToAbs(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotI_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotI_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotI_moveJointToAbsNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToAbsNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotI_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotI_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    retval = linkbot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotI_moveToAbs_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToAbs(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotI_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotI_moveToAbsNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToAbsNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotI_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotI_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_recordAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double* time;
    double* angle;
    int num;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    time = Ch_VaArg(interp, ap, double*);
    angle = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    retval = linkbot->recordAngle(id, time, angle, num, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_recordAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    double* time;
    double* angle1;
    double* angle2;
    double* angle3;
    int num;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    time = Ch_VaArg(interp, ap, double*);
    angle1 = Ch_VaArg(interp, ap, double*);
    angle2 = Ch_VaArg(interp, ap, double*);
    angle3 = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    retval = linkbot->recordAngles(time, angle1, angle2, angle3, num, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_recordWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->recordWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    retval = linkbot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = linkbot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotI_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotI *linkbot;
    linkbotJointID_t id;
    double speed1;
    double speed2;
    double speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotI *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH void CLinkbotL_CLinkbotL_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CLinkbotL *c=new CLinkbotL();
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(CLinkbotL));
  Ch_VaEnd(interp, ap);
}

EXPORTCH void CLinkbotL_dCLinkbotL_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class CLinkbotL *c;
  Ch_VaStart(interp, ap, varg);
  c = Ch_VaArg(interp, ap, class CLinkbotL *);
  if(Ch_CppIsArrayElement(interp))
    c->~CLinkbotL();
  else
    delete c;
  Ch_VaEnd(interp, ap);
  return;
}

EXPORTCH int CLinkbotL_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->connect();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int id;
    double *angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    angle = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointAngle((linkbotJointID_t)id, *angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointSpeed((linkbotJointID_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double *speed1;
    double *speed2;
    double *speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed1 = Ch_VaArg(interp, ap, double *);
    speed2 = Ch_VaArg(interp, ap, double *);
    speed3 = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointSpeeds(*speed1, *speed2, *speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotL_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int id;
    double *speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, int);
    speed = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointSpeedRatio((linkbotJointID_t)id, *speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double *ratio1;
    double *ratio2;
    double *ratio3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    ratio1 = Ch_VaArg(interp, ap, double *);
    ratio2 = Ch_VaArg(interp, ap, double *);
    ratio3 = Ch_VaArg(interp, ap, double *);
    retval = linkbot->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotL_motionArch_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionArch(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionInchwormLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int);
    retval = linkbot->motionInchwormLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionInchwormRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int);
    retval = linkbot->motionInchwormRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionRollBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionRollBackward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionRollForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionRollForward(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionStand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->motionStand();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionTumbleLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int);
    retval = linkbot->motionTumbleLeft(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionTumbleRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int num;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    num = Ch_VaArg(interp, ap, int);
    retval = linkbot->motionTumbleRight(num);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionTurnLeft_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionTurnLeft(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionTurnRight_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->motionTurnRight(angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_motionUnstand_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->motionUnstand();
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotL_motionWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->motionWait();
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotL_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->move(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotL_moveContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointState_t dir1;
    linkbotJointState_t dir2;
    linkbotJointState_t dir3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = (linkbotJointState_t)Ch_VaArg(interp, ap, int);
    dir2 = (linkbotJointState_t)Ch_VaArg(interp, ap, int);
    dir3 = (linkbotJointState_t)Ch_VaArg(interp, ap, int);
    retval = linkbot->moveContinuousNB(dir1, dir2, dir3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotL_moveContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointState_t dir1;
    linkbotJointState_t dir2;
    linkbotJointState_t dir3;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    dir1 = Ch_VaArg(interp, ap, linkbotJointState_t);
    dir2 = Ch_VaArg(interp, ap, linkbotJointState_t);
    dir3 = Ch_VaArg(interp, ap, linkbotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveContinuousTime(dir1, dir2, dir3, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotL_moveJointContinuousNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    linkbotJointState_t dir;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t );
    dir = Ch_VaArg(interp, ap, linkbotJointState_t);
    retval = linkbot->moveJointContinuousNB(id, dir);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotL_moveJointContinuousTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    linkbotJointState_t dir;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    dir = Ch_VaArg(interp, ap, linkbotJointState_t);
    seconds = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointContinuousTime(id, dir, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotL_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotL_moveJointToAbs_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToAbs(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotL_moveJointToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToDirect(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotL_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotL_moveJointToAbsNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToAbsNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotL_moveJointToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double angle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    angle = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveJointToDirectNB(id, angle);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotL_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    retval = linkbot->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotL_moveToAbs_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToAbs(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotL_moveToDirect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToDirect(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}

/*EXPORTCH int CLinkbotL_moveToAbsNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToAbsNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

/*EXPORTCH int CLinkbotL_moveToDirectNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double angle1;
    double angle2;
    double angle3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    angle1 = Ch_VaArg(interp, ap, double);
    angle2 = Ch_VaArg(interp, ap, double);
    angle3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->moveToDirectNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return retval;
}*/

EXPORTCH int CLinkbotL_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->moveToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->moveToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->moveWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double* time;
    double* angle;
    int num;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    time = Ch_VaArg(interp, ap, double*);
    angle = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    retval = linkbot->recordAngle(id, time, angle, num, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    double* time;
    double* angle1;
    double* angle2;
    double* angle3;
    int num;
    double seconds;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    time = Ch_VaArg(interp, ap, double*);
    angle1 = Ch_VaArg(interp, ap, double*);
    angle2 = Ch_VaArg(interp, ap, double*);
    angle3 = Ch_VaArg(interp, ap, double*);
    num = Ch_VaArg(interp, ap, int);
    seconds = Ch_VaArg(interp, ap, double);
    retval = linkbot->recordAngles(time, angle1, angle2, angle3, num, seconds);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_recordWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->recordWait();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->resetToZero();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    retval = linkbot->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double speed;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    id = Ch_VaArg(interp, ap, linkbotJointID_t);
    speed = Ch_VaArg(interp, ap, double);
    retval = linkbot->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CLinkbotL_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CLinkbotL *linkbot;
    linkbotJointID_t id;
    double speed1;
    double speed2;
    double speed3;
    int retval;

    Ch_VaStart(interp, ap, varg);
    linkbot = Ch_VaArg(interp, ap, class CLinkbotL *);
    speed1 = Ch_VaArg(interp, ap, double);
    speed2 = Ch_VaArg(interp, ap, double);
    speed3 = Ch_VaArg(interp, ap, double);
    retval = linkbot->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return retval;
}

