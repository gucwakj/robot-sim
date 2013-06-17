#include "../crobotsim/robotsim.h"
#include "../crobotsim/mobotsim.h"
#include "../crobotsim/linkbotsim.h"
#include <windows.h>
#include <ch.h>

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