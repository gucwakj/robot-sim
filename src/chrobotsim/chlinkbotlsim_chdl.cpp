#include "../crobotsim/robotsim.h"
#include "../crobotsim/mobotsim.h"
#include "../crobotsim/linkbotsim.h"
#include <windows.h>
#include <ch.h>

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