#include "../crobotsim/robotsim.h"
#include "../crobotsim/mobotsim.h"
#include "../crobotsim/linkbotsim.h"
#include <windows.h>
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

/*EXPORTCH int CRobotSim_addRobot_chdl(void *varg) {
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
}*/

EXPORTCH int CRobotSim_addMobot_chdl(void *varg) {
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

EXPORTCH int CRobotSim_addLinkbotI_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CRobotSim *sim;
	class CLinkbotI *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    sim = Ch_VaArg(interp, ap, class CRobotSim *);
	robot = Ch_VaArg(interp, ap, class CLinkbotI *);
	retval = sim->addRobot(*robot);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int CRobotSim_addLinkbotL_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class CRobotSim *sim;
	class CLinkbotL *robot;
    int retval;

    Ch_VaStart(interp, ap, varg);
    sim = Ch_VaArg(interp, ap, class CRobotSim *);
	robot = Ch_VaArg(interp, ap, class CLinkbotL *);
	retval = sim->addRobot(*robot);
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