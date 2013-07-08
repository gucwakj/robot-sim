#include "../crobotsim/robotsim.h"
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