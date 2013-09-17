#include "../librobosim/robotsim.h"
#ifdef _WIN32
#include <windows.h>
#endif
#include <ch.h>

EXPORTCH void RoboSim_RoboSim_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RoboSim *c = new RoboSim();
	Ch_VaStart(interp, ap, varg);
	Ch_CppChangeThisPointer(interp, c, sizeof(RoboSim));
	Ch_VaEnd(interp, ap);
}

EXPORTCH void RoboSim_dRoboSim_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class RoboSim *c;
	Ch_VaStart(interp, ap, varg);
	c = Ch_VaArg(interp, ap, class RoboSim *);
	if(Ch_CppIsArrayElement(interp))
		c->~RoboSim();
	else
		delete c;
	Ch_VaEnd(interp, ap);
	return;
}
