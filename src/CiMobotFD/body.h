#ifndef BODY_H_
#define BODY_H_

#include <ode/ode.h>
#include "config.h"
class Body {
    public:
        Body(void);
        ~Body(void);

        dBodyID bodyID;                         // id of body part
        dGeomID *geomID;                        // ids of geoms which make up each body part
        #ifdef ENABLE_DRAWSTUFF
        float color[3];                         // rgb color for each body part
        int num_geomID;                         // total number of geomID for part
        #endif
};

#endif  /* BODY_H_ */