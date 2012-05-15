#ifndef BODY_H_
#define BODY_H_

#include "config.h"
#include <ode/ode.h>
#ifdef ENABLE_DRAWSTUFF
    #include <drawstuff/drawstuff.h>
    #define DRAWSTUFF_TEXTURE_PATH "../opende/drawstuff/textures"
    #ifdef dDOUBLE
        #define dsDrawSphere dsDrawSphereD
        #define dsDrawBox dsDrawBoxD
        #define dsDrawCylinder dsDrawCylinderD
        #define dsDrawCapsule dsDrawCapsuleD
    #endif
#endif

#ifndef CIMOBOTIK_H_
#define CENTER_LENGTH       0.07303
#define CENTER_WIDTH        0.02540
#define CENTER_HEIGHT       0.06909
#define CENTER_RADIUS       0.03554
#define BODY_LENGTH         0.03785
#define BODY_WIDTH          0.07239
#define BODY_HEIGHT         0.07239
#define BODY_RADIUS         0.03620
#define BODY_INNER_WIDTH    0.02287
#define BODY_END_DEPTH      0.01994
#define BODY_MOUNT_CENTER   0.03792
#define END_WIDTH           0.07239
#define END_HEIGHT          0.07239
#define END_DEPTH           0.00476
#define END_RADIUS          0.01778
#endif

class Body {
    public:
        Body(dWorldID &world, dSpaceID &space, int num_geomID);
        ~Body(void);

        dBodyID getBodyID(void);

        void buildLeftBody(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_lb);
        void buildRightBody(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_rb);
        void buildCenter(dReal x, dReal y, dReal z, dMatrix3 R);
        void buildEndcap(dReal x, dReal y, dReal z, dMatrix3 R);

        #ifdef ENABLE_DRAWSTUFF
        void drawBody(void);
        #endif
    private:
        dWorldID world;                         // world for all robots
        dSpaceID space;                         // space for this robot
        dBodyID bodyID;                         // id of body part
        dGeomID *geomID;                        // ids of geoms which make up each body part

        #ifdef ENABLE_DRAWSTUFF
        float color[3];                         // rgb color for each body part
        int num_geomID;                         // total number of geomID for part
        #endif
};

#endif  /* BODY_H_ */