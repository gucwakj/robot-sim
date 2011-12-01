#include "body.h"
using namespace std;

Body::Body(dWorldID &world, dSpaceID &space, int num_geomID) {
    this->world = world;
    this->space = space;
    this->geomID = new dGeomID[num_geomID];
    #ifdef ENABLE_DRAWSTUFF
    this->num_geomID = num_geomID;
    #endif
}

Body::~Body(void) {
}

dBodyID Body::getBodyID(void) {
    return this->bodyID;
}

void Body::buildLeftBody(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_lb, int rebuild) {
    // define parameters
    dMass m, m1, m2, m3;
    dMatrix3 R1, R2, R3;

    // set mass of body
    dMassSetZero(&m);
    // create mass 1
    dMassSetBox(&m1, 2700, BODY_END_DEPTH, CENTER_HEIGHT, BODY_WIDTH );
    dMassAdd(&m, &m1);
    // create mass 2
    dMassSetBox(&m2, 2700, BODY_INNER_WIDTH, END_DEPTH, BODY_WIDTH );
    dMassTranslate(&m2, 0.01524, -0.0346, 0 );
    dMassAdd(&m, &m2);
    // create mass 3
    dMassSetBox(&m3, 2700, BODY_INNER_WIDTH, END_DEPTH, BODY_WIDTH );
    dMassTranslate(&m3, 0.01524, 0.0346, 0 );
    dMassAdd(&m, &m3);
    //dMassSetParameters( &m, 500, 1, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // create body
    //if ( !rebuild ) this->bodyID = dBodyCreate(this->world);
    this->bodyID = dBodyCreate(this->world);

    // set body parameters
    dBodySetPosition(this->bodyID, x, y, z);
    dBodySetRotation(this->bodyID, R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
    dRFromAxisAndAngle(R3, 0, 0, 1, -r_lb);
    dMultiply0(R2, R1, R3, 3, 3, 3);

    // set geometry 1 - face
    this->geomID[0] = dCreateBox(this->space, BODY_END_DEPTH, BODY_WIDTH, BODY_HEIGHT);
    dGeomSetBody(this->geomID[0], this->bodyID);
    dGeomSetOffsetPosition(this->geomID[0], -m.c[0], -m.c[1], -m.c[2]);

    // set geometry 2 - side square
    this->geomID[1] = dCreateBox( this->space, BODY_LENGTH, BODY_INNER_WIDTH, BODY_HEIGHT );
    dGeomSetBody( this->geomID[1], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[1], BODY_LENGTH/2 + BODY_END_DEPTH/2 - m.c[0], -BODY_RADIUS + BODY_INNER_WIDTH/2 - m.c[1], -m.c[2] );

    // set geometry 3 - side square
    this->geomID[2] = dCreateBox( this->space, BODY_LENGTH, BODY_INNER_WIDTH, BODY_HEIGHT );
    dGeomSetBody( this->geomID[2], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[2], BODY_LENGTH/2 + BODY_END_DEPTH/2 - m.c[0], BODY_RADIUS - BODY_INNER_WIDTH/2 - m.c[1], -m.c[2] );

    // set geometry 4 - side curve
    this->geomID[3] = dCreateCylinder( this->space, BODY_RADIUS, BODY_INNER_WIDTH );
    dGeomSetBody( this->geomID[3], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[3], BODY_LENGTH + BODY_END_DEPTH/2 - m.c[0], -BODY_RADIUS + BODY_INNER_WIDTH/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geomID[3], R2);

    // set geometry 5 - side curve
    this->geomID[4] = dCreateCylinder( this->space, BODY_RADIUS, BODY_INNER_WIDTH );
    dGeomSetBody( this->geomID[4], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[4], BODY_LENGTH + BODY_END_DEPTH/2 - m.c[0], BODY_RADIUS - BODY_INNER_WIDTH/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geomID[4], R2);

    // set mass center to (0,0,0) of this->bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(this->bodyID, &m);

    #ifdef ENABLE_DRAWSTUFF
    this->color[0] = 1;
    this->color[1] = 0;
    this->color[2] = 0;
    #endif
}

void Body::buildRightBody(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_rb, int rebuild) {
    // define parameters
    dMass m, m1, m2, m3;
    dMatrix3 R1, R2, R3;

    // set mass of body
    dMassSetZero(&m);
    // create mass 1
    dMassSetBox(&m1, 2700, BODY_END_DEPTH, CENTER_HEIGHT, BODY_WIDTH );
    dMassAdd(&m, &m1);
    // create mass 2
    dMassSetBox(&m2, 2700, BODY_INNER_WIDTH, END_DEPTH, BODY_WIDTH );
    dMassTranslate(&m2, -0.01524, -0.0346, 0 );
    dMassAdd(&m, &m2);
    // create mass 3
    dMassSetBox(&m3, 2700, BODY_INNER_WIDTH, END_DEPTH, BODY_WIDTH );
    dMassTranslate(&m3, -0.01524, 0.0346, 0 );
    dMassAdd(&m, &m3);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // create body
    //if ( !rebuild ) this->bodyID = dBodyCreate(this->world);
    this->bodyID = dBodyCreate(this->world);

    // set body parameters
    dBodySetPosition(this->bodyID, x, y, z);
    dBodySetRotation(this->bodyID, R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
    dRFromAxisAndAngle(R3, 0, 0, 1, -r_rb);
    dMultiply0(R2, R1, R3, 3, 3, 3);

    // set geometry 1 - face
    this->geomID[0] = dCreateBox(this->space, BODY_END_DEPTH, BODY_WIDTH, BODY_HEIGHT );
    dGeomSetBody( this->geomID[0], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - side square
    this->geomID[1] = dCreateBox(this->space, BODY_LENGTH, BODY_INNER_WIDTH, BODY_HEIGHT );
    dGeomSetBody( this->geomID[1], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[1], -BODY_LENGTH/2 - BODY_END_DEPTH/2 - m.c[0], -BODY_RADIUS + BODY_INNER_WIDTH/2 - m.c[1], -m.c[2] );

    // set geometry 3 - side square
    this->geomID[2] = dCreateBox(this->space, BODY_LENGTH, BODY_INNER_WIDTH, BODY_HEIGHT );
    dGeomSetBody( this->geomID[2], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[2], -BODY_LENGTH/2 - BODY_END_DEPTH/2 - m.c[0], BODY_RADIUS - BODY_INNER_WIDTH/2 - m.c[1], -m.c[2] );

    // set geometry 4 - side curve
    this->geomID[3] = dCreateCylinder(this->space, BODY_RADIUS, BODY_INNER_WIDTH );
    dGeomSetBody( this->geomID[3], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[3], -BODY_LENGTH - BODY_END_DEPTH/2 - m.c[0], -BODY_RADIUS + BODY_INNER_WIDTH/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geomID[3], R2);

    // set geometry 5 - side curve
    this->geomID[4] = dCreateCylinder(this->space, BODY_RADIUS, BODY_INNER_WIDTH );
    dGeomSetBody( this->geomID[4], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[4], -BODY_LENGTH - BODY_END_DEPTH/2 - m.c[0], BODY_RADIUS - BODY_INNER_WIDTH/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geomID[4], R2);

    // set mass center to (0,0,0) of body
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(this->bodyID, &m);

    #ifdef ENABLE_DRAWSTUFF
    this->color[0] = 1;
    this->color[1] = 1;
    this->color[2] = 1;
    #endif
}

void Body::buildCenter(dReal x, dReal y, dReal z, dMatrix3 R, int rebuild) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetZero(&m);
    dMassSetCapsule(&m, 2700, 1, CENTER_RADIUS, CENTER_LENGTH );
    dMassAdjust(&m, 0.24);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // create body
    //if ( !rebuild ) this->bodyID = dBodyCreate(this->world);
    this->bodyID = dBodyCreate(this->world);

    // set body parameters
    dBodySetPosition(this->bodyID, x, y, z);
    dBodySetRotation(this->bodyID, R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);

    // set geometry 1 - center rectangle
    this->geomID[0] = dCreateBox(this->space, CENTER_LENGTH, CENTER_WIDTH, CENTER_HEIGHT );
    dGeomSetBody( this->geomID[0], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - side curve
    this->geomID[1] = dCreateCylinder(this->space, CENTER_RADIUS, CENTER_WIDTH );
    dGeomSetBody( this->geomID[1], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[1], -CENTER_LENGTH/2 - m.c[0], -m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geomID[1], R1);

    // set geometry 3 - side curve
    this->geomID[2] = dCreateCylinder(this->space, CENTER_RADIUS, CENTER_WIDTH );
    dGeomSetBody( this->geomID[2], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[2], CENTER_LENGTH/2 - m.c[0], -m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geomID[2], R1);

    // set mass center to (0,0,0) of body
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(this->bodyID, &m);

    #ifdef ENABLE_DRAWSTUFF
    this->color[0] = 0;
    this->color[1] = 1;
    this->color[2] = 0;
    #endif
}

void Body::buildEndcap(dReal x, dReal y, dReal z, dMatrix3 R, int rebuild) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetBox(&m, 2700, END_DEPTH, END_WIDTH, END_HEIGHT );
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // create body
    //if ( !rebuild ) this->bodyID = dBodyCreate(this->world);
    this->bodyID = dBodyCreate(this->world);

    // set body parameters
    dBodySetPosition(this->bodyID, x, y, z);
    dBodySetRotation(this->bodyID, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    this->geomID[0] = dCreateBox(this->space, END_DEPTH, END_WIDTH - 2*END_RADIUS, END_HEIGHT );
    dGeomSetBody( this->geomID[0], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - left box
    this->geomID[1] = dCreateBox(this->space, END_DEPTH, END_RADIUS, END_HEIGHT - 2*END_RADIUS );
    dGeomSetBody( this->geomID[1], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[1], -m.c[0], -END_WIDTH/2 + END_RADIUS/2 - m.c[1], -m.c[2] );

    // set geometry 3 - right box
    this->geomID[2] = dCreateBox(this->space, END_DEPTH, END_RADIUS, END_HEIGHT - 2*END_RADIUS );
    dGeomSetBody( this->geomID[2], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[2], -m.c[0], END_WIDTH/2 - END_RADIUS/2 - m.c[1], -m.c[2] );

    // set geometry 4 - fillet upper left
    this->geomID[3] = dCreateCylinder(this->space, END_RADIUS, END_DEPTH );
    dGeomSetBody( this->geomID[3], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[3], -m.c[0], -END_WIDTH/2 + END_RADIUS - m.c[1], END_WIDTH/2 - END_RADIUS - m.c[2] );
    dGeomSetOffsetRotation( this->geomID[3], R1);

    // set geometry 5 - fillet upper right
    this->geomID[4] = dCreateCylinder(this->space, END_RADIUS, END_DEPTH );
    dGeomSetBody( this->geomID[4], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[4], -m.c[0], END_WIDTH/2 - END_RADIUS - m.c[1], END_WIDTH/2 - END_RADIUS - m.c[2] );
    dGeomSetOffsetRotation( this->geomID[4], R1);

    // set geometry 6 - fillet lower right
    this->geomID[5] = dCreateCylinder(this->space, END_RADIUS, END_DEPTH );
    dGeomSetBody( this->geomID[5], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[5], -m.c[0], END_WIDTH/2 - END_RADIUS - m.c[1], -END_WIDTH/2 + END_RADIUS - m.c[2] );
    dGeomSetOffsetRotation( this->geomID[5], R1);

    // set geometry 7 - fillet lower left
    this->geomID[6] = dCreateCylinder(this->space, END_RADIUS, END_DEPTH );
    dGeomSetBody( this->geomID[6], this->bodyID);
    dGeomSetOffsetPosition( this->geomID[6], -m.c[0], -END_WIDTH/2 + END_RADIUS - m.c[1], -END_WIDTH/2 + END_RADIUS - m.c[2] );
    dGeomSetOffsetRotation( this->geomID[6], R1);

    // set mass center to (0,0,0) of this->bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(this->bodyID, &m);

    #ifdef ENABLE_DRAWSTUFF
    this->color[0] = 0;
    this->color[1] = 0;
    this->color[2] = 1;
    #endif
}