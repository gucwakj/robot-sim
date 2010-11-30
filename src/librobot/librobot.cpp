#include <ode/ode.h>
#include <math.h>
#include "librobot.h"

/*
 *	create struct to store all robot data
 */
robots_t *createWorldRobots(int num) {
	// array of pointers to structs for each individual robot
	robots_t *robots = (robots_t *) malloc(sizeof(robots_t));
	robots->robots = (robot_t **) malloc(sizeof(robot_t) * num);

	// total number of robots
	robots->num_robots = num;

	// completion flags for each robot
	robots->flags = (int *) malloc(sizeof(int) * num);
	for (int i = 0; i < num; i++) { robots->flags[i] = 0; }

	// flag if each robot is enabled/disabled in simulation
	robots->disable = (int *) malloc(sizeof(int) * num);
	for (int i = 0; i < num; i++) { robots->disable[i] = 0; }

	// current step of motion (set of angles)
	robots->step = 1;

	// success of motion
	robots->success = 0;
	robots->time = 0.0;

	return robots;
}

/*
 *	minimum value of array
 */
int minVal(int *a, int length) {
	int min = a[0];
	for (int i = 1; i < length; i++) {
		if (a[i] < min) {
			min = a[i];
		}
	}
	return min;
}

dReal angMod(dReal pasAng, dReal curAng, dReal angRat) {
	dReal newAng = 0;
	int stp = (int)( fabs(pasAng) / M_PI );
	dReal pasAngMod = fabs(pasAng) - stp*M_PI;

	if ( (int)angRat == 0 ) {
		newAng = pasAng;
	}
	// positive angular velocity, positive angle
	else if ( angRat > 0 && pasAng >= 0 ) {
		// cross 180
		if ( curAng < 0 && !(stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod + 2*M_PI);	}
		// negative
		else if ( curAng < 0 && (stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod + M_PI);	}
		// cross 0
		else if ( curAng > 0 && (stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod + M_PI);	}
		// positive
		else if ( curAng > 0 && !(stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod);	}
	}
	// positive angular velocity, negative angle
	else if ( angRat > 0 && pasAng < 0 ) {
		// cross 180
		if ( curAng < 0 && (stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod + M_PI);	}
		// negative
		else if ( curAng < 0 && !(stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod);	}
		// cross 0
		else if ( curAng > 0 && !(stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod);	}
		// positive
		else if ( curAng > 0 && (stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod - M_PI);	}
	}
	// negative angular velocity, positive angle
	else if ( angRat < 0 && pasAng >= 0 ) {
		// cross 180
		if ( curAng > 0 && (stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod - M_PI);	}
		// negative
		else if ( curAng < 0 && (stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod + M_PI);	}
		// cross 0
		else if ( curAng < 0 && !(stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod);	}
		// positive
		else if ( curAng > 0 && !(stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod);	}
	}
	// negative angular velocity, negative angle
	else if ( angRat < 0 && pasAng < 0 ) {
		// cross 180
		if ( curAng > 0 && !(stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod - 2*M_PI);	}
		// negative
		else if ( curAng < 0 && !(stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod);	}
		// cross 0
		else if ( curAng < 0 && (stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod - M_PI);	}
		// positive
		else if ( curAng > 0 && (stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod - M_PI);	}
	}

	return newAng;
}

/*
 *	build copy of left body
 */
void buildLeftBody(dWorldID *world, dSpaceID *space, robot_part_t *part, dReal x, dReal y, dReal z, dMatrix3 R) {
	
	// create array for all geoms of body
	part->num_geomID = 5;
	part->geomID = (dGeomID *)malloc(sizeof(dGeomID) * part->num_geomID);

	// define parameters
	dMass m, m1, m2, m3;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

	dMassSetZero(&m);
	// create mass 1
	dMassSetBox(&m1, DENALUM, I2M(0.2), I2M(2.60), I2M(2.85) );
	dMassAdd(&m, &m1);
	// create mass 2
	dMassSetBox(&m2, DENALUM, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m2, I2M(0.5 + 0.1), -I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m2);
	// create mass 3
	dMassSetBox(&m3, DENALUM, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m3, I2M(0.5 + 0.1), I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m3);
	//dMassSetParameters( &m, 500, I2M(1), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2] - m.c[0];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2] - m.c[1];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2] - m.c[2];

	// create body
	body = dBodyCreate(*world);
	dBodySetPosition(body, x + m.c[0], y + m.c[1], z + m.c[2]);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1,1,0,0,M_PI/2);

	// set geometry 1 - face
	geom = dCreateBox( *space, I2M(0.2), I2M(2.85), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part->geomID[0] = geom;

	// set geometry 2 - side square
	geom = dCreateBox( *space, I2M(1.55), I2M(0.875), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.55/2 + 0.1) - m.c[0], -I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	part->geomID[1] = geom;

	// set geometry 3 - side square
	geom = dCreateBox( *space, I2M(1.55), I2M(0.875), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.55/2 + 0.1) - m.c[0], I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	part->geomID[2] = geom;

	// set geometry 4 - side curve
	geom = dCreateCylinder( *space, I2M(2.85/2), I2M(0.875) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.65) - m.c[0], -I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[3] = geom;

	// set geometry 5 - side curve
	geom = dCreateCylinder( *space, I2M(2.85/2), I2M(0.875) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.65) - m.c[0], I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[4] = geom;

	// put into robot struct
	part->bodyID = body;
	part->color[0] = 1;
	part->color[1] = 0;
	part->color[2] = 0;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);
}

/*
 *	build copy of right body
 */
void buildRightBody(dWorldID *world, dSpaceID *space, robot_part_t *part, dReal x, dReal y, dReal z, dMatrix3 R) {
	// create array for all geoms of body
	part->num_geomID = 5;
	part->geomID = (dGeomID *)malloc(sizeof(dGeomID) * part->num_geomID);

	// define parameters
	dMass m, m1, m2, m3;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

	dMassSetZero(&m);
	// create mass 1
	dMassSetBox(&m1, DENALUM, I2M(0.2), I2M(2.60), I2M(2.85) );
	dMassAdd(&m, &m1);
	// create mass 2
	dMassSetBox(&m2, DENALUM, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m2, -I2M(0.5 + 0.1), -I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m2);
	// create mass 3
	dMassSetBox(&m3, DENALUM, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m3, -I2M(0.5 + 0.1), I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m3);
	//dMassSetParameters( &m, 500, I2M(0.45), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2] - m.c[0];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2] - m.c[1];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2] - m.c[2];

	// create body
	body = dBodyCreate(*world);
	dBodySetPosition(body, x + m.c[0], y + m.c[1], z + m.c[2]);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1,1,0,0,M_PI/2);

	// set geometry 1 - face
	geom = dCreateBox( *space, I2M(0.2), I2M(2.85), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part->geomID[0] = geom;

	// set geometry 2 - side square
	geom = dCreateBox( *space, I2M(1.55), I2M(0.875), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.55/2 + 0.1) - m.c[0], -I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	part->geomID[1] = geom;

	// set geometry 3 - side square
	geom = dCreateBox( *space, I2M(1.55), I2M(0.875), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.55/2 + 0.1) - m.c[0], I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	part->geomID[2] = geom;

	// set geometry 4 - side curve
	geom = dCreateCylinder( *space, I2M(2.85/2), I2M(0.875) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.65) - m.c[0], -I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[3] = geom;

	// set geometry 5 - side curve
	geom = dCreateCylinder( *space, I2M(2.85/2), I2M(0.875) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.65) - m.c[0], I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[4] = geom;

	// put into robot struct
	part->bodyID = body;
	part->color[0] = 1;
	part->color[1] = 1;
	part->color[2] = 1;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);
}

/*
 *	build copy of center
 */
void buildCenter(dWorldID *world, dSpaceID *space, robot_part_t *part, dReal x, dReal y, dReal z, dMatrix3 R) {

	// create array for all geoms of body
	part->num_geomID = 3;
	part->geomID = (dGeomID *)malloc(sizeof(dGeomID) * part->num_geomID);

	// define parameters
	dMass m;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

	// create mass
	dMassSetZero(&m);
	dMassSetCapsule(&m, DENALUM, 1, I2M(1.3), I2M(5.465 - 1.3 - 1.3) );
	dMassAdjust(&m, 0.24);
	//dMassSetParameters( &m, 500, I2M(0.45), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2] - m.c[0];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2] - m.c[1];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2] - m.c[2];

	// create body
	body = dBodyCreate(*world);
	dBodySetPosition(body, x + m.c[0], y + m.c[1], z + m.c[2]);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1,1,0,0,M_PI/2);

	// set geometry 1 - center rectangle
	geom = dCreateBox( *space, I2M(2.865), I2M(0.125*2 + 0.8), I2M(2.6) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part->geomID[0] = geom;

	// set geometry 2 - side curve
	geom = dCreateCylinder( *space, I2M(1.3), I2M(0.125*2 + 0.8) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.4325) - m.c[0], -m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[1] = geom;

	// set geometry 3 - side curve
	geom = dCreateCylinder( *space, I2M(1.3), I2M(0.125*2 + 0.8) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.4325) - m.c[0], -m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[2] = geom;

	// put into robot struct
	part->bodyID = body;
	part->color[0] = 0;
	part->color[1] = 1;
	part->color[2] = 0;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);
}

/*
 *	build copy of endcap
 */
void buildEndcap(dWorldID *world, dSpaceID *space, robot_part_t *part, dReal x, dReal y, dReal z, dMatrix3 R) {
	
	// create array for all geoms of body
	part->num_geomID = 7;
	part->geomID = (dGeomID *)malloc(sizeof(dGeomID) * part->num_geomID);

	// define parameters
	dMass m;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

	// create mass
	dMassSetBox(&m, DENALUM, I2M(0.125), I2M(2.85), I2M(2.85) );
	//dMassSetParameters( &m, 500, I2M(0.45), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2] - m.c[0];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2] - m.c[1];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2] - m.c[2];

	// create body
	body = dBodyCreate(*world);
	dBodySetPosition(body, x + m.c[0], y + m.c[1], z + m.c[2]);
	dBodySetRotation(body, R);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1,0,1,0,M_PI/2);

	// set geometry 1 - center box
	geom = dCreateBox( *space, I2M(0.125), I2M(1.15), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part->geomID[0] = geom;

	// set geometry 2 - left box
	geom = dCreateBox( *space, I2M(0.125), I2M(0.85), I2M(1.15) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(0.575 + 0.85/2) - m.c[1], -m.c[2] );
	part->geomID[1] = geom;

	// set geometry 3 - right box
	geom = dCreateBox( *space, I2M(0.125), I2M(0.85), I2M(1.15) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(0.575 + 0.85/2) - m.c[1], -m.c[2] );
	part->geomID[2] = geom;

	// set geometry 4 - fillet upper left
	geom = dCreateCylinder( *space, I2M(0.85), I2M(0.125) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(0.575) - m.c[1], I2M(0.575) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[3] = geom;

	// set geometry 5 - fillet upper right
	geom = dCreateCylinder( *space, I2M(0.85), I2M(0.125) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(0.575) - m.c[1], I2M(0.575) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[4] = geom;

	// set geometry 6 - fillet lower right
	geom = dCreateCylinder( *space, I2M(0.85), I2M(0.125) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(0.575) - m.c[1], -I2M(0.575) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[5] = geom;

	// set geometry 7 - fillet lower left
	geom = dCreateCylinder( *space, I2M(0.85), I2M(0.125) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(0.575) - m.c[1], -I2M(0.575) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[6] = geom;

	// put into robot struct
	part->bodyID = body;
	part->color[0] = 0;
	part->color[1] = 0;
	part->color[2] = 1;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);
}
/*
 *	build copy of robot in default orientation
 *		- long axis along the x-axis
 *		- no rotation possible
 */
robot_t *buildRobot(dWorldID *world, dSpaceID *space, dReal x, dReal y, dReal z) {
	dMatrix3 R;
	dRFromAxisAndAngle(R,0,0,1,0);
	robot_t *robot = buildRobotRotated(world, space, x, y, z, R);
	return robot;
}

/*
 *	build copy of robot
 *		- ability to place at any rotation around z-axis
 */
robot_t *buildRobotRotated(dWorldID *world, dSpaceID *space, dReal x, dReal y, dReal z, dMatrix3 R) {
	// create structure for data
	robot_t *robot;
	robot = (robot_t *)malloc(sizeof(robot_t));

	// pointers to structs for each part of robot
	robot->parts = (robot_part_t *)malloc(sizeof(robot_part_t) * NUM_PARTS);

	// flag for each part of body reaching final position
	for (int i = 0; i < NUM_BODIES; i++) robot->cmpStp[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->futEncCnt[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->curEncCnt[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->delEncCnt[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->futAng[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->curAng[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->pasAng[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->delAng[i] = 0;
	for (int i = 0; i < 12; i++) robot->rotation[i] = R[i];

	// minimum velocity of each body
	robot->jntVelMin[LE] = OMEGA_LE_MIN;
	robot->jntVelMin[LB] = OMEGA_LB_MIN;
	robot->jntVelMin[RB] = OMEGA_RB_MIN;
	robot->jntVelMin[RE] = OMEGA_RE_MIN;
	// maximum velocity of each body
	robot->jntVelMax[LE] = OMEGA_LE_MAX;
	robot->jntVelMax[LB] = OMEGA_LB_MAX;
	robot->jntVelMax[RB] = OMEGA_RB_MAX;
	robot->jntVelMax[RE] = OMEGA_RE_MAX;
	// desired velocity of each body
	robot->jntVel[LE] = robot->jntVelMax[LE];
	robot->jntVel[LB] = robot->jntVelMax[LB];
	robot->jntVel[RB] = robot->jntVelMax[RB];
	robot->jntVel[RE] = robot->jntVelMax[RE];

	// angle between each interrupt in motor
	robot->radPerEnc[LE] = RAD_PER_INTERRUPT_LE;
	robot->radPerEnc[LB] = RAD_PER_INTERRUPT_LB;
	robot->radPerEnc[RB] = RAD_PER_INTERRUPT_RB;
	robot->radPerEnc[RE] = RAD_PER_INTERRUPT_RE;

	// offset values for each body part[0-2] and joint[3-5] from center
	dReal le[6] = {-2.865/2-1.55-0.2-0.125/2, 0, 0, -2.865/2 - 1.55 - 0.2, 0, 0};
	dReal lb[6] = {-2.865/2-1.55-0.2/2, 0, 0, -2.865/2, 1.3, 0};
	dReal ce[3] = {0, 0, 0};
	dReal rb[6] = {2.865/2+1.55+0.2/2, 0, 0, 2.865/2, 1.3, 0};
	dReal re[6] = {2.865/2+1.55+0.2+0.125/2, 0, 0, 2.865/2 + 1.55 + 0.2, 0, 0};

	// build pieces of robot
	buildEndcap(world, space, &(robot->parts[ENDCAP_L]),	I2M(R[0]*le[0] + R[1]*le[1] + R[2]*le[2] + x),
															I2M(R[4]*le[0] + R[5]*le[1] + R[6]*le[2] + y),
															I2M(R[8]*le[0] + R[9]*le[1] + R[10]*le[2] + z + 1.425), R);
	buildLeftBody(world, space, &(robot->parts[BODY_L]),	I2M(R[0]*lb[0] + R[1]*lb[1] + R[2]*lb[2] + x),
															I2M(R[4]*lb[0] + R[5]*lb[1] + R[6]*lb[2] + y),
															I2M(R[8]*lb[0] + R[9]*lb[1] + R[10]*lb[2] + z + 1.425), R);
	buildCenter(world, space, &(robot->parts[CENTER]),		I2M(R[0]*ce[0] + R[1]*ce[1] + R[2]*ce[2] + x),
															I2M(R[4]*ce[0] + R[5]*ce[1] + R[6]*ce[2] + y),
															I2M(R[8]*ce[0] + R[9]*ce[1] + R[10]*ce[2] + z + 1.425), R);
	buildRightBody(world, space, &(robot->parts[BODY_R]),	I2M(R[0]*rb[0] + R[1]*rb[1] + R[2]*rb[2] + x),
															I2M(R[4]*rb[0] + R[5]*rb[1] + R[6]*rb[2] + y),
															I2M(R[8]*rb[0] + R[9]*rb[1] + R[10]*rb[2] + z + 1.425), R);
	buildEndcap(world, space, &(robot->parts[ENDCAP_R]),	I2M(R[0]*re[0] + R[1]*re[1] + R[2]*re[2] + x),
															I2M(R[4]*re[0] + R[5]*re[1] + R[6]*re[2] + y),
															I2M(R[8]*re[0] + R[9]*re[1] + R[10]*re[2] + z + 1.425), R);
	robot->position[0] = I2M(R[0]*ce[0] + R[1]*ce[1] + R[2]*ce[2] + x);
	robot->position[1] = I2M(R[4]*ce[0] + R[5]*ce[1] + R[6]*ce[2] + y);
	robot->position[2] = I2M(R[8]*ce[0] + R[9]*ce[1] + R[10]*ce[2] + z);

	// create joint
	dJointID joint;

	// joint for left endcap to body
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[BODY_L].bodyID, robot->parts[ENDCAP_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x),
								I2M(R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y),
								I2M(R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*1 + R[1]*0 + R[2]*0,
								R[4]*1 + R[5]*0 + R[6]*0,
								R[8]*1 + R[9]*0 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[0] = joint;

	// joint for left body 1 to center
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[CENTER].bodyID, robot->parts[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*lb[3] + R[1]*lb[4] + R[2]*lb[5] + x),
								I2M(R[4]*lb[3] + R[5]*lb[4] + R[6]*lb[5] + y),
								I2M(R[8]*lb[3] + R[9]*lb[4] + R[10]*lb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*-1 + R[2]*0,
								R[4]*0 + R[5]*-1 + R[6]*0,
								R[8]*0 + R[9]*-1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[1] = joint;

	// joint for left body 2 to center
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[CENTER].bodyID, robot->parts[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*lb[3] - R[1]*lb[4] + R[2]*lb[5] + x),
								I2M(R[4]*lb[3] - R[5]*lb[4] + R[6]*lb[5] + y),
								I2M(R[8]*lb[3] - R[9]*lb[4] + R[10]*lb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*1 + R[2]*0,
								R[4]*0 + R[5]*1 + R[6]*0,
								R[8]*0 + R[9]*1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[4] = joint;

	// joint for center to right body 1
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[CENTER].bodyID, robot->parts[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*rb[3] + R[1]*rb[4] + R[2]*rb[5] + x),
								I2M(R[4]*rb[3] + R[5]*rb[4] + R[6]*rb[5] + y),
								I2M(R[8]*rb[3] + R[9]*rb[4] + R[10]*rb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*1 + R[2]*0,
								R[4]*0 + R[5]*1 + R[6]*0,
								R[8]*0 + R[9]*1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[2] = joint;

	// joint for center to right body 2
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[CENTER].bodyID, robot->parts[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*rb[3] - R[1]*rb[4] + R[2]*rb[5] + x),
								I2M(R[4]*rb[3] - R[5]*rb[4] + R[6]*rb[5] + y),
								I2M(R[8]*rb[3] - R[9]*rb[4] + R[10]*rb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*-1 + R[2]*0,
								R[4]*0 + R[5]*-1 + R[6]*0,
								R[8]*0 + R[9]*-1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[5] = joint;

	// joint for right body to endcap
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[BODY_R].bodyID, robot->parts[ENDCAP_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x),
								I2M(R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y),
								I2M(R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*-1 + R[1]*0 + R[2]*0,
								R[4]*-1 + R[5]*0 + R[6]*0,
								R[8]*-1 + R[9]*0 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[3] = joint;

	// create motor
	dJointID motor;

	// motor for left endcap to body joint
	motor = dJointCreateAMotor(*world, 0);
	dJointAttach(motor, robot->parts[BODY_L].bodyID, robot->parts[ENDCAP_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[0], R[4], R[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, FMAX_LE);
	robot->motors[0] = motor;

	// motor for left body to center joint
	motor = dJointCreateAMotor(*world, 0);
	dJointAttach(motor, robot->parts[CENTER].bodyID, robot->parts[BODY_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, FMAX_LB);
	robot->motors[1] = motor;

	// motor for center to right body 1 joint
	motor = dJointCreateAMotor(*world, 0);
	dJointAttach(motor, robot->parts[CENTER].bodyID, robot->parts[BODY_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[1], R[5], R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, FMAX_RB);
	robot->motors[2] = motor;

	// motor for right body to endcap joint
	motor = dJointCreateAMotor(*world, 0);
	dJointAttach(motor, robot->parts[BODY_R].bodyID, robot->parts[ENDCAP_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[0], -R[4], -R[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, FMAX_RE);
	robot->motors[3] = motor;

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) {
		dBodySetDamping(robot->parts[i].bodyID, 0.1, 0.1);
	}

	return robot;
}

robot_t *buildRobotAttached(dWorldID *world, dSpaceID *space, robots_t *robots, int robot_num, int face1, int face2) {
	dMatrix3 R, R1;
	dVector3 m;
	dReal x, y, z;
	int face1_part, face2_part;

	if ( face1 == 1 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = 2.865/2+1.55+0.2-1.28;
		m[1] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 1 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = 2.865/2+1.55+0.2-1.28;
		m[1] = -2.865/2-1.55-0.2-0.125-2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 1 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2-1.28);
		m[1] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 1 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2-1.28);
		m[1] = -2.865/2-1.55-0.2-0.125-2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 1 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*(2.865/2+1.55+0.2+0.125);
		m[1] = 0;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 2 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[1] = -1*(2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 2 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 2 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 2 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*1.9025;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 2 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = (2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 3 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = (2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = (2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 3 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 3 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*1.9025;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 3 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 3 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = -1*(2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 4 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[1] = 2.865/2+1.55+0.2-1.28;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 4 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 4 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*1.9025;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 4 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 4 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = -1*(2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 5 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = (2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = -1*(2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 5 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*1.9025;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 5 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 5 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 5 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = (2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 6 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*(2.865/2+1.55+0.2+0.125);
		m[1] = 0;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 6 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = 2.865/2+1.55+0.2-1.28;
		m[1] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 6 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = 2.865/2+1.55+0.2-1.28;
		m[1] = -2.865/2-1.55-0.2-0.125-2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 6 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2-1.28);
		m[1] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 6 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2-1.28);
		m[1] = -2.865/2-1.55-0.2-0.125-2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_R;
	}

	dMultiply0(R1, robots->robots[robot_num]->rotation, R, 3, 3, 3);

	x = M2I(robots->robots[robot_num]->position[0]) + R1[0]*m[0] + R1[1]*m[1] + R1[2]*m[2];
	y = M2I(robots->robots[robot_num]->position[1]) + R1[4]*m[0] + R1[5]*m[1] + R1[6]*m[2];
	z = R1[8]*m[0] + R1[9]*m[1] + R1[10]*m[2];

	robot_t *robot = buildRobotRotated(world, space, x, y, z, R1);

	dJointID joint = dJointCreateFixed(*world, 0);
	dJointAttach(joint, robots->robots[robot_num]->parts[face1_part].bodyID, robot->parts[face2_part].bodyID);
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.85);

	return robot;
}

robot_t *buildRobotPositioned(dWorldID *world, dSpaceID *space, dReal x, dReal y, dReal z, dMatrix3 R, dReal le, dReal lb, dReal rb, dReal re) {
	dMatrix3 R1, R2, R3, R4, R_le, R_lb, R_rb, R_re;

	// left side rotations
	dRFromAxisAndAngle(R1, R[1], R[5], R[9], lb);
	dRFromAxisAndAngle(R2, -R[0], -R[4], -R[8], le);
	dMultiply0(R_lb, R, R1, 3, 3, 3);
	dMultiply0(R_le, R_lb, R2, 3, 3, 3);
	
	// right side rotations
	dRFromAxisAndAngle(R3, -R[1], -R[5], -R[9], rb);
	dRFromAxisAndAngle(R4, R[0], R[4], R[8], re);
	dMultiply0(R_rb, R, R3, 3, 3, 3);
	dMultiply0(R_re, R_rb, R4, 3, 3, 3);


	robot_t *robot = buildRobotRotated2(world, space, x, y, z, R, le, lb, rb, re);

	return robot;
}

/*
 *	build copy of robot
 *		- ability to place at any rotation around z-axis
 */
robot_t *buildRobotRotated2(dWorldID *world, dSpaceID *space, dReal x, dReal y, dReal z, dMatrix3 R, dReal rle, dReal rlb, dReal rrb, dReal rre) {
	// create structure for data
	robot_t *robot;
	robot = (robot_t *)malloc(sizeof(robot_t));

	// pointers to structs for each part of robot
	robot->parts = (robot_part_t *)malloc(sizeof(robot_part_t) * NUM_PARTS);

	// flag for each part of body reaching final position
	for (int i = 0; i < NUM_BODIES; i++) robot->cmpStp[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->futEncCnt[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->curEncCnt[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->delEncCnt[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->futAng[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->curAng[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->pasAng[i] = 0;
	for (int i = 0; i < NUM_BODIES; i++) robot->delAng[i] = 0;
	for (int i = 0; i < 12; i++) robot->rotation[i] = R[i];

	// minimum velocity of each body
	robot->jntVelMin[LE] = OMEGA_LE_MIN;
	robot->jntVelMin[LB] = OMEGA_LB_MIN;
	robot->jntVelMin[RB] = OMEGA_RB_MIN;
	robot->jntVelMin[RE] = OMEGA_RE_MIN;
	// maximum velocity of each body
	robot->jntVelMax[LE] = OMEGA_LE_MAX;
	robot->jntVelMax[LB] = OMEGA_LB_MAX;
	robot->jntVelMax[RB] = OMEGA_RB_MAX;
	robot->jntVelMax[RE] = OMEGA_RE_MAX;
	// range of possible joint velocities
	robot->jntVelDel[LE] = robot->jntVelMax[LE] - robot->jntVelMin[LE];
	robot->jntVelDel[LB] = robot->jntVelMax[LB] - robot->jntVelMin[LB];
	robot->jntVelDel[RB] = robot->jntVelMax[RB] - robot->jntVelMin[RB];
	robot->jntVelDel[RE] = robot->jntVelMax[RE] - robot->jntVelMin[RE];
	// desired velocity of each body
	robot->jntVel[LE] = robot->jntVelMax[LE];
	robot->jntVel[LB] = robot->jntVelMax[LB];
	robot->jntVel[RB] = robot->jntVelMax[RB];
	robot->jntVel[RE] = robot->jntVelMax[RE];

	// angle between each interrupt in motor
	robot->radPerEnc[LE] = RAD_PER_INTERRUPT_LE;
	robot->radPerEnc[LB] = RAD_PER_INTERRUPT_LB;
	robot->radPerEnc[RB] = RAD_PER_INTERRUPT_RB;
	robot->radPerEnc[RE] = RAD_PER_INTERRUPT_RE;

	// offset values for each body part[0-2] and joint[3-5] from center
	dReal le[6] = {-2.865/2-1.55-0.2-0.125/2, 0, 0, -2.865/2 - 1.55 - 0.2, 0, 0};
	dReal lb[6] = {-2.865/2-1.55-0.2/2, 0, 0, -2.865/2, 1.3, 0};
	dReal ce[3] = {0, 0, 0};
	dReal rb[6] = {2.865/2+1.55+0.2/2, 0, 0, 2.865/2, 1.3, 0};
	dReal re[6] = {2.865/2+1.55+0.2+0.125/2, 0, 0, 2.865/2 + 1.55 + 0.2, 0, 0};

	// rotations of body parts about center
	dMatrix3 R1, R2, R3, R4, R_le, R_lb, R_rb, R_re;
	// left side rotations
	dRFromAxisAndAngle(R1, R[1], R[5], R[9], rlb);
	dRFromAxisAndAngle(R2, -R[0], -R[4], -R[8], rle);
	dMultiply0(R_lb, R, R1, 3, 3, 3);
	dMultiply0(R_le, R_lb, R2, 3, 3, 3);
	// right side rotations
	dRFromAxisAndAngle(R3, -R[1], -R[5], -R[9], rrb);
	dRFromAxisAndAngle(R4, R[0], R[4], R[8], rre);
	dMultiply0(R_rb, R3, R, 3, 3, 3);
	dMultiply0(R_re, R_rb, R4, 3, 3, 3);

	// build pieces of robot
	buildEndcap(world, space, &(robot->parts[ENDCAP_L]),	I2M(R_lb[0]*le[0] + R_lb[1]*le[1] + R_lb[2]*le[2] + x),
															I2M(R_lb[4]*le[0] + R_lb[5]*le[1] + R_lb[6]*le[2] + y),
															I2M(R_lb[8]*le[0] + R_lb[9]*le[1] + R_lb[10]*le[2] + z + 1.425), R_le);
	buildLeftBody(world, space, &(robot->parts[BODY_L]),	I2M(R_lb[0]*lb[0] + R_lb[1]*lb[1] + R_lb[2]*lb[2] + x),
															I2M(R_lb[4]*lb[0] + R_lb[5]*lb[1] + R_lb[6]*lb[2] + y),
															I2M(R_lb[8]*lb[0] + R_lb[9]*lb[1] + R_lb[10]*lb[2] + z + 1.425), R_lb);
	buildCenter(world, space, &(robot->parts[CENTER]),		I2M(R[0]*ce[0] + R[1]*ce[1] + R[2]*ce[2] + x),
															I2M(R[4]*ce[0] + R[5]*ce[1] + R[6]*ce[2] + y),
															I2M(R[8]*ce[0] + R[9]*ce[1] + R[10]*ce[2] + z + 1.425), R);
	buildRightBody(world, space, &(robot->parts[BODY_R]),	I2M(R[0]*rb[0] + R[1]*rb[1] + R[2]*rb[2] + x),
															I2M(R[4]*rb[0] + R[5]*rb[1] + R[6]*rb[2] + y),
															I2M(R[8]*rb[0] + R[9]*rb[1] + R[10]*rb[2] + z + 1.425), R_rb);
	buildEndcap(world, space, &(robot->parts[ENDCAP_R]),	I2M(R_re[0]*re[0] + R_re[1]*re[1] + R_re[2]*re[2] + x),
															I2M(R_re[4]*re[0] + R_re[5]*re[1] + R_re[6]*re[2] + y),
															I2M(R_re[8]*re[0] + R_re[9]*re[1] + R_re[10]*re[2] + z + 1.425), R_re);
	robot->position[0] = I2M(R[0]*ce[0] + R[1]*ce[1] + R[2]*ce[2] + x);
	robot->position[1] = I2M(R[4]*ce[0] + R[5]*ce[1] + R[6]*ce[2] + y);
	robot->position[2] = I2M(R[8]*ce[0] + R[9]*ce[1] + R[10]*ce[2] + z);

	// create joint
	dJointID joint;

	// joint for left body to endcap
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[BODY_L].bodyID, robot->parts[ENDCAP_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R_lb[0]*le[3] + R_lb[1]*le[4] + R_lb[2]*le[5] + x),
								I2M(R_lb[4]*le[3] + R_lb[5]*le[4] + R_lb[6]*le[5] + y),
								I2M(R_lb[8]*le[3] + R_lb[9]*le[4] + R_lb[10]*le[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R_lb[0]*1 + R_lb[1]*0 + R_lb[2]*0,
								R_lb[4]*1 + R_lb[5]*0 + R_lb[6]*0,
								R_lb[8]*1 + R_lb[9]*0 + R_lb[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[0] = joint;

	// joint for center to left body 1
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[CENTER].bodyID, robot->parts[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*lb[3] + R[1]*lb[4] + R[2]*lb[5] + x),
								I2M(R[4]*lb[3] + R[5]*lb[4] + R[6]*lb[5] + y),
								I2M(R[8]*lb[3] + R[9]*lb[4] + R[10]*lb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*-1 + R[2]*0,
								R[4]*0 + R[5]*-1 + R[6]*0,
								R[8]*0 + R[9]*-1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[1] = joint;

	// joint for center to left body 2
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[CENTER].bodyID, robot->parts[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*lb[3] - R[1]*lb[4] + R[2]*lb[5] + x),
								I2M(R[4]*lb[3] - R[5]*lb[4] + R[6]*lb[5] + y),
								I2M(R[8]*lb[3] - R[9]*lb[4] + R[10]*lb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*1 + R[2]*0,
								R[4]*0 + R[5]*1 + R[6]*0,
								R[8]*0 + R[9]*1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[4] = joint;

	// joint for center to right body 1
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[CENTER].bodyID, robot->parts[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*rb[3] + R[1]*rb[4] + R[2]*rb[5] + x),
								I2M(R[4]*rb[3] + R[5]*rb[4] + R[6]*rb[5] + y),
								I2M(R[8]*rb[3] + R[9]*rb[4] + R[10]*rb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*1 + R[2]*0,
								R[4]*0 + R[5]*1 + R[6]*0,
								R[8]*0 + R[9]*1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[2] = joint;

	// joint for center to right body 2
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[CENTER].bodyID, robot->parts[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*rb[3] - R[1]*rb[4] + R[2]*rb[5] + x),
								I2M(R[4]*rb[3] - R[5]*rb[4] + R[6]*rb[5] + y),
								I2M(R[8]*rb[3] - R[9]*rb[4] + R[10]*rb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*-1 + R[2]*0,
								R[4]*0 + R[5]*-1 + R[6]*0,
								R[8]*0 + R[9]*-1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[5] = joint;

	// joint for right body to endcap
	joint = dJointCreateHinge(*world, 0);
	dJointAttach(joint, robot->parts[BODY_R].bodyID, robot->parts[ENDCAP_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R_rb[0]*re[3] + R_rb[1]*re[4] + R_rb[2]*re[5] + x),
								I2M(R_rb[4]*re[3] + R_rb[5]*re[4] + R_rb[6]*re[5] + y),
								I2M(R_rb[8]*re[3] + R_rb[9]*re[4] + R_rb[10]*re[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R_rb[0]*-1 + R_rb[1]*0 + R_rb[2]*0,
								R_rb[4]*-1 + R_rb[5]*0 + R_rb[6]*0,
								R_rb[8]*-1 + R_rb[9]*0 + R_rb[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	robot->joints[3] = joint;

	// create motor
	dJointID motor;

	// motor for left endcap to body joint
	motor = dJointCreateAMotor(*world, 0);
	dJointAttach(motor, robot->parts[BODY_L].bodyID, robot->parts[ENDCAP_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R_lb[0], R_lb[4], R_lb[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, FMAX_LE);
	robot->motors[0] = motor;

	// motor for left body to center joint
	motor = dJointCreateAMotor(*world, 0);
	dJointAttach(motor, robot->parts[CENTER].bodyID, robot->parts[BODY_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, FMAX_LB);
	robot->motors[1] = motor;

	// motor for center to right body 1 joint
	motor = dJointCreateAMotor(*world, 0);
	dJointAttach(motor, robot->parts[CENTER].bodyID, robot->parts[BODY_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[1], R[5], R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, FMAX_RB);
	robot->motors[2] = motor;

	// motor for right body to endcap joint
	motor = dJointCreateAMotor(*world, 0);
	dJointAttach(motor, robot->parts[BODY_R].bodyID, robot->parts[ENDCAP_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R_rb[0], -R_rb[4], -R_rb[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, FMAX_RE);
	robot->motors[3] = motor;

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) {
		dBodySetDamping(robot->parts[i].bodyID, 0.1, 0.1);
	}

	return robot;
}
