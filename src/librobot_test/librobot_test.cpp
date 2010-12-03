/* ODE includes */
#include "config.h"
#include <ode/ode.h>
#ifdef ENABLE_DRAWSTUFF
#include <drawstuff/drawstuff.h>
#define DRAWSTUFF_TEXTURE_PATH "../opende/drawstuff/textures"
#endif
/* C++ includes */
#include <stdio.h>
#include <time.h>
/* local includes */
#include "librobot.h"
#include "input.h"

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

// dynamics and collision objects
static dWorldID g_world;
static dSpaceID g_space;
static dSpaceID g_space_robots[NUM_ROBOTS];
static dJointGroupID g_group;

// create structure of robots
robots_t *g_robots;

// simulation functions
void nearCallback(void *data, dGeomID o1, dGeomID o2);
#ifdef ENABLE_DRAWSTUFF
static void start();
void drawPart(dGeomID part);
void command(int cmd);
void simulationLoop(int pause);
#else
void simulationLoop(void);
#endif

int main(int argc, char **argv) {
	// start timekeeping
	time_t begin, begin2;
	begin = time(NULL);

	// convert degrees to radians
	for (int i = 0; i < NUM_STEPS; i++) {
		for (int j = 0; j < 4*NUM_ROBOTS; j++) {
			ang[i][j] = D2R(ang[i][j]);
		}
	}

#ifdef ENABLE_DRAWSTUFF
  /* Drawstuff */
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simulationLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
#endif

	// create world
	dInitODE2(0);
	g_world  = dWorldCreate();
	g_space  = dHashSpaceCreate(0);
	g_group  = dJointGroupCreate(1000000);
	g_robots = createWorldRobots(NUM_ROBOTS);
	for ( int i = 0; i < NUM_ROBOTS; i++ ) {
		g_space_robots[i] = dHashSpaceCreate(g_space);
		dSpaceSetCleanup(g_space_robots[i],1);
	}
	dWorldSetAutoDisableFlag(g_world, 1);					// auto-disable bodies that are not moving
	dWorldSetAutoDisableAngularThreshold(g_world, 0.01);	// threshold velocity for defining movement
	dWorldSetAutoDisableLinearThreshold(g_world, 0.01);		// linear velocity threshold
	dWorldSetAutoDisableSteps(g_world, 10);					// number of steps below thresholds before stationary
	dWorldSetCFM(g_world, 0.0000000001);					// constraint force mixing - how much a joint can be violated by excess force
	dWorldSetContactSurfaceLayer(g_world, 0.001);			// depth each body can sink into another body before resting
	dWorldSetDamping(g_world, 0.1, 0.1);					// damping of all bodies in world
	dWorldSetERP(g_world, 0.95);							// error reduction parameter (0-1) - how much error is corrected on each step
	dWorldSetGravity(g_world, 0, 0, -9.81);					// gravity

	/* ---- create ground ---- */
	/*// use trimesh for ground
	int numv = sizeof(world_vertices) / (3*sizeof(float));
	int numi = sizeof(world_indices) / sizeof(dTriIndex);
	dTriMeshDataID data = dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildSingle(data, world_vertices, 3*sizeof(float), numv, world_indices, numi, 3*sizeof(dTriIndex) );
	dGeomID ground = dCreateTriMesh(*space, data, 0, 0, 0);
	dGeomSetPosition(ground, 0, 0, 0 );
	*/
	// build ground out of geometric objects
	dGeomID ground = dCreatePlane(g_space, 0, 0, 1, 0);
	dGeomID ground_box = dCreateBox(g_space, 2, 1.5, 2);
	dGeomSetPosition(ground_box, 2, 0, 0);

	/* ---- create robots ---- */
	//dMatrix3 R;
	//dRFromAxisAndAngle(R,1,0,0,M_PI/2);
	g_robots->robots[0] = buildRobot(&g_world, &g_space_robots[0], 0, -4, 0);
	//g_robots->robots[0] = buildRobotRotated(&g_world, &g_space_robots[0], 0, 0, 0, R);
	//g_robots->robots[0] = buildRobotPositioned(&g_world, &g_space_robots[0], 0, 0, 0, R, 0, -45, -45, 0);

	// build dog
	//g_robots->robots[0] = buildRobot(&g_world, &g_space_robots[0], 0, -4, 0);
	//g_robots->robots[1] = buildRobotAttached(&g_world, &g_space_robots[1], g_robots, 0, FACE2, FACE6);
	//g_robots->robots[2] = buildRobotAttached(&g_world, &g_space_robots[2], g_robots, 0, FACE3, FACE6);
	//g_robots->robots[3] = buildRobotAttached(&g_world, &g_space_robots[3], g_robots, 0, FACE4, FACE6);
	//g_robots->robots[4] = buildRobotAttached(&g_world, &g_space_robots[4], g_robots, 0, FACE5, FACE6);

	// simulation
#ifdef ENABLE_DRAWSTUFF
	dsSimulationLoop(argc, argv, 352, 288, &fn);
#else
	simulationLoop();
#endif

	// free memory
	for (int i = 0; i < NUM_ROBOTS; i++) {
		delete(g_robots->robots[i]->parts);
		delete(g_robots->robots[i]);
	}
	delete(g_robots->flags);
	delete(g_robots->disable);

	// destroy world
	dJointGroupDestroy(g_group);
	for ( int i = 0; i < NUM_ROBOTS; i++ ) {
		dSpaceDestroy(g_space_robots[i]);
	}
	dSpaceDestroy(g_space);
	dWorldDestroy(g_world);
	dCloseODE();

	// stop timekeeping
	begin2 = time(NULL);
	printf("total	time: %lld %lld\n", begin2, begin);
	system("pause");
	
	return 0;
}

#ifdef ENABLE_DRAWSTUFF
static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);

	static float xyz[3] = {I2M(10), I2M(-10), I2M(10)};
	static float hpr[3] = {135.0, -20.0, 0.0};	// defined in degrees
	dsSetViewpoint(xyz, hpr);
}

/*
 *	called when a key pressed
 */
void command(int cmd) {
	switch (cmd) {
		case 'q': case 'Q':
			dsStop();
	}
}

void drawPart(dGeomID geom) {
	// make sure correct geom is passed to function
	if (!geom) return;

	// get pos, rot of body
	const dReal *position = dGeomGetPosition(geom);
	const dReal *rotation = dGeomGetRotation(geom);
	
	switch (dGeomGetClass(geom)) {
		case dSphereClass:
			{
				dReal r = dGeomSphereGetRadius(geom);
				dsDrawSphere(position, rotation, r);
				break;
			}
		case dBoxClass:
			{
				dVector3 sides;
				dGeomBoxGetLengths(geom, sides);
				dsDrawBox(position, rotation, sides);
				break;
			}
		case dCylinderClass:
			{
				dReal r, l;
				dGeomCylinderGetParams(geom, &r, &l);
				dsDrawCylinder(position, rotation, l, r);
				break;
			}
		case dCapsuleClass:
			{
				dReal r, l;
				dGeomCapsuleGetParams(geom, &r, &l);
				dsDrawCapsule(position, rotation, l, r);
			}
	}
}
#endif

/*
 *	check whether bodies are in contact
 */
void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	// get two bodies that might be in contact
	dBodyID b1, b2;
	b1 = dGeomGetBody(o1);
	b2 = dGeomGetBody(o2);

	// exit without doing anything if the two bodies are connected by a joint
	if (b1 && b2 && dAreConnected(b1, b2)) return;
	
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		// colliding a space with something:
		dSpaceCollide2(o1, o2, data, &nearCallback);
		// collide all geoms internal to the space(s)
		if ( dGeomIsSpace(o1) )
			dSpaceCollide( (dSpaceID)o1, data, &nearCallback);
		if ( dGeomIsSpace(o2) )
			dSpaceCollide( (dSpaceID)o2, data, &nearCallback);
	}
	else {
		// create contact points between geoms (up to N)
		const int N = 8;
		dContact contact[N];
		int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
		if (n > 0) {
			for (int i = 0; i < n; i++) {
				if ( (dSpaceID)o1 == g_space || (dSpaceID)o2 == g_space ) {
					contact[i].surface.mode = dContactBounce | dContactApprox1;
					contact[i].surface.mu = MU_GROUND;
					contact[i].surface.bounce = COR_GROUND;
				}
				else {
					contact[i].surface.mode = dContactBounce | dContactApprox1;
					contact[i].surface.mu = MU_BODY;
					contact[i].surface.bounce = COR_BODY;
				}
				dJointID c = dJointCreateContact(g_world,g_group,contact+i);
				dJointAttach(c, b1, b2);
			}
		}
	}
}

/*
 *	simulation loop
 */
#ifdef ENABLE_DRAWSTUFF
void simulationLoop(int pause) 
#else
void simulationLoop(void) 
#endif
{
	// time of simulation
	static double t = 0.0;
	// time of each simulation step
	double tStep = 0.004;			// tStep <= 0.006 to prevent overstepping interrupt in one time step
	// time to completion counter
	static int tComp = 0;
	// change on each new step
	static int newstep = 1;
	// loop counters
	int i = 0;
	int j = 0;
	static int flags[NUM_ROBOTS] = {0};

#ifndef ENABLE_DRAWSTUFF
	while (1) {
#endif
		if (newstep) {
			newstep = 0;
			for (i = 0; i < NUM_ROBOTS; i++) {
				for (j = 0; j < NUM_BODIES; j++) {
					if ( ang[g_robots->step-1][4*i + j] == D2R(123456789) ) {
						dJointDisable(g_robots->robots[i]->motors[j]);
						if ( j == LE || j == RE ) {
							g_robots->robots[i]->futAng[j] = angMod(g_robots->robots[i]->curAng[j],
												dJointGetHingeAngle(g_robots->robots[i]->joints[j]),
												dJointGetHingeAngleRate(g_robots->robots[i]->joints[j]));
						}
						else {
							g_robots->robots[i]->futAng[j] = dJointGetHingeAngle(g_robots->robots[i]->joints[j]);
						}
						g_robots->robots[i]->curAng[j] = g_robots->robots[i]->futAng[j];
						g_robots->robots[i]->pasAng[j] = g_robots->robots[i]->curEncCnt[j] * g_robots->robots[i]->radPerEnc[j];
						g_robots->robots[i]->futEncCnt[j] = g_robots->robots[i]->curEncCnt[j];
						dJointSetAMotorAngle(g_robots->robots[i]->motors[j], 0, g_robots->robots[i]->curAng[j]);
						g_robots->robots[i]->cmpStp[j] = 1;
					}
					else {
						dJointEnable(g_robots->robots[i]->motors[j]);
						if ( j == LE || j == RE ) {
							g_robots->robots[i]->futAng[j] += ang[g_robots->step-1][4*i+j];
							g_robots->robots[i]->curAng[j] = angMod(g_robots->robots[i]->curAng[j],
												dJointGetHingeAngle(g_robots->robots[i]->joints[j]),
												dJointGetHingeAngleRate(g_robots->robots[i]->joints[j]));
						}
						else {
							g_robots->robots[i]->futAng[j] = ang[g_robots->step-1][4*i + j];
							g_robots->robots[i]->curAng[j] = dJointGetHingeAngle(g_robots->robots[i]->joints[j]);
						}
						g_robots->robots[i]->pasAng[j] = g_robots->robots[i]->curEncCnt[j] * g_robots->robots[i]->radPerEnc[j];
						g_robots->robots[i]->delAng[j] = g_robots->robots[i]->futAng[j] - g_robots->robots[i]->pasAng[j];
						g_robots->robots[i]->delEncCnt[j] = (int)(g_robots->robots[i]->delAng[j]/g_robots->robots[i]->radPerEnc[j] + 0.5);
						g_robots->robots[i]->futEncCnt[j] += g_robots->robots[i]->delEncCnt[j];
						g_robots->robots[i]->jntVel[j] = g_robots->robots[i]->jntVelMin[j] + vel[g_robots->step-1][4*i + j]*g_robots->robots[i]->jntVelDel[j];
						dJointSetAMotorAngle(g_robots->robots[i]->motors[j], 0, g_robots->robots[i]->curAng[j]);
					}
				}
			}
		}
		else {
			for (i = 0; i < NUM_ROBOTS; i++) {
				for (j = 0; j < NUM_BODIES; j++) {
					if ( j == LE || j == RE ) {
						g_robots->robots[i]->curAng[j] = angMod(g_robots->robots[i]->curAng[j],
											dJointGetHingeAngle(g_robots->robots[i]->joints[j]),
											dJointGetHingeAngleRate(g_robots->robots[i]->joints[j]));
					}
					else {
						g_robots->robots[i]->curAng[j] = dJointGetHingeAngle(g_robots->robots[i]->joints[j]);
					}
					dJointSetAMotorAngle(g_robots->robots[i]->motors[j], 0, g_robots->robots[i]->curAng[j]);
					g_robots->robots[i]->delAng[j] = g_robots->robots[i]->curAng[j] - g_robots->robots[i]->pasAng[j];
					if ( fabs(g_robots->robots[i]->delAng[j]) >= g_robots->robots[i]->radPerEnc[j] ) {
						g_robots->robots[i]->delEncCnt[j] = (int)(g_robots->robots[i]->delAng[j] / g_robots->robots[i]->radPerEnc[j]);
						g_robots->robots[i]->curEncCnt[j] += g_robots->robots[i]->delEncCnt[j];
						g_robots->robots[i]->pasAng[j] += g_robots->robots[i]->delEncCnt[j] * g_robots->robots[i]->radPerEnc[j];
					}
					if ( !dJointIsEnabled(g_robots->robots[i]->motors[j]) ) {
						g_robots->robots[i]->futAng[j] = g_robots->robots[i]->curAng[j];
						g_robots->robots[i]->pasAng[j] = g_robots->robots[i]->curEncCnt[j] * g_robots->robots[i]->radPerEnc[j];
						g_robots->robots[i]->futEncCnt[j] = g_robots->robots[i]->curEncCnt[j];
					}
					else {
						if (g_robots->robots[i]->curEncCnt[j] < g_robots->robots[i]->futEncCnt[j]) {
							dJointSetAMotorParam(g_robots->robots[i]->motors[j], dParamVel, g_robots->robots[i]->jntVel[j]);
							g_robots->robots[i]->cmpStp[j] = 0;
						}
						else if (g_robots->robots[i]->curEncCnt[j] > g_robots->robots[i]->futEncCnt[j]) {
							dJointSetAMotorParam(g_robots->robots[i]->motors[j], dParamVel, -g_robots->robots[i]->jntVel[j]);
							g_robots->robots[i]->cmpStp[j] = 0;
						}
						else {
							dJointSetAMotorParam(g_robots->robots[i]->motors[j], dParamVel, 0);
							g_robots->robots[i]->cmpStp[j] = 1;
						}
					}
				}
			}
		}

		// collisions in the world
		dSpaceCollide(g_space, 0, &nearCallback);
		dWorldStep(g_world, tStep);
		dJointGroupEmpty(g_group);

		for (i = 0; i < NUM_ROBOTS; i++) {
			// check that each robot has completed step
			if ( minVal(g_robots->robots[i]->cmpStp, NUM_BODIES) )
				//g_robots->flags[i] = 1;
				flags[i] = 1;
			// check that each robot is enabled
			if ( !dBodyIsEnabled(g_robots->robots[i]->parts[0].bodyID) )
				g_robots->disable[i] = 1;
			else
				g_robots->disable[i] = 0;
		}

		// if all robots have completed the step, move onto next one
		//if ( minVal(g_robots->flags, NUM_ROBOTS) ) 
		if ( minVal(flags, NUM_ROBOTS) ) {
			g_robots->step++;
			newstep = 1;
			for (i = 0; i < NUM_ROBOTS; i++) {
				//g_robots->flags[i] = 0;
				flags[i] = 0;
				for (j = 0; j < NUM_BODIES; j++) {
					g_robots->robots[i]->cmpStp[j] = 0;
				}
			}
			if ( g_robots->step - 1 == NUM_STEPS ) {
				tComp++;
				if ( tComp == 1 ) {
					g_robots->success = 1;
					g_robots->time = t;
				}
				g_robots->step = NUM_STEPS;
				newstep = 0;
			}
		}

		// output to console
		//printf("%d ", t/tStep);
		//printf("%f ", t);
		//printf("%d ", g_robots->step);
		//printf("%d ", tComp);
		//printf("%d\t", g_robots->success);
		//const dReal *pos, *lin, *ang;
		for (i = 0; i < NUM_ROBOTS; i++) {
			//pos = dBodyGetPosition(g_robots->robots[i]->parts[ENDCAP_L].bodyID);
			//printf("[%f %f %f]\t", M2I(pos[0]), M2I(pos[1]), M2I(pos[2]));
			//lin = dBodyGetLinearVel(g_robots->robots[i]->parts[ENDCAP_L].bodyID);
			//printf("[%f %f %f]\t", lin[0], lin[1], lin[2] );
			//ang = dBodyGetAngularVel(g_robots->robots[i]->parts[ENDCAP_L].bodyID);
			//printf("[%f %f %f]\t", ang[0], ang[1], ang[2] );
			//printf("%f ", g_robots->robots[i]->futAng[LE] );
			//printf("%f\t", g_robots->robots[i]->curAng[LE] );
			//printf("%f\t", g_robots->robots[i]->pasAng[LE] );
			//printf("%d ", g_robots->robots[i]->futEncCnt[LE]);
			//printf("%d ", g_robots->robots[i]->curEncCnt[LE]);
			//printf("%f ", g_robots->robots[i]->radPerEnc[LE] );
			//printf("%.2f ", g_robots->robots[i]->jntVel[LE] );
			//printf("%d\t", g_robots->robots[i]->cmpStp[LE]);
			//printf("%f\t", dJointGetAMotorParam(g_robots->robots[i]->motors[LE], dParamVel));
			//printf("%f ", dJointGetHingeAngle(g_robots->robots[i]->joints[LE]));
			//printf("%f\t", dJointGetHingeAngleRate(g_robots->robots[i]->joints[LE]));
			//
			//const dReal *test = dBodyGetPosition(g_robots->robots[i]->parts[BODY_L].bodyID);
			//printf("[%f %f %f]\t", M2I(test[0]), M2I(test[1]), M2I(test[2]));
			//const dReal *lin = dBodyGetLinearVel(g_robots->robots[i]->parts[BODY_L].bodyID);
			//printf("[%f %f %f]\t", lin[0], lin[1], lin[2] );
			//printf("%f ", g_robots->robots[i]->futAng[LB] );
			///printf("%f ", g_robots->robots[i]->curAng[LB] );
			//printf("%f\t", g_robots->robots[i]->pasAng[LB] );
			//printf("%d ", g_robots->robots[i]->futEncCnt[LB]);
			//printf("%d ", g_robots->robots[i]->curEncCnt[LB]);
			//printf("%f ", g_robots->robots[i]->radPerEnc[LB] );
			//printf("%.2f ", g_robots->robots[i]->jntVel[LB] );
			//printf("%d\t", g_robots->robots[i]->cmpStp[LB]);
			//printf("%f\t", dJointGetAMotorParam(g_robots->robots[i]->motors[LB], dParamVel));
			//printf("%f\t", dJointGetHingeAngle(g_robots->robots[i]->joints[LB]));
			//printf("%f\t", dJointGetHingeAngleRate(g_robots->robots[i]->joints[LB]));
			//
			//pos = dBodyGetPosition(g_robots->robots[i]->parts[CENTER].bodyID);
			//printf("[%f %f %f]\t", M2I(pos[0]), M2I(pos[1]), M2I(pos[2]));
			//lin = dBodyGetLinearVel(g_robots->robots[i]->parts[CENTER].bodyID);
			//printf("[%f %f %f]\t", lin[0], lin[1], lin[2] );
			//ang = dBodyGetAngularVel(g_robots->robots[i]->parts[CENTER].bodyID);
			//printf("[%f %f %f]\t", ang[0], ang[1], ang[2] );
			//
			//pos = dBodyGetPosition(g_robots->robots[i]->parts[BODY_R].bodyID);
			//printf("[%.2f %.2f %.2f]\t", M2I(pos[0]), M2I(pos[1]), M2I(pos[2]));
			//lin = dBodyGetLinearVel(g_robots->robots[i]->parts[BODY_R].bodyID);
			//printf("[%f %f %f]\n", lin[0], lin[1], lin[2] );
			//ang = dBodyGetAngularVel(g_robots->robots[i]->parts[BODY_R].bodyID);
			//printf("[%.2f %.2f %.2f]\t", ang[0], ang[1], ang[2] );
			printf("%f ", g_robots->robots[i]->futAng[RB] );
			printf("%f ", g_robots->robots[i]->curAng[RB] );
			printf("%f\t", g_robots->robots[i]->pasAng[RB] );
			//printf("%d ", g_robots->robots[i]->futEncCnt[RB]);
			//printf("%d ", g_robots->robots[i]->curEncCnt[RB]);
			//printf("%f ", g_robots->robots[i]->radPerEnc[RB] );
			//printf("%.2f ", g_robots->robots[i]->jntVel[RB] );
			//printf("%d\t", g_robots->robots[i]->cmpStp[RB]);
			//printf("%f\t", dJointGetHingeAngle(g_robots->robots[i]->joints[RB]) );
			//printf("%.2f\t", dJointGetHingeAngleRate(g_robots->robots[i]->joints[RB]));
			//printf("%.2f\t", dJointGetAMotorParam(g_robots->robots[i]->motors[RB], dParamVel) );
			//
			//pos = dBodyGetPosition(g_robots->robots[i]->parts[ENDCAP_R].bodyID);
			//printf("[%f %f %f]\t", M2I(pos[0]), M2I(pos[1]), M2I(pos[2]));
			//lin = dBodyGetLinearVel(g_robots->robots[i]->parts[ENDCAP_R].bodyID);
			//printf("[%f %f %f]\t", lin[0], lin[1], lin[2] );
			//ang = dBodyGetAngularVel(g_robots->robots[i]->parts[ENDCAP_R].bodyID);
			//printf("[%.2f %.2f %.2f]\t", ang[0], ang[1], ang[2] );
			//printf("%f ", g_robots->robots[i]->futAng[RE] );
			//printf("%f ", g_robots->robots[i]->curAng[RE] );
			//printf("%f\t", g_robots->robots[i]->pasAng[RE] );
			//printf("%d ", g_robots->robots[i]->futEncCnt[RE]);
			//printf("%d ", g_robots->robots[i]->curEncCnt[RE]);
			//printf("%d\t", g_robots->robots[i]->cmpStp[RE]);
			//printf("%f ", dJointGetAMotorParam(g_robots->robots[i]->motors[RE], dParamVel));
			//printf("%f ", dJointGetHingeAngle(g_robots->robots[i]->joints[RE]));
			//printf("%f", dJointGetHingeAngleRate(g_robots->robots[i]->joints[RE]));
		}
		//for (int i = 0; i < NUM_ROBOTS; i++) {
		//	printf("%d ", g_robots->flags[i]);
		//}
		//printf("\t");
		//for (int i = 0; i < NUM_ROBOTS; i++) {
		//	printf("%d ", g_robots->disable[i]);
		//}
		printf("\n");

		// stop simulation loop after set amount of time (failure)
		if ( (TIME_TOTAL - t) < 0.0000000596047 ) {
			printf("Failure: reached end of simulation time\n");
#ifdef ENABLE_DRAWSTUFF
      dsStop();
#else
			break;
#endif
		}
		// stop simulation loop if all robots are stationary
		if (minVal(g_robots->disable, NUM_ROBOTS)) {
			// if all robots have completed all steps == SUCCESS
			if ( g_robots->success ) {
				printf("Success: all robots have completed all of their steps\n");
#ifdef ENABLE_DRAWSTUFF
        dsStop();
#else
				break;
#endif
			}
			// else everything is stalled == FAILURE
			else {
				printf("Failure: all robots are stationary\n");
#ifdef ENABLE_DRAWSTUFF
        dsStop();
#else
				break;
#endif
			}
		}

		// increment time of simulation
		t += tStep;
#ifdef ENABLE_DRAWSTUFF
    // draw the bodies
    for (int i = 0; i < NUM_ROBOTS; i++) {
      for (int j = 0; j < NUM_PARTS; j++) {
        if (dBodyIsEnabled(g_robots->robots[i]->parts[j].bodyID)) {
          dsSetColor(	g_robots->robots[i]->parts[j].color[0],
              g_robots->robots[i]->parts[j].color[1],
              g_robots->robots[i]->parts[j].color[2]);
        }
        else { dsSetColor(0.5,0.5,0.5); }
        for (int k = 0; k < g_robots->robots[i]->parts[j].num_geomID; k++) {
          drawPart(g_robots->robots[i]->parts[j].geomID[k]);
        }
      }
    }
#endif

#ifndef ENABLE_DRAWSTUFF
	}
#endif
}
