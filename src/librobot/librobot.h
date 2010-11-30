#ifndef LIBROBOT_H_
#define LIBROBOT_H_

// for cross-compilation on windows/unix
#if defined(_WIN32)
#define EXTERN extern __declspec(dllexport)
#else
#define EXTERN extern
#endif

#define RAD_PER_INTERRUPT_LE	0.02094
#define RAD_PER_INTERRUPT_LB	0.00873
#define RAD_PER_INTERRUPT_RB	0.00873
#define RAD_PER_INTERRUPT_RE	0.02094
#define OMEGA_LE_MAX	6.70
#define OMEGA_LB_MAX	2.61
#define OMEGA_RB_MAX	2.61
#define OMEGA_RE_MAX	6.70
#define OMEGA_LE_MIN	3.22
#define OMEGA_LB_MIN	1.25
#define OMEGA_RB_MIN	1.25
#define OMEGA_RE_MIN	3.22
#define FMAX_LE		0.26
#define FMAX_LB		1.059
#define FMAX_RB		1.059
#define FMAX_RE		0.26
#define I2M(x)		((x)*(0.0254))		// convert inches to meters
#define M2I(x)		((x)/(0.0254))		// convert meters to inches
#define D2R(x)		((x)*(M_PI/180))	// convert degrees to radians
#define R2D(x)		((x)/(M_PI/180))	// convert radians to degrees
#define MU_BODY		0.1					// coefficient of friction (0-1) between parts of robot
#define COR_BODY	0.45				// coefficient of restitution (0-1) between parts of robot
// only needed until mass properties are gotten from cad
#define DENALUM		2700			// density of aluminum 6061 [kg/m^3]


enum robot_pieces_e {
	BODY_L,
	BODY_R,
	CENTER,
	ENDCAP_L,
	ENDCAP_R,
	NUM_PARTS
};

enum robot_bodies_e {
	LE,
	LB,
	RB,
	RE,
	NUM_BODIES
};

enum robot_faces_e {
	FACE1 = 1,
	FACE2,
	FACE3,
	FACE4,
	FACE5,
	FACE6
};

typedef struct robot_part_s {
	dBodyID bodyID;
	dGeomID *geomID;
	int num_geomID;
	float color[3];
} robot_part_t;

typedef struct robot_s {
	robot_part_t *parts;
	dJointID joints[6];
	dJointID motors[4];
	dVector4 futAng;
	dVector4 curAng;
	dVector4 pasAng;
	dVector4 delAng;
	dVector4 jntVel;
	dVector4 jntVelMax;
	dVector4 jntVelMin;
	dVector4 jntVelDel;
	dVector4 radPerEnc;
	int futEncCnt[4];
	int curEncCnt[4];
	int delEncCnt[4];
	int cmpStp[4];
	dVector3 position;
	dMatrix3 rotation;
} robot_t;

typedef struct robots_s {
	robot_t **robots;
	int num_robots;
	int *flags;
	int *disable;
	int step;
	int success;
	dReal time;
} robots_t;

/*
 *	create world for robots
 */
EXTERN robots_t *createWorldRobots(int num);

/*
 *	minimum value of array
 */
EXTERN int minVal(int *a, int length);

/*
 *	modify endcap angles to record continuously
 */
EXTERN dReal angMod(dReal pasAng, dReal curAng, dReal angRat);

/*
 *	build parts of the robot
 */
EXTERN void buildLeftBody(dWorldID *world, dSpaceID *space, robot_part_t *part, dReal x, dReal y, dReal z, dMatrix3 R);
EXTERN void buildRightBody(dWorldID *world, dSpaceID *space, robot_part_t *part, dReal x, dReal y, dReal z, dMatrix3 R);
EXTERN void buildCenter(dWorldID *world, dSpaceID *space, robot_part_t *part, dReal x, dReal y, dReal z, dMatrix3 R);
EXTERN void buildEndcap(dWorldID *world, dSpaceID *space, robot_part_t *part, dReal x, dReal y, dReal z, dMatrix3 R);

/*
 *	build model of robot
 */
// build in default orientation
EXTERN robot_t *buildRobot(dWorldID *world, dSpaceID *space, dReal x, dReal y, dReal z);
// ability to rotate about z-axis
EXTERN robot_t *buildRobotRotated(dWorldID *world, dSpaceID *space, dReal x, dReal y, dReal z, dMatrix3 R);
// build robot attached to an already built robot
EXTERN robot_t *buildRobotAttached(dWorldID *world, dSpaceID *space, robots_t *robots, int robot_num, int face1, int face2);
EXTERN robot_t *buildRobotRotated2(dWorldID *world, dSpaceID *space, dReal x, dReal y, dReal z, dMatrix3 R, dReal rle, dReal rlb, dReal rrb, dReal rre);
EXTERN robot_t *buildRobotPositioned(dWorldID *world, dSpaceID *space, dReal x, dReal y, dReal z, dMatrix3 R, dReal le, dReal lb, dReal rb, dReal re);

#endif	/* LIBROBOT_H_ */
