#ifndef NXT_H_
#define NXT_H_

#include "robosim.h"
#include "config.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CNXT : virtual public Robot {
		friend class nxtNodeCallback;

	// public api
	public:
		CNXT(void);
		virtual ~CNXT(void);

		int getJointAngles(double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&);
		int getJointSpeeds(double&, double&);
		int getJointSpeedRatios(double&, double&);
		int jumpTo(double, double);
		int jumpToNB(double, double);
		int move(double, double);
		int moveNB(double, double);
		int moveTo(double, double);
		int moveToNB(double, double);
		int recordAngles(double[], double[], double[], int, double, int = 1);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int setJointSpeeds(double, double);
		int setJointSpeedRatios(double, double);

	// inherited functions from Robot class
    private:
		int build(xml_robot_t);
		int buildIndividual(double, double, double, dMatrix3, double*);
#ifdef ENABLE_GRAPHICS
		int draw(osg::Group*, int);
#endif // ENABLE_GRAPHICS
		double getAngle(int);
		int initParams(int, int);
		int initDims(void);
		void simPreCollisionThread(void);
		void simPostCollisionThread(void);

	// private functions
    private:
		int build_body(double, double, double, dMatrix3, double);		// build body of linkbot
		int build_wheel(int, double, double, double, dMatrix3, double);	// build wheels of nxt

	// private data
	private:
		// robot body parts
		enum robot_pieces_e {
			BODY,
			WHEEL1,
			WHEEL2,
			NUM_PARTS
		};
};

class DLLIMPORT CNXTGroup : virtual public RobotGroup {
	// public api
	public:
		CNXTGroup(void);
		virtual ~CNXTGroup(void);
		int addRobot(CNXT&);
		int addRobots(CNXT[], int);

		int jumpTo(double, double);
		int jumpToNB(double, double);
		int move(double, double);
		int moveNB(double, double);
		int moveTo(double, double);
		int moveToNB(double, double);
		int setJointSpeeds(double, double);
		int setJointSpeedRatios(double, double);

	// private data
	private:
		typedef struct robots_s {
			CNXT *robot;
			struct robots_s *next;
		} *robots_t;
		robots_t _robots;
};

// simulation
extern RoboSim *g_sim;

#endif // NXT_H_

