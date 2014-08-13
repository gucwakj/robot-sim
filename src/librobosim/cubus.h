#ifndef CUBUS_H_
#define CUBUS_H_

#include "config.h"
#include "robosim.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT Cubus : public ModularRobot {
		friend class cubusNodeCallback;

	// public api
	public:
		Cubus(void);
		virtual ~Cubus(void);

		int accelJointAngleNB(robotJointId_t, double, double);
		int accelJointCycloidalNB(robotJointId_t, double, double);
		int accelJointHarmonicNB(robotJointId_t, double, double);
		int accelJointSmoothNB(robotJointId_t, double, double, double, double);
		int accelJointTimeNB(robotJointId_t, double, double);
		int accelJointToMaxSpeedNB(robotJointId_t, double);
		int accelJointToVelocityNB(robotJointId_t, double, double);
		int driveAccelCycloidalNB(double, double, double);
		int driveAccelDistanceNB(double, double, double);
		int driveAccelHarmonicNB(double, double, double);
		int driveAccelSmoothNB(double, double, double, double, double);
		int driveAccelTimeNB(double, double, double);
		int driveAccelToMaxSpeedNB(double, double);
		int driveAccelToVelocityNB(double, double, double);
		int driveForeverNB(void);
		int driveForwardNB(double);
		int drivexyTo(double, double, double, double);
		int getJointAngles(double&, double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&, double&);
		int getJointSpeeds(double&, double&, double&);
		int getJointSpeedRatios(double&, double&, double&);
		int move(double, double, double);
		int moveNB(double, double, double);
		int moveTo(double, double, double);
		int moveToNB(double, double, double);
		int moveToByTrackPos(double, double, double);
		int moveToByTrackPosNB(double, double, double);
		int recordAngles(double[], double[], double[], double[], int, double, int = 1);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int setJointSpeeds(double, double, double);
		int setJointSpeedRatios(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRightNB(double, double, double);

	// inherited functions from ModularRobot class
	private:
		virtual int addConnector(int, int, double);
		virtual int build(xml_robot_t, dMatrix3, double*, dBodyID, xml_conn_t);
#ifdef ENABLE_GRAPHICS
		virtual int drawConnector(conn_t, osg::Group*);
#endif // ENABLE_GRAPHICS
		virtual int fixBodyToConnector(dBodyID, int);
		virtual int fixConnectorToBody(int, dBodyID, int = -1);
		virtual int getConnectorParams(int, int, dMatrix3, double*);
		virtual int getFaceParams(int, dMatrix3, double*);

	// inherited functions from Robot class
	private:
		virtual int build(xml_robot_t);
		virtual int buildIndividual(double, double, double, dMatrix3, double*);
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group*, int);
#endif // ENABLE_GRAPHICS
		virtual double getAngle(int);
		virtual int initParams(int, int);
		virtual int initDims(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

	// private functions
	private:
		int build_body(double, double, double, dMatrix3, double);
		int build_face(int, double, double, double, dMatrix3, double);
		int build_simple(conn_t, int face, int = -1, int = -1);

	// private data
	private:
		// robot body parts
		enum robot_pieces_e {
			BODY,
			FACE1,
			FACE2,
			FACE3,
			FACE4,
			FACE5,
			FACE6,
			NUM_PARTS
		};

		// dimensions
		double	_face_depth,
				_face_radius;
};

class DLLIMPORT CubusGroup : virtual public RobotGroup {
	// public api
	public:
		CubusGroup(void);
		virtual ~CubusGroup(void);
		int addRobot(Cubus&);
		int addRobots(Cubus[], int);

		int accelJointAngleNB(robotJointId_t, double, double);
		int accelJointCycloidalNB(robotJointId_t, double, double);
		int accelJointHarmonicNB(robotJointId_t, double, double);
		int accelJointSmoothNB(robotJointId_t, double, double, double, double);
		int accelJointTimeNB(robotJointId_t, double, double);
		int accelJointToMaxSpeedNB(robotJointId_t, double);
		int accelJointToVelocityNB(robotJointId_t, double, double);
		int driveAccelCycloidalNB(double, double, double);
		int driveAccelDistanceNB(double, double, double);
		int driveAccelHarmonicNB(double, double, double);
		int driveAccelSmoothNB(double, double, double, double, double);
		int driveAccelTimeNB(double, double, double);
		int driveAccelToMaxSpeedNB(double, double);
		int driveAccelToVelocityNB(double, double, double);
		int move(double, double, double);
		int moveNB(double, double, double);
		int moveTo(double, double, double);
		int moveToNB(double, double, double);
		int moveToByTrackPos(double, double, double);
		int moveToByTrackPosNB(double, double, double);
		int setJointSpeeds(double, double, double);
		int setJointSpeedRatios(double, double, double);

	// private data
	private:
		typedef struct robots_s {
			Cubus *robot;
			struct robots_s *next;
		} *robots_t;
		robots_t _robots;
};

// simulation
extern RoboSim *g_sim;

#endif // CUBUS_H_

