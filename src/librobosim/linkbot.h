#ifndef LINKBOT_H_
#define LINKBOT_H_

#include "config.h"
#include "robosim.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CLinkbotT : public ModularRobot {
		friend class linkbotNodeCallback;

	// public api
	public:
		CLinkbotT(int disabled = -1, int type = LINKBOTT);
		virtual ~CLinkbotT();

		int accelJointAngleNB(robotJointId_t, double, double);
		int accelJointCycloidalNB(robotJointId_t, double, double);
		int accelJointHarmonicNB(robotJointId_t, double, double);
		int accelJointSmoothNB(robotJointId_t, double, double, double, double);
		int accelJointTimeNB(robotJointId_t, double, double);
		int accelJointToMaxSpeedNB(robotJointId_t, double);
		int accelJointToVelocityNB(robotJointId_t, double, double);
		int closeGripper(void);
		int closeGripperNB(void);
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
		int drivexyToSmooth(double, double, double, double, double, double, double, double);
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
		int openGripper(double);
		int openGripperNB(double);
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
		int add_connector_daisy(int, int, double, int, int);			// add daisy chained connector
		int build_bigwheel(conn_t, int, int = -1, int = -1);			// build big wheel connector
		int build_body(double, double, double, dMatrix3, double);		// build body of linkbot
		int build_bridge(conn_t, int, int = -1, int = -1);				// build bridge connector
		int build_caster(conn_t, int, int, int = -1, int = -1);			// build caster connector
		int build_cube(conn_t, int, int = -1, int = -1);				// build cube connector
		int build_face(int, double, double, double, dMatrix3, double);	// build face of linkbot
		int build_faceplate(conn_t, int, int = -1, int = -1);			// build faceplate connector
		int build_gripper(conn_t, int);									// build gripper connector
		int build_omnidrive(conn_t, int, int = -1, int = -1);			// build omnidrive connector
		int build_simple(conn_t, int face, int = -1, int = -1);			// build simple connector
		int build_smallwheel(conn_t, int, int = -1, int = -1);			// build small wheel connector
		int build_tinywheel(conn_t, int, int = -1, int = -1);			// build tiny wheel connector
		int build_wheel(conn_t, int, double, int = -1, int = -1);		// build custom wheel connector
#ifdef ENABLE_GRAPHICS
		int draw_custom_caster(conn_t conn, osg::Group *robot);			// draw custom sized caster
#endif // ENABLE_GRAPHICS
		static void* closeGripperNBThread(void*);						// thread to close gripper

	// private data
	private:
		// robot body parts
		enum robot_pieces_e {
			BODY,
			FACE1,
			FACE2,
			FACE3,
			NUM_PARTS
		};

		// dimensions
		double	_bridge_length,
				_cubic_length,
				_face_depth,
				_face_radius,
				_omni_length,
				_tinywheel_radius;
};

class DLLIMPORT CLinkbotI : public CLinkbotT {
	public:
		CLinkbotI(void) : Robot(JOINT1, JOINT3), CLinkbotT(1, LINKBOTI) {}
};

class DLLIMPORT CLinkbotL : public CLinkbotT {
	public:
		CLinkbotL(void) : Robot(JOINT1, JOINT2), CLinkbotT(2, LINKBOTL) {}
};

class DLLIMPORT CLinkbotTGroup : public Group<CLinkbotT> {
	// public api
	public:
		CLinkbotTGroup(void) : Group<CLinkbotT>() {};
		virtual ~CLinkbotTGroup(void) {};

		inline int accelJointAngleNB(robotJointId_t, double, double);
		inline int accelJointCycloidalNB(robotJointId_t, double, double);
		inline int accelJointHarmonicNB(robotJointId_t, double, double);
		inline int accelJointSmoothNB(robotJointId_t, double, double, double, double);
		inline int accelJointTimeNB(robotJointId_t, double, double);
		inline int accelJointToMaxSpeedNB(robotJointId_t, double);
		inline int accelJointToVelocityNB(robotJointId_t, double, double);
		inline int closeGripper(void);
		inline int closeGripperNB(void);
		inline int driveAccelCycloidalNB(double, double, double);
		inline int driveAccelDistanceNB(double, double, double);
		inline int driveAccelHarmonicNB(double, double, double);
		inline int driveAccelSmoothNB(double, double, double, double, double);
		inline int driveAccelTimeNB(double, double, double);
		inline int driveAccelToMaxSpeedNB(double, double);
		inline int driveAccelToVelocityNB(double, double, double);
		inline int move(double, double, double);
		inline int moveNB(double, double, double);
		inline int moveTo(double, double, double);
		inline int moveToNB(double, double, double);
		inline int moveToByTrackPos(double, double, double);
		inline int moveToByTrackPosNB(double, double, double);
		inline int openGripper(double);
		inline int openGripperNB(double);
		inline int setJointSpeeds(double, double, double);
		inline int setJointSpeedRatios(double, double, double);
};
class DLLIMPORT CLinkbotIGroup : public CLinkbotTGroup {};
class DLLIMPORT CLinkbotLGroup : public CLinkbotTGroup {};
#include "linkbotgroup.cpp"

// global structs for threading
typedef struct linkbotMoveArg_s {
	double x, y, radius, trackwidth;
	int i;
	double (*func)(double x);
	char *expr;
	CLinkbotT *robot;
} linkbotMoveArg_t;

// simulation
extern RoboSim *g_sim;

#endif // LINKBOT_H_

