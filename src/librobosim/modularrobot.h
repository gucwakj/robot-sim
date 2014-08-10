#ifndef MODULARROBOT_H_
#define MODULARROBOT_H_

#include "robot.h"

// connector
typedef struct conn_s {
	int face, type;
	int d_side, d_type;
	dBodyID body;
	dGeomID *geom;
	struct conn_s *next;
} *conn_t;

class DLLIMPORT ModularRobot : virtual public Robot {
		friend class RoboSim;
	// common public api
	public:
		ModularRobot(void);
		~ModularRobot(void);

		int connect(char* = NULL, int = 3);

		// TODO: make private-ish functions protected
		dBodyID getConnectorBodyID(int);
		virtual int getConnectionParams(int, dMatrix3, double*) = 0;

	// utility functions for inherited and friend classes
	protected:
		dBodyID getConnectorBodyIDs(int);

	// virual functions for inherited classes
	protected:
		virtual int build(xml_robot_t) = 0;
		virtual int build(xml_robot_t, ModularRobot*, xml_conn_t) = 0;
		virtual int buildIndividual(double, double, double, dMatrix3, double*) = 0;
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group*, int) = 0;
#endif // ENABLE_GRAPHICS
		virtual double getAngle(int) = 0;
		virtual int initParams(int, int) = 0;
		virtual int initDims(void) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;

	// data members
	protected:
		conn_t _conn;				// connectors
		double _conn_depth;			// dimension: connector depth
		double _conn_height;		// dimension: connector height
		double _conn_radius;		// dimension: connector radius
		double _smallwheel_radius;	// dimension: small wheel radius
};

#endif // MODULARROBOT_H_

