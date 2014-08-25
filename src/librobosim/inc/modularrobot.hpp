#ifndef MODULARROBOT_HPP_
#define MODULARROBOT_HPP_

#include "config.h"
#include "robot.hpp"

// connector
struct Connector {
	dBodyID body;
	dGeomID *geom;
	double o[3];
	int face, type;
	int d_side, d_type;
};

class DLLIMPORT ModularRobot : virtual public Robot {
		friend class RoboSim;

	// common public api
	public:
		ModularRobot(void);
		virtual ~ModularRobot(void);

		int connect(char* = NULL, int = 3);

	// utility functions for inherited and friend classes
	protected:
		dBodyID getConnectorBodyID(int);

	// virual functions for inherited classes
	protected:
		virtual int addConnector(int, int, double) = 0;
		virtual int build(xml_robot_t, dMatrix3, double*, dBodyID, xml_conn_t) = 0;
#ifdef ENABLE_GRAPHICS
		virtual int drawConnector(Connector*, osg::Group*) = 0;
#endif // ENABLE_GRAPHICS
		virtual int fixBodyToConnector(dBodyID, int) = 0;
		virtual int fixConnectorToBody(int, dBodyID, int = -1) = 0;
		virtual int getConnectorParams(int, int, dMatrix3, double*) = 0;
		virtual int getFaceParams(int, dMatrix3, double*) = 0;

	// virtual functions from Robot class
	protected:
		virtual int build(xml_robot_t) { return 0; };
		virtual int buildIndividual(double, double, double, dMatrix3, double*) { return 0; };
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group*, int) { return 0; };
#endif // ENABLE_GRAPHICS
		virtual double getAngle(int) { return 0; };
		virtual int initParams(int, int) { return 0; };
		virtual int initDims(void) { return 0; };
		virtual void simPreCollisionThread(void) { return; };
		virtual void simPostCollisionThread(void) { return; };

	// data members
	protected:
		std::vector<Connector*> _conn;	// connectors
		double _bigwheel_radius;		// dimension: big wheel radius
		double _conn_depth;				// dimension: connector depth
		double _conn_height;			// dimension: connector height
		double _conn_radius;			// dimension: connector radius
		double _smallwheel_radius;		// dimension: small wheel radius
};

#endif // MODULARROBOT_HPP_

