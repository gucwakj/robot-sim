#ifndef CMOBOTFD_H_
#define CMOBOTFD_H_

#include <iostream>
#include <stdbool.h>
#include "mobot.h"
#include "graphics.h"

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

class CMobotFD {
	//friend class ViewerFrameThread;
	friend class iMobotNodeCallback;
	public:
        CMobotFD(void);
		~CMobotFD(void);

		// set simulation variables
		void setCOR(dReal cor_g, dReal cor_b);
		void setMu(dReal mu_g, dReal mu_b);
		void setGroundBox(dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundSphere(dReal r, dReal px, dReal py, dReal pz);

		// build models of the iMobot
		void addiMobot(iMobotSim &imobot);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2);
		void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        //void iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		// build models of the Mobot
		void addMobot(mobotSim &mobot);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2);
		void addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        //void iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
	private:
		// private variables to store general information about simulation
		dWorldID world;						// world in which simulation occurs
		dSpaceID space;						// space for robots in which to live
		dJointGroupID group;				// group to store joints
		dGeomID *ground;					// ground (static) objects
		pthread_mutex_t ground_mutex;		// mutex for ground collisions
		int groundNumber;					// number of ground objects
		robotSim **robot[NUM_TYPES];		// array of robots
		pthread_mutex_t robot_mutex;		// mutex for robots
		int robotNumber[NUM_TYPES];			// number of robots
		pthread_t *robotThread[NUM_TYPES];	// thread for each robot
		pthread_t simulation;				// simulation thread
		dReal _time_step;					// time of each step of simulation
		dReal _mu[2];						// coefficient of friction [body/ground, body/body]
		dReal _cor[2];						// coefficient of restitution [body/ground, body/body]
		ViewerFrameThread *_osgThread;		// osg thread
		osg::Group *_osgRoot;				// osg root node

		// simulation functions
		int graphics_init(void);
		void print_intermediate_data(void);			// print data out at each time step for analysis
		static void* simulationThread(void *arg);
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class
};

class iMobotNodeCallback : public osg::NodeCallback {
    public:
        iMobotNodeCallback(CMobotFD *sim, int number, int part) : _sim(sim), _number(number), _part(part) {}
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
            osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *> (node);
            if (pat) {
				//osg::Vec3f current = mat->getPosition();
				const dReal *position = dBodyGetPosition(_sim->robot[IMOBOT][_number]->getBodyID(_part));
				const dReal *quaternion = dBodyGetQuaternion(_sim->robot[IMOBOT][_number]->getBodyID(_part));
				//printf("[%lf, %lf, %lf, %lf]\n", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
				osg::Vec3f pos = osg::Vec3f(position[0], position[1], position[2]);
                //osg::Quat quat = osg::Quat(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
                pat->setPosition(pos);
                //pat->setAttitude(quat);
            }
            traverse(node, nv);
        }
    private:
        int _number;
		int _part;
		CMobotFD *_sim;
};
#endif	/* CMOBOTFD_H_ */
