#ifndef ROBOT_H_
#define ROBOT_H_
class CRobot4Sim;
class robotSim {
	public:
		static void* simPreCollisionThreadEntry(void *arg) { robotSim *p = (robotSim *)arg; p->simPreCollisionThread(); }
		static void* simPostCollisionThreadEntry(void *arg) { robotSim *p = (robotSim *)arg; p->simPostCollisionThread(); }

		virtual dReal getAngle(int i) = 0;
		virtual bool getSuccess(int i) = 0;
		virtual dReal getPosition(int i) = 0;
		virtual dReal getRotation(int i) = 0;
		virtual dBodyID getBodyID(int body) = 0;
		virtual dJointID getMotorID(int motor) = 0;

		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) = 0;
		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;
		virtual void buildAttached00(CRobot4Sim *attach, int face1, int face2) = 0;
		virtual void buildAttached10(CRobot4Sim *attach, int face1, int face2) = 0;
		virtual void buildAttached01(CRobot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;
		virtual void buildAttached11(CRobot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;

		virtual bool isHome(void) = 0;

		virtual void simAddRobot(dWorldID &world, dSpaceID &space) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;
	private:
};

#endif /* ROBOT_H_ */
