#ifndef CIMOBOTIK_H_
#define CIMOBOTIK_H_

#include "config.h"
#include "jacobian.h"
#include "matrixR33.h"
#include "node.h"
#include "tree.h"
#include "vectorR3.h"

/*
 *  iMobot dimension macros
 */
#define CENTER_LENGTH       0.07303
#define CENTER_WIDTH        0.02540
#define CENTER_HEIGHT       0.06909
#define CENTER_RADIUS       0.03554
#define BODY_LENGTH         0.03785
#define BODY_WIDTH          0.07239
#define BODY_HEIGHT         0.07239
#define BODY_RADIUS         0.03620
#define BODY_INNER_WIDTH    0.02287
#define BODY_END_DEPTH      0.01994
#define BODY_MOUNT_CENTER   0.03792
#define END_WIDTH           0.07239
#define END_HEIGHT          0.07239
#define END_DEPTH           0.00476
#define END_RADIUS          0.01778
#define NUM_DOF 4

enum anchor_point_e {
	LE,
	RE
};

class CiMobotIK {
	public:
		CiMobotIK(int num_bot, int num_targets);
		~CiMobotIK(void);

		void iMobotAnchor(int end, double x, double y, double z, double psi, double theta, double phi, double r_le, double r_lb, double r_rb, double r_re);
		void iMobotAttach(int bot_num, int att_num, int face1, int face2, double r_le, double r_lb, double r_rb, double r_re);
		void addEffector(int eff_num, int bot_num, int face);

		void setCurrentMode(int mode);			// Type of updating mode for Jacobian
		void setCurrentType(int type);			// Jacobian type: END or TARGET
		void setCurrentDLSMode(int mode);		// CLAMPED or TRADITIONAL
		void setDampingDLS(double lambda);		// DLS damping
		void setTarget(int num, double x, double y, double z, double psi, double theta, double phi);
		void setTargetPosition(int num, double x, double y, double z);
        void setTargetRotation(int num, double psi, double theta, double phi);

		int getCurrentMode(void);				// Type of updating mode for Jacobian
		int getCurrentType(void);				// Jacobian type: END or TARGET
		int getCurrentDLSMode(void);			// CLAMPED or TRADITIONAL
		void getEffectorPosition(int num, double &x, double &y, double &z);
		double getEffectorX(int num);
		double getEffectorY(int num);
		double getEffectorZ(int num);
		void getEffectorRotation(int num, double &phi, double &theta, double &psi);
		double getEffectorPhi(int num);
		double getEffectorTheta(int num);
		double getEffectorPsi(int num);
		void getTargetPosition(int num, double &x, double &y, double &z);
		double getTargetX(int num);
		double getTargetY(int num);
		double getTargetZ(int num);
        void getTargetRotation(int num, double &psi, double &theta, double &phi);
		double getTargetPhi(int num);
		double getTargetTheta(int num);
		double getTargetPsi(int num);

		void runSimulation(int argc, char **argv);
	private:
		Jacobian *jacob;
		Node **node;
		Node **node_right;
		Node **node_effector;
		Tree tree;
		VectorR3 *target_pos;
		MatrixR33 *target_rot;

		bool *m_del_theta;
		int m_t_count;
		int m_num_bot;
		int m_num_targets;
		double m_t;
		double m_t_step;
        int m_j_mode;
        int m_j_type;
        int m_j_dls;
        double m_j_lambda;

		void update_targets(void);
		void print_intermediate_data(void);
		void set_flags(void);
		void increment_step(void);
		bool end_simulation(void);
		bool is_true(bool *a, int length);
		inline double D2R(double deg);
		inline double R2D(double rad);
};

#endif	/* CIMOBOTIK_H_ */