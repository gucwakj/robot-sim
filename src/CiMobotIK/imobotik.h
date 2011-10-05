#ifndef CIMOBOTIK_H_
#define CIMOBOTIK_H_

#include "config.h"
#include "jacobian.h"
#include "linearR3.h"
#include "node.h"
#include "tree.h"
#ifdef ENABLE_GRAPHICS
#include <GL/glui.h>
#define ROTATE = 0;
#define SCALE = 1;
#endif

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

class CiMobotIK {
	public:
		CiMobotIK(int num_bot, int num_targets);
		~CiMobotIK(void);

		void iMobotAnchor(double x, double y, double z, double r_le, double r_lb, double r_rb, double r_re);
		void iMobotAttach(int bot_num, int att_num, int face1, int face2, double r_le, double r_lb, double r_rb, double r_re);
		void addEffector(int eff_num, int bot_num);

		void setCurrentMode(int mode);			// Type of updating mode for Jacobian
		void setCurrentType(int type);			// Jacobian type: END or TARGET
		void setCurrentDLSMode(int mode);		// CLAMPED or TRADITIONAL
		void setDampingDLS(double lambda);		// DLS damping
		void setTarget(int num, double x, double y, double z);

		int getCurrentMode(void);				// Type of updating mode for Jacobian
		int getCurrentType(void);				// Jacobian type: END or TARGET
		int getCurrentDLSMode(void);			// CLAMPED or TRADITIONAL
		void getEffector(int num, double &x, double &y, double &z);
		double getEffectorX(int num);
		double getEffectorY(int num);
		double getEffectorZ(int num);
		void getTarget(int num, double &x, double &y, double &z);
		double getTargetX(int num);
		double getTargetY(int num);
		double getTargetZ(int num);

		void runSimulation(int argc, char **argv);
	private:
		Jacobian *jacob;
		Node **node;
		Tree tree;
		VectorR3 *target;

		double m_t;
		double m_t_step;
		int m_t_count;
		int m_num_bot;
		int m_num_targets;

		void update_targets(void);
		void print_intermediate_data(void);
		void increment_step(void);
		bool end_simulation(void);
		inline double D2R(double deg);
		inline double R2D(double rad);

		#ifdef ENABLE_GRAPHICS
		float ANGFACT;
		float SCLFACT;
		float MINSCALE;
		int LEFT;
		int MIDDLE;
		int RIGHT;
		int ORTHO;
		int PERSP;
		int ActiveButton;
		float BACKCOLOR[4];
		float AXES_COLOR[3];
		float AXES_WIDTH;
		GLUI *Glui;			/* instance of glui window		*/
		int	GluiWindow;		/* the glut id for the glui window	*/
		int	GrWindow;		/* window id for graphics window	*/
		int	LeftButton;		/* either ROTATE or SCALE		*/
		float	RotMatrix[4][4];	/* set by glui rotation widget		*/
		float	Scale, Scale2;		/* scaling factors			*/
		int	WhichProjection;	/* ORTHO or PERSP			*/
		int	Xmouse, Ymouse;		/* mouse values				*/
		float	Xrot, Yrot;		/* rotation angles in degrees		*/
		float	TransXYZ[3];		/* set by glui translation widgets	*/

		void Animate(void);
		void Display(void);
		void InitGraphics(void);
		void ResetGraphics(void);
		void MouseMotion(int x, int y);
		void MouseButton(int button, int state, int x, int y);
		#endif
};

#endif	/* CIMOBOTIK_H_ */