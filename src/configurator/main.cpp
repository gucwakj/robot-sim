#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#include <Shlwapi.h>
#endif
#include <gtk/gtk.h>
#include <tinyxml2.h>

#define I2M(x) ((x)/39.370)

GtkBuilder *g_builder; 
GtkWidget *g_window;
tinyxml2::XMLDocument g_doc;
FILE *fp1 = NULL;
char g_xml[512], g_chrc[512], fpbuf1[10240], fpbuf2[5120];

#ifdef __cplusplus
extern "C" {
#endif

G_MODULE_EXPORT void on_window_destroy(GtkWidget* widget, gpointer data) {
	// save configuration file
	g_doc.SaveFile(g_xml);

	// write config file for hardware robots
	fp1 = fopen(g_chrc, "w");
	fputs(fpbuf1, fp1);
#ifdef _WIN32
		fputs("// RoboSim Begin\n//_ipath = stradd(\"C:/Ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp1);
#else
		fputs("// RoboSim Begin\n//_ipath = stradd(\"/usr/local/ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp1);
#endif
	fputs(fpbuf2, fp1);
	fclose(fp1);

	// quit
    gtk_main_quit();
}

G_MODULE_EXPORT void on_real_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "real")))) {
		fp1 = fopen(g_chrc, "w");
		fputs(fpbuf1, fp1);
#ifdef _WIN32
		fputs("// RoboSim Begin\n//_ipath = stradd(\"C:/Ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp1);
#else
		fputs("// RoboSim Begin\n//_ipath = stradd(\"/usr/local/ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp1);
#endif
		fputs(fpbuf2, fp1);
		fclose(fp1);
	}
}

G_MODULE_EXPORT void on_simulated_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "simulated")))) {
		fp1 = fopen(g_chrc, "w");
		fputs(fpbuf1, fp1);
#ifdef _WIN32
		fputs("// RoboSim Begin\n_ipath = stradd(\"C:/Ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp1);
#else
		fputs("// RoboSim Begin\n_ipath = stradd(\"/usr/local/ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp1);
#endif
		fputs(fpbuf2, fp1);
		fclose(fp1);
	}
}

G_MODULE_EXPORT void on_first_robot_clicked(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")))) {
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")), false);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")), true);
	}
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "none")), true);
}

G_MODULE_EXPORT void on_first_robot_toggled(GtkWidget* widget, gpointer data) {
	if (!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")))) {
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")), false);
	}
}

G_MODULE_EXPORT void on_second_robot_clicked(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")))) {
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")), false);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")), true);
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")), false);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")), true);
	}
}

G_MODULE_EXPORT void on_second_robot_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")))) {
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")), true);
	}
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "none")), true);
}

G_MODULE_EXPORT void on_explorer_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")))) {
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")), true);
	}
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")))) {
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")), true);
	}
}

G_MODULE_EXPORT void on_snake_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")))) {
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")), true);
	}
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")))) {
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")), true);
	}
}

G_MODULE_EXPORT void on_stand_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")))) {
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")), true);
	}
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")))) {
		gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")), true);
	}
}

G_MODULE_EXPORT void on_save_clicked(GtkWidget* widget, gpointer data) {
	// clean out sim node
	tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
	sim->DeleteChildren();

	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")))) {
		tinyxml2::XMLElement *robot1, *robot2, *robot3, *robot4, *robot5;

		// create first robot
		robot1 = g_doc.NewElement("linkboti");
		// set id
		robot1->SetAttribute("id", 0);
		// set position
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", 0);
		pos->SetAttribute("y", 0);
		pos->SetAttribute("z", 0);
		robot1->InsertFirstChild(pos);
		// set rotation
		tinyxml2::XMLElement *rot = g_doc.NewElement("rotation");
		rot->SetAttribute("psi", 0);
		rot->SetAttribute("theta", 0);
		rot->SetAttribute("phi", 0);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkboti");
		robot2->SetAttribute("id", 1);
		robot3 = g_doc.NewElement("linkboti");
		robot3->SetAttribute("id", 2);
		robot4 = g_doc.NewElement("linkboti");
		robot4->SetAttribute("id", 3);
		robot5 = g_doc.NewElement("linkbotl");
		robot5->SetAttribute("id", 4);
		sim->InsertAfterChild(robot1, robot2);
		sim->InsertAfterChild(robot2, robot3);
		sim->InsertAfterChild(robot3, robot4);
		sim->InsertAfterChild(robot4, robot5);

		// add wheels
		tinyxml2::XMLElement *wheel1 = g_doc.NewElement("smallwheel"),
							 *wheel2 = g_doc.NewElement("smallwheel");
		wheel1->SetAttribute("robot", 0);
		wheel1->SetAttribute("face", 3);
		wheel2->SetAttribute("robot", 1);
		wheel2->SetAttribute("face", 1);
		sim->InsertAfterChild(robot5, wheel1);
		sim->InsertAfterChild(wheel1, wheel2);

		// insert cube
		tinyxml2::XMLElement *cube = g_doc.NewElement("cube");
		sim->InsertAfterChild(wheel2, cube);
		tinyxml2::XMLElement *cside1 = g_doc.NewElement("side");
		cside1->SetAttribute("id", 1);
		cside1->SetAttribute("robot", 0);
		cside1->SetAttribute("face", 1);
		cube->InsertFirstChild(cside1);
		tinyxml2::XMLElement *cside2 = g_doc.NewElement("side");
		cside2->SetAttribute("id", 2);
		cside2->SetAttribute("robot", 0);
		cside2->SetAttribute("conn", 2);
		cube->InsertAfterChild(cside1, cside2);
		tinyxml2::XMLElement *cside3 = g_doc.NewElement("side");
		cside3->SetAttribute("id", 3);
		cside3->SetAttribute("robot", 1);
		cside3->SetAttribute("face", 3);
		cube->InsertAfterChild(cside2, cside3);
		tinyxml2::XMLElement *cside5 = g_doc.NewElement("side");
		cside5->SetAttribute("id", 5);
		cside5->SetAttribute("robot", 2);
		cside5->SetAttribute("face", 2);
		cube->InsertAfterChild(cside3, cside5);

		// insert bridge 1
		tinyxml2::XMLElement *bridge1 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(cube, bridge1);
		tinyxml2::XMLElement *b1side1 = g_doc.NewElement("side");
		b1side1->SetAttribute("id", 1);
		b1side1->SetAttribute("robot", 2);
		b1side1->SetAttribute("face", 3);
		bridge1->InsertFirstChild(b1side1);
		tinyxml2::XMLElement *b1side2 = g_doc.NewElement("side");
		b1side2->SetAttribute("id", 2);
		b1side2->SetAttribute("robot", 3);
		b1side2->SetAttribute("face", 3);
		bridge1->InsertAfterChild(b1side1, b1side2);

		// insert bridge 2
		tinyxml2::XMLElement *bridge2 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge1, bridge2);
		tinyxml2::XMLElement *b2side1 = g_doc.NewElement("side");
		b2side1->SetAttribute("id", 1);
		b2side1->SetAttribute("robot", 2);
		b2side1->SetAttribute("face", 1);
		bridge2->InsertFirstChild(b2side1);
		tinyxml2::XMLElement *b2side2 = g_doc.NewElement("side");
		b2side2->SetAttribute("id", 2);
		b2side2->SetAttribute("robot", 3);
		b2side2->SetAttribute("face", 1);
		bridge2->InsertAfterChild(b2side1, b2side2);

		// insert simple
		tinyxml2::XMLElement *simple = g_doc.NewElement("simple");
		sim->InsertAfterChild(bridge2, simple);
		tinyxml2::XMLElement *sside1 = g_doc.NewElement("side");
		sside1->SetAttribute("id", 1);
		sside1->SetAttribute("robot", 3);
		sside1->SetAttribute("face", 2);
		simple->InsertFirstChild(sside1);
		tinyxml2::XMLElement *sside2 = g_doc.NewElement("side");
		sside2->SetAttribute("id", 2);
		sside2->SetAttribute("robot", 4);
		sside2->SetAttribute("face", 2);
		simple->InsertAfterChild(sside1, sside2);

		// insert gripper
		tinyxml2::XMLElement *gripper = g_doc.NewElement("gripper");
		sim->InsertAfterChild(simple, gripper);
		gripper->SetAttribute("robot", 4);
	}
	else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")))) {
		tinyxml2::XMLElement *robot1, *robot2, *robot3, *robot4, *robot5;

		// create first robot
		robot1 = g_doc.NewElement("linkboti");
		// set id
		robot1->SetAttribute("id", 0);
		// set position
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", 0);
		pos->SetAttribute("y", 0);
		pos->SetAttribute("z", 0);
		robot1->InsertFirstChild(pos);
		// set rotation
		tinyxml2::XMLElement *rot = g_doc.NewElement("rotation");
		rot->SetAttribute("psi", 0);
		rot->SetAttribute("theta", 0);
		rot->SetAttribute("phi", 0);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkboti");
		robot2->SetAttribute("id", 1);
		robot3 = g_doc.NewElement("linkboti");
		robot3->SetAttribute("id", 2);
		robot4 = g_doc.NewElement("linkboti");
		robot4->SetAttribute("id", 3);
		robot5 = g_doc.NewElement("linkboti");
		robot5->SetAttribute("id", 4);
		sim->InsertAfterChild(robot1, robot2);
		sim->InsertAfterChild(robot2, robot3);
		sim->InsertAfterChild(robot3, robot4);
		sim->InsertAfterChild(robot4, robot5);

		// insert gripper
		tinyxml2::XMLElement *gripper = g_doc.NewElement("gripper");
		sim->InsertAfterChild(robot5, gripper);
		gripper->SetAttribute("robot", 0);

		// insert simple 1
		tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
		sim->InsertAfterChild(gripper, simple1);
		tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
		s1side1->SetAttribute("id", 1);
		s1side1->SetAttribute("robot", 0);
		s1side1->SetAttribute("face", 2);
		simple1->InsertFirstChild(s1side1);
		tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
		s1side2->SetAttribute("id", 2);
		s1side2->SetAttribute("robot", 1);
		s1side2->SetAttribute("face", 2);
		simple1->InsertAfterChild(s1side1, s1side2);

		// insert bridge 1
		tinyxml2::XMLElement *bridge1 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(simple1, bridge1);
		tinyxml2::XMLElement *b1side1 = g_doc.NewElement("side");
		b1side1->SetAttribute("id", 1);
		b1side1->SetAttribute("robot", 1);
		b1side1->SetAttribute("face", 3);
		bridge1->InsertFirstChild(b1side1);
		tinyxml2::XMLElement *b1side2 = g_doc.NewElement("side");
		b1side2->SetAttribute("id", 2);
		b1side2->SetAttribute("robot", 2);
		b1side2->SetAttribute("face", 3);
		bridge1->InsertAfterChild(b1side1, b1side2);

		// insert bridge 2
		tinyxml2::XMLElement *bridge2 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge1, bridge2);
		tinyxml2::XMLElement *b2side1 = g_doc.NewElement("side");
		b2side1->SetAttribute("id", 1);
		b2side1->SetAttribute("robot", 1);
		b2side1->SetAttribute("face", 1);
		bridge2->InsertFirstChild(b2side1);
		tinyxml2::XMLElement *b2side2 = g_doc.NewElement("side");
		b2side2->SetAttribute("id", 2);
		b2side2->SetAttribute("robot", 2);
		b2side2->SetAttribute("face", 1);
		bridge2->InsertAfterChild(b2side1, b2side2);

		// insert simple 2
		tinyxml2::XMLElement *simple2 = g_doc.NewElement("simple");
		sim->InsertAfterChild(bridge2, simple2);
		tinyxml2::XMLElement *s2side1 = g_doc.NewElement("side");
		s2side1->SetAttribute("id", 1);
		s2side1->SetAttribute("robot", 2);
		s2side1->SetAttribute("face", 2);
		simple2->InsertFirstChild(s2side1);
		tinyxml2::XMLElement *s2side2 = g_doc.NewElement("side");
		s2side2->SetAttribute("id", 2);
		s2side2->SetAttribute("robot", 3);
		s2side2->SetAttribute("face", 2);
		simple2->InsertAfterChild(s2side1, s2side2);

		// insert bridge 3
		tinyxml2::XMLElement *bridge3 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(simple2, bridge3);
		tinyxml2::XMLElement *b3side1 = g_doc.NewElement("side");
		b3side1->SetAttribute("id", 1);
		b3side1->SetAttribute("robot", 3);
		b3side1->SetAttribute("face", 3);
		bridge3->InsertFirstChild(b3side1);
		tinyxml2::XMLElement *b3side2 = g_doc.NewElement("side");
		b3side2->SetAttribute("id", 2);
		b3side2->SetAttribute("robot", 4);
		b3side2->SetAttribute("face", 3);
		bridge3->InsertAfterChild(b3side1, b3side2);

		// insert bridge 4
		tinyxml2::XMLElement *bridge4 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge3, bridge4);
		tinyxml2::XMLElement *b4side1 = g_doc.NewElement("side");
		b4side1->SetAttribute("id", 1);
		b4side1->SetAttribute("robot", 3);
		b4side1->SetAttribute("face", 1);
		bridge4->InsertFirstChild(b4side1);
		tinyxml2::XMLElement *b4side2 = g_doc.NewElement("side");
		b4side2->SetAttribute("id", 2);
		b4side2->SetAttribute("robot", 4);
		b4side2->SetAttribute("face", 1);
		bridge4->InsertAfterChild(b4side1, b4side2);

		// insert caster
		tinyxml2::XMLElement *caster = g_doc.NewElement("caster");
		sim->InsertAfterChild(bridge4, caster);
		caster->SetAttribute("robot", 4);
		caster->SetAttribute("face", 2);
	}
	else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")))) {
		tinyxml2::XMLElement *robot1, *robot2;

		// create first robot
		robot1 = g_doc.NewElement("linkbotl");
		// set id
		robot1->SetAttribute("id", 0);
		// set position
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", 0);
		pos->SetAttribute("y", 0);
		pos->SetAttribute("z", 0);
		robot1->InsertFirstChild(pos);
		// set rotation
		tinyxml2::XMLElement *rot = g_doc.NewElement("rotation");
		rot->SetAttribute("psi", 0);
		rot->SetAttribute("theta", 0);
		rot->SetAttribute("phi", 0);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkbotl");
		robot2->SetAttribute("id", 1);
		sim->InsertAfterChild(robot1, robot2);

		// insert bridge 1
		tinyxml2::XMLElement *bridge1 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(robot2, bridge1);
		tinyxml2::XMLElement *b1side1 = g_doc.NewElement("side");
		b1side1->SetAttribute("id", 1);
		b1side1->SetAttribute("robot", 0);
		b1side1->SetAttribute("face", 3);
		bridge1->InsertFirstChild(b1side1);
		tinyxml2::XMLElement *b1side2 = g_doc.NewElement("side");
		b1side2->SetAttribute("id", 2);
		b1side2->SetAttribute("robot", 1);
		b1side2->SetAttribute("face", 1);
		bridge1->InsertAfterChild(b1side1, b1side2);

		// insert faceplate 
		tinyxml2::XMLElement *faceplate1 = g_doc.NewElement("faceplate");
		sim->InsertAfterChild(bridge1, faceplate1);
		faceplate1->SetAttribute("robot", 0);
		faceplate1->SetAttribute("face", 2);

		// insert faceplate 
		tinyxml2::XMLElement *faceplate2 = g_doc.NewElement("faceplate");
		sim->InsertAfterChild(faceplate1, faceplate2);
		faceplate2->SetAttribute("robot", 1);
		faceplate2->SetAttribute("face", 2);
	}
	else {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_robot")))) {
#ifdef _WIN32
		const gchar *type = gtk_combo_box_get_active_text(GTK_COMBO_BOX(gtk_builder_get_object(g_builder, "combo_first_type")));
#else
		const gchar *type = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(gtk_builder_get_object(g_builder, "combo_first_type")));
#endif
		tinyxml2::XMLElement *robot1;
		if (!strcmp(type, "Linkbot L"))
			robot1 = g_doc.NewElement("linkbotl");
		else if (!strcmp(type, "Linkbot T"))
			robot1 = g_doc.NewElement("linkbott");
		else if (!strcmp(type, "Mobot"))
			robot1 = g_doc.NewElement("mobot");
		else
			robot1 = g_doc.NewElement("linkboti");
		sim->InsertFirstChild(robot1);

		// set id
		robot1->SetAttribute("id", 0);

		// set position
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", I2M(gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "first_spin_x")))));
		pos->SetAttribute("y", I2M(gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "first_spin_y")))));
		pos->SetAttribute("z", 0);
		robot1->InsertFirstChild(pos);

		// set rotation
		tinyxml2::XMLElement *rot = g_doc.NewElement("rotation");
		rot->SetAttribute("psi", 0);
		rot->SetAttribute("theta", 0);
		rot->SetAttribute("phi", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "spin_first_angle"))));
		robot1->InsertAfterChild(pos, rot);

		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "first_wheel_yes")))) {
			tinyxml2::XMLElement 	*wheel1 = g_doc.NewElement("smallwheel"), 
									*wheel2 = g_doc.NewElement("smallwheel"), 
									*caster = g_doc.NewElement("caster");
			wheel1->SetAttribute("robot", 0);
			wheel1->SetAttribute("face", 1);
			caster->SetAttribute("robot", 0);

			if ( !strcmp(robot1->Value(), "mobot") ) {
				wheel2->SetAttribute("robot", 0);
				wheel2->SetAttribute("face", 8);
				caster->SetAttribute("face", 3);
			}
			else if ( !strcmp(robot1->Value(), "linkboti") ) {
				wheel2->SetAttribute("robot", 0);
				wheel2->SetAttribute("face", 3);
				caster->SetAttribute("face", 2);
			}
			else if ( !strcmp(robot1->Value(), "linkbotl") ) {
				wheel2->SetAttribute("robot", 0);
				wheel2->SetAttribute("face", 2);
				caster->SetAttribute("face", 3);
			}
			else if ( !strcmp(robot1->Value(), "linkbott") ) {
				wheel2->SetAttribute("robot", 0);
				wheel2->SetAttribute("face", 3);
				caster->SetAttribute("face", 2);
			}
			sim->InsertAfterChild(robot1, wheel1);
			sim->InsertAfterChild(wheel1, wheel2);
			sim->InsertAfterChild(wheel2, caster);
		}
	}

	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_robot")))) {
		// set robot type
#ifdef _WIN32
		const gchar *type = gtk_combo_box_get_active_text(GTK_COMBO_BOX(gtk_builder_get_object(g_builder, "combo_second_type")));
#else
		const gchar *type = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(gtk_builder_get_object(g_builder, "combo_second_type")));
#endif
		tinyxml2::XMLElement *robot2;
		if (!strcmp(type, "Linkbot L"))
			robot2 = g_doc.NewElement("linkbotl");
		else if (!strcmp(type, "Linkbot T"))
			robot2 = g_doc.NewElement("linkbott");
		else if (!strcmp(type, "Mobot"))
			robot2 = g_doc.NewElement("mobot");
		else
			robot2 = g_doc.NewElement("linkboti");
		sim->InsertEndChild(robot2);

		// set id
		robot2->SetAttribute("id", 1);

		// set position
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", I2M(gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "second_spin_x")))));
		pos->SetAttribute("y", I2M(gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "second_spin_y")))));
		//pos->SetAttribute("z", I2M(gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "second_spin_z")))));
		pos->SetAttribute("z", 0);
		robot2->InsertFirstChild(pos);

		// set rotation
		tinyxml2::XMLElement *rot = g_doc.NewElement("rotation");
		rot->SetAttribute("psi", 0);
		rot->SetAttribute("theta", 0);
		rot->SetAttribute("phi", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "spin_second_angle"))));
		robot2->InsertAfterChild(pos, rot);

		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "second_wheel_yes")))) {
			tinyxml2::XMLElement 	*wheel1 = g_doc.NewElement("smallwheel"), 
									*wheel2 = g_doc.NewElement("smallwheel"), 
									*caster = g_doc.NewElement("caster");
			wheel1->SetAttribute("robot", 1);
			wheel1->SetAttribute("face", 1);
			caster->SetAttribute("robot", 1);

			if ( !strcmp(robot2->Value(), "mobot") ) {
				wheel2->SetAttribute("robot", 1);
				wheel2->SetAttribute("face", 8);
				caster->SetAttribute("face", 3);
			}
			else if ( !strcmp(robot2->Value(), "linkboti") ) {
				wheel2->SetAttribute("robot", 1);
				wheel2->SetAttribute("face", 3);
				caster->SetAttribute("face", 2);
			}
			else if ( !strcmp(robot2->Value(), "linkbotl") ) {
				wheel2->SetAttribute("robot", 1);
				wheel2->SetAttribute("face", 2);
				caster->SetAttribute("face", 3);
			}
			else if ( !strcmp(robot2->Value(), "linkbott") ) {
				wheel2->SetAttribute("robot", 1);
				wheel2->SetAttribute("face", 3);
				caster->SetAttribute("face", 2);
			}
			sim->InsertAfterChild(robot2, wheel1);
			sim->InsertAfterChild(wheel1, wheel2);
			sim->InsertAfterChild(wheel2, caster);
		}
	}
	}

	g_doc.SaveFile(g_xml);
}

#ifdef __cplusplus
}
#endif

int main (int argc, char *argv[]) {
	// init gtk
    gtk_init(&argc, &argv);

	// load gtk window
    g_builder = gtk_builder_new();
#ifdef _WIN32
    gtk_builder_add_from_file(g_builder, "interface2.glade", NULL);
#else
    gtk_builder_add_from_file(g_builder, "interface3.glade", NULL);
#endif
    g_window = GTK_WIDGET(gtk_builder_get_object(g_builder, "window1"));
	if (g_window == NULL) { fprintf(stderr, "Unable to file object with id \"window1\" \n"); exit(1); }
    gtk_builder_connect_signals(g_builder, NULL);

	// set declaration
	tinyxml2::XMLDeclaration *dec = g_doc.NewDeclaration();
	g_doc.InsertFirstChild(dec);

	// root sim node
	tinyxml2::XMLElement *sim = g_doc.NewElement("sim");
	g_doc.InsertAfterChild(dec, sim);
	tinyxml2::XMLElement	*robot1 = g_doc.NewElement("linkboti"),
							*wheel1 = g_doc.NewElement("smallwheel"), 
							*wheel2 = g_doc.NewElement("smallwheel"), 
							*caster = g_doc.NewElement("caster");
	robot1->SetAttribute("id", 0);
	wheel1->SetAttribute("robot", 0);
	wheel1->SetAttribute("face", 1);
	wheel2->SetAttribute("robot", 0);
	wheel2->SetAttribute("face", 3);
	caster->SetAttribute("robot", 0);
	caster->SetAttribute("face", 2);
	sim->InsertFirstChild(robot1);
	sim->InsertAfterChild(robot1, wheel1);
	sim->InsertAfterChild(wheel1, wheel2);
	sim->InsertAfterChild(wheel2, caster);

	// get config file paths
#ifdef _WIN32
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, g_xml))) {
		strcat(g_xml, "\\robosimrc");
	}
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, g_chrc))) {
		strcat(g_chrc, "\\_chrc");
	}
#else
	strcpy(g_xml, getenv("HOME"));
	strcat(g_xml, "/.robosimrc");
	strcpy(g_chrc, getenv("HOME"));
	strcat(g_chrc, "/.chrc");
#endif

	// set default robot type
	gtk_combo_box_set_active(GTK_COMBO_BOX(gtk_builder_get_object(g_builder, "combo_first_type")), 0);
	gtk_combo_box_set_active(GTK_COMBO_BOX(gtk_builder_get_object(g_builder, "combo_second_type")), 0);

	// save xml config file
	g_doc.SaveFile(g_xml);

	// if chrc exists, get file into buffer(s)
	if (fopen(g_chrc, "r") != NULL) {
		char line[1024];
		fp1 = fopen(g_chrc, "r");
		while (fgets(line, 1024, fp1)) {
			if (!strcmp(line, "// RoboSim Begin\n")) {
				fgets(line, 1024, fp1);		// ipath line
				fgets(line, 1024, fp1);		// robosim end line
				break;
			}
			strcat(fpbuf1, line);
		}
		while (fgets(line, 1024, fp1)) {
			strcat(fpbuf2, line);
		}
		fclose(fp1);
	}

	// write config file for simulated robots
	fp1 = fopen(g_chrc, "w");
	fputs(fpbuf1, fp1);
#ifdef _WIN32
		fputs("// RoboSim Begin\n_ipath = stradd(\"C:/Ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp1);
#else
		fputs("// RoboSim Begin\n_ipath = stradd(\"/usr/local/ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp1);
#endif
	fputs(fpbuf2, fp1);
	fclose(fp1);

	// show main window
	gtk_widget_show(g_window);                
	gtk_main();

	return 0;
}
