#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#include <Shlwapi.h>
#endif
#include <gtk/gtk.h>
#include <tinyxml2.h>

GtkBuilder *g_builder; 
GtkWidget *g_window;
tinyxml2::XMLDocument g_doc;
FILE *fp1 = NULL;
char g_xml[512], g_chrc[512], fpbuf1[10240], fpbuf2[5120];

#ifdef __cplusplus
extern "C" {
#endif

G_MODULE_EXPORT void on_window_destroy(GtkWidget* widget, gpointer data) {
	g_doc.SaveFile(g_xml);
    gtk_main_quit();
}

G_MODULE_EXPORT void on_real_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "real")))) {
		fp1 = fopen(g_chrc, "w");
		fputs(fpbuf1, fp1);
#ifdef _WIN32
		fputs("// RoboSim Begin\n//_ipath = stradd(\"C:/Ch/package/chrobotsim/include;\", _ipath);\n// RoboSim End\n", fp1);
#else
		fputs("// RoboSim Begin\n//_ipath = stradd(\"/usr/local/ch/package/chrobotsim/include;\", _ipath);\n// RoboSim End\n", fp1);
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
		fputs("// RoboSim Begin\n_ipath = stradd(\"C:/Ch/package/chrobotsim/include;\", _ipath);\n// RoboSim End\n", fp1);
#else
		fputs("// RoboSim Begin\n_ipath = stradd(\"/usr/local/ch/package/chrobotsim/include;\", _ipath);\n// RoboSim End\n", fp1);
#endif
		fputs(fpbuf2, fp1);
		fclose(fp1);
	}
}

G_MODULE_EXPORT void on_mobot_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot")))) {
		tinyxml2::XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("null");
		while (node) {
			node->SetName("mobot");
			node = node->NextSiblingElement("null");
		}
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "wheeled"))) ||
				gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "twowheeled")))) {
			tinyxml2::XMLElement *wheel = g_doc.FirstChildElement("sim")->FirstChildElement("smallwheel");
			while (wheel) {
				if (wheel->IntAttribute("face") != 1)
					wheel->SetAttribute("face", 4);
				wheel = wheel->NextSiblingElement("smallwheel");
			}
		}
	}
	else {
		tinyxml2::XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("mobot");
		while (node) {
			node->SetName("null");
			node = node->NextSiblingElement("mobot");
		}
	}
}

G_MODULE_EXPORT void on_linkboti_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti")))) {
		tinyxml2::XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("null");
		while (node) {
			node->SetName("linkboti");
			node = node->NextSiblingElement("null");
		}
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "wheeled"))) ||
				gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "twowheeled")))) {
			tinyxml2::XMLElement *wheel = g_doc.FirstChildElement("sim")->FirstChildElement("smallwheel");
			while (wheel) {
				if (wheel->IntAttribute("face") != 1)
					wheel->SetAttribute("face", 3);
				wheel = wheel->NextSiblingElement("smallwheel");
			}
		}
	}
	else {
		tinyxml2::XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("linkboti");
		while (node) {
			node->SetName("null");
			node = node->NextSiblingElement("linkboti");
		}
	}
}

G_MODULE_EXPORT void on_linkbotl_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl")))) {
		tinyxml2::XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("null");
		while (node) {
			node->SetName("linkbotl");
			node = node->NextSiblingElement("null");
		}
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "wheeled"))) ||
				gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "twowheeled")))) {
			tinyxml2::XMLElement *wheel = g_doc.FirstChildElement("sim")->FirstChildElement("smallwheel");
			while (wheel) {
				if (wheel->IntAttribute("face") != 1)
					wheel->SetAttribute("face", 2);
				wheel = wheel->NextSiblingElement("smallwheel");
			}
		}
	}
	else {
		tinyxml2::XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("linkbotl");
		while (node) {
			node->SetName("null");
			node = node->NextSiblingElement("linkbotl");
		}
	}
}

G_MODULE_EXPORT void on_save_clicked(GtkWidget* widget, gpointer data) {
	g_doc.SaveFile(g_xml);
}

G_MODULE_EXPORT void on_one_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "one")))) {
		tinyxml2::XMLElement *robot, *ele = g_doc.FirstChildElement("sim");
		ele->DeleteChildren();
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot"))))
			robot = g_doc.NewElement("mobot");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti"))))
			robot = g_doc.NewElement("linkboti");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl"))))
			robot = g_doc.NewElement("linkbotl");
		robot->SetAttribute("id", 0);
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_x"))));
		pos->SetAttribute("y", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_y"))));
		pos->SetAttribute("z", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_z"))));
		ele->InsertFirstChild(robot);
	}
}

G_MODULE_EXPORT void on_two_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "two")))) {
		tinyxml2::XMLElement *robot, *robot2, *ele = g_doc.FirstChildElement("sim");
		ele->DeleteChildren();
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot"))))
			robot = g_doc.NewElement("mobot");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti"))))
			robot = g_doc.NewElement("linkboti");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl"))))
			robot = g_doc.NewElement("linkbotl");
		robot->SetAttribute("id", 0);
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_x"))));
		pos->SetAttribute("y", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_y"))));
		pos->SetAttribute("z", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_z"))));
		robot->InsertFirstChild(pos);
		ele->InsertFirstChild(robot);
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot"))))
			robot2 = g_doc.NewElement("mobot");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti"))))
			robot2 = g_doc.NewElement("linkboti");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl"))))
			robot2 = g_doc.NewElement("linkbotl");
		robot2->SetAttribute("id", 1);
		pos = g_doc.NewElement("position");
		pos->SetAttribute("x", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_x"))));
		pos->SetAttribute("y", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_y"))));
		pos->SetAttribute("z", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_z"))));
		robot2->InsertFirstChild(pos);
		ele->InsertAfterChild(robot, robot2);
	}
}

tinyxml2::XMLElement* newSmallwheel(int num, int face) {
	tinyxml2::XMLElement *wheel = g_doc.NewElement("smallwheel");
	wheel->SetAttribute("robot", num);
	wheel->SetAttribute("face", face);
	return wheel;
}

G_MODULE_EXPORT void on_wheeled_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "wheeled")))) {
		tinyxml2::XMLElement *robot, *wheel1, *wheel2, *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();
		wheel1 = newSmallwheel(0, 1);
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot")))) {
			robot = g_doc.NewElement("mobot");
			wheel2 = newSmallwheel(0, 4);
		}
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti")))) {
			robot = g_doc.NewElement("linkboti");
			wheel2 = newSmallwheel(0, 3);
		}
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl")))) {
			robot = g_doc.NewElement("linkbotl");
			wheel2 = newSmallwheel(0, 2);
		}
		robot->SetAttribute("id", 0);
		sim->InsertFirstChild(robot);
		sim->InsertAfterChild(robot, wheel1);
		sim->InsertAfterChild(wheel1, wheel2);
	}
}

G_MODULE_EXPORT void on_twowheeled_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "twowheeled")))) {
		tinyxml2::XMLElement *robot1, *robot2, *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();
		tinyxml2::XMLElement *wheel1 = g_doc.NewElement("smallwheel");
		wheel1->SetAttribute("robot", 0);
		tinyxml2::XMLElement *wheel2 = g_doc.NewElement("smallwheel");
		wheel2->SetAttribute("robot", 0);
		tinyxml2::XMLElement *wheel3 = g_doc.NewElement("smallwheel");
		wheel3->SetAttribute("robot", 1);
		tinyxml2::XMLElement *wheel4 = g_doc.NewElement("smallwheel");
		wheel4->SetAttribute("robot", 1);
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot")))) {
			robot1 = g_doc.NewElement("mobot");
			robot2 = g_doc.NewElement("mobot");
			wheel1->SetAttribute("face", 1);
			wheel2->SetAttribute("face", 4);
			wheel3->SetAttribute("face", 1);
			wheel4->SetAttribute("face", 4);
		}
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti")))) {
			robot1 = g_doc.NewElement("linkboti");
			robot2 = g_doc.NewElement("linkboti");
			wheel1->SetAttribute("face", 1);
			wheel2->SetAttribute("face", 3);
			wheel3->SetAttribute("face", 1);
			wheel4->SetAttribute("face", 3);
		}
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl")))) {
			robot1 = g_doc.NewElement("linkbotl");
			robot2 = g_doc.NewElement("linkbotl");
			wheel1->SetAttribute("face", 1);
			wheel2->SetAttribute("face", 2);
			wheel3->SetAttribute("face", 1);
			wheel4->SetAttribute("face", 2);
		}
		robot1->SetAttribute("id", 0);
		robot2->SetAttribute("id", 1);
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", 0.5);
		robot2->InsertFirstChild(pos);
		sim->InsertFirstChild(robot1);
		sim->InsertAfterChild(robot1, robot2);
		sim->InsertAfterChild(robot2, wheel1);
		sim->InsertAfterChild(wheel1, wheel2);
		sim->InsertAfterChild(wheel2, wheel3);
		sim->InsertAfterChild(wheel3, wheel4);
	}
}

G_MODULE_EXPORT void on_one_x_value_changed(GtkWidget* widget, gpointer data) {
	tinyxml2::XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
	while (robot && robot->IntAttribute("id") != 0)
		robot = robot->NextSiblingElement();
	if (robot && !(pos = robot->FirstChildElement("position"))) {
		pos = g_doc.NewElement("position");
		pos->SetAttribute("x", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_x"))));
		robot->InsertFirstChild(pos);
	}
	else if (robot)
		pos->SetAttribute("x", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_x"))));
}

G_MODULE_EXPORT void on_one_y_value_changed(GtkWidget* widget, gpointer data) {
	tinyxml2::XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
	while (robot && robot->IntAttribute("id") != 0)
		robot = robot->NextSiblingElement();
	if (robot && !(pos = robot->FirstChildElement("position"))) {
		pos = g_doc.NewElement("position");
		pos->SetAttribute("y", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_y"))));
		robot->InsertFirstChild(pos);
	}
	else if (robot)
		pos->SetAttribute("y", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_y"))));
}

G_MODULE_EXPORT void on_one_z_value_changed(GtkWidget* widget, gpointer data) {
	tinyxml2::XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
	while (robot && robot->IntAttribute("id") != 0)
		robot = robot->NextSiblingElement();
	if (robot && !(pos = robot->FirstChildElement("position"))) {
		pos = g_doc.NewElement("position");
		pos->SetAttribute("z", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_z"))));
		robot->InsertFirstChild(pos);
	}
	else if (robot)
		pos->SetAttribute("z", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_z"))));
}

G_MODULE_EXPORT void on_two_x_value_changed(GtkWidget* widget, gpointer data) {
	tinyxml2::XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
	while (robot && robot->IntAttribute("id") != 1)
		robot = robot->NextSiblingElement();
	if (robot && !(pos = robot->FirstChildElement("position"))) {
		pos = g_doc.NewElement("position");
		pos->SetAttribute("x", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_x"))));
		robot->InsertFirstChild(pos);
	}
	else if (robot)
		pos->SetAttribute("x", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_x"))));
}

G_MODULE_EXPORT void on_two_y_value_changed(GtkWidget* widget, gpointer data) {
	tinyxml2::XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
	while (robot && robot->IntAttribute("id") != 1)
		robot = robot->NextSiblingElement();
	if (robot && !(pos = robot->FirstChildElement("position"))) {
		pos = g_doc.NewElement("position");
		pos->SetAttribute("y", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_y"))));
		robot->InsertFirstChild(pos);
	}
	else if (robot)
		pos->SetAttribute("y", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_y"))));
}

G_MODULE_EXPORT void on_two_z_value_changed(GtkWidget* widget, gpointer data) {
	tinyxml2::XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
	while (robot && robot->IntAttribute("id") != 1)
		robot = robot->NextSiblingElement();
	if (robot && !(pos = robot->FirstChildElement("position"))) {
		pos = g_doc.NewElement("position");
		pos->SetAttribute("z", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_z"))));
		robot->InsertFirstChild(pos);
	}
	else if (robot)
		pos->SetAttribute("z", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_z"))));
}

#ifdef __cplusplus
}
#endif

int main (int argc, char *argv[]) {
    gtk_init(&argc, &argv);

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
	tinyxml2::XMLElement *ele = g_doc.NewElement("sim");
	g_doc.InsertAfterChild(dec, ele);
	// first robot - default is mobot
	tinyxml2::XMLElement *robot = g_doc.NewElement("mobot");
	robot->SetAttribute("id", 0);
	ele->InsertFirstChild(robot);

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

	// write config file for hardware robots
	fp1 = fopen(g_chrc, "w");
	fputs(fpbuf1, fp1);
#ifdef _WIN32
		fputs("// RoboSim Begin\n//_ipath = stradd(\"C:/Ch/package/chrobotsim/include;\", _ipath);\n// RoboSim End\n", fp1);
#else
		fputs("// RoboSim Begin\n//_ipath = stradd(\"/usr/local/ch/package/chrobotsim/include;\", _ipath);\n// RoboSim End\n", fp1);
#endif
	fputs(fpbuf2, fp1);
	fclose(fp1);

	// show main window
	gtk_widget_show(g_window);                
	gtk_main();

	return 0;
}
