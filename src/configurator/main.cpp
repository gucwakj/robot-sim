#include <stdlib.h>
#include <gtk/gtk.h>
#include "tinyxml2.h"

using namespace tinyxml2;

//bool g_output = 0;
GtkBuilder *g_builder; 
GtkWidget *g_window;
XMLDocument g_doc;
char g_path[512];

#ifdef __cplusplus
extern "C" {
#endif

G_MODULE_EXPORT void on_window_destroy(GtkWidget* widget, gpointer data) {
	g_doc.SaveFile(g_path);
    gtk_main_quit();
}

G_MODULE_EXPORT void on_real_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "real")))) {
		//g_output = 0;
		//printf("output: %s\n", (g_output ? "simulated" : "real"));
	}
}

G_MODULE_EXPORT void on_simulated_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "simulated")))) {
		//g_output = 1;
		//printf("output: %s\n", (g_output ? "simulated" : "real"));
	}
}

G_MODULE_EXPORT void on_mobot_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot")))) {
		XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("null");
		while (node) {
			node->SetName("mobot");
			node = node->NextSiblingElement("null");
		}
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "wheeled"))) ||
				gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "twowheeled")))) {
			XMLElement *wheel = g_doc.FirstChildElement("sim")->FirstChildElement("smallwheel");
			while (wheel) {
				if (wheel->IntAttribute("face") != 1)
					wheel->SetAttribute("face", 4);
				wheel = wheel->NextSiblingElement("smallwheel");
			}
		}
	}
	else {
		XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("mobot");
		while (node) {
			node->SetName("null");
			node = node->NextSiblingElement("mobot");
		}
	}
}

G_MODULE_EXPORT void on_linkboti_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti")))) {
		XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("null");
		while (node) {
			node->SetName("linkboti");
			node = node->NextSiblingElement("null");
		}
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "wheeled"))) ||
				gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "twowheeled")))) {
			XMLElement *wheel = g_doc.FirstChildElement("sim")->FirstChildElement("smallwheel");
			while (wheel) {
				if (wheel->IntAttribute("face") != 1)
					wheel->SetAttribute("face", 3);
				wheel = wheel->NextSiblingElement("smallwheel");
			}
		}
	}
	else {
		XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("linkboti");
		while (node) {
			node->SetName("null");
			node = node->NextSiblingElement("linkboti");
		}
	}
}

G_MODULE_EXPORT void on_linkbotl_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl")))) {
		XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("null");
		while (node) {
			node->SetName("linkbotl");
			node = node->NextSiblingElement("null");
		}
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "wheeled"))) ||
				gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "twowheeled")))) {
			XMLElement *wheel = g_doc.FirstChildElement("sim")->FirstChildElement("smallwheel");
			while (wheel) {
				if (wheel->IntAttribute("face") != 1)
					wheel->SetAttribute("face", 2);
				wheel = wheel->NextSiblingElement("smallwheel");
			}
		}
	}
	else {
		XMLElement *node = g_doc.FirstChildElement("sim")->FirstChildElement("linkbotl");
		while (node) {
			node->SetName("null");
			node = node->NextSiblingElement("linkbotl");
		}
	}
}

G_MODULE_EXPORT void on_save_clicked(GtkWidget* widget, gpointer data) {
	g_doc.SaveFile(g_path);
}

G_MODULE_EXPORT void on_one_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "one")))) {
		XMLElement *robot, *ele = g_doc.FirstChildElement("sim");
		ele->DeleteChildren();
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot"))))
			robot = g_doc.NewElement("mobot");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti"))))
			robot = g_doc.NewElement("linkboti");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl"))))
			robot = g_doc.NewElement("linkbotl");
		robot->SetAttribute("id", 0);
		XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_x"))));
		pos->SetAttribute("y", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_y"))));
		pos->SetAttribute("z", gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_z"))));
		ele->InsertFirstChild(robot);
	}
}

G_MODULE_EXPORT void on_two_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "two")))) {
		XMLElement *robot, *robot2, *ele = g_doc.FirstChildElement("sim");
		ele->DeleteChildren();
		if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "mobot"))))
			robot = g_doc.NewElement("mobot");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkboti"))))
			robot = g_doc.NewElement("linkboti");
		else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "linkbotl"))))
			robot = g_doc.NewElement("linkbotl");
		robot->SetAttribute("id", 0);
		XMLElement *pos = g_doc.NewElement("position");
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

XMLElement* newSmallwheel(int num, int face) {
	XMLElement *wheel = g_doc.NewElement("smallwheel");
	wheel->SetAttribute("robot", num);
	wheel->SetAttribute("face", face);
	return wheel;
}

G_MODULE_EXPORT void on_wheeled_toggled(GtkWidget* widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "wheeled")))) {
		XMLElement *robot, *wheel1, *wheel2, *sim = g_doc.FirstChildElement("sim");
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
		XMLElement *robot1, *robot2, *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();
		XMLElement *wheel1 = g_doc.NewElement("smallwheel");
		wheel1->SetAttribute("robot", 0);
		XMLElement *wheel2 = g_doc.NewElement("smallwheel");
		wheel2->SetAttribute("robot", 0);
		XMLElement *wheel3 = g_doc.NewElement("smallwheel");
		wheel3->SetAttribute("robot", 1);
		XMLElement *wheel4 = g_doc.NewElement("smallwheel");
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
		XMLElement *pos = g_doc.NewElement("position");
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
	XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
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
	XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
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
	XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
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
	XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
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
	XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
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
	XMLElement *pos, *robot = g_doc.FirstChildElement("sim")->FirstChildElement();
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
    gtk_builder_add_from_file(g_builder, "interface.glade", NULL);
    g_window = GTK_WIDGET(gtk_builder_get_object(g_builder, "window1"));
	if (g_window == NULL) { fprintf(stderr, "Unable to file object with id \"window1\" \n"); exit(1); }
    gtk_builder_connect_signals(g_builder, NULL);

	// set declaration
	XMLDeclaration *dec = g_doc.NewDeclaration();
	g_doc.InsertFirstChild(dec);
	// root sim node
	XMLElement *ele = g_doc.NewElement("sim");
	g_doc.InsertAfterChild(dec, ele);
	// first robot - default is mobot
	XMLElement *robot = g_doc.NewElement("mobot");
	robot->SetAttribute("id", 0);
	ele->InsertFirstChild(robot);

	GtkAdjustment *adjustment = gtk_adjustment_new(0.0, -50.0, 50.0, 0.1, 0.1, 0.0);
	GtkSpinButton *spin = GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_x"));
	gtk_spin_button_set_adjustment(spin, adjustment);
	adjustment = gtk_adjustment_new(0.0, -50.0, 50.0, 0.1, 0.1, 0.0);
	spin = GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_y"));
	gtk_spin_button_set_adjustment(spin, adjustment);
	adjustment = gtk_adjustment_new(0.0, 0.0, 50.0, 0.1, 0.1, 0.0);
	spin = GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "one_z"));
	gtk_spin_button_set_adjustment(spin, adjustment);
	adjustment = gtk_adjustment_new(0.5, -50.0, 50.0, 0.1, 0.1, 0.0);
	spin = GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_x"));
	gtk_spin_button_set_adjustment(spin, adjustment);
	adjustment = gtk_adjustment_new(0.0, -50.0, 50.0, 0.1, 0.1, 0.0);
	spin = GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_y"));
	gtk_spin_button_set_adjustment(spin, adjustment);
	adjustment = gtk_adjustment_new(0.0, 0.0, 50.0, 0.1, 0.1, 0.0);
	spin = GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "two_z"));
	gtk_spin_button_set_adjustment(spin, adjustment);

	// store config file
#ifdef _WIN32
	SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, g_path);
	strcat(g_path, "\\robosimrc");
#else
	strcpy(g_path, getenv("HOME"));
	strcat(g_path, "/.robosimrc");
#endif
	g_doc.SaveFile(g_path);
	
    gtk_widget_show(g_window);                
    gtk_main();

    return 0;
}

