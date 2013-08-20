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

G_MODULE_EXPORT void on_save_clicked(GtkWidget* widget, gpointer data) {
	tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
	sim->DeleteChildren();
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
		//pos->SetAttribute("z", I2M(gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "first_spin_z")))));
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
