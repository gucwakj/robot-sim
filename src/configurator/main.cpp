#include <stdlib.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#include <Shlwapi.h>
#endif
#include <gtk/gtk.h>
#include <tinyxml2.h>
#include "../librobosim/macros.h"

#define XML_VERSION 2

typedef enum preconfig_e {
	BOW = 1,
	EXPLORER,
	FOURBOTDRIVE,
	FOURWHEELDRIVE,
	FOURWHEELEXPLORER,
	GROUPBOW,
	INCHWORM,
	LIFT,
	P_OMNIDRIVE,
	SNAKE,
	STAND
} preconfig_t;
typedef struct robots_s {
	int id;
	int type;
	double x, y, z;
	double psi, theta, phi;
	int wheel;
	double radius;
	struct robots_s *next;
} *robots_t;

GtkBuilder *g_builder;
GtkWidget *g_window;
robots_t g_robots = NULL;
tinyxml2::XMLDocument g_doc;
FILE *fp = NULL;
char g_xml[512] = "", g_chrc[512] = "", g_chhome[512] = "", *fpbuf;
int g_num = 0, g_type = 0, g_units = 1;
double g_grid[6][2] = {	12,		50,		// major
						1,		5,		// tics
						-48,	-200,	// minx
						48,		200,	// maxx
						-48,	-200,	// miny
						48,		200		// maxy
					};

#ifdef __cplusplus
extern "C" {
#endif

G_MODULE_EXPORT void on_window_destroy(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_aboutdialog_activate(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_aboutdialog_activate_link(GtkAboutDialog *label, gchar *uri, gpointer data);
G_MODULE_EXPORT void on_aboutdialog_close(GtkDialog *dialog, gpointer user_data);
G_MODULE_EXPORT void on_aboutdialog_response(GtkDialog *dialog, gint response_id, gpointer user_data);
G_MODULE_EXPORT void on_changelog_activate(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_menuitem_help_activate(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_notebook_switch_page(GtkWidget *widget, gpointer data);

G_MODULE_EXPORT void on_real_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_simulated_toggled(GtkWidget *widget, gpointer data);

G_MODULE_EXPORT void on_type_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_x_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_y_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_phi_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_wheeled_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_wheel_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_button_add_clicked(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_button_remove_clicked(GtkWidget* widget, gpointer data);

G_MODULE_EXPORT void on_us_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_metric_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_major_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_tics_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_minx_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_maxx_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_miny_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_maxy_value_changed(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_defaults_clicked(GtkWidget *widget, gpointer data);

G_MODULE_EXPORT void on_bow_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_explorer_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_fourbotdrive_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_fourwheeldrive_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_fourwheelexplorer_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_groupbow_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_inchworm_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_lift_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_omnidrive_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_snake_toggled(GtkWidget *widget, gpointer data);
G_MODULE_EXPORT void on_stand_toggled(GtkWidget *widget, gpointer data);

G_MODULE_EXPORT void on_tracking_toggled(GtkWidget *widget, gpointer data);

double convert(double value, int tometer);
void printRoboSimPath(void);
void readXMLConfig(void);
void refreshRobotList(void);
void saveRobotList(int force = 0);

#ifdef __cplusplus
}
#endif

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE hinst, HINSTANCE hinstPrev, LPSTR lpCmdLine, int nCmdShow) {
	// init gtk
	gtk_init(NULL, NULL);

	// check for multiple instances
	HANDLE hMutex;
	hMutex = CreateMutex(NULL, TRUE, TEXT("Global\\RoboSimMutex"));
	DWORD dwerror = GetLastError();
#else
int main(int argc, char *argv[]) {
	// init gtk
	gtk_init(&argc, &argv);
#endif
	// load builder
	g_builder = gtk_builder_new();

#ifdef _WIN32
	// another instance running
	if (dwerror == ERROR_ALREADY_EXISTS) {
		GtkWidget *d = gtk_message_dialog_new(
			GTK_WINDOW(gtk_builder_get_object(g_builder, "window1")),
			GTK_DIALOG_DESTROY_WITH_PARENT,
			GTK_MESSAGE_ERROR,
			GTK_BUTTONS_OK,
			"Another instance of RoboSim GUI is already running.  Please terminate the other process and try again.");
		int rc = gtk_dialog_run(GTK_DIALOG(d));
		exit(0);
	}

	// get Ch home path
	DWORD size;
	HKEY key;
#if defined(_WIN64)
	RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\Wow6432Node\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#else
	RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#endif
	char path[1024];
	RegQueryValueEx(key, TEXT("CHHOME"), NULL, NULL, (LPBYTE)path, &size);
	path[size] = '\0';
	if (path[0] == '\0') {
		strncpy(g_chhome, "C:/Ch", 5);
		path[5] = '\0';
	}
	else {
		strncpy(g_chhome, path, size);
	}
#endif

	// load gtk window
	gtk_builder_add_from_file(g_builder, "interface.glade", NULL);
	g_window = GTK_WIDGET(gtk_builder_get_object(g_builder, "window1"));
	if (g_window == NULL) {
		g_warning("Unable to find interface file.");
		exit(1);
	}
	gtk_builder_connect_signals(g_builder, NULL);

	// load linkbot coordinates picture
	GtkImage *image_l = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_linkbot"));
	GdkPixbuf *original_l = gdk_pixbuf_new_from_file("images/linkbot.jpg", NULL);
	GdkPixbuf *scaled_l = gdk_pixbuf_scale_simple(original_l, 264, 154, GDK_INTERP_HYPER);
	gtk_image_set_from_pixbuf(image_l, scaled_l);

	// load mobot coordinates picture
	GtkImage *image_m = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_mobot"));
	GdkPixbuf *original_m = gdk_pixbuf_new_from_file("images/mobot.jpg", NULL);
	GdkPixbuf *scaled_m = gdk_pixbuf_scale_simple(original_m, 264, 154, GDK_INTERP_HYPER);
	gtk_image_set_from_pixbuf(image_m, scaled_m);

	// get config file paths
#ifdef _WIN32
	// store robosimrc into local appdata path
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, g_xml))) {
		strcat(g_xml, "\\robosimrc");
	}
	// find _chrc file
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, g_chrc))) {
		strcat(g_chrc, "\\_chrc");
	}
	// try method 2 on failure
	if ((fp = fopen(g_chrc, "r+")) == NULL) {
		char *home = getenv("HOME");
		if (home != NULL) {
			strncpy(g_chrc, home, strlen(home));
			g_chrc[strlen(home)] = '\0';
			strcat(g_chrc, "\\_chrc");
		}
	}
	// copy default chrc on repeated failure
	if ((fp = fopen(g_chrc, "r+")) == NULL) {
		char path[1024];
		strncpy(path, g_chhome, strlen(g_chhome));
		path[strlen(g_chhome)] = '\0';
		strcat(path, "\\config\\_chrc");
		CopyFile(path, g_chrc, true);
	}
#else
	strcpy(g_xml, getenv("HOME"));
	strcat(g_xml, "/.robosimrc");
	strcpy(g_chrc, getenv("HOME"));
	strcat(g_chrc, "/.chrc");
#endif

	// parse xml config file
	readXMLConfig();

	// get size of chrc
	struct stat stbuf;
	if (!stat(g_chrc, &stbuf)) {
		fpbuf = new char[stbuf.st_size+1];
	}
	else {
		fpbuf = new char;
	}
	fpbuf[0] = '\0';

	// if chrc exists, get file into buffer(s)
	if ((fp = fopen(g_chrc, "r")) != NULL) {
		char line[1024];
		while (fgets(line, 1024, fp)) {
			if (!strcmp(line, "// RoboSim Begin\n")) {
				fgets(line, 1024, fp);		// ipath line
				fgets(line, 1024, fp);		// robosim end line
				continue;
			}
			strcat(fpbuf, line);
		}
		fclose(fp);
	}
	else {
		GtkWidget *d = gtk_message_dialog_new(
			GTK_WINDOW(gtk_builder_get_object(g_builder, "window1")),
			GTK_DIALOG_DESTROY_WITH_PARENT,
			GTK_MESSAGE_ERROR,
			GTK_BUTTONS_OK,
			"Could not load CHRC file.  Please run 'ch -d' from the Ch Command Shell.");
		int rc = gtk_dialog_run(GTK_DIALOG(d));
		exit(1);
	}

	// write config file for simulated robots
	fp = fopen(g_chrc, "w");
	fputs(fpbuf, fp);
	printRoboSimPath();
	fclose(fp);

	// show main window
	gtk_widget_show(g_window);
	gtk_main();

	return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
/*
 * When window is closed
 */
G_MODULE_EXPORT void on_window_destroy(GtkWidget *widget, gpointer data) {
	// save configuration file
	g_doc.SaveFile(g_xml);

	// write config file for hardware robots
	fp = fopen(g_chrc, "w");
	fputs(fpbuf, fp);
	fclose(fp);

	// quit
    gtk_main_quit();
}

/*
 * Aboutdialog open
 */
G_MODULE_EXPORT void on_aboutdialog_activate(GtkWidget *widget, gpointer data) {
	GtkWidget *w;
	w = GTK_WIDGET(gtk_builder_get_object(g_builder, "aboutdialog"));
	gtk_dialog_run(GTK_DIALOG(w));
}

/*
 * Open webpage when URL is clicked
 */
G_MODULE_EXPORT void on_aboutdialog_activate_link(GtkAboutDialog *label, gchar *uri, gpointer data) {
#ifdef _WIN32
	ShellExecuteA(NULL, "open", uri, NULL, NULL, 0);
#endif
}

/*
 * Aboutdialog X button
 */
G_MODULE_EXPORT void on_aboutdialog_close(GtkDialog *dialog, gpointer user_data) {
	gtk_widget_hide(GTK_WIDGET(dialog));
}

/*
 * Aboutdialog close button
 */
G_MODULE_EXPORT void on_aboutdialog_response(GtkDialog *dialog, gint response_id, gpointer user_data) {
	gtk_widget_hide(GTK_WIDGET(dialog));
}

/*
 * Changelog dialog open
 */
G_MODULE_EXPORT void on_changelog_activate(GtkWidget *widget, gpointer data) {
	GtkWidget *dialog = gtk_dialog_new_with_buttons("RoboSim ChangeLog",
		GTK_WINDOW(gtk_builder_get_object(g_builder, "window1")),
		GTK_DIALOG_DESTROY_WITH_PARENT,
		("_OK"),
		GTK_RESPONSE_NONE,
		NULL);
	gtk_window_set_default_size(GTK_WINDOW(dialog), -1, 500);
	GtkWidget *content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

	// scrolled area for text display
	GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
	gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window), GTK_POLICY_NEVER, GTK_POLICY_ALWAYS);

	// open changelog and read contents
	FILE *fp2 = fopen("CHANGELOG", "r");
	char *fp2buf;
	struct stat stbuf;
	if (!stat("CHANGELOG", &stbuf)) {
		fp2buf = new char[stbuf.st_size+1];
	}
	else {
		fp2buf = new char;
	}
	fp2buf[0] = '\0';
	if (fp2 != NULL) {
		char line[1024];
		while (fgets(line, 1024, fp2)) {
			strcat(fp2buf, line);
		}
		fclose(fp2);
	}
	GtkWidget *label = gtk_label_new(fp2buf);

	// connect signal to close button
	g_signal_connect_swapped(dialog, "response", G_CALLBACK(gtk_widget_destroy), dialog);

	// add content to widget
#ifdef _WIN32
	// gtk2
	gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(scrolled_window), label);
#else
	// gtk3 - above function deprecated
	gtk_container_add(GTK_CONTAINER(scrolled_window), label);
#endif
	gtk_box_pack_start(GTK_BOX(content_area), scrolled_window, TRUE, TRUE, 0);

	// show widget
	gtk_widget_show_all(dialog);
}

/*
 * Help menu
 */
G_MODULE_EXPORT void on_menuitem_help_activate(GtkWidget *widget, gpointer data) {
#ifdef _WIN32
	char path[1024];
	strncpy(path, g_chhome, strlen(g_chhome));
	path[strlen(g_chhome)] = '\0';
	strcat(path, "\\package\\chrobosim\\docs\\robosim.pdf");
	ShellExecuteA(NULL, "open", path, NULL, NULL, SW_SHOWNORMAL);
#endif
}

/*
 * When switching tabs of notebook
 */
G_MODULE_EXPORT void on_notebook_switch_page(GtkWidget *widget, gpointer data) {
	int page = gtk_notebook_get_current_page(GTK_NOTEBOOK(gtk_builder_get_object(g_builder, "notebook")));

	if (page == 1) {
		saveRobotList(1);
	}
	else if (page == 0) {
		switch (g_type) {
			case BOW:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 1);
				break;
			case EXPLORER:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 1);
				break;
			case FOURBOTDRIVE:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 1);
				break;
			case FOURWHEELDRIVE:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 1);
				break;
			case FOURWHEELEXPLORER:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 1);
				break;
			case GROUPBOW:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 1);
				break;
			case INCHWORM:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 1);
				break;
			case LIFT:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 1);
				break;
			case P_OMNIDRIVE:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 1);
				break;
			case SNAKE:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 1);
				break;
			case STAND:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 1);
				break;
		}
	}
}

/*
 * When hardware robots are selected
 */
G_MODULE_EXPORT void on_real_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "real")))) {
		fp = fopen(g_chrc, "w");
		fputs(fpbuf, fp);
		fclose(fp);
	}
}

/*
 * When simulated robots are selected
 */
G_MODULE_EXPORT void on_simulated_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "simulated")))) {
		fp = fopen(g_chrc, "w");
		fputs(fpbuf, fp);
		printRoboSimPath();
		fclose(fp);
	}
}

/*
 * When a robots type is changed
 */
G_MODULE_EXPORT void on_type_changed(GtkWidget *widget, gpointer data) {
	// cast id of robot
	gint id = GPOINTER_TO_INT(data);

	// scan through robots to find id
	robots_t tmp = g_robots;
	while (tmp && tmp->id != id)
		tmp = tmp->next;

	// if the robot is found, change its type
	if (tmp) {
		// get new robot type
#ifdef _WIN32
		const gchar *type = gtk_combo_box_get_active_text(GTK_COMBO_BOX(widget));
#else
		const gchar *type = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(widget));
#endif
		// store into database
		if (!strcmp(type, "Linkbot I"))
			tmp->type = 0;
		else if (!strcmp(type, "Linkbot L"))
			tmp->type = 1;
		else if (!strcmp(type, "Linkbot T"))
			tmp->type = 2;
		else if (!strcmp(type, "Mobot"))
			tmp->type = 3;

		// save configuration
		saveRobotList();
	}
}

/*
 * When robot's x value is changed
 */
G_MODULE_EXPORT void on_x_value_changed(GtkWidget *widget, gpointer data) {
	// cast id of robot
	gint id = GPOINTER_TO_INT(data);

	// scan through robots to find id
	robots_t tmp = g_robots;
	while (tmp && tmp->id != id)
		tmp = tmp->next;

	// if the robot is found, change its x value
	if (tmp) {
		// store new value
		tmp->x = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));

		// save configuration
		saveRobotList();
	}
}

/*
 * When robot's y value is changed
 */
G_MODULE_EXPORT void on_y_value_changed(GtkWidget *widget, gpointer data) {
	// cast id of robot
	gint id = GPOINTER_TO_INT(data);

	// scan through robots to find id
	robots_t tmp = g_robots;
	while (tmp && tmp->id != id)
		tmp = tmp->next;

	// if the robot is found, change its y value
	if (tmp) {
		// store new value
		tmp->y = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));

		// save configuration
		saveRobotList();
	}
}

/*
 * When robot's phi value is changed
 */
G_MODULE_EXPORT void on_phi_value_changed(GtkWidget *widget, gpointer data) {
	// cast id of robot
	gint id = GPOINTER_TO_INT(data);

	// scan through robots to find id
	robots_t tmp = g_robots;
	while (tmp && tmp->id != id)
		tmp = tmp->next;

	// if the robot is found, change its phi value
	if (tmp) {
		// store new value
		tmp->phi = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));

		// save configuration
		saveRobotList();
	}
}

/*
 * When a robot's wheel type is changed
 */
G_MODULE_EXPORT void on_wheeled_changed(GtkWidget *widget, gpointer data) {
	// cast id of robot
	gint id = GPOINTER_TO_INT(data);

	// scan through robots to find id
	robots_t tmp = g_robots;
	while (tmp && tmp->id != id)
		tmp = tmp->next;

	// if the robot is found, change its type
	if (tmp) {
		// get new robot type
#ifdef _WIN32
		const gchar *type = gtk_combo_box_get_active_text(GTK_COMBO_BOX(widget));
#else
		const gchar *type = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(widget));
#endif
		// store into database
		if (g_units) {
			if (!strcmp(type, "None"))
				tmp->wheel = -1;
			else if (!strcmp(type, "2.0"))
				tmp->wheel = BIGWHEEL;
			else if (!strcmp(type, "1.75"))
				tmp->wheel = SMALLWHEEL;
			else if (!strcmp(type, "1.625"))
				tmp->wheel = TINYWHEEL;
			else if (!strcmp(type, "Custom"))
				tmp->wheel = WHEEL;
		}
		else {
			if (!strcmp(type, "None"))
				tmp->wheel = -1;
			else if (!strcmp(type, "5.08"))
				tmp->wheel = BIGWHEEL;
			else if (!strcmp(type, "4.45"))
				tmp->wheel = SMALLWHEEL;
			else if (!strcmp(type, "4.13"))
				tmp->wheel = TINYWHEEL;
			else if (!strcmp(type, "Custom"))
				tmp->wheel = WHEEL;
		}

		// save configuration
		saveRobotList();
	}

	// refesh robot list
	refreshRobotList();
}

/*
 * When a robot's wheel type is changed
 */
G_MODULE_EXPORT void on_wheel_value_changed(GtkWidget *widget, gpointer data) {
	// cast id of robot
	gint id = GPOINTER_TO_INT(data);

	// scan through robots to find id
	robots_t tmp = g_robots;
	while (tmp && tmp->id != id)
		tmp = tmp->next;

	// if the robot is found, change its type
	if (tmp) {
		// save new wheel radius
		tmp->radius = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));

		// save configuration
		saveRobotList();
	}
}

/*
 * When a robot is added to the list
 */
G_MODULE_EXPORT void on_button_add_clicked(GtkWidget *widget, gpointer data) {
	// pointer to linked list
	robots_t tmp = g_robots;

	// new robot
	robots_t nr = new struct robots_s;
	nr->id = 0;
	nr->x = 0;
	if (tmp) {
		while (tmp->next)
			tmp = tmp->next;
		nr->id = tmp->id + 1;
		nr->x = tmp->x + ((g_units) ? 6 : 15); // add 6in or 15 cm
	}
	nr->type = 0;
	nr->y = 0;
	nr->z = 0;
	nr->psi = 0;
	nr->theta = 0;
	nr->phi = 0;
	nr->wheel = SMALLWHEEL;
	nr->next = NULL;

	// add robot to linked list
	tmp = g_robots;
	if (g_robots == NULL) {
		g_robots = nr;
	}
	else {
		while (tmp->next)
			tmp = tmp->next;
		tmp->next = nr;
	}

	// increase total number of robots
	g_num++;

	// refresh gui list with new robot
	refreshRobotList();

	// save configuration
	saveRobotList();

	// reset toggle buttons
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);
}

/*
 * When robot a robot is removed from the list
 */
G_MODULE_EXPORT void on_button_remove_clicked(GtkWidget* widget, gpointer data) {
	// cast id of robot
	gint id = GPOINTER_TO_INT(data);

	// find and remove ndoe
	robots_t tmp = g_robots;
	if (g_robots->id == id) {
		g_robots = g_robots->next;
	}
	else {
		while (tmp->next) {
			if (tmp->next->id == id) {
				tmp->next = tmp->next->next;
				break;
			}
			tmp = tmp->next;
		}
	}

	// reset robot ids to be sequential
	tmp = g_robots;
	int i = 0;
	while (tmp) {
		tmp->id = i++;
		tmp = tmp->next;
	}

	// decrease total number of robots
	g_num--;

	// refresh robot list
	refreshRobotList();

	// save configuration
	saveRobotList();
}

/*
 * When us customary units are chosen
 */
G_MODULE_EXPORT void on_us_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "us")))) {
		// get current values
		double major = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")));
		double tics = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "tics")));
		double minx = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "minx")));
		double maxx = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxx")));
		double miny = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "miny")));
		double maxy = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxy")));

		// set to customary defaults if at metric defaults
		if (	((int)major == g_grid[0][1]) &&
				((int)tics == g_grid[1][1]) &&
				((int)minx == g_grid[2][1]) &&
				((int)maxx == g_grid[3][1]) &&
				((int)miny == g_grid[4][1]) &&
				((int)maxy == g_grid[5][1])		) {
			major = g_grid[0][0], tics = g_grid[1][0], minx = g_grid[2][0], maxx = g_grid[3][0], miny = g_grid[4][0], maxy = g_grid[5][0];
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")), major);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "tics")), tics);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "minx")), minx);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxx")), maxx);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "miny")), miny);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxy")), maxy);
		}

		// set configuration options
		tinyxml2::XMLElement *grid = g_doc.FirstChildElement("config")->FirstChildElement("grid");
		grid->SetAttribute("units", 1);
		grid->SetAttribute("major", major);
		grid->SetAttribute("tics", tics);
		grid->SetAttribute("minx", minx);
		grid->SetAttribute("maxx", maxx);
		grid->SetAttribute("miny", miny);
		grid->SetAttribute("maxy", maxy);

		// change labels
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_major")), "Distance Between Hashmarks (in): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_tics")), "Distance Between Tics (in): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_minx")), "Min X (in): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_maxx")), "Max X (in): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_miny")), "Min Y (in): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_maxy")), "Max Y (in): ");

		// save units
		g_units = 1;

		// change labels
		refreshRobotList();

		// save configuration
		saveRobotList();

		// save file
		g_doc.SaveFile(g_xml);
	}
}

/*
 * When metric units are chosen
 */
G_MODULE_EXPORT void on_metric_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "metric")))) {
		// get current values
		double major = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")));
		double tics = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "tics")));
		double minx = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "minx")));
		double maxx = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxx")));
		double miny = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "miny")));
		double maxy = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxy")));

		// set to metric defaults if at customary defaults
		if (	((int)major == g_grid[0][0]) &&
				((int)tics == g_grid[1][0]) &&
				((int)minx == g_grid[2][0]) &&
				((int)maxx == g_grid[3][0]) &&
				((int)miny == g_grid[4][0]) &&
				((int)maxy == g_grid[5][0])		) {
			major = g_grid[0][1], tics = g_grid[1][1], minx = g_grid[2][1], maxx = g_grid[3][1], miny = g_grid[4][1], maxy = g_grid[5][1];
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")), major);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "tics")), tics);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "minx")), minx);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxx")), maxx);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "miny")), miny);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxy")), maxy);
		}

		// set configuration options
		tinyxml2::XMLElement *grid = g_doc.FirstChildElement("config")->FirstChildElement("grid");
		grid->SetAttribute("units", 0);
		grid->SetAttribute("major", major);
		grid->SetAttribute("tics", tics);
		grid->SetAttribute("minx", minx);
		grid->SetAttribute("maxx", maxx);
		grid->SetAttribute("miny", miny);
		grid->SetAttribute("maxy", maxy);

		// change labels
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_major")), "Distance Between Hashmarks (cm): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_tics")), "Distance Between Tics (cm): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_minx")), "Min X (cm): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_maxx")), "Max X (cm): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_miny")), "Min Y (cm): ");
		gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_maxy")), "Max Y (cm): ");

		// save units
		g_units = 0;

		// change labels
		refreshRobotList();

		// save configuration
		saveRobotList();

		// save file
		g_doc.SaveFile(g_xml);
	}
}

/*
 * When major value changed
 */
G_MODULE_EXPORT void on_major_value_changed(GtkWidget *widget, gpointer data) {
	double val = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
	tinyxml2::XMLElement *grid = g_doc.FirstChildElement("config")->FirstChildElement("grid");
	grid->SetAttribute("major", val);

	// save file
	g_doc.SaveFile(g_xml);
}

/*
 * When tics value changed
 */
G_MODULE_EXPORT void on_tics_value_changed(GtkWidget *widget, gpointer data) {
	// get old  value
	tinyxml2::XMLElement *grid = g_doc.FirstChildElement("config")->FirstChildElement("grid");
	double oldval = 0;
	grid->QueryDoubleAttribute("tics", &oldval);

	// get new value
	double val = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));

	// check if new val is factor of major value
	double maj = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")));
	int imaj = (int)maj;
	int ival = (int)val;
	if ( ival && ((imaj % ival) != 0) ) {
		int i = 1, j = 1;
		while ( (ival != imaj) && ((imaj % (ival + i)) != 0) ) { i++; }
		while ( (ival != imaj) && ((imaj % (ival - j)) != 0) ) { j++; }
		if ((int)val - (int)oldval == 1) {
			val += i;
		}
		else if ((int)val - (int)oldval == -1) {
			val -= j;
		}
		else {
			if (i < j)
				val += i;
			else
				val -= j;
		}
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(widget), val);
	}

	// set new value
	grid->SetAttribute("tics", val);

	// save file
	g_doc.SaveFile(g_xml);
}

/*
 * When minx value changed
 */
G_MODULE_EXPORT void on_minx_value_changed(GtkWidget *widget, gpointer data) {
	double val = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
	tinyxml2::XMLElement *grid = g_doc.FirstChildElement("config")->FirstChildElement("grid");
	grid->SetAttribute("minx", val);

	// save file
	g_doc.SaveFile(g_xml);
}

/*
 * When maxx value changed
 */
G_MODULE_EXPORT void on_maxx_value_changed(GtkWidget *widget, gpointer data) {
	double val = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
	tinyxml2::XMLElement *grid = g_doc.FirstChildElement("config")->FirstChildElement("grid");
	grid->SetAttribute("maxx", val);

	// save file
	g_doc.SaveFile(g_xml);
}

/*
 * When miny value changed
 */
G_MODULE_EXPORT void on_miny_value_changed(GtkWidget *widget, gpointer data) {
	double val = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
	tinyxml2::XMLElement *grid = g_doc.FirstChildElement("config")->FirstChildElement("grid");
	grid->SetAttribute("miny", val);

	// save file
	g_doc.SaveFile(g_xml);
}

/*
 * When maxy value changed
 */
G_MODULE_EXPORT void on_maxy_value_changed(GtkWidget *widget, gpointer data) {
	double val = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
	tinyxml2::XMLElement *grid = g_doc.FirstChildElement("config")->FirstChildElement("grid");
	grid->SetAttribute("maxy", val);

	// save file
	g_doc.SaveFile(g_xml);
}

/*
 * Restore default grid values
 */
G_MODULE_EXPORT void on_defaults_clicked(GtkWidget *widget, gpointer data) {
	if (g_units) {
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")), g_grid[0][0]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "tics")), g_grid[1][0]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "minx")), g_grid[2][0]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxx")), g_grid[3][0]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "miny")), g_grid[4][0]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxy")), g_grid[5][0]);
	}
	else {
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")), g_grid[0][1]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "tics")), g_grid[1][1]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "minx")), g_grid[2][1]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxx")), g_grid[3][1]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "miny")), g_grid[4][1]);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxy")), g_grid[5][1]);
	}

	// save file
	g_doc.SaveFile(g_xml);
}

/*
 * When bow is selected
 */
G_MODULE_EXPORT void on_bow_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/bow.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", BOW);
		g_type = BOW;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
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
		rot->SetAttribute("psi", -90);
		rot->SetAttribute("theta", 180);
		rot->SetAttribute("phi", 0);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkbotl");
		robot2->SetAttribute("id", 1);
		sim->InsertAfterChild(robot1, robot2);

		// insert bridge
		tinyxml2::XMLElement *bridge1 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(robot2, bridge1);
		tinyxml2::XMLElement *b1side1 = g_doc.NewElement("side");
		b1side1->SetAttribute("id", 1);
		b1side1->SetAttribute("robot", 0);
		b1side1->SetAttribute("face", 1);
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

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When the explorer is selected
 */
G_MODULE_EXPORT void on_explorer_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/explorer.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", EXPLORER);
		g_type = EXPLORER;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
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
		rot->SetAttribute("phi", 180);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkboti");
		robot2->SetAttribute("id", 1);
		robot3 = g_doc.NewElement("linkboti");
		robot3->SetAttribute("id", 2);
		robot3->SetAttribute("orientation", 3);
		tinyxml2::XMLElement *r3j = g_doc.NewElement("joint");
		r3j->SetAttribute("f1", -12);
		r3j->SetAttribute("f2", 0);
		r3j->SetAttribute("f3", 12);
		robot3->InsertFirstChild(r3j);
		robot4 = g_doc.NewElement("linkboti");
		robot4->SetAttribute("id", 3);
		tinyxml2::XMLElement *r4j = g_doc.NewElement("joint");
		r4j->SetAttribute("f1", -90);
		r4j->SetAttribute("f2", 0);
		r4j->SetAttribute("f3", 90);
		robot4->InsertFirstChild(r4j);
		robot5 = g_doc.NewElement("linkbotl");
		robot5->SetAttribute("id", 4);
		robot5->SetAttribute("orientation", 3);
		sim->InsertAfterChild(robot1, robot2);
		sim->InsertAfterChild(robot2, robot3);
		sim->InsertAfterChild(robot3, robot4);
		sim->InsertAfterChild(robot4, robot5);

		// add wheels
		tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
		s1side1->SetAttribute("id", 1);
		s1side1->SetAttribute("robot", 0);
		s1side1->SetAttribute("face", 3);
		simple1->InsertFirstChild(s1side1);
		tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
		s1side2->SetAttribute("id", 2);
		s1side2->SetAttribute("robot", 0);
		s1side2->SetAttribute("conn", 0);
		simple1->InsertAfterChild(s1side1, s1side2);
		sim->InsertAfterChild(robot5, simple1);

		tinyxml2::XMLElement *simple2 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s2side1 = g_doc.NewElement("side");
		s2side1->SetAttribute("id", 1);
		s2side1->SetAttribute("robot", 1);
		s2side1->SetAttribute("face", 1);
		simple2->InsertFirstChild(s2side1);
		tinyxml2::XMLElement *s2side2 = g_doc.NewElement("side");
		s2side2->SetAttribute("id", 2);
		s2side2->SetAttribute("robot", 1);
		s2side2->SetAttribute("conn", 0);
		simple2->InsertAfterChild(s2side1, s2side2);
		sim->InsertAfterChild(simple1, simple2);

		// insert cube
		tinyxml2::XMLElement *cube = g_doc.NewElement("cube");
		sim->InsertAfterChild(simple2, cube);
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
		b1side1->SetAttribute("face", 1);
		bridge1->InsertFirstChild(b1side1);
		tinyxml2::XMLElement *b1side2 = g_doc.NewElement("side");
		b1side2->SetAttribute("id", 2);
		b1side2->SetAttribute("robot", 3);
		b1side2->SetAttribute("face", 1);
		bridge1->InsertAfterChild(b1side1, b1side2);

		// insert bridge 2
		tinyxml2::XMLElement *bridge2 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge1, bridge2);
		tinyxml2::XMLElement *b2side1 = g_doc.NewElement("side");
		b2side1->SetAttribute("id", 1);
		b2side1->SetAttribute("robot", 2);
		b2side1->SetAttribute("face", 3);
		bridge2->InsertFirstChild(b2side1);
		tinyxml2::XMLElement *b2side2 = g_doc.NewElement("side");
		b2side2->SetAttribute("id", 2);
		b2side2->SetAttribute("robot", 3);
		b2side2->SetAttribute("face", 3);
		bridge2->InsertAfterChild(b2side1, b2side2);

		// insert simple
		tinyxml2::XMLElement *simple3 = g_doc.NewElement("simple");
		sim->InsertAfterChild(bridge2, simple3);
		tinyxml2::XMLElement *s3side1 = g_doc.NewElement("side");
		s3side1->SetAttribute("id", 1);
		s3side1->SetAttribute("robot", 3);
		s3side1->SetAttribute("face", 2);
		simple3->InsertFirstChild(s3side1);
		tinyxml2::XMLElement *s3side2 = g_doc.NewElement("side");
		s3side2->SetAttribute("id", 2);
		s3side2->SetAttribute("robot", 4);
		s3side2->SetAttribute("face", 2);
		simple3->InsertAfterChild(s3side1, s3side2);

		// insert gripper
		tinyxml2::XMLElement *gripper = g_doc.NewElement("gripper");
		sim->InsertAfterChild(simple3, gripper);
		gripper->SetAttribute("robot", 4);

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When fourbotdrive is selected
 */
G_MODULE_EXPORT void on_fourbotdrive_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/fourbotdrive.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", FOURBOTDRIVE);
		g_type = FOURBOTDRIVE;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
		tinyxml2::XMLElement *robot1, *robot2, *robot3, *robot4;

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
		rot->SetAttribute("psi", 90);
		rot->SetAttribute("theta", 180);
		rot->SetAttribute("phi", 180);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkboti");
		robot2->SetAttribute("id", 1);
		sim->InsertAfterChild(robot1, robot2);
		robot3 = g_doc.NewElement("linkboti");
		robot3->SetAttribute("id", 2);
		sim->InsertAfterChild(robot2, robot3);
		robot4 = g_doc.NewElement("linkboti");
		robot4->SetAttribute("id", 3);
		sim->InsertAfterChild(robot3, robot4);

		// insert omnidrive plate
		tinyxml2::XMLElement *omni = g_doc.NewElement("omnidrive");
		sim->InsertAfterChild(robot4, omni);
		tinyxml2::XMLElement *omnis1 = g_doc.NewElement("side");
		omnis1->SetAttribute("id", 1);
		omnis1->SetAttribute("robot", 0);
		omnis1->SetAttribute("face", 2);
		omni->InsertFirstChild(omnis1);
		tinyxml2::XMLElement *omnis2 = g_doc.NewElement("side");
		omnis2->SetAttribute("id", 2);
		omnis2->SetAttribute("robot", 1);
		omnis2->SetAttribute("face", 2);
		omni->InsertAfterChild(omnis1, omnis2);
		tinyxml2::XMLElement *omnis3 = g_doc.NewElement("side");
		omnis3->SetAttribute("id", 3);
		omnis3->SetAttribute("robot", 2);
		omnis3->SetAttribute("face", 2);
		omni->InsertAfterChild(omnis2, omnis3);
		tinyxml2::XMLElement *omnis4 = g_doc.NewElement("side");
		omnis4->SetAttribute("id", 4);
		omnis4->SetAttribute("robot", 3);
		omnis4->SetAttribute("face", 2);
		omni->InsertAfterChild(omnis3, omnis4);

		// add wheels
		tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
		s1side1->SetAttribute("id", 1);
		s1side1->SetAttribute("robot", 0);
		s1side1->SetAttribute("face", 1);
		simple1->InsertFirstChild(s1side1);
		tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
		s1side2->SetAttribute("id", 2);
		s1side2->SetAttribute("robot", 0);
		s1side2->SetAttribute("conn", 9);
		simple1->InsertAfterChild(s1side1, s1side2);
		sim->InsertAfterChild(omni, simple1);

		tinyxml2::XMLElement *simple2 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s2side1 = g_doc.NewElement("side");
		s2side1->SetAttribute("id", 1);
		s2side1->SetAttribute("robot", 1);
		s2side1->SetAttribute("face", 1);
		simple2->InsertFirstChild(s2side1);
		tinyxml2::XMLElement *s2side2 = g_doc.NewElement("side");
		s2side2->SetAttribute("id", 2);
		s2side2->SetAttribute("robot", 1);
		s2side2->SetAttribute("conn", 9);
		simple2->InsertAfterChild(s2side1, s2side2);
		sim->InsertAfterChild(simple1, simple2);

		tinyxml2::XMLElement *simple3 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s3side1 = g_doc.NewElement("side");
		s3side1->SetAttribute("id", 1);
		s3side1->SetAttribute("robot", 2);
		s3side1->SetAttribute("face", 3);
		simple3->InsertFirstChild(s3side1);
		tinyxml2::XMLElement *s3side2 = g_doc.NewElement("side");
		s3side2->SetAttribute("id", 2);
		s3side2->SetAttribute("robot", 2);
		s3side2->SetAttribute("conn", 9);
		simple3->InsertAfterChild(s3side1, s3side2);
		sim->InsertAfterChild(simple2, simple3);

		tinyxml2::XMLElement *simple4 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s4side1 = g_doc.NewElement("side");
		s4side1->SetAttribute("id", 1);
		s4side1->SetAttribute("robot", 3);
		s4side1->SetAttribute("face", 3);
		simple4->InsertFirstChild(s4side1);
		tinyxml2::XMLElement *s4side2 = g_doc.NewElement("side");
		s4side2->SetAttribute("id", 2);
		s4side2->SetAttribute("robot", 3);
		s4side2->SetAttribute("conn", 9);
		simple4->InsertAfterChild(s4side1, s4side2);
		sim->InsertAfterChild(simple3, simple4);

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When fourwheeldrive is selected
 */
G_MODULE_EXPORT void on_fourwheeldrive_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/fourwheeldrive.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", FOURWHEELDRIVE);
		g_type = FOURWHEELDRIVE;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
		tinyxml2::XMLElement *robot1, *robot2;

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
		sim->InsertAfterChild(robot1, robot2);

		// insert cube
		tinyxml2::XMLElement *cube = g_doc.NewElement("cube");
		sim->InsertAfterChild(robot2, cube);
		tinyxml2::XMLElement *cside1 = g_doc.NewElement("side");
		cside1->SetAttribute("id", 1);
		cside1->SetAttribute("robot", 0);
		cside1->SetAttribute("face", 2);
		cube->InsertFirstChild(cside1);
		tinyxml2::XMLElement *cside3 = g_doc.NewElement("side");
		cside3->SetAttribute("id", 3);
		cside3->SetAttribute("robot", 1);
		cside3->SetAttribute("face", 2);
		cube->InsertAfterChild(cside1, cside3);

		// robot 1 wheels
		tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
		s1side1->SetAttribute("id", 1);
		s1side1->SetAttribute("robot", 0);
		s1side1->SetAttribute("face", 1);
		simple1->InsertFirstChild(s1side1);
		tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
		s1side2->SetAttribute("id", 2);
		s1side2->SetAttribute("robot", 0);
		s1side2->SetAttribute("conn", 9);
		simple1->InsertAfterChild(s1side1, s1side2);
		sim->InsertAfterChild(cube, simple1);
		tinyxml2::XMLElement *simple2 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s2side1 = g_doc.NewElement("side");
		s2side1->SetAttribute("id", 1);
		s2side1->SetAttribute("robot", 0);
		s2side1->SetAttribute("face", 3);
		simple2->InsertFirstChild(s2side1);
		tinyxml2::XMLElement *s2side2 = g_doc.NewElement("side");
		s2side2->SetAttribute("id", 2);
		s2side2->SetAttribute("robot", 0);
		s2side2->SetAttribute("conn", 9);
		simple2->InsertAfterChild(s2side1, s2side2);
		sim->InsertAfterChild(simple1, simple2);

		// robot 2 wheels
		tinyxml2::XMLElement *simple3 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s3side1 = g_doc.NewElement("side");
		s3side1->SetAttribute("id", 1);
		s3side1->SetAttribute("robot", 1);
		s3side1->SetAttribute("face", 1);
		simple3->InsertFirstChild(s3side1);
		tinyxml2::XMLElement *s3side2 = g_doc.NewElement("side");
		s3side2->SetAttribute("id", 2);
		s3side2->SetAttribute("robot", 1);
		s3side2->SetAttribute("conn", 9);
		simple3->InsertAfterChild(s3side1, s3side2);
		sim->InsertAfterChild(simple2, simple3);
		tinyxml2::XMLElement *simple4 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s4side1 = g_doc.NewElement("side");
		s4side1->SetAttribute("id", 1);
		s4side1->SetAttribute("robot", 1);
		s4side1->SetAttribute("face", 3);
		simple4->InsertFirstChild(s4side1);
		tinyxml2::XMLElement *s4side2 = g_doc.NewElement("side");
		s4side2->SetAttribute("id", 2);
		s4side2->SetAttribute("robot", 1);
		s4side2->SetAttribute("conn", 9);
		simple4->InsertAfterChild(s4side1, s4side2);
		sim->InsertAfterChild(simple3, simple4);

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When the fourwheelexplorer is selected
 */
G_MODULE_EXPORT void on_fourwheelexplorer_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/fourwheelexplorer.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", FOURWHEELEXPLORER);
		g_type = FOURWHEELEXPLORER;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
		tinyxml2::XMLElement *robot1, *robot2, *robot3, *robot4, *robot5;

		// create first robot
		robot1 = g_doc.NewElement("linkboti");
		// set id
		robot1->SetAttribute("id", 0);
		// set position
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", 0);
		pos->SetAttribute("y", 0.074925);
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
		robot3->SetAttribute("orientation", 3);
		tinyxml2::XMLElement *r3j = g_doc.NewElement("joint");
		r3j->SetAttribute("f1", -12);
		r3j->SetAttribute("f2", 0);
		r3j->SetAttribute("f3", 12);
		robot3->InsertFirstChild(r3j);
		robot4 = g_doc.NewElement("linkboti");
		robot4->SetAttribute("id", 3);
		tinyxml2::XMLElement *r4j = g_doc.NewElement("joint");
		r4j->SetAttribute("f1", -90);
		r4j->SetAttribute("f2", 0);
		r4j->SetAttribute("f3", 90);
		robot4->InsertFirstChild(r4j);
		robot5 = g_doc.NewElement("linkbotl");
		robot5->SetAttribute("id", 4);
		robot5->SetAttribute("orientation", 3);
		sim->InsertAfterChild(robot1, robot2);
		sim->InsertAfterChild(robot2, robot3);
		sim->InsertAfterChild(robot3, robot4);
		sim->InsertAfterChild(robot4, robot5);

		// add wheels
		tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
		s1side1->SetAttribute("id", 1);
		s1side1->SetAttribute("robot", 0);
		s1side1->SetAttribute("face", 1);
		simple1->InsertFirstChild(s1side1);
		tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
		s1side2->SetAttribute("id", 2);
		s1side2->SetAttribute("robot", 0);
		s1side2->SetAttribute("conn", 9);
		simple1->InsertAfterChild(s1side1, s1side2);
		sim->InsertAfterChild(robot5, simple1);

		tinyxml2::XMLElement *simple2 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s2side1 = g_doc.NewElement("side");
		s2side1->SetAttribute("id", 1);
		s2side1->SetAttribute("robot", 0);
		s2side1->SetAttribute("face", 3);
		simple2->InsertFirstChild(s2side1);
		tinyxml2::XMLElement *s2side2 = g_doc.NewElement("side");
		s2side2->SetAttribute("id", 2);
		s2side2->SetAttribute("robot", 0);
		s2side2->SetAttribute("conn", 9);
		simple2->InsertAfterChild(s2side1, s2side2);
		sim->InsertAfterChild(simple1, simple2);

		tinyxml2::XMLElement *simple3 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s3side1 = g_doc.NewElement("side");
		s3side1->SetAttribute("id", 1);
		s3side1->SetAttribute("robot", 1);
		s3side1->SetAttribute("face", 1);
		simple3->InsertFirstChild(s3side1);
		tinyxml2::XMLElement *s3side2 = g_doc.NewElement("side");
		s3side2->SetAttribute("id", 2);
		s3side2->SetAttribute("robot", 1);
		s3side2->SetAttribute("conn", 9);
		simple3->InsertAfterChild(s3side1, s3side2);
		sim->InsertAfterChild(simple2, simple3);

		tinyxml2::XMLElement *simple4 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s4side1 = g_doc.NewElement("side");
		s4side1->SetAttribute("id", 1);
		s4side1->SetAttribute("robot", 1);
		s4side1->SetAttribute("face", 3);
		simple4->InsertFirstChild(s4side1);
		tinyxml2::XMLElement *s4side2 = g_doc.NewElement("side");
		s4side2->SetAttribute("id", 2);
		s4side2->SetAttribute("robot", 1);
		s4side2->SetAttribute("conn", 9);
		simple4->InsertAfterChild(s4side1, s4side2);
		sim->InsertAfterChild(simple3, simple4);

		// insert cube
		tinyxml2::XMLElement *cube = g_doc.NewElement("cube");
		sim->InsertAfterChild(simple4, cube);
		tinyxml2::XMLElement *cside1 = g_doc.NewElement("side");
		cside1->SetAttribute("id", 1);
		cside1->SetAttribute("robot", 0);
		cside1->SetAttribute("face", 2);
		cube->InsertFirstChild(cside1);
		tinyxml2::XMLElement *cside2 = g_doc.NewElement("side");
		cside2->SetAttribute("id", 2);
		cside2->SetAttribute("robot", 0);
		cside2->SetAttribute("conn", 2);
		cube->InsertAfterChild(cside1, cside2);
		tinyxml2::XMLElement *cside3 = g_doc.NewElement("side");
		cside3->SetAttribute("id", 3);
		cside3->SetAttribute("robot", 1);
		cside3->SetAttribute("face", 2);
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
		b1side1->SetAttribute("face", 1);
		bridge1->InsertFirstChild(b1side1);
		tinyxml2::XMLElement *b1side2 = g_doc.NewElement("side");
		b1side2->SetAttribute("id", 2);
		b1side2->SetAttribute("robot", 3);
		b1side2->SetAttribute("face", 1);
		bridge1->InsertAfterChild(b1side1, b1side2);

		// insert bridge 2
		tinyxml2::XMLElement *bridge2 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge1, bridge2);
		tinyxml2::XMLElement *b2side1 = g_doc.NewElement("side");
		b2side1->SetAttribute("id", 1);
		b2side1->SetAttribute("robot", 2);
		b2side1->SetAttribute("face", 3);
		bridge2->InsertFirstChild(b2side1);
		tinyxml2::XMLElement *b2side2 = g_doc.NewElement("side");
		b2side2->SetAttribute("id", 2);
		b2side2->SetAttribute("robot", 3);
		b2side2->SetAttribute("face", 3);
		bridge2->InsertAfterChild(b2side1, b2side2);

		// insert simple
		tinyxml2::XMLElement *simple5 = g_doc.NewElement("simple");
		sim->InsertAfterChild(bridge2, simple5);
		tinyxml2::XMLElement *s5side1 = g_doc.NewElement("side");
		s5side1->SetAttribute("id", 1);
		s5side1->SetAttribute("robot", 3);
		s5side1->SetAttribute("face", 2);
		simple5->InsertFirstChild(s5side1);
		tinyxml2::XMLElement *s5side2 = g_doc.NewElement("side");
		s5side2->SetAttribute("id", 2);
		s5side2->SetAttribute("robot", 4);
		s5side2->SetAttribute("face", 2);
		simple5->InsertAfterChild(s5side1, s5side2);

		// insert gripper
		tinyxml2::XMLElement *gripper = g_doc.NewElement("gripper");
		sim->InsertAfterChild(simple5, gripper);
		gripper->SetAttribute("robot", 4);

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When group bow is selected
 */
G_MODULE_EXPORT void on_groupbow_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/groupbow.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", GROUPBOW);
		g_type = GROUPBOW;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
		tinyxml2::XMLElement *robot1, *robot2, *robot3, *robot4;

		//
		// group 1
		//
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
		rot->SetAttribute("psi", -90);
		rot->SetAttribute("theta", 180);
		rot->SetAttribute("phi", 0);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkbotl");
		robot2->SetAttribute("id", 1);
		sim->InsertAfterChild(robot1, robot2);

		// insert bridge
		tinyxml2::XMLElement *bridge1 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(robot2, bridge1);
		tinyxml2::XMLElement *b1side1 = g_doc.NewElement("side");
		b1side1->SetAttribute("id", 1);
		b1side1->SetAttribute("robot", 0);
		b1side1->SetAttribute("face", 1);
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

		//
		// group 2
		//
		// create first robot
		robot3 = g_doc.NewElement("linkbotl");
		// set id
		robot3->SetAttribute("id", 2);
		// set position
		tinyxml2::XMLElement *pos1 = g_doc.NewElement("position");
		pos1->SetAttribute("x", convert(6, 1));
		pos1->SetAttribute("y", 0);
		pos1->SetAttribute("z", 0);
		robot3->InsertFirstChild(pos1);
		// set rotation
		tinyxml2::XMLElement *rot1 = g_doc.NewElement("rotation");
		rot1->SetAttribute("psi", -90);
		rot1->SetAttribute("theta", 180);
		rot1->SetAttribute("phi", 0);
		robot3->InsertAfterChild(pos1, rot1);
		// insert robot1
		sim->InsertAfterChild(robot1, robot3);

		// add remaining robots
		robot4 = g_doc.NewElement("linkbotl");
		robot4->SetAttribute("id", 3);
		sim->InsertAfterChild(robot2, robot4);

		// insert bridge
		tinyxml2::XMLElement *bridge2 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(faceplate2, bridge2);
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

		// insert faceplate
		tinyxml2::XMLElement *faceplate3 = g_doc.NewElement("faceplate");
		sim->InsertAfterChild(bridge2, faceplate3);
		faceplate3->SetAttribute("robot", 2);
		faceplate3->SetAttribute("face", 2);

		// insert faceplate
		tinyxml2::XMLElement *faceplate4 = g_doc.NewElement("faceplate");
		sim->InsertAfterChild(faceplate3, faceplate4);
		faceplate4->SetAttribute("robot", 3);
		faceplate4->SetAttribute("face", 2);

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When the inchworm is selected
 */
G_MODULE_EXPORT void on_inchworm_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/inchworm.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", INCHWORM);
		g_type = INCHWORM;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
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
		rot->SetAttribute("theta", 180);
		rot->SetAttribute("phi", 0);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkbotl");
		robot2->SetAttribute("id", 1);
		sim->InsertAfterChild(robot1, robot2);

		// insert bridge
		tinyxml2::XMLElement *bridge1 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(robot2, bridge1);
		tinyxml2::XMLElement *b1side1 = g_doc.NewElement("side");
		b1side1->SetAttribute("id", 1);
		b1side1->SetAttribute("robot", 0);
		b1side1->SetAttribute("face", 1);
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

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When the lift is selected
 */
G_MODULE_EXPORT void on_lift_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/lift.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", LIFT);
		g_type = LIFT;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
		tinyxml2::XMLElement *robot1, *robot2, *robot3, *robot4;

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
		sim->InsertAfterChild(robot1, robot2);
		robot3 = g_doc.NewElement("linkboti");
		robot3->SetAttribute("id", 2);
		robot3->SetAttribute("orientation", 3);
		sim->InsertAfterChild(robot2, robot3);
		robot4 = g_doc.NewElement("linkboti");
		robot4->SetAttribute("id", 3);
		sim->InsertAfterChild(robot3, robot4);

		// insert bridge 1
		tinyxml2::XMLElement *bridge1 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(robot4, bridge1);
		tinyxml2::XMLElement *b1side1 = g_doc.NewElement("side");
		b1side1->SetAttribute("id", 1);
		b1side1->SetAttribute("robot", 0);
		b1side1->SetAttribute("face", 1);
		bridge1->InsertFirstChild(b1side1);
		tinyxml2::XMLElement *b1side2 = g_doc.NewElement("side");
		b1side2->SetAttribute("id", 2);
		b1side2->SetAttribute("robot", 1);
		b1side2->SetAttribute("face", 1);
		bridge1->InsertAfterChild(b1side1, b1side2);

		// insert bridge 2
		tinyxml2::XMLElement *bridge2 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge1, bridge2);
		tinyxml2::XMLElement *b2side1 = g_doc.NewElement("side");
		b2side1->SetAttribute("id", 1);
		b2side1->SetAttribute("robot", 0);
		b2side1->SetAttribute("face", 3);
		bridge2->InsertFirstChild(b2side1);
		tinyxml2::XMLElement *b2side2 = g_doc.NewElement("side");
		b2side2->SetAttribute("id", 2);
		b2side2->SetAttribute("robot", 1);
		b2side2->SetAttribute("face", 3);
		bridge2->InsertAfterChild(b2side1, b2side2);

		// insert simple 1
		tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
		sim->InsertAfterChild(bridge2, simple1);
		tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
		s1side1->SetAttribute("id", 1);
		s1side1->SetAttribute("robot", 1);
		s1side1->SetAttribute("face", 2);
		simple1->InsertFirstChild(s1side1);
		tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
		s1side2->SetAttribute("id", 2);
		s1side2->SetAttribute("robot", 2);
		s1side2->SetAttribute("face", 2);
		simple1->InsertAfterChild(s1side1, s1side2);

		// insert bridge 3
		tinyxml2::XMLElement *bridge3 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(simple1, bridge3);
		tinyxml2::XMLElement *b3side1 = g_doc.NewElement("side");
		b3side1->SetAttribute("id", 1);
		b3side1->SetAttribute("robot", 2);
		b3side1->SetAttribute("face", 1);
		bridge3->InsertFirstChild(b3side1);
		tinyxml2::XMLElement *b3side2 = g_doc.NewElement("side");
		b3side2->SetAttribute("id", 2);
		b3side2->SetAttribute("robot", 3);
		b3side2->SetAttribute("face", 1);
		bridge3->InsertAfterChild(b3side1, b3side2);

		// insert bridge 4
		tinyxml2::XMLElement *bridge4 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge3, bridge4);
		tinyxml2::XMLElement *b4side1 = g_doc.NewElement("side");
		b4side1->SetAttribute("id", 1);
		b4side1->SetAttribute("robot", 2);
		b4side1->SetAttribute("face", 3);
		bridge4->InsertFirstChild(b4side1);
		tinyxml2::XMLElement *b4side2 = g_doc.NewElement("side");
		b4side2->SetAttribute("id", 2);
		b4side2->SetAttribute("robot", 3);
		b4side2->SetAttribute("face", 3);
		bridge4->InsertAfterChild(b4side1, b4side2);

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When the omnidrive is selected
 */
G_MODULE_EXPORT void on_omnidrive_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/omnidrive.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", P_OMNIDRIVE);
		g_type = P_OMNIDRIVE;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
		tinyxml2::XMLElement *robot1, *robot2, *robot3, *robot4;

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
		rot->SetAttribute("psi", 90);
		rot->SetAttribute("theta", 180);
		rot->SetAttribute("phi", -90);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkbotl");
		robot2->SetAttribute("id", 1);
		sim->InsertAfterChild(robot1, robot2);
		robot3 = g_doc.NewElement("linkbotl");
		robot3->SetAttribute("id", 2);
		robot3->SetAttribute("orientation", 3);
		sim->InsertAfterChild(robot2, robot3);
		robot4 = g_doc.NewElement("linkbotl");
		robot4->SetAttribute("id", 3);
		robot4->SetAttribute("orientation", 3);
		sim->InsertAfterChild(robot3, robot4);

		// insert omnidrive plate
		tinyxml2::XMLElement *omni = g_doc.NewElement("omnidrive");
		sim->InsertAfterChild(robot4, omni);
		tinyxml2::XMLElement *omnis1 = g_doc.NewElement("side");
		omnis1->SetAttribute("id", 1);
		omnis1->SetAttribute("robot", 0);
		omnis1->SetAttribute("face", 2);
		omni->InsertFirstChild(omnis1);
		tinyxml2::XMLElement *omnis2 = g_doc.NewElement("side");
		omnis2->SetAttribute("id", 2);
		omnis2->SetAttribute("robot", 1);
		omnis2->SetAttribute("face", 2);
		omni->InsertAfterChild(omnis1, omnis2);
		tinyxml2::XMLElement *omnis3 = g_doc.NewElement("side");
		omnis3->SetAttribute("id", 3);
		omnis3->SetAttribute("robot", 2);
		omnis3->SetAttribute("face", 2);
		omni->InsertAfterChild(omnis2, omnis3);
		tinyxml2::XMLElement *omnis4 = g_doc.NewElement("side");
		omnis4->SetAttribute("id", 4);
		omnis4->SetAttribute("robot", 3);
		omnis4->SetAttribute("face", 2);
		omni->InsertAfterChild(omnis3, omnis4);

		// add wheels
		tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
		s1side1->SetAttribute("id", 1);
		s1side1->SetAttribute("robot", 0);
		s1side1->SetAttribute("face", 1);
		simple1->InsertFirstChild(s1side1);
		tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
		s1side2->SetAttribute("id", 2);
		s1side2->SetAttribute("robot", 0);
		s1side2->SetAttribute("conn", 9);
		simple1->InsertAfterChild(s1side1, s1side2);
		sim->InsertAfterChild(omni, simple1);

		tinyxml2::XMLElement *simple2 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s2side1 = g_doc.NewElement("side");
		s2side1->SetAttribute("id", 1);
		s2side1->SetAttribute("robot", 1);
		s2side1->SetAttribute("face", 1);
		simple2->InsertFirstChild(s2side1);
		tinyxml2::XMLElement *s2side2 = g_doc.NewElement("side");
		s2side2->SetAttribute("id", 2);
		s2side2->SetAttribute("robot", 1);
		s2side2->SetAttribute("conn", 9);
		simple2->InsertAfterChild(s2side1, s2side2);
		sim->InsertAfterChild(simple1, simple2);

		tinyxml2::XMLElement *simple3 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s3side1 = g_doc.NewElement("side");
		s3side1->SetAttribute("id", 1);
		s3side1->SetAttribute("robot", 2);
		s3side1->SetAttribute("face", 1);
		simple3->InsertFirstChild(s3side1);
		tinyxml2::XMLElement *s3side2 = g_doc.NewElement("side");
		s3side2->SetAttribute("id", 2);
		s3side2->SetAttribute("robot", 2);
		s3side2->SetAttribute("conn", 9);
		simple3->InsertAfterChild(s3side1, s3side2);
		sim->InsertAfterChild(simple2, simple3);

		tinyxml2::XMLElement *simple4 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s4side1 = g_doc.NewElement("side");
		s4side1->SetAttribute("id", 1);
		s4side1->SetAttribute("robot", 3);
		s4side1->SetAttribute("face", 1);
		simple4->InsertFirstChild(s4side1);
		tinyxml2::XMLElement *s4side2 = g_doc.NewElement("side");
		s4side2->SetAttribute("id", 2);
		s4side2->SetAttribute("robot", 3);
		s4side2->SetAttribute("conn", 9);
		simple4->InsertAfterChild(s4side1, s4side2);
		sim->InsertAfterChild(simple3, simple4);


		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When the snake is selected
 */
G_MODULE_EXPORT void on_snake_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/snake.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", SNAKE);
		g_type = SNAKE;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
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
		rot->SetAttribute("phi", 180);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkboti");
		robot2->SetAttribute("id", 1);
		robot2->SetAttribute("orientation", 3);
		robot3 = g_doc.NewElement("linkboti");
		robot3->SetAttribute("id", 2);
		robot4 = g_doc.NewElement("linkboti");
		robot4->SetAttribute("id", 3);
		robot4->SetAttribute("orientation", 3);
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
		b1side1->SetAttribute("face", 1);
		bridge1->InsertFirstChild(b1side1);
		tinyxml2::XMLElement *b1side2 = g_doc.NewElement("side");
		b1side2->SetAttribute("id", 2);
		b1side2->SetAttribute("robot", 2);
		b1side2->SetAttribute("face", 1);
		bridge1->InsertAfterChild(b1side1, b1side2);

		// insert bridge 2
		tinyxml2::XMLElement *bridge2 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge1, bridge2);
		tinyxml2::XMLElement *b2side1 = g_doc.NewElement("side");
		b2side1->SetAttribute("id", 1);
		b2side1->SetAttribute("robot", 1);
		b2side1->SetAttribute("face", 3);
		bridge2->InsertFirstChild(b2side1);
		tinyxml2::XMLElement *b2side2 = g_doc.NewElement("side");
		b2side2->SetAttribute("id", 2);
		b2side2->SetAttribute("robot", 2);
		b2side2->SetAttribute("face", 3);
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
		b3side1->SetAttribute("face", 1);
		bridge3->InsertFirstChild(b3side1);
		tinyxml2::XMLElement *b3side2 = g_doc.NewElement("side");
		b3side2->SetAttribute("id", 2);
		b3side2->SetAttribute("robot", 4);
		b3side2->SetAttribute("face", 1);
		bridge3->InsertAfterChild(b3side1, b3side2);

		// insert bridge 4
		tinyxml2::XMLElement *bridge4 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge3, bridge4);
		tinyxml2::XMLElement *b4side1 = g_doc.NewElement("side");
		b4side1->SetAttribute("id", 1);
		b4side1->SetAttribute("robot", 3);
		b4side1->SetAttribute("face", 3);
		bridge4->InsertFirstChild(b4side1);
		tinyxml2::XMLElement *b4side2 = g_doc.NewElement("side");
		b4side2->SetAttribute("id", 2);
		b4side2->SetAttribute("robot", 4);
		b4side2->SetAttribute("face", 3);
		bridge4->InsertAfterChild(b4side1, b4side2);

		// insert caster
		tinyxml2::XMLElement *caster = g_doc.NewElement("caster");
		sim->InsertAfterChild(bridge4, caster);
		caster->SetAttribute("robot", 4);
		caster->SetAttribute("face", 2);

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When the stand is selected
 */
G_MODULE_EXPORT void on_stand_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);

		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/stand.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 300, 200, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);

		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", STAND);
		g_type = STAND;

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// create robots
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
		b1side1->SetAttribute("face", 1);
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

		// save file
		g_doc.SaveFile(g_xml);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);

		// reset to individual config
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// save robot list
		saveRobotList();
	}
}

/*
 * When tracking is enabled
 */
G_MODULE_EXPORT void on_tracking_toggled(GtkWidget *widget, gpointer data) {
	tinyxml2::XMLElement *tracking = g_doc.FirstChildElement("config")->FirstChildElement("tracking");

	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		tracking->SetAttribute("val", 1);
		g_doc.SaveFile(g_xml);
	}
	else {
		tracking->SetAttribute("val", 0);
		g_doc.SaveFile(g_xml);
	}
}

/*
 * Convert [in/cm] to [m] and back
 */
double convert(double value, int tometer) {
	double tmp = 0;

	if (tometer)
		tmp = ((g_units) ? value/39.370 : value/100);
	else
		tmp = ((g_units) ? value*39.370 : value*100);

	return tmp;
}

/*
 * Ch installation path
 */
void printRoboSimPath(void) {
	// print RoboSim config options to file buffer
#ifdef _WIN32
	fputs("// RoboSim Begin\n_ipath = stradd(\"", fp);
	fputs(g_chhome, fp);
	fputs("/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp);
#else
	fputs("// RoboSim Begin\n_ipath = stradd(\"/usr/local/ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp);
#endif
}

/*
 * Read config file
 */
void readXMLConfig(void) {
	// check if config file exists
	if ( g_doc.LoadFile(g_xml) ) {
		// set declaration
		tinyxml2::XMLDeclaration *dec = g_doc.NewDeclaration();
		g_doc.InsertFirstChild(dec);

		// set configuration options
		tinyxml2::XMLElement *config = g_doc.NewElement("config");
		g_doc.InsertAfterChild(dec, config);

		// set new version
		tinyxml2::XMLElement *version = g_doc.NewElement("version");
		version->SetAttribute("val", XML_VERSION);
		config->InsertFirstChild(version);

		// set individual robots as default
		tinyxml2::XMLElement *type = g_doc.NewElement("type");
		type->SetAttribute("val", 0);
		config->InsertAfterChild(version, type);

		// set grid values
		tinyxml2::XMLElement *grid = g_doc.NewElement("grid");
		grid->SetAttribute("units", 1);
		grid->SetAttribute("major", g_grid[0][0]);
		grid->SetAttribute("tics", g_grid[1][0]);
		grid->SetAttribute("minx", g_grid[2][0]);
		grid->SetAttribute("maxx", g_grid[3][0]);
		grid->SetAttribute("miny", g_grid[4][0]);
		grid->SetAttribute("maxy", g_grid[5][0]);
		config->InsertAfterChild(type, grid);

		// set tracking of robots
		tinyxml2::XMLElement *tracking = g_doc.NewElement("tracking");
		tracking->SetAttribute("val", 1);
		config->InsertAfterChild(grid, tracking);

		// create empty simulation
		tinyxml2::XMLElement *sim = g_doc.NewElement("sim");
		g_doc.InsertAfterChild(config, sim);

		// set robot type
		tinyxml2::XMLElement *robot = g_doc.NewElement("linkboti");
		robot->SetAttribute("id", 0);
		sim->InsertFirstChild(robot);

		// set position
		tinyxml2::XMLElement *pos = g_doc.NewElement("position");
		pos->SetAttribute("x", 0);
		pos->SetAttribute("y", 0);
		pos->SetAttribute("z", 0);
		robot->InsertFirstChild(pos);

		// set rotation
		tinyxml2::XMLElement *rot = g_doc.NewElement("rotation");
		rot->SetAttribute("psi", 0);
		rot->SetAttribute("theta", 0);
		rot->SetAttribute("phi", 0);
		robot->InsertAfterChild(pos, rot);

		// add wheels
		tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
		s1side1->SetAttribute("id", 1);
		s1side1->SetAttribute("robot", 0);
		s1side1->SetAttribute("face", 1);
		simple1->InsertFirstChild(s1side1);
		tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
		s1side2->SetAttribute("id", 2);
		s1side2->SetAttribute("robot", 0);
		s1side2->SetAttribute("conn", 9);
		simple1->InsertAfterChild(s1side1, s1side2);

		tinyxml2::XMLElement *simple2 = g_doc.NewElement("simple");
		tinyxml2::XMLElement *s2side1 = g_doc.NewElement("side");
		s2side1->SetAttribute("id", 1);
		s2side1->SetAttribute("robot", 0);
		s2side1->SetAttribute("face", 3);
		simple2->InsertFirstChild(s2side1);
		tinyxml2::XMLElement *s2side2 = g_doc.NewElement("side");
		s2side2->SetAttribute("id", 2);
		s2side2->SetAttribute("robot", 0);
		s2side2->SetAttribute("conn", 9);
		simple2->InsertAfterChild(s2side1, s2side2);

		// add caster
		tinyxml2::XMLElement *caster = g_doc.NewElement("caster");
		caster->SetAttribute("robot", 0);
		caster->SetAttribute("face", 2);

		// add accessories to robot
		sim->InsertAfterChild(robot, simple1);
		sim->InsertAfterChild(simple1, simple2);
		sim->InsertAfterChild(simple2, caster);

		// save default config file
		g_doc.SaveFile(g_xml);
	}

	tinyxml2::XMLElement *node;
	tinyxml2::XMLElement *ele;
	int version = 0;

	// create config node if it doesn't exist
	if ( (node = g_doc.FirstChildElement("config")) == NULL) {
		node = g_doc.NewElement("config");
		g_doc.InsertAfterChild(g_doc.FirstChild(), node);
	}

	// check version of config file
	if ( (node = g_doc.FirstChildElement("config")->FirstChildElement("version")) ) {
		node->QueryIntAttribute("val", &version);
	}
	else {
		tinyxml2::XMLElement *version = g_doc.NewElement("version");
		version->SetAttribute("val", XML_VERSION);
		g_doc.FirstChildElement("config")->InsertFirstChild(version);
	}

	// check invidivual vs preconfigured
	if ( (node = g_doc.FirstChildElement("config")->FirstChildElement("type")) ) {
		node->QueryIntAttribute("val", &g_type);
		if (g_type) {
			gtk_notebook_set_current_page(GTK_NOTEBOOK(gtk_builder_get_object(g_builder, "notebook")), 1);
		}
		switch (g_type) {
			case BOW:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "bow")), 1);
				break;
			case EXPLORER:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 1);
				break;
			case FOURBOTDRIVE:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourbotdrive")), 1);
				break;
			case FOURWHEELDRIVE:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheeldrive")), 1);
				break;
			case FOURWHEELEXPLORER:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "fourwheelexplorer")), 1);
				break;
			case GROUPBOW:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "groupbow")), 1);
				break;
			case INCHWORM:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 1);
				break;
			case LIFT:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 1);
				break;
			case P_OMNIDRIVE:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 1);
				break;
			case SNAKE:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 1);
				break;
			case STAND:
				gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 1);
				break;
		}
	}
	else {
		tinyxml2::XMLElement *type = g_doc.NewElement("type");
		type->SetAttribute("val", 0);
		g_doc.FirstChildElement("config")->InsertFirstChild(type);
	}

	// read individual robot configuration
	if (!g_type) {
		if ( (node = g_doc.FirstChildElement("sim")) == NULL) {
			node = g_doc.NewElement("sim");
			g_doc.InsertAfterChild(g_doc.FirstChildElement("config"), node);
		}
		node = g_doc.FirstChildElement("sim")->FirstChildElement();
		robots_t tmp = g_robots;
		while (node) {
			if ( !strcmp(node->Value(), "linkboti") ) {
				robots_t nr = new struct robots_s;
				nr->type = 0;
				nr->x = 0; nr->y = 0; nr->z = 0;
				nr->psi = 0; nr->theta = 0; nr->phi = 0;
				nr->wheel = -1;
				node->QueryIntAttribute("id", &(nr->id));
				if ( (ele = node->FirstChildElement("position")) ) {
					ele->QueryDoubleAttribute("x", &(nr->x));
					ele->QueryDoubleAttribute("y", &(nr->y));
					ele->QueryDoubleAttribute("z", &(nr->z));
				}
				if ( (ele = node->FirstChildElement("rotation")) ) {
					ele->QueryDoubleAttribute("psi", &(nr->psi));
					ele->QueryDoubleAttribute("theta", &(nr->theta));
					ele->QueryDoubleAttribute("phi", &(nr->phi));
				}
				nr->next = NULL;
				g_num++;

				// add robot to linked list
				tmp = g_robots;
				if (g_robots == NULL) {
					g_robots = nr;
				}
				else {
					while (tmp->next)
						tmp = tmp->next;
					tmp->next = nr;
				}
			}
			else if ( !strcmp(node->Value(), "linkbotl") ) {
				robots_t nr = new struct robots_s;
				nr->type = 1;
				nr->x = 0; nr->y = 0; nr->z = 0;
				nr->psi = 0; nr->theta = 0; nr->phi = 0;
				nr->wheel = -1;
				node->QueryIntAttribute("id", &(nr->id));
				if ( (ele = node->FirstChildElement("position")) ) {
					ele->QueryDoubleAttribute("x", &(nr->x));
					ele->QueryDoubleAttribute("y", &(nr->y));
					ele->QueryDoubleAttribute("z", &(nr->z));
				}
				if ( (ele = node->FirstChildElement("rotation")) ) {
					ele->QueryDoubleAttribute("psi", &(nr->psi));
					ele->QueryDoubleAttribute("theta", &(nr->theta));
					ele->QueryDoubleAttribute("phi", &(nr->phi));
				}
				nr->next = NULL;
				g_num++;

				// add robot to linked list
				tmp = g_robots;
				if (g_robots == NULL) {
					g_robots = nr;
				}
				else {
					while (tmp->next)
						tmp = tmp->next;
					tmp->next = nr;
				}
			}
			else if ( !strcmp(node->Value(), "linkbott") ) {
				robots_t nr = new struct robots_s;
				nr->type = 2;
				nr->x = 0; nr->y = 0; nr->z = 0;
				nr->psi = 0; nr->theta = 0; nr->phi = 0;
				nr->wheel = -1;
				node->QueryIntAttribute("id", &(nr->id));
				if ( (ele = node->FirstChildElement("position")) ) {
					ele->QueryDoubleAttribute("x", &(nr->x));
					ele->QueryDoubleAttribute("y", &(nr->y));
					ele->QueryDoubleAttribute("z", &(nr->z));
				}
				if ( (ele = node->FirstChildElement("rotation")) ) {
					ele->QueryDoubleAttribute("psi", &(nr->psi));
					ele->QueryDoubleAttribute("theta", &(nr->theta));
					ele->QueryDoubleAttribute("phi", &(nr->phi));
				}
				nr->next = NULL;
				g_num++;

				// add robot to linked list
				tmp = g_robots;
				if (g_robots == NULL) {
					g_robots = nr;
				}
				else {
					while (tmp->next)
						tmp = tmp->next;
					tmp->next = nr;
				}
			}
			else if ( !strcmp(node->Value(), "mobot") ) {
				robots_t nr = new struct robots_s;
				nr->type = 3;
				nr->x = 0; nr->y = 0; nr->z = 0;
				nr->psi = 0; nr->theta = 0; nr->phi = 0;
				nr->wheel = -1;
				node->QueryIntAttribute("id", &(nr->id));
				if ( (ele = node->FirstChildElement("position")) ) {
					ele->QueryDoubleAttribute("x", &(nr->x));
					ele->QueryDoubleAttribute("y", &(nr->y));
					ele->QueryDoubleAttribute("z", &(nr->z));
				}
				if ( (ele = node->FirstChildElement("rotation")) ) {
					ele->QueryDoubleAttribute("psi", &(nr->psi));
					ele->QueryDoubleAttribute("theta", &(nr->theta));
					ele->QueryDoubleAttribute("phi", &(nr->phi));
				}
				nr->next = NULL;
				g_num++;

				// add robot to linked list
				tmp = g_robots;
				if (g_robots == NULL) {
					g_robots = nr;
				}
				else {
					while (tmp->next)
						tmp = tmp->next;
					tmp->next = nr;
				}
			}
			else if ( !strcmp(node->Value(), "simple") ) {
				tinyxml2::XMLElement *side = node->LastChildElement();
				if (side) {
					int id = -1, wheeltype = -1;
					double radius = 0;
					side->QueryIntAttribute("robot", &id);
					if (side->QueryIntAttribute("conn", &wheeltype) != tinyxml2::XML_NO_ATTRIBUTE) {
						tmp = g_robots;
						while (tmp && tmp->id != id)
							tmp = tmp->next;

						side->QueryDoubleAttribute("radius", &radius);
						tmp->radius = convert(radius, 0);
					}
				}
			}

			// go to next element
			node = node->NextSiblingElement();
		}

		// refesh robot list
		refreshRobotList();
	}

	// check grid settings
	if ( (node = g_doc.FirstChildElement("config")->FirstChildElement("grid")) ) {
		node->QueryIntAttribute("units", &g_units);

		// set grid line variables
		double major, tics, minx, maxx, miny, maxy;
		node->QueryDoubleAttribute("major", &major);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")), major);
		node->QueryDoubleAttribute("tics", &tics);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "tics")), tics);
		if (XML_VERSION == 1) {
			double dist;
			node->QueryDoubleAttribute("dist", &dist);
			minx = -dist/2;
			maxx = dist/2;
			miny = -dist/2;
			maxy = dist/2;
		}
		else {
			node->QueryDoubleAttribute("minx", &minx);
			node->QueryDoubleAttribute("maxx", &maxx);
			node->QueryDoubleAttribute("miny", &miny);
			node->QueryDoubleAttribute("maxy", &maxy);
		}
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "minx")), minx);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxx")), maxx);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "miny")), miny);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxy")), maxy);

		// set labels
		if (g_units) {
			gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "us")), 1);
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_major")), "Distance Between Hashmarks (in): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_tics")), "Distance Between Tics (in): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_minx")), "Min X (in): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_maxx")), "Max X (in): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_miny")), "Min Y (in): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_maxy")), "Max Y (in): ");
		}
		else {
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_major")), "Distance Between Hashmarks (cm): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_tics")), "Distance Between Tics (cm): ");
			gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "metric")), 1);
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_minx")), "Min X (cm): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_maxx")), "Max X (cm): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_miny")), "Min Y (cm): ");
			gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(g_builder, "label_maxy")), "Max Y (cm): ");
		}
	}
	else {
		tinyxml2::XMLElement *grid = g_doc.NewElement("grid");
		grid->SetAttribute("units", 1);
		double major = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "major")));
		grid->SetAttribute("major", major);
		double tics = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "tics")));
		grid->SetAttribute("tics", tics);
		double minx = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "minx")));
		grid->SetAttribute("minx", minx);
		double maxx = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxx")));
		grid->SetAttribute("maxx", maxx);
		double miny = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "miny")));
		grid->SetAttribute("miny", miny);
		double maxy = gtk_spin_button_get_value(GTK_SPIN_BUTTON(gtk_builder_get_object(g_builder, "maxy")));
		grid->SetAttribute("maxy", maxy);
		g_doc.FirstChildElement("config")->InsertFirstChild(grid);
	}

	// check if tracking is enabled
	if ( (node = g_doc.FirstChildElement("config")->FirstChildElement("tracking")) ) {
		int tracking = 1;
		node->QueryIntAttribute("val", &tracking);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "tracking")), tracking);
	}
	else {
		tinyxml2::XMLElement *tracking = g_doc.NewElement("tracking");
		tracking->SetAttribute("val", 1);
		g_doc.FirstChildElement("config")->InsertFirstChild(tracking);
	}

	// convert meters into gui units
	robots_t tmp = g_robots;
	while (tmp) {
		tmp->x = convert(tmp->x, 0);
		tmp->y = convert(tmp->y, 0);
		tmp->z = convert(tmp->z, 0);
		tmp = tmp->next;
	}

	// refesh robot list
	refreshRobotList();

	// success
	return;
}

/*
 * Refresh the table of robot data
 */
void refreshRobotList(void) {
	// new table of robots
	static GtkWidget *rootTable = NULL;
	if(rootTable != NULL)
		gtk_widget_destroy(rootTable);
	rootTable = gtk_table_new(g_num, 13, FALSE);

	// fill table with widgets
	GtkWidget *w;
	GtkAdjustment *x_adj, *y_adj, *phi_adj, *wheel_adj;
	robots_t tmp = g_robots;
	int i = 0;
	while (tmp) {
		// label for robot with id
		char label[11];
		sprintf(label, "Robot %2d: ", tmp->id+1);
		w = gtk_label_new(label);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 0, 1, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		// robot type
		w = gtk_combo_box_text_new();
#ifdef _WIN32
		gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "Linkbot I");
		gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "Linkbot L");
		//gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "Linkbot T");
		gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "Mobot");
#else
		gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "0", "Linkbot I");
		gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "1", "Linkbot L");
		//gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "2", "Linkbot T");
		gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "3", "Mobot");
#endif
		gtk_combo_box_set_active(GTK_COMBO_BOX(w), (tmp->type <= 1) ? tmp->type : tmp->type-1);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 2, 3, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "changed", G_CALLBACK(on_type_changed), (void *)(tmp->id));
		// x position
		w = ((g_units) ? gtk_label_new(" X [in]:") : gtk_label_new(" X [cm]:"));
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 3, 4, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		x_adj = GTK_ADJUSTMENT(gtk_adjustment_new(tmp->x, -500, 500, 0.1, 0.1, 1));
		w = gtk_spin_button_new(x_adj, 0.0, 1);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 4, 5, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "value-changed", G_CALLBACK(on_x_value_changed), (void *)(tmp->id));
		// y position
		w = ((g_units) ? gtk_label_new(" Y [in]:") : gtk_label_new(" Y [cm]:"));
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 5, 6, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		y_adj = GTK_ADJUSTMENT(gtk_adjustment_new(tmp->y, -500, 500, 0.1, 0.1, 1));
		w = gtk_spin_button_new(y_adj, 0.0, 1);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 6, 7, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "value-changed", G_CALLBACK(on_y_value_changed), (void *)(tmp->id));
		// phi angle
		w = gtk_label_new(" Angle [deg]:");
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 7, 8, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		phi_adj = GTK_ADJUSTMENT(gtk_adjustment_new(tmp->phi, -180, 180, 1, 1, 1));
		w = gtk_spin_button_new(phi_adj, 0.0, 1);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 8, 9, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "value-changed", G_CALLBACK(on_phi_value_changed), (void *)(tmp->id));
		// wheeled
		w = ((g_units) ? gtk_label_new(" Wheels [in]:") : gtk_label_new(" Wheels [cm]:"));
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 9, 10, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		w = gtk_combo_box_text_new();
		if (g_units) {
#ifdef _WIN32
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "None");
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "2.0");
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "1.75");
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "1.625");
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "Custom");
#else
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "0", "None");
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "1", "2.0");
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "2", "1.75");
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "3", "1.625");
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "4", "Custom");
#endif
		}
		else {
#ifdef _WIN32
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "None");
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "5.08");
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "4.45");
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "4.13");
			gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(w), "Custom");
#else
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "0", "None");
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "1", "5.08");
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "2", "4.45");
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "3", "4.13");
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(w), "4", "Custom");
#endif
		}
		switch (tmp->wheel) {
			case -1:
				gtk_combo_box_set_active(GTK_COMBO_BOX(w), 0);
				break;
			case BIGWHEEL:
				gtk_combo_box_set_active(GTK_COMBO_BOX(w), 1);
				break;
			case SMALLWHEEL:
				gtk_combo_box_set_active(GTK_COMBO_BOX(w), 2);
				break;
			case TINYWHEEL:
				gtk_combo_box_set_active(GTK_COMBO_BOX(w), 3);
				break;
			case WHEEL:
				gtk_combo_box_set_active(GTK_COMBO_BOX(w), 4);
				break;
		}
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 10, 11, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "changed", G_CALLBACK(on_wheeled_changed), (void *)(tmp->id));
		// wheel radius
		if (tmp->wheel == WHEEL) {
			wheel_adj = GTK_ADJUSTMENT(gtk_adjustment_new(tmp->radius, 0.1, 180, 0.1, 1, 1));
			w = gtk_spin_button_new(wheel_adj, 0.1, 3);
			gtk_widget_show(w);
			gtk_table_attach(GTK_TABLE(rootTable), w, 11, 12, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
			g_signal_connect(G_OBJECT(w), "value-changed", G_CALLBACK(on_wheel_value_changed), (void *)(tmp->id));
		}
		// remove button
		w = gtk_button_new_with_label("Remove");
		gtk_widget_show(w);
		if (tmp->wheel == WHEEL)
			gtk_table_attach(GTK_TABLE(rootTable), w, 12, 13, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		else
			gtk_table_attach(GTK_TABLE(rootTable), w, 11, 12, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "clicked", G_CALLBACK(on_button_remove_clicked), (void *)(tmp->id));
		// next robot
		tmp = tmp->next;
		i++;
	}

	// set size of table
	gtk_widget_show(rootTable);
	GtkRequisition sizeRequest;
	gtk_widget_size_request(rootTable, &sizeRequest);
	GtkWidget *layout = GTK_WIDGET(gtk_builder_get_object(g_builder, "layout_robots"));
	gtk_layout_set_size(GTK_LAYOUT(layout), sizeRequest.width, sizeRequest.height);
	gtk_layout_put(GTK_LAYOUT(layout), rootTable, 0, 0);

	// update liststore
	static GtkListStore *liststore = GTK_LIST_STORE(gtk_builder_get_object(g_builder, "liststore"));
	gtk_list_store_clear(liststore);
	GtkTreeIter iter;
	tmp = g_robots;
	while (tmp) {
		gtk_list_store_append(liststore, &iter);
		gtk_list_store_set(liststore, &iter, 0, tmp->id, -1);
		tmp = tmp->next;
	}
}

/*
 * When the save button is clicked
 */
void saveRobotList(int force) {
	tinyxml2::XMLElement *node;
	int type = 0;
	if ( (node = g_doc.FirstChildElement("config")->FirstChildElement("type")) ) {
		node->QueryIntAttribute("val", &type);
	}

	if (!type || force) {
		// set configuration options
		tinyxml2::XMLElement *type = g_doc.FirstChildElement("config")->FirstChildElement("type");
		type->SetAttribute("val", 0);

		// clean out sim node
		tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
		sim->DeleteChildren();

		// add each robot in list
		robots_t tmp = g_robots;
		while (tmp) {
			// set robot type
			tinyxml2::XMLElement *robot;
			if (tmp->type == 0)
				robot = g_doc.NewElement("linkboti");
			else if (tmp->type == 1)
				robot = g_doc.NewElement("linkbotl");
			else if (tmp->type == 2)
				robot = g_doc.NewElement("linkbott");
			else if (tmp->type == 3)
				robot = g_doc.NewElement("mobot");

			if (!(tmp->id))
				sim->InsertFirstChild(robot);
			else
				sim->InsertEndChild(robot);

			// set id
			robot->SetAttribute("id", tmp->id);

			// set position
			tinyxml2::XMLElement *pos = g_doc.NewElement("position");
			pos->SetAttribute("x", convert(tmp->x, 1));
			pos->SetAttribute("y", convert(tmp->y, 1));
			pos->SetAttribute("z", convert(tmp->z, 1));
			robot->InsertFirstChild(pos);

			// set rotation
			tinyxml2::XMLElement *rot = g_doc.NewElement("rotation");
			rot->SetAttribute("psi", tmp->psi);
			rot->SetAttribute("theta", tmp->theta);
			rot->SetAttribute("phi", tmp->phi);
			robot->InsertAfterChild(pos, rot);

			// add wheels
			if (tmp->type != 3 && tmp->wheel != -1) {
				tinyxml2::XMLElement *simple1 = g_doc.NewElement("simple");
				tinyxml2::XMLElement *s1side1 = g_doc.NewElement("side");
				s1side1->SetAttribute("id", 1);
				s1side1->SetAttribute("robot", tmp->id);
				s1side1->SetAttribute("face", 1);
				simple1->InsertFirstChild(s1side1);
				tinyxml2::XMLElement *s1side2 = g_doc.NewElement("side");
				s1side2->SetAttribute("id", 2);
				s1side2->SetAttribute("robot", tmp->id);
				simple1->InsertAfterChild(s1side1, s1side2);

				tinyxml2::XMLElement *simple2 = g_doc.NewElement("simple");
				tinyxml2::XMLElement *s2side1 = g_doc.NewElement("side");
				s2side1->SetAttribute("id", 1);
				s2side1->SetAttribute("robot", tmp->id);
				simple2->InsertFirstChild(s2side1);
				tinyxml2::XMLElement *s2side2 = g_doc.NewElement("side");
				s2side2->SetAttribute("id", 2);
				s2side2->SetAttribute("robot", tmp->id);
				simple2->InsertAfterChild(s2side1, s2side2);

				if (tmp->wheel != -1) {
					s1side2->SetAttribute("conn", tmp->wheel);
					s2side2->SetAttribute("conn", tmp->wheel);
				}
				if (tmp->wheel == WHEEL) {
					s1side2->SetAttribute("radius", convert(tmp->radius, 1));
					s2side2->SetAttribute("radius", convert(tmp->radius, 1));
				}

				tinyxml2::XMLElement *caster = g_doc.NewElement("caster");
				caster->SetAttribute("robot", tmp->id);

				if (tmp->type == 0) {
					s2side1->SetAttribute("face", 3);
					caster->SetAttribute("face", 2);
				}
				else if (tmp->type == 1) {
					s2side1->SetAttribute("face", 2);
					caster->SetAttribute("face", 3);
				}
				else if (tmp->type == 2) {
					s2side1->SetAttribute("face", 3);
					caster->SetAttribute("face", 2);
				}
				else if (tmp->type == 3) {
					s2side1->SetAttribute("face", 8);
					caster->SetAttribute("face", 3);
				}
				sim->InsertAfterChild(robot, simple1);
				sim->InsertAfterChild(simple1, simple2);
				sim->InsertAfterChild(simple2, caster);
			}
			else if (tmp->type == 3 && tmp->wheel) {
				tinyxml2::XMLElement	*wheel1 = g_doc.NewElement("smallwheel"),
										*wheel2 = g_doc.NewElement("smallwheel"),
										*caster = g_doc.NewElement("caster");
				wheel1->SetAttribute("robot", tmp->id);
				wheel1->SetAttribute("face", 1);
				caster->SetAttribute("robot", tmp->id);

				if (tmp->type == 0) {
					wheel2->SetAttribute("robot", tmp->id);
					wheel2->SetAttribute("face", 3);
					caster->SetAttribute("face", 2);
				}
				else if (tmp->type == 1) {
					wheel2->SetAttribute("robot", tmp->id);
					wheel2->SetAttribute("face", 2);
					caster->SetAttribute("face", 3);
				}
				else if (tmp->type == 2) {
					wheel2->SetAttribute("robot", tmp->id);
					wheel2->SetAttribute("face", 3);
					caster->SetAttribute("face", 2);
				}
				else if (tmp->type == 3) {
					wheel2->SetAttribute("robot", tmp->id);
					wheel2->SetAttribute("face", 8);
					caster->SetAttribute("face", 3);
				}
				sim->InsertAfterChild(robot, wheel1);
				sim->InsertAfterChild(wheel1, wheel2);
				sim->InsertAfterChild(wheel2, caster);
			}

			// go to next robot
			tmp = tmp->next;
		}
	}

	// save file
	g_doc.SaveFile(g_xml);
}
#ifdef __cplusplus
}
#endif
