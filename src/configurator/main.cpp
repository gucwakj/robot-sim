#include <stdlib.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#include <Shlwapi.h>
#endif
#include <gtk/gtk.h>
#include <tinyxml2.h>

#define I2M(x) ((x)/39.370)

typedef struct robots_s {
	int id;
	int type;
	double x, y, z;
	double psi, theta, phi;
	bool wheeled;
	struct robots_s *next;
} *robots_t;

GtkBuilder *g_builder; 
GtkWidget *g_window;
robots_t g_robots = NULL;
tinyxml2::XMLDocument g_doc;
FILE *fp = NULL;
char g_xml[512], g_chrc[512], *fpbuf;
int g_num = 0;

#ifdef __cplusplus
extern "C" {
#endif

void printRoboSimPath(void) {
	// print RoboSim config options to file buffer
#ifdef _WIN32
	fputs("// RoboSim Begin\n_ipath = stradd(\"C:/Ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp);
#else
	fputs("// RoboSim Begin\n_ipath = stradd(\"/usr/local/ch/package/chrobosim/include;\", _ipath);\n// RoboSim End\n", fp);
#endif
}

/*
 * When windows is closed
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
 * About Dialog Open
 */
G_MODULE_EXPORT void on_about_activate(GtkWidget *widget, gpointer data) {
	// Find the about dialog and show it
	GtkWidget *w;
	w = GTK_WIDGET(gtk_builder_get_object(g_builder, "aboutdialog"));
	gtk_dialog_run(GTK_DIALOG(w));
}

/*
 * About Dialog Close
 */
G_MODULE_EXPORT void on_aboutdialog_close(GtkDialog *dialog, gpointer user_data) {
	//gtk_widget_hide(GTK_WIDGET(dialog));
printf("close\n");
	gtk_widget_destroy(GTK_WIDGET(dialog));
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
	if (tmp) tmp->x = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
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
	if (tmp) tmp->y = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
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
	if (tmp) tmp->phi = gtk_spin_button_get_value(GTK_SPIN_BUTTON(widget));
}

/*
 * When robot's wheeled status is changed
 */
G_MODULE_EXPORT void on_wheeled_clicked(GtkWidget *widget, gpointer data) {
	// cast id of robot
	gint id = GPOINTER_TO_INT(data);

	// scan through robots to find id
	robots_t tmp = g_robots;
	while (tmp && tmp->id != id)
		tmp = tmp->next;

	// if the robot is found, change its wheeled state
	if (tmp) tmp->wheeled = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget));
}

/*
 * When robot a robot is removed from the list
 */
G_MODULE_EXPORT void refreshRobotList();
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
}

/*
 * Refresh the table of robot data
 */
G_MODULE_EXPORT void refreshRobotList() {
	// new table of robots
	static GtkWidget *rootTable = NULL;
	if(rootTable != NULL)
		gtk_widget_destroy(rootTable);
	rootTable = gtk_table_new(g_num, 11, FALSE);

	// fill table with widgets
	GtkWidget *w;
	GtkAdjustment *x_adj, *y_adj, *phi_adj;
	robots_t tmp = g_robots;
	int i = 0;
	while (tmp) {
		// label for robot with id
		char label[11];
		sprintf(label, "Robot %2d: ", tmp->id);
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
		gtk_combo_box_set_active(GTK_COMBO_BOX(w), 0);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 2, 3, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "changed", G_CALLBACK(on_type_changed), (void *)(tmp->id));
		// x position
		w = gtk_label_new(" X:");
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 3, 4, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		x_adj = GTK_ADJUSTMENT(gtk_adjustment_new(tmp->x, -500, 500, 0.1, 0.1, 1));
		w = gtk_spin_button_new(x_adj, 0.0, 1);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 4, 5, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "value-changed", G_CALLBACK(on_x_value_changed), (void *)(tmp->id));
		// y position
		w = gtk_label_new(" Y:");
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 5, 6, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		y_adj = GTK_ADJUSTMENT(gtk_adjustment_new(tmp->y, -500, 500, 0.1, 0.1, 1));
		w = gtk_spin_button_new(y_adj, 0.0, 1);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 6, 7, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "value-changed", G_CALLBACK(on_y_value_changed), (void *)(tmp->id));
		// phi angle
		w = gtk_label_new(" Angle:");
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 7, 8, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		phi_adj = GTK_ADJUSTMENT(gtk_adjustment_new(tmp->phi, -180, 180, 1, 1, 1));
		w = gtk_spin_button_new(phi_adj, 0.0, 1);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 8, 9, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "value-changed", G_CALLBACK(on_phi_value_changed), (void *)(tmp->id));
		// wheeled
		w = gtk_check_button_new_with_label("Wheeled");
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(w), 1);
		gtk_widget_show(w);
		gtk_table_attach(GTK_TABLE(rootTable), w, 9, 10, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
		g_signal_connect(G_OBJECT(w), "clicked", G_CALLBACK(on_wheeled_clicked), (void *)(tmp->id));
		if (g_num != 1) {
			// remove button
			w = gtk_button_new_with_label("Remove");
			gtk_widget_show(w);
			gtk_table_attach(GTK_TABLE(rootTable), w, 10, 11, i*3, (i*3)+2, GTK_FILL, GTK_FILL, 2, 2);
			g_signal_connect(G_OBJECT(w), "clicked", G_CALLBACK(on_button_remove_clicked), (void *)(tmp->id));
		}
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
 * When a robot is added to the list
 */
G_MODULE_EXPORT void on_button_add_robot_clicked(GtkWidget *widget, gpointer data) {
	// pointer to linked list
	robots_t tmp = g_robots;

	// new robot
	robots_t nr = new struct robots_s;
	while (tmp->next)
		tmp = tmp->next;
	nr->id = tmp->id + 1;
	nr->type = 0;
	nr->x = tmp->x + 6;
	nr->y = 0;
	nr->z = 0;
	nr->psi = 0;
	nr->theta = 0;
	nr->phi = 0;
	nr->wheeled = true;
	nr->next = NULL;

	// add robot to linked list
	tmp = g_robots;
	while (tmp->next)
		tmp = tmp->next;
	tmp->next = nr;

	// increase total number of robots
	g_num++;

	// refresh gui list with new robot
	refreshRobotList();

	// reset toggle buttons
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);
}

/*
 * When the explorer is selected
 */
G_MODULE_EXPORT void on_explorer_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);
		
		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/explorer.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 225, 150, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);
	}
}

/*
 * When the inchworm is selected
 */
G_MODULE_EXPORT void on_inchworm_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);
		
		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/inchworm.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 225, 150, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);
	}
}

/*
 * When the lift is selected
 */
G_MODULE_EXPORT void on_lift_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);
		
		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/lift.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 225, 150, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);
	}
}

/*
 * When the omnidrive is selected
 */
G_MODULE_EXPORT void on_omnidrive_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);
		
		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/omnidrive.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 225, 150, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);
	}
}

/*
 * When the snake is selected
 */
G_MODULE_EXPORT void on_snake_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "stand")), 0);
		
		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/snake.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 225, 150, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);
	}
}

/*
 * When the stand is selected
 */
G_MODULE_EXPORT void on_stand_toggled(GtkWidget *widget, gpointer data) {
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
		// uncheck other buttons
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")), 0);
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "snake")), 0);
		
		// change image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		GdkPixbuf *original = gdk_pixbuf_new_from_file("images/stand.jpg", NULL);
		GdkPixbuf *scaled = gdk_pixbuf_scale_simple(original, 225, 150, GDK_INTERP_HYPER);
		gtk_image_set_from_pixbuf(image, scaled);
	}
	else {
		// clear image
		GtkImage *image = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_shapes"));
		gtk_image_clear(image);
	}
}

/*
 * When the save button is clicked
 */
G_MODULE_EXPORT void on_save_clicked(GtkWidget *widget, gpointer data) {
	// clean out sim node
	tinyxml2::XMLElement *sim = g_doc.FirstChildElement("sim");
	sim->DeleteChildren();

	// first check if preconfigured motions are selected, then default to individual robots
	if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "explorer")))) {
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
	else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "inchworm")))) {
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

		// insert bridge 1
		tinyxml2::XMLElement *bridge2 = g_doc.NewElement("bridge");
		sim->InsertAfterChild(bridge1, bridge2);
		tinyxml2::XMLElement *b2side1 = g_doc.NewElement("side");
		b2side1->SetAttribute("id", 1);
		b2side1->SetAttribute("robot", 0);
		b2side1->SetAttribute("face", 1);
		bridge2->InsertFirstChild(b2side1);
		tinyxml2::XMLElement *b2side2 = g_doc.NewElement("side");
		b2side2->SetAttribute("id", 2);
		b2side2->SetAttribute("robot", 1);
		b2side2->SetAttribute("face", 3);
		bridge2->InsertAfterChild(b2side1, b2side2);
	}
	else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "lift")))) {
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
		b1side1->SetAttribute("face", 3);
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
		b2side1->SetAttribute("face", 1);
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
		b3side1->SetAttribute("face", 3);
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
		b4side1->SetAttribute("face", 1);
		bridge4->InsertFirstChild(b4side1);
		tinyxml2::XMLElement *b4side2 = g_doc.NewElement("side");
		b4side2->SetAttribute("id", 2);
		b4side2->SetAttribute("robot", 3);
		b4side2->SetAttribute("face", 3);
		bridge4->InsertAfterChild(b4side1, b4side2);
	}
	else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(gtk_builder_get_object(g_builder, "omnidrive")))) {
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
		rot->SetAttribute("psi", -90);
		rot->SetAttribute("theta", 0);
		rot->SetAttribute("phi", 0);
		robot1->InsertAfterChild(pos, rot);
		// insert robot1
		sim->InsertFirstChild(robot1);

		// add remaining robots
		robot2 = g_doc.NewElement("linkbotl");
		robot2->SetAttribute("id", 1);
		sim->InsertAfterChild(robot1, robot2);
		robot3 = g_doc.NewElement("linkbotl");
		robot3->SetAttribute("id", 2);
		sim->InsertAfterChild(robot2, robot3);
		robot4 = g_doc.NewElement("linkbotl");
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
		tinyxml2::XMLElement *wheel1 = g_doc.NewElement("smallwheel");
		wheel1->SetAttribute("robot", 0);
		wheel1->SetAttribute("face", 1);
		sim->InsertAfterChild(omni, wheel1);
		tinyxml2::XMLElement *wheel2 = g_doc.NewElement("smallwheel");
		wheel2->SetAttribute("robot", 1);
		wheel2->SetAttribute("face", 3);
		sim->InsertAfterChild(wheel1, wheel2);
		tinyxml2::XMLElement *wheel3 = g_doc.NewElement("smallwheel");
		wheel3->SetAttribute("robot", 2);
		wheel3->SetAttribute("face", 3);
		sim->InsertAfterChild(wheel2, wheel3);
		tinyxml2::XMLElement *wheel4 = g_doc.NewElement("smallwheel");
		wheel4->SetAttribute("robot", 3);
		wheel4->SetAttribute("face", 1);
		sim->InsertAfterChild(wheel3, wheel4);
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
			pos->SetAttribute("x", I2M(tmp->x));
			pos->SetAttribute("y", I2M(tmp->y));
			pos->SetAttribute("z", I2M(tmp->z));
			robot->InsertFirstChild(pos);

			// set rotation
			tinyxml2::XMLElement *rot = g_doc.NewElement("rotation");
			rot->SetAttribute("psi", tmp->psi);
			rot->SetAttribute("theta", tmp->theta);
			rot->SetAttribute("phi", tmp->phi);
			robot->InsertAfterChild(pos, rot);

			if (tmp->wheeled) {
				tinyxml2::XMLElement 	*wheel1 = g_doc.NewElement("smallwheel"), 
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

			tmp = tmp->next;
		}
	}
	g_doc.SaveFile(g_xml);
}

#ifdef __cplusplus
}
#endif

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE hinst, HINSTANCE hinstPrev, LPSTR lpCmdLine, int nCmdShow) {
	// init gtk
	gtk_init(NULL, NULL);
#else
int main(int argc, char *argv[]) {
	// init gtk
	gtk_init(&argc, &argv);
#endif

	// load gtk window
	g_builder = gtk_builder_new();
	gtk_builder_add_from_file(g_builder, "interface.glade", NULL);
	g_window = GTK_WIDGET(gtk_builder_get_object(g_builder, "window1"));
	if (g_window == NULL) { fprintf(stderr, "Unable to file object with id \"window1\" \n"); exit(1); }
	gtk_builder_connect_signals(g_builder, NULL);

	// load linkbot coordinates picture
	GtkImage *image_l = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_linkbot"));
	GdkPixbuf *original_l = gdk_pixbuf_new_from_file("images/linkbot.jpg", NULL);
	GdkPixbuf *scaled_l = gdk_pixbuf_scale_simple(original_l, 300, 116, GDK_INTERP_HYPER);
	gtk_image_set_from_pixbuf(image_l, scaled_l);

	// load mobot coordinates picture
	GtkImage *image_m = GTK_IMAGE(gtk_builder_get_object(g_builder, "image_mobot"));
	GdkPixbuf *original_m = gdk_pixbuf_new_from_file("images/mobot.jpg", NULL);
	GdkPixbuf *scaled_m = gdk_pixbuf_scale_simple(original_m, 300, 116, GDK_INTERP_HYPER);
	gtk_image_set_from_pixbuf(image_m, scaled_m);

	// add first robot
	robots_t nr = new struct robots_s;
	nr->id = 0;
	nr->type = 0;
	nr->x = 0;
	nr->y = 0;
	nr->z = 0;
	nr->psi = 0;
	nr->theta = 0;
	nr->phi = 0;
	nr->wheeled = true;
	nr->next = NULL;
	g_robots = nr;
	g_num = 1;
	refreshRobotList();

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

	// save xml config file
	g_doc.SaveFile(g_xml);

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
