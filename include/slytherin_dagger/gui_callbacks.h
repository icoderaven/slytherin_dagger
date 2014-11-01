
#ifndef GUI_CALLBACKS_H
#define GUI_CALLBACKS_H


#include <gtk/gtk.h>

extern int quit_called;

gboolean on_quit (gpointer data);
void on_apply_btn_clicked (GtkButton * button, gpointer user_data);
void manual_value_changed (GtkRange * range, gpointer user_data);
gboolean on_main_notebook_change_current_page (GtkNotebook * notebook,
		GtkNotebookPage *page, guint page_num, gpointer user_data);
void
on_checkbutton2_toggled                (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

void
on_rezero1_activate                    (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

gboolean
on_inner_slide_set_pos_button_press_event
                                        (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_outer_slide_set_pos_button_press_event
                                        (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_inner_set_pos_button_press_event    (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_bot_set_pos_button_press_event      (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_ul_set_pos_button_press_event       (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_ur_set_pos_button_press_event       (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);


gboolean
on_inner_slide_set_pos_button_release_event
                                        (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_outer_slide_set_pos_button_release_event
                                        (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_inner_set_pos_button_release_event  (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_bot_set_pos_button_release_event    (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_ul_set_pos_button_release_event     (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
on_ur_set_pos_button_release_event     (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

int relative_checked(void);

float get_slider_pos(char *slider_name);


#endif /* GUI_CALLBACKS_H */

