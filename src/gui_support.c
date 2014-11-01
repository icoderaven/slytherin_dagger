/**	@file gui_support.c
 *	@brief Support functions for the GUI. This file was originally generated
 *	by glade.
 *
 *	@author Cornell Wright
 */

#ifdef HAVE_CONFIG_H
#  include "gui_config.h"
#endif

#include <sys/types.h>
#include <sys/stat.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <string.h>
#include <stdio.h>

#include <gtk/gtk.h>

#include "gui_support.h"

extern GtkWidget *main_window;

GtkWidget *
lookup_widget (GtkWidget * widget, const gchar * widget_name)
{
  GtkWidget *parent, *found_widget;

  for (;;) {
    if (GTK_IS_MENU (widget))
      parent = gtk_menu_get_attach_widget (GTK_MENU (widget));
    else
      parent = widget->parent;
    if (!parent)
      parent =
	(GtkWidget *) g_object_get_data (G_OBJECT (widget), "GladeParentKey");
    if (parent == NULL)
      break;
    widget = parent;
  }

  found_widget = (GtkWidget *) g_object_get_data (G_OBJECT (widget),
						  widget_name);
  if (!found_widget)
    g_warning ("Widget not found: %s", widget_name);
  return found_widget;
}

static GList *pixmaps_directories = NULL;

/* Use this function to set the directory containing installed pixmaps. */
void
add_pixmap_directory (const gchar * directory)
{
  pixmaps_directories = g_list_prepend (pixmaps_directories,
					g_strdup (directory));
}

/* This is an internally used function to find pixmap files. */
static gchar *
find_pixmap_file (const gchar * filename)
{
  GList *elem;

  /* We step through each of the pixmaps directory to find it. */
  elem = pixmaps_directories;
  while (elem) {
    gchar *pathname = g_strdup_printf ("%s%s%s", (gchar *) elem->data,
				       G_DIR_SEPARATOR_S, filename);
    if (g_file_test (pathname, G_FILE_TEST_EXISTS))
      return pathname;
    g_free (pathname);
    elem = elem->next;
  }
  return NULL;
}

/* This is an internally used function to create pixmaps. */
GtkWidget *
create_pixmap (GtkWidget * widget, const gchar * filename)
{
  gchar *pathname = NULL;
  GtkWidget *pixmap;

  if (!filename || !filename[0])
    return gtk_image_new ();

  pathname = find_pixmap_file (filename);

  if (!pathname) {
    g_warning (_("Couldn't find pixmap file: %s"), filename);
    return gtk_image_new ();
  }

  pixmap = gtk_image_new_from_file (pathname);
  g_free (pathname);
  return pixmap;
}

/* This is an internally used function to create pixmaps. */
GdkPixbuf *
create_pixbuf (const gchar * filename)
{
  gchar *pathname = NULL;
  GdkPixbuf *pixbuf;
  GError *error = NULL;

  if (!filename || !filename[0])
    return NULL;

  pathname = find_pixmap_file (filename);

  if (!pathname) {
    g_warning (_("Couldn't find pixmap file: %s"), filename);
    return NULL;
  }

  pixbuf = gdk_pixbuf_new_from_file (pathname, &error);
  if (!pixbuf) {
    fprintf (stderr, "Failed to load pixbuf file: %s: %s\n",
	     pathname, error->message);
    g_error_free (error);
  }
  g_free (pathname);
  return pixbuf;
}

/* This is used to set ATK action descriptions. */
void
glade_set_atk_action_description (AtkAction * action,
				  const gchar * action_name,
				  const gchar * description)
{
  gint n_actions, i;

  n_actions = atk_action_get_n_actions (action);
  for (i = 0; i < n_actions; i++) {
    if (!strcmp (atk_action_get_name (action, i), action_name))
      atk_action_set_description (action, i, description);
  }
}

void update_label_text(char *label_name, char *new_text) {
	GtkLabel *label;

	label = GTK_LABEL(lookup_widget(main_window, label_name));

	gtk_label_set_text(label, new_text);
}

int get_notebook_page(char *notebook_name) {
	GtkNotebook *nb;

	nb = GTK_NOTEBOOK(lookup_widget(main_window, notebook_name));

	return gtk_notebook_get_current_page(nb);
}

void update_slider_pos(char *slide_name, float new_pos) {
	GtkAdjustment *adj;
	GtkRange *range;

	range = GTK_RANGE(lookup_widget(main_window, slide_name));
	adj = gtk_range_get_adjustment(range);

	if (gtk_adjustment_get_value(adj) != new_pos) {
		gtk_adjustment_set_value(adj, new_pos);
		g_signal_emit_by_name(G_OBJECT(adj), "changed");
		gtk_widget_hide(GTK_WIDGET(range));
		gtk_widget_show(GTK_WIDGET(range));
	}
}

void update_image_sensitive(char *image_name, gboolean sensitive) {
	GtkWidget *image;

	image = lookup_widget(main_window, image_name);

	gtk_widget_set_sensitive(image, sensitive);
}

void append_textview(char *name, char *text, int len) {
	GtkWidget *view;
	GtkTextBuffer *buf;
	GtkTextIter iter;

	view = lookup_widget(main_window, name);
	buf = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view));

	gtk_text_buffer_get_end_iter(buf, &iter);
	gtk_text_buffer_place_cursor(buf, &iter);

	gtk_text_buffer_insert_at_cursor(buf, text, len);

	gtk_text_view_scroll_to_mark(GTK_TEXT_VIEW(view), gtk_text_buffer_get_insert(buf), .45, TRUE, 0, 0.8);


}

