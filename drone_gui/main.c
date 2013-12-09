#include <stdlib.h>
#include <gtk/gtk.h>
int init_flag = 0;

static void helloWorld (GtkWidget *wid, GtkWidget *win)
{
  GtkWidget *dialog = NULL;

  dialog = gtk_message_dialog_new (GTK_WINDOW (win), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE, "Hello World!");
  gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
  gtk_dialog_run (GTK_DIALOG (dialog));
  gtk_widget_destroy (dialog);
}
static void initalize_drone (GtkWidget *wid, GtkWidget *win)
{
    GtkWidget *dialog = NULL;
    system("roslaunch ardrone_autonomy ardrone.launch &");
    // add delay here
    dialog = gtk_message_dialog_new (GTK_WINDOW (win), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_OK, "Drone has been initialized. Wait 10 seconds!");
    gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
    gtk_dialog_run (GTK_DIALOG (dialog));
    gtk_widget_destroy (dialog);
    init_flag = 1;
}

static void run_drone (GtkWidget *wid, GtkWidget *win)
{

    GtkWidget *dialog = NULL;
    if(init_flag == 1)
    {
        system("rosrun mdb_drone tag_controller &");
        dialog = gtk_message_dialog_new (GTK_WINDOW (win), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_NONE, "tag_controller is running");
    }
    else
    {
        dialog = gtk_message_dialog_new (GTK_WINDOW (win), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_NONE, "Drone Not Initialized!");
    }
    gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
    gtk_dialog_run (GTK_DIALOG (dialog));
    gtk_widget_destroy (dialog);
}

static void emerg_land (GtkWidget *wid, GtkWidget *win)
{
  GtkWidget *dialog = NULL;

  dialog = gtk_message_dialog_new (GTK_WINDOW (win), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_NONE, "Emergency Landing called!");
  gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
  system("rosrun mdb_drone land &");
  gtk_dialog_run (GTK_DIALOG (dialog));
  gtk_widget_destroy (dialog);
}

static void camera_feed (GtkWidget *wid, GtkWidget *win)
{
  GtkWidget *dialog = NULL;

  dialog = gtk_message_dialog_new (GTK_WINDOW (win), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_NONE, "Emergency Landing called!");
  gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
  system("python /home/ardrone/ros_workspace/mdb_drone/src/drone_video_display.py");
  gtk_dialog_run (GTK_DIALOG (dialog));
  gtk_widget_destroy (dialog);
}

GdkPixbuf *load_pixbuf_from_file (const char *filename)
{
    GError *error = NULL;
    //GdkPixbuf *pixbuf = gdk_pixbuf_new_from_file_at_size(filename,256,256,&error);
    GdkPixbuf *pixbuf = gdk_pixbuf_new_from_file_at_scale(filename, 256, 256, TRUE, &error);

    if (pixbuf == NULL)
    {
        g_print ("Error loading file: %d : %s\n", error->code, error->message);
        g_error_free (error);
        exit (1);
    }
    return pixbuf;
}

int main (int argc, char *argv[])
{
    GtkWidget *button = NULL;
    GtkWidget *win = NULL;
    GtkWidget *vbox = NULL;
    GdkPixmap *background = NULL;
    GdkPixbuf *image = NULL;
    GtkStyle  *style = NULL;

    /* Initialize GTK+ */
    g_log_set_handler ("Gtk", G_LOG_LEVEL_WARNING, (GLogFunc) gtk_false, NULL);
    gtk_init (&argc, &argv);
    g_log_set_handler ("Gtk", G_LOG_LEVEL_WARNING, g_log_default_handler, NULL);

    /*Load background image */
    //image = load_pixbuf_from_file ("/home/birkiro/drone.bmp");
    image = load_pixbuf_from_file ("/home/ardrone/ros_workspace/mdb_drone/bin/drone.bmp");
    //  widget = gtk_image_new_from_pixbuf (image);
    gdk_pixbuf_render_pixmap_and_mask (image, &background, NULL, 0);
    style = gtk_style_new ();
    style->bg_pixmap [0] = background;

    /* Create the main window */
    win = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_container_set_border_width (GTK_CONTAINER (win), 100);
    gtk_window_set_title (GTK_WINDOW (win), "MDB AR.Drone GUI");
    gtk_window_set_position (GTK_WINDOW (win), GTK_WIN_POS_CENTER);
    gtk_widget_set_style (GTK_WIDGET(win), GTK_STYLE (style));
    gtk_widget_realize (win);
    g_signal_connect (win, "destroy", gtk_main_quit, NULL);

    /* Create a vertical box with buttons */
    vbox = gtk_vbox_new (TRUE, 25);
    gtk_container_add (GTK_CONTAINER (win), vbox);

    button = gtk_button_new_with_label("Initialize");
    g_signal_connect (G_OBJECT (button), "clicked", G_CALLBACK (initalize_drone), (gpointer) win);
    gtk_box_pack_start (GTK_BOX (vbox), button, TRUE, TRUE, 0);

    button = gtk_button_new_with_label("Run Algorithm");
    g_signal_connect (G_OBJECT (button), "clicked", G_CALLBACK (run_drone), (gpointer) win);
    gtk_box_pack_start (GTK_BOX (vbox), button, TRUE, TRUE, 0);

    button = gtk_button_new_with_label("Emergency Landing");
    g_signal_connect (G_OBJECT (button), "clicked", G_CALLBACK (emerg_land), (gpointer) win);
    gtk_box_pack_start (GTK_BOX (vbox), button, TRUE, TRUE, 0);

    button = gtk_button_new_with_label("View Camera Feed");
    g_signal_connect (G_OBJECT (button), "clicked", G_CALLBACK (camera_feed), (gpointer) win);
    gtk_box_pack_start (GTK_BOX (vbox), button, TRUE, TRUE, 0);

    button = gtk_button_new_from_stock (GTK_STOCK_DIALOG_INFO);
    g_signal_connect (G_OBJECT (button), "clicked", G_CALLBACK (helloWorld), (gpointer) win);
    gtk_box_pack_start (GTK_BOX (vbox), button, TRUE, TRUE, 0);

    button = gtk_button_new_from_stock (GTK_STOCK_CLOSE);
    g_signal_connect (button, "clicked", gtk_main_quit, NULL);
    gtk_box_pack_start (GTK_BOX (vbox), button, TRUE, TRUE, 0);

    /* Enter the main loop */
    gtk_widget_show_all (win);
    gtk_main ();
    return 0;
}
