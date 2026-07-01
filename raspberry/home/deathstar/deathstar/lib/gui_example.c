#define _GNU_SOURCE
#include <gtk/gtk.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>

#include "esc.h"
#include "hall.h"
#include "gpio_outputs.h"

// ring buffer for one sensor's history 
// 20 s @ 10 Hz = 200 samples
#define HIST_LEN  200

typedef struct {
    double instant[HIST_LEN];
    double avg [HIST_LEN];
    int head;   
    int count; 
} SensorHist;

static void hist_push(SensorHist *h, double inst, double av)
{
    h->instant[h->head] = inst;
    h->avg [h->head] = av;
    h->head = (h->head + 1) % HIST_LEN;
    if (h->count < HIST_LEN) h->count++;
}

// shared state (GUI thread writes via GTK idle, data thread reads)
static SensorHist g_hist[2];
static GMutex g_hist_mutex;

static int g_avg_n = HALL_PULSES_PER_REV; //settable
static double g_rpm_inst[2] = {0, 0};
static double g_rpm_avg [2] = {0, 0};

// background data thread  calls hall at 10 Hz, pushes history
static volatile int g_running = 1;
static pthread_t g_data_thread;

static void *data_thread_fn(void *arg)
{
    (void)arg;
    while (g_running) {
        double inst0 = hall_get_rpm_instant(0);
        double inst1 = hall_get_rpm_instant(1);
        double avg0 = hall_get_rpm_avg(0, g_avg_n);
        double avg1 = hall_get_rpm_avg(1, g_avg_n);

        g_mutex_lock(&g_hist_mutex);
        g_rpm_inst[0] = inst0; g_rpm_inst[1] = inst1;
        g_rpm_avg [0] = avg0; g_rpm_avg [1] = avg1;
        hist_push(&g_hist[0], inst0, avg0);
        hist_push(&g_hist[1], inst1, avg1);
        g_mutex_unlock(&g_hist_mutex);

        g_usleep(100000); // 10 Hz
    }
    return NULL;
}

// widgets we need to reach from callbacks
static GtkWidget *g_label_inst[2];
static GtkWidget *g_label_avg[2];
static GtkWidget *g_draw [2];
static GtkWidget *g_err_buf_view;
static GtkTextBuffer *g_err_buf;
static GtkWidget *g_laser_btn;
static GtkWidget *g_led_btn;

static bool g_laser_on = false;
static bool g_led_on   = false;

// error log 
static void log_error(const char *msg)
{
    GtkTextIter end;
    gtk_text_buffer_get_end_iter(g_err_buf, &end);
    gtk_text_buffer_insert(g_err_buf, &end, msg, -1);
    gtk_text_buffer_insert(g_err_buf, &end, "\n", -1);
    // scroll to end
    GtkTextMark *mark = gtk_text_buffer_get_insert(g_err_buf);
    gtk_text_view_scroll_mark_onscreen(GTK_TEXT_VIEW(g_err_buf_view), mark);
}

// GTK periodic update - at 10 Hz on the main thread
static gboolean on_tick(gpointer user_data)
{
    (void)user_data;
    if (!g_running) return G_SOURCE_REMOVE;

    g_mutex_lock(&g_hist_mutex);
    double inst0 = g_rpm_inst[0], inst1 = g_rpm_inst[1];
    double avg0  = g_rpm_avg [0], avg1  = g_rpm_avg [1];
    g_mutex_unlock(&g_hist_mutex);

    char buf[64];
    snprintf(buf, sizeof(buf), "%.0f rpm", inst0);
    gtk_label_set_text(GTK_LABEL(g_label_inst[0]), buf);
    snprintf(buf, sizeof(buf), "%.0f rpm", avg0);
    gtk_label_set_text(GTK_LABEL(g_label_avg[0]),  buf);
    snprintf(buf, sizeof(buf), "%.0f rpm", inst1);
    gtk_label_set_text(GTK_LABEL(g_label_inst[1]), buf);
    snprintf(buf, sizeof(buf), "%.0f rpm", avg1);
    gtk_label_set_text(GTK_LABEL(g_label_avg[1]),  buf);

    gtk_widget_queue_draw(g_draw[0]);
    gtk_widget_queue_draw(g_draw[1]);

    return G_SOURCE_CONTINUE;
}

//  plot draw callback
static void draw_plot(GtkDrawingArea *area, cairo_t *cr,
                      int w, int h, gpointer sensor_ptr)
{
    int sensor = GPOINTER_TO_INT(sensor_ptr);

    // background
    cairo_set_source_rgb(cr, 0.98, 0.98, 0.98);
    cairo_paint(cr);

    g_mutex_lock(&g_hist_mutex);
    SensorHist snap = g_hist[sensor]; // copy under lock
    g_mutex_unlock(&g_hist_mutex);

    if (snap.count < 2) return;

    // find y range
    double ymin = 1e9, ymax = -1e9;
    for (int i = 0; i < snap.count; i++) {
        if (snap.instant[i] < ymin) ymin = snap.instant[i];
        if (snap.instant[i] > ymax) ymax = snap.instant[i];
        if (snap.avg[i] < ymin) ymin = snap.avg[i];
        if (snap.avg[i] > ymax) ymax = snap.avg[i];
    }
    if (ymax - ymin < 100) { ymin -= 50; ymax += 50; }
    else { ymin *= 0.95; ymax *= 1.05; }

    const int PAD_L = 42, PAD_R = 8, PAD_T = 6, PAD_B = 18;
    double pw = w - PAD_L - PAD_R;
    double ph = h - PAD_T - PAD_B;

    // y-axis labels
    cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
    cairo_set_font_size(cr, 9);
    for (int i = 0; i <= 4; i++) {
        double v = ymin + (ymax - ymin) * i / 4.0;
        double y = PAD_T + ph - ph * i / 4.0;
        // gridline
        cairo_set_source_rgba(cr, 0.5, 0.5, 0.5, 0.15);
        cairo_move_to(cr, PAD_L, y);
        cairo_line_to(cr, PAD_L + pw, y);
        cairo_stroke(cr);
        // label
        char lbl[16]; snprintf(lbl, sizeof(lbl), "%.0f", v);
        cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
        cairo_move_to(cr, 2, y + 3);
        cairo_show_text(cr, lbl);
    }

    // helper: index => canvas x,y
    #define IX(i) (PAD_L + pw * (i) / (double)(snap.count - 1))
    #define IY(v) (PAD_T + ph - ph * ((v) - ymin) / (ymax - ymin))

    // read out in ring-buffer order
    #define SAMPLE_I(k) snap.instant[( snap.head - snap.count + (k) + HIST_LEN) % HIST_LEN]
    #define SAMPLE_A(k) snap.avg[( snap.head - snap.count + (k) + HIST_LEN) % HIST_LEN]

    // avg line (lighter)
    double r, g_c, b;
    if (sensor == 0) { r=0.52; g_c=0.71; b=0.87; }
    else { r=0.93; g_c=0.58; b=0.69; }
    cairo_set_source_rgb(cr, r, g_c, b);
    cairo_set_line_width(cr, 1.5);
    cairo_move_to(cr, IX(0), IY(SAMPLE_A(0)));
    for (int k = 1; k < snap.count; k++)
        cairo_line_to(cr, IX(k), IY(SAMPLE_A(k)));
    cairo_stroke(cr);

    // instant line (darker)
    if (sensor == 0) { r=0.22; g_c=0.54; b=0.87; }
    else { r=0.83; g_c=0.33; b=0.49; }
    cairo_set_source_rgb(cr, r, g_c, b);
    cairo_set_line_width(cr, 1.5);
    cairo_move_to(cr, IX(0), IY(SAMPLE_I(0)));
    for (int k = 1; k < snap.count; k++)
        cairo_line_to(cr, IX(k), IY(SAMPLE_I(k)));
    cairo_stroke(cr);

    #undef IX
    #undef IY
    #undef SAMPLE_I
    #undef SAMPLE_A
}

// Shutdown
static void do_shutdown(void)
{
    g_running = 0;
    pthread_join(g_data_thread, NULL);
    esc_set_throttle_both(0.0, 0.0);
    hall_cleanup();
    esc_cleanup();
    gpio_outputs_cleanup();
}

static void sig_handler(int sig)
{
    (void)sig;
    do_shutdown();
    exit(0);
}

// callbacks
static void on_quit(GtkWidget *btn, gpointer win)
{
    (void)btn;
    do_shutdown();
    gtk_window_destroy(GTK_WINDOW(win));
}

static void on_t1_changed(GtkRange *r, gpointer entry)
{
    double v = gtk_range_get_value(r);
    char buf[16]; snprintf(buf, sizeof(buf), "%.0f", v);
    gtk_editable_set_text(GTK_EDITABLE(entry), buf);
    esc_set_throttle_1(v);
}
static void on_t2_changed(GtkRange *r, gpointer entry)
{
    double v = gtk_range_get_value(r);
    char buf[16]; snprintf(buf, sizeof(buf), "%.0f", v);
    gtk_editable_set_text(GTK_EDITABLE(entry), buf);
    esc_set_throttle_2(v);
}

static void on_t1_entry(GtkEntry *e, gpointer slider)
{
    double v = atof(gtk_editable_get_text(GTK_EDITABLE(e)));
    v = CLAMP(v, -100.0, 100.0);
    gtk_range_set_value(GTK_RANGE(slider), v);
    esc_set_throttle_1(v);
}
static void on_t2_entry(GtkEntry *e, gpointer slider)
{
    double v = atof(gtk_editable_get_text(GTK_EDITABLE(e)));
    v = CLAMP(v, -100.0, 100.0);
    gtk_range_set_value(GTK_RANGE(slider), v);
    esc_set_throttle_2(v);
}

static void on_stop1(GtkButton *b, gpointer slider)
{
    (void)b; gtk_range_set_value(GTK_RANGE(slider), 0.0);
    esc_set_throttle_1(0.0);
}
static void on_stop2(GtkButton *b, gpointer slider)
{
    (void)b; gtk_range_set_value(GTK_RANGE(slider), 0.0);
    esc_set_throttle_2(0.0);
}

static void on_laser(GtkToggleButton *b, gpointer d)
{
    (void)d;
    g_laser_on = gtk_toggle_button_get_active(b);
    laser_set(g_laser_on);
    gtk_button_set_label(GTK_BUTTON(b), g_laser_on ? "Laser  ON" : "Laser  OFF");
}
static void on_led(GtkToggleButton *b, gpointer d)
{
    (void)d;
    g_led_on = gtk_toggle_button_get_active(b);
    led_set(g_led_on);
    gtk_button_set_label(GTK_BUTTON(b), g_led_on ? "LED  ON" : "LED  OFF");
}
static void on_thruster(GtkToggleButton *b, gpointer thr_ptr)
{
    int  thr = GPOINTER_TO_INT(thr_ptr);
    bool on  = gtk_toggle_button_get_active(b);
    thruster_set(thr, on);
    char lbl[16]; snprintf(lbl, sizeof(lbl), "Thr %d  %s", thr, on ? "ON" : "OFF");
    gtk_button_set_label(GTK_BUTTON(b), lbl);
}

static void on_avg_apply(GtkButton *btn, gpointer entry)
{
    (void)btn;
    int n = atoi(gtk_editable_get_text(GTK_EDITABLE(entry)));
    if (n < 1)  n = 1;
    if (n > 64) n = 64;
    g_avg_n = n;
}

// Build a throttle box (shared pattern for ESC1 / ESC2)
static GtkWidget *make_throttle_box(const char *label_txt,
                                    GCallback on_slider,
                                    GCallback on_entry_cb,
                                    GCallback on_stop,
                                    GtkWidget **out_slider)
{
    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);

    GtkWidget *lbl = gtk_label_new(label_txt);
    gtk_label_set_xalign(GTK_LABEL(lbl), 0);
    gtk_box_append(GTK_BOX(box), lbl);

    GtkWidget *slider = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL,
                                                  -100.0, 100.0, 1.0);
    gtk_range_set_value(GTK_RANGE(slider), 0.0);
    gtk_scale_set_draw_value(GTK_SCALE(slider), FALSE);
    gtk_box_append(GTK_BOX(box), slider);
    *out_slider = slider;

    GtkWidget *row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *entry = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(entry), 6);
    gtk_editable_set_text(GTK_EDITABLE(entry), "0");
    gtk_widget_set_size_request(entry, 64, -1);

    GtkWidget *stop = gtk_button_new_with_label("Stop (0)");
    gtk_box_append(GTK_BOX(row), gtk_label_new("value:"));
    gtk_box_append(GTK_BOX(row),entry);
    gtk_box_append(GTK_BOX(row),stop);
    gtk_box_append(GTK_BOX(box),row);

    g_signal_connect(slider,"value-changed", on_slider, entry);
    g_signal_connect(entry,"activate", on_entry_cb, slider);
    g_signal_connect(stop,"clicked", on_stop, slider);

    return box;
}

// Build the left control panel
static GtkWidget *build_left_panel(GtkWidget *window)
{
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_widget_set_margin_start(vbox, 10);
    gtk_widget_set_margin_end(vbox, 10);
    gtk_widget_set_margin_top(vbox, 10);
    gtk_widget_set_margin_bottom(vbox, 10);

    // ESC 1 throttle
    GtkWidget *s1;
    GtkWidget *t1 = make_throttle_box("ESC 1 - throttle",
        G_CALLBACK(on_t1_changed), G_CALLBACK(on_t1_entry),
        G_CALLBACK(on_stop1), &s1);
    gtk_box_append(GTK_BOX(vbox), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL));
    gtk_box_append(GTK_BOX(vbox), t1);

    // ESC 2 throttle
    GtkWidget *s2;
    GtkWidget *t2 = make_throttle_box("ESC 2 - throttle",
        G_CALLBACK(on_t2_changed), G_CALLBACK(on_t2_entry),
        G_CALLBACK(on_stop2), &s2);
    gtk_box_append(GTK_BOX(vbox), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL));
    gtk_box_append(GTK_BOX(vbox), t2);

    // Outputs (laser / LED)
    gtk_box_append(GTK_BOX(vbox), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL));
    GtkWidget *out_lbl = gtk_label_new("Outputs");
    gtk_label_set_xalign(GTK_LABEL(out_lbl), 0);
    gtk_box_append(GTK_BOX(vbox), out_lbl);

    GtkWidget *out_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    g_laser_btn = gtk_toggle_button_new_with_label("Laser  OFF");
    g_led_btn = gtk_toggle_button_new_with_label("LED  OFF");
    gtk_widget_set_hexpand(g_laser_btn, TRUE);
    gtk_widget_set_hexpand(g_led_btn, TRUE);
    g_signal_connect(g_laser_btn, "toggled", G_CALLBACK(on_laser), NULL);
    g_signal_connect(g_led_btn, "toggled", G_CALLBACK(on_led),   NULL);
    gtk_box_append(GTK_BOX(out_row), g_laser_btn);
    gtk_box_append(GTK_BOX(out_row), g_led_btn);
    gtk_box_append(GTK_BOX(vbox), out_row);
    
    // Thrusters
    gtk_box_append(GTK_BOX(vbox), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL));
    GtkWidget *thr_lbl = gtk_label_new("Thrusters");
    gtk_label_set_xalign(GTK_LABEL(thr_lbl), 0);
    gtk_box_append(GTK_BOX(vbox), thr_lbl);

    GtkWidget *thr_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    for (int i = 1; i <= 4; i++) {
        char lbl[16]; snprintf(lbl, sizeof(lbl), "Thr %d  OFF", i);
        GtkWidget *btn = gtk_toggle_button_new_with_label(lbl);
        gtk_widget_set_hexpand(btn, TRUE);
        g_signal_connect_data(btn, "toggled",
            G_CALLBACK(on_thruster), GINT_TO_POINTER(i), NULL, 0);
        gtk_box_append(GTK_BOX(thr_row), btn);
    }
    gtk_box_append(GTK_BOX(vbox), thr_row);

    // Avg window
    gtk_box_append(GTK_BOX(vbox), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL));
    GtkWidget *avg_lbl = gtk_label_new("RPM avg window (pulses)");
    gtk_label_set_xalign(GTK_LABEL(avg_lbl), 0);
    gtk_box_append(GTK_BOX(vbox), avg_lbl);

    GtkWidget *avg_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    GtkWidget *avg_entry = gtk_entry_new();
    char avg_def[8]; snprintf(avg_def, sizeof(avg_def), "%d", g_avg_n);
    gtk_editable_set_text(GTK_EDITABLE(avg_entry), avg_def);
    gtk_widget_set_size_request(avg_entry, 56, -1);
    GtkWidget *avg_apply = gtk_button_new_with_label("Apply");
    g_signal_connect(avg_apply, "clicked", G_CALLBACK(on_avg_apply), avg_entry);
    gtk_box_append(GTK_BOX(avg_row), avg_entry);
    gtk_box_append(GTK_BOX(avg_row), avg_apply);
    gtk_box_append(GTK_BOX(vbox), avg_row);

    // Error log
    gtk_box_append(GTK_BOX(vbox), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL));
    GtkWidget *err_lbl = gtk_label_new("Error log");
    gtk_label_set_xalign(GTK_LABEL(err_lbl), 0);
    gtk_box_append(GTK_BOX(vbox), err_lbl);

    g_err_buf = gtk_text_buffer_new(NULL);
    g_err_buf_view = gtk_text_view_new_with_buffer(g_err_buf);
    gtk_text_view_set_editable(GTK_TEXT_VIEW(g_err_buf_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(g_err_buf_view), GTK_WRAP_WORD_CHAR);
    gtk_widget_set_vexpand(g_err_buf_view, TRUE);

    GtkWidget *scroll = gtk_scrolled_window_new();
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroll),
                                    GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_size_request(scroll, -1, 80);
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(scroll), g_err_buf_view);
    gtk_box_append(GTK_BOX(vbox), scroll);

    // quit
    gtk_box_append(GTK_BOX(vbox), gtk_separator_new(GTK_ORIENTATION_HORIZONTAL));
    GtkWidget *quit = gtk_button_new_with_label("Stop motors & quit");
    g_signal_connect(quit, "clicked", G_CALLBACK(on_quit), window);

    GtkCssProvider *css = gtk_css_provider_new();
    gtk_css_provider_load_from_data(css,
    "button.quit { background: #a32d2d; color: white; font-weight: bold; }", -1);
    gtk_widget_add_css_class(quit, "quit");
    gtk_style_context_add_provider_for_display(
        gdk_display_get_default(),
        GTK_STYLE_PROVIDER(css),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    gtk_box_append(GTK_BOX(vbox), quit);

    return vbox;
}

// build one sensor panel (drawing area + numeric labels)
static GtkWidget *build_sensor_panel(int sensor)
{
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_margin_start (vbox, 6);
    gtk_widget_set_margin_end (vbox, 6);
    gtk_widget_set_vexpand(vbox, TRUE);

    char title[32];
    snprintf(title, sizeof(title), "Hall sensor %d - RPM", sensor + 1);
    GtkWidget *lbl = gtk_label_new(title);
    gtk_label_set_xalign(GTK_LABEL(lbl), 0);
    gtk_box_append(GTK_BOX(vbox), lbl);

    // numeric row
    GtkWidget *num_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);

    g_label_inst[sensor] = gtk_label_new("- rpm");
    g_label_avg [sensor] = gtk_label_new("- rpm");
    GtkWidget *i_lbl = gtk_label_new("instant:");
    GtkWidget *a_lbl = gtk_label_new("avg:");
    gtk_box_append(GTK_BOX(num_row), i_lbl);
    gtk_box_append(GTK_BOX(num_row), g_label_inst[sensor]);
    gtk_box_append(GTK_BOX(num_row), a_lbl);
    gtk_box_append(GTK_BOX(num_row), g_label_avg[sensor]);
    gtk_box_append(GTK_BOX(vbox), num_row);

    // drawing area
    g_draw[sensor] = gtk_drawing_area_new();
    gtk_widget_set_vexpand(g_draw[sensor], TRUE);
    gtk_drawing_area_set_draw_func(
        GTK_DRAWING_AREA(g_draw[sensor]),
        draw_plot,
        GINT_TO_POINTER(sensor),
        NULL);
    gtk_box_append(GTK_BOX(vbox), g_draw[sensor]);

    return vbox;
}

// Activate
static void activate(GtkApplication *app, gpointer user_data)
{
    (void)user_data;

    GtkWidget *window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(window), "GUI example");
    gtk_window_set_default_size(GTK_WINDOW(window), 1100, 800);

    // top-level horizontal split
    GtkWidget *paned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_paned_set_position(GTK_PANED(paned), 500);

    // left: scrollable control panel
    GtkWidget *left_scroll = gtk_scrolled_window_new();
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(left_scroll),
                                    GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);
    GtkWidget *left = build_left_panel(window);
    gtk_scrolled_window_set_child(GTK_SCROLLED_WINDOW(left_scroll), left);
    gtk_paned_set_start_child(GTK_PANED(paned), left_scroll);
    gtk_paned_set_resize_start_child(GTK_PANED(paned), FALSE);

    // right: two sensor panels stacked
    GtkWidget *right_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_set_margin_top(right_box, 8);
    gtk_widget_set_margin_bottom(right_box, 8);

    gtk_box_append(GTK_BOX(right_box), build_sensor_panel(0));
    gtk_box_append(GTK_BOX(right_box),
                   gtk_separator_new(GTK_ORIENTATION_HORIZONTAL));
    gtk_box_append(GTK_BOX(right_box), build_sensor_panel(1));

    gtk_paned_set_end_child(GTK_PANED(paned), right_box);

    gtk_window_set_child(GTK_WINDOW(window), paned);
    gtk_window_present(GTK_WINDOW(window));

    // 10 Hz UI refresh
    g_timeout_add(100, on_tick, NULL);
}

// main
int main(int argc, char **argv)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    
    // init hardware
    if (!gpio_outputs_init(NULL)) {
        fprintf(stderr, "gpio_outputs_init failed\n"); return 1;
    }
    if (!esc_init(0)) {
        fprintf(stderr, "esc_init failed\n");
        gpio_outputs_cleanup(); return 1;
    }
    if (!hall_init(HALL_GPIO_1, HALL_GPIO_2)) {
        fprintf(stderr, "hall_init failed\n");
        esc_cleanup(); gpio_outputs_cleanup(); return 1;
    }

    g_mutex_init(&g_hist_mutex);
    pthread_create(&g_data_thread, NULL, data_thread_fn, NULL);

    GtkApplication *app = gtk_application_new("com.example.motorctl",
                                               G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);
    int ret = g_application_run(G_APPLICATION(app), argc, argv);
    g_object_unref(app);

    // if the user closed the window without pressing quit
    if (g_running) do_shutdown();

    g_mutex_clear(&g_hist_mutex);
    return ret;
}
