#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include "common/getopt.h"

#include "imagesource/image_u8x3.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

typedef struct state state_t;
struct state {
    char     *url; // image_source url
    image_source_t *isrc;
    int fidx;

    getopt_t *gopt;
    pthread_t runthread;
    
    GtkWidget *window;
    GtkWidget *image;

    pthread_mutex_t mutex;
};


void
my_gdkpixbufdestroy (guchar *pixels, gpointer data)
{
    free (pixels);
}


void
change_feature(int feature_index, double change, image_source_t* isrc)
{
    double value = isrc->get_feature_value(isrc, feature_index);
    isrc->set_feature_value(isrc, feature_index, value + change);
    printf("%s:%f\n", isrc->get_feature_name(isrc, feature_index), isrc->get_feature_value(isrc, feature_index));
}


gint
callback_func (GtkWidget *widget, GdkEventKey *event, gpointer callback_data)
{
    // Definition of the feature being adjusted by a given key
    
    const int WHITEBALANCE_RV_INDEX = 6;
    const int WHITEBALANCE_UB_INDEX = 5;
    const int BRIGHTNESS_INDEX = 1;
    const int GAIN_INDEX = 12;
    
    // r and f control the red portion of white balance
    const int R_FEATURE = WHITEBALANCE_RV_INDEX;   
    const int F_FEATURE = WHITEBALANCE_RV_INDEX;
    
    const double R_CHANGE = 5;
    const double F_CHANGE = -5;
    
    // t and g control the blue portion of white balance
    const int T_FEATURE = WHITEBALANCE_UB_INDEX;
    const int G_FEATURE = WHITEBALANCE_UB_INDEX;
    
    const double T_CHANGE = 5;
    const double G_CHANGE = -5;
    
    // w and s control the gain
    const int W_FEATURE = GAIN_INDEX;
    const int S_FEATURE = GAIN_INDEX;
    
    const double W_CHANGE = 0.05;
    const double S_CHANGE = -0.05;
    
    // e and d control camera brightness
    const int E_FEATURE = BRIGHTNESS_INDEX;
    const int D_FEATURE = BRIGHTNESS_INDEX;
    
    const double E_CHANGE = 0.1;
    const double D_CHANGE = -0.1;
    
    
    state_t *state = (state_t*) callback_data;
    image_source_t *isrc = state->isrc;
    
    switch (event->keyval) {
        case GDK_KEY_Tab: {
            printf("tab\n");
            state->fidx = (state->fidx + 1) % isrc->num_formats(isrc);
            pthread_mutex_lock(&state->mutex);
            isrc->stop(isrc);
            isrc->set_format(isrc, state->fidx);
            printf("set format %d\n", state->fidx);
            isrc->start(isrc);
            pthread_mutex_unlock(&state->mutex);
            break;
        }

        case GDK_KEY_r:
        case GDK_KEY_R:
            change_feature(R_FEATURE, R_CHANGE, isrc);
            break;
        
        case GDK_KEY_f:
        case GDK_KEY_F:
            change_feature(F_FEATURE, F_CHANGE, isrc);
            break;
            
        case GDK_KEY_t:
        case GDK_KEY_T:
            change_feature(T_FEATURE, T_CHANGE, isrc);
            break;
            
        case GDK_KEY_g:
        case GDK_KEY_G:
            change_feature(G_FEATURE, G_CHANGE, isrc);
            break;
            
        case GDK_KEY_w:
        case GDK_KEY_W:
            change_feature(W_FEATURE, W_CHANGE, isrc);
            break;
            
        case GDK_KEY_s:
        case GDK_KEY_S:
            change_feature(S_FEATURE, S_CHANGE, isrc);
            break;
            
        case GDK_KEY_e:
        case GDK_KEY_E:
            change_feature(E_FEATURE, E_CHANGE, isrc);
            break;
            
        case GDK_KEY_d:
        case GDK_KEY_D:
            change_feature(D_FEATURE, D_CHANGE, isrc);
            break;
    }
    
    return 0;
}

void *
runthread (void *_p)
{
    state_t *state = (state_t*) _p;
    image_source_t *isrc = state->isrc;

    while (1) {

        image_source_data_t isdata;
        image_u8x3_t *im = NULL;

        pthread_mutex_lock(&state->mutex);
        int res = isrc->get_frame(isrc, &isdata);
        if (!res) {
            im = image_convert_u8x3(&isdata);
        }

        isrc->release_frame(isrc, &isdata);
        pthread_mutex_unlock(&state->mutex);

        if (res)
            goto error;

        if (im != NULL) {
            gdk_threads_enter();

            GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data((guchar*) im->buf,
                                                         GDK_COLORSPACE_RGB, 0, 8,
                                                         im->width, im->height, im->stride,
                                                         my_gdkpixbufdestroy,
                                                         NULL);


            gtk_image_set_from_pixbuf(GTK_IMAGE(state->image), pixbuf);
            g_object_unref(G_OBJECT(pixbuf));

            gdk_threads_leave();


        }

        usleep(0);
    }

  error:
    isrc->stop(isrc);
    printf("exiting\n");

    return NULL;
}


int
main (int argc, char *argv[] )
{
    state_t* state = calloc(1, sizeof(state_t));
    state->gopt = getopt_create();

    getopt_add_bool(state->gopt, 'h', "--help", 0, "Show this help");

    if (!getopt_parse(state->gopt, argc, argv, 0)) {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    const zarray_t *args = getopt_get_extra_args(state->gopt);
    if (zarray_size(args) > 0) {
        zarray_get(args, 0, &state->url);
    } else {
        zarray_t *urls = image_source_enumerate();

        printf("Cameras:\n");
        for (int i = 0; i < zarray_size(urls); i++) {
            char *url;
            zarray_get(urls, i, &url);
            printf("  %3d: %s\n", i, url);
        }

        if (zarray_size(urls) == 0) {
            printf("No cameras found.\n");
            exit(0);
        }
        zarray_get(urls, 0, &state->url);
    }

    g_type_init();
    gtk_init (&argc, &argv);
    gdk_threads_init();

    state->window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(state->window), state->url);

    pthread_mutex_init(&state->mutex, NULL);
    state->image = gtk_image_new();

    gtk_container_add(GTK_CONTAINER(state->window), state->image);
    gtk_widget_show(state->image);
    gtk_widget_show(state->window);

    g_signal_connect(state->window, "key_press_event",
                     G_CALLBACK(callback_func), state);

    //////////////////////////////////////////////////////////
    state->isrc = image_source_open(state->url);
    if (state->isrc == NULL) {
        printf("Unable to open device %s\n", state->url);
        exit(-1);
    }

    image_source_t *isrc = state->isrc;

    if (isrc->start(isrc))
        exit(-1);

    state->fidx = isrc->get_current_format(isrc);

    printf("Image source formats:\n");
    for (int i = 0; i < isrc->num_formats(isrc); i++) {
        image_source_format_t ifmt;
        isrc->get_format(isrc, i, &ifmt);
        printf("\t%d\t%4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
    }

    printf("Image source features:\n");
    for (int i = 0; i < isrc->num_features(isrc); i++) {
        const char *feature_name = isrc->get_feature_name(isrc, i);
        char *feature_type = isrc->get_feature_type(isrc, i);
        double v = isrc->get_feature_value(isrc, i);

        printf("\t%i: %-20s %10f     %s\n", i, feature_name, v, feature_type);
        free(feature_type);
    }
    
    // Set the features we are adjusting to manual mode
    isrc->set_feature_value(isrc, 0, 1); // brightness
    isrc->set_feature_value(isrc, 11, 2); // gain

    pthread_create(&state->runthread, NULL, runthread, state);
    
    gtk_main();

    return 0;
}
