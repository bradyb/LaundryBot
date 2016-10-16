#ifndef VXGUI_HPP
#define VXGUI_HPP

#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>
#include <iostream>
#include <string>

#include "eecs467/vx_gtk_window_base.hpp"
#include "vx/vxo_text.h"
#include "vx/vxo_pix_coords.h"
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_key_codes.h"
#include "vx/vx_colors.h"
#include "vx/vxo_image.h"
#include "common/getopt.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "math/point.hpp"

class VxGui : public eecs467::VxGtkWindowBase
{
public:
    bool settingMask;

    // Constructor
    VxGui(int argc, char** argv, std::string savePath);

    // Destructor
    ~VxGui() {}

    static void start(VxGui* gui);
    void updateImage(image_u32_t * image, int imgFlags);
    void setImageDimensions(int width, int height);
    void copyImage(image_u32_t * image);
    void storeMaskPts();
    void addMaskPoint(float x, float y, uint32_t buttonMask);
    void saveImg();
    std::vector<eecs467::Point<int> > getMask() { return pointsClicked_; }

    int onMouseEvent(vx_layer_t* layer, vx_camera_pos_t* cameraPosition, vx_mouse_event_t* event)
    {
        addMaskPoint(event->x, event->y, event->button_mask);
        return 0;
    }

    int onKeyEvent(vx_layer_t* layer, vx_key_event_t* event)
    {
        // save a pnm image of current image
        if( (event->key_code == VX_KEY_s) && (event->released == 1) )
        {
            saveImg();
        }

        // save a set of mask points 
        else if( (event->key_code == VX_KEY_ENTER) && (event->released == 1) )
        {
            storeMaskPts();
        }

        return 0;
    }

    void render(void);
private:

    // store points clicked (in rescaled coordinates)
    std::vector< eecs467::Point<int> > pointsClicked_;

    // Image info
    int imgWidth_;
    int imgHeight_;
    image_u32_t* imgPtr_;
    std::mutex imgLock_;
    

    // image scale
    float xScaleFactor_, yScaleFactor_;
    float xScaleFactorMask_, yScaleFactorMask_;

    int img_flags_;
    std::string saveDir_;

    void plotMaskPosition(vx_buffer_t*);
};


#endif // VXGUI_HPP
