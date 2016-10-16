#ifndef IMAGEMANAGER_HPP
#define IMAGEMANAGER_HPP

#include <vector>

#include "vx/vxo_image.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "math/point.hpp"

#include "VxGui.hpp"
#include "blob.hpp"

typedef std::pair<char, double> camera_setting_t;

const camera_setting_t WHITEBALANCE_RV = {6, 970};
const camera_setting_t WHITEBALANCE_UB = {5, 450};
const camera_setting_t BRIGHTNESS = {1, 0};
const camera_setting_t GAIN = {12, -1.37};
const std::string DEFAULT_CAMERA = "pgusb://b09d01007ea5e8";

class ImageManager
{
private:
    // Image stuff
    image_source_t* imgSrc_;
    image_source_format_t imgFmt_;
    image_u32_t* curImgPtr_;       // stored in left-hand coordinate frame, (0,0) = top left
    
    // Image properties
    int imgWidth_, imgHeight_;

    // current frame information
    image_source_data_t curFrame_;
    bool frameHeld_;

    // Filter properties
    bool filterSet_;
    int minX_, maxX_, minY_, maxY_;

    // Set format for camera
    int camFmt_;
    std::string camUrl_;
    VxGui* gui_;
    
public:
    ImageManager(std::string camUrl, int camFormat);
    ~ImageManager() {};

    inline void setGui(VxGui* guiPtr) { gui_ = guiPtr; }
    inline VxGui* getGui() { return gui_; }
    static void help();
    static void listCameras();
    static void run(ImageManager* imgMgr);

    bool selectCamera(const char* url);
    void initCamera();
    void changeFeature(int featureIndex, double value, image_source_t* isrc);
    image_u32 * getImageFrame();  // get image frame as uint32_t type (rgb)
    void setMaskFilter();
    // void applyMaskFilter(image_u32_t* imagePtr);
    image_u32_t * cropImageMask(image_u32_t* imagePtr);
    inline int getHeight() { return imgHeight_; }
    inline int getWidth() { return imgWidth_; }
};

#endif // IMAGEMANAGER_HPP
