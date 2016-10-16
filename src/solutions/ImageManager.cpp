#include <string>
#include <iostream>
#include <utility>

#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"   // image_source_enumerate, image_source_open, image_source_*, ...
#include "common/zarray.h"              // zarray_t, zarray_size, zarray_get, zarray_*, ...

#include "ImageManager.hpp"

using namespace std;

ImageManager::ImageManager(string camUrl, int camFmt)
{
    imgSrc_ = nullptr;
    curImgPtr_ = nullptr;   
    frameHeld_ = filterSet_ = false;
    imgWidth_ = 0;
    imgHeight_ = 0;
    minX_ = INT_MAX;
    minY_ = INT_MAX;
    maxX_ = -1;
    maxY_ = -1;
    camFmt_ = camFmt;
    camUrl_ = camUrl;
}

void ImageManager::listCameras()
{
    // Show all available and then use the first
    zarray_t *urls = image_source_enumerate();
    cout << "$$  Cameras:\n";
    for (int i = 0; i < zarray_size (urls); i++) 
    {
        char *url;
        zarray_get (urls, i, &url);
        cout << "$$  " << i << ": " << url << endl;
    }

    if (zarray_size(urls) == 0) 
    {
        cerr << "Found no cameras." << endl;
    }
}

void ImageManager::help()
{
    cout << "$$  Usage: Pass in a Camera url or file url (file:///)" << endl;
    cout << "$$\t  run with -l flag for list of cameras" << endl;
    cout << "$$\t  run with -m to load mask from file" << endl;
}

bool ImageManager::selectCamera(const char* url)
{
    char* selectedUrl = const_cast<char*> (url);
    if(!imgSrc_)
    {
        if(selectedUrl[0] == '\0')
        {
            // Show all available and then use the first
            zarray_t* urls = image_source_enumerate ();

            if (zarray_size(urls) == 0) 
            {
                cerr << "Found no cameras.\n";
                return false;
            }
            zarray_get (urls, 0, &selectedUrl);
        }

        imgSrc_ = image_source_open (selectedUrl);
        if(imgSrc_ == NULL){
            cerr << "Failed to start Camera (url " << selectedUrl << " did not work!" << endl;
            return false;
        } 

        if(camFmt_ > 0)
        {
            imgSrc_->set_format(imgSrc_, camFmt_);
        }
    }

    cout << "$$  Camera set to: " << selectedUrl << endl;
    return true;
}

void ImageManager::initCamera()
{
    // start image reading
    imgSrc_->start(imgSrc_);

    //TODO: this is not getting the right values for a still image file
    // get image format
    int curFmtIdx = imgSrc_->get_current_format(imgSrc_);
    image_source_format_t imgFmt;
    imgSrc_->get_format(imgSrc_, curFmtIdx, &imgFmt);
    imgWidth_   = imgFmt.width;
    imgHeight_  = imgFmt.height;
    cout << "$$  img width: " << imgWidth_ << endl;
    cout << "$$  img height: " << imgHeight_ << endl;

    // Initialize features
    cout << "$$  changing camera settings\n";
    changeFeature(WHITEBALANCE_RV.first, WHITEBALANCE_RV.second, imgSrc_);
    changeFeature(WHITEBALANCE_UB.first, WHITEBALANCE_UB.second, imgSrc_);
    changeFeature(BRIGHTNESS.first, BRIGHTNESS.second, imgSrc_);
    changeFeature(GAIN.first, GAIN.second, imgSrc_);
}

void ImageManager::changeFeature(int featureIdx, double value, image_source_t* isrc)
{
    isrc->set_feature_value(isrc, featureIdx, value);
    cout << "$$  " << isrc->get_feature_name(isrc, featureIdx) << ":" << isrc->get_feature_value(isrc, featureIdx) << endl;
}

image_u32* ImageManager::getImageFrame()
{
    if( frameHeld_ )
    {
        imgSrc_->release_frame(imgSrc_, &curFrame_);
        frameHeld_ = false;
    }

    image_u32* allocImgPtr = NULL;

    imgSrc_->get_frame (imgSrc_, &curFrame_);
    frameHeld_ = true;
    allocImgPtr = image_convert_u32 (&curFrame_);

    // Apply the mask filter, if set
    // if(filterSet_)
    // {
    //     allocImgPtr = cropImageMask(allocImgPtr);
    // }

    // Destroy the previous image if it exists
    if( curImgPtr_ != NULL )
    {
        // cout <<  "attempting to destroy prev image" << endl;
        image_u32_destroy (curImgPtr_);
        // cout << "destroyed prev image" << endl;
        curImgPtr_ = NULL;
    }

    // Store latest image pointer
    curImgPtr_ = allocImgPtr;
    
    // return the image pointer
    return allocImgPtr;
}

void ImageManager::setMaskFilter()
{
    vector<eecs467::Point<int> > maskPts = gui_->getMask();
    assert(maskPts.size() == 4);  

    minX_ = maskPts[0].x;
    maxY_ = maskPts[0].y;
    maxX_ = maskPts[1].x;
    minY_ = maskPts[1].y;
    filterSet_ = true;

    imgWidth_  = maxX_ - minX_;
    imgHeight_ = maxY_ - minY_;
}

// void ImageManager::applyMaskFilter(image_u32_t* imagePtr)
// {
//     assert(filterSet_);
    
//     // set any pixel outside mask to 0,0,0 - r g b
//     for(int y = 0; y < imagePtr->height; y++)
//     {
//         for(int x = 0; x < imagePtr->stride; x++)
//         {
//             // Ignore pixels outside of width
//             //  (can happen if stride > width)
//             if(x >= imagePtr->width)
//             {
//                 continue;
//             }

//             // If it is outside of mask, set pixel to black
//             if(    (x <= minX_) || (x >= maxX_) 
//                 || (y <= minY_) || (y >= maxY_)  )
//             {
//                 setRGB(
//                     &(imagePtr->buf[ y*imagePtr->stride + x ]), 
//                     0, 0, 0
//                 );
//             }
//         }
//     }
// }

image_u32_t * ImageManager::cropImageMask(image_u32_t* imagePtr)
{
    assert(filterSet_);
    // we have: max-x, min-x, max-y, min-y
    //  img_mask_width, img_mask_height --> what we want to crop

    // set the stride = the old image's stride
    image_u32_t* newImage = image_u32_create_alignment( imgWidth_, 
                                                            imgHeight_,
                                                            imagePtr->stride
                                                        );

    // set the pixels by copying from the old image
    // Create copy in right_hand coordinate frame --> (0,0) is bottom left corner
    for(int y = 0; y < newImage->height; y++)
    {
        for(int x = 0; x < newImage->stride; x++)
        {
            if(x >= newImage->width)
            {
                continue;
            }

            // set normal pixel in new image, to shifted pixel in 
            // (assumming pixel starts from bottom left corner)
            newImage->buf[ (newImage->height - y - 1) * newImage->stride + x ] =
                    imagePtr->buf[ (y + minY_) * imagePtr->stride + (x + minX_) ];
        }
    }

    // Destroy previous masked image if exists
    if(curImgPtr_ != NULL ){
        image_u32_destroy (curImgPtr_);
        curImgPtr_ = NULL;
    }

    // set new cropped image ptr
    curImgPtr_ = newImage;

    // return new image ptr
    return newImage;
}

void ImageManager::run(ImageManager* imgMgr)
{
    thread guiThread(VxGui::start, imgMgr->getGui());
    guiThread.detach();

    if(!imgMgr->selectCamera(imgMgr->camUrl_.c_str()))
    {
        cerr << "Failed to select cam. Exiting..." << endl;
        return;
    }
    
    // start camera and get dimensions
    imgMgr->initCamera();
    imgMgr->getGui()->setImageDimensions(imgMgr->getHeight(), imgMgr->getWidth());
    imgMgr->getGui()->settingMask = true;

    // Loop until 4 pts selected
    while(imgMgr->getGui()->settingMask)
    {
        // get frame
        //      this image is with left-hand coordinate frame 
        //      ( (0,0) is tope left of image)

        // This is plotting the image by flipping it
        imgMgr->getGui()->updateImage(imgMgr->getImageFrame(), VXO_IMAGE_NOFLAGS);
        // if(!viz.getMaskMode()){
        //     break;
        // }
    }

    imgMgr->setMaskFilter();


    //suppose that croppedImage is the image 
    //ready for kmeans
    /*                                                                        //how do I get these?
    vector<vector<HSV>> hsvIm = move(imageu32_2_hsv_cropped(croppedImage, getBottomLeft, getBottomRight));
    vector<Blob> blobVec = findBlobs(hsvIm, imgMgr.getWidth, imgMgr.getHeight);
    */
    cout << "done with mask" << endl;
}
