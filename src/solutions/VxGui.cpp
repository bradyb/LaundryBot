#include <algorithm>

#include "VxGui.hpp"

using namespace std;

VxGui::VxGui(int argc, char** argv, string saveDir)
: VxGtkWindowBase(argc, argv, 640, 480, 15)
{
    imgWidth_ = imgHeight_ = 0;
    imgPtr_ = NULL;

    settingMask = false;

    img_flags_ = VXO_IMAGE_NOFLAGS; 

    xScaleFactor_ =1;
    yScaleFactor_ =1;

    xScaleFactorMask_ = 3;
    yScaleFactorMask_ = 3;
    saveDir_ = saveDir;
}

void VxGui::addMaskPoint(float x, float y, uint32_t buttonMask)
{
    // Store mask points in rescaled coordinates
    if(settingMask && buttonMask)
    {
        // If 2 points have already been clicked, clear points to restart selection
        if(pointsClicked_.size() >= 2)
        {
            pointsClicked_.clear();
        }

        pointsClicked_.push_back(eecs467::Point<double>(x * xScaleFactor_, y * yScaleFactor_));
    }
}

void VxGui::storeMaskPts()
{
    // exit mask mode, given that 2 points have been clicked
    if(pointsClicked_.size() == 2 && settingMask)
    {
        settingMask = false;
    }
    else
    {
        cout << "$$  not enough points to create mask" << endl;
    }
}

void VxGui::saveImg()
{
    imgLock_.lock();
    if(image_u32_write_pnm (imgPtr_, saveDir_.c_str()) == 0)
    {
        cout << "saved image to " << saveDir_ << endl;
    }
    else
    {
        cout << "failed to save image to file" << endl;
    }
    imgLock_.unlock();
}

void VxGui::start(VxGui* gui)
{
    cout << "$$  starting up gui" << endl;
    gui->run();
}

void VxGui::setImageDimensions(int w, int h)
{
    imgWidth_ = w;
    imgHeight_ = h; 
}

void VxGui::updateImage(image_u32_t* newImg, int imgFlags)
{
    img_flags_ = imgFlags;

    imgLock_.lock();
    image_u32_destroy (imgPtr_);
    imgPtr_ = image_u32_create_alignment (newImg->width, newImg->height, newImg->stride);

    for(int y = 0; y < newImg->height; y++)
    {
        for( int x = 0; x < newImg->stride; x++ )
        {
            imgPtr_->buf[ y * newImg->stride + x ] = newImg->buf[ y * newImg->stride + x ];
        }
    }

    assert (imgPtr_ != NULL);
    imgLock_.unlock();
}

void VxGui::plotMaskPosition(vx_buffer_t* imgBuf)
{
    if (pointsClicked_.size() == 2)
    {
        int maskXDiff = pointsClicked_[1].x - pointsClicked_[0].x;
        int maskYDiff = pointsClicked_[0].y - pointsClicked_[1].y;
        // third point (top right)
        pointsClicked_.push_back(
            eecs467::Point<int>(
                pointsClicked_[0].x + maskXDiff,
                pointsClicked_[0].y)
                );

        // fourth point (bottom left)
        pointsClicked_.push_back(
            eecs467::Point<int>(
                pointsClicked_[0].x,
                pointsClicked_[0].y - maskYDiff)
                );
    }

    
    vector<float> plotPoints;
    for(size_t i = 0; i < pointsClicked_.size(); i++)
    {
        plotPoints.push_back(pointsClicked_[i].x);
        plotPoints.push_back(pointsClicked_[i].y);
        vx_buffer_add_back(
            imgBuf, 
            vxo_chain(
                vxo_mat_translate3(pointsClicked_[i].x * xScaleFactor_, pointsClicked_[i].y * yScaleFactor_ , 0),
                vxo_mat_scale3(5,5, 1),
                vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vxo_circle(vxo_mesh_style(vx_red)))
                )
            );
    }

    if (pointsClicked_.size() == 4)
    {
        // this magic is swapping the interpolated points,
        // since the 3 and 4th positions in the vector are
        // the bottom right point, cauing the lines
        // to appear not as a square. 
        plotPoints.push_back(pointsClicked_.front().x);
        plotPoints.push_back(pointsClicked_.front().y);
        iter_swap(plotPoints.begin() + 2, plotPoints.begin() + 4);
        iter_swap(plotPoints.begin() + 3, plotPoints.begin() + 5);

        vx_buffer_add_back(imgBuf, vxo_chain(
            vxo_pix_coords(
                    VX_ORIGIN_BOTTOM_LEFT, 
                    vxo_lines(vx_resc_copyf(plotPoints.data(), plotPoints.size()),
                        plotPoints.size() / 2, 
                        GL_LINE_STRIP,
                        vxo_lines_style(vx_purple, 2.0f) 
                        )
                )
                )
        );

        plotPoints.clear();
        pointsClicked_.pop_back();
        pointsClicked_.pop_back();
    }
}

void VxGui::render(void)
{
    vx_buffer_t* imgBuf = vx_world_get_buffer(world_, "imgBuf");
    vx_buffer_add_back(imgBuf, vxo_grid());

    // Add image object
    imgLock_.lock();
    
    if(imgPtr_ != NULL)
    {
        {   
            // Plotting Full Image with bottom left anchor
            vx_buffer_add_back(
                imgBuf, 
                vxo_chain(  
                    vxo_mat_scale3(xScaleFactor_, yScaleFactor_, 1),
                    vxo_pix_coords(
                        VX_ORIGIN_BOTTOM_LEFT, 
                        vxo_image_from_u32(
                            imgPtr_, 
                            img_flags_, 
                            VXO_IMAGE_NOFLAGS)
                        )
                    )
                );
        }
        if (settingMask)
        {
            plotMaskPosition(imgBuf);   
        }

        // if(settingMask == true)
        
        // Also plot points from mask to show what we have selected
            

        //     // add interpolated points for remaining corners of mask
        //     if(pointsClicked_.size() == 2)
        //     {
        //         // NOTE: assumes first point clicked is top left corner and 2nd is bottom right
        //         int maskXDiff = pointsClicked_[1].x - pointsClicked_[0].x;
        //         int maskYDiff = pointsClicked_[0].y - pointsClicked_[1].y;
        //         // int maskXDiff = ;
        //         // int maskYDiff = ;
        //         // double cropRotation = atan2(
        //         //     pointsClicked_[1].y - pointsClicked_[0].y, 
        //         //      pointsClicked_[1].x - pointsClicked_[0].x);

        //         vector< eecs467::Point<int> > interpPts;
        //         // top right point
        //         interpPts.push_back(
        //                 eecs467::Point<int>(
        //                     pointsClicked_[0].x + maskXDiff,
        //                     pointsClicked_[0].y )
        //             );

        //         // botom left point
        //         interpPts.push_back(
        //                 eecs467::Point<int>(
        //                     pointsClicked_[0].x,
        //                     pointsClicked_[0].y - maskYDiff )
        //             );

        //         for(int i = 0; i < (int)interpPts.size(); i++)
        //         {
        //             vx_buffer_add_back(
        //                 imgBuf, 
        //                 vxo_chain(
        //                     vxo_mat_translate3(
        //                         interpPts[i].x*xScaleFactor_, 
        //                         interpPts[i].y*yScaleFactor_ , 
        //                         0
        //                         ),
        //                     vxo_mat_scale3(5,5,1),
        //                     vxo_pix_coords(
        //                         VX_ORIGIN_BOTTOM_LEFT, 
        //                         vxo_circle(vxo_mesh_style(vx_blue))
        //                         )
        //                     )
        //                 );
        //         }

        //         int npoints = 8;
        //         float points[npoints*3];
        //         int k = 0, ptsIdx = 0, interpPtsIdx = 0;
        //         for(int j = 0; j < (npoints) / 2; j++)
        //         {
        //             if( (j % 2) == 0)  // even, set point from pointsClicked_
        //             {
        //                 points[3*k + 0] = pointsClicked_[ptsIdx].x * xScaleFactor_;
        //                 points[3*k + 1] = pointsClicked_[ptsIdx].y * yScaleFactor_;
        //                 points[3*k + 2] = 0;
        //                 ptsIdx++;
        //                 if(ptsIdx >= 2)
        //                 {
        //                     ptsIdx = 0;
        //                 }

        //                 points[3*(k+1) + 0] = interpPts[interpPtsIdx].x * xScaleFactor_;
        //                 points[3*(k+1) + 1] = interpPts[interpPtsIdx].y * yScaleFactor_;
        //                 points[3*(k+1) + 2] = 0;
        //             }
        //             else 
        //             {
        //                 points[3*k + 0] = interpPts[interpPtsIdx].x * xScaleFactor_;
        //                 points[3*k + 1] = interpPts[interpPtsIdx].y * yScaleFactor_;
        //                 points[3*k + 2] = 0;
        //                 interpPtsIdx++;
        //                 if(interpPtsIdx >= 2)
        //                 {
        //                     interpPtsIdx = 0;
        //                 }

        //                 points[3*(k+1) + 0] = pointsClicked_[ptsIdx].x * xScaleFactor_;
        //                 points[3*(k+1) + 1] = pointsClicked_[ptsIdx].y * yScaleFactor_;
        //                 points[3*(k+1) + 2] = 0;
        //             }
        //             k += 2;
        //         }

        //         // Add each line to the buffer to be drawn
        //         vx_buffer_add_back (
        //             imgBuf, 
        //             vxo_chain(
        //                 vxo_pix_coords(
        //                     VX_ORIGIN_BOTTOM_LEFT, 
        //                     vxo_lines(vx_resc_copyf (points, npoints*3),
        //                         npoints, 
        //                         GL_LINES,
        //                         vxo_lines_style(vx_purple, 2.0f)
        //                         )
        //                     )
        //                 )
        //             );
        //     }
        // }
        // else // plot masked image
        // {
        //     // if we have a masked image, we can plot only the blob cluster parts of it
        //     // This modifies our_img_ptr to only show pixels at following locations
        //     // if(detector.result.size() > 0)
        //     // {
        //     //     int width   = imgPtr_->width;
        //     //     int height  = imgPtr_->height;

        //     //     // This copies pixels to a new image with only the selected pixels from the mask
        //     //     for(int i = 0; i < (int)detector.result.size(); i++)
        //     //     {
        //     //         int colorHue = detector.H_value[detector.result[i].color];
        //     //         if( (colorHue> 40) && (colorHue<65))
        //     //         {
        //     //             continue;
        //     //         }

        //     //         uint32_t rgbVal = convertHSVtoRGB( colorHue );

        //     //         int8_t r, g, b;
        //     //         r = (rgbVal & RED_MASK)     >> 0;
        //     //         g = (rgbVal & GREEN_MASK)   >> 8; 
        //     //         b = (rgbVal & BLUE_MASK)    >> 16;

        //     //         for(int j = 0; j < (int)detector.result[i].x_cords.size(); j++)
        //     //         {
        //     //             if (detector.result[i].color == 6) continue;
        //     //             int x = detector.result[i].x_cords[j];
        //     //             int y = detector.result[i].y_cords[j];
        //     //             int stride = imgPtr_->stride;
        //     //             if( (y*stride + x) < (height * stride + width))
        //     //             {
        //     //                 // blob_only_image->buf[y*stride + x] = 
        //     //                 setRGB(&imgPtr_->buf[y*stride + x], r,g,b);

        //     //                 // cout << "y*stride+x = "<< y*stride+x << endl;
        //     //             }
        //     //         }
        //     //     }
        //     //     // plot_blob_only = true;
        //     // }

        //     // We have already found the mask, plot masked image on normal grid
        //     vx_buffer_add_back(
        //         imgBuf, 
        //         vxo_chain(  
        //             vxo_mat_scale3(xScaleFactorMask_,yScaleFactorMask_,1),
        //             vxo_pix_coords(
        //                 VX_ORIGIN_BOTTOM_LEFT, 
        //                 vxo_image_from_u32(imgPtr_, img_flags_, VXO_IMAGE_NOFLAGS)
        //                 )
        //             )
        //         );
        // }
    }
    imgLock_.unlock();
    // update window with new buffer
    vx_buffer_swap(imgBuf);
}