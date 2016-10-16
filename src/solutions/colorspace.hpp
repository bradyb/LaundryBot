#ifndef COLORSPACE_H
#define COLORSPACE_H

#include <math.h>
#include <cstdlib>
#include <vector>

using namespace std;




class HSV {
public:
    float h_;
    float s_;
    float v_;

    HSV(float h=0, float s=0, float v=0): h_(h), s_(s), v_(v) {}
};

class RGB {
public:
    float r_;
    float g_;
    float b_;


    RGB(float r = 0, float g = 0, float b = 0): r_(r), g_(g), b_(b){}
};
float distanceHSV(HSV p1, HSV p2);

HSV randColor();

HSV rgb2hsv(RGB p);

void convertImage(vector<vector<RGB>>& originalImage, vector<vector<HSV>>& newImage);

#endif
