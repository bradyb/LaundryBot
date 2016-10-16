#include "colorspace.hpp"



vector<vector<HSV>> convertImage(vector<vector<RGB>>& originalImage);

float distanceHSV(HSV p1, HSV p2) {
    float x1 = p1.s_ * cos(p1.h_);
    float y1 = p1.s_ * sin(p1.h_);

    float x2 = p2.s_ * cos(p2.h_);
    float y2 = p2.s_ * sin(p2.h_);

    float dist = sqrt( pow(x1 - x2,2) + pow(y1 - y2, 2) + pow(p1.v_ - p2.v_, 2));

    return dist;    
}

HSV randColor() {
    float temp_H = (( (float) rand()) / RAND_MAX) * 360.0;
    float temp_S = (( (float) rand()) / RAND_MAX);
    float temp_V = (( (float) rand()) / RAND_MAX);

    HSV newColor(temp_H, temp_S, temp_V);

    return newColor;
}

HSV rgb2hsv(RGB p) {
    float r = p.r_ / 255.0;
    float g = p.g_ / 255.0;
    float b = p.b_ / 255.0;

    float M = max(r,max(g,b));
    float m = min(r,min(g,b));
    float C = M - m;
  
    float V = M;
    float S = C / V;

    float temp_H;
    if(C == 0)
	temp_H = 0;
    else if(M == r)
	temp_H =(float) ((int)((p.g_ - p.b_) / C) % 6);
    else if(M == g)
	temp_H =(p.b_ - p.r_) / C + 2;
    else if(M == b)
	temp_H =(p.r_ - p.g_) / C + 4;

    float H = temp_H * 60;

    HSV newColor(H,S,V);

    return newColor;
}

void  convertImage(vector<vector<RGB>>& originalImage, vector<vector<HSV>>& newImage){
    
    
    newImage.resize(originalImage.size());
    for (uint i = 0; i < newImage.size(); ++i){
	newImage[i].resize(originalImage[i].size());    	
    }

    for (uint i = 0; i < newImage.size(); ++i){
	for(uint j = 0; j < newImage[0].size(); ++j){
	    newImage[i][j] = rgb2hsv(originalImage[i][j]);
	}
    }
  
}
