#ifndef BLOB_H
#define BLOB_H


#include "kmeans.hpp"
#include <algorithm>
#include <math/svd22.h>

using namespace std;

struct pixelCoord {
    int x, y;
};

class Blob{
private:
    HSV color_;
    int cluster_;
    vector<pixelCoord> content_;

    float parameter;
    float area;
    double cov[4];
    double eigen_small;
    double eigen_large;
    double meanX;
    double meanY;


public:
    Blob(HSV color, int cluster): color_(color), cluster_(cluster),
    parameter(0), area(0) {
    //guard
        cov[0]=-1;
    }

    HSV getColor() {
        return color_;
    }

    int getCluster() {
       return cluster_;
   }

   float getArea() {
       return area;
   }

   float getParameter() {
       return parameter;
   }

   float getMeanX() {
       return calcMoment(1,0) / area;
   }

   float getMeanY() {
       return calcMoment(1,0) / area;
   }

   pixelCoord getCenter() {
       return {getMeanX, getMeanY};
   }



   float getCircularity() {
	   return 4.0 * M_PI * area / (parameter * parameter);
    }

    bool isCircle() {

        float thresh = .03;
        float circ = getCircularity();
        if (fabs(1 - circ) < thresh)
            return true;
        return false;
    }   

float getEccentricity() {
	return sqrt(1-eigen_small / eigen_large);
}

float calcMoment(int i, int j, float offsetX=0, float offsetY=0) {
	int m = 0;
	for (auto &p: content_) {
       m += pow(p.x - offsetX, i) * pow(p.y - offsetY, j);
   }
   return m;
}

void calcCov() {
	meanX = getMeanX();
	meanY = getMeanY();
	cov[0] = calcMoment(2,0,meanX,meanY) / area;
	cov[1] = cov[2] = calcMoment(1,1,meanX,meanY) / area;
	cov[3] = calcMoment(0,2,meanX,meanY) / area;
}


void calcEigenValues() {
	if (cov[0] == -1) calcCov();
	double U[4]; double S[2]; double V[4];
	svd22(cov, U, S, V);
	eigen_large = max(S[0],S[1]);
	eigen_small = min(S[0],S[1]);
}

void addPoint(int x, int y) {
	content_.push_back({x,y});
	area++;
}

void accumulateParameter() {
	parameter++;
}


};

bool dfs(int i, int j, vector<vector<int>>& assignment, Blob& blob, int cluster);

vector<Blob> assignmentToBlobs(vector<vector<int>>& assignment, 
  vector<HSV>& colors);



//Given the cropped image as a 2D vector with
//RGB values, this function will return all of the
//blobs foudn in the image
vector<Blob> findBlobs(vector<vector<HSV>>& croppedImage, uint w, uint h);

vector<pixelCoord> getCenters(vector<Blob>& blobs);

void cleanBlobs(vector<Blob> imageBlobs, uint w, uint h);



#endif
