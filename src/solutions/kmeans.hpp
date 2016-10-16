/*
1;2c  helper for the task3, assign each pixel to a class
  assumptions:
  1. use hsv color space
*/
#ifndef KMEANS_H
#define KMEANS_H

#include <stdint.h>
#include <utility>      // std::move
#include <iostream>
#include "colorspace.hpp"
#include <vector>
#include <assert.h>
#include <time.h>
#include "math/angle_functions.hpp"
using namespace std;

// random number between [0, 1) steal from math_util.h since there's bug in math_util

typedef unsigned int uint;

class Kmeans {
private:
  int k_;
  vector<HSV> means_;
  vector<vector<HSV> > image_;
  vector<vector<int> > assignment_;
  
public:
  Kmeans(vector<vector<HSV> > cropped_image, int k=5): k_(k) {
    assert(k>0 && cropped_image.size() > 0 && k<10);
    // default to 5 b/c
    // Red ball, Green ball, Blue registration square,
    // White/cream playing surface, Black or blue arm mounts

    // initialize mean
    srand(time(NULL));
    means_.resize(k_);
    for (int i=0; i<k; i++) {
      means_[i] = randColor();
    }

    image_ = move(cropped_image); // use move to avoid copy of vectors
    assignment_ = vector<vector<int> >(image_.size(),
				      vector<int>(image_[0].size(), -1)); 
  }

  void run() {
    // start the kmeans algorithm until convergence
    while (true){
      if (assign()) { // assign returns whether convereged
	break;
      }
      update();
    }
  }

  vector<vector<int> > getAssignment() {
    return assignment_;
  }

  vector<HSV> getMeans() {
    assert(assignment_[0][0] != -1); // make sure not the first time
    return means_;
  }
  
private:
  
  
  bool assign() {
    // helper for run, returns true if no change of cluster from previous iteration
    // assign each data point to a cluster
    bool converged = true;
    for (uint i=0; i<assignment_.size(); i++) {
      for (uint j=0; j<assignment_[0].size(); j++) {
	int cluster = assignOne(image_[i][j]);
	assert(cluster >= 0 && cluster < k_);
	if (assignment_[i][j] != cluster) {
	  converged = false;
	  assignment_[i][j] = cluster;
	}
      }
    }
    return converged;
  }

  int assignOne(HSV& c) {
    // helper for assign
    // find the closest means_ and return its index
    float minDist = distanceHSV(means_[0], c); 
    int minIndex = 0;
    for (int i=1; i<k_; i++) {
      float d = distanceHSV(means_[i], c);
      if (d < minDist) {
	minIndex = i;
	minDist = d;
      }
    }
    assert(minIndex >= 0 && minIndex < k_);
    return minIndex;
  }
  
  void update() {
    // helper for run
    // update means_ given the current assignment

    // clear means_
    for (int i=0; i<k_; i++) {
      means_[i] = HSV(0,0,0);
    }
    vector<uint> count(k_, 0);
    vector<float> h_x(k_, 0); // for calculating mean of hue
    vector<float> h_y(k_, 0); // for calculating mean of hue    
    
    // accumulate means
    for (uint i=0; i<assignment_.size(); i++) {
      for (uint j=0; j<assignment_[0].size(); j++) {
	int cluster = assignment_[i][j];
	assert(cluster >= 0 && cluster < k_);
	count[cluster]++;
	h_x[cluster] += cos(image_[i][j].h_);
	h_y[cluster] += sin(image_[i][j].h_);	
	means_[cluster].s_ += image_[i][j].s_;
	means_[cluster].v_ += image_[i][j].v_;
      }
    }
    
    // normalize to find means
    for (int i=0; i<k_; i++) {
      if (count[i] == 0) {
	cout << "no assigned pt for cluster " << i << ", random initialize a mean" << endl;
	means_[i] = randColor();
      } else {
	means_[i].h_ = eecs467::wrap_to_2pi(atan2(h_y[i], h_x[i]));
	means_[i].s_ /= (float) count[i];
	means_[i].v_ /= (float) count[i];
      }
    }
    
    
  }
};

#endif
