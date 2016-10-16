#include "blob.hpp"

bool dfs(int i, int j, vector<vector<int>>& assignment, Blob& blob, int cluster);

vector<Blob> assignmentToBlobs(vector<vector<int>>& assignment, 
			       vector<HSV>& colors);

bool dfs(int i, int j, vector<vector<int>>& assignment, Blob& blob, int cluster) {
    //deal with out of range, not in the same cluster, assigned cluster
    if (i < 0 || j < 0 || cluster < 0 ||
        i >= (int) assignment.size() || j >= assignment[0].size() ||
	(assignment[i][j] >=0 && cluster != assignment[i][j]) )
	
	return true;

    if (assignment[i][j] < 0) {
	if (assignment[i][j] != (-cluster-1))
	    return true;
	return false;
    }

    blob.addPoint(i,j);
    assignment[i][j] = -cluster-1;

    bool isBoundary = false;
    for(int dx = -1;dx < 2; ++dx){
	for(int dy = -1; dy < 2; ++dy) {
	    isBoundary = dfs(i+dx,j+dy, assignment, blob, cluster) || isBoundary;
	}
    }

    if (isBoundary) blob.accumulateParameter();

    return false;
}

vector<Blob> assignmentToBlobs(vector<vector<int>>& assignment, 
			       vector<HSV>& colors) {
    vector<Blob> blobs;
    for (uint i = 0; i < assignment.size(); i++) {
	for(uint j = 0; j < assignment[0].size(); j++) {
	    if (assignment[i][j] >= 0) {
	        int cluster = assignment[i][j];
	        Blob b(colors[cluster], cluster);
		dfs(i,j, assignment, b, cluster);
		blobs.push_back(move(b));
	    }
	}
    }
    return blobs;
}

vector<Blob> findBlobs(vector<vector<HSV>>& croppedImage, uint w, uint h){

    //vector<vector<HSV>> newImage;
    //convertImage(croppedImage, newImage);


    Kmeans classifier(croppedImage, 4);
    classifier.run();
    
    auto assignments = classifier.getAssignment();
    auto means = classifier.getMeans();
    vector<Blob> imageBlobs = assignmentToBlobs(assignments, means);
    //Not sure if this is needed. If so, we would need to
    //find the "right" upper and lower threshold values.
    cleanBlobs(imageBlobs, w, h);
    return imageBlobs;
}


vector<pixelCoord> getCenters(vector<Blob>& blobs) {

    vector<piexelCoord> centers;
    for (uint i = 0; i < blobs.size(); ++i) {

        centers.push_back({blobs[i].getMeanX, blobs[i].getMeanY});
    }

    return centers;
}



void cleanBlobs(vector<Blob> imageBlobs, uint w, uint h){
    vector<Blob> newBlobs;

    float lower_thresh = .005 * w * h;
    float upper_thresh = .1 * w * h;

    for (auto b: imageBlobs){
	if (b.getArea() >= lower_thresh && b.getArea() <= upper_thresh)
	    newBlobs.push_back(b);
    }

    swap(imageBlobs, newBlobs);
}
