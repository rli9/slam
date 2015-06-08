/*
2D tracker using Particle Filter
@author Jiajie Yao  3/22/2015
*/

#include <opencv2/opencv.hpp>
#include <vector>

class ParticleFilterTracker2D
{

public:
	ParticleFilterTracker2D();
	~ParticleFilterTracker2D();

	void setInitialLocation(cv::Point2f location);
	void generateNSamplesUniformly(int N, float range);
	
	int getSampleNumber();
	int getPrevSampleNumber();
	
	cv::Point2f getSampleByIndex(int i);
	float getSampleWeightByIndex(int i);
		
	cv::Point2f getPrevSampleByIndex(int i);
	float getPrevSampleWeightByIndex(int i);
	
	void setNewSampleNeighborhoodRadius(float radius);

	// set p(z_k | x^i_k ). z_k is the observed value and x^i_k is the predicted sample location.
	void setSampleWeightByIndex(int i, float weight); 

	// update all samples' weights before calling resample().
	void resample();

	void offsetAllSamples(float offset_x, float offset_y);
	
	void setLimit(float x_low, float x_high, float y_low, float y_high);
	
private:
	
	float xLowLimit;
	float yLowLimit;
	float xHighLimit;
	float yHighLimit;
		
	cv::Point2f initialLocation;
	std::vector<std::pair<cv::Point2f, float> > samples;
	std::vector<std::pair<cv::Point2f, float> > oldSamples;
	
	// radius of the area where we generate new samples from an old sample ( the old sample is in the center ).
	float newSampleNeighborhoodRadius;

	// generate N new samples in a rectangle area, centered at "center", with size of range X range.
	void generateNSamplesUniformly(std::vector<std::pair<cv::Point2f, float> > & out, cv::Point2f center, float range, int N);

	void sortSamples();

};
