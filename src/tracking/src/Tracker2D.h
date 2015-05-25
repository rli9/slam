// Tracker based on color features.

#include <opencv2/opencv.hpp>
#include "ParticleFilterTracker2D.h"


#define DEFAULT_SIGMA 1.0f
#define DEFAULT_TRACKER_SIZE 110
#define DEFAULT_HISTOGRAM_SIZE 10
#define DEFAULT_SAMPLE_NUM 50
#define DEFAULT_NEW_SAMPLE_RANGE 20
#define DEFAULT_INITIAL_SAMPLE_RANGE 30

class Tracker2D
{
public:
	Tracker2D(cv::Point2f & init_location);
	Tracker2D() {};
	~Tracker2D();

	void readFrame(const cv::Mat & image); // BGR image.
	void updateTrackerStatus();
	void displayCurrentStatus();
	void displayCurrentStatus(std::string & windowName);

	void drawTrackers(cv::Mat & disp_img);
	
	void setObjectFeature(cv::Mat & obj_image); 
	
	void setSigma(float sigma);
	void setTrackerSize(int tracker_size);
	void setHistogramSize(int histogram_size);
	void setSampleNumber(int sample_num);
	void setNewSampleRange(float range);
	void saveResult(int seq_num);
	void writeToVideo(cv::VideoWriter & writer);

	void initialize(cv::Point2f init_location);
	void initialize(cv::Point2f init_location, int tracker_size);
	
private:

	ParticleFilterTracker2D m_particleFilterTracker;

	int m_trackerSize;
	int m_histogramSize;

	cv::Mat m_currentBGR;
	cv::Mat m_currentHSV;

	float * m_objectHfeature;

	float m_sigma; // the std dev for Gaussian distribution used for calculating the probability of one sample.

	int m_sampleNum;

	float m_newSampleRange;

	int m_initialSampleRange;

	float getSampleProbability(int sample_id);

	void calculateHueHistogram(cv::Mat & inputHSVImage, cv::Mat & hist);

};
