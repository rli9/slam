// Tracker based on color features.

#include <opencv2/opencv.hpp>
#include "ParticleFilterTracker2D.h"


#define DEFAULT_SIGMA 1.0f
#define DEFAULT_TRACKER_SIZE 110
#define DEFAULT_HISTOGRAM_SIZE 10
#define DEFAULT_SAMPLE_NUM 80
#define DEFAULT_NEW_SAMPLE_RANGE 50
#define DEFAULT_INITIAL_SAMPLE_RANGE 30

#define TRACKER_UP 1
#define TRACKER_DOWN 2
#define TRACKER_LEFT 3
#define TRACKER_RIGHT 4
#define TRACKER_MOVE_STEP 20 // offset by 20 pixels each time.

#define PROB_THRESHOD 0.4F
#define HIGH_CONFID_NUM_RATIO_THRESHOLD 0.2F


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

	cv::Point2i getWeightedAverageLocation();
	
	int getSampleNum();
	int getNumOfSamplesHasProbLargerThan(float prob_threshold);

	void offsetTracker(int direction, int offset = TRACKER_MOVE_STEP );

	void setLimit(int x_low, int x_high, int y_low, int y_high);

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



