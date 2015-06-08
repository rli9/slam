#include "Tracker2D.h"
#include <cmath>
#include <stdio.h>
#include <iostream>


Tracker2D::Tracker2D(cv::Point2f & init_location)
{
	initialize(init_location);
}

Tracker2D::~Tracker2D()
{
	if (m_objectHfeature != NULL)
	{
		delete[] m_objectHfeature;
	}
}

void Tracker2D::initialize(cv::Point2f  init_location)
{
	m_objectHfeature = NULL;

	m_sigma = DEFAULT_SIGMA;
	m_histogramSize = DEFAULT_HISTOGRAM_SIZE;
	m_trackerSize = DEFAULT_TRACKER_SIZE;
	m_sampleNum = DEFAULT_SAMPLE_NUM;
	m_newSampleRange = DEFAULT_NEW_SAMPLE_RANGE;
	m_initialSampleRange = DEFAULT_INITIAL_SAMPLE_RANGE;
	
	
	m_particleFilterTracker.setInitialLocation(init_location);
	m_particleFilterTracker.setNewSampleNeighborhoodRadius(m_newSampleRange / 2);
	m_particleFilterTracker.generateNSamplesUniformly(m_sampleNum, m_initialSampleRange);

	std::cout << " Generate " << m_particleFilterTracker.getSampleNumber() << " samples" << std::endl;

}

// add an initializing function with parameters to specify the size of template.
void Tracker2D::initialize(cv::Point2f  init_location, int tracker_size)
{
	m_objectHfeature = NULL;

	m_sigma = DEFAULT_SIGMA;
	m_histogramSize = DEFAULT_HISTOGRAM_SIZE;
	m_trackerSize = tracker_size;
	m_sampleNum = DEFAULT_SAMPLE_NUM;
	m_newSampleRange = DEFAULT_NEW_SAMPLE_RANGE;
	m_initialSampleRange = DEFAULT_INITIAL_SAMPLE_RANGE;

	m_particleFilterTracker.setInitialLocation(init_location);
	m_particleFilterTracker.setNewSampleNeighborhoodRadius(m_newSampleRange / 2);
	m_particleFilterTracker.generateNSamplesUniformly(m_sampleNum, m_initialSampleRange);

	std::cout << "Generate " << m_particleFilterTracker.getSampleNumber() << " samples" << std::endl;  
}

float Tracker2D::getSampleProbability(int sample_id)
{
	cv::Point2f sample = m_particleFilterTracker.getSampleByIndex(sample_id);

	/*
	 *calculate the histogram of H value in the ROI
	 */

	int height = m_currentHSV.rows;
	int width = m_currentHSV.cols;

	// need to check the boundary
	int leftUpX = (int)(sample.x - m_trackerSize /2 );
	leftUpX = leftUpX < 0 ? 0: leftUpX;
	
	int leftUpY = (int)(sample.y - m_trackerSize /2 );
	leftUpY = leftUpY < 0 ? 0 : leftUpY;
	
	int rightBottomX = (int)(sample.x + m_trackerSize / 2);
	rightBottomX = rightBottomX >= width ? width - 1 : rightBottomX;
	
	int rightBottomY = (int)(sample.y + m_trackerSize / 2);
	rightBottomY =  rightBottomY >= height ? height - 1 : rightBottomY;

	int roiWidth = rightBottomX - leftUpX;
	int roiHeight = rightBottomY - leftUpY;

	if( roiWidth <= 5 && roiHeight <= 5){
	  return 0.0f;
	}

	//cv::Mat roiMat(m_currentHSV, cv::Rect((int)(sample.x - m_trackerSize / 2), (int)(sample.y - m_trackerSize / 2), m_trackerSize, m_trackerSize));
	cv::Mat roiMat(m_currentHSV, cv::Rect(leftUpX, leftUpY, roiWidth, roiHeight));

	cv::Mat histforH;

	calculateHueHistogram(roiMat, histforH);

	//std::cout << "Sample feature: " << std::endl;
	//std::cout << histforH << std::endl;

	cv::Mat histForHNorm;
	cv::normalize(histforH, histForHNorm, 1.0, 0.0, cv::NORM_MINMAX);

	/*
	 *Gaussian distribution.
 	 */
	float differenceSquare = 0;
	for (int i = 0; i < m_histogramSize ; i++)
	{
		float delta = m_objectHfeature[i] - histForHNorm.at<float>(i);
		delta = delta * delta;
		differenceSquare += delta;
	}

	//std::cout << differenceSquare  << std::endl;
	//std::cout << expf(-differenceSquare / m_sigma) << std::endl;

	return expf(-differenceSquare / m_sigma);
}

void Tracker2D::readFrame(const cv::Mat & image)
{
  
  if( m_currentHSV.rows == 0){
    m_currentHSV.create(image.rows, image.cols, CV_8UC3);
  }

  if( m_currentBGR.rows == 0){
    m_currentBGR.create(image.rows, image.cols, CV_8UC3);
  }
  
  image.copyTo(m_currentBGR);
  cv::cvtColor(m_currentBGR, m_currentHSV, cv::COLOR_BGR2HSV);
}

void Tracker2D::setObjectFeature(cv::Mat & obj_image)
{
	cv::Mat hsvImage;
	cv::Mat histforH;
	
	cv::cvtColor(obj_image, hsvImage, cv::COLOR_BGR2HSV);
	calculateHueHistogram(hsvImage, histforH);

	cv::Mat histForHNorm;
	cv::normalize(histforH, histForHNorm, 1.0, 0.0, cv::NORM_MINMAX);

	m_objectHfeature = new float[m_histogramSize];

	for (int i = 0; i < m_histogramSize; i++)
	{
		m_objectHfeature[i] = histForHNorm.at<float>(i);
	}

	std::cout << "Object feature: " << std::endl;
	std::cout << histforH << std::endl;
}

void Tracker2D::calculateHueHistogram(cv::Mat & inputHSVImage, cv::Mat & hist)
{
	int hbins = m_histogramSize;
	int bins[] = { hbins };
	float hranges[] = { 0, 180 }; // Hue value has the range of 0 --> 179
	const float * ranges[] = { hranges };
	int channels[] = { 0 };
	cv::calcHist(& inputHSVImage, 1, channels, cv::Mat(), hist, 1, bins, ranges);
}

void Tracker2D::updateTrackerStatus()
{
	int sampleNum = m_particleFilterTracker.getSampleNumber();
	
	std::cout << "Sample number : " << sampleNum << std::endl;
	
	for (int i = 0; i < sampleNum; i++)
	{
		float prob = getSampleProbability(i);
		m_particleFilterTracker.setSampleWeightByIndex(i, prob); // assign new weights.
	}

	std::cout << "Starting resampling ... " << std::endl;
	
	m_particleFilterTracker.resample();
}

void Tracker2D::displayCurrentStatus()
{
	cv::Mat dispImg;
	drawTrackers(dispImg);
	cv::imshow("tracking status", dispImg);
	cv::waitKey();
}

void Tracker2D::displayCurrentStatus(std::string & window_name)
{
	cv::Mat dispImg;
	drawTrackers(dispImg);
	cv::imshow(window_name, dispImg);
	//cv::waitKey();
}

void  Tracker2D::saveResult(int seq_num)
{
	cv::Mat dispImg;
	drawTrackers(dispImg);
	char name[100];
	sprintf(name, "seq.%06d.png", seq_num);
	cv::imwrite(name, dispImg);
}

void Tracker2D::writeToVideo(cv::VideoWriter & writer)
{

	cv::Mat dispImage;
	drawTrackers(dispImage);
	writer.write(dispImage);
}

void Tracker2D::drawTrackers(cv::Mat & disp_img)
{
  int sampleNum = m_particleFilterTracker.getPrevSampleNumber();

  m_currentBGR.copyTo(disp_img);
  cv::Scalar drawingColor;

  for (int i = 0; i < sampleNum; i++){
    
    cv::Point2f sample = m_particleFilterTracker.getPrevSampleByIndex(i);
    float weight = m_particleFilterTracker.getPrevSampleWeightByIndex(i);
    
    if( weight < PROB_THRESHOD ){
      drawingColor = cv::Scalar(0,0,255);
    }
    else{
      drawingColor = cv::Scalar(0,255,0);
    }
    

    cv::Point2f leftUpConer(sample.x - m_trackerSize / 2, sample.y - m_trackerSize / 2);
    cv::rectangle(disp_img, cv::Rect((int)leftUpConer.x, (int)leftUpConer.y, m_trackerSize, m_trackerSize), drawingColor);

  }
}

cv::Point2i Tracker2D::getWeightedAverageLocation()
{

  // use the old samples

  cv::Point2f averagePosition = cv::Point2f(0,0);

  int sampleNum = m_particleFilterTracker.getPrevSampleNumber();
  
  float totalWeight = 0;

  for(int i = 0; i < sampleNum; i ++){
    float w = m_particleFilterTracker.getPrevSampleWeightByIndex(i);

    cv::Point2f pt = m_particleFilterTracker.getPrevSampleByIndex(i);

    // std::cout << "pt(" << pt.x << " , " << pt.y << ")" << " w = " << w << std::endl;

    totalWeight += w;
    averagePosition.x += pt.x * w;
    averagePosition.y += pt.y * w;
  }
  
  if( totalWeight <= 0){
    return cv::Point2i(-1,-1);
  }

  averagePosition.x /= totalWeight;
  averagePosition.y /= totalWeight;

  return cv::Point2i((int)averagePosition.x, (int)averagePosition.y);
}

int  Tracker2D::getNumOfSamplesHasProbLargerThan(float prob_threshold)
{
  int sampleNum = m_particleFilterTracker.getPrevSampleNumber();

  int count = 0;

  for(int i = 0 ; i < sampleNum; i ++){
    
    float prob = m_particleFilterTracker.getPrevSampleWeightByIndex(i);
        
    if( prob >= prob_threshold){
      count ++;
    }
  }

  return count;
}

int Tracker2D::getSampleNum(){
  return m_particleFilterTracker.getPrevSampleNumber();
}

void Tracker2D::offsetTracker(int direction, int offset)
{
  int offsetX, offsetY;
  bool offsetSet = false;

  switch (direction){
  
  case TRACKER_UP:
    offsetX = 0;
    offsetY = - offset;
    offsetSet = true;
    break;

  case TRACKER_DOWN:
    offsetX = 0;
    offsetY = offset;
    offsetSet = true;
    break;

  case TRACKER_LEFT:
    offsetX = -offset;
    offsetY = 0;
    offsetSet = true;
    break;

  case TRACKER_RIGHT:
    offsetX = offset;
    offsetY = 0;
    offsetSet = true;
    break;
    
  default:
    break;
  }
  
  if( offsetSet){
    m_particleFilterTracker.offsetAllSamples(offsetX, offsetY);
  }
}

void Tracker2D::setLimit(int x_low, int x_high, int y_low, int y_high)
{
  m_particleFilterTracker.setLimit((float)x_low, (float)x_high, (float)y_low, (float)y_high);
}
