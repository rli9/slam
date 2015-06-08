#include "ParticleFilterTracker2D.h"


bool init_sampling = true;

ParticleFilterTracker2D::ParticleFilterTracker2D()
{
  xLowLimit = yLowLimit = xHighLimit = yHighLimit = 0;
}

ParticleFilterTracker2D::~ParticleFilterTracker2D()
{

}

void ParticleFilterTracker2D::resample()
{

  // Assume that f_i = w_i / total_w, 
  // then, near sample i, we generate ( f_i * N ) new samples uniformly.
  // the number of smaples may change after resampling.
  
  // backup samples.
  oldSamples.clear();
  for(int i = 0; i < samples.size(); i ++){
    oldSamples.push_back(samples.at(i));
  }

  
  std::vector<std::pair<cv::Point2f, float> > newSamples;
  
  float totalWeight = 0;
  for (int i = 0; i < samples.size(); i++)
    {
      totalWeight += samples.at(i).second;
    }


  sortSamples();
  int samplesRemaining = samples.size();
  int i = 0;

  while (samplesRemaining > 0)
    {
      float newSampleNum = samples.at(i).second / totalWeight * samples.size();

      int newSampleNumInt = (int)newSampleNum + 1;

      if (samplesRemaining - newSampleNumInt < 0)
	{
	  newSampleNumInt = samplesRemaining;
	}

      if (newSampleNumInt > 1)
	{
	  generateNSamplesUniformly(newSamples, samples.at(i).first, 2 * newSampleNeighborhoodRadius, newSampleNumInt);
	}
      else // newSampleNumInt == 1
	{
	  newSamples.push_back(samples.at(i));
	}

      samplesRemaining -= newSampleNumInt;
      i++;
    }

  //for (int i = 0; i < samples.size(); i++)
  //{

  //	float newSampleNum = samples.at(i).second / totalWeight * samples.size();

  //	int newSampleNum = (int)(samples.at(i).second / totalWeight * samples.size());

  //	if (newSampleNum > 1)
  //	{
  //		generateNSamplesUniformly(newSamples, samples.at(i).first, 2 * newSampleNeighborhoodRadius, newSampleNum);
  //	}

  //	else if ( newSampleNum > 0)
  //	{
  //		newSamples.push_back(samples.at(i));
  //	}
  //}

  samples = newSamples;

}

void ParticleFilterTracker2D::generateNSamplesUniformly(std::vector<std::pair<cv::Point2f, float> > & out, cv::Point2f center, float range, int N)
{
  //int pointsOnOneCol = (int)sqrtf(N); // Y axis
  //int pointsOnOneRow = N / pointsOnOneCol; // X axis

  //float rowDelta = range / pointsOnOneRow;
  //float colDelta = range / pointsOnOneCol;

  //int count = 0;
  //int rowCount = 0;
  //float newSampleX = center.x - range / 2;
  //float newSampleY = center.y - range / 2;

  //while (count < N)
  //{
  //	out.push_back(std::pair<cv::Point2f, float>(cv::Point2f(newSampleX, newSampleY), 0.0f));
  //	newSampleX += rowDelta;

  //	if (newSampleX > center.x + range / 2)
  //	{

  //		if (rowCount % 2 == 0)
  //		{
  //			newSampleX = center.x - range / 2;
  //		}
  //		else
  //		{
  //			// add an offset at the begining by rowDelta (trying to make it more random ).
  //			newSampleX = center.x - range / 2 + rowDelta;
  //		}
  //		newSampleY += colDelta;
  //		rowCount++;
  //	}
  //	count++;
  //}

  int count = 0;
  int randomRange = 100;

  while (count < N){
    
    float randomX = (float)(rand() % randomRange);
    float randomY = (float)(rand() % randomRange);
    
    randomX = (randomX - randomRange * 1.0f / 2) / randomRange;
    randomY = (randomY - randomRange * 1.0f / 2) / randomRange;
    
    float newX = center.x + randomX * range;
    float newY = center.y + randomY * range;

    if( init_sampling)
      std::cout << "new sample: (" << newX << "," << newY << ")" << std::endl;
    
    newX = newX <= xLowLimit ? xLowLimit : newX;
    newX = newX >= xHighLimit ? xHighLimit : newX;
    newY = newY <= yLowLimit ? yLowLimit : newY;
    newY = newY >= yHighLimit ? yHighLimit : newY;

    if( init_sampling)
      std::cout << "new sample (after adjusted): (" << newX << "," << newY << ")" << std::endl;
    
    out.push_back((std::pair<cv::Point2f, float>(cv::Point2f(newX, newY), 0.0f)));

    count++;
  }

  
}

void ParticleFilterTracker2D::setInitialLocation(cv::Point2f location)
{
  initialLocation = location;
}

void ParticleFilterTracker2D::generateNSamplesUniformly(int N, float range)
{
  generateNSamplesUniformly(samples, initialLocation, range, N);
  if( init_sampling) init_sampling = false;
}

int ParticleFilterTracker2D::getSampleNumber()
{
  return samples.size();
}

int ParticleFilterTracker2D::getPrevSampleNumber(){
  return oldSamples.size();
}

cv::Point2f ParticleFilterTracker2D::getSampleByIndex(int i)
{
  return samples.at(i).first;
}

cv::Point2f ParticleFilterTracker2D::getPrevSampleByIndex(int i)
{
  return oldSamples.at(i).first;
}

float ParticleFilterTracker2D::getSampleWeightByIndex(int i)
{
  return samples.at(i).second;
}

float ParticleFilterTracker2D::getPrevSampleWeightByIndex(int i)
{
  return oldSamples.at(i).second;
}

void ParticleFilterTracker2D::setNewSampleNeighborhoodRadius(float radius)
{
  newSampleNeighborhoodRadius = radius;
}

void ParticleFilterTracker2D::setSampleWeightByIndex(int i, float weight)
{
  samples.at(i).second = weight;
}


static bool compareTwoSample(std::pair<cv::Point2f, float> sample1, std::pair<cv::Point2f, float> sample2)
{
  return sample1.second > sample2.second;
}
void ParticleFilterTracker2D::sortSamples()
{
  std::sort(samples.begin(), samples.end(), compareTwoSample);
}

void ParticleFilterTracker2D::offsetAllSamples(float offset_x, float offset_y)
{
  for(int i = 0; i < samples.size(); i ++){
    
    float x = samples.at(i).first.x + offset_x;
    float y = samples.at(i).first.y + offset_y;

    x = x <= xLowLimit ? xLowLimit : x;
    y = y <= yLowLimit ? yLowLimit : y;
    x = x >= xHighLimit ? xHighLimit : x;
    y = y >= yHighLimit ? yHighLimit : y;

    samples.at(i).first = cv::Point2f(x,y);

  }
}

void ParticleFilterTracker2D::setLimit(float x_low, float x_high, float y_low, float y_high)
{
  xLowLimit = x_low;
  xHighLimit = x_high;
  yLowLimit = y_low;
  yHighLimit = y_high;
}

