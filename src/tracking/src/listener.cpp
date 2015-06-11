#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>

//#include <boost/function.hpp>
//#include <boost/bind.hpp>
//#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "Tracker2D.h"
#include "utilities.h"
#include <iostream>
#include <cmath>
#include <std_msgs/String.h>


#define PRINT_ROS_INFO

bool objectSpecified = false;
bool startTracking = false;

Tracker2D *  tracker = NULL;

// the tracker size.
int trackerSize = 10;

// publish tracking images.
image_transport::Publisher pub;

// publish the object location ( & image size)
ros::Publisher location_pub; 

int trackerMaxSize = -1;

// lock for the tracker.
boost::mutex trackerMutex;

// the direction of the object's last movement.
int lastMovementDirection = -1;

void imageCallback(const sensor_msgs::ImageConstPtr & msg){

#ifdef PRINT_ROS_INFO
  ROS_INFO("Got image message.");
#endif

  // get the compressed image, and convert it to Opencv format.
  cv::Mat img;
  try{
   img =  cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch(cv_bridge::Exception & e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

#ifdef PRINT_ROS_INFO
  ROS_INFO("Converting image done.");
#endif  

  //std::cout << "image size = ( " << img.rows << " X " << img.cols << " )." << std::endl;
  //printf("image data address 0x%x\n", img.data);

  if( startTracking ){

    trackerMutex.lock();

#ifdef PRINT_ROS_INFO

    ROS_INFO("Tracker: Reading Frame ... ");
#endif    

    // update the tracking status, and draw the result.
    tracker->readFrame(img);
    
#ifdef PRINT_ROS_INFO
    ROS_INFO("Tracker: Updating status ... ");
#endif    

    tracker->updateTrackerStatus();

#ifdef PRINT_ROS_INFO
    ROS_INFO("Tracker: status updated ... ");
    ROS_INFO("Tracker: drawing ... ");
#endif    

    cv::Mat temp;
    img.copyTo(temp);
    tracker->drawTrackers(temp);
    
#ifdef PRINT_ROS_INFO
    ROS_INFO("Tracker: Publishing ... ");
#endif    

    // republish this image.
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp).toImageMsg();
    pub.publish(msg);

    // publish to topic -- object_location
    cv::Point2i location = tracker->getWeightedAverageLocation();
    std::stringstream locationStrStream;

    int currentNum = tracker->getSampleNum();
    int numWithHigConfidence = tracker->getNumOfSamplesHasProbLargerThan(PROB_THRESHOD);

    
    float highConfidenceSampleRatio;
    if( currentNum <= 0){
      highConfidenceSampleRatio = 0;
    }else{
      highConfidenceSampleRatio = numWithHigConfidence * 1.0f / currentNum;
    }

    std::cout << "High confidence sample ratio = " << highConfidenceSampleRatio << std::endl; 
    
    if( location.x < 0 || location.y < 0 || highConfidenceSampleRatio <= HIGH_CONFID_NUM_RATIO_THRESHOLD ){
      //locationStrStream << "object_x " << "0" << " , " << "object_y " << "0" << " , ";
      
      locationStrStream << "object_x " << img.cols /2   << ", " << "object_y " << img.rows / 2 << ", ";
      
      // make offsets to the samples:
      
      ROS_INFO("Tracker offset!");
      if( lastMovementDirection == TRACKER_UP)
	tracker->offsetTracker(TRACKER_DOWN);
      else if( lastMovementDirection == TRACKER_DOWN)
	tracker->offsetTracker(TRACKER_UP);
      else if( lastMovementDirection == TRACKER_LEFT)
	tracker->offsetTracker(TRACKER_RIGHT);
      else if( lastMovementDirection == TRACKER_RIGHT)
	tracker->offsetTracker(TRACKER_LEFT);
      
      
    }else{
      // "x 10, y 10, width 360, height 640"
      locationStrStream << "object_x " << location.x << ", " << "object_y " << location.y << ", ";
      lastMovementDirection = -1;
    }

    locationStrStream << "width " << img.cols << ", " << "height " << img.rows << ", ";

    locationStrStream << "direction follow" ;

    std_msgs::String locationMsg;
    locationMsg.data = locationStrStream.str();
    location_pub.publish(locationMsg);
        
    // release the lock
    trackerMutex.unlock();

  }

  else if (! objectSpecified ) {
    // show the image and let the user specify the inital result.

    //std::cout << img.rows << "," << img.cols << std::endl;
    
    cv::Mat tempImage(img.rows, img.cols, img.type());
    img.copyTo(tempImage);

    
    if( trackerMaxSize < 0){
      trackerMaxSize = MIN(img.rows, img.cols) - 1;
    }
    
    ROS_INFO("Drawing tracker ... ");

    cv::rectangle(tempImage, cv::Rect(tempImage.cols / 2 - trackerSize / 2, tempImage.rows / 2 - trackerSize / 2, trackerSize, trackerSize), cv::Scalar(0,0,255));
    
    // republish this image.
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tempImage).toImageMsg();
    pub.publish(msg);

  }
  
  else{

    trackerMutex.lock();
    
    // haven't started tracking, but the initial object is specified.
    // create a tracker;
    if( tracker != NULL) delete tracker;

    tracker = new Tracker2D();

    tracker->setLimit(5 , img.cols - 5, 5 , img.rows - 5);
    tracker->initialize(cv::Point2f(img.cols / 2, img.rows / 2), trackerSize);
    
    // set object feature.
    cv::Mat objImage = img(cv::Rect(img.cols / 2 - trackerSize /2, img.rows / 2 - trackerSize /2, trackerSize, trackerSize));
    
    tracker->setObjectFeature(objImage);
    
    ROS_INFO("Starting Tracking ... " );

    startTracking = true;

    trackerMutex.unlock();
  }
}

void trackerAdjustingCallback(const geometry_msgs::Twist::ConstPtr & msg)
{

  ROS_INFO("Change Tracker Size ... ");
  
  float sizeFactor = msg->linear.x;
  sizeFactor = sizeFactor < 0? - sizeFactor : sizeFactor;

  std::cout << "size factor = " << sizeFactor << std::endl;
  
  int size = (int)(trackerMaxSize * sizeFactor);
  
  size = size < 10? 10 : size;

  std::cout << "Tracker size = " << size << std::endl;

  trackerSize = size;
}


void trackingToggleCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string cmd = msg->data;
  
  if( cmd.compare("start") == 0){
    objectSpecified = true;
  }
  else if( cmd.compare("stop") == 0){
    startTracking = false;
    objectSpecified = false;
  }
}



void carFeedbackCallback(const std_msgs::String::ConstPtr & msg)
{
  
  std::string status = msg->data;

  // if the tracker lose its target, then move to the direction of the object's last move.

  if( status.compare("left") == 0){
    lastMovementDirection = TRACKER_LEFT;
  }

  else if( status.compare("right") == 0){
    lastMovementDirection = TRACKER_RIGHT;
  }
  
  else if( status.compare("forward") == 0){
    lastMovementDirection = TRACKER_UP;
  }
  
  else if( status.compare("backward") == 0){
    lastMovementDirection = TRACKER_DOWN;
  }

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;

  ROS_INFO("Begin subscribing...");

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub;

  pub = it.advertise("camera/tracking_image", 1);
  
  location_pub = nh.advertise<std_msgs::String>("/car_control", 1);
  
  try{
    sub = it.subscribe("/camera/decomp_image", 5, imageCallback);
  }
  catch(image_transport::TransportLoadException& e){
    ROS_ERROR("Subscribing Error!");
  }

  ros::Subscriber trackSizeSub = nh.subscribe<geometry_msgs::Twist>("/change_tracker_size", 1, trackerAdjustingCallback);
 
  ros::Subscriber trackingToggleSub = nh.subscribe<std_msgs::String>("/tracking_toggle", 1, trackingToggleCallback);
  
  ros::Subscriber robotFeedbackSub = nh.subscribe<std_msgs::String>("/car_feedback", 1, carFeedbackCallback);

  ROS_INFO("Subscribing done...");
  
  ros::spin();
}
