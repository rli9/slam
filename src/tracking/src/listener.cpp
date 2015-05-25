#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>
//#include <boost/function.hpp>
//#include <boost/bind.hpp>
//#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "Tracker2D.h"
#include "utilities.h"
#include <iostream>
#include <cmath>
#include <std_msgs/String.h>

bool objectSpecified = false;
bool startTracking = false;

Tracker2D *  tracker = NULL;

// the tracker size.
int trackerSize = 10;

// publish tracking images.
image_transport::Publisher pub;

int trackerMaxSize = -1;

void imageCallback(const sensor_msgs::ImageConstPtr & msg){

  ROS_INFO("Got image message.");

  // get the compressed image, and convert it to Opencv format.
  cv::Mat img;
  try{
   img =  cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch(cv_bridge::Exception & e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  ROS_INFO("Converting image done.");
  
  //std::cout << "image size = ( " << img.rows << " X " << img.cols << " )." << std::endl;
  //printf("image data address 0x%x\n", img.data);

  if( startTracking ){
    

    ROS_INFO("Tracker: Reading Frame ... ");
    
    // update the tracking status, and draw the result.
    tracker->readFrame(img);
    
    ROS_INFO("Tracker: Updating status ... ");
    
    tracker->updateTrackerStatus();

    ROS_INFO("Tracker: status updated ... ");
    
    ROS_INFO("Tracker: drawing ... ");
    
    cv::Mat temp;
    img.copyTo(temp);
    tracker->drawTrackers(temp);
    
    ROS_INFO("Tracker: Publishing ... ");
    
    // republish this image.
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", temp).toImageMsg();
    pub.publish(msg);
   
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
    
    // haven't started tracking, but the initial object is specified.
    // create a tracker;
    if( tracker != NULL) delete tracker;

    tracker = new Tracker2D();

    tracker->initialize(cv::Point2f(img.cols / 2, img.rows / 2), trackerSize);

    // set object feature.
    cv::Mat objImage = img(cv::Rect(img.cols / 2 - trackerSize /2, img.rows / 2 - trackerSize /2, trackerSize, trackerSize));

    tracker->setObjectFeature(objImage);
    
    ROS_INFO("Starting Tracking ... " );

    startTracking = true;
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


int main(int argc, char **argv)
{

  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;

  ROS_INFO("Begin subscribing...");

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub;

  pub = it.advertise("camera/tracking_image", 1);
  
  try{
    sub = it.subscribe("/camera/decomp_image", 5, imageCallback);
  }
  catch(image_transport::TransportLoadException& e){
    ROS_ERROR("Subscribing Error!");
  }

  
  ros::Subscriber trackSizeSub = nh.subscribe<geometry_msgs::Twist>("/change_tracker_size", 1, trackerAdjustingCallback);
 
  ros::Subscriber trackingToggleSub = nh.subscribe<std_msgs::String>("/tracking_toggle", 1, trackingToggleCallback);
  
  ROS_INFO("Subscribing done...");
  
  ros::spin();
}
