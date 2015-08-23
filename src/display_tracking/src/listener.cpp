#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <std_msgs/String.h>

#define WIN_NAME "Tracking"

ros::Publisher trackerPub;
ros::Publisher trackingTogglePub;

bool isDrawing = false;
int startX;
int startY;
int endX;
int endY;


void imageCallback(const sensor_msgs::ImageConstPtr & msg){
  cv::Mat img;

  ROS_INFO("Got image.");

  try{
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch(cv_bridge::Exception & e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  if( isDrawing){
    
    cv::Mat drawImg;
    img.copyTo(drawImg);

    cv::rectangle(drawImg, cv::Point2i(startX, startY), cv::Point2i(endX, endY),
		  cv::Scalar(0,0,255), 3);
    cv::imshow(WIN_NAME, drawImg);
    cv::waitKey(1);
  }
  else{
    cv::imshow(WIN_NAME, img);
    cv::waitKey(1);
  }
}

void mouseCallBack(int event, int x, int y, int flag, void * param){

  std_msgs::String startMsg;
  std_msgs::String stopMsg;

  switch(event){
  case cv::EVENT_LBUTTONDOWN :
    startX = x;
    startY = y;
    endX = x;
    endY = y;
    isDrawing = true;
    break;
  case cv::EVENT_LBUTTONUP :
    endX = x;
    endY = y;
    if( endY > startY && endX > startX){
      // publish the tracker-size information
      std::stringstream trackerSizeInfo;
      trackerSizeInfo << startX << " " ;
      trackerSizeInfo << startY << " " ;
      trackerSizeInfo << endX - startX << " ";
      trackerSizeInfo << endY - startY << " ";
      
      std_msgs::String msg;
      msg.data = trackerSizeInfo.str();
      trackerPub.publish(msg);
      
    }
    isDrawing = false;
    break;
  case cv::EVENT_MOUSEMOVE:
    if( isDrawing ){
      endX = x;
      endY = y; 
    }
    break;
  case cv::EVENT_RBUTTONDOWN:
    // publish the start tracking command!
    if( endY > startY && endX > startX){
      
      startMsg.data = std::string("start");
      trackingTogglePub.publish(startMsg);
    }
    break;
    
  // add a case where we can stop tracking.
  case cv::EVENT_LBUTTONDBLCLK:

    stopMsg.data = std::string("stop");
    trackingTogglePub.publish(stopMsg);
    break;
    
  default:
    break;
  }
}

int main(int argc, char ** argv){
  
  ros::init(argc, argv, "tracking_display");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  ROS_INFO("Begin subscribing...");

  image_transport::Subscriber sub;
  trackerPub = nh.advertise<std_msgs::String>("/change_tracker_size", 1);
  trackingTogglePub = nh.advertise<std_msgs::String>("/tracking_toggle", 1);
  
  try{
    sub = it.subscribe("/camera/tracking_image", 5, imageCallback);
  }
  catch(image_transport::TransportLoadException& e){
    ROS_ERROR("Subscribing Error!");
  }

  cv::namedWindow(WIN_NAME);
  cv::setMouseCallback(WIN_NAME, mouseCallBack);
  //cv::waitKey();
  
  ros::spin();
}
