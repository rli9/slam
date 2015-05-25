#include <opencv2/opencv.hpp>
#include <string>

/*****************************************************************/

bool mouseDownFlag = false;

void mouse_callback_drawRectMask(int event, int x, int y, int flags, void* parameters)
{

  void **  params = (void **) parameters;
  std::string * windowName = (std::string *)(params[0]);
  int * xBeg = (int*)(params[1]);
  int * xEnd = (int*)(params[2]);
  int * yBeg = (int*)(params[3]);
  int * yEnd = (int*)(params[4]);
  cv::Mat * img = (cv::Mat *)(params[5]);
  cv::Mat drawedImage;
  
  switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:
      if (!mouseDownFlag)
	{
	  mouseDownFlag = true;
	  *xBeg = x;
	  *yBeg = y;
	}
      break;
  
    case CV_EVENT_MOUSEMOVE:
      if (mouseDownFlag)
	{
	  img->copyTo(drawedImage);
	  cv::rectangle(drawedImage, cv::Point2i(*xBeg, *yBeg), cv::Point2i(x, y), cv::Scalar(255, 255, 255), 1);
	  cv::imshow(* windowName, drawedImage);
	  *xEnd = x;
	  *yEnd = y;
	}
      break;
    
    case CV_EVENT_LBUTTONUP:
      mouseDownFlag = false;      
      break;

    default:
      break;
    }
}

/*****************************************************************/
