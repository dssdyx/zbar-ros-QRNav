#ifndef _ZBAR_OPENCV_H
#define _ZBAR_OPENCV_H

#include <ros/ros.h>
#include <zbar.h>
#include <opencv/cv.h>
#include <sensor_msgs/CompressedImage.h> //MJPEG
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace zbar;
using namespace std;

namespace zbar_opencv {
  class QRcodeREAD {
    public:
      QRcodeREAD(ros::NodeHandle nh_);
      void Image_Callback(const sensor_msgs::CompressedImageConstPtr& Img);
      cv::Mat SensorToMat(cv_bridge::CvImagePtr cv_ptr);
      bool DetectQR(cv::Mat inImage);
      void QRdecoder(cv::Mat& Img, cv::Rect rect);
      vector<vector<Point> > RecogniseContour(cv::Mat Img);
      void BuildQRROI(vector<vector<Point> > contour, cv::Mat pic,cv_bridge::CvImagePtr cv_Ptr);
      void StringToPoint(std_msgs::String data);
      ~QRcodeREAD();
  private:
      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;
      image_transport::Publisher pub_Frame,pub_Frame_1;
      ros::Publisher pub_QRdata,pub_QRpoint,pub_Fourpoint,pub_Centralpoint,pub_Judge;
      ros::Subscriber sub_Frame;
      std_msgs::String QRdata;
      geometry_msgs::Point Centralpoint,QRpoint;
      bool Judge; //Judge whether the qrcode is in view.
      std_msgs::UInt8 Judge_topic;
      zbar::ImageScanner scanner;
      cv::Mat pic_0,pic_1;
      vector<vector<Point> > contour_;
      geometry_msgs::Point32 x0,x1,x2,x3;

    };
}

#endif //_ZBAR_OPENCV_H
