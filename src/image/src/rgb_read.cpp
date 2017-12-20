#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/photo.hpp"
#include "opencv2/highgui/highgui.hpp"

void chatterCallback(const sensor_msgs::Image::ConstPtr& msg);

struct output
{
  float turn = 0;
  bool time = false;
}

class ImageConverter
{
  ros::NodeHandle n;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_RGB_sub;
  

public:

  int lastXR = -1;
  int lastYR = -1;
  bool red = false;

  int lastXY = -1;
  int lastYY = -1;
  bool yellow = false;

  ImageConverter()
    : it(n)
  {
    image_RGB_sub = it.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat image, imageLines, HSVImage, ThreshImageR, ThreshImageY;

    image = cv_ptr->image;

    imageLines = cv::Mat::zeros(image.size(), CV_8UC3);

    cvtColor(image, HSVImage, CV_BGR2HSV);

    // Red
    inRange(HSVImage, cv::Scalar(150, 140, 100), cv::Scalar(200, 200, 255), ThreshImageR);
    //Yellow
    inRange(HSVImage, cv::Scalar(15, 100, 120), cv::Scalar(50, 255, 255), ThreshImageY);

    cv::Moments MomentsR = moments(ThreshImageR);
    double m10R = MomentsR.m10;
    double m01R = MomentsR.m01;
    double AreaR = MomentsR.m00;
    red = false;

    if(AreaR > 30000){
      unsigned int xR = m10R / AreaR;
      unsigned int yR = m01R / AreaR;

      if(xR >= 0 && yR >= 0){
        cv::line(imageLines, cv::Point(xR, yR), cv::Point(lastXR, lastYR), cv::Scalar(255, 0, 0), 2);
      }

      lastXR = xR;
      lastYR = yR;
      red = true;
    } //homographic
    // 2d <-> 3d conversion

    cv::Moments MomentsY = moments(ThreshImageY);
    double m10Y = MomentsY.m10;
    double m01Y = MomentsY.m01;
    double AreaY = MomentsY.m00;
    yellow = false;

    if(AreaY > 100000){
      unsigned int xY = m10Y / AreaY;
      unsigned int yY = m01Y / AreaY;

      if(xY >= 0 && yY >= 0){
        cv::line(imageLines, cv::Point(xY, yY), cv::Point(lastXY, lastYY), cv::Scalar(0, 0, 255), 2);
      }

      lastXY = xY;
      lastYY = yY;
      yellow = true;
    }

    image += imageLines;



    // Update GUI Window
    cv::imshow("Image", image);
    //cv::imshow("Y", ThreshImageY);
    cv::imshow("R", ThreshImageR);

    cv::waitKey(3);
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "rgb_reader");

  ImageConverter ic;

  ros::spin();
  return 0;
}
