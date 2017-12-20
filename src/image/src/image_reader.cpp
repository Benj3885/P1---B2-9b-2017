#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/photo.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "math.h"

class ImageConverter
{
  ros::NodeHandle n;
  image_transport::ImageTransport it;
  image_transport::Subscriber rgb_sub;
  image_transport::Subscriber depth_sub;
  ros::Publisher turn_pub = n.advertise<std_msgs::Float32>("/target_turner", 1);
  ros::Publisher dist_pub = n.advertise<std_msgs::Float32>("/object_distance", 2);
  ros::Publisher color_pub = n.advertise<std_msgs::String>("/object_color", 2);

  public:
  ImageConverter()
    : it(n)
  {
    rgb_sub = it.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
  }

// Tracking location of the pixel a certain color was last seen
//Ending with R is for red, while Y is for yellow
  int lastXR = -1;
  int lastYR = -1;
  int lastXY = -1;
  int lastYY = -1;

  //These are for keeping track of which colors are seen
  bool red = false;
  bool yellow = false;

  //To see later on, which color to prioritize
  bool redTarget = false;
  bool yellowTarget = false;

  //These are for finding out whether the program should still track a color
  //When true, it will no longer send the location of the color
  bool redDone = false;
  bool yellowDone = false;

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //A cv_pointer is created, and it's made to point to the new image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //Mat-variables are being declared for use
    cv::Mat image, imageLines, HSVImage, ThreshImageR, ThreshImageY;

    //The image from the cv_pointer is put into image
    image = cv_ptr->image;

    //The color is converted from BGR to HSV
    cvtColor(image, HSVImage, CV_BGR2HSV);

    //The pixels with values between scalar1 and scalar2 is set to 255 (white), others are set to 0 (black)
    //Red
    inRange(HSVImage, cv::Scalar(150, 140, 100), cv::Scalar(200, 200, 255), ThreshImageR);
    //Yellow
    inRange(HSVImage, cv::Scalar(15, 100, 120), cv::Scalar(50, 255, 255), ThreshImageY);

    //This variable is set to have the same size as image for later use
    imageLines = cv::Mat::zeros(image.size(), CV_8UC3);

    //Moments from red threshImage are found and extracted into double variables
    cv::Moments MomentsR = moments(ThreshImageR);
    double m10R = MomentsR.m10;
    double m01R = MomentsR.m01;
    double AreaR = MomentsR.m00;

    //red is set to false in order to avoid an ealier positive
    red = false;

    //If not enough pixels has been seen, the if-statement won't run
    if(AreaR > 30000){
      //x- and y-values are found from moments
      unsigned int xR = m10R / AreaR;
      unsigned int yR = m01R / AreaR;

      if(xR >= 0 && yR >= 0){
        //A blue line is written to x and y from earlier positive
        cv::line(imageLines, cv::Point(xR, yR), cv::Point(lastXR, lastYR), cv::Scalar(255, 0, 0), 4);
      }

      //Saving location of positive
      lastXR = xR;
      lastYR = yR;
      //Setting red to true, since it has been seen
      red = true;
    }

    //Below code is the same as for red, but with yellow
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

    //A constant has been found for focal point of camera
    float Fc = 577.295;
    //Float to hold value for publishing how much to turn in order to see object
    std_msgs::Float32 turn;
    turn.data = 0;

    //If red has been seen, and hasn't been measured before, it will now
    //otherwise, the same will be seen for yellow
    if(red && !redDone){
      float toTan = lastXR / Fc;
      turn.data = atan(toTan) * 180 / 3.14591 - 29;
      redTarget = true;
      turn_pub.publish(turn);
      std::cout << turn << "  ||  " << lastXR << std::endl;
    } else if(yellow && !yellowDone){
      float toTan = lastXY / Fc;
      turn.data = atan(toTan) * 180 / 3.14591 - 29;
      yellowTarget = true;
      turn_pub.publish(turn);
    }

    //If the robot is looking at the object to measure, and it has seen a color, it will call depth image
    if(turn.data > -0.2 && turn.data < 0.2)
      if(!red && !yellow)
        depth_sub = it.subscribe("/camera/depth/image", 1, &ImageConverter::depthCb, this);

    //The lines are added to the image
    image += imageLines;

    //Reset
    redTarget = false;
    yellowTarget = false;

    //For viewing on screen
    cv::imshow("Image", image);
    cv::imshow("Y", ThreshImageY);
    cv::imshow("R", ThreshImageR);

    cv::waitKey(3);
  }

  void depthCb(const sensor_msgs::ImageConstPtr &msg)
  {
    //Same story as before, pointer and image
    cv_bridge::CvImagePtr depth_ptr;
    depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat depthImage = depth_ptr->image;

    //Variables for publishing are declared
    std_msgs::Float32 dist;
    std_msgs::String color;

    //If red is the target
    if(redTarget){
      //dist is getting the value of the distance to target
      dist.data = depthImage.at<float>(depthImage.cols / 2, lastYR);
      //color is getting the color red
      color.data = "red";
      // red will no longer be looked for
      redDone = true;
      //The variables are published
      dist_pub.publish(dist);
      color_pub.publish(color);
    }

    //Same story as with red
    if(yellowTarget){
      dist.data = depthImage.at<float>(depthImage.cols / 2, lastYR);
      color.data = "yellow";
      yellowDone = true;
      dist_pub.publish(dist);
      color_pub.publish(color);
    }
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "read");

  ImageConverter ic;

  ros::spin();
  return 0;
}
