#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/photo.hpp"

class ImageConverter
{
  ros::NodeHandle n;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_depth_sub;

public:
  ImageConverter()
    : it(n)
  {
    image_depth_sub = it.subscribe("/camera/depth/image", 1, &ImageConverter::imageCB, this);
  }

  void imageCB(const sensor_msgs::Image::ConstPtr& msg)
  {
    cv_bridge::CvImagePtr depth_ptr;
    depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat depthImage = depth_ptr->image;

    if(input->time)
      std::cout << depthImage.at<float>(depthImage.cols / 2, depthImage.rows / 2) << std::endl;
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_reader");



  return 0;
}
