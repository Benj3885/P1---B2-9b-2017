#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

ros::Time last_turn_received;
bool target = false;

class brain
{
  ros::NodeHandle n;
  ros::Subscriber turner_sub; // /target_turner
  ros::Subscriber color_sub; // /object_color
  ros::Subscriber dist_sub; // /object_distance
  ros::Subscriber angle_sub; // /object_angle
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

  public:
  turner_sub = n.subscribe<std_msgs::Float32>("/target_turner", 1, &brain::turnCb, this);
  std::string color[2] = {"", ""};
  int position[3][2];
  unsigned char count = 0;

  void turnCb(const std_msgs::Float32::ConstPtr& msg)
  {
    float turn = msg->data;

    last_turn_received = ros::Time::now();
    target = false;

    if(turn == 0){
      target = true;
      color_sub = n.subscribe<std_msgs::String>("/object_color", 2, &brain::colorCb, this);
      ros::Duration(0.2).sleep();
      dist_sub = n.subscribe<std_msgs::Float32>("/object_distance", 2, &brain::distCb, this);
      count++;
      if(count == 2)
        getDone();
    } else {
      // STOP NAVIGATION STACK!!

      geometry_msgs::Twist move;

      float turner_z = (turn < 0 ? 0.1 : -0.1);

      move.linear.x = 0;
      move.angular.z = turner_z;

      movement_pub.publish(move);
    }
  }

  void colorCb(const std_msgs::String::ConstPtr& msg)
  {
    color[count] = msg->data;
  }

  void distCb(const std_msgs::Float32::ConstPtr& msg)
  {
    float distance = msg->data;

    // USE TF TO FIND LOCATION IN WORLD!!
    // FIND DIRECTION ROBOT IS HEADED!!
    /*
    position[0][count] = x;
    position[1][count] = y;
    position[2][count] = z;
    */

  }

  void getDone(){
    unsigned char a = 0, b = 1;

    int y0 = 0, y1 = 0, x0 = 0, x1 = 0, z0 = 0, z1 = 0;
    int x_diff = 0, y_diff = 0;

    if(color[0] == 'yellow'){
      a = 1;
      b = 0;
    }
    x0 = position[0][a];
    x1 = position[0][b];
    y0 = position[1][a];
    y1 = position[1][b];
    z0 = position[2][a];
    z1 = position[2][b];

    x_diff = sqrt((x1 - x0)² + (y1 - y0)²);
    y_diff = y1 - y0;


  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "center");

  brain b;
  ros::Rate rate(10);

  while(ros::ok()){
    if(ros::Time::now() - last_turn_received > ros::Duration(2))
      target = false;

    ros::spin();
    rate.sleep();
  }

  return 0;
}
