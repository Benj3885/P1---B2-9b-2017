#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global variable to keep track of how long since the last resource was spotted
ros::Time last_turn_received;

// Bool to make sure that the robot's goal is only saved once
bool target = false;
bool end = false;
bool start = false;

// Creating an object to store robot's destination
move_base_msgs::MoveBaseGoal goal;
move_base_msgs::MoveBaseGoal current_position;

// Linked list to store resources
struct resources
{
  int x = 0;
  int y = 0;
  std::string type;

  resources *next;
};

class brain
{
  ros::NodeHandle n;
  ros::Subscriber turner_sub; // /target_turner
  ros::Subscriber nav_goal_sub; // /move_base_simple/goal
  ros::Subscriber color_sub; // /object_color
  ros::Subscriber dist_sub; // /object_distance
  ros::Subscriber pos_sub; // odom
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

  public:
  brain()
  {
    turner_sub = n.subscribe<std_msgs::Float32>("/target_turner", 1, &brain::turnCb, this);
    color_sub = n.subscribe<std_msgs::String>("/object_color", 2, &brain::colorCb, this);
    nav_goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &brain::pauseNavStack, this);
    dist_sub = n.subscribe<std_msgs::Float32>("/object_distance", 2, &brain::distCb, this);
    // Subscribing to robot's location
    pos_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, &brain::odomCB, this);
  }

  std::string color = "";

  // Declaring the root of a resources object and pointer for current object
  resources *root = new resources;
  resources *current = root;

  // Robot's location variables
  double locx = 0, locy = 0;
  double yaw_degrees = 0;

// Distance to object
  float distance = 0;

  void turnCb(const std_msgs::Float32::ConstPtr& msg)
  {
    // Float for input from "/target_turner"
    float turn = msg->data;

    std::cout << "Yeah" << "  ||  " << turn << std::endl;

    // Keeping track of when the last timed a message on /target_turner was received
    last_turn_received = ros::Time::now();

    if(!target)
      start = true;

    target = true;

    // If the robot is looking straight at the object
    // A tolerance of 1 degrees has been given
    if(turn > -1 && turn < 1){
      // A delay to make sure that all topics has been received before attempting to read them
      ros::Duration(0.2).sleep();

      current->type = color;
      // Calculating target location
      current->x = cos(yaw_degrees) * distance + locx;
      current->y = sin(yaw_degrees) * distance + locy;

      // Target is set to false because the object has already been recorded
      target = false;
      end = true;

      // Creating the next link in the linked list
      createClass();
    } else {
      // Making a twist command to send orders to turtlebot
      geometry_msgs::Twist move;

      // Deciding which way to turn the robot
      float turner_z = (turn < 0 ? 0.25 : -0.25);

      // Inputting movement commands to turtlebot
      move.linear.x = 0;
      move.angular.z = turner_z;

      movement_pub.publish(move);
    }
  }

  void colorCb(const std_msgs::String::ConstPtr& msg)
  {
    // Saving which type of resource has been found
    color = msg->data;
  }

  void odomCB(const nav_msgs::Odometry::ConstPtr &msg){
    // Saving the robot's x, y coordinates
    locx = msg->pose.pose.position.x;
    locy = msg->pose.pose.position.y;

    // Creating a pose object
    tf::Pose pose;
    // Converting a pose msg to pose
    tf::poseMsgToTF(msg->pose.pose, pose);
    // Getting yaw for the robot and converting it to degrees
    double yaw = tf::getYaw(pose.getRotation());
    yaw_degrees = yaw * 57.29577;

    // Setting up a command to navigate to current position
    current_position.target_pose.header.frame_id = msg->header.frame_id;
    current_position.target_pose.header.stamp = ros::Time::now();

    current_position.target_pose.pose.position.x = msg->pose.pose.position.x;
    current_position.target_pose.pose.orientation.w = msg->pose.pose.orientation.w;
  }

  void distCb(const std_msgs::Float32::ConstPtr& msg)
  {
    // Saving the distance to the object
    distance = msg->data;
  }

  // Create next object in linked list and changing current
  void createClass(){
    resources *temp = new resources;
    current->next = temp;
    current = temp;
  }

  void pauseNavStack(const geometry_msgs::PoseStamped::ConstPtr &msg){
    if (target)
      return;

    // Saving what's needed in order to send the robot on its way after looking at object
    goal.target_pose.header.frame_id = msg->header.frame_id;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = msg->pose.position.x;
    goal.target_pose.pose.orientation.w = msg->pose.orientation.w;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "center");

  brain b;

  ros::Rate rate(10);

  MoveBaseClient ac("move_base", true);

  while(ros::ok()){
    // Stopping the robot from changing position
    if(start){
      // Makes the turtlebot stop
      ac.cancelGoal();
      start = false;
      std::cout << "Stop" << std::endl;
    }
    if(end){
      // Making the robot resume its interrupted journey
      ac.sendGoal(goal);
      end = false;
      std::cout << "Start" << std::endl;
    }

    ros::spin();
    rate.sleep();
  }

  return 0;
}
